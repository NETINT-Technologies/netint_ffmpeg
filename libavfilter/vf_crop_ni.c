/*
 * Copyright (c) 2007 Bobby Bingham
 * Copyright (c) 2020 NetInt
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * video crop filter
 */

#include <stdio.h>

#include "nifilter.h"
#include "filters.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
#include "video.h"
#include "libavutil/eval.h"
#include "libavutil/avstring.h"
#include "libavutil/internal.h"
#include "libavutil/libm.h"
#include "libavutil/imgutils.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"

#include <ni_device_api.h>

static const char *const var_names[] = {
    "in_w", "iw",   ///< width  of the input video
    "in_h", "ih",   ///< height of the input video
    "out_w", "ow",  ///< width  of the cropped video
    "out_h", "oh",  ///< height of the cropped video
    "a",
    "sar",
    "dar",
    "hsub",
    "vsub",
    "x",
    "y",
    "n",            ///< number of frame
#if FF_API_FRAME_PKT
    "pos",          ///< position in the file
#endif
    "t",            ///< timestamp expressed in seconds
    NULL
};

enum var_name {
    VAR_IN_W,  VAR_IW,
    VAR_IN_H,  VAR_IH,
    VAR_OUT_W, VAR_OW,
    VAR_OUT_H, VAR_OH,
    VAR_A,
    VAR_SAR,
    VAR_DAR,
    VAR_HSUB,
    VAR_VSUB,
    VAR_X,
    VAR_Y,
    VAR_N,
#if FF_API_FRAME_PKT
    VAR_POS,
#endif
    VAR_T,
    VAR_VARS_NB
};

typedef struct NetIntCropContext {
    const AVClass *class;
    int  x;             ///< x offset of the non-cropped area with respect to the input area
    int  y;             ///< y offset of the non-cropped area with respect to the input area
    int  w;             ///< width of the cropped area
    int  h;             ///< height of the cropped area

    AVRational out_sar; ///< output sample aspect ratio
    int keep_aspect;    ///< keep display aspect ratio when cropping

    int max_step[4];    ///< max pixel step for each plane, expressed as a number of bytes
    int hsub, vsub;     ///< chroma subsampling
    char *x_expr, *y_expr, *w_expr, *h_expr;
    AVExpr *x_pexpr, *y_pexpr;  /* parsed expressions for x and y */
    double var_values[VAR_VARS_NB];

    AVBufferRef *out_frames_ref;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;

    int initialized;
    int session_opened;
    int keep_alive_timeout; /* keep alive timeout setting */
    bool is_p2p;

    int auto_skip;
    int skip_filter;
    int buffer_limit;
} NetIntCropContext;

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] =
        {AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE};
    AVFilterFormats *formats;

    formats = ff_make_format_list(pix_fmts);

    if (!formats)
        return AVERROR(ENOMEM);

    return ff_set_common_formats(ctx, formats);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntCropContext *s = ctx->priv;

    av_expr_free(s->x_pexpr);
    s->x_pexpr = NULL;
    av_expr_free(s->y_pexpr);
    s->y_pexpr = NULL;

    if (s->api_dst_frame.data.frame.p_buffer)
        ni_frame_buffer_free(&s->api_dst_frame.data.frame);

    if (s->session_opened) {
        /* Close operation will free the device frames */
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
    }

    av_buffer_unref(&s->out_frames_ref);
}

static inline int normalize_double(int *n, double d)
{
    int ret = 0;

    if (isnan(d)) {
        ret = AVERROR(EINVAL);
    } else if (d > INT_MAX || d < INT_MIN) {
        *n = d > INT_MAX ? INT_MAX : INT_MIN;
        ret = AVERROR(EINVAL);
    } else {
        *n = (int)lrint(d);
    }

    return ret;
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_input(AVFilterLink *link)
#else
static int config_input(AVFilterLink *link, AVFrame *in)
#endif
{
    AVFilterContext *ctx = link->dst;
    AVHWFramesContext *hwctx;
    NetIntCropContext *s;
    const AVPixFmtDescriptor *pix_desc;
    int ret;
    const char *expr;
    double res;
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(link);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    hwctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (link->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    hwctx = (AVHWFramesContext *)link->hw_frames_ctx->data;
#else
    hwctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif

    s = ctx->priv;
    pix_desc = av_pix_fmt_desc_get(hwctx->sw_format);

    s->var_values[VAR_IN_W]  = s->var_values[VAR_IW] = ctx->inputs[0]->w;
    s->var_values[VAR_IN_H]  = s->var_values[VAR_IH] = ctx->inputs[0]->h;
    s->var_values[VAR_A] = (double)link->w / (double)link->h;
    s->var_values[VAR_SAR]   = link->sample_aspect_ratio.num ? av_q2d(link->sample_aspect_ratio) : 1;
    s->var_values[VAR_DAR]   = s->var_values[VAR_A] * s->var_values[VAR_SAR];
    s->var_values[VAR_HSUB]  = 1<<pix_desc->log2_chroma_w;
    s->var_values[VAR_VSUB]  = 1<<pix_desc->log2_chroma_h;
    s->var_values[VAR_X]     = NAN;
    s->var_values[VAR_Y]     = NAN;
    s->var_values[VAR_OUT_W] = s->var_values[VAR_OW] = NAN;
    s->var_values[VAR_OUT_H] = s->var_values[VAR_OH] = NAN;
    s->var_values[VAR_N]     = 0;
    s->var_values[VAR_T]     = NAN;
#if FF_API_FRAME_PKT
    s->var_values[VAR_POS]   = NAN;
#endif

    av_image_fill_max_pixsteps(s->max_step, NULL, pix_desc);
    s->hsub = pix_desc->log2_chroma_w;
    s->vsub = pix_desc->log2_chroma_h;

    if ((ret = av_expr_parse_and_eval(&res, (expr = s->w_expr),
                                      var_names, s->var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto fail_expr;
    s->var_values[VAR_OUT_W] = s->var_values[VAR_OW] = res;
    if ((ret = av_expr_parse_and_eval(&res, (expr = s->h_expr),
                                      var_names, s->var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto fail_expr;
    s->var_values[VAR_OUT_H] = s->var_values[VAR_OH] = res;
    /* evaluate again ow as it may depend on oh */
    if ((ret = av_expr_parse_and_eval(&res, (expr = s->w_expr),
                                      var_names, s->var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto fail_expr;

    s->var_values[VAR_OUT_W] = s->var_values[VAR_OW] = res;
    if (normalize_double(&s->w, s->var_values[VAR_OUT_W]) < 0 ||
        normalize_double(&s->h, s->var_values[VAR_OUT_H]) < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "Too big value or invalid expression for out_w/ow or out_h/oh. "
               "Maybe the expression for out_w:'%s' or for out_h:'%s' is self-referencing.\n",
               s->w_expr, s->h_expr);
        return AVERROR(EINVAL);
    }

    s->w &= ~((1 << s->hsub) - 1);
    s->h &= ~((1 << s->vsub) - 1);

    av_expr_free(s->x_pexpr);
    s->x_pexpr = NULL;
    av_expr_free(s->y_pexpr);
    s->y_pexpr = NULL;
    if ((av_expr_parse(&s->x_pexpr, s->x_expr, var_names, NULL, NULL, NULL,
                       NULL, 0, ctx) < 0) ||
        (av_expr_parse(&s->y_pexpr, s->y_expr, var_names, NULL, NULL, NULL,
                       NULL, 0, ctx) < 0))
        return AVERROR(EINVAL);

    if (s->keep_aspect) {
        AVRational dar = av_mul_q(link->sample_aspect_ratio,
                                  (AVRational){ link->w, link->h });
        av_reduce(&s->out_sar.num, &s->out_sar.den,
                  dar.num * s->h, dar.den * s->w, INT_MAX);
    } else {
        s->out_sar = link->sample_aspect_ratio;
    }

    av_log(ctx, AV_LOG_VERBOSE, "w:%d h:%d sar:%d/%d -> w:%d h:%d sar:%d/%d\n",
           link->w, link->h, link->sample_aspect_ratio.num, link->sample_aspect_ratio.den,
           s->w, s->h, s->out_sar.num, s->out_sar.den);

    if (s->w <= 0 || s->h <= 0 ||
        s->w > link->w || s->h > link->h) {
        av_log(ctx, AV_LOG_ERROR,
               "Invalid too big or non positive size for width '%d' or height '%d'\n",
               s->w, s->h);
        return AVERROR(EINVAL);
    }

    /* set default, required in the case the first computed value for x/y is NAN */
    s->x = (link->w - s->w) / 2;
    s->y = (link->h - s->h) / 2;

    s->x &= ~((1 << s->hsub) - 1);
    s->y &= ~((1 << s->vsub) - 1);

    return 0;

fail_expr:
    av_log(NULL, AV_LOG_ERROR, "Error when evaluating the expression '%s'\n", expr);
    return ret;
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntCropContext *s = ctx->priv;
    AVHWFramesContext *out_frames_ctx;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    out_frames_ctx = (AVHWFramesContext*)s->out_frames_ref->data;

    /* Don't check return code, this will intentionally fail */
    // av_hwframe_ctx_init(s->out_frames_ref);

    if (s->api_ctx.isP2P) {
        pool_size = 1;
    }
#if IS_FFMPEG_71_AND_ABOVE
    else {
        pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
    }
#endif
#if IS_FFMPEG_61_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create frame pool on device */
    return ff_ni_build_frame_pool(&s->api_ctx, out_frames_ctx->width,
                                  out_frames_ctx->height,
                                  out_frames_ctx->sw_format, pool_size,
                                  s->buffer_limit);
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *link)
#else
static int config_output(AVFilterLink *link, AVFrame *in)
#endif
{
    NetIntCropContext *s = link->src->priv;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    AVFilterContext *ctx = (AVFilterContext *)link->src;;

    link->w = s->w;
    link->h = s->h;
    link->sample_aspect_ratio = s->out_sar;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[0]);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)ctx->inputs[0]->hw_frames_ctx->data;
#else
    if (in->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
        av_log(ctx, AV_LOG_ERROR, "bgrp not supported\n");
        return AVERROR(EINVAL);
    }
    if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
        in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(ctx, AV_LOG_ERROR, "tile4x4 not supported\n");
        return AVERROR(EINVAL);
    }

    //skip the color range check
    if (s->auto_skip &&
        !s->is_p2p && //the input and output are always on the same card, but filter with p2p always need to be executed
        (s->x_expr && strcmp(s->x_expr, "0") == 0 && s->y_expr && strcmp(s->y_expr, "0") == 0) &&
        (in_frames_ctx->width == link->w && in_frames_ctx->height == link->h)
       ) {
        //skip hardware crop
        s->skip_filter = 1;

#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *lo = ff_filter_link(link);
        s->out_frames_ref = av_buffer_ref(li->hw_frames_ctx);
        if (!s->out_frames_ref) {
            return AVERROR(ENOMEM);
        }
        av_buffer_unref(&lo->hw_frames_ctx);
        lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        if (!lo->hw_frames_ctx) {
            return AVERROR(ENOMEM);
        }
#else
        AVFilterLink *inlink_for_skip = link->src->inputs[0];

        s->out_frames_ref = av_buffer_ref(inlink_for_skip->hw_frames_ctx);
        if (!s->out_frames_ref) {
            return AVERROR(ENOMEM);
        }

        av_buffer_unref(&link->hw_frames_ctx);
        link->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        if (!link->hw_frames_ctx) {
            return AVERROR(ENOMEM);
        }
#endif
        return 0;
    }

    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    out_frames_ctx->format    = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width     = s->w;
    out_frames_ctx->height    = s->h;
    out_frames_ctx->sw_format = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size =
        NI_CROP_ID; // Repurposed as identity code

    av_hwframe_ctx_init(s->out_frames_ref);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(link);
    av_buffer_unref(&lo->hw_frames_ctx);

    lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
    av_buffer_unref(&link->hw_frames_ctx);

    link->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!link->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif
    return 0;
}

static int filter_frame(AVFilterLink *link, AVFrame *frame)
{
    AVFilterContext *ctx = link->dst;
    NetIntCropContext *s = ctx->priv;
    AVFilterLink *outlink = link->dst->outputs[0];
    AVFrame *out = NULL;
    niFrameSurface1_t* frame_surface,*new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    ni_retcode_t retcode;
    uint32_t scaler_format;
    int cardno;
    uint16_t tempFID;

    pAVHFWCtx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    if (!pAVHFWCtx) {
        return AVERROR(EINVAL);
    }

    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    if (!pAVNIDevCtx) {
        return AVERROR(EINVAL);
    }

    cardno = ni_get_cardno(frame);

    if (s->skip_filter) {
        //skip hardware crop
        return ff_filter_frame(link->dst->outputs[0], frame);
    }

    if (!s->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        retcode = config_input(link, frame);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR, "ni crop filter config input failure\n");
            goto fail;
        }

        retcode = config_output(outlink, frame);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR, "ni crop filter config output failure\n");
            goto fail;
        }
#endif

        retcode = ni_device_session_context_init(&s->api_ctx);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni crop filter session context init failure\n");
            goto fail;
        }

        s->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

        s->api_ctx.hw_id             = cardno;
        s->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
        s->api_ctx.scaler_operation  = NI_SCALER_OPCODE_CROP;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
        s->api_ctx.isP2P              = s->is_p2p;

        retcode = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't open device session on card %d\n",
                   cardno);

            /* Close operation will free the device frames */
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->api_ctx);
            goto fail;
        }

        s->session_opened = 1;

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           ctx->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        retcode = init_out_pool(ctx);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            goto fail;
        }

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(pAVHFWCtx, out_frames_ctx);
        ni_device_session_copy(&s->api_ctx, &out_ni_ctx->api_ctx);

        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

        if ((frame->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(link->dst, AV_LOG_WARNING,
                   "WARNING: Full color range input, limited color range output\n");
        }

        s->initialized = 1;
    }
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *l = ff_filter_link(link);
    s->var_values[VAR_N] = l->frame_count_out;
#elif IS_FFMPEG_342_AND_ABOVE
    s->var_values[VAR_N] = link->frame_count_out;
#else
    s->var_values[VAR_N] = link->frame_count;
#endif
    s->var_values[VAR_T] = frame->pts == AV_NOPTS_VALUE ?
        NAN : frame->pts * av_q2d(link->time_base);
#if FF_API_FRAME_PKT
FF_DISABLE_DEPRECATION_WARNINGS // FFmpeg-n6.1+ deprecates AVFrame->pkt_pos
    s->var_values[VAR_POS] = frame->pkt_pos == -1 ?
        NAN : frame->pkt_pos;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);
    s->var_values[VAR_Y] = av_expr_eval(s->y_pexpr, s->var_values, NULL);
    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);

    normalize_double(&s->x, s->var_values[VAR_X]);
    normalize_double(&s->y, s->var_values[VAR_Y]);

    if (s->x < 0)
        s->x = 0;
    if (s->y < 0)
        s->y = 0;
    if ((unsigned)s->x + (unsigned)s->w > link->w)
        s->x = link->w - s->w;
    if ((unsigned)s->y + (unsigned)s->h > link->h)
        s->y = link->h - s->h;

    s->x &= ~((1 << s->hsub) - 1);
    s->y &= ~((1 << s->vsub) - 1);

    av_log(ctx, AV_LOG_TRACE, "n:%d t:%f x:%d y:%d x+w:%d y+h:%d\n",
           (int)s->var_values[VAR_N], s->var_values[VAR_T], s->x, s->y,
           s->x+s->w, s->y+s->h);

    frame_surface = (niFrameSurface1_t *) frame->data[3];
    if (frame_surface == NULL) {
        retcode = AVERROR(EINVAL);
        goto fail;
    }

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          outlink->w,
                                          outlink->h,
                                          0);

    if (retcode != NI_RETCODE_SUCCESS) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(&s->api_ctx,
                                    FFALIGN(frame->width, 2),
                                    FFALIGN(frame->height, 2),
                                    scaler_format,
                                    0,    // input frame
                                    s->w, // src rectangle width
                                    s->h, // src rectangle height
                                    s->x, // src rectangle x
                                    s->y, // src rectangle y
                                    frame_surface->ui32nodeAddress,
                                    frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't assign input frame %d\n",
               retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Allocate device destination frame This will acquire a frame from the pool */
    retcode = ni_device_alloc_frame(&s->api_ctx,
                        FFALIGN(outlink->w,2),
                        FFALIGN(outlink->h,2),
                        scaler_format,
                        NI_SCALER_FLAG_IO,
                        0,
                        0,
                        0,
                        0,
                        0,
                        -1,
                        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate device output frame %d\n",
               retcode);

        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    out = av_frame_alloc();
    if (!out) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    av_frame_copy_props(out,frame);

    out->width  = s->w;
    out->height = s->h;

    out->format = AV_PIX_FMT_NI_QUAD;

    /* Quadra 2D engine always outputs limited color range */
    out->color_range = AVCOL_RANGE_MPEG;

    /* Reference the new hw frames context */
    out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);

    out->data[3] = av_malloc(sizeof(niFrameSurface1_t));

    if (!out->data[3]) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Copy the frame surface from the incoming frame */
    memcpy(out->data[3], frame->data[3], sizeof(niFrameSurface1_t));

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR,
               "Can't acquire output frame %d\n",retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_crop");
#endif

    tempFID           = frame_surface->ui16FrameIdx;
    frame_surface = (niFrameSurface1_t *) out->data[3];
    new_frame_surface = (niFrameSurface1_t *) s->api_dst_frame.data.frame.p_data[3];
    frame_surface->ui16FrameIdx   = new_frame_surface->ui16FrameIdx;
    frame_surface->ui16session_ID = new_frame_surface->ui16session_ID;
    frame_surface->device_handle  = (int32_t)pAVNIDevCtx->cards[cardno];
    frame_surface->output_idx     = new_frame_surface->output_idx;
    frame_surface->src_cpu        = new_frame_surface->src_cpu;
    frame_surface->dma_buf_fd     = 0;

    ff_ni_set_bit_depth_and_encoding_type(&frame_surface->bit_depth,
                                          &frame_surface->encoding_type,
                                          pAVHFWCtx->sw_format);

    /* Remove ni-split specific assets */
    frame_surface->ui32nodeAddress = 0;

    frame_surface->ui16width = out->width;
    frame_surface->ui16height = out->height;

    av_log(ctx, AV_LOG_DEBUG,
           "vf_crop_ni.c:IN trace ui16FrameIdx = [%d] --> out = [%d] \n",
           tempFID, frame_surface->ui16FrameIdx);

    out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t), ff_ni_frame_free, NULL, 0);

    av_frame_free(&frame);

    return ff_filter_frame(link->dst->outputs[0], out);

fail:
    av_frame_free(&frame);
    if (out)
        av_frame_free(&out);
    return retcode;
}

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    AVFrame *frame = NULL;
    int ret = 0;
    NetIntCropContext *s = inlink->dst->priv;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        if (s->initialized) {
            ret = ni_device_session_query_buffer_avail(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        }

        if (ret == NI_RETCODE_ERROR_UNSUPPORTED_FW_VERSION) {
            av_log(ctx, AV_LOG_WARNING, "No backpressure support in FW\n");
        } else if (ret < 0) {
            av_log(ctx, AV_LOG_WARNING, "%s: query ret %d, ready %u inlink framequeue %u available_frame %d outlink framequeue %u frame_wanted %d - return NOT READY\n",
                __func__, ret, ctx->ready, ff_inlink_queued_frames(inlink), ff_inlink_check_available_frame(inlink), ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));
            return FFERROR_NOT_READY;
        }

        ret = ff_inlink_consume_frame(inlink, &frame);
        if (ret < 0)
            return ret;

        ret = filter_frame(inlink, frame);
        if (ret >= 0) {
            ff_filter_set_ready(ctx, 300);
        }
        return ret;
    }

    // We did not get a frame from input link, check its status
    FF_FILTER_FORWARD_STATUS(inlink, outlink);

    // We have no frames yet from input link and no EOF, so request some.
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}
#endif

#define OFFSET(x) offsetof(NetIntCropContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)

static const AVOption ni_crop_options[] = {
    { "out_w",       "set the width crop area expression",  OFFSET(w_expr),      AV_OPT_TYPE_STRING, {.str = "iw"}, .flags = FLAGS },
    { "w",           "set the width crop area expression",  OFFSET(w_expr),      AV_OPT_TYPE_STRING, {.str = "iw"}, .flags = FLAGS },
    { "out_h",       "set the height crop area expression", OFFSET(h_expr),      AV_OPT_TYPE_STRING, {.str = "ih"}, .flags = FLAGS },
    { "h",           "set the height crop area expression", OFFSET(h_expr),      AV_OPT_TYPE_STRING, {.str = "ih"}, .flags = FLAGS },
    { "x",           "set the x crop area expression",      OFFSET(x_expr),      AV_OPT_TYPE_STRING, {.str = "(in_w-out_w)/2"}, .flags = FLAGS },
    { "y",           "set the y crop area expression",      OFFSET(y_expr),      AV_OPT_TYPE_STRING, {.str = "(in_h-out_h)/2"}, .flags = FLAGS },
    { "keep_aspect", "keep aspect ratio",                   OFFSET(keep_aspect), AV_OPT_TYPE_BOOL,   {.i64=0}, 0, 1, FLAGS },
    NI_FILT_OPTION_AUTO_SKIP,
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_crop);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_input,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_output,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_crop_ni_quadra = {
    .name            = "ni_quadra_crop",
    .description     = NULL_IF_CONFIG_SMALL("NETINT Quadra crop the input video v" NI_XCODER_REVISION),
    .priv_size       = sizeof(NetIntCropContext),
    .priv_class      = &ni_crop_class,
    .uninit          = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate        = activate,
#endif
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs          = inputs,
    .outputs         = outputs,
    .query_formats   = query_formats,
#endif
};
