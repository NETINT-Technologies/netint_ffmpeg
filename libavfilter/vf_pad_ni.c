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
 * video padding filter
 */

#include <float.h>  /* DBL_MAX */

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
#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/pixdesc.h"
#include "libavutil/colorspace.h"
#include "libavutil/imgutils.h"
#include "libavutil/parseutils.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "drawutils.h"

static const char *const var_names[] = {
    "in_w",   "iw",
    "in_h",   "ih",
    "out_w",  "ow",
    "out_h",  "oh",
    "x",
    "y",
    "a",
    "sar",
    "dar",
    "hsub",
    "vsub",
    NULL
};

enum var_name {
    VAR_IN_W,   VAR_IW,
    VAR_IN_H,   VAR_IH,
    VAR_OUT_W,  VAR_OW,
    VAR_OUT_H,  VAR_OH,
    VAR_X,
    VAR_Y,
    VAR_A,
    VAR_SAR,
    VAR_DAR,
    VAR_HSUB,
    VAR_VSUB,
    VARS_NB
};

typedef struct NetIntPadContext {
    const AVClass *class;
    int w, h;               ///< output dimensions, a value of 0 will result in the input size
    int x, y;               ///< offsets of the input area with respect to the padded area
    int in_w, in_h;         ///< width and height for the padded input video, which has to be aligned to the chroma values in order to avoid chroma issues
    int inlink_w, inlink_h;
    AVRational aspect;

    char *w_expr;           ///< width  expression string
    char *h_expr;           ///< height expression string
    char *x_expr;           ///< width  expression string
    char *y_expr;           ///< height expression string
    uint8_t rgba_color[4];  ///< color for the padding area
    FFDrawContext draw;
    FFDrawColor color;

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
} NetIntPadContext;

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
    NetIntPadContext *s = ctx->priv;

    if (s->api_dst_frame.data.frame.p_buffer)
        ni_frame_buffer_free(&s->api_dst_frame.data.frame);

    if (s->session_opened) {
        /* Close operation will free the device frames */
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
    }

    av_buffer_unref(&s->out_frames_ref);
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntPadContext  *s = ctx->priv;
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
    return ff_ni_build_frame_pool(&s->api_ctx,
                                  out_frames_ctx->width, out_frames_ctx->height,
                                  out_frames_ctx->sw_format,
                                  pool_size,
                                  s->buffer_limit);
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_input(AVFilterLink *inlink)
#else
static int config_input(AVFilterLink *inlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = inlink->dst;
    NetIntPadContext *s = ctx->priv;
    AVRational adjusted_aspect = s->aspect;
    int ret;
    double var_values[VARS_NB], res;
    char *expr;
    AVHWFramesContext *avhwctx;

    if (inlink->format == AV_PIX_FMT_NI_QUAD) {
#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *li = ff_filter_link(inlink);
        if (li->hw_frames_ctx == NULL) {
            av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
            return AVERROR(EINVAL);
        }
        avhwctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
        if (inlink->hw_frames_ctx == NULL) {
            av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
            return AVERROR(EINVAL);
        }
        avhwctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
#else
        avhwctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif
        if (ff_draw_init(&s->draw, avhwctx->sw_format, 0) < 0)
            return AVERROR(EINVAL);
    } else {
        if (ff_draw_init(&s->draw, inlink->format, 0) < 0)
            return AVERROR(EINVAL);
    }

    ff_draw_color(&s->draw, &s->color, s->rgba_color);

    var_values[VAR_IN_W]  = var_values[VAR_IW] = inlink->w;
    var_values[VAR_IN_H]  = var_values[VAR_IH] = inlink->h;
    var_values[VAR_OUT_W] = var_values[VAR_OW] = NAN;
    var_values[VAR_OUT_H] = var_values[VAR_OH] = NAN;
    var_values[VAR_A]     = (double) inlink->w / inlink->h;
    var_values[VAR_SAR]   = inlink->sample_aspect_ratio.num ?
        (double) inlink->sample_aspect_ratio.num / inlink->sample_aspect_ratio.den : 1;
    var_values[VAR_DAR]   = var_values[VAR_A] * var_values[VAR_SAR];
    var_values[VAR_HSUB]  = 1 << s->draw.hsub_max;
    var_values[VAR_VSUB]  = 1 << s->draw.vsub_max;

    /* evaluate width and height */
    av_expr_parse_and_eval(&res, s->w_expr, var_names, var_values, NULL, NULL,
                           NULL, NULL, NULL, 0, ctx);
    s->w                  = (int)res;
    var_values[VAR_OUT_W] = var_values[VAR_OW] = res;
    if ((ret = av_expr_parse_and_eval(&res, (expr = s->h_expr),
                                      var_names, var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto eval_fail;
    s->h                  = (int)res;
    var_values[VAR_OUT_H] = var_values[VAR_OH] = res;
    if (!s->h)
        var_values[VAR_OUT_H] = var_values[VAR_OH] = s->h = inlink->h;

    /* evaluate the width again, as it may depend on the evaluated output height */
    if ((ret = av_expr_parse_and_eval(&res, (expr = s->w_expr),
                                      var_names, var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto eval_fail;
    s->w                  = (int)res;
    var_values[VAR_OUT_W] = var_values[VAR_OW] = res;
    if (!s->w)
        var_values[VAR_OUT_W] = var_values[VAR_OW] = s->w = inlink->w;

    if (adjusted_aspect.num && adjusted_aspect.den) {
        adjusted_aspect = av_div_q(adjusted_aspect, inlink->sample_aspect_ratio);
        if (s->h < av_rescale(s->w, adjusted_aspect.den, adjusted_aspect.num)) {
            s->h = av_rescale(s->w, adjusted_aspect.den, adjusted_aspect.num);
            var_values[VAR_OUT_H] = var_values[VAR_OH] = (double)s->h;
        } else {
            s->w = av_rescale(s->h, adjusted_aspect.num, adjusted_aspect.den);
            var_values[VAR_OUT_W] = var_values[VAR_OW] = (double)s->w;
        }
    }

    /* evaluate x and y */
    av_expr_parse_and_eval(&res, s->x_expr, var_names, var_values, NULL, NULL,
                           NULL, NULL, NULL, 0, ctx);
    s->x              = (int)res;
    var_values[VAR_X] = res;
    if ((ret = av_expr_parse_and_eval(&res, (expr = s->y_expr),
                                      var_names, var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto eval_fail;
    s->y              = (int)res;
    var_values[VAR_Y] = res;
    /* evaluate x again, as it may depend on the evaluated y value */
    if ((ret = av_expr_parse_and_eval(&res, (expr = s->x_expr),
                                      var_names, var_values,
                                      NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
        goto eval_fail;
    s->x              = (int)res;
    var_values[VAR_X] = res;

    if (s->x < 0 || s->x + inlink->w > s->w) {
        var_values[VAR_X] = (double)(s->w - inlink->w) / 2.0;
        s->x              = (int)var_values[VAR_X];
    }
    if (s->y < 0 || s->y + inlink->h > s->h) {
        var_values[VAR_Y] = (double)(s->h - inlink->h) / 2.0;
        s->y              = (int)var_values[VAR_Y];
    }

    /* sanity check params */
    if (s->w < 0 || s->h < 0) {
        av_log(ctx, AV_LOG_ERROR, "Negative values are not acceptable.\n");
        return AVERROR(EINVAL);
    }

    s->w    = ff_draw_round_to_sub(&s->draw, 0, -1, s->w);
    s->h    = ff_draw_round_to_sub(&s->draw, 1, -1, s->h);
    s->x    = ff_draw_round_to_sub(&s->draw, 0, -1, s->x);
    s->y    = ff_draw_round_to_sub(&s->draw, 1, -1, s->y);
    s->in_w = ff_draw_round_to_sub(&s->draw, 0, -1, inlink->w);
    s->in_h = ff_draw_round_to_sub(&s->draw, 1, -1, inlink->h);
    s->inlink_w = inlink->w;
    s->inlink_h = inlink->h;

    av_log(ctx, AV_LOG_VERBOSE, "w:%d h:%d -> w:%d h:%d x:%d y:%d color:0x%02X%02X%02X%02X\n",
           inlink->w, inlink->h, s->w, s->h, s->x, s->y,
           s->rgba_color[0], s->rgba_color[1], s->rgba_color[2], s->rgba_color[3]);

    if (s->x <  0 || s->y <  0                      ||
        s->w <= 0 || s->h <= 0                      ||
        (unsigned)s->x + (unsigned)inlink->w > s->w ||
        (unsigned)s->y + (unsigned)inlink->h > s->h) {
        av_log(ctx, AV_LOG_ERROR,
               "Input area %d:%d:%d:%d not within the padded area 0:0:%d:%d or zero-sized\n",
               s->x, s->y, s->x + inlink->w, s->y + inlink->h, s->w, s->h);
        return AVERROR(EINVAL);
    }

    if (s->w > NI_MAX_RESOLUTION_WIDTH || s->h > NI_MAX_RESOLUTION_HEIGHT) {
        av_log(ctx, AV_LOG_ERROR, "Padded value (%dx%d) > 8192, not allowed\n", s->w, s->h);
        return AVERROR(EINVAL);
    }

    return 0;

eval_fail:
    av_log(ctx, AV_LOG_ERROR,
           "Error when evaluating the expression '%s'\n", expr);
    return ret;

}

#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *outlink)
#else
static int config_output(AVFilterLink *outlink, AVFrame *in)
#endif
{
    NetIntPadContext *s = outlink->src->priv;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    AVFilterContext *ctx;

    outlink->w = s->w;
    outlink->h = s->h;

    ctx           = (AVFilterContext *)outlink->src;
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

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP ||
        in_frames_ctx->sw_format == AV_PIX_FMT_YUYV422 ||
        in_frames_ctx->sw_format == AV_PIX_FMT_UYVY422) {
        av_log(ctx, AV_LOG_ERROR, "bgrp/yuyv/uyvy not supported\n");
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
        s->w == in_frames_ctx->width &&
        s->h == in_frames_ctx->height &&
        s->x == 0 &&
        s->y == 0
       ) {
        //skip hardware pad
        s->skip_filter = 1;
#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *lt = ff_filter_link(outlink->src->inputs[0]);
        s->out_frames_ref = av_buffer_ref(lt->hw_frames_ctx);
        if (!s->out_frames_ref) {
            return AVERROR(ENOMEM);
        }
        FilterLink *lo = ff_filter_link(outlink);
        av_buffer_unref(&lo->hw_frames_ctx);
        lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        if (!lo->hw_frames_ctx) {
            return AVERROR(ENOMEM);
        }
#else
        AVFilterLink *inlink_for_skip = outlink->src->inputs[0];

        s->out_frames_ref = av_buffer_ref(inlink_for_skip->hw_frames_ctx);
        if (!s->out_frames_ref) {
            return AVERROR(ENOMEM);
        }

        av_buffer_unref(&outlink->hw_frames_ctx);
        outlink->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        if (!outlink->hw_frames_ctx) {
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
        NI_PAD_ID; // Repurposed as identity code

    av_hwframe_ctx_init(s->out_frames_ref);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(outlink);
    av_buffer_unref(&lo->hw_frames_ctx);

    lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
    av_buffer_unref(&outlink->hw_frames_ctx);

    outlink->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!outlink->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif
    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    NetIntPadContext *s = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    AVFrame *out = NULL;
    niFrameSurface1_t* frame_surface,*new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    ni_retcode_t retcode;
    uint32_t ui32RgbaColor, scaler_format;
    uint16_t tempFID;
    int cardno;

    frame_surface = (niFrameSurface1_t *) in->data[3];
    if (frame_surface == NULL) {
        return AVERROR(EINVAL);
    }

    pAVHFWCtx         = (AVHWFramesContext *)in->hw_frames_ctx->data;
    pAVNIDevCtx       = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno            = ni_get_cardno(in);

    if (s->skip_filter) {
         //skip hardware pad
        return ff_filter_frame(inlink->dst->outputs[0], in);
    }

    if (!s->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        retcode = config_input(inlink, in);
        if (retcode < 0) {
            av_log(inlink->dst, AV_LOG_ERROR,
                   "ni pad filter config_input failure\n");
            goto fail;
        }

        retcode = config_output(outlink, in);
        if (retcode < 0) {
            av_log(inlink->dst, AV_LOG_ERROR,
                   "ni pad filter config_output failure\n");
            goto fail;
        }
#endif
        retcode = ni_device_session_context_init(&s->api_ctx);
        if (retcode < 0) {
            av_log(inlink->dst, AV_LOG_ERROR,
                   "ni pad filter session context init failure\n");
            goto fail;
        }

        s->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

        s->api_ctx.hw_id             = cardno;
        s->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
        s->api_ctx.scaler_operation  = NI_SCALER_OPCODE_PAD;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
        s->api_ctx.isP2P = s->is_p2p;

        retcode = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(inlink->dst, AV_LOG_ERROR,
                   "Can't open device session on card %d\n", cardno);

            /* Close operation will free the device frames */
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->api_ctx);
            goto fail;
        }

        s->session_opened = 1;

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           inlink->dst->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        retcode = init_out_pool(inlink->dst);

        if (retcode < 0) {
            av_log(inlink->dst, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            goto fail;
        }

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(pAVHFWCtx, out_frames_ctx);
        ni_device_session_copy(&s->api_ctx, &out_ni_ctx->api_ctx);

        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

        if ((in->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(inlink->dst, AV_LOG_WARNING,
                   "WARNING: Full color range input, limited color range output\n");
        }

        s->initialized = 1;
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

    av_log(inlink->dst, AV_LOG_DEBUG,
           "inlink->w = %d;inlink->h = %d;outlink->w = %d;outlink->h = %d\n",
           inlink->w, inlink->h, outlink->w, outlink->h);
    av_log(inlink->dst, AV_LOG_DEBUG,
           "s->w=%d;s->h=%d;s->x=%d;s->y=%d;c=%02x:%02x:%02x:%02x\n", s->w,
           s->h, s->x, s->y, s->rgba_color[0], s->rgba_color[1],
           s->rgba_color[2], s->rgba_color[3]);

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(&s->api_ctx,
                                    FFALIGN(in->width, 2),
                                    FFALIGN(in->height, 2),
                                    scaler_format,
                                    0,                      // input frame
                                    in->width,  // src rectangle width
                                    in->height, // src rectangle height
                                    0,          // src rectangle x = 0
                                    0,          // src rectangle y = 0
                                    frame_surface->ui32nodeAddress,
                                    frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(inlink->dst, AV_LOG_DEBUG, "Can't allocate device input frame %d\n", retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Scaler uses BGRA color, or ARGB in little-endian */
    ui32RgbaColor = (s->rgba_color[3] << 24) | (s->rgba_color[0] << 16) |
                    (s->rgba_color[1] << 8) | s->rgba_color[2];

    /* Allocate device destination frame. This will acquire a frame from the pool */
    retcode = ni_device_alloc_frame(&s->api_ctx,
                          FFALIGN(outlink->w,2),
                          FFALIGN(outlink->h,2),
                          scaler_format,
                          NI_SCALER_FLAG_IO,    // output frame
                          in->width,            // dst rectangle width
                          in->height,           // dst rectangle height
                          s->x,                 // dst rectangle x
                          s->y,                 // dst rectangle y
                          ui32RgbaColor,        // rgba color
                          -1,
                          NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(inlink->dst, AV_LOG_DEBUG,
               "Can't allocate device output frame %d\n", retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    out = av_frame_alloc();
    if (!out) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    av_frame_copy_props(out,in);

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
    memcpy(out->data[3], in->data[3], sizeof(niFrameSurface1_t));

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(inlink->dst, AV_LOG_ERROR,
               "Can't acquire output frame %d\n",retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_pad");
#endif

    tempFID = frame_surface->ui16FrameIdx;
    frame_surface = (niFrameSurface1_t *) out->data[3];
    new_frame_surface = (niFrameSurface1_t *) s->api_dst_frame.data.frame.p_data[3];
    frame_surface->ui16FrameIdx   = new_frame_surface->ui16FrameIdx;
    frame_surface->ui16session_ID = new_frame_surface->ui16session_ID;
    frame_surface->device_handle  = new_frame_surface->device_handle;
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

    av_log(inlink->dst, AV_LOG_DEBUG,
           "vf_pad_ni.c:IN trace ui16FrameIdx = [%d] --> out [%d] \n", tempFID,
           frame_surface->ui16FrameIdx);

    out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t), ff_ni_frame_free, NULL, 0);

    av_frame_free(&in);

    return ff_filter_frame(inlink->dst->outputs[0], out);

fail:
    av_frame_free(&in);
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
    NetIntPadContext *s = inlink->dst->priv;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        if (s->initialized) {
            ret = ni_device_session_query_buffer_avail(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        }

        if (ret == NI_RETCODE_ERROR_UNSUPPORTED_FW_VERSION) {
            av_log(ctx, AV_LOG_WARNING, "No backpressure support in FW\n");
        } else if (ret < 0) {
            av_log(ctx, AV_LOG_WARNING, "%s: query ret %d, inlink framequeue %lu available_frame %d outlink framequeue %lu frame_wanted %d - return NOT READY\n",
                __func__, ret, ff_inlink_queued_frames(inlink), ff_inlink_check_available_frame(inlink), ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));
            return FFERROR_NOT_READY;
        }

        ret = ff_inlink_consume_frame(inlink, &frame);
        if (ret < 0)
            return ret;

        ret = filter_frame(inlink, frame);
        if (ret >= 0) {
            ff_filter_set_ready(ctx, 100);
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

#define OFFSET(x) offsetof(NetIntPadContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)

static const AVOption ni_pad_options[] = {
    { "width",  "set the pad area width expression",                        OFFSET(w_expr),     AV_OPT_TYPE_STRING,   {.str = "iw"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "w",      "set the pad area width expression",                        OFFSET(w_expr),     AV_OPT_TYPE_STRING,   {.str = "iw"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "height", "set the pad area height expression",                       OFFSET(h_expr),     AV_OPT_TYPE_STRING,   {.str = "ih"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "h",      "set the pad area height expression",                       OFFSET(h_expr),     AV_OPT_TYPE_STRING,   {.str = "ih"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "x",      "set the x offset expression for the input image position", OFFSET(x_expr),     AV_OPT_TYPE_STRING,   {.str = "0"},  CHAR_MIN, CHAR_MAX, FLAGS },
    { "y",      "set the y offset expression for the input image position", OFFSET(y_expr),     AV_OPT_TYPE_STRING,   {.str = "0"},  CHAR_MIN, CHAR_MAX, FLAGS },
    { "color",  "set the color of the padded area border",                  OFFSET(rgba_color), AV_OPT_TYPE_COLOR,    {.str = "black"}, .flags = FLAGS },
    { "aspect", "pad to fit an aspect instead of a resolution",             OFFSET(aspect),     AV_OPT_TYPE_RATIONAL, {.dbl = 0}, 0, DBL_MAX, FLAGS },
    NI_FILT_OPTION_AUTO_SKIP,
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_pad);

static const AVFilterPad inputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .filter_frame     = filter_frame,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props     = config_input,
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

AVFilter ff_vf_pad_ni_quadra = {
    .name          = "ni_quadra_pad",
    .description   = NULL_IF_CONFIG_SMALL("NETINT Quadra pad the input video v" NI_XCODER_REVISION),
    .priv_size     = sizeof(NetIntPadContext),
    .priv_class    = &ni_pad_class,
    .uninit        = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate      = activate,
#endif
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal= FF_FILTER_FLAG_HWFRAME_AWARE,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs        = inputs,
    .outputs       = outputs,
    .query_formats = query_formats,
#endif
};
