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
 * drawbox video filter
 */

#include <stdio.h>
#include <string.h>

#include "nifilter.h"
#include "version.h"
#include "filters.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
 // Needed for FFmpeg-n4.3+
#if (LIBAVFILTER_VERSION_MAJOR >= 8 || LIBAVFILTER_VERSION_MAJOR >= 7 && LIBAVFILTER_VERSION_MINOR >= 85)
#include "scale_eval.h"
#else
#include "scale.h"
#endif
#include "video.h"
#include "libavutil/avstring.h"
#include "libavutil/internal.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/eval.h"
#include "libavutil/parseutils.h"
#include "libavutil/avassert.h"
#include "libswscale/swscale.h"

enum OutputFormat {
    OUTPUT_FORMAT_YUV420P,
    OUTPUT_FORMAT_YUYV422,
    OUTPUT_FORMAT_UYVY422,
    OUTPUT_FORMAT_NV12,
    OUTPUT_FORMAT_ARGB,
    OUTPUT_FORMAT_RGBA,
    OUTPUT_FORMAT_ABGR,
    OUTPUT_FORMAT_BGRA,
    OUTPUT_FORMAT_YUV420P10LE,
    OUTPUT_FORMAT_NV16,
    OUTPUT_FORMAT_BGR0,
    OUTPUT_FORMAT_P010LE,
    OUTPUT_FORMAT_BGRP,
    OUTPUT_FORMAT_AUTO,
    OUTPUT_FORMAT_NB
};

static const char *const var_names[] = {
    "dar",
    "in_h", "ih",      ///< height of the input video
    "in_w", "iw",      ///< width  of the input video
    "sar",
    "x",
    "y",
    "h",              ///< height of the rendered box
    "w",              ///< width  of the rendered box
    "fill",
    NULL
};

enum { R, G, B, A };

enum var_name {
    VAR_DAR,
    VAR_IN_H, VAR_IH,
    VAR_IN_W, VAR_IW,
    VAR_SAR,
    VAR_X,
    VAR_Y,
    VAR_H,
    VAR_W,
    VAR_MAX,
    VARS_NB
};

typedef struct NetIntDrawBoxContext {
    const AVClass *class;

    /**
     * New dimensions. Special values are:
     *   0 = original width/height
     *  -1 = keep original aspect
     *  -N = try to keep aspect but make sure it is divisible by N
     */
    int w, h;
    int box_x[NI_MAX_SUPPORT_DRAWBOX_NUM], box_y[NI_MAX_SUPPORT_DRAWBOX_NUM], box_w[NI_MAX_SUPPORT_DRAWBOX_NUM], box_h[NI_MAX_SUPPORT_DRAWBOX_NUM];
    unsigned char box_rgba_color[NI_MAX_SUPPORT_DRAWBOX_NUM][4];
    ni_scaler_multi_drawbox_params_t scaler_drawbox_paras;
    char *size_str;

    char *box_x_expr[NI_MAX_SUPPORT_DRAWBOX_NUM];
    char *box_y_expr[NI_MAX_SUPPORT_DRAWBOX_NUM];
    char *box_w_expr[NI_MAX_SUPPORT_DRAWBOX_NUM];
    char *box_h_expr[NI_MAX_SUPPORT_DRAWBOX_NUM];
    char *box_color_str[NI_MAX_SUPPORT_DRAWBOX_NUM];

    int format;

    enum AVPixelFormat out_format;
    AVBufferRef *out_frames_ref;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
    ni_scaler_params_t params;

    int initialized;
    int session_opened;
    int keep_alive_timeout; /* keep alive timeout setting */
    int inplace;
    bool is_p2p;
    int buffer_limit;

    ni_frame_config_t frame_in;
    ni_frame_config_t frame_out;
} NetIntDrawBoxContext;

AVFilter ff_vf_drawbox_ni;

static const int NUM_EXPR_EVALS = 4;

static av_cold int init(AVFilterContext *ctx)
{
    NetIntDrawBoxContext *drawbox = ctx->priv;

    uint8_t rgba_color[4];

    // Check for incompatible options
    if (drawbox->inplace && drawbox->is_p2p) {
        av_log(ctx, AV_LOG_ERROR, "Cannot use 'inplace' and 'is_p2p' options together\n");
        return AVERROR(EINVAL);
    }

    if (av_parse_color(rgba_color, drawbox->box_color_str[0], -1, ctx) < 0)
        return AVERROR(EINVAL);

    drawbox->box_rgba_color[0][R] = rgba_color[0];
    drawbox->box_rgba_color[0][G] = rgba_color[1];
    drawbox->box_rgba_color[0][B] = rgba_color[2];
    drawbox->box_rgba_color[0][A] = rgba_color[3];

    return 0;
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_input(AVFilterLink *inlink)
#else
static int config_input(AVFilterLink *inlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = inlink->dst;
    NetIntDrawBoxContext *s = ctx->priv;
    double var_values[VARS_NB], res;
    char *expr;
    int ret;
    int i;

    var_values[VAR_IN_H] = var_values[VAR_IH] = inlink->h;
    var_values[VAR_IN_W] = var_values[VAR_IW] = inlink->w;
    var_values[VAR_SAR]  = inlink->sample_aspect_ratio.num ? av_q2d(inlink->sample_aspect_ratio) : 1;
    var_values[VAR_DAR]  = (double)inlink->w / inlink->h * var_values[VAR_SAR];
    var_values[VAR_X] = NAN;
    var_values[VAR_Y] = NAN;
    var_values[VAR_H] = NAN;
    var_values[VAR_W] = NAN;

    for (i = 0; i < NI_MAX_SUPPORT_DRAWBOX_NUM; i++) {
        /* evaluate expressions, fail on last iteration */
        var_values[VAR_MAX] = inlink->w;
        if ((ret = av_expr_parse_and_eval(&res, (expr = s->box_x_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
            goto fail;
        s->box_x[i] = var_values[VAR_X] = ((res < var_values[VAR_MAX]) ? res : (var_values[VAR_MAX] - 1));

        var_values[VAR_MAX] = inlink->h;
        if ((ret = av_expr_parse_and_eval(&res, (expr = s->box_y_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
            goto fail;
        s->box_y[i] = var_values[VAR_Y] = ((res < var_values[VAR_MAX]) ? res : (var_values[VAR_MAX] - 1));

        var_values[VAR_MAX] = inlink->w - s->box_x[i];
        if ((ret = av_expr_parse_and_eval(&res, (expr = s->box_w_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
            goto fail;
        s->box_w[i] = var_values[VAR_W] = ((res < var_values[VAR_MAX]) ? res : var_values[VAR_MAX]);

        var_values[VAR_MAX] = inlink->h - s->box_y[i];
        if ((ret = av_expr_parse_and_eval(&res, (expr = s->box_h_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, ctx)) < 0)
            goto fail;
        s->box_h[i] = var_values[VAR_H] = ((res < var_values[VAR_MAX]) ? res : var_values[VAR_MAX]);

        /* if w or h are zero, use the input w/h */
        s->box_w[i] = (s->box_w[i] > 0) ? s->box_w[i] : inlink->w;
        s->box_h[i] = (s->box_h[i] > 0) ? s->box_h[i] : inlink->h;

        /* sanity check width and height */
        if (s->box_w[i] <  0 || s->box_h[i] <  0) {
            av_log(ctx, AV_LOG_ERROR, "Size values less than 0 are not acceptable.\n");
            return AVERROR(EINVAL);
        }
        av_log(ctx, AV_LOG_VERBOSE, "%d: x:%d y:%d w:%d h:%d color:0x%02X%02X%02X%02X\n",
            i, s->box_x[i], s->box_y[i], s->box_w[i], s->box_h[i],
            s->box_rgba_color[0][R], s->box_rgba_color[0][G], s->box_rgba_color[0][B], s->box_rgba_color[0][A]);
    }

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[0]);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#elif IS_FFMPEG_342_AND_ABOVE
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#endif

    return 0;

fail:
    av_log(ctx, AV_LOG_ERROR,
           "Error when evaluating the expression '%s'.\n",
           expr);
    return ret;
}

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
    NetIntDrawBoxContext *drawbox = ctx->priv;

    if (drawbox->api_dst_frame.data.frame.p_buffer)
        ni_frame_buffer_free(&drawbox->api_dst_frame.data.frame);

    if (drawbox->session_opened) {
        /* Close operation will free the device frames */
        ni_device_session_close(&drawbox->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&drawbox->api_ctx);
    }

    av_buffer_unref(&drawbox->out_frames_ref);
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntDrawBoxContext *s = ctx->priv;
    AVHWFramesContext *out_frames_ctx;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    out_frames_ctx   = (AVHWFramesContext*)s->out_frames_ref->data;

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
                                  out_frames_ctx->height, s->out_format,
                                  pool_size,
                                  s->buffer_limit);
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_props(AVFilterLink *outlink)
#else
static int config_props(AVFilterLink *outlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink0 = outlink->src->inputs[0];
    AVFilterLink *inlink = outlink->src->inputs[0];
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    NetIntDrawBoxContext *drawbox = ctx->priv;
    int w, h, ret, h_shift, v_shift;

    if ((ret = ff_scale_eval_dimensions(ctx,
                                        "iw", "ih",
                                        inlink, outlink,
                                        &w, &h)) < 0)
        goto fail;

    /* Note that force_original_aspect_ratio may overwrite the previous set
     * dimensions so that it is not divisible by the set factors anymore
     * unless force_divisible_by is defined as well */

    if (w > NI_MAX_RESOLUTION_WIDTH || h > NI_MAX_RESOLUTION_HEIGHT) {
        av_log(ctx, AV_LOG_ERROR, "DrawBox value (%dx%d) > 8192 not allowed\n", w, h);
        return AVERROR(EINVAL);
    }

    if ((w <= 0) || (h <= 0)) {
        av_log(ctx, AV_LOG_ERROR, "DrawBox value (%dx%d) not allowed\n", w, h);
        return AVERROR(EINVAL);
    }

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

    /* Set the output format */
    drawbox->out_format = in_frames_ctx->sw_format;

    av_pix_fmt_get_chroma_sub_sample(drawbox->out_format, &h_shift, &v_shift);

    outlink->w = FFALIGN(w, (1 << h_shift));
    outlink->h = FFALIGN(h, (1 << v_shift));

    if (inlink0->sample_aspect_ratio.num) {
        outlink->sample_aspect_ratio = av_mul_q((AVRational){outlink->h * inlink0->w, outlink->w * inlink0->h}, inlink0->sample_aspect_ratio);
    } else {
        outlink->sample_aspect_ratio = inlink0->sample_aspect_ratio;

    }

    av_log(ctx, AV_LOG_VERBOSE,
           "w:%d h:%d fmt:%s sar:%d/%d -> w:%d h:%d fmt:%s sar:%d/%d\n",
           inlink->w, inlink->h, av_get_pix_fmt_name(inlink->format),
           inlink->sample_aspect_ratio.num, inlink->sample_aspect_ratio.den,
           outlink->w, outlink->h, av_get_pix_fmt_name(outlink->format),
           outlink->sample_aspect_ratio.num, outlink->sample_aspect_ratio.den);

    drawbox->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!drawbox->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)drawbox->out_frames_ref->data;

    out_frames_ctx->format    = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width     = outlink->w;
    out_frames_ctx->height    = outlink->h;
    out_frames_ctx->sw_format = drawbox->out_format;
    out_frames_ctx->initial_pool_size =
        NI_DRAWBOX_ID; // Repurposed as identity code

    av_hwframe_ctx_init(drawbox->out_frames_ref);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(ctx->outputs[0]);
    av_buffer_unref(&lo->hw_frames_ctx);
    lo->hw_frames_ctx = av_buffer_ref(drawbox->out_frames_ref);

    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
    av_buffer_unref(&ctx->outputs[0]->hw_frames_ctx);
    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(drawbox->out_frames_ref);

    if (!ctx->outputs[0]->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif

    return 0;

fail:
    return ret;
}

/* Process a received frame */
static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    NetIntDrawBoxContext *drawbox = link->dst->priv;
    AVFilterLink *outlink = link->dst->outputs[0];
    AVFrame *out = NULL;
    niFrameSurface1_t* frame_surface,*new_frame_surface;
    AVHWFramesContext *pAVHFWCtx,*out_frames_ctx;
    AVNIDeviceContext *pAVNIDevCtx;
    AVNIFramesContext *out_ni_ctx;
    ni_retcode_t retcode;
    int drawbox_format, cardno;
    uint16_t tempFID;
    double var_values[VARS_NB], res;
    char *expr;
    int ret;
    int i;
    uint32_t box_count = 0;
    const AVPixFmtDescriptor *desc;

    frame_surface = (niFrameSurface1_t *) in->data[3];
    if (frame_surface == NULL) {
        return AVERROR(EINVAL);
    }

    pAVHFWCtx = (AVHWFramesContext *) in->hw_frames_ctx->data;
    pAVNIDevCtx       = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno            = ni_get_cardno(in);

    if (!drawbox->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        ret = config_input(link, in);
        if (ret) {
            av_log(link->dst, AV_LOG_ERROR, "ni_drawbox failed config_input\n");
            retcode = AVERROR(EINVAL);
            goto fail;
        }

        ret = config_props(outlink, in);
        if (ret) {
            av_log(link->dst, AV_LOG_ERROR, "ni_drawbox failed config_output\n");
            retcode = AVERROR(EINVAL);
            goto fail;
        }
#endif

        retcode = ni_device_session_context_init(&drawbox->api_ctx);
        if (retcode < 0) {
            av_log(link->dst, AV_LOG_ERROR,
                   "ni drawbox filter session context init failure\n");
            goto fail;
        }

        drawbox->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        drawbox->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

        drawbox->api_ctx.hw_id             = cardno;
        drawbox->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
        drawbox->api_ctx.scaler_operation  = NI_SCALER_OPCODE_DRAWBOX;
        drawbox->api_ctx.keep_alive_timeout = drawbox->keep_alive_timeout;
        drawbox->api_ctx.isP2P = drawbox->is_p2p;

        av_log(link->dst, AV_LOG_INFO,
               "Open drawbox session to card %d, hdl %d, blk_hdl %d\n", cardno,
               drawbox->api_ctx.device_handle, drawbox->api_ctx.blk_io_handle);

        retcode =
            ni_device_session_open(&drawbox->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(link->dst, AV_LOG_ERROR,
                   "Can't open device session on card %d\n", cardno);

            /* Close operation will free the device frames */
            ni_device_session_close(&drawbox->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&drawbox->api_ctx);
            goto fail;
        }

        drawbox->session_opened = 1;

        if (drawbox->params.filterblit) {
            retcode = ni_scaler_set_params(&drawbox->api_ctx, &(drawbox->params));
            if (retcode < 0)
                goto fail;
        }

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           link->dst->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        retcode = init_out_pool(link->dst);

        if (retcode < 0) {
            av_log(link->dst, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            goto fail;
        }

        out_frames_ctx = (AVHWFramesContext *)drawbox->out_frames_ref->data;
        out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(pAVHFWCtx, out_frames_ctx);
        ni_device_session_copy(&drawbox->api_ctx, &out_ni_ctx->api_ctx);

        desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

        if ((in->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(link->dst, AV_LOG_WARNING,
                   "WARNING: Full color range input, limited color range output\n");
        }

        drawbox->initialized = 1;
    }

    drawbox_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&drawbox->api_dst_frame.data.frame,
                                          outlink->w,
                                          outlink->h,
                                          0);

    if (retcode != NI_RETCODE_SUCCESS) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    var_values[VAR_IN_H] = var_values[VAR_IH] = link->h;
    var_values[VAR_IN_W] = var_values[VAR_IW] = link->w;
    var_values[VAR_X] = NAN;
    var_values[VAR_Y] = NAN;
    var_values[VAR_H] = NAN;
    var_values[VAR_W] = NAN;

    memset(&drawbox->scaler_drawbox_paras, 0, sizeof(drawbox->scaler_drawbox_paras));
    for (i = 0; i < NI_MAX_SUPPORT_DRAWBOX_NUM; i++) {
        /* evaluate expressions, fail on last iteration */
        var_values[VAR_MAX] = link->w;
        if ((ret = av_expr_parse_and_eval(&res, (expr = drawbox->box_x_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, link->dst)) < 0)
            goto fail;
        drawbox->box_x[i] = var_values[VAR_X] = ((res < var_values[VAR_MAX]) ? ((res < 0) ? 0 : res) : (var_values[VAR_MAX] - 1));

        var_values[VAR_MAX] = link->h;
        if ((ret = av_expr_parse_and_eval(&res, (expr = drawbox->box_y_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, link->dst)) < 0)
            goto fail;
        drawbox->box_y[i] = var_values[VAR_Y] = ((res < var_values[VAR_MAX]) ? ((res < 0) ? 0 : res) : (var_values[VAR_MAX] - 1));

        var_values[VAR_MAX] = link->w - drawbox->box_x[i];
        if ((ret = av_expr_parse_and_eval(&res, (expr = drawbox->box_w_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, link->dst)) < 0)
            goto fail;
        drawbox->box_w[i] = var_values[VAR_W] = ((res < var_values[VAR_MAX]) ? res : var_values[VAR_MAX]);
        drawbox->box_w[i] = (drawbox->box_w[i] >= 0) ? drawbox->box_w[i] : var_values[VAR_MAX];

        var_values[VAR_MAX] = link->h - drawbox->box_y[i];
        if ((ret = av_expr_parse_and_eval(&res, (expr = drawbox->box_h_expr[i]),
                                          var_names, var_values,
                                          NULL, NULL, NULL, NULL, NULL, 0, link->dst)) < 0)
            goto fail;
        drawbox->box_h[i] = var_values[VAR_H] = ((res < var_values[VAR_MAX]) ? res : var_values[VAR_MAX]);

        drawbox->box_h[i] = (drawbox->box_h[i] >= 0) ? drawbox->box_h[i] : var_values[VAR_MAX];
        /* sanity check width and height */
        if (drawbox->box_w[i] <  0 || drawbox->box_h[i] <  0) {
            av_log(link->dst, AV_LOG_ERROR, "Size values less than 0 are not acceptable.\n");
            return AVERROR(EINVAL);
        }

            // please use drawbox->scaler_drawbox_paras to pass draw parameters
        av_log(link->dst, AV_LOG_DEBUG,"%d: x %d, y %d, w %d, h %d, color %x\n", \
            i, drawbox->box_x[i], drawbox->box_y[i], drawbox->box_w[i], drawbox->box_h[i], \
            drawbox->box_rgba_color[i][0] + (drawbox->box_rgba_color[i][1] << 8) + (drawbox->box_rgba_color[i][2] << 16) + (drawbox->box_rgba_color[i][3] << 24));

        if ((drawbox->box_w[i] > 0) && (drawbox->box_h[i] > 0)) {
            drawbox->scaler_drawbox_paras.multi_drawbox_params[box_count].start_x = drawbox->box_x[i];
            drawbox->scaler_drawbox_paras.multi_drawbox_params[box_count].start_y = drawbox->box_y[i];
            drawbox->scaler_drawbox_paras.multi_drawbox_params[box_count].end_x = drawbox->box_x[i] + drawbox->box_w[i] - 1;
            drawbox->scaler_drawbox_paras.multi_drawbox_params[box_count].end_y = drawbox->box_y[i] + drawbox->box_h[i] - 1;
            drawbox->scaler_drawbox_paras.multi_drawbox_params[box_count].rgba_c = drawbox->box_rgba_color[0][B] + (drawbox->box_rgba_color[0][G] << 8) + (drawbox->box_rgba_color[0][R] << 16) + (drawbox->box_rgba_color[0][A] << 24);
            if ((drawbox->box_w[i] > 0) && (drawbox->box_h[i] > 0))
                box_count++;
        }
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    retcode = ni_scaler_set_drawbox_params(&drawbox->api_ctx,
                    &drawbox->scaler_drawbox_paras.multi_drawbox_params[0]);
    if (retcode != NI_RETCODE_SUCCESS) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    drawbox->frame_in.picture_width  = FFALIGN(in->width, 2);
    drawbox->frame_in.picture_height = FFALIGN(in->height, 2);
    drawbox->frame_in.picture_format = drawbox_format;
    drawbox->frame_in.session_id     = frame_surface->ui16session_ID;
    drawbox->frame_in.output_index   = frame_surface->output_idx;
    drawbox->frame_in.frame_index    = frame_surface->ui16FrameIdx;

    /*
     * Config device input frame parameters
     */
    retcode = ni_device_config_frame(&drawbox->api_ctx, &drawbox->frame_in);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(link->dst, AV_LOG_DEBUG,
               "Can't allocate device input frame %d\n", retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    drawbox_format = ff_ni_ffmpeg_to_gc620_pix_fmt(drawbox->out_format);

    drawbox->frame_out.picture_width  = outlink->w;
    drawbox->frame_out.picture_height = outlink->h;
    drawbox->frame_out.picture_format = drawbox_format;

    /* Allocate hardware device destination frame. This acquires a frame
     * from the pool
     */
    retcode = ni_device_alloc_frame(&drawbox->api_ctx,
                                    FFALIGN(outlink->w, 2),
                                    FFALIGN(outlink->h, 2),
                                    drawbox_format,
                                    NI_SCALER_FLAG_IO,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    drawbox->inplace ? frame_surface->ui16FrameIdx : -1,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(link->dst, AV_LOG_DEBUG,
               "Can't allocate device output frame %d\n", retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&drawbox->api_ctx, &drawbox->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(link->dst, AV_LOG_ERROR,
               "Can't acquire output frame %d\n",retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_drawbox");
#endif

    /*
     * For an in-place drawbox, we have modified the input
     * frame so just pass it along to the downstream.
     */
    if (drawbox->inplace) {
        av_log(link->dst, AV_LOG_DEBUG,
               "vf_drawbox_ni.c:IN trace ui16FrameIdx = [%d] --> out [%d] \n",
               frame_surface->ui16FrameIdx, frame_surface->ui16FrameIdx);
        return ff_filter_frame(link->dst->outputs[0], in);
    }

    out = av_frame_alloc();
    if (!out) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    av_frame_copy_props(out,in);

    out->width  = outlink->w;
    out->height = outlink->h;

    out->format = AV_PIX_FMT_NI_QUAD;

    /* Quadra 2D engine always outputs limited color range */
    out->color_range = AVCOL_RANGE_MPEG;

    /* Reference the new hw frames context */
    out->hw_frames_ctx = av_buffer_ref(drawbox->out_frames_ref);

    out->data[3] = av_malloc(sizeof(niFrameSurface1_t));

    if (!out->data[3]) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Copy the frame surface from the incoming frame */
    memcpy(out->data[3], in->data[3], sizeof(niFrameSurface1_t));

    tempFID = frame_surface->ui16FrameIdx;
    frame_surface = (niFrameSurface1_t *)out->data[3];
    new_frame_surface = (niFrameSurface1_t *)drawbox->api_dst_frame.data.frame.p_data[3];
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
    frame_surface->ui16width  = out->width;
    frame_surface->ui16height = out->height;

    av_log(link->dst, AV_LOG_DEBUG,
           "vf_drawbox_ni.c:IN trace ui16FrameIdx = [%d] --> out [%d] \n",
           tempFID, frame_surface->ui16FrameIdx);

    out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t),
                                   ff_ni_frame_free, NULL, 0);

    av_frame_free(&in);

    return ff_filter_frame(link->dst->outputs[0], out);

fail:
    av_frame_free(&in);
    if (out)
        av_frame_free(&out);
    return retcode;
}

static int process_command(AVFilterContext *ctx, const char *cmd, const char *args, char *res, int res_len, int flags)
{
    AVFilterLink *inlink = ctx->inputs[0];
    NetIntDrawBoxContext *s = ctx->priv;
    int old_x = s->box_x[0];
    int old_y = s->box_y[0];
    int old_w = s->box_w[0];
    int old_h = s->box_h[0];
    char *old_color = av_strdup(s->box_color_str[0]);
    int ret;

    ret = ff_filter_process_command(ctx, cmd, args, res, res_len, flags);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Bad command/arguments (%d)\n", ret);
        return ret;
    }

    ret = init(ctx);
    if (ret < 0)
        goto end;
    ret = config_input(inlink);
end:
    if (ret < 0) {
        s->box_x[0] = old_x;
        s->box_y[0] = old_y;
        s->box_w[0] = old_w;
        s->box_h[0] = old_h;
        memcpy(s->box_color_str[0], old_color, strlen(old_color));
    }

    av_free(old_color);
    return ret;
}

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    AVFrame *frame = NULL;
    int ret = 0;
    NetIntDrawBoxContext *s = inlink->dst->priv;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        if (s->initialized && !s->inplace) {
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

#define OFFSET(x) offsetof(NetIntDrawBoxContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)
#define RFLAGS (FLAGS | AV_OPT_FLAG_RUNTIME_PARAM)

static const AVOption ni_drawbox_options[] = {
    { "x",         "set horizontal position of the left box edge", OFFSET(box_x_expr[0]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "y",         "set vertical position of the top box edge",    OFFSET(box_y_expr[0]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "width",     "set width of the box",                         OFFSET(box_w_expr[0]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "w",         "set width of the box",                         OFFSET(box_w_expr[0]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "height",    "set height of the box",                        OFFSET(box_h_expr[0]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "h",         "set height of the box",                        OFFSET(box_h_expr[0]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "color",     "set color of the box",                         OFFSET(box_color_str[0]),  AV_OPT_TYPE_STRING, {.str="black"}, 0, 0, RFLAGS },
    { "c",         "set color of the box",                         OFFSET(box_color_str[0]),  AV_OPT_TYPE_STRING, {.str="black"}, 0, 0, RFLAGS },
    { "x1",         "",                                            OFFSET(box_x_expr[1]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "y1",         "",                                            OFFSET(box_y_expr[1]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "w1",         "",                                            OFFSET(box_w_expr[1]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "h1",         "",                                            OFFSET(box_h_expr[1]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "x2",         "",                                            OFFSET(box_x_expr[2]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "y2",         "",                                            OFFSET(box_y_expr[2]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "w2",         "",                                            OFFSET(box_w_expr[2]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "h2",         "",                                            OFFSET(box_h_expr[2]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "x3",         "",                                            OFFSET(box_x_expr[3]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "y3",         "",                                            OFFSET(box_y_expr[3]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "w3",         "",                                            OFFSET(box_w_expr[3]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "h3",         "",                                            OFFSET(box_h_expr[3]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "x4",         "",                                            OFFSET(box_x_expr[4]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "y4",         "",                                            OFFSET(box_y_expr[4]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "w4",         "",                                            OFFSET(box_w_expr[4]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "h4",         "",                                            OFFSET(box_h_expr[4]),     AV_OPT_TYPE_STRING, {.str="0"},     0, 0, RFLAGS },
    { "filterblit", "filterblit enable",                           OFFSET(params.filterblit), AV_OPT_TYPE_BOOL,   {.i64=0},       0, 1, FLAGS },
    { "inplace",    "draw boxes in-place",                         OFFSET(inplace),           AV_OPT_TYPE_BOOL,   {.i64=0},       0, 1, FLAGS },
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_drawbox);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        // FFmpeg 3.4.2 and above only
#if IS_FFMPEG_342_AND_ABOVE
        .config_props   = config_input,
#endif
        .filter_frame = filter_frame,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        // FFmpeg 3.4.2 and above only
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_props,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_drawbox_ni_quadra = {
    .name            = "ni_quadra_drawbox",
    .description     = NULL_IF_CONFIG_SMALL("NETINT Quadra video drawbox v" NI_XCODER_REVISION),
    .init            = init,
    .uninit          = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate      = activate,
#endif

    .priv_size       = sizeof(NetIntDrawBoxContext),
    .priv_class      = &ni_drawbox_class,

    // FFmpeg 3.4.2 and above only
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
    .process_command = process_command,
};
