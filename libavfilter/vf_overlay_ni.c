/*
 * Copyright (c) 2010 Stefano Sabatini
 * Copyright (c) 2010 Baptiste Coudurier
 * Copyright (c) 2007 Bobby Bingham
 * Copyright (c) 2021 NetInt
 *
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
 * overlay one video on top of another
 */

#include "nifilter.h"
#include "filters.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/avstring.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/timestamp.h"
#include "libavutil/hwcontext.h"
#include "drawutils.h"
#include "framesync.h"
#include "video.h"
#include <ni_device_api.h>

static const char *const var_names[] = {
    "main_w",    "W", ///< width  of the main    video
    "main_h",    "H", ///< height of the main    video
    "overlay_w", "w", ///< width  of the overlay video
    "overlay_h", "h", ///< height of the overlay video
    "hsub",
    "vsub",
    "x",
    "y",
    "t",
    NULL
};

enum var_name {
    VAR_MAIN_W,    VAR_MW,
    VAR_MAIN_H,    VAR_MH,
    VAR_OVERLAY_W, VAR_OW,
    VAR_OVERLAY_H, VAR_OH,
    VAR_HSUB,
    VAR_VSUB,
    VAR_X,
    VAR_Y,
    VAR_T,
    VAR_VARS_NB
};

#define MAIN    0
#define OVERLAY 1

typedef struct NetIntOverlayContext {
    const AVClass *class;
    int x, y;                   ///< position of overlaid picture

    uint8_t main_has_alpha;
    uint8_t overlay_has_alpha;
    int alpha_format;

    FFFrameSync fs;

    int hsub, vsub;             ///< chroma subsampling values

    double var_values[VAR_VARS_NB];
    char *x_expr, *y_expr;

    AVExpr *x_pexpr, *y_pexpr;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;

    AVBufferRef* out_frames_ref;

    int initialized;
    int session_opened;
    int crop_session_opened;
    int keep_alive_timeout; /* keep alive timeout setting */
    int inplace;
    bool is_p2p;
    int buffer_limit;
    uint16_t ui16CropFrameIdx;
#if !IS_FFMPEG_342_AND_ABOVE
    int config_input_initialized[2];
    int opt_repeatlast;
    int opt_shortest;
    int opt_eof_action;
#endif

    ni_session_context_t crop_api_ctx;
    ni_session_data_io_t crop_api_dst_frame;
} NetIntOverlayContext;

static int process_frame(FFFrameSync *fs);
static int process_frame_inplace(FFFrameSync *fs);

static int set_expr(AVExpr **pexpr, const char *expr, const char *option, void *log_ctx)
{
    int ret;
    AVExpr *old = NULL;

    if (*pexpr)
        old = *pexpr;
    ret = av_expr_parse(pexpr, expr, var_names,
                        NULL, NULL, NULL, NULL, 0, log_ctx);
    if (ret < 0) {
        av_log(log_ctx, AV_LOG_ERROR,
               "Error when evaluating the expression '%s' for %s\n",
               expr, option);
        *pexpr = old;
        return ret;
    }

    av_expr_free(old);
    return 0;
}

static int query_formats(AVFilterContext *ctx)
{
    /* We only accept hardware frames */
    static const enum AVPixelFormat pix_fmts[] =
        {AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE};
    AVFilterFormats *formats;

    formats = ff_make_format_list(pix_fmts);

    if (!formats)
        return AVERROR(ENOMEM);

    return ff_set_common_formats(ctx, formats);
}

static int init_framesync(AVFilterContext *ctx)
{
    NetIntOverlayContext *s = ctx->priv;
    int ret, i;

    s->fs.on_event = s->inplace ? process_frame_inplace : process_frame;
    s->fs.opaque   = s;
    ret = ff_framesync_init(&s->fs, ctx, ctx->nb_inputs);
    if (ret < 0)
        return ret;

    for (i = 0; i < ctx->nb_inputs; i++) {
        FFFrameSyncIn *in = &s->fs.in[i];
        in->before    = EXT_STOP;
        in->after     = EXT_INFINITY;
        in->sync      = i ? 1 : 2;
        in->time_base = ctx->inputs[i]->time_base;
    }

#if !IS_FFMPEG_342_AND_ABOVE
    if (!s->opt_repeatlast || s->opt_eof_action == EOF_ACTION_PASS) {
        s->opt_repeatlast = 0;
        s->opt_eof_action = EOF_ACTION_PASS;
    }
    if (s->opt_shortest || s->opt_eof_action == EOF_ACTION_ENDALL) {
        s->opt_shortest = 1;
        s->opt_eof_action = EOF_ACTION_ENDALL;
    }
    if (!s->opt_repeatlast) {
        for (i = 1; i < s->fs.nb_in; i++) {
            s->fs.in[i].after = EXT_NULL;
            s->fs.in[i].sync  = 0;
        }
    }
    if (s->opt_shortest) {
        for (i = 0; i < s->fs.nb_in; i++)
            s->fs.in[i].after = EXT_STOP;
    }
#endif

    return ff_framesync_configure(&s->fs);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntOverlayContext *s = ctx->priv;

    ff_framesync_uninit(&s->fs);
    av_expr_free(s->x_pexpr); s->x_pexpr = NULL;
    av_expr_free(s->y_pexpr); s->y_pexpr = NULL;

    if (s->api_dst_frame.data.frame.p_buffer) {
        ni_frame_buffer_free(&s->api_dst_frame.data.frame);
    }

    if (s->crop_api_dst_frame.data.frame.p_buffer) {
        ni_frame_buffer_free(&s->crop_api_dst_frame.data.frame);
    }

    if (s->session_opened) {
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
    }

    if (s->crop_session_opened) {
        ni_device_session_close(&s->crop_api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->crop_api_ctx);
    }

    av_buffer_unref(&s->out_frames_ref);
}

static inline int normalize_xy(double d, int chroma_sub)
{
    if (isnan(d))
        return INT_MAX;
    return (int)d & ~((1 << chroma_sub) - 1);
}

static void eval_expr(AVFilterContext *ctx)
{
    NetIntOverlayContext *s = ctx->priv;

    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);
    s->var_values[VAR_Y] = av_expr_eval(s->y_pexpr, s->var_values, NULL);
    s->var_values[VAR_X] = av_expr_eval(s->x_pexpr, s->var_values, NULL);
    s->x = normalize_xy(s->var_values[VAR_X], s->hsub);
    s->y = normalize_xy(s->var_values[VAR_Y], s->vsub);
}

static int overlay_intersects_background(
    const AVFilterContext *ctx,
    const AVFrame *overlay,
    const AVFrame *main)
{
    const NetIntOverlayContext *s = (NetIntOverlayContext *) ctx->priv;

    if (s->x >= main->width)
        return 0;

    if (s->y >= main->height)
        return 0;

    if (s->x + overlay->width <= 0)
        return 0;

    if (s->y + overlay->height <= 0)
        return 0;

    return 1;
}

static void calculate_src_rectangle(
    int *px,
    int *py,
    int *pw,
    int *ph,
    int bgnd_x,
    int bgnd_y,
    int bgnd_w,
    int bgnd_h,
    int ovly_x,
    int ovly_y,
    int ovly_w,
    int ovly_h)

{
    *px = (ovly_x > 0) ? 0 : -ovly_x;
    *py = (ovly_y > 0) ? 0 : -ovly_y;

    if (ovly_x > 0) {
        *pw = FFMIN(bgnd_w - ovly_x, ovly_w);
    } else {
        *pw = FFMIN(ovly_w + ovly_x, bgnd_w);
    }

    if (ovly_y > 0) {
        *ph = FFMIN(bgnd_h - ovly_y, ovly_h);
    } else {
        *ph = FFMIN(ovly_h + ovly_y, bgnd_h);
    }
}

static void calculate_dst_rectangle(
    int *px,
    int *py,
    int *pw,
    int *ph,
    int bgnd_x,
    int bgnd_y,
    int bgnd_w,
    int bgnd_h,
    int ovly_x,
    int ovly_y,
    int ovly_w,
    int ovly_h)
{
    *px = FFMAX(0, ovly_x);
    *py = FFMAX(0, ovly_y);

    if (ovly_x > 0) {
        *pw = FFMIN(bgnd_w - ovly_x, ovly_w);
    } else {
        *pw = FFMIN(ovly_w + ovly_x, bgnd_w);
    }

    if (ovly_y > 0) {
        *ph = FFMIN(bgnd_h - ovly_y, ovly_h);
    } else {
        *ph = FFMIN(ovly_h + ovly_y, bgnd_h);
    }
}

static const enum AVPixelFormat alpha_pix_fmts[] = {
    AV_PIX_FMT_ARGB, AV_PIX_FMT_ABGR, AV_PIX_FMT_RGBA,
    AV_PIX_FMT_BGRA, AV_PIX_FMT_NONE
};

#if IS_FFMPEG_342_AND_ABOVE
static int config_input_overlay(AVFilterLink *inlink)
#else
static int config_input_overlay(AVFilterLink *inlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx  = inlink->dst;
    NetIntOverlayContext  *s = inlink->dst->priv;
    AVHWFramesContext *in_frames_ctx;
    const AVPixFmtDescriptor *pix_desc;
    int ret;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li == NULL) {
        av_log(inlink->dst, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (inlink->hw_frames_ctx == NULL) {
        av_log(inlink->dst, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
#else
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif
    if (!in_frames_ctx) {
        return AVERROR(EINVAL);
    }

    pix_desc = av_pix_fmt_desc_get(in_frames_ctx->sw_format);

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
        av_log(ctx, AV_LOG_ERROR, "bgrp not supported for overlay\n");
        return AVERROR(EINVAL);
    }

    if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(ctx, AV_LOG_ERROR, "tile4x4 10b not supported for overlay!\n");
        return AVERROR(EINVAL);
    }

    /* Finish the configuration by evaluating the expressions
       now when both inputs are configured. */
    s->var_values[VAR_MAIN_W   ] = s->var_values[VAR_MW] = ctx->inputs[MAIN   ]->w;
    s->var_values[VAR_MAIN_H   ] = s->var_values[VAR_MH] = ctx->inputs[MAIN   ]->h;
    s->var_values[VAR_OVERLAY_W] = s->var_values[VAR_OW] = ctx->inputs[OVERLAY]->w;
    s->var_values[VAR_OVERLAY_H] = s->var_values[VAR_OH] = ctx->inputs[OVERLAY]->h;
    s->var_values[VAR_HSUB]  = 1<<pix_desc->log2_chroma_w;
    s->var_values[VAR_VSUB]  = 1<<pix_desc->log2_chroma_h;
    s->var_values[VAR_X]     = NAN;
    s->var_values[VAR_Y]     = NAN;
    s->var_values[VAR_T]     = NAN;

    if ((ret = set_expr(&s->x_pexpr,      s->x_expr,      "x",      ctx)) < 0 ||
        (ret = set_expr(&s->y_pexpr,      s->y_expr,      "y",      ctx)) < 0)
        return ret;

    s->overlay_has_alpha = ff_fmt_is_in(in_frames_ctx->sw_format,
                                        alpha_pix_fmts);

    av_log(ctx, AV_LOG_VERBOSE,
           "main w:%d h:%d fmt:%s overlay w:%d h:%d fmt:%s\n",
           ctx->inputs[MAIN]->w, ctx->inputs[MAIN]->h,
           av_get_pix_fmt_name(ctx->inputs[MAIN]->format),
           ctx->inputs[OVERLAY]->w, ctx->inputs[OVERLAY]->h,
           av_get_pix_fmt_name(ctx->inputs[OVERLAY]->format));
    return 0;
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntOverlayContext *s = ctx->priv;
    AVHWFramesContext *out_frames_ctx;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

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
                                  out_frames_ctx->height, out_frames_ctx->sw_format,
                                  pool_size,
                                  s->buffer_limit);
}


#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *outlink)
#else
static int config_output_dummy(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    return init_framesync(ctx);
}

static int config_output(AVFilterLink *outlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = outlink->src;
    NetIntOverlayContext *s = ctx->priv;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    int ret = 0;

    outlink->w = ctx->inputs[MAIN]->w;
    outlink->h = ctx->inputs[MAIN]->h;
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[MAIN]);
    FilterLink *lo = ff_filter_link(outlink);
    lo->frame_rate = li->frame_rate;
#else
    outlink->frame_rate = ctx->inputs[MAIN]->frame_rate;
#endif
    outlink->time_base = ctx->inputs[MAIN]->time_base;

#if IS_FFMPEG_342_AND_ABOVE
    ret = init_framesync(ctx);
    if (ret < 0)
        return ret;
#if IS_FFMPEG_71_AND_ABOVE
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#else
    in_frames_ctx = (AVHWFramesContext *)ctx->inputs[MAIN]->hw_frames_ctx->data;
#endif
#else
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif
    if (!s->inplace) {
#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *lt;
#endif
        AVHWFramesContext *tmp_frames_ctx;

        s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
        if (!s->out_frames_ref)
            return AVERROR(ENOMEM);

        out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

        out_frames_ctx->format    = AV_PIX_FMT_NI_QUAD;
        out_frames_ctx->width     = outlink->w;
        out_frames_ctx->height    = outlink->h;

        av_hwframe_ctx_init(s->out_frames_ref);

#if IS_FFMPEG_71_AND_ABOVE
        lt = ff_filter_link(ctx->inputs[OVERLAY]);
        tmp_frames_ctx = (AVHWFramesContext *)lt->hw_frames_ctx->data;
#else
        tmp_frames_ctx = (AVHWFramesContext *)ctx->inputs[OVERLAY]->hw_frames_ctx->data;
#endif
        // HW does not support NV12 Compress + RGB -> NV12 Compress
        if (((in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4) ||
            (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4)) &&
            ((tmp_frames_ctx->sw_format >= AV_PIX_FMT_ARGB) &&
            (tmp_frames_ctx->sw_format <= AV_PIX_FMT_BGRA))) {
            out_frames_ctx->sw_format = AV_PIX_FMT_NV12;
            av_log(ctx, AV_LOG_WARNING, "Overlay output is changed to nv12\n");
        } else {
            out_frames_ctx->sw_format = in_frames_ctx->sw_format;
        }

        out_frames_ctx->initial_pool_size =
            NI_OVERLAY_ID; // Repurposed as identity code
    } else {
#if IS_FFMPEG_71_AND_ABOVE
        s->out_frames_ref = av_buffer_ref(li->hw_frames_ctx);
#else
        s->out_frames_ref = av_buffer_ref(ctx->inputs[MAIN]->hw_frames_ctx);
#endif
    }

#if IS_FFMPEG_71_AND_ABOVE
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

    return ret;
}

static int do_intermediate_crop_and_overlay(AVFilterContext *ctx,
                                            AVFrame *overlay, AVFrame *frame)
{
    NetIntOverlayContext *s = (NetIntOverlayContext *) ctx->priv;
    AVHWFramesContext    *main_frame_ctx,*ovly_frame_ctx;
    niFrameSurface1_t    *frame_surface;
    ni_retcode_t          retcode;
    uint16_t              ui16FrameIdx;
    int                   main_scaler_format,ovly_scaler_format;
    int                   flags;
    int                   crop_x,crop_y,crop_w,crop_h;
    int                   src_x,src_y,src_w,src_h;

    main_frame_ctx = (AVHWFramesContext *) frame->hw_frames_ctx->data;
    main_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(main_frame_ctx->sw_format);

    ovly_frame_ctx = (AVHWFramesContext *) overlay->hw_frames_ctx->data;
    ovly_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(ovly_frame_ctx->sw_format);

    /* Allocate a ni_frame_t for the intermediate crop operation */
    retcode = ni_frame_buffer_alloc_hwenc(&s->crop_api_dst_frame.data.frame,
                                          ctx->inputs[OVERLAY]->w,
                                          ctx->inputs[OVERLAY]->h,
                                          0);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't allocate interim crop frame\n");
        return AVERROR(ENOMEM);
    }

    calculate_dst_rectangle(&crop_x, &crop_y, &crop_w, &crop_h,
                            0, 0, frame->width, frame->height,
                            FFALIGN(s->x,2), FFALIGN(s->y,2),
                            overlay->width, overlay->height);

    frame_surface = (niFrameSurface1_t *) frame->data[3];

    /* Assign a device input frame. Send incoming frame index to crop session */
    retcode = ni_device_alloc_frame(
        &s->crop_api_ctx,
        FFALIGN(ctx->inputs[MAIN]->w, 2),
        FFALIGN(ctx->inputs[MAIN]->h, 2),
        main_scaler_format,
        0,
        crop_w,
        crop_h,
        crop_x,
        crop_y,
        0,
        frame_surface->ui16FrameIdx,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't assign input crop frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Allocate destination frame. This acquires a frame from the pool */
    retcode = ni_device_alloc_frame(
        &s->crop_api_ctx,
        FFALIGN(ctx->inputs[OVERLAY]->w, 2),
        FFALIGN(ctx->inputs[OVERLAY]->h, 2),
        ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA),
        NI_SCALER_FLAG_IO,
        crop_w,
        crop_h,
        0,
        0,
        0,
        -1,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate output crop frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    retcode = ni_device_session_read_hwdesc(&s->crop_api_ctx,
                                            &s->crop_api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "No cropped output frame %d\n", retcode);
        return AVERROR(ENOMEM);
    }

    /* Get the acquired frame */
    frame_surface = (niFrameSurface1_t *)
        s->crop_api_dst_frame.data.frame.p_data[3];
    s->ui16CropFrameIdx = frame_surface->ui16FrameIdx;

    /* Overlay the icon over the intermediate cropped frame */

    /* Allocate a ni_frame_t for the intermediate overlay */
    retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          ctx->inputs[OVERLAY]->w,
                                          ctx->inputs[OVERLAY]->h,
                                          0);

    if (retcode < 0) {
        av_log(ctx, AV_LOG_ERROR, "Can't allocate interim ovly frame\n");
        return AVERROR(ENOMEM);
    }

    frame_surface = (niFrameSurface1_t *) overlay->data[3];
    ui16FrameIdx = frame_surface->ui16FrameIdx;

    calculate_src_rectangle(&src_x, &src_y, &src_w, &src_h,
                            0, 0, frame->width, frame->height,
                            FFALIGN(s->x,2), FFALIGN(s->y,2),
                            overlay->width, overlay->height);

    /* Assign input frame to intermediate overlay session */
    retcode = ni_device_alloc_frame(
        &s->api_ctx,
        FFALIGN(ctx->inputs[OVERLAY]->w, 2),
        FFALIGN(ctx->inputs[OVERLAY]->h, 2),
        ovly_scaler_format,
        0,
        src_w,
        src_h,
        src_x,
        src_y,
        0,
        ui16FrameIdx,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't assign input overlay frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* In-place overlay frame. Send down frame index of background frame */
    flags = NI_SCALER_FLAG_IO;                        /* Configure output */
    flags |= s->alpha_format ? NI_SCALER_FLAG_PA : 0; /* Premultiply/straight */

    retcode = ni_device_alloc_frame(
       &s->api_ctx,
       FFALIGN(ctx->inputs[OVERLAY]->w, 2),
       FFALIGN(ctx->inputs[OVERLAY]->h, 2),
       ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA),
       flags,
       crop_w,
       crop_h,
       0,
       0,
       0,
       s->ui16CropFrameIdx,
       NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't overlay frame for output %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    retcode = ni_device_session_read_hwdesc(&s->api_ctx,
                                            &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't acquire intermediate frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    return NI_RETCODE_SUCCESS;
}

static int process_frame(FFFrameSync *fs)
{
    AVFilterContext      *ctx = fs->parent;
    NetIntOverlayContext *s = (NetIntOverlayContext *) ctx->priv;
    AVHWFramesContext    *main_frame_ctx,*ovly_frame_ctx;
    AVNIDeviceContext    *pAVNIDevCtx;
    AVFilterLink         *inlink_main = ctx->inputs[MAIN];
    AVFilterLink         *outlink = ctx->outputs[0];
#if IS_FFMPEG_61_AND_ABOVE
    AVFilterLink         *inlink_overlay = ctx->inputs[OVERLAY];
#endif
    AVFrame              *frame = NULL;
    AVFrame              *overlay = NULL;
    AVFrame              *out = NULL;
    niFrameSurface1_t    *frame_surface,*new_frame_surface;
    int flags, main_cardno, ovly_cardno;
    int main_scaler_format, ovly_scaler_format;
    ni_retcode_t retcode;
    uint16_t tempFIDOverlay = 0;
    uint16_t tempFIDFrame   = 0;

#if IS_FFMPEG_61_AND_ABOVE
    av_log(ctx, AV_LOG_TRACE, "%s: ready %u inlink framequeue %u available_frame %d inlink_overlay framequeue %u available_frame %d outlink framequeue %u frame_wanted %d\n",
        __func__, ctx->ready,
        ff_inlink_queued_frames(inlink_main), ff_inlink_check_available_frame(inlink_main),
        ff_inlink_queued_frames(inlink_overlay), ff_inlink_check_available_frame(inlink_overlay),
        ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));

    // Consume from inlink framequeue only when outlink framequeue is empty, to prevent filter from exhausting all pre-allocated device buffers
    if (ff_inlink_check_available_frame(outlink))
        return FFERROR_NOT_READY;
#endif

    /* ff_framesync_get_frame() always returns 0 for hw frames */
    ff_framesync_get_frame(fs, OVERLAY, &overlay, 0);

    if (!overlay) {
        ff_framesync_get_frame(fs, MAIN, &frame, 1);
        return ff_filter_frame(ctx->outputs[0], frame);
    }

    ff_framesync_get_frame(fs, MAIN, &frame, 0);

    frame->pts =
        av_rescale_q(fs->pts, fs->time_base, ctx->outputs[0]->time_base);

    if (overlay) {
        s->var_values[VAR_OVERLAY_W] = s->var_values[VAR_OW] = overlay->width;
        s->var_values[VAR_OVERLAY_H] = s->var_values[VAR_OH] = overlay->height;
    }

    s->var_values[VAR_MAIN_W   ] = s->var_values[VAR_MW] = frame->width;
    s->var_values[VAR_MAIN_H   ] = s->var_values[VAR_MH] = frame->height;
    s->var_values[VAR_T] = frame->pts == AV_NOPTS_VALUE ?
                            NAN : frame->pts * av_q2d(inlink_main->time_base);

    // This can satisfy some customers or demos to modify the location when using ni_overlay
    set_expr(&s->x_pexpr, s->x_expr,"x", ctx);
    set_expr(&s->y_pexpr, s->y_expr,"y", ctx);

    eval_expr(ctx);
    av_log(ctx, AV_LOG_DEBUG, "x:%f xi:%d y:%f yi:%d t:%f\n",
           s->var_values[VAR_X], s->x,
           s->var_values[VAR_Y], s->y,
           s->var_values[VAR_T]);

    main_frame_ctx = (AVHWFramesContext *) frame->hw_frames_ctx->data;
    main_scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(main_frame_ctx->sw_format);

    main_cardno = ni_get_cardno(frame);

    if (overlay) {
        ovly_frame_ctx = (AVHWFramesContext *) overlay->hw_frames_ctx->data;
        ovly_scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(ovly_frame_ctx->sw_format);
        ovly_cardno        = ni_get_cardno(overlay);

        if (main_cardno != ovly_cardno) {
            av_log(ctx, AV_LOG_ERROR,
                   "Main/Overlay frames on different cards\n");
            return AVERROR(EINVAL);
        }
    } else {
        ovly_scaler_format = 0;
    }

    if (!s->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        retcode = config_output(outlink, frame);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni overlay filter config output failure\n");
            return retcode;
        }
#endif

        retcode = ni_device_session_context_init(&s->api_ctx);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni overlay filter session context init failure\n");
            return retcode;
        }

        pAVNIDevCtx = (AVNIDeviceContext *)main_frame_ctx->device_ctx->hwctx;
        s->api_ctx.device_handle = pAVNIDevCtx->cards[main_cardno];
        s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[main_cardno];

        s->api_ctx.hw_id              = main_cardno;
        s->api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
        s->api_ctx.scaler_operation   = NI_SCALER_OPCODE_OVERLAY;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
        s->api_ctx.isP2P              = s->is_p2p;

        retcode = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't open device session on card %d\n",
                   main_cardno);
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->api_ctx);
            return retcode;
        }

        s->session_opened = 1;

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           inlink_main->dst->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        retcode = init_out_pool(inlink_main->dst);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            return retcode;
        }

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(main_frame_ctx, out_frames_ctx);
        ni_device_session_copy(&s->api_ctx, &out_ni_ctx->api_ctx);

        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(main_frame_ctx->sw_format);

        if (((frame && frame->color_range == AVCOL_RANGE_JPEG) ||
            (overlay && overlay->color_range == AVCOL_RANGE_JPEG)) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(ctx, AV_LOG_WARNING,
                   "WARNING: Full color range input, limited color range output\n");
        }

        if (av_buffer_get_ref_count(frame->buf[0]) > 1) {
            av_log(ctx, AV_LOG_WARNING,
                   "WARNING: In-place overlay being used after split "
                   "filter may cause corruption\n");
        }

        s->initialized = 1;
    }

    /* Allocate a ni_frame for the overlay output */
    retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          outlink->w,
                                          outlink->h,
                                          0);

    if (retcode != NI_RETCODE_SUCCESS) {
        return AVERROR(ENOMEM);
    }

    if (overlay) {
        frame_surface = (niFrameSurface1_t *)overlay->data[3];
        tempFIDOverlay = frame_surface->ui16FrameIdx;
    } else {
        frame_surface = NULL;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    /*
     * Assign an input frame for overlay picture. Send the
     * incoming hardware frame index to the scaler manager.
     */
    retcode = ni_device_alloc_frame(
        &s->api_ctx,
        overlay ? FFALIGN(overlay->width, 2) : 0,
        overlay ? FFALIGN(overlay->height, 2) : 0,
        ovly_scaler_format,
        (frame_surface && frame_surface->encoding_type == 2) ? NI_SCALER_FLAG_CMP : 0,
        overlay ? FFALIGN(overlay->width, 2) : 0,
        overlay ? FFALIGN(overlay->height, 2) : 0,
        s->x,
        s->y,
        frame_surface ? (int)frame_surface->ui32nodeAddress : 0,
        frame_surface ? frame_surface->ui16FrameIdx : 0,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't assign frame for overlay input %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    frame_surface = (niFrameSurface1_t *) frame->data[3];
    if (frame_surface == NULL) {
        return AVERROR(EINVAL);
    }

    tempFIDFrame = frame_surface->ui16FrameIdx;
    /*
     * Allocate device output frame from the pool. We also send down the frame index
     * of the background frame to the scaler manager.
     */
    flags = (s->alpha_format ? NI_SCALER_FLAG_PA : 0) | NI_SCALER_FLAG_IO;
    flags |= (frame_surface && frame_surface->encoding_type == 2) ? NI_SCALER_FLAG_CMP : 0;
    retcode = ni_device_alloc_frame(&s->api_ctx,
                                    FFALIGN(frame->width, 2),
                                    FFALIGN(frame->height, 2),
                                    main_scaler_format,
                                    flags,
                                    FFALIGN(frame->width, 2),
                                    FFALIGN(frame->height, 2),
                                    0,                              // x
                                    0,                              // y
                                    frame_surface->ui32nodeAddress,
                                    frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate frame for output %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    out = av_frame_alloc();
    if (!out) {
        return AVERROR(ENOMEM);
    }

    av_frame_copy_props(out,frame);

    out->width = outlink->w;
    out->height = outlink->h;
    out->format = AV_PIX_FMT_NI_QUAD;

    /* Quadra 2D engine always outputs limited color range */
    out->color_range = AVCOL_RANGE_MPEG;

    /* Reference the new hw frames context */
    out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    out->data[3] = av_malloc(sizeof(niFrameSurface1_t));

    if (!out->data[3]) {
        av_frame_free(&out);
        return AVERROR(ENOMEM);
    }

    /* Copy the frame surface from the incoming frame */
    memcpy(out->data[3], frame->data[3], sizeof(niFrameSurface1_t));

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR,
               "Can't acquire output frame %d\n", retcode);
        av_frame_free(&out);
        return AVERROR(ENOMEM);
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_overlay");
#endif

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
                                          main_frame_ctx->sw_format);

    /* Remove ni-split specific assets */
    frame_surface->ui32nodeAddress = 0;

    frame_surface->ui16width = out->width;
    frame_surface->ui16height = out->height;

    av_log(ctx, AV_LOG_DEBUG,
           "%s:IN trace ui16FrameIdx = [%d] and [%d] --> out [%d] \n", __FILE__,
           tempFIDFrame, tempFIDOverlay, frame_surface->ui16FrameIdx);

    out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t), ff_ni_frame_free, NULL, 0);

    return ff_filter_frame(ctx->outputs[0], out);
}

static int process_frame_inplace(FFFrameSync *fs)
{
    AVFilterContext      *ctx = fs->parent;
    NetIntOverlayContext *s = (NetIntOverlayContext *) ctx->priv;
    AVHWFramesContext    *main_frame_ctx,*ovly_frame_ctx;
    AVNIDeviceContext    *pAVNIDevCtx;
#if IS_FFMPEG_61_AND_ABOVE
    AVFilterLink         *inlink_main = ctx->inputs[MAIN];
    AVFilterLink         *inlink_overlay = ctx->inputs[OVERLAY];
#endif
    AVFilterLink         *outlink = ctx->outputs[0];
    AVFrame              *frame = NULL;
    AVFrame              *overlay = NULL;
    AVFrame              *out = NULL;
    niFrameSurface1_t    *frame_surface;
    ni_retcode_t          retcode;
    uint16_t              ovly_frame_idx = 0;
    uint16_t              main_frame_idx = 0;
    int                   flags, main_cardno, ovly_cardno;
    int                   main_scaler_format, ovly_scaler_format;
    int                   src_x, src_y, src_w, src_h;
    int                   dst_x, dst_y, dst_w, dst_h;

#if IS_FFMPEG_61_AND_ABOVE
    av_log(ctx, AV_LOG_TRACE, "%s: ready %u inlink framequeue %u available_frame %d inlink_overlay framequeue %u available_frame %d outlink framequeue %u frame_wanted %d\n",
        __func__, ctx->ready,
        ff_inlink_queued_frames(inlink_main), ff_inlink_check_available_frame(inlink_main),
        ff_inlink_queued_frames(inlink_overlay), ff_inlink_check_available_frame(inlink_overlay),
        ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));

    // Consume from inlink framequeue only when outlink framequeue is empty, to prevent filter from exhausting all pre-allocated device buffers
    if (ff_inlink_check_available_frame(outlink))
        return FFERROR_NOT_READY;
#endif

    ff_framesync_get_frame(fs, OVERLAY, &overlay, 0);

    if (!overlay) {
        ff_framesync_get_frame(fs, MAIN, &frame, 1);
        return ff_filter_frame(ctx->outputs[0], frame);
    }

    ff_framesync_get_frame(fs, MAIN, &frame, 0);

    frame->pts =
        av_rescale_q(fs->pts, fs->time_base, ctx->outputs[0]->time_base);

    s->var_values[VAR_OVERLAY_W] = s->var_values[VAR_OW] = overlay->width;
    s->var_values[VAR_OVERLAY_H] = s->var_values[VAR_OH] = overlay->height;
    s->var_values[VAR_MAIN_W   ] = s->var_values[VAR_MW] = frame->width;
    s->var_values[VAR_MAIN_H   ] = s->var_values[VAR_MH] = frame->height;
    s->var_values[VAR_T] = frame->pts == AV_NOPTS_VALUE ?
                            NAN : frame->pts * av_q2d(ctx->inputs[0]->time_base);

    // Allow location modification
    set_expr(&s->x_pexpr, s->x_expr, "x", ctx);
    set_expr(&s->y_pexpr, s->y_expr, "y", ctx);

    eval_expr(ctx);
    av_log(ctx, AV_LOG_DEBUG, "x:%f xi:%d y:%f yi:%d t:%f\n",
           s->var_values[VAR_X], s->x,
           s->var_values[VAR_Y], s->y,
           s->var_values[VAR_T]);

    main_frame_ctx = (AVHWFramesContext *) frame->hw_frames_ctx->data;
    main_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(main_frame_ctx->sw_format);

    frame_surface = (niFrameSurface1_t *) frame->data[3];

    if (frame_surface == NULL)
        return AVERROR(EINVAL);

    main_frame_idx = frame_surface->ui16FrameIdx;

    frame_surface = (niFrameSurface1_t *) overlay->data[3];

    if (frame_surface == NULL)
        return AVERROR(EINVAL);

    ovly_frame_idx = frame_surface->ui16FrameIdx;

    main_cardno = ni_get_cardno(frame);

    ovly_frame_ctx = (AVHWFramesContext *) overlay->hw_frames_ctx->data;
    ovly_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(ovly_frame_ctx->sw_format);
    ovly_cardno = ni_get_cardno(overlay);

    if (main_cardno != ovly_cardno) {
        av_log(ctx, AV_LOG_ERROR, "Main/Overlay frames on different cards\n");
        return AVERROR(EINVAL);
    }

    // If overlay does not intersect the background, pass
    // the frame through the overlay filter.
    if (!overlay_intersects_background(ctx, overlay, frame)) {
        out = av_frame_clone(frame);

        if (!out) {
            av_log(ctx, AV_LOG_ERROR, "Can't clone frame\n");
            return AVERROR(ENOMEM);
        }
        return ff_filter_frame(ctx->outputs[0], out);
    }

    if (!s->initialized) {
        /* Set up a scaler session for the in-place overlay */
        retcode = ni_device_session_context_init(&s->api_ctx);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni overlay filter session context init failure\n");
            return retcode;
        }

        pAVNIDevCtx = (AVNIDeviceContext *)main_frame_ctx->device_ctx->hwctx;
        s->api_ctx.device_handle = pAVNIDevCtx->cards[main_cardno];
        s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[main_cardno];

        s->api_ctx.hw_id              = main_cardno;
        s->api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
        s->api_ctx.scaler_operation   = NI_SCALER_OPCODE_IPOVLY;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

        retcode = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't open device session on card %d\n",
                   main_cardno);
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->api_ctx);
            return retcode;
        }

        s->session_opened = 1;

        // If the in-place overlay is rgba over yuv, we need to set up
        // an extra intermediate crop session.
        if (s->overlay_has_alpha && !s->main_has_alpha) {
            /* Set up a scaler session for the crop operation */
            retcode = ni_device_session_context_init(&s->crop_api_ctx);
            if (retcode < 0) {
                av_log(ctx, AV_LOG_ERROR,
                       "ni overlay filter (crop) session context init failure\n");
                return retcode;
            }

            s->crop_api_ctx.device_handle = pAVNIDevCtx->cards[main_cardno];
            s->crop_api_ctx.blk_io_handle = pAVNIDevCtx->cards[main_cardno];

            s->crop_api_ctx.hw_id              = main_cardno;
            s->crop_api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
            s->crop_api_ctx.scaler_operation   = NI_SCALER_OPCODE_CROP;
            s->crop_api_ctx.keep_alive_timeout = s->keep_alive_timeout;

            retcode = ni_device_session_open(&s->crop_api_ctx,
                                             NI_DEVICE_TYPE_SCALER);
            if (retcode != NI_RETCODE_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR,
                       "Can't open device session on card %d\n", main_cardno);
                ni_device_session_close(&s->crop_api_ctx, 1, NI_DEVICE_TYPE_SCALER);
                ni_device_session_context_clear(&s->crop_api_ctx);
                return retcode;
            }

            s->crop_session_opened = 1;

            /* init the out pool for the crop session, make it rgba */
            retcode = ff_ni_build_frame_pool(&s->crop_api_ctx, overlay->width,
                                             overlay->height,
                                             AV_PIX_FMT_RGBA, 1, 0);

            if (retcode < 0) {
                av_log(ctx, AV_LOG_ERROR,
                       "Internal output allocation failed rc = %d\n", retcode);
                return retcode;
            }
        }
#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           inlink_main->dst->extra_hw_frames = ((DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0) - DEFAULT_NI_FILTER_POOL_SIZE;
        }
        retcode = init_out_pool(inlink_main->dst);

        if (retcode < 0) {
            av_log(inlink_main->dst, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            return retcode;
        }
#endif
        s->initialized = 1;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    /* For rgba over yuv, we do an intermediate crop and overlay */
    if (s->overlay_has_alpha && !s->main_has_alpha) {
        retcode = do_intermediate_crop_and_overlay(ctx, overlay, frame);

        if (retcode < 0)
            return retcode;

        /* Allocate a ni_frame for the overlay output */
        retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                              outlink->w,
                                              outlink->h,
                                              0);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't allocate inplace overlay frame\n");
            return AVERROR(ENOMEM);
        }

        calculate_src_rectangle(&src_x, &src_y, &src_w, &src_h,
                                0, 0, frame->width, frame->height,
                                FFALIGN(s->x,2),FFALIGN(s->y,2), overlay->width, overlay->height);

        /*
         * Assign an input frame for overlay picture. Send the
         * incoming hardware frame index to the scaler manager.
         */
        retcode = ni_device_alloc_frame(
            &s->api_ctx,
            FFALIGN(overlay->width, 2),  // ovly width
            FFALIGN(overlay->height, 2), // ovly height
            ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA), // ovly pix fmt
            0,                           // flags
            src_w,                       // src rect width
            src_h,                       // src rect height
            0,                           // src rect x
            0,                           // src rect y
            0,                           // n/a
            s->ui16CropFrameIdx,         // ovly frame idx
            NI_DEVICE_TYPE_SCALER);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't assign input overlay frame %d\n",
                   retcode);
            return AVERROR(ENOMEM);
        }

        calculate_dst_rectangle(&dst_x, &dst_y, &dst_w, &dst_h,
                                0, 0, frame->width, frame->height,
                                FFALIGN(s->x,2), FFALIGN(s->y, 2),
                                overlay->width, overlay->height);

        /*
         * Allocate device output frame from the pool. We also send down the
         * frame index of the background frame to the scaler manager.
         */

        /* configure the output */
        flags = NI_SCALER_FLAG_IO;
        /* premultiply vs straight alpha */
        flags |= (s->alpha_format) ? NI_SCALER_FLAG_PA : 0;

        retcode = ni_device_alloc_frame(
            &s->api_ctx,
            FFALIGN(frame->width, 2),       // main width
            FFALIGN(frame->height, 2),      // main height
            main_scaler_format,             // main pix fmt
            flags,                          // flags
            dst_w,                          // dst rect width
            dst_h,                          // dst rect height
            dst_x,                          // dst rect x
            dst_y,                          // dst rect y
            0,                              // n/a
            main_frame_idx,                 // main frame idx
            NI_DEVICE_TYPE_SCALER);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't allocate overlay output %d\n",
                   retcode);
            return AVERROR(ENOMEM);
        }

        /* Set the new frame index */
        retcode = ni_device_session_read_hwdesc(&s->api_ctx,
                                                &s->api_dst_frame,
                                                NI_DEVICE_TYPE_SCALER);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can't acquire output overlay frame %d\n", retcode);
            return AVERROR(ENOMEM);
        }
    } else {
        /* Not rgba over yuv. For yuv over yuv, yuv over rgba, */
        /* rgba over rgba, we can perform an in-place overlay immediately. */

        /* Allocate ni_frame for the overlay output */
        retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                              outlink->w,
                                              outlink->h,
                                              0);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Cannot allocate in-place frame\n");
            return AVERROR(ENOMEM);
        }

        calculate_src_rectangle(&src_x, &src_y, &src_w, &src_h,
                                0, 0, frame->width, frame->height,
                                FFALIGN(s->x,2), FFALIGN(s->y,2),
                                overlay->width, overlay->height);

        /*
         * Assign input frame for overlay picture. Sends the
         * incoming hardware frame index to the scaler manager.
         */
        retcode = ni_device_alloc_frame(
            &s->api_ctx,
            FFALIGN(overlay->width, 2),         // overlay width
            FFALIGN(overlay->height, 2),        // overlay height
            ovly_scaler_format,                 // overlay pix fmt
            0,                                  // flags
            src_w,                              // src rect width
            src_h,                              // src rect height
            src_x,                              // src rect x
            src_y,                              // src rect y
            0,                                  // n/a
            ovly_frame_idx,                     // overlay frame idx
            NI_DEVICE_TYPE_SCALER);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can't assign frame for overlay input %d\n", retcode);
            return AVERROR(ENOMEM);
        }

        /* In-place overlay frame. Send down frame index of background frame */

        /* Configure the output */
        flags = NI_SCALER_FLAG_IO;
        /* Premultiply vs straight alpha */
        flags |= s->alpha_format ? NI_SCALER_FLAG_PA : 0;

        calculate_dst_rectangle(&dst_x,&dst_y,&dst_w,&dst_h,
                                0,0,frame->width,frame->height,
                                FFALIGN(s->x,2),FFALIGN(s->y,2),
                                overlay->width,overlay->height);

        retcode = ni_device_alloc_frame(
            &s->api_ctx,
            FFALIGN(frame->width, 2),       // main width
            FFALIGN(frame->height, 2),      // main height
            main_scaler_format,             // main pix fmt
            flags,                          // flags
            dst_w,                          // dst rect width
            dst_h,                          // dst rect height
            dst_x,                          // dst rect x
            dst_y,                          // dst rect y
            0,                              // n/a
            main_frame_idx,                 // main frame idx
            NI_DEVICE_TYPE_SCALER);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can't allocate frame for output ovly %d\n", retcode);
            return AVERROR(ENOMEM);
        }

        retcode = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                                NI_DEVICE_TYPE_SCALER);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can't acquire output frame of overlay %d\n", retcode);
            return AVERROR(ENOMEM);
        }
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_overlay");
#endif

    /* Do an in-place overlay onto the background frame */
    out = av_frame_clone(frame);

    if (!out) {
        av_log(ctx, AV_LOG_ERROR, "Cannot clone frame\n");
        return AVERROR(ENOMEM);
    }

    /* Quadra 2D engine always outputs limited color range */
    out->color_range = AVCOL_RANGE_MPEG;

    if (s->overlay_has_alpha && !s->main_has_alpha) {
        av_log(ctx, AV_LOG_DEBUG,
            "%s:IN trace ui16FrameIdx = [%d] and [%d] and [%d] --> out [%d]\n",
            __func__, main_frame_idx, ovly_frame_idx, s->ui16CropFrameIdx,
            main_frame_idx);
    } else {
        av_log(ctx, AV_LOG_DEBUG,
           "%s:IN trace ui16FrameIdx = [%d] and [%d] --> out [%d]\n",
           __func__, main_frame_idx, ovly_frame_idx, main_frame_idx);
    }

    if (s->overlay_has_alpha && !s->main_has_alpha) {
        ni_hwframe_buffer_recycle((niFrameSurface1_t *)
                                  s->crop_api_dst_frame.data.frame.p_data[3],
                                  (int32_t)s->crop_api_ctx.device_handle);
    }

    return ff_filter_frame(ctx->outputs[0], out);
}

/**
 * Blend image in src to destination buffer dst at position (x, y).
 */
#if IS_FFMPEG_342_AND_ABOVE
static int config_input_main(AVFilterLink *inlink)
#else
static int config_input_main(AVFilterLink *inlink, AVFrame *in)
#endif
{
    NetIntOverlayContext *s = inlink->dst->priv;
    AVHWFramesContext *in_frames_ctx;
    const AVPixFmtDescriptor *pix_desc;

    // Check for incompatible options
    if (s->inplace && s->is_p2p) {
        av_log(inlink->dst, AV_LOG_ERROR, "Cannot use 'inplace' and 'is_p2p' options together\n");
        return AVERROR(EINVAL);
    }

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
        av_log(inlink->dst, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (inlink->hw_frames_ctx == NULL) {
        av_log(inlink->dst, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
#else
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif
    if (!in_frames_ctx) {
        return AVERROR(EINVAL);
    }

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
        av_log(inlink->dst, AV_LOG_ERROR,
               "bgrp not supported for background\n");
        return AVERROR(EINVAL);
    }

    if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(inlink->dst, AV_LOG_ERROR, "tile4x4 10b not supported for overlay!\n");
        return AVERROR(EINVAL);
    }

    s->main_has_alpha = ff_fmt_is_in(in_frames_ctx->sw_format, alpha_pix_fmts);

    pix_desc = av_pix_fmt_desc_get(in_frames_ctx->sw_format);

    s->hsub = pix_desc->log2_chroma_w;
    s->vsub = pix_desc->log2_chroma_h;

    return 0;
}

#if !IS_FFMPEG_342_AND_ABOVE
static int filter_frame(AVFilterLink *inlink, AVFrame *inpicref)
{
    AVFilterContext *ctx = inlink->dst;
    NetIntOverlayContext *s = ctx->priv;

    av_log(inlink->dst, AV_LOG_DEBUG, "Incoming frame (time:%s) from link #%d\n", av_ts2timestr(inpicref->pts, &inlink->time_base), FF_INLINK_IDX(inlink));

    if (!s->config_input_initialized[MAIN] && ctx->inputs[MAIN] == inlink) {
        config_input_main(inlink, inpicref);
        s->config_input_initialized[MAIN] = 1;
    }

    if (!s->config_input_initialized[OVERLAY] && ctx->inputs[OVERLAY] == inlink) {
        config_input_overlay(inlink, inpicref);
        s->config_input_initialized[OVERLAY] = 1;
    }

    return ff_framesync_filter_frame(&s->fs, inlink, inpicref);
}

static int request_frame(AVFilterLink *outlink)
{
    NetIntOverlayContext *s = outlink->src->priv;
    return ff_framesync_request_frame(&s->fs, outlink);
}
#else
static int activate(AVFilterContext *ctx)
{
    NetIntOverlayContext *s = ctx->priv;
    return ff_framesync_activate(&s->fs);
}
#endif

#define OFFSET(x) offsetof(NetIntOverlayContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_overlay_options[] = {
    { "x",       "set the x expression", OFFSET(x_expr),       AV_OPT_TYPE_STRING, {.str = "0"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "y",       "set the y expression", OFFSET(y_expr),       AV_OPT_TYPE_STRING, {.str = "0"}, CHAR_MIN, CHAR_MAX, FLAGS },
    { "alpha",   "alpha format",         OFFSET(alpha_format), AV_OPT_TYPE_INT,    {.i64 = 0},   0,        1,        FLAGS, "alpha_format" },
        { "straight",      "", 0, AV_OPT_TYPE_CONST, {.i64 = 0}, .flags = FLAGS, .unit = "alpha_format" },
        { "premultiplied", "", 0, AV_OPT_TYPE_CONST, {.i64 = 1}, .flags = FLAGS, .unit = "alpha_format" },
    { "inplace", "overlay in-place",     OFFSET(inplace),      AV_OPT_TYPE_BOOL,   {.i64 = 0},   0,        1,        FLAGS },
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

#if IS_FFMPEG_342_AND_ABOVE
// NOLINTNEXTLINE(clang-diagnostic-deprecated-declarations)
FRAMESYNC_DEFINE_CLASS(ni_overlay, NetIntOverlayContext, fs);
#else
AVFILTER_DEFINE_CLASS(ni_overlay);
#endif

static const AVFilterPad inputs[] = {
    {
        .name         = "main",
        .type         = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_input_main,
#else
        .filter_frame = filter_frame,
#endif
    },
    {
        .name         = "overlay",
        .type         = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_input_overlay,
#else
        .filter_frame = filter_frame,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props  = config_output,
#else
        .config_props = config_output_dummy,
        .request_frame = request_frame,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_overlay_ni_quadra = {
    .name          = "ni_quadra_overlay",
    .description   = NULL_IF_CONFIG_SMALL("NETINT Quadra overlay a video source on top of the input v" NI_XCODER_REVISION),
    .uninit        = uninit,
    .priv_size     = sizeof(NetIntOverlayContext),
    .priv_class    = &ni_overlay_class,
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs        = inputs,
    .outputs       = outputs,
    .query_formats = query_formats,
#endif
// only FFmpeg 3.4.2 and above have .flags_internal
#if IS_FFMPEG_342_AND_ABOVE
    .preinit       = ni_overlay_framesync_preinit,
    .activate      = activate,
    .flags_internal= FF_FILTER_FLAG_HWFRAME_AWARE
#endif
};
