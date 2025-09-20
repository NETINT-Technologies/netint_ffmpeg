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
 * scale video filter
 */

#include <stdio.h>
#include <string.h>

#include "nifilter.h"
#include "filters.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "framesync.h"
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
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/avassert.h"
#include "libavutil/eval.h"
#include "libswscale/swscale.h"

static const char *const var_names[] = {
    "in_w",   "iw",
    "in_h",   "ih",
    "out_w",  "ow",
    "out_h",  "oh",
    "a",
    "sar",
    "dar",
    "hsub",
    "vsub",
    "ohsub",
    "ovsub",
    "ref_w", "rw",
    "ref_h", "rh",
    "ref_a",
    "ref_sar",
    "ref_dar", "rdar",
    "ref_hsub",
    "ref_vsub",
    "main_w",
    "main_h",
    "main_a",
    "main_sar",
    "main_dar", "mdar",
    "main_hsub",
    "main_vsub",
    NULL
};

enum var_name {
    VAR_IN_W,   VAR_IW,
    VAR_IN_H,   VAR_IH,
    VAR_OUT_W,  VAR_OW,
    VAR_OUT_H,  VAR_OH,
    VAR_A,
    VAR_SAR,
    VAR_DAR,
    VAR_HSUB,
    VAR_VSUB,
    VAR_OHSUB,
    VAR_OVSUB,
    VAR_REF_W, VAR_RW,
    VAR_REF_H, VAR_RH,
    VAR_REF_A,
    VAR_REF_SAR,
    VAR_REF_DAR, VAR_RDAR,
    VAR_REF_HSUB,
    VAR_REF_VSUB,
    VAR_S2R_MAIN_W,
    VAR_S2R_MAIN_H,
    VAR_S2R_MAIN_A,
    VAR_S2R_MAIN_SAR,
    VAR_S2R_MAIN_DAR, VAR_S2R_MDAR,
    VAR_S2R_MAIN_HSUB,
    VAR_S2R_MAIN_VSUB,
    VARS_NB
};

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

enum AVPixelFormat ff_output_fmt[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUYV422, AV_PIX_FMT_UYVY422,
    AV_PIX_FMT_NV12,    AV_PIX_FMT_ARGB,    AV_PIX_FMT_RGBA,
    AV_PIX_FMT_ABGR,    AV_PIX_FMT_BGRA,    AV_PIX_FMT_YUV420P10LE,
    AV_PIX_FMT_NV16,    AV_PIX_FMT_BGR0,    AV_PIX_FMT_P010LE,
    AV_PIX_FMT_BGRP};

typedef struct NetIntScaleContext {
    const AVClass *class;
    AVDictionary *opts;

#if IS_FFMPEG_71_AND_ABOVE
    FFFrameSync fs;
#endif

    int uses_ref;

    /**
     * New dimensions. Special values are:
     *   0 = original width/height
     *  -1 = keep original aspect
     *  -N = try to keep aspect but make sure it is divisible by N
     */
    int w, h;
    char *size_str;

    char *w_expr;               ///< width  expression string
    char *h_expr;               ///< height expression string

    char *flags_str;

    char *in_color_matrix;
    char *out_color_matrix;

    int force_original_aspect_ratio;
    int force_divisible_by;
    int format;

    enum AVPixelFormat out_format;
    AVBufferRef *out_frames_ref;
    AVBufferRef *out_frames_ref_1;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
    ni_scaler_params_t params;

    int initialized;
    int session_opened;
    int keep_alive_timeout; /* keep alive timeout setting */
    int output_compressed;
    bool is_p2p;

    int auto_skip;
    int skip_filter;
    int autoselect;
    int buffer_limit;
    AVExpr *w_pexpr;
    AVExpr *h_pexpr;
    double var_values[VARS_NB];
} NetIntScaleContext;

AVFilter ff_vf_scale_ni;
AVFilter ff_vf_scale2ref_ni_quadra;
#if IS_FFMPEG_342_AND_ABOVE
static int config_props(AVFilterLink *outlink);
#else
static int config_props(AVFilterLink *outlink, AVFrame *in);
#endif

#if IS_FFMPEG_71_AND_ABOVE
static int do_scale(FFFrameSync *fs);
#endif

#if ((LIBAVFILTER_VERSION_MAJOR >= 8) || ((LIBAVFILTER_VERSION_MAJOR == 7) && (LIBAVFILTER_VERSION_MINOR >= 85)))
static int check_exprs(AVFilterContext *ctx)
{
    NetIntScaleContext *scale = ctx->priv;
    unsigned vars_w[VARS_NB] = { 0 }, vars_h[VARS_NB] = { 0 };

    if (!scale->w_pexpr && !scale->h_pexpr)
        return AVERROR(EINVAL);

    if (scale->w_pexpr)
        av_expr_count_vars(scale->w_pexpr, vars_w, VARS_NB);
    if (scale->h_expr)
        av_expr_count_vars(scale->h_pexpr, vars_h, VARS_NB);

    if (vars_w[VAR_OUT_W] || vars_w[VAR_OW]) {
        av_log(ctx, AV_LOG_ERROR, "Width expression cannot be self-referencing: '%s'.\n", scale->w_expr);
        return AVERROR(EINVAL);
    }

    if (vars_h[VAR_OUT_H] || vars_h[VAR_OH]) {
        av_log(ctx, AV_LOG_ERROR, "Height expression cannot be self-referencing: '%s'.\n", scale->h_expr);
        return AVERROR(EINVAL);
    }

    if ((vars_w[VAR_OUT_H] || vars_w[VAR_OH]) &&
        (vars_h[VAR_OUT_W] || vars_h[VAR_OW])) {
        av_log(ctx, AV_LOG_WARNING, "Circular references detected for width '%s' and height '%s' - possibly invalid.\n", scale->w_expr, scale->h_expr);
    }

    if (vars_w[VAR_REF_W]    || vars_h[VAR_REF_W]    ||
        vars_w[VAR_RW]       || vars_h[VAR_RW]       ||
        vars_w[VAR_REF_H]    || vars_h[VAR_REF_H]    ||
        vars_w[VAR_RH]       || vars_h[VAR_RH]       ||
        vars_w[VAR_REF_A]    || vars_h[VAR_REF_A]    ||
        vars_w[VAR_REF_SAR]  || vars_h[VAR_REF_SAR]  ||
        vars_w[VAR_REF_DAR]  || vars_h[VAR_REF_DAR]  ||
        vars_w[VAR_RDAR]     || vars_h[VAR_RDAR]     ||
        vars_w[VAR_REF_HSUB] || vars_h[VAR_REF_HSUB] ||
        vars_w[VAR_REF_VSUB] || vars_h[VAR_REF_VSUB]) {
        scale->uses_ref = 1;
    }

    if (ctx->filter != &ff_vf_scale2ref_ni_quadra &&
        (vars_w[VAR_S2R_MAIN_W]    || vars_h[VAR_S2R_MAIN_W]    ||
         vars_w[VAR_S2R_MAIN_H]    || vars_h[VAR_S2R_MAIN_H]    ||
         vars_w[VAR_S2R_MAIN_A]    || vars_h[VAR_S2R_MAIN_A]    ||
         vars_w[VAR_S2R_MAIN_SAR]  || vars_h[VAR_S2R_MAIN_SAR]  ||
         vars_w[VAR_S2R_MAIN_DAR]  || vars_h[VAR_S2R_MAIN_DAR]  ||
         vars_w[VAR_S2R_MDAR]      || vars_h[VAR_S2R_MDAR]      ||
         vars_w[VAR_S2R_MAIN_HSUB] || vars_h[VAR_S2R_MAIN_HSUB] ||
         vars_w[VAR_S2R_MAIN_VSUB] || vars_h[VAR_S2R_MAIN_VSUB])) {
        av_log(ctx, AV_LOG_ERROR, "Expressions with scale2ref variables are not valid in scale filter.\n");
        return AVERROR(EINVAL);
    }

    return 0;
}

static int scale_parse_expr(AVFilterContext *ctx, char *str_expr, AVExpr **pexpr_ptr, const char *var, const char *args)
{
    NetIntScaleContext *scale = ctx->priv;
    int ret, is_inited = 0;
    char *old_str_expr = NULL;
    AVExpr *old_pexpr = NULL;

    if (str_expr) {
        old_str_expr = av_strdup(str_expr);
        if (!old_str_expr)
            return AVERROR(ENOMEM);
        av_opt_set(scale, var, args, 0);
    }

    if (*pexpr_ptr) {
        old_pexpr = *pexpr_ptr;
        *pexpr_ptr = NULL;
        is_inited = 1;
    }

    ret = av_expr_parse(pexpr_ptr, args, var_names,
                        NULL, NULL, NULL, NULL, 0, ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Cannot parse expression for %s: '%s'\n", var, args);
        goto revert;
    }

    ret = check_exprs(ctx);
    if (ret < 0)
        goto revert;

    if (is_inited && (ret = config_props(ctx->outputs[0])) < 0)
        goto revert;

    av_expr_free(old_pexpr);
    old_pexpr = NULL;
    av_freep(&old_str_expr);

    return 0;

revert:
    av_expr_free(*pexpr_ptr);
    *pexpr_ptr = NULL;
    if (old_str_expr) {
        av_opt_set(scale, var, old_str_expr, 0);
        av_free(old_str_expr);
    }
    if (old_pexpr)
        *pexpr_ptr = old_pexpr;

    return ret;
}

static int scale_eval_dimensions(AVFilterContext *ctx)
{
    NetIntScaleContext *scale = ctx->priv;
    int scale2ref = ctx->filter == &ff_vf_scale2ref_ni_quadra;
    const AVFilterLink *inlink = scale2ref ? ctx->inputs[1] : ctx->inputs[0];
    const AVFilterLink *outlink = ctx->outputs[0];
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);
    const AVPixFmtDescriptor *out_desc = av_pix_fmt_desc_get(outlink->format);
    char *expr;
    int ret;
    double res;
    const AVPixFmtDescriptor *main_desc;
    const AVFilterLink *main_link;

    if (scale2ref) {
        main_link = ctx->inputs[0];
        main_desc = av_pix_fmt_desc_get(main_link->format);
    }

    scale->var_values[VAR_IN_W]  = scale->var_values[VAR_IW] = inlink->w;
    scale->var_values[VAR_IN_H]  = scale->var_values[VAR_IH] = inlink->h;
    scale->var_values[VAR_OUT_W] = scale->var_values[VAR_OW] = NAN;
    scale->var_values[VAR_OUT_H] = scale->var_values[VAR_OH] = NAN;
    scale->var_values[VAR_A]     = (double) inlink->w / inlink->h;
    scale->var_values[VAR_SAR]   = inlink->sample_aspect_ratio.num ?
        (double) inlink->sample_aspect_ratio.num / inlink->sample_aspect_ratio.den : 1;
    scale->var_values[VAR_DAR]   = scale->var_values[VAR_A] * scale->var_values[VAR_SAR];
    scale->var_values[VAR_HSUB]  = 1 << desc->log2_chroma_w;
    scale->var_values[VAR_VSUB]  = 1 << desc->log2_chroma_h;
    scale->var_values[VAR_OHSUB] = 1 << out_desc->log2_chroma_w;
    scale->var_values[VAR_OVSUB] = 1 << out_desc->log2_chroma_h;

    if (scale2ref) {
        scale->var_values[VAR_S2R_MAIN_W] = main_link->w;
        scale->var_values[VAR_S2R_MAIN_H] = main_link->h;
        scale->var_values[VAR_S2R_MAIN_A] = (double) main_link->w / main_link->h;
        scale->var_values[VAR_S2R_MAIN_SAR] = main_link->sample_aspect_ratio.num ?
            (double) main_link->sample_aspect_ratio.num / main_link->sample_aspect_ratio.den : 1;
        scale->var_values[VAR_S2R_MAIN_DAR] = scale->var_values[VAR_S2R_MDAR] =
            scale->var_values[VAR_S2R_MAIN_A] * scale->var_values[VAR_S2R_MAIN_SAR];
        scale->var_values[VAR_S2R_MAIN_HSUB] = 1 << main_desc->log2_chroma_w;
        scale->var_values[VAR_S2R_MAIN_VSUB] = 1 << main_desc->log2_chroma_h;
    }

    if (scale->uses_ref) {
        const AVFilterLink *reflink = ctx->inputs[1];
        const AVPixFmtDescriptor *ref_desc = av_pix_fmt_desc_get(reflink->format);
        scale->var_values[VAR_REF_W] = scale->var_values[VAR_RW] = reflink->w;
        scale->var_values[VAR_REF_H] = scale->var_values[VAR_RH] = reflink->h;
        scale->var_values[VAR_REF_A] = (double) reflink->w / reflink->h;
        scale->var_values[VAR_REF_SAR] = reflink->sample_aspect_ratio.num ?
            (double) reflink->sample_aspect_ratio.num / reflink->sample_aspect_ratio.den : 1;
        scale->var_values[VAR_REF_DAR] = scale->var_values[VAR_RDAR] =
            scale->var_values[VAR_REF_A] * scale->var_values[VAR_REF_SAR];
        scale->var_values[VAR_REF_HSUB] = 1 << ref_desc->log2_chroma_w;
        scale->var_values[VAR_REF_VSUB] = 1 << ref_desc->log2_chroma_h;
    }

    res = av_expr_eval(scale->w_pexpr, scale->var_values, NULL);
    scale->var_values[VAR_OUT_W] = scale->var_values[VAR_OW] = (int) res == 0 ? inlink->w : (int) res;

    res = av_expr_eval(scale->h_pexpr, scale->var_values, NULL);
    if (isnan(res)) {
        expr = scale->h_expr;
        ret = AVERROR(EINVAL);
        goto fail;
    }
    scale->var_values[VAR_OUT_H] = scale->var_values[VAR_OH] = (int) res == 0 ? inlink->h : (int) res;

    res = av_expr_eval(scale->w_pexpr, scale->var_values, NULL);
    if (isnan(res)) {
        expr = scale->w_expr;
        ret = AVERROR(EINVAL);
        goto fail;
    }
    scale->var_values[VAR_OUT_W] = scale->var_values[VAR_OW] = (int) res == 0 ? inlink->w : (int) res;

    scale->w = (int)scale->var_values[VAR_OUT_W];
    scale->h = (int)scale->var_values[VAR_OUT_H];

    return 0;

fail:
    av_log(ctx, AV_LOG_ERROR,
           "Error when evaluating the expression '%s'.\n", expr);
    return ret;
}
#endif

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

#if (LIBAVFILTER_VERSION_MAJOR > 9 || (LIBAVFILTER_VERSION_MAJOR == 9 && LIBAVFILTER_VERSION_MINOR >= 3))
static av_cold int init(AVFilterContext *ctx)
#else
static av_cold int init_dict(AVFilterContext *ctx, AVDictionary **opts)
#endif
{
    NetIntScaleContext *scale = ctx->priv;
    int ret;

#if IS_FFMPEG_71_AND_ABOVE
    if (ctx->filter == &ff_vf_scale2ref_ni_quadra) {
        av_log(ctx, AV_LOG_ERROR, "ni_quadra_scale2ref is deprecated, use ni_quadra_scale=rw:rh instead\n");
        return AVERROR(EINVAL);
    }
#endif

    if (scale->size_str && (scale->w_expr || scale->h_expr)) {
        av_log(ctx, AV_LOG_ERROR,
               "Size and width/height expressions cannot be set at the same time.\n");
            return AVERROR(EINVAL);
    }

    if (scale->w_expr && !scale->h_expr)
        FFSWAP(char *, scale->w_expr, scale->size_str);

    if (scale->size_str) {
        char buf[32];

        if ((ret = av_parse_video_size(&scale->w, &scale->h, scale->size_str)) < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Invalid size '%s'\n", scale->size_str);
            return ret;
        }
        snprintf(buf, sizeof(buf)-1, "%d", scale->w);
        av_opt_set(scale, "w", buf, 0);
        snprintf(buf, sizeof(buf)-1, "%d", scale->h);
        av_opt_set(scale, "h", buf, 0);
    }
    if (!scale->w_expr)
        av_opt_set(scale, "w", "iw", 0);
    if (!scale->h_expr)
        av_opt_set(scale, "h", "ih", 0);

#if ((LIBAVFILTER_VERSION_MAJOR >= 8) || ((LIBAVFILTER_VERSION_MAJOR == 7) && (LIBAVFILTER_VERSION_MINOR >= 85)))
    ret = scale_parse_expr(ctx, NULL, &scale->w_pexpr, "width", scale->w_expr);
    if (ret < 0)
        return ret;

    ret = scale_parse_expr(ctx, NULL, &scale->h_pexpr, "height", scale->h_expr);
    if (ret < 0)
        return ret;
#endif

    av_log(ctx, AV_LOG_VERBOSE, "w:%s h:%s\n", scale->w_expr, scale->h_expr);

#if (LIBAVFILTER_VERSION_MAJOR > 9 || (LIBAVFILTER_VERSION_MAJOR == 9 && LIBAVFILTER_VERSION_MINOR >= 3))
#else
    scale->opts = *opts;
    *opts = NULL;
#endif

#if IS_FFMPEG_71_AND_ABOVE
    if (ctx->filter != &ff_vf_scale2ref_ni_quadra && scale->uses_ref) {
        AVFilterPad pad = {
            .name = "ref",
            .type = AVMEDIA_TYPE_VIDEO,
        };
        ret = ff_append_inpad(ctx, &pad);
        if (ret < 0) {
            return ret;
        }
    }
#endif

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntScaleContext *scale = ctx->priv;

    av_expr_free(scale->w_pexpr);
    av_expr_free(scale->h_pexpr);
    scale->w_pexpr = scale->h_pexpr = NULL;
    av_dict_free(&scale->opts);

    if (scale->api_dst_frame.data.frame.p_buffer)
        ni_frame_buffer_free(&scale->api_dst_frame.data.frame);

    if (scale->session_opened) {
        /* Close operation will free the device frames */
        ni_device_session_close(&scale->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&scale->api_ctx);
    }

    av_buffer_unref(&scale->out_frames_ref);
    av_buffer_unref(&scale->out_frames_ref_1);

#if IS_FFMPEG_71_AND_ABOVE
    if (scale->uses_ref) {
        ff_framesync_uninit(&scale->fs);
    }
#endif
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntScaleContext *s = ctx->priv;
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
                                  s->out_format, pool_size,
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
    AVFilterLink *inlink  = ctx->filter == &ff_vf_scale2ref_ni_quadra ?
                            outlink->src->inputs[1] : outlink->src->inputs[0];
    // AVFilterLink *inlink = outlink->src->inputs[0];
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    NetIntScaleContext *scale = ctx->priv;
    int w, h, ret, h_shift, v_shift;

#if ((LIBAVFILTER_VERSION_MAJOR >= 8) || ((LIBAVFILTER_VERSION_MAJOR == 7) && (LIBAVFILTER_VERSION_MINOR >= 85)))
    // FFmpeg 4.3.0 onwards
    if ((ret = scale_eval_dimensions(ctx)) < 0)
        goto fail;

    w = scale->w;
    h = scale->h;

    ff_scale_adjust_dimensions(inlink, &w, &h,
                               scale->force_original_aspect_ratio,
                               scale->force_divisible_by);
#else
    if ((ret = ff_scale_eval_dimensions(ctx,
                                        scale->w_expr, scale->h_expr,
                                        inlink, outlink,
                                        &w, &h)) < 0)
        goto fail;

    /* Note that force_original_aspect_ratio may overwrite the previous set
     * dimensions so that it is not divisible by the set factors anymore
     * unless force_divisible_by is defined as well */
    if (scale->force_original_aspect_ratio) {
        int tmp_w = av_rescale(h, inlink->w, inlink->h);
        int tmp_h = av_rescale(w, inlink->h, inlink->w);

        if (scale->force_original_aspect_ratio == 1) {
            w = FFMIN(tmp_w, w);
            h = FFMIN(tmp_h, h);
            if (scale->force_divisible_by > 1) {
                // round down
                w = w / scale->force_divisible_by * scale->force_divisible_by;
                h = h / scale->force_divisible_by * scale->force_divisible_by;
            }
        } else {
            w = FFMAX(tmp_w, w);
            h = FFMAX(tmp_h, h);
            if (scale->force_divisible_by > 1) {
                // round up
                w = (w + scale->force_divisible_by - 1) / scale->force_divisible_by * scale->force_divisible_by;
                h = (h + scale->force_divisible_by - 1) / scale->force_divisible_by * scale->force_divisible_by;
            }
        }
    }
#endif

    if (w > NI_MAX_RESOLUTION_WIDTH || h > NI_MAX_RESOLUTION_HEIGHT) {
        av_log(ctx, AV_LOG_ERROR, "Scaled value (%dx%d) > 8192 not allowed\n", w, h);
        return AVERROR(EINVAL);
    }

    if ((w <= 0) || (h <= 0)) {
        av_log(ctx, AV_LOG_ERROR, "Scaled value (%dx%d) not allowed\n", w, h);
        return AVERROR(EINVAL);
    }

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
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
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
        av_log(ctx, AV_LOG_ERROR, "bgrp not supported\n");
        return AVERROR(EINVAL);
    }
    if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(ctx, AV_LOG_ERROR, "tile4x4 10b not supported for scale!\n");
        return AVERROR(EINVAL);
    }

    /* Set the output format */
    if (scale->format == OUTPUT_FORMAT_AUTO) {
        scale->out_format = in_frames_ctx->sw_format;
    } else {
        scale->out_format = ff_output_fmt[scale->format];
    }
    if (scale->out_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4)
        scale->output_compressed = 1;
    else
        scale->output_compressed = 0;

    av_pix_fmt_get_chroma_sub_sample(scale->out_format, &h_shift, &v_shift);

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

    //skip the color range check
    if (scale->auto_skip &&
        !scale->is_p2p && //the input and output are always on the same card, but filter with p2p always need to be executed
        inlink->w == outlink->w &&
        inlink->h == outlink->h &&
        in_frames_ctx->sw_format == scale->out_format &&
        (
         (!scale->in_color_matrix && (!scale->out_color_matrix || strcmp(scale->out_color_matrix, "bt709") == 0)) ||
         (!scale->out_color_matrix && (!scale->in_color_matrix || strcmp(scale->in_color_matrix, "bt709") == 0)) ||
         (scale->in_color_matrix && scale->out_color_matrix && strcmp(scale->in_color_matrix, scale->out_color_matrix) == 0)
        )
       ) {
        //skip hardware scale
        scale->skip_filter = 1;

#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *lo = ff_filter_link(outlink);
        scale->out_frames_ref = av_buffer_ref(li->hw_frames_ctx);
        if (!scale->out_frames_ref) {
            return AVERROR(ENOMEM);
        }
        av_buffer_unref(&lo->hw_frames_ctx);
        lo->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref);
        if (!lo->hw_frames_ctx) {
            return AVERROR(ENOMEM);
        }
#else
        AVFilterLink *inlink_for_skip = outlink->src->inputs[0];
        scale->out_frames_ref = av_buffer_ref(inlink_for_skip->hw_frames_ctx);
        if (!scale->out_frames_ref) {
            return AVERROR(ENOMEM);
        }

        av_buffer_unref(&outlink->hw_frames_ctx);
        outlink->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref);
        if (!outlink->hw_frames_ctx) {
            return AVERROR(ENOMEM);
        }
#endif
        return 0;
    }

    scale->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!scale->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)scale->out_frames_ref->data;

    out_frames_ctx->format    = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width     = outlink->w;
    out_frames_ctx->height    = outlink->h;
    out_frames_ctx->sw_format = scale->out_format;
    out_frames_ctx->initial_pool_size =
        NI_SCALE_ID; // Repurposed as identity code

    av_hwframe_ctx_init(scale->out_frames_ref);//call this?

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lt = ff_filter_link(ctx->outputs[0]);
    av_buffer_unref(&lt->hw_frames_ctx);
    lt->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref);
    if (!lt->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
    av_buffer_unref(&ctx->outputs[0]->hw_frames_ctx);
    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref);

    if (!ctx->outputs[0]->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif

#if IS_FFMPEG_71_AND_ABOVE
    if (scale->uses_ref && ctx->filter != &ff_vf_scale2ref_ni_quadra) {
        ff_framesync_uninit(&scale->fs);
        ret = ff_framesync_init(&scale->fs, ctx, ctx->nb_inputs);
        if (ret < 0) {
            return ret;
        }

        scale->fs.on_event        = do_scale;
        scale->fs.in[0].time_base = ctx->inputs[0]->time_base;
        scale->fs.in[0].sync      = 1;
        scale->fs.in[0].before    = EXT_STOP;
        scale->fs.in[0].after     = EXT_STOP;

        av_assert0(ctx->nb_inputs == 2);
        scale->fs.in[1].time_base = ctx->inputs[1]->time_base;
        scale->fs.in[1].sync      = 0;
        scale->fs.in[1].before    = EXT_NULL;
        scale->fs.in[1].after     = EXT_INFINITY;

        ret = ff_framesync_configure(&scale->fs);
        if (ret < 0)
            return ret;
    }
#endif

    return 0;

fail:
    return ret;
}

static int config_props_ref(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[1];

    // AVFilterLink *inlink = outlink->src->inputs[0];
    NetIntScaleContext *scale = ctx->priv;
    outlink->w = inlink->w;
    outlink->h = inlink->h;
    outlink->sample_aspect_ratio = inlink->sample_aspect_ratio;
    outlink->time_base = inlink->time_base;
    outlink->format = inlink->format;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    FilterLink *lo = ff_filter_link(outlink);
    lo->frame_rate = li->frame_rate;
    scale->out_frames_ref_1 = av_buffer_ref(li->hw_frames_ctx);
    if (!scale->out_frames_ref_1) {
        return AVERROR(ENOMEM);
    }
    av_buffer_unref(&lo->hw_frames_ctx);
    lo->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref_1);
    if (!lo->hw_frames_ctx) {
        return AVERROR(ENOMEM);
    }
#else
    outlink->frame_rate = inlink->frame_rate;
    scale->out_frames_ref_1 = av_buffer_ref(inlink->hw_frames_ctx);
    if (!scale->out_frames_ref_1) {
        return AVERROR(ENOMEM);
    }
    av_buffer_unref(&outlink->hw_frames_ctx);
    outlink->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref_1);
    if (!outlink->hw_frames_ctx) {
        return AVERROR(ENOMEM);
    }
#endif

    av_log(ctx, AV_LOG_VERBOSE,
        "w:%d h:%d fmt:%s sar:%d/%d -> w:%d h:%d fmt:%s sar:%d/%d\n",
        inlink->w, inlink->h, av_get_pix_fmt_name(inlink->format),
        inlink->sample_aspect_ratio.num, inlink->sample_aspect_ratio.den,
        outlink->w, outlink->h, av_get_pix_fmt_name(outlink->format),
        outlink->sample_aspect_ratio.num, outlink->sample_aspect_ratio.den);

    return 0;
}

static int request_frame(AVFilterLink *outlink)
{
    return ff_request_frame(outlink->src->inputs[0]);
}

static int request_frame_ref(AVFilterLink *outlink)
{
    return ff_request_frame(outlink->src->inputs[1]);
}

/* Process a received frame */
static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    NetIntScaleContext *scale = link->dst->priv;
    AVFilterLink *outlink = link->dst->outputs[0];
    AVFrame *out = NULL;
    niFrameSurface1_t* frame_surface,*new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    ni_retcode_t retcode;
    int scaler_format, cardno;
    uint16_t tempFID;
    uint16_t options;

    frame_surface = (niFrameSurface1_t *) in->data[3];
    if (frame_surface == NULL) {
        return AVERROR(EINVAL);
    }

    pAVHFWCtx = (AVHWFramesContext *) in->hw_frames_ctx->data;
    pAVNIDevCtx       = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno            = ni_get_cardno(in);

    if (scale->skip_filter) {
        //skip hardware scale
        return ff_filter_frame(link->dst->outputs[0], in);
    }

    if (!scale->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        retcode = config_props(outlink, in);
        if (retcode < 0) {
            av_log(link->dst, AV_LOG_ERROR,
                   "ni scale filter session config_props failure\n");
            goto fail;
        }
#endif

        retcode = ni_device_session_context_init(&scale->api_ctx);
        if (retcode < 0) {
            av_log(link->dst, AV_LOG_ERROR,
                   "ni scale filter session context init failure\n");
            goto fail;
        }

        scale->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        scale->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

        scale->api_ctx.hw_id             = cardno;
        scale->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
        scale->api_ctx.scaler_operation  = NI_SCALER_OPCODE_SCALE;
        scale->api_ctx.keep_alive_timeout = scale->keep_alive_timeout;
        scale->api_ctx.isP2P              = scale->is_p2p;

        av_log(link->dst, AV_LOG_INFO,
               "Open scaler session to card %d, hdl %d, blk_hdl %d\n", cardno,
               scale->api_ctx.device_handle, scale->api_ctx.blk_io_handle);

        retcode =
            ni_device_session_open(&scale->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(link->dst, AV_LOG_ERROR,
                   "Can't open device session on card %d\n", cardno);

            /* Close operation will free the device frames */
            ni_device_session_close(&scale->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&scale->api_ctx);
            goto fail;
        }

        scale->session_opened = 1;

        if (scale->autoselect) {
            if (outlink->w <= 540 || outlink->h <= 540)
                scale->params.filterblit = 1;
            else
                scale->params.filterblit = 2;
        }

#if ((LIBXCODER_API_VERSION_MAJOR > 2) ||                                        \
      (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 76))
        if (scale->params.scaler_param_b != 0 || scale->params.scaler_param_c != 0.75) {
            scale->params.enable_scaler_params = true;
        } else {
            scale->params.enable_scaler_params = false;
        }
#endif
        if (scale->params.filterblit) {
            retcode = ni_scaler_set_params(&scale->api_ctx, &(scale->params));
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

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)scale->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(pAVHFWCtx, out_frames_ctx);
        ni_device_session_copy(&scale->api_ctx, &out_ni_ctx->api_ctx);

        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

        if ((in->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(link->dst, AV_LOG_WARNING,
                   "WARNING: Full color range input, limited color range output\n");
        }

        scale->initialized = 1;
    }

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&scale->api_dst_frame.data.frame,
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

    options = 0;
    if (scale->in_color_matrix && strcmp(scale->in_color_matrix,"bt2020") == 0)
        options |= NI_SCALER_FLAG_CS;
    options |= (frame_surface->encoding_type == 2) ? NI_SCALER_FLAG_CMP : 0;

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(&scale->api_ctx,
                                    FFALIGN(in->width, 2),
                                    FFALIGN(in->height, 2),
                                    scaler_format,
                                    options,
                                    0,
                                    0,
                                    0,
                                    0,
                                    0,
                                    frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(link->dst, AV_LOG_DEBUG,
               "Can't assign input frame %d\n", retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(scale->out_format);

    options = NI_SCALER_FLAG_IO;
    if (scale->out_color_matrix && strcmp(scale->out_color_matrix, "bt2020") == 0)
        options |= NI_SCALER_FLAG_CS;
    options |= (scale->output_compressed) ? NI_SCALER_FLAG_CMP : 0;

    /* Allocate hardware device destination frame. This acquires a frame from the pool */
    retcode = ni_device_alloc_frame(&scale->api_ctx,
                          FFALIGN(outlink->w,2),
                          FFALIGN(outlink->h,2),
                          scaler_format,
                          options,
                          0,
                          0,
                          0,
                          0,
                          0,
                          -1,
                          NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(link->dst, AV_LOG_DEBUG,
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

    out->width  = outlink->w;
    out->height = outlink->h;

    out->format = AV_PIX_FMT_NI_QUAD;

#if !IS_FFMPEG_342_AND_ABOVE
    out->sample_aspect_ratio = outlink->sample_aspect_ratio;
#endif

    /* Quadra 2D engine always outputs limited color range */
    out->color_range = AVCOL_RANGE_MPEG;

    /* Reference the new hw frames context */
    out->hw_frames_ctx = av_buffer_ref(scale->out_frames_ref);

    out->data[3] = av_malloc(sizeof(niFrameSurface1_t));

    if (!out->data[3]) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Copy the frame surface from the incoming frame */
    memcpy(out->data[3], in->data[3], sizeof(niFrameSurface1_t));

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&scale->api_ctx, &scale->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(link->dst, AV_LOG_ERROR,
               "Can't acquire output frame %d\n",retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_scale");
#endif

    tempFID = frame_surface->ui16FrameIdx;
    frame_surface = (niFrameSurface1_t *)out->data[3];
    new_frame_surface = (niFrameSurface1_t *)scale->api_dst_frame.data.frame.p_data[3];
    frame_surface->ui16FrameIdx = new_frame_surface->ui16FrameIdx;
    frame_surface->ui16session_ID = new_frame_surface->ui16session_ID;
    frame_surface->device_handle = new_frame_surface->device_handle;
    frame_surface->output_idx     = new_frame_surface->output_idx;
    frame_surface->src_cpu = new_frame_surface->src_cpu;
    frame_surface->dma_buf_fd = 0;

    ff_ni_set_bit_depth_and_encoding_type(&frame_surface->bit_depth,
                                          &frame_surface->encoding_type,
                                          scale->out_format);

    /* Remove ni-split specific assets */
    frame_surface->ui32nodeAddress = 0;

    frame_surface->ui16width  = out->width;
    frame_surface->ui16height = out->height;

    av_log(link->dst, AV_LOG_DEBUG,
           "vf_scale_ni.c:IN trace ui16FrameIdx = [%d] --> out [%d]\n",
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

static int filter_frame_ref(AVFilterLink *link, AVFrame *in)
{
    AVFilterLink *outlink = link->dst->outputs[1];
    return ff_filter_frame(outlink, in);
}

#if IS_FFMPEG_71_AND_ABOVE
static int do_scale(FFFrameSync *fs)
{
    AVFilterContext *ctx = fs->parent;
    NetIntScaleContext *scale = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    AVFrame *in = NULL, *ref = NULL;
    int ret = 0;

    if (ff_inlink_check_available_frame(outlink)) {
        return FFERROR_NOT_READY;
    }

    ret = ff_framesync_get_frame(fs, 0, &in, 1);
    if (ret < 0) {
        goto err;
    }

    if (scale->uses_ref) {
        ret = ff_framesync_get_frame(fs, 1, &ref, 0);
        if (ret < 0) {
            goto err;
        }
    }

    ret = filter_frame(inlink, in);
    return ret;

err:
    if (in) {
      av_frame_free(&in);
    }
    return ret;
}
#endif

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
#if IS_FFMPEG_71_AND_ABOVE
    NetIntScaleContext *scale = ctx->priv;
    if (scale->uses_ref) {
        return ff_framesync_activate(&scale->fs);
    }
#endif

    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    AVFrame *frame = NULL;
    int ret = 0;
    NetIntScaleContext *s = inlink->dst->priv;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    av_log(ctx, AV_LOG_TRACE, "%s: inlink framequeue %lu available_frame %d outlink framequeue %lu frame_wanted %d\n",
        __func__, ff_inlink_queued_frames(inlink), ff_inlink_check_available_frame(inlink), ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));

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

#define OFFSET(x) offsetof(NetIntScaleContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_scale_options[] = {
    { "w",      "Output video width",  OFFSET(w_expr),   AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "width",  "Output video width",  OFFSET(w_expr),   AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "h",      "Output video height", OFFSET(h_expr),   AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "height", "Output video height", OFFSET(h_expr),   AV_OPT_TYPE_STRING, .flags = FLAGS },
    { "size",   "set video size",      OFFSET(size_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, FLAGS },
    { "s",      "set video size",      OFFSET(size_str), AV_OPT_TYPE_STRING, {.str = NULL}, 0, FLAGS },
    { "in_color_matrix",  "set input YCbCr type",   OFFSET(in_color_matrix),  AV_OPT_TYPE_STRING, {.str = NULL}, .flags = FLAGS, "color" },
    { "out_color_matrix", "set output YCbCr type",  OFFSET(out_color_matrix), AV_OPT_TYPE_STRING, {.str = NULL}, .flags = FLAGS, "color"},
        { "bt709",       NULL, 0, AV_OPT_TYPE_CONST, {.str = "bt709"},     0, 0, FLAGS, "color" },
        { "bt2020",      NULL, 0, AV_OPT_TYPE_CONST, {.str = "bt2020"},    0, 0, FLAGS, "color" },
    { "force_original_aspect_ratio", "decrease or increase w/h if necessary to keep the original AR", OFFSET(force_original_aspect_ratio), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 2, FLAGS, "force_oar" },
    { "format", "set_output_format", OFFSET(format), AV_OPT_TYPE_INT, {.i64=OUTPUT_FORMAT_AUTO}, 0, OUTPUT_FORMAT_NB-1, FLAGS, "format" },
        { "yuv420p",     "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_YUV420P},     .flags = FLAGS, .unit = "format" },
        { "yuyv422",     "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_YUYV422},     .flags = FLAGS, .unit = "format" },
        { "uyvy422",     "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_UYVY422},     .flags = FLAGS, .unit = "format" },
        { "nv12",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_NV12},        .flags = FLAGS, .unit = "format" },
        { "argb",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_ARGB},        .flags = FLAGS, .unit = "format" },
        { "rgba",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_RGBA},        .flags = FLAGS, .unit = "format" },
        { "abgr",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_ABGR},        .flags = FLAGS, .unit = "format" },
        { "bgra",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_BGRA},        .flags = FLAGS, .unit = "format" },
        { "yuv420p10le", "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_YUV420P10LE}, .flags = FLAGS, .unit = "format" },
        { "nv16",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_NV16},        .flags = FLAGS, .unit = "format" },
        { "bgr0",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_BGR0},        .flags = FLAGS, .unit = "format" },
        { "p010le",      "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_P010LE},      .flags = FLAGS, .unit = "format" },
        { "bgrp",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_BGRP},        .flags = FLAGS, .unit = "format" },
        { "auto",        "", 0, AV_OPT_TYPE_CONST, {.i64=OUTPUT_FORMAT_AUTO},        .flags = FLAGS, .unit = "format"},
    { "disable",  NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 0}, 0, 0, FLAGS, "force_oar" },
    { "decrease", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 1}, 0, 0, FLAGS, "force_oar" },
    { "increase", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 2}, 0, 0, FLAGS, "force_oar" },
    { "force_divisible_by", "enforce that the output resolution is divisible by a defined integer when force_original_aspect_ratio is used", OFFSET(force_divisible_by), AV_OPT_TYPE_INT, {.i64 = 1}, 1, 256, FLAGS },
    { "filterblit", "filterblit enable", OFFSET(params.filterblit), AV_OPT_TYPE_INT, {.i64=0}, 0, 4, FLAGS },
#if ((LIBXCODER_API_VERSION_MAJOR > 2) || (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 76))
    { "param_b", "Parameter B for bicubic", OFFSET(params.scaler_param_b), AV_OPT_TYPE_DOUBLE, {.dbl=0.0},  0, 1, FLAGS },
    { "param_c", "Parameter C for bicubic", OFFSET(params.scaler_param_c), AV_OPT_TYPE_DOUBLE, {.dbl=0.75}, 0, 1, FLAGS },
#endif
    { "autoselect", "auto select filterblit mode according to resolution", OFFSET(autoselect), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    NI_FILT_OPTION_AUTO_SKIP,
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_scale);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_props,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

AVFilter ff_vf_scale_ni_quadra = {
    .name           = "ni_quadra_scale",
    .description    = NULL_IF_CONFIG_SMALL(
        "NETINT Quadra video scaler v" NI_XCODER_REVISION),
#if (LIBAVFILTER_VERSION_MAJOR > 9 || (LIBAVFILTER_VERSION_MAJOR == 9 && LIBAVFILTER_VERSION_MINOR >= 3))
    .init           = init,
#else
    .init_dict      = init_dict,
#endif
    .uninit         = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate       = activate,
#endif
    .priv_size      = sizeof(NetIntScaleContext),
    .priv_class     = &ni_scale_class,
// only FFmpeg 3.4.2 and above have .flags_internal
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
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

#define ni_scale2ref_options ni_scale_options
AVFILTER_DEFINE_CLASS(ni_scale2ref);

static const AVFilterPad scale2ref_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
    {
        .name         = "ref",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame_ref,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

static const AVFilterPad scale2ref_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_props,
        .request_frame= request_frame,
    },
    {
        .name         = "ref",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_props_ref,
        .request_frame= request_frame_ref,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

AVFilter ff_vf_scale2ref_ni_quadra = {
    .name            = "ni_quadra_scale2ref",
    .description     = NULL_IF_CONFIG_SMALL("Scale the input video size and/or convert the image format to the given reference."),
#if (LIBAVFILTER_VERSION_MAJOR > 9 || (LIBAVFILTER_VERSION_MAJOR == 9 && LIBAVFILTER_VERSION_MINOR >= 3))
    .init            = init,
#else
    .init_dict       = init_dict,
#endif
    .uninit          = uninit,
    .priv_size       = sizeof(NetIntScaleContext),
    .priv_class      = &ni_scale2ref_class,
    // only FFmpeg 3.4.2 and above have .flags_internal
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(scale2ref_inputs),
    FILTER_OUTPUTS(scale2ref_outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs          = scale2ref_inputs,
    .outputs         = scale2ref_outputs,
    .query_formats   = query_formats,
#endif
};
