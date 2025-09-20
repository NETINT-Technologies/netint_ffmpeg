/*
 * Copyright (c) 2007 Bobby Bingham
 * Copyright (c) 2021 NetInt
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
 * audio and video splitter
 */

#include <stdio.h>

#include "libavutil/attributes.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "nifilter.h"
#include "version.h"
#include <ni_device_api.h>
#if IS_FFMPEG_70_AND_ABOVE
#include "avfilter_internal.h"
#endif
#define FF_INTERNAL_FIELDS 1

#if IS_FFMPEG_342_AND_ABOVE
#include "framequeue.h"
#endif

#include "avfilter.h"
#include "audio.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "filters.h"
#endif
#include "video.h"

typedef struct NetIntSplitContext {
    const AVClass *class;
    bool initialized;
    int nb_output0;
    int nb_output1;
    int nb_output2;
    int total_outputs;
    int frame_contexts_applied;
    ni_split_context_t src_ctx;
    AVBufferRef *out_frames_ref[3];
} NetIntSplitContext;

#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *link);
#endif

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat input_pix_fmts[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUV420P10LE,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_P010LE,
        AV_PIX_FMT_NI_QUAD,
        AV_PIX_FMT_NONE,
    };
    static const enum AVPixelFormat output_pix_fmts[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUV420P10LE,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_P010LE,
        AV_PIX_FMT_NI_QUAD,
        AV_PIX_FMT_NONE,
    };
    AVFilterFormats *in_fmts = ff_make_format_list(input_pix_fmts);
    AVFilterFormats *out_fmts = ff_make_format_list(output_pix_fmts);
    int ret;

    // Needed for FFmpeg-n4.4+
#if (LIBAVFILTER_VERSION_MAJOR >= 8 || LIBAVFILTER_VERSION_MAJOR >= 7 && LIBAVFILTER_VERSION_MINOR >= 110)
    if ((ret = ff_formats_ref(in_fmts, &ctx->inputs[0]->outcfg.formats)) < 0)
        return ret;
    if ((ret = ff_formats_ref(out_fmts, &ctx->outputs[0]->incfg.formats)) < 0)
        return ret;
#else
    if ((ret = ff_formats_ref(in_fmts, &ctx->inputs[0]->out_formats)) < 0)
        return ret;
    if ((ret = ff_formats_ref(out_fmts, &ctx->outputs[0]->in_formats)) < 0)
        return ret;
#endif

    return 0;
}

static av_cold int split_init(AVFilterContext *ctx)
{
    NetIntSplitContext *s = ctx->priv;
    int i, ret;

    av_log(ctx, AV_LOG_DEBUG, "ni_quadra_split INIT out0,1,2 = %d %d %d ctx->nb_outputs = %d\n",
        s->nb_output0, s->nb_output1, s->nb_output2,
        ctx->nb_outputs);
    if (s->nb_output2 && s->nb_output1 == 0) {
        //swap them for reorder to use out1 first
        s->nb_output1 = s->nb_output2;
        s->nb_output2 = 0;
        av_log(ctx, AV_LOG_DEBUG, "ni_quadra_split INIT out2 moved to out1\n");
    }

    s->total_outputs = s->nb_output0 + s->nb_output1 + s->nb_output2;
    //ctx->nb_outputs = s->total_outputs;
    for (i = 0; i < s->total_outputs; i++) {
        char name[32];
        AVFilterPad pad = { 0 };

        snprintf(name, sizeof(name), "output%d", i);
        pad.type = ctx->filter->inputs[0].type;
        pad.name = av_strdup(name);
        if (!pad.name) {
            return AVERROR(ENOMEM);
        }
#if IS_FFMPEG_342_AND_ABOVE
        pad.config_props = config_output;
#endif

        if (!pad.name)
            return AVERROR(ENOMEM);

#if (LIBAVFILTER_VERSION_MAJOR >= 8)
        if ((ret = ff_append_outpad_free_name(ctx, &pad)) < 0) {
#else
        if ((ret = ff_insert_outpad(ctx, i, &pad)) < 0) {
#endif
            av_freep(&pad.name);
            return ret;
        }
    }

    return 0;
}

static av_cold void split_uninit(AVFilterContext *ctx)
{
    int i;
    NetIntSplitContext *s = ctx->priv;
    for (i = 0; i < ctx->nb_outputs; i++)
        av_freep(&ctx->output_pads[i].name);

    for (i = 0; i < 3; i++) {
        if (s->out_frames_ref[i])
            av_buffer_unref(&s->out_frames_ref[i]);
    }
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_input(AVFilterLink *inlink)
#else
static int config_input(AVFilterLink *inlink, AVFrame *in)
#endif
{
    AVFilterContext *avctx = inlink->dst;
    NetIntSplitContext *s        = avctx->priv;
    AVHWFramesContext *ctx;
    ni_split_context_t *p_split_ctx_dst = &s->src_ctx;
    AVNIFramesContext *src_ctx;
    ni_split_context_t *p_split_ctx_src;
    int i;
    s->frame_contexts_applied = -1;
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);

    if (li->hw_frames_ctx == NULL)
#elif IS_FFMPEG_342_AND_ABOVE
    if (inlink->hw_frames_ctx == NULL)
#else
    if (in->hw_frames_ctx == NULL)
#endif
    {
        for (i = 0; i < 3; i++) {
            s->src_ctx.w[i]   = inlink->w;
            s->src_ctx.h[i]   = inlink->h;
            s->src_ctx.f[i]   = -1;
            s->src_ctx.f8b[i] = -1;
        }
    } else {
#if IS_FFMPEG_71_AND_ABOVE
        ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
        ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
#else
        ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif
        src_ctx         = (AVNIFramesContext*) ctx->hwctx;
        p_split_ctx_src = &src_ctx->split_ctx;
        memcpy(p_split_ctx_dst, p_split_ctx_src, sizeof(ni_split_context_t));
        for (i = 0; i < 3; i++) {
            s->frame_contexts_applied = 0;
            av_log(avctx, AV_LOG_DEBUG, "[%d] %d x %d  f8b %d\n", i,
                   p_split_ctx_dst->w[i], p_split_ctx_dst->h[i],
                   p_split_ctx_dst->f8b[i]);
            // p_split_ctx_dst->crop_meta_data_rltb[i][0],
            // p_split_ctx_dst->crop_meta_data_rltb[i][1],
            // p_split_ctx_dst->crop_meta_data_rltb[i][2],
            // p_split_ctx_dst->crop_meta_data_rltb[i][3]);
        }
        if (p_split_ctx_dst->enabled == 0) {
            for (i = 0; i < 3; i++) {
                s->src_ctx.w[i]   = inlink->w;
                s->src_ctx.h[i]   = inlink->h;
                s->src_ctx.f[i]   = -1;
                s->src_ctx.f8b[i] = -1;
            }
        }
    }

    return 0;
}

#if IS_FFMPEG_342_AND_ABOVE
static int init_out_hwctxs(AVFilterContext *ctx)
#else
static int init_out_hwctxs(AVFilterContext *ctx, AVFrame *in)
#endif
{
    NetIntSplitContext *s = ctx->priv;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx[3];

    enum AVPixelFormat out_format;
    int i, j;

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
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif

    if (s->src_ctx.enabled == 1) {
        for (i = 0; i < 3; i++) {
            s->out_frames_ref[i] =
                av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
            if (!s->out_frames_ref[i])
                return AVERROR(ENOMEM);
            out_frames_ctx[i] = (AVHWFramesContext *)s->out_frames_ref[i]->data;

            out_frames_ctx[i]->format = AV_PIX_FMT_NI_QUAD;
            out_frames_ctx[i]->width  = s->src_ctx.w[i];
            out_frames_ctx[i]->height = s->src_ctx.h[i];

            if (s->src_ctx.f[i] == -1) {
                return AVERROR(EINVAL);
            }

            switch (s->src_ctx.f[i]) {
                case NI_PIXEL_PLANAR_FORMAT_PLANAR: // yuv420p or p10
                    out_format = (s->src_ctx.f8b[i] == 1) ? AV_PIX_FMT_YUV420P
                                                          : AV_PIX_FMT_YUV420P10LE;
                    break;
                case NI_PIXEL_PLANAR_FORMAT_TILED4X4: // tiled
                    out_format = (s->src_ctx.f8b[i] == 1)
                                     ? AV_PIX_FMT_NI_QUAD_8_TILE_4X4
                                     : AV_PIX_FMT_NI_QUAD_10_TILE_4X4;
                    break;
                case NI_PIXEL_PLANAR_FORMAT_SEMIPLANAR: // NV12
                    out_format = (s->src_ctx.f8b[i] == 1) ? AV_PIX_FMT_NV12
                                                          : AV_PIX_FMT_P010LE;
                    break;
                default:
                    av_log(ctx, AV_LOG_ERROR, "PPU%d invalid pixel format %d in hwframe ctx\n", i, s->src_ctx.f[i]);
                    return AVERROR(EINVAL);
            }
            out_frames_ctx[i]->sw_format = out_format;
            out_frames_ctx[i]->initial_pool_size =
                -1; // already has its own pool

            /* Don't check return code, this will intentionally fail */
            av_hwframe_ctx_init(s->out_frames_ref[i]);

            ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx[i]);
            ((AVNIFramesContext *) out_frames_ctx[i]->hwctx)->split_ctx.enabled = 0;
        }

        for (i = 0; i < ctx->nb_outputs; i++) {
#if IS_FFMPEG_71_AND_ABOVE
            FilterLink *lo = ff_filter_link(ctx->outputs[i]);
            av_buffer_unref(&lo->hw_frames_ctx);
            if (i < s->nb_output0) {
                j = 0;
            } else if (i < s->nb_output0 + s->nb_output1) {
                j = 1;
            } else {
                j = 2;
            }
            lo->hw_frames_ctx =
                av_buffer_ref(s->out_frames_ref[j]);

            av_log(ctx, AV_LOG_DEBUG, "NI:%s:out\n",
                   (s->src_ctx.f[j] == 0)
                       ? "semiplanar"
                       : (s->src_ctx.f[j] == 2) ? "tiled" : "planar");
            if (!lo->hw_frames_ctx)
                return AVERROR(ENOMEM);

            av_log(ctx, AV_LOG_DEBUG,
                   "ni_split superframe config_output_hwctx[%d] %p\n", i,
                   lo->hw_frames_ctx);
#else
            av_buffer_unref(&ctx->outputs[i]->hw_frames_ctx);
            if (i < s->nb_output0) {
                j = 0;
            } else if (i < s->nb_output0 + s->nb_output1) {
                j = 1;
            } else {
                j = 2;
            }
            ctx->outputs[i]->hw_frames_ctx =
                av_buffer_ref(s->out_frames_ref[j]);

            av_log(ctx, AV_LOG_DEBUG, "NI:%s:out\n",
                   (s->src_ctx.f[j] == 0)
                       ? "semiplanar"
                       : (s->src_ctx.f[j] == 2) ? "tiled" : "planar");
            if (!ctx->outputs[i]->hw_frames_ctx)
                return AVERROR(ENOMEM);

            av_log(ctx, AV_LOG_DEBUG,
                   "ni_split superframe config_output_hwctx[%d] %p\n", i,
                   ctx->outputs[i]->hw_frames_ctx);
#endif
        }
    } else { // no possibility of using extra outputs
        for (i = 0; i < ctx->nb_outputs; i++) {
#if IS_FFMPEG_71_AND_ABOVE
            FilterLink *lo = ff_filter_link(ctx->outputs[i]);
            av_buffer_unref(&lo->hw_frames_ctx);
            if (i < s->nb_output0) {
                lo->hw_frames_ctx =
                    av_buffer_ref(li->hw_frames_ctx);
            }
            if (!lo->hw_frames_ctx)
                return AVERROR(ENOMEM);

            av_log(ctx, AV_LOG_DEBUG, "ni_split config_output_hwctx[%d] %p\n",
                   i, lo->hw_frames_ctx);
#else
            av_buffer_unref(&ctx->outputs[i]->hw_frames_ctx);
            if (i < s->nb_output0) {
                ctx->outputs[i]->hw_frames_ctx =
                    av_buffer_ref(ctx->inputs[0]->hw_frames_ctx);
            }
            if (!ctx->outputs[i]->hw_frames_ctx)
                return AVERROR(ENOMEM);

            av_log(ctx, AV_LOG_DEBUG, "ni_split config_output_hwctx[%d] %p\n",
                   i, ctx->outputs[i]->hw_frames_ctx);
#endif
        }
        av_log(ctx, AV_LOG_DEBUG,
               "ni_split config_output_hwctx set direct to output\n");
    }
    return 0;
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *link)
#else
static int config_output(AVFilterLink *link, AVFrame *in)
#endif
{
    // config output sets all outputs at a time since there's no
    // easy way to track the target output based on inlink.
    // fairly trivial assignments here so no performance worries
    AVFilterContext *ctx = link->src;
    NetIntSplitContext *s = ctx->priv;
    int i, ret;

    for (i = 0; i < ctx->nb_outputs; i++) {
        if (i < s->nb_output0) {
            ctx->outputs[i]->w = s->src_ctx.w[0];
            ctx->outputs[i]->h = s->src_ctx.h[0];
        } else if (i < s->nb_output0 + s->nb_output1) {
            ctx->outputs[i]->w = s->src_ctx.w[1];
            ctx->outputs[i]->h = s->src_ctx.h[1];
        } else {
            ctx->outputs[i]->w = s->src_ctx.w[2];
            ctx->outputs[i]->h = s->src_ctx.h[2];
        }
        av_log(ctx, AV_LOG_DEBUG,
               "ni_split config_output[%d] w x h = %d x %d\n", i,
               ctx->outputs[i]->w, ctx->outputs[i]->h);
    }
    if (s->frame_contexts_applied == 0) {
        s->frame_contexts_applied = 1; // run once per set ni_split, not per output
#if IS_FFMPEG_342_AND_ABOVE
        ret = init_out_hwctxs(ctx);
#else
        ret = init_out_hwctxs(ctx, in);
#endif
        if (ret < 0)
            return ret;
    }
    return 0;
}

static int filter_ni_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    NetIntSplitContext *s = ctx->priv;
    int i, ret = AVERROR_EOF;
    int output_index;
    niFrameSurface1_t *p_data3;

    if (!s->initialized) {
        for (i = 0; i < 3; i++) {
            AVHWFramesContext *in_frames_ctx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
            AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref[i]->data;
            ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx);
            AVNIFramesContext *ni_frames_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
            ni_frames_ctx->split_ctx.enabled = 0;
        }
        s->initialized = 1;
    }

    for (i = 0; i < ctx->nb_outputs; i++) {
        AVFrame *buf_out;
#if IS_FFMPEG_70_AND_ABOVE
        FilterLinkInternal* const li = ff_link_internal(ctx->outputs[i]);
        if (li->status_in)
#elif IS_FFMPEG_342_AND_ABOVE
        if (ctx->outputs[i]->status_in)
#else
        if (ctx->outputs[i]->status)
#endif
            continue;

        buf_out = av_frame_alloc();
        if (!buf_out) {
            ret = AVERROR(ENOMEM);
            break;
        }
        av_frame_copy_props(buf_out, frame);

        buf_out->format = frame->format;

        if (i < s->nb_output0) {
            output_index = 0;
        } else if (i < s->nb_output0 + s->nb_output1) {
            if (!frame->buf[1]) {
                ret = AVERROR(ENOMEM);
                av_frame_free(&buf_out);
                break;
            }
            output_index = 1;
        } else {
            if (!frame->buf[2]) {
                ret = AVERROR(ENOMEM);
                av_frame_free(&buf_out);
                break;
            }
            output_index = 2;
        }
        buf_out->buf[0]        = av_buffer_ref(frame->buf[output_index]);
        buf_out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref[output_index]);
        buf_out->data[3] = buf_out->buf[0]->data;
        p_data3 = (niFrameSurface1_t*)((uint8_t*)buf_out->data[3]);

        buf_out->width = ctx->outputs[i]->w = p_data3->ui16width;
        buf_out->height = ctx->outputs[i]->h = p_data3->ui16height;

        av_log(ctx, AV_LOG_DEBUG, "output %d supplied WxH = %d x %d FID %d offset %d\n",
               i, buf_out->width, buf_out->height,
               p_data3->ui16FrameIdx, p_data3->ui32nodeAddress);

        ret = ff_filter_frame(ctx->outputs[i], buf_out);
        if (ret < 0)
            break;
    }
    return ret;
}

static int filter_std_frame(AVFilterLink *inlink, AVFrame *frame)
{//basically clone of native split
    AVFilterContext *ctx = inlink->dst;
    NetIntSplitContext *s = ctx->priv;
    int i, ret = AVERROR_EOF;
    if (s->nb_output0 < 2) {
        av_log(ctx, AV_LOG_ERROR, "ni_split must have at least 2 outputs for Standard split!\n");
        ret = AVERROR(EINVAL);
        return ret;
    }
    if (s->nb_output1) {
        av_log(ctx, AV_LOG_ERROR, "ni_split output1 or output2 param must not be used for Standard splitting!\n");
        ret = AVERROR(E2BIG);
        return ret;
    }

    for (i = 0; i < ctx->nb_outputs; i++) {
        AVFrame *buf_out;
#if IS_FFMPEG_70_AND_ABOVE
        FilterLinkInternal* const li = ff_link_internal(ctx->outputs[i]);
        if (li->status_in)
#elif IS_FFMPEG_342_AND_ABOVE
        if (ctx->outputs[i]->status_in)
#else
        if (ctx->outputs[i]->status)
#endif
            continue;
        buf_out = av_frame_clone(frame);
        if (!buf_out) {
            ret = AVERROR(ENOMEM);
            break;
        }

        ret = ff_filter_frame(ctx->outputs[i], buf_out);
        if (ret < 0)
            break;
    }
    return ret;
}

#if IS_FFMPEG_71_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    NetIntSplitContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFrame *frame;
    int status, ret, nb_eofs = 0;
    int64_t pts;

    for (int i = 0; i < ctx->nb_outputs; i++) {
        nb_eofs += ff_outlink_get_status(ctx->outputs[i]) == AVERROR_EOF;
    }

    if (nb_eofs == ctx->nb_outputs) {
        ff_inlink_set_status(inlink, AVERROR_EOF);
        return 0;
    }

    ret = ff_inlink_consume_frame(inlink, &frame);
    if (ret < 0) {
        return ret;
    }
    if (ret > 0) {
        av_log(ctx, AV_LOG_TRACE, "out0,1,2 = %d %d %d total = %d\n",
               s->nb_output0, s->nb_output1, s->nb_output2,
               ctx->nb_outputs);

        av_log(ctx, AV_LOG_DEBUG, "ni_split: filter_frame, in format=%d, Sctx %d\n",
               frame->format,
               s->src_ctx.enabled);

        if (frame->format == AV_PIX_FMT_NI_QUAD && s->src_ctx.enabled == 1)
        {
            ret = filter_ni_frame(inlink, frame);
        }
        else
        {
            ret = filter_std_frame(inlink, frame);
        }

        av_frame_free(&frame);
        if (ret < 0) {
            return ret;
        } else {
            ff_filter_set_ready(ctx, 100);
        }
    }

    if (ff_inlink_acknowledge_status(inlink, &status, &pts)) {
        for (int i = 0; i < ctx->nb_outputs; i++) {
            if (ff_outlink_get_status(ctx->outputs[i])) {
                continue;
            }
            ff_outlink_set_status(ctx->outputs[i], status, pts);
        }
        return 0;
    }

    for (int i = 0; i < ctx->nb_outputs; i++) {
        if (ff_outlink_get_status(ctx->outputs[i])) {
            continue;
        }

        if (ff_outlink_frame_wanted(ctx->outputs[i])) {
            ff_inlink_request_frame(inlink);
            return 0;
        }
    }

    return FFERROR_NOT_READY;
}
#else
static int filter_frame(AVFilterLink *inlink, AVFrame *frame) //separate hw and sw into different function calls
{
    AVFilterContext *ctx = inlink->dst;
    NetIntSplitContext *s = ctx->priv;
    int ret              = AVERROR_EOF;
#if !IS_FFMPEG_342_AND_ABOVE
    AVFilterLink *outlink;
    if (!s->initialized) {
        ret = config_input(inlink, frame);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "ni_split config_input failed.\n");
            av_frame_free(&frame);
            return ret;
        }

        outlink = inlink->dst->outputs[0];
        ret     = config_output(outlink, frame);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "ni_split config_output failed.\n");
            av_frame_free(&frame);
            return ret;
        }
        s->initialized = true;
    }
#endif

    av_log(ctx, AV_LOG_TRACE, "out0,1,2 = %d %d %d total = %d\n",
           s->nb_output0, s->nb_output1, s->nb_output2,
           ctx->nb_outputs);

    av_log(ctx, AV_LOG_DEBUG, "ni_split: filter_frame, in format=%d, Sctx %d\n",
           frame->format,
           s->src_ctx.enabled);

    if (frame->format == AV_PIX_FMT_NI_QUAD && s->src_ctx.enabled == 1) {
        ret = filter_ni_frame(inlink, frame);
    }
    else {
        ret = filter_std_frame(inlink, frame);
    }
    av_frame_free(&frame);
    return ret;
}
#endif

#define OFFSET(x) offsetof(NetIntSplitContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_split_options[] = {
    { "output0", "Copies of output0", OFFSET(nb_output0), AV_OPT_TYPE_INT, {.i64 = 2}, 0, INT_MAX, FLAGS },
    { "output1", "Copies of output1", OFFSET(nb_output1), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS },
    { "output2", "Copies of output2", OFFSET(nb_output2), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_split);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_input,
#endif
#if !IS_FFMPEG_71_AND_ABOVE
        .filter_frame = filter_frame,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

AVFilter ff_vf_split_ni_quadra = {
    .name        = "ni_quadra_split",
    .description = NULL_IF_CONFIG_SMALL(
        "NETINT Quadra demux input from decoder post-processor unit (PPU) to N video outputs v" NI_XCODER_REVISION),
    .priv_size  = sizeof(NetIntSplitContext),
    .priv_class = &ni_split_class,
    .init       = split_init,
    .uninit     = split_uninit,
    .flags      = AVFILTER_FLAG_DYNAMIC_OUTPUTS,
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
#endif
#if IS_FFMPEG_71_AND_ABOVE
    .activate    = activate,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs         = inputs,
    .query_formats  = query_formats,
#endif
};
