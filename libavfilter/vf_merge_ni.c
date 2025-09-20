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
 * merge one video y and the other uv to a new video
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
#include "libavutil/opt.h"
#include "libavutil/hwcontext.h"
#include <ni_device_api.h>
#include "libavutil/avstring.h"

typedef struct NetIntMergeContext {
    const AVClass *class;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;

    AVBufferRef* out_frames_ref;

    int initialized;
    int session_opened;
    int keep_alive_timeout; /* keep alive timeout setting */
    int buffer_limit;
    ni_scaler_params_t params;
    ni_split_context_t src_ctx;
} NetIntMergeContext;

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

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntMergeContext *s = ctx->priv;

    if (s->api_dst_frame.data.frame.p_buffer) {
        ni_frame_buffer_free(&s->api_dst_frame.data.frame);
    }

    if (s->session_opened) {
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
    }

    av_buffer_unref(&s->out_frames_ref);
}


#if IS_FFMPEG_342_AND_ABOVE
static int config_input(AVFilterLink *inlink)
#else
static int config_input_0(AVFilterLink *inlink, AVFrame *in)
#endif
{
    NetIntMergeContext *s = inlink->dst->priv;
    AVHWFramesContext *in_frames_ctx;
    AVNIFramesContext *src_ctx;
    ni_split_context_t *p_split_ctx_src;

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

    src_ctx = (AVNIFramesContext*) in_frames_ctx->hwctx;
    p_split_ctx_src = &src_ctx->split_ctx;
    if (!p_split_ctx_src->enabled) {
        av_log(inlink->dst, AV_LOG_ERROR, "There is no extra ppu output\n");
        return AVERROR(EINVAL);
    }
    memcpy(&s->src_ctx, p_split_ctx_src, sizeof(ni_split_context_t));

    if (in_frames_ctx->sw_format != AV_PIX_FMT_YUV420P &&
        in_frames_ctx->sw_format != AV_PIX_FMT_NV12 &&
        in_frames_ctx->sw_format != AV_PIX_FMT_YUV420P10LE &&
        in_frames_ctx->sw_format != AV_PIX_FMT_P010LE) {
        av_log(inlink->dst, AV_LOG_ERROR,
               "merge filter does not support this format: %s\n", av_get_pix_fmt_name(in_frames_ctx->sw_format));
        return AVERROR(EINVAL);
    }
    if ((s->src_ctx.f[0] != s->src_ctx.f[1]) || (s->src_ctx.f[0] != s->src_ctx.f[1])) {
        av_log(inlink->dst, AV_LOG_ERROR, "The PPU0 and PPU1 must have the same format\n");
        return AVERROR(EINVAL);
    }

    return 0;
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntMergeContext *s = ctx->priv;
    AVHWFramesContext *out_frames_ctx;
    int pool_size = 1;

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
    return 0;
}

static int config_output(AVFilterLink *outlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = outlink->src;
    NetIntMergeContext *s = ctx->priv;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    int ret = 0;

    outlink->w = s->src_ctx.w[1];
    outlink->h = s->src_ctx.h[1];
    outlink->sample_aspect_ratio = outlink->src->inputs[0]->sample_aspect_ratio;

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
    if (s->src_ctx.h[0] == s->src_ctx.h[1] && s->src_ctx.w[0] == s->src_ctx.w[1]) {
#if IS_FFMPEG_71_AND_ABOVE
        s->out_frames_ref = av_buffer_ref(li->hw_frames_ctx);
#else
        s->out_frames_ref = av_buffer_ref(ctx->inputs[0]->hw_frames_ctx);
#endif
    }
    else {
        s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
        if (!s->out_frames_ref)
            return AVERROR(ENOMEM);

        out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        out_frames_ctx->format    = AV_PIX_FMT_NI_QUAD;
        out_frames_ctx->width     = outlink->w;
        out_frames_ctx->height    = outlink->h;
        out_frames_ctx->sw_format = in_frames_ctx->sw_format;
        out_frames_ctx->initial_pool_size =
            NI_MERGE_ID; // Repurposed as identity code

        av_hwframe_ctx_init(s->out_frames_ref);

    }

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

    return ret;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext      *ctx = inlink->dst;
    NetIntMergeContext *s = (NetIntMergeContext *) ctx->priv;
    AVHWFramesContext    *frame_ctx;
    AVNIDeviceContext    *pAVNIDevCtx;
    AVFilterLink         *outlink;
    AVFrame              *out = NULL;
    niFrameSurface1_t    *frame_0_surface, *frame_1_surface, *new_frame_surface;
    int flags, frame_cardno;
    int frame_scaler_format;
    ni_retcode_t retcode;

    frame_ctx = (AVHWFramesContext *) frame->hw_frames_ctx->data;
    frame_scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(frame_ctx->sw_format);
    outlink = ctx->outputs[0];

    frame_cardno = ni_get_cardno(frame);
    frame_0_surface = ((niFrameSurface1_t*)(frame->buf[0]->data));
    frame_1_surface = ((niFrameSurface1_t*)(frame->buf[1]->data));
    if (s->src_ctx.h[0] == s->src_ctx.h[1] && s->src_ctx.w[0] == s->src_ctx.w[1]) {
        av_buffer_unref(&frame->buf[1]);
        return ff_filter_frame(ctx->outputs[0], frame);
    }

    if (!s->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        retcode = config_output(outlink, frame_0);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni merge filter config output failure\n");
            goto fail;
        }
#endif

        retcode = ni_device_session_context_init(&s->api_ctx);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni merge filter session context init failure\n");
            goto fail;
        }

        pAVNIDevCtx = (AVNIDeviceContext *)frame_ctx->device_ctx->hwctx;
        s->api_ctx.device_handle = pAVNIDevCtx->cards[frame_cardno];
        s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[frame_cardno];

        s->api_ctx.hw_id              = frame_cardno;
        s->api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
        s->api_ctx.scaler_operation   = NI_SCALER_OPCODE_MERGE;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
        s->api_ctx.isP2P              = 0;

        av_log(ctx, AV_LOG_INFO,
                       "Open merge session to card %d, hdl %d, blk_hdl %d\n", frame_cardno,
                       s->api_ctx.device_handle, s->api_ctx.blk_io_handle);

        retcode = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't open device session on card %d\n",
                   frame_cardno);
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->api_ctx);
            goto fail;
        }

        s->session_opened = 1;
#if ((LIBXCODER_API_VERSION_MAJOR > 2) ||                                        \
      (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 76))
        if (s->params.scaler_param_b != 0 || s->params.scaler_param_c != 0.75) {
            s->params.enable_scaler_params = true;
        }
        else {
            s->params.enable_scaler_params = false;
        }
#endif
        if (s->params.filterblit) {
            retcode = ni_scaler_set_params(&s->api_ctx, &(s->params));
            if (retcode < 0) {
                av_log(ctx, AV_LOG_ERROR,
                   "Set params error %d\n", retcode);
                goto fail;
            }
        }

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           inlink->dst->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        retcode = init_out_pool(inlink->dst);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            goto fail;
        }

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(frame_ctx, out_frames_ctx);
        ni_device_session_copy(&s->api_ctx, &out_ni_ctx->api_ctx);
        ((AVNIFramesContext *) out_frames_ctx->hwctx)->split_ctx.enabled = 0;

        AVHWFramesContext *pAVHFWCtx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

        if ((frame->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(ctx, AV_LOG_WARNING,
                   "WARNING: Full color range input, limited color range output\n");
        }

        s->initialized = 1;
    }

    /* Allocate a ni_frame for the merge output */
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
     * Assign an input frame for merge picture. Send the
     * incoming hardware frame index to the scaler manager.
     */
    retcode = ni_device_alloc_frame(
        &s->api_ctx,
        FFALIGN(frame->width, 2),
        FFALIGN(frame->height, 2),
        frame_scaler_format,
        (frame_0_surface && frame_1_surface->encoding_type == 2) ? NI_SCALER_FLAG_CMP : 0,
        FFALIGN(frame->width, 2),
        FFALIGN(frame->height, 2),
        0,
        0,
        frame_0_surface->ui32nodeAddress,
        frame_0_surface->ui16FrameIdx,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't assign frame for merge input %d\n",
               retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /*
     * Allocate device output frame from the pool. We also send down the frame index
     * of the background frame to the scaler manager.
     */
    flags =  NI_SCALER_FLAG_IO;
    flags |= (frame_1_surface && frame_1_surface->encoding_type == 2) ? NI_SCALER_FLAG_CMP : 0;
    retcode = ni_device_alloc_frame(&s->api_ctx,
                                    FFALIGN(outlink->w, 2),
                                    FFALIGN(outlink->h, 2),
                                    frame_scaler_format,
                                    flags,
                                    FFALIGN(outlink->w, 2),
                                    FFALIGN(outlink->h, 2),
                                    0,                              // x
                                    0,                              // y
                                    frame_1_surface->ui32nodeAddress,
                                    frame_1_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate frame for output %d\n",
               retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR,
               "Can't acquire output frame %d\n", retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_merge");
#endif

    out = av_frame_alloc();

    if (!out) {
        av_log(ctx, AV_LOG_ERROR, "Cannot clone frame\n");
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    av_frame_copy_props(out,frame);
    out->width  = outlink->w;
    out->height = outlink->h;
    out->format = AV_PIX_FMT_NI_QUAD;
#if !IS_FFMPEG_342_AND_ABOVE
        out->sample_aspect_ratio = outlink->sample_aspect_ratio;
#endif

    out->buf[0]        = av_buffer_ref(frame->buf[1]);
    out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    out->data[3] = out->buf[0]->data;

    frame_1_surface = (niFrameSurface1_t *) out->data[3];
    new_frame_surface = (niFrameSurface1_t *) s->api_dst_frame.data.frame.p_data[3];
    frame_1_surface->ui16FrameIdx   = new_frame_surface->ui16FrameIdx;
    frame_1_surface->ui16session_ID = new_frame_surface->ui16session_ID;
    frame_1_surface->device_handle  = new_frame_surface->device_handle;
    frame_1_surface->output_idx     = new_frame_surface->output_idx;
    frame_1_surface->src_cpu        = new_frame_surface->src_cpu;
    frame_1_surface->dma_buf_fd     = 0;

    ff_ni_set_bit_depth_and_encoding_type(&frame_1_surface->bit_depth,
                                          &frame_1_surface->encoding_type,
                                          frame_ctx->sw_format);

    /* Remove ni-split specific assets */
    frame_1_surface->ui32nodeAddress = 0;

    frame_1_surface->ui16width = out->width;
    frame_1_surface->ui16height = out->height;
    av_log(inlink->dst, AV_LOG_DEBUG,
               "%s:IN trace ui16FrameIdx = [%d] --> out [%d]\n",
               __func__, frame_0_surface->ui16FrameIdx, frame_1_surface->ui16FrameIdx);

    //out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t), ff_ni_frame_free, NULL, 0);
    av_frame_free(&frame);

    return ff_filter_frame(ctx->outputs[0], out);

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
    int ret;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    av_log(ctx, AV_LOG_TRACE, "%s: inlink framequeue %lu outlink framequeue %lu\n",
        __func__, ff_inlink_queued_frames(inlink), ff_inlink_queued_frames(outlink));

    if (ff_inlink_check_available_frame(inlink)) {
        // Consume from inlink framequeue only when outlink framequeue is empty, to prevent filter from exhausting all pre-allocated device buffers
        if (ff_inlink_check_available_frame(outlink))
            return FFERROR_NOT_READY;

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

#define OFFSET(x) offsetof(NetIntMergeContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_merge_options[] = {
    { "filterblit", "filterblit enable",       OFFSET(params.filterblit),     AV_OPT_TYPE_INT,    {.i64=0},    0, 4, FLAGS },
#if ((LIBXCODER_API_VERSION_MAJOR > 2) || (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 76))
    { "param_b",    "Parameter B for bicubic", OFFSET(params.scaler_param_b), AV_OPT_TYPE_DOUBLE, {.dbl=0.0},  0, 1, FLAGS },
    { "param_c",    "Parameter C for bicubic", OFFSET(params.scaler_param_c), AV_OPT_TYPE_DOUBLE, {.dbl=0.75}, 0, 1, FLAGS },
#endif
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_merge);

static const AVFilterPad inputs[] = {
    {
        .name         = "input",
        .type         = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_input,
#endif
        .filter_frame = filter_frame,
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
        .config_props  = config_output_dummy,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_merge_ni_quadra = {
    .name           = "ni_quadra_merge",
    .description    = NULL_IF_CONFIG_SMALL("NETINT Quadra merge a video source on top of the input v" NI_XCODER_REVISION),
    .uninit         = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate       = activate,
#endif
    .priv_size      = sizeof(NetIntMergeContext),
    .priv_class     = &ni_merge_class,
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs         = inputs,
    .outputs        = outputs,
    .query_formats  = query_formats,
#endif
};
