/*
 * Copyright (c) 2013 Stefano Sabatini
 * Copyright (c) 2008 Vitor Sessak
 * Copyright (c) 2022 NETINT Technologies Inc.
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
 * flip filter, based on the FFmpeg flip filter
*/

#include <string.h>

#include "libavutil/opt.h"

#include "nifilter.h"
#include "filters.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
#include "libavutil/avstring.h"

typedef struct NetIntFlipContext {
    const AVClass *class;

    AVBufferRef *out_frames_ref;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;

    int flip_type;
    bool initialized;
    bool session_opened;
    int64_t keep_alive_timeout;
    bool is_p2p;
    int buffer_limit;
} NetIntFlipContext;

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE };
    AVFilterFormats *fmts_list = NULL;

    fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list) {
        return AVERROR(ENOMEM);
    }

    return ff_set_common_formats(ctx, fmts_list);
}

static av_cold int init(AVFilterContext *ctx)
{
    NetIntFlipContext *flip = ctx->priv;
    flip->is_p2p = 0;
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntFlipContext *flip = ctx->priv;

    if (flip->api_dst_frame.data.frame.p_buffer) {
        ni_frame_buffer_free(&flip->api_dst_frame.data.frame);
    }

    if (flip->session_opened) {
        /* Close operation will free the device frames */
        ni_device_session_close(&flip->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&flip->api_ctx);
    }

    av_buffer_unref(&flip->out_frames_ref);

}

#if IS_FFMPEG_342_AND_ABOVE
static int config_props(AVFilterLink *outlink)
#else
static int config_props_delayed(AVFilterLink *outlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = outlink->src;
    NetIntFlipContext *flip = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVHWFramesContext *in_frames_ctx, *out_frames_ctx;

    // Quadra 2D engine only supports even pixel widths and heights
    outlink->w = FFALIGN(inlink->w, 2);
    outlink->h = FFALIGN(inlink->h, 2);

    if (outlink->w > NI_MAX_RESOLUTION_WIDTH ||
        outlink->h > NI_MAX_RESOLUTION_HEIGHT) {
        av_log(ctx, AV_LOG_ERROR, "Resolution %dx%d > %dx%d is not allowed\n",
               outlink->w, outlink->h,
               NI_MAX_RESOLUTION_WIDTH, NI_MAX_RESOLUTION_HEIGHT);
        return AVERROR(EINVAL);
    }

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *) li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *) ctx->inputs[0]->hw_frames_ctx->data;
#else
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

    av_log(ctx, AV_LOG_VERBOSE,
           "w:%d h:%d fmt:%s sar:%d/%d -> w:%d h:%d fmt:%s sar:%d/%d\n",
           inlink->w, inlink->h, av_get_pix_fmt_name(inlink->format),
           inlink->sample_aspect_ratio.num, inlink->sample_aspect_ratio.den,
           outlink->w, outlink->h, av_get_pix_fmt_name(outlink->format),
           outlink->sample_aspect_ratio.num, outlink->sample_aspect_ratio.den);


    flip->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!flip->out_frames_ref) {
        return AVERROR(ENOMEM);
    }

    out_frames_ctx = (AVHWFramesContext *) flip->out_frames_ref->data;

    out_frames_ctx->format = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width = outlink->w;
    out_frames_ctx->height = outlink->h;
    out_frames_ctx->sw_format = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size = NI_FLIP_ID; // Repurposed as identity code

    av_hwframe_ctx_init(flip->out_frames_ref);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(ctx->outputs[0]);
    av_buffer_unref(&lo->hw_frames_ctx);
    lo->hw_frames_ctx = av_buffer_ref(flip->out_frames_ref);

    if (!lo->hw_frames_ctx) {
        return AVERROR(ENOMEM);
    }
#else
    av_buffer_unref(&ctx->outputs[0]->hw_frames_ctx);
    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(flip->out_frames_ref);

    if (!ctx->outputs[0]->hw_frames_ctx) {
        return AVERROR(ENOMEM);
    }
#endif

    return 0;
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntFlipContext *flip = ctx->priv;
    AVHWFramesContext *out_frames_context;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    out_frames_context = (AVHWFramesContext*)flip->out_frames_ref->data;

    /* Don't check return code, this will intentionally fail */
    // av_hwframe_ctx_init(flip->out_frames_ref);

    if (flip->api_ctx.isP2P) {
        pool_size = 1;
    }
#if IS_FFMPEG_71_AND_ABOVE
    else {
        pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
    }
#endif
#if IS_FFMPEG_61_AND_ABOVE
    flip->buffer_limit = 1;
#endif
    /* Create frame pool on device */
    return ff_ni_build_frame_pool(&flip->api_ctx,
                                  out_frames_context->width,
                                  out_frames_context->height,
                                  out_frames_context->sw_format,
                                  pool_size,
                                  flip->buffer_limit);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    AVFrame *out = NULL;
    NetIntFlipContext *flip = ctx->priv;
    AVBufferRef *out_buffer_ref = flip->out_frames_ref;
    AVHWFramesContext *in_frames_context = (AVHWFramesContext *) in->hw_frames_ctx->data;
    AVNIDeviceContext *av_ni_device_context = (AVNIDeviceContext *) in_frames_context->device_ctx->hwctx;
    ni_retcode_t ni_retcode = NI_RETCODE_SUCCESS;
    niFrameSurface1_t *frame_surface = (niFrameSurface1_t *) in->data[3], *frame_surface2 = NULL;
    ni_frame_config_t input_frame_config = {0};
    uint32_t scaler_format;
    int retcode = 0, card_number =  ni_get_cardno(in);

    if (!frame_surface) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter frame_surface should not be NULL\n");
        return AVERROR(EINVAL);
    }

    if (!flip->initialized) {
#if !IS_FFMPEG_342_AND_ABOVE
        retcode = config_props_delayed(outlink, in);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni flip filter session config_props_delayed failure\n");
            retcode = AVERROR(EINVAL);
            goto FAIL;
        }

        out_buffer_ref = flip->out_frames_ref;
#endif

        ni_retcode = ni_device_session_context_init(&flip->api_ctx);
        if (ni_retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "ni flip filter session context init failed with %d\n", ni_retcode);
            retcode = AVERROR(EINVAL);
            goto FAIL;
        }

        flip->api_ctx.device_handle = flip->api_ctx.blk_io_handle = av_ni_device_context->cards[card_number];

        flip->api_ctx.hw_id = card_number;
        flip->api_ctx.device_type = NI_DEVICE_TYPE_SCALER;
        flip->api_ctx.scaler_operation = NI_SCALER_OPCODE_FLIP; //Flip operation compatible with crop
        flip->api_ctx.keep_alive_timeout = flip->keep_alive_timeout;
        flip->api_ctx.isP2P = flip->is_p2p;

        ni_retcode = ni_device_session_open(&flip->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (ni_retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "ni flip filter device session open failed with %d\n", ni_retcode);
            retcode = ni_retcode;
            /* Close operation will free the device frames */
            ni_device_session_close(&flip->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&flip->api_ctx);
            goto FAIL;
        }

        flip->session_opened = true;

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           inlink->dst->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        ni_retcode = init_out_pool(inlink->dst);
        if (ni_retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "ni flip filter init out pool failed with %d\n", ni_retcode);
            goto FAIL;
        }

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)out_buffer_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(in_frames_context, out_frames_ctx);
        ni_device_session_copy(&flip->api_ctx, &out_ni_ctx->api_ctx);

        AVHWFramesContext *pAVHFWCtx = (AVHWFramesContext *) in->hw_frames_ctx->data;
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

        if ((in->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
            av_log(ctx, AV_LOG_WARNING, "Full color range input, limited color output\n");
        }

        flip->initialized = true;
    }

    ni_retcode = ni_frame_buffer_alloc_hwenc(&flip->api_dst_frame.data.frame,
                                             outlink->w,
                                             outlink->h,
                                             0);
    if (ni_retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter frame buffer alloc hwenc failed with %d\n", ni_retcode);
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }

    // Input.

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(in_frames_context->sw_format);
    input_frame_config.picture_format = scaler_format;

    input_frame_config.rgba_color = frame_surface->ui32nodeAddress;
    input_frame_config.frame_index = frame_surface->ui16FrameIdx;

    input_frame_config.rectangle_x = 0;
    input_frame_config.rectangle_y = 0;
    input_frame_config.rectangle_width = input_frame_config.picture_width = in->width;
    input_frame_config.rectangle_height = input_frame_config.picture_height = in->height;

    if (flip->flip_type == 0) {
        //hflip
        input_frame_config.orientation = 4;
    } else if (flip->flip_type == 1) {
        //vflip
        input_frame_config.orientation = 5;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    // use ni_device_config_frame() instead of ni_device_alloc_frame()
    // such that input_frame_config's orientation can be configured
    ni_retcode = ni_device_config_frame(&flip->api_ctx, &input_frame_config);
    if (ni_retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter device config input frame failed with %d\n", ni_retcode);
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }

    ni_retcode = ni_device_alloc_frame(&flip->api_ctx,
                                       outlink->w,
                                       outlink->h,
                                       scaler_format,
                                       NI_SCALER_FLAG_IO,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       -1,
                                       NI_DEVICE_TYPE_SCALER);

    if (ni_retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter device alloc output frame failed with %d\n", ni_retcode);
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }

    out = av_frame_alloc();
    if (!out) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter av_frame_alloc returned NULL\n");
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }

    av_frame_copy_props(out, in);

    out->width = outlink->w;
    out->height = outlink->h;
    out->format = AV_PIX_FMT_NI_QUAD;
    out->color_range = AVCOL_RANGE_MPEG;

    out->hw_frames_ctx = av_buffer_ref(out_buffer_ref);
    out->data[3] = av_malloc(sizeof(niFrameSurface1_t));
    if (!out->data[3]) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter av_alloc returned NULL\n");
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }
    memcpy(out->data[3], frame_surface, sizeof(niFrameSurface1_t));

    ni_retcode = ni_device_session_read_hwdesc(&flip->api_ctx,
                                               &flip->api_dst_frame,
                                               NI_DEVICE_TYPE_SCALER);
    if (ni_retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter read hwdesc failed with %d\n", ni_retcode);
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_flip");
#endif

    frame_surface2 = (niFrameSurface1_t *) flip->api_dst_frame.data.frame.p_data[3];

    frame_surface = (niFrameSurface1_t *) out->data[3];
    frame_surface->ui16FrameIdx = frame_surface2->ui16FrameIdx;
    frame_surface->ui16session_ID = frame_surface2->ui16session_ID;
    frame_surface->device_handle = frame_surface2->device_handle;
    frame_surface->output_idx = frame_surface2->output_idx;
    frame_surface->src_cpu = frame_surface2->src_cpu;
    frame_surface->ui32nodeAddress = 0;
    frame_surface->dma_buf_fd = 0;
    ff_ni_set_bit_depth_and_encoding_type(&frame_surface->bit_depth,
                                          &frame_surface->encoding_type,
                                          in_frames_context->sw_format);
    frame_surface->ui16width = out->width;
    frame_surface->ui16height = out->height;

    out->buf[0] = av_buffer_create(out->data[3],
                                   sizeof(niFrameSurface1_t),
                                   ff_ni_frame_free,
                                   NULL,
                                   0);
    if (!out->buf[0]) {
        av_log(ctx, AV_LOG_ERROR, "ni flip filter av_buffer_create returned NULL\n");
        retcode = AVERROR(ENOMEM);
        goto FAIL;
    }

    av_frame_free(&in);
    return ff_filter_frame(inlink->dst->outputs[0], out);

FAIL:
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
    NetIntFlipContext *s = inlink->dst->priv;

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

#define OFFSET(x) offsetof(NetIntFlipContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM)

static const AVOption ni_flip_options[] = {
    { "flip_type",    "choose horizontal or vertical flip", OFFSET(flip_type), AV_OPT_TYPE_INT, {.i64 = 0},  0,  1, FLAGS, "flip_type" },
        { "horizontal", NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 0}, 0, 0, FLAGS, "flip_type" },
        { "h",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 0}, 0, 0, FLAGS, "flip_type" },
        { "veritcal",   NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 1}, 0, 0, FLAGS, "flip_type" },
        { "v",          NULL, 0, AV_OPT_TYPE_CONST, {.i64 = 1}, 0, 0, FLAGS, "flip_type" },
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_flip);

static const AVFilterPad inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
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

AVFilter ff_vf_flip_ni_quadra = {
    .name        = "ni_quadra_flip",
    .description = NULL_IF_CONFIG_SMALL(
        "NETINT Quadra flip the input video v" NI_XCODER_REVISION),
    .priv_size  = sizeof(NetIntFlipContext),
    .priv_class = &ni_flip_class,
    .init       = init,
    .uninit     = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate      = activate,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_QUERY_FUNC(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
#else
    .query_formats = query_formats,
    .inputs        = inputs,
    .outputs       = outputs,
#endif
// only FFmpeg 3.4.2 and above have .flags_internal
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
#endif
};
