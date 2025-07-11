/*
* Copyright (c) 2024 NetInt
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

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "nifilter.h"
#include "filters.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
#if HAVE_IO_H
#include <io.h>
#endif
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/eval.h"
#include "libavutil/colorspace.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/time.h"
#include "libswscale/swscale.h"
#include "drawutils.h"
#include "ni_device_api.h"
#include "ni_util.h"
#include "video.h"

#define NI_NUM_FRAMES_IN_QUEUE 8

// hvsplus related definition
typedef struct _ni_hvsplus_network_layer {
    int32_t width;
    int32_t height;
    int32_t channel;
    int32_t classes;
    int32_t component;
    int32_t output_number;
    float *output;
} ni_hvsplus_network_layer_t;

typedef struct _ni_hvsplus_nbsize {
    int32_t width;
    int32_t height;
} ni_hvsplus_nbsize_t;

typedef struct _ni_hvsplus_network {
    int32_t netw;
    int32_t neth;
    int32_t net_out_w;
    int32_t net_out_h;
    ni_network_data_t raw;
    ni_hvsplus_network_layer_t *layers;
} ni_hvsplus_network_t;

typedef struct HwPadContext {
    uint8_t rgba_color[4];  ///< color for the padding area
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
} HwPadContext;

typedef struct HwCropContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
} HwCropContext;

typedef struct AiContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_src_frame;
    ni_session_data_io_t api_dst_frame;
} AiContext;

typedef struct NetIntHvsplusContext {
    const AVClass *class;
    int level;
    int initialized;
    int devid;
    int in_width, in_height;
    int out_width, out_height;
    int nb_width, nb_height;
    int need_padding;

    AiContext *ai_ctx;
    AVBufferRef *out_frames_ref;
    HwPadContext *hwp_ctx;
    HwCropContext *hwc_ctx;

    ni_hvsplus_network_t network;

    int keep_alive_timeout; /* keep alive timeout setting */
    int ai_timeout;
    int channel_mode;
    int buffer_limit;
} NetIntHvsplusContext;

static const ni_hvsplus_nbsize_t nbSizes[] = {
    {512, 288},
    {704, 396},
    {720, 1280},
    {960, 540},
    {1280, 720},
    {1920, 1080},
    {3840, 2160}
};

// Find the smallest NB size that is equal to or larger than the input size
// -1: not supported, 0: matched, >0: index of nbSize + 1
static int findNBSize(int frameWidth, int frameHeight)
{

    int numSizes = sizeof(nbSizes) / sizeof(nbSizes[0]);
    int retval = -1;

    // Iterate through the existing NB sizes to find the smallest one that fits
    for (int i = 0; i < numSizes; i++) {
        if (frameWidth == nbSizes[i].width && frameHeight == nbSizes[i].height) {
            av_log(NULL, AV_LOG_INFO, "%s: matched w %d h %d\n", __func__, nbSizes[i].width, nbSizes[i].height);
            retval = 0;
            break;
        } else if (frameWidth <= nbSizes[i].width && frameHeight <= nbSizes[i].height) {
            av_log(NULL, AV_LOG_INFO, "%s: w %d h %d\n", __func__, nbSizes[i].width, nbSizes[i].height);
            retval = i+1;
            break;
        }
    }
    return retval;
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats;

    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUV420P10LE,
        AV_PIX_FMT_NI_QUAD,
        AV_PIX_FMT_NONE,
    };

    formats = ff_make_format_list(pix_fmts);
    if (!formats)
        return AVERROR(ENOMEM);

    return ff_set_common_formats(ctx, formats);
}

static void cleanup_ai_context(AVFilterContext *ctx, NetIntHvsplusContext *s)
{
    ni_retcode_t retval;
    AiContext *ai_ctx = s->ai_ctx;

    if (ai_ctx) {
        ni_frame_buffer_free(&ai_ctx->api_src_frame.data.frame);

        retval =
            ni_device_session_close(&ai_ctx->api_ctx, 1, NI_DEVICE_TYPE_AI);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                    "Error: failed to close ai session. retval %d\n", retval);
        }
        if (ai_ctx->api_ctx.hw_action != NI_CODEC_HW_ENABLE) {
#ifdef _WIN32
            if (ai_ctx->api_ctx.device_handle != NI_INVALID_DEVICE_HANDLE) {
                ni_device_close(ai_ctx->api_ctx.device_handle);
            }
#elif __linux__
            if (ai_ctx->api_ctx.device_handle != NI_INVALID_DEVICE_HANDLE) {
                ni_device_close(ai_ctx->api_ctx.device_handle);
            }
            if (ai_ctx->api_ctx.blk_io_handle != NI_INVALID_DEVICE_HANDLE) {
                ni_device_close(ai_ctx->api_ctx.blk_io_handle);
            }
#endif
            ni_packet_buffer_free(&ai_ctx->api_dst_frame.data.packet);
            ai_ctx->api_ctx.device_handle = NI_INVALID_DEVICE_HANDLE;
            ai_ctx->api_ctx.blk_io_handle = NI_INVALID_DEVICE_HANDLE;
        } else {
            ni_frame_buffer_free(&ai_ctx->api_dst_frame.data.frame);
        }
        ni_device_session_context_clear(&ai_ctx->api_ctx);
        av_free(ai_ctx);
        s->ai_ctx = NULL;
    }
}


static int init_ai_context(AVFilterContext *ctx, NetIntHvsplusContext *s,
                           AVFrame *frame)
{
    ni_retcode_t retval;
    AiContext *ai_ctx;
    ni_hvsplus_network_t *network = &s->network;
    int hwframe = frame->format == AV_PIX_FMT_NI_QUAD ? 1 : 0;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    AVHWFramesContext *out_frames_ctx;
    AVNIFramesContext *f_hwctx;
    int cardno;
    int format;
    int options;

    av_log(ctx, AV_LOG_INFO, "%s: %d x %d format %s\n", __func__,
           s->out_width, s->out_height, av_get_pix_fmt_name(frame->format));

    ai_ctx = av_mallocz(sizeof(AiContext));
    if (!ai_ctx) {
        av_log(ctx, AV_LOG_ERROR, "Error: failed to allocate ai context\n");
        return AVERROR(ENOMEM);
    }
    s->ai_ctx = ai_ctx;
    retval = ni_device_session_context_init(&ai_ctx->api_ctx);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: ai session context init failure\n");
        return AVERROR(EIO);
    }

    if (hwframe) {
        pAVHFWCtx = (AVHWFramesContext*) frame->hw_frames_ctx->data;
        pAVNIDevCtx = (AVNIDeviceContext*) pAVHFWCtx->device_ctx->hwctx;
        cardno = ni_get_cardno(frame);

        ai_ctx->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        ai_ctx->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];
        ai_ctx->api_ctx.hw_action = NI_CODEC_HW_ENABLE;
        ai_ctx->api_ctx.hw_id = cardno;
    } else {
        ai_ctx->api_ctx.hw_id = s->devid;
    }

    ai_ctx->api_ctx.device_type = NI_DEVICE_TYPE_AI;
    ai_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    retval = ni_device_session_open(&ai_ctx->api_ctx, NI_DEVICE_TYPE_AI);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: failed to open ai session. retval %d\n",
                retval);
        ret = AVERROR(EIO);
        goto failed_out;
    }

    // Configure NB file
    av_log(ctx, AV_LOG_DEBUG, "%s: out w %d h %d NB w %d h %d sw_format %s pixel_format %d\n", __func__,
            s->out_width, s->out_height, s->nb_width, s->nb_height, av_get_pix_fmt_name(hwframe?pAVHFWCtx->sw_format:frame->format),
            ai_ctx->api_ctx.pixel_format);

    ai_ctx->api_ctx.active_video_width = s->nb_width;
    ai_ctx->api_ctx.active_video_height = s->nb_height;
    ai_ctx->api_ctx.hvsplus_level = s->level;
    ai_ctx->api_ctx.pixel_format = ff_ni_ffmpeg_to_libxcoder_pix_fmt(
            (hwframe ? pAVHFWCtx->sw_format : frame->format));

    retval = ni_ai_config_hvsplus(&ai_ctx->api_ctx, &network->raw);

    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: failed to configure ai session. retval %d\n",
                retval);
        ret = AVERROR(EIO);
        goto failed_out;
    }

    if (!hwframe) {
        return 0;
    }
    out_frames_ctx = (AVHWFramesContext*) s->out_frames_ref->data;
    f_hwctx = (AVNIFramesContext*) out_frames_ctx->hwctx;
    f_hwctx->api_ctx.session_timestamp = ai_ctx->api_ctx.session_timestamp;

    // Create frame pool
    format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);
    options = NI_AI_FLAG_IO | NI_AI_FLAG_PC;
    if (s->buffer_limit)
        options |= NI_AI_FLAG_LM;

    /* Allocate a pool of frames by the AI */
    retval = ni_device_alloc_frame(&ai_ctx->api_ctx, FFALIGN(s->nb_width, 2),
            FFALIGN(s->nb_height, 2), format, options, 0, // rec width
            0, // rec height
            0, // rec X pos
            0, // rec Y pos
            8, // rgba color/pool size
            0, // frame index
            NI_DEVICE_TYPE_AI);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: failed to create buffer pool\n");
        ret = AVERROR(ENOMEM);
        goto failed_out;
    }
    retval = ni_frame_buffer_alloc_hwenc(&ai_ctx->api_dst_frame.data.frame,
            FFALIGN(s->nb_width, 2), FFALIGN(s->nb_height, 2), 0);

    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: failed to allocate ni dst frame\n");
        ret = AVERROR(ENOMEM);
        goto failed_out;
    }

    return 0;

    failed_out: cleanup_ai_context(ctx, s);
    return ret;
}

static void ni_destroy_network(AVFilterContext *ctx,
                               ni_hvsplus_network_t *network) {
    if (network) {
        int i;

        if (network->layers) {
            for (i = 0; i < network->raw.output_num; i++) {
                if (network->layers[i].output) {
                    free(network->layers[i].output);
                    network->layers[i].output = NULL;
                }
            }

            free(network->layers);
            network->layers = NULL;
        }
    }
}

static int ni_create_network(AVFilterContext *ctx, ni_hvsplus_network_t *network)
{
    int ret;
    int i;
    ni_network_data_t *ni_network = &network->raw;

    av_log(ctx, AV_LOG_INFO, "network input number %d, output number %d\n",
          ni_network->input_num, ni_network->output_num);

    if (ni_network->input_num == 0 || ni_network->output_num == 0) {
        av_log(ctx, AV_LOG_ERROR, "Error: invalid network layer\n");
        return AVERROR(EINVAL);
    }

    network->layers =
        malloc(sizeof(ni_hvsplus_network_layer_t) * ni_network->output_num);
    if (!network->layers) {
        av_log(ctx, AV_LOG_ERROR, "Error: cannot allocate network layer memory\n");
        return AVERROR(ENOMEM);
    }
    memset(network->layers, 0,
          sizeof(ni_hvsplus_network_layer_t) * ni_network->output_num);

    for (i = 0; i < ni_network->output_num; i++) {
        network->layers[i].channel   = ni_network->linfo.out_param[i].sizes[0];
        network->layers[i].width     = ni_network->linfo.out_param[i].sizes[1];
        network->layers[i].height    = ni_network->linfo.out_param[i].sizes[2];
        network->layers[i].component = 3;
        network->layers[i].classes =
            (network->layers[i].channel / network->layers[i].component) -
            (4 + 1);
        network->layers[i].output_number =
            ni_ai_network_layer_dims(&ni_network->linfo.out_param[i]);
        av_assert0(network->layers[i].output_number ==
                   network->layers[i].width * network->layers[i].height *
                   network->layers[i].channel);

        network->layers[i].output =
            malloc(network->layers[i].output_number * sizeof(float));
        if (!network->layers[i].output) {
            av_log(ctx, AV_LOG_ERROR,
                   "Error: failed to allocate network layer %d output buffer\n", i);
            ret = AVERROR(ENOMEM);
            goto out;
        }

        av_log(ctx, AV_LOG_DEBUG, "%s: network layer %d: w %d, h %d, ch %d, co %d, cl %d\n", __func__, i,
               network->layers[i].width, network->layers[i].height,
               network->layers[i].channel, network->layers[i].component,
               network->layers[i].classes);
    }

    network->netw = ni_network->linfo.in_param[0].sizes[1];
    network->neth = ni_network->linfo.in_param[0].sizes[2];
    network->net_out_w = ni_network->linfo.out_param[0].sizes[1];
    network->net_out_h = ni_network->linfo.out_param[0].sizes[2];

    return 0;
out:
    ni_destroy_network(ctx, network);
    return ret;
}

static av_cold int init_hwframe_pad(AVFilterContext *ctx, NetIntHvsplusContext *s,
                                        enum AVPixelFormat format,
                                        AVFrame *frame)
{
    ni_retcode_t retval;
    HwPadContext *hwp_ctx;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    int cardno;

    av_log(ctx, AV_LOG_INFO, "%s: format %s\n", __func__, av_get_pix_fmt_name(format));

    hwp_ctx = av_mallocz(sizeof(HwPadContext));
    if (!hwp_ctx) {
        av_log(ctx, AV_LOG_ERROR, "Error: could not allocate hwframe ctx\n");
        return AVERROR(ENOMEM);
    }
    s->hwp_ctx = hwp_ctx;
    ni_device_session_context_init(&hwp_ctx->api_ctx);

    pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno      = ni_get_cardno(frame);

    hwp_ctx->api_ctx.device_handle      = pAVNIDevCtx->cards[cardno];
    hwp_ctx->api_ctx.blk_io_handle      = pAVNIDevCtx->cards[cardno];
    hwp_ctx->api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
    hwp_ctx->api_ctx.scaler_operation   = NI_SCALER_OPCODE_PAD;
    hwp_ctx->api_ctx.hw_id              = cardno;
    hwp_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
    hwp_ctx->api_ctx.isP2P              = 0;
    hwp_ctx->rgba_color[0]              = 0;
    hwp_ctx->rgba_color[1]              = 0;
    hwp_ctx->rgba_color[2]              = 0;
    hwp_ctx->rgba_color[3]              = 255;


    retval = ni_device_session_open(&hwp_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: could not open scaler session\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&hwp_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hwp_ctx->api_ctx);
        goto out;
    }
#if IS_FFMPEG_61_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create scale frame pool on device */
    retval = ff_ni_build_frame_pool(&hwp_ctx->api_ctx, s->nb_width,
                                    s->nb_height, format,
                                    DEFAULT_NI_FILTER_POOL_SIZE, s->buffer_limit);

    if (retval < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error: could not build frame pool\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&hwp_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hwp_ctx->api_ctx);
        goto out;
    }

    return 0;
out:
    av_free(hwp_ctx);
    return ret;
}

static void cleanup_hwframe_pad(NetIntHvsplusContext *s)
{
    HwPadContext *hwp_ctx = s->hwp_ctx;

    if (hwp_ctx) {
        ni_frame_buffer_free(&hwp_ctx->api_dst_frame.data.frame);
        ni_device_session_close(&hwp_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hwp_ctx->api_ctx);
        av_free(hwp_ctx);
        s->hwp_ctx = NULL;
    }
}

static av_cold int init_hwframe_crop(AVFilterContext *ctx, NetIntHvsplusContext *s,
                                        enum AVPixelFormat format,
                                        AVFrame *frame)
{
    ni_retcode_t retval;
    HwCropContext *hwc_ctx;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    int cardno;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    av_log(ctx, AV_LOG_INFO, "%s: format %s frame pool for w %d h %d\n",
            __func__, av_get_pix_fmt_name(format), s->in_width, s->in_height);

    hwc_ctx = av_mallocz(sizeof(HwCropContext));
    if (!hwc_ctx) {
        av_log(ctx, AV_LOG_ERROR, "Error: could not allocate hwframe ctx\n");
        return AVERROR(ENOMEM);
    }
    s->hwc_ctx = hwc_ctx;
    ni_device_session_context_init(&hwc_ctx->api_ctx);

    pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno      = ni_get_cardno(frame);

    hwc_ctx->api_ctx.device_handle     = pAVNIDevCtx->cards[cardno];
    hwc_ctx->api_ctx.blk_io_handle     = pAVNIDevCtx->cards[cardno];
    hwc_ctx->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
    hwc_ctx->api_ctx.scaler_operation  = NI_SCALER_OPCODE_CROP;
    hwc_ctx->api_ctx.hw_id             = cardno;
    hwc_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    retval = ni_device_session_open(&hwc_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error: could not open scaler session\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&hwc_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hwc_ctx->api_ctx);
        goto out;
    }
#if IS_FFMPEG_71_AND_ABOVE
    pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
#endif
#if IS_FFMPEG_70_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create scale frame pool on device */
    retval = ff_ni_build_frame_pool(&hwc_ctx->api_ctx, s->in_width,
                                    s->in_height, format,
                                    pool_size, s->buffer_limit);

    if (retval < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error: could not build frame pool\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&hwc_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hwc_ctx->api_ctx);
        goto out;
    }

    return 0;
out:
    av_free(hwc_ctx);
    return ret;
}

static void cleanup_hwframe_crop(NetIntHvsplusContext *s)
{
    HwCropContext *hwc_ctx = s->hwc_ctx;

    if (hwc_ctx) {
        ni_frame_buffer_free(&hwc_ctx->api_dst_frame.data.frame);
        ni_device_session_close(&hwc_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hwc_ctx->api_ctx);
        av_free(hwc_ctx);
        s->hwc_ctx = NULL;
    }
}

static av_cold int init(AVFilterContext *ctx)
{
    NetIntHvsplusContext *s = ctx->priv;

    s->initialized = 0;
    s->nb_width = -1;
    s->nb_height = -1;
    s->need_padding = 0;

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntHvsplusContext *s  = ctx->priv;
    ni_hvsplus_network_t *network = &s->network;

    cleanup_ai_context(ctx, s);

    ni_destroy_network(ctx, network);

    av_buffer_unref(&s->out_frames_ref);
    s->out_frames_ref = NULL;

    if (s->need_padding) {
        cleanup_hwframe_pad(s);
        cleanup_hwframe_crop(s);
    }
}

static int config_input(AVFilterContext *ctx, AVFrame *frame)
{
    NetIntHvsplusContext *s = ctx->priv;
    int hwframe = frame->format == AV_PIX_FMT_NI_QUAD ? 1 : 0;
    int ret;

    if (s->initialized)
        return 0;

    ret = init_ai_context(ctx, s, frame);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error: failed to initialize ai context\n");
        return ret;
    }

    ret = ni_create_network(ctx, &s->network);
    if (ret != 0) {
        goto fail_out;
    }

    if (hwframe && s->need_padding) {
        AVHWFramesContext *pAVHFWCtx = (AVHWFramesContext*) frame->hw_frames_ctx->data;
        av_log(ctx, AV_LOG_INFO, "%s: hw frame sw format %s\n", __func__, av_get_pix_fmt_name(pAVHFWCtx->sw_format));

        ret = init_hwframe_pad(ctx, s, pAVHFWCtx->sw_format, frame);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Error: could not initialized hwframe pad context\n");
            goto fail_out;
        }

        ret = init_hwframe_crop(ctx, s, pAVHFWCtx->sw_format, frame);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Error: could not initialized hwframe crop context\n");
            goto fail_out;
        }
    }

    s->initialized = 1;
    return 0;

fail_out:
    cleanup_ai_context(ctx, s);

    ni_destroy_network(ctx, &s->network);

    return ret;
}

static int output_config_props_internal(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    NetIntHvsplusContext *s = ctx->priv;
    int out_width, out_height;

    if (s->out_width == -1 || s->out_height == -1) {
        out_width = inlink->w;
        out_height = inlink->h;
        s->out_width = out_width;
        s->out_height = out_height;
    } else {
        out_width = s->out_width;
        out_height = s->out_height;
    }

    s->in_width = inlink->w;
    s->in_height = inlink->h;

    av_log(ctx, AV_LOG_INFO, "%s: need_padding %d s->out_width %d s->out_height %d\n", __func__, s->need_padding, s->out_width, s->out_height);

    outlink->w = out_width;
    outlink->h = out_height;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_DEBUG, "sw frame\n");
        return 0;
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#else
    if (inlink->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_DEBUG, "sw frame\n");
        return 0;
    }
    in_frames_ctx = (AVHWFramesContext *)ctx->inputs[0]->hw_frames_ctx->data;
#endif

    if (in_frames_ctx->format != AV_PIX_FMT_NI_QUAD) {
        av_log(ctx, AV_LOG_ERROR, "Error: pixel format not supported, format=%d\n", in_frames_ctx->format);
        return AVERROR(EINVAL);
    }
    if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
        in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(ctx, AV_LOG_ERROR, "tile4x4 not supported\n");
        return AVERROR(EINVAL);
    }


    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    out_frames_ctx->format            = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width             = outlink->w;
    out_frames_ctx->height            = outlink->h;
    out_frames_ctx->sw_format         = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size = NI_HVSPLUS_ID;

    av_log(ctx, AV_LOG_INFO, "%s: w %d h %d\n", __func__, out_frames_ctx->width, out_frames_ctx->height);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(ctx->outputs[0]);
    av_buffer_unref(&lo->hw_frames_ctx);
    lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);

    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
    av_buffer_unref(&ctx->outputs[0]->hw_frames_ctx);
    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);

    if (!ctx->outputs[0]->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif

    return 0;
}

static int output_config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    NetIntHvsplusContext *s = ctx->priv;
    int out_width, out_height, retval, ret;

    av_log(ctx, AV_LOG_DEBUG, "%s: inlink src %s dst %s filter %p w %d h %d\n", __func__, inlink->src->name, inlink->dst->name, s, inlink->w, inlink->h);
    av_log(ctx, AV_LOG_DEBUG, "%s: outlink src %s dst %s filter %p w %d h %d\n", __func__, outlink->src->name, outlink->dst->name, s, outlink->w, outlink->h);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if ((li->hw_frames_ctx == NULL) && (inlink->format == AV_PIX_FMT_NI_QUAD)) {
        av_log(ctx, AV_LOG_ERROR, "Error: No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#else
    if ((inlink->hw_frames_ctx == NULL) && (inlink->format == AV_PIX_FMT_NI_QUAD)) {
        av_log(ctx, AV_LOG_ERROR, "Error: No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#endif
    if (s->out_width == -1 || s->out_height == -1) {
        out_width = inlink->w;
        out_height = inlink->h;
    } else {
        out_width = s->out_width;
        out_height = s->out_height;
    }

    // Find the width and height to be used for the AI hvs filter.
    // If they match the supporting sizes of network binary, proceed.
    // If they don't match, padding and cropping are needed before and after hvsplus filter respectively.
    // If they are greater than 4k, it is not supported. Then, exit.
    retval = findNBSize(inlink->w, inlink->h);
    if (retval < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error: hvsplus doesn't support resolution greater than 4K (width %d height %d).\n", out_width, out_height);
        return AVERROR(EINVAL);
    }

    if (retval == 0) {
        s->nb_width = inlink->w;
        s->nb_height = inlink->h;
    } else {
        s->nb_width = nbSizes[retval-1].width;
        s->nb_height = nbSizes[retval-1].height;
        s->need_padding = 1;
    }

    av_log(ctx, AV_LOG_DEBUG, "%s: inlink w %d h %d NB w %d h %d need_padding %d\n",
            __func__, inlink->w, inlink->h, s->nb_width, s->nb_height, s->need_padding);

    ret = output_config_props_internal(outlink);

    return ret;
}

static int av_to_niframe_copy(NetIntHvsplusContext *s, ni_frame_t *dst, const AVFrame *src, int nb_planes)
{
    int dst_stride[4],src_height[4], hpad[4], vpad[4], linesize[4];
    int i, j, h;
    uint8_t *src_line, *dst_line, YUVsample, *sample, *dest;
    uint16_t lastidx;
    bool tenBit;

    av_log(NULL, AV_LOG_DEBUG, "%s: src width %d height %d nb w %d nb h %d format %s linesize %d %d %d nb_planes %d\n", __func__,
           src->width, src->height, s->nb_width, s->nb_height, av_get_pix_fmt_name(src->format),
           src->linesize[0], src->linesize[1], src->linesize[2], nb_planes);

    linesize[0] = src->linesize[0];
    linesize[1] = src->linesize[1];
    linesize[2] = src->linesize[2];
    linesize[3] = 0;

    switch (src->format) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
        dst_stride[0] = FFALIGN(s->nb_width, 128);
        dst_stride[1] = FFALIGN((s->nb_width / 2), 128);
        dst_stride[2] = dst_stride[1];
        dst_stride[3] = 0;

        linesize[0] = FFALIGN(src->width, 2);
        linesize[1] = FFALIGN(src->width, 2) / 2;
        linesize[2] = linesize[1];

        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - linesize[1], 0);
        hpad[2] = FFMAX(dst_stride[2] - linesize[2], 0);
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = FFALIGN(src->height, 2) / 2;
        src_height[3] = 0;

        vpad[0] = FFALIGN(s->nb_height, 2) - src->height;
        vpad[1] = (FFALIGN(s->nb_height, 2) / 2) - (FFALIGN(src->height, 2) / 2);
        vpad[2] = (FFALIGN(s->nb_height, 2) / 2) - (FFALIGN(src->height, 2) / 2);
        vpad[3] = 0;

        tenBit = false;
        break;
    case AV_PIX_FMT_YUV420P10LE:
        dst_stride[0] = FFALIGN(s->nb_width * 2, 128);
        dst_stride[1] = FFALIGN(s->nb_width, 128);
        dst_stride[2] = dst_stride[1];
        dst_stride[3] = 0;

        linesize[0] = src->width * 2;
        linesize[1] = src->width;
        linesize[2] = linesize[1];

        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - linesize[1], 0);
        hpad[2] = FFMAX(dst_stride[2] - linesize[2], 0);
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = FFALIGN(src->height, 2) / 2;
        src_height[3] = 0;

        vpad[0] = FFALIGN(s->nb_height, 2) - src->height;
        vpad[1] = (FFALIGN(s->nb_height, 2) / 2) - (FFALIGN(src->height, 2) / 2);
        vpad[2] = (FFALIGN(s->nb_height, 2) / 2) - (FFALIGN(src->height, 2) / 2);
        vpad[3] = 0;

        tenBit = true;
        break;
    case AV_PIX_FMT_NV12:
        dst_stride[0] = FFALIGN(src->width, 128);
        dst_stride[1] = dst_stride[0];
        dst_stride[2] = 0;
        dst_stride[3] = 0;
        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - linesize[1], 0);
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = FFALIGN(src_height[0], 2) - src_height[0];
        vpad[1] = FFALIGN(src_height[1], 2) - src_height[1];
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;
    case AV_PIX_FMT_NV16:
        dst_stride[0] = FFALIGN(src->width, 64);
        dst_stride[1] = dst_stride[0];
        dst_stride[2] = 0;
        dst_stride[3] = 0;
        hpad[0] = 0;
        hpad[1] = 0;
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = src->height;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;
    case AV_PIX_FMT_P010LE:
        dst_stride[0] = FFALIGN(src->width * 2, 128);
        dst_stride[1] = dst_stride[0];
        dst_stride[2] = 0;
        dst_stride[3] = 0;
        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - linesize[1], 0);
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = FFALIGN(src_height[0], 2) - src_height[0];
        vpad[1] = FFALIGN(src_height[1], 2) - src_height[1];
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = true;
        break;
    case AV_PIX_FMT_RGBA:
    case AV_PIX_FMT_BGRA:
    case AV_PIX_FMT_ABGR:
    case AV_PIX_FMT_ARGB:
    case AV_PIX_FMT_BGR0:
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;
        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = 0;
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;
    case AV_PIX_FMT_BGRP:
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;
        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - linesize[1], 0);
        hpad[2] = FFMAX(dst_stride[2] - linesize[2], 0);
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = src->height;
        src_height[2] = src->height;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;
    case AV_PIX_FMT_YUYV422:
    case AV_PIX_FMT_UYVY422:
        dst_stride[0] = FFALIGN(src->width, 16) * 2;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;
        hpad[0] = FFMAX(dst_stride[0] - linesize[0], 0);
        hpad[1] = 0;
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;
    default:
        av_log(NULL, AV_LOG_ERROR, "Error: Pixel format %s not supported\n",
               av_get_pix_fmt_name(src->format));
        return AVERROR(EINVAL);
    }
    av_log(NULL, AV_LOG_DEBUG, "%s: dst_stride %d %d %d linesize %d %d %d hpad %d %d %d\n", __func__,
           dst_stride[0], dst_stride[1], dst_stride[2],
           src->linesize[0], src->linesize[1], src->linesize[2],
           hpad[0], hpad[1], hpad[2]);
    av_log(NULL, AV_LOG_DEBUG, "%s: src_height %d %d %d vpad %d %d %d tenBit %d\n", __func__,
           src_height[0], src_height[1], src_height[2],
           vpad[0], vpad[1], vpad[2], tenBit);

    dst_line = dst->p_buffer;
    for (i = 0; i < nb_planes; i++) {
        src_line = src->data[i];
        for (h = 0; h < src_height[i]; h++) {
            memcpy(dst_line, src_line, FFMIN(linesize[i], dst_stride[i]));

            if (h == 0)
                av_log(NULL, AV_LOG_DEBUG, "%s: i %d h %d to %d memcpy size %d\n", __func__, i, h, src_height[i]-1, FFMIN(linesize[i], dst_stride[i]));

            if (hpad[i]) {
                lastidx = linesize[i];

                if (tenBit) {
                    sample = &src_line[lastidx - 2];
                    dest   = &dst_line[lastidx];

                    /* two bytes per sample */
                    for (j = 0; j < hpad[i] / 2; j++) {
                        memcpy(dest, sample, 2);
                        dest += 2;
                    }
                    if (h == 0)
                        av_log(NULL, AV_LOG_DEBUG, "%s: i %d hpad %d to %d memset size %d value %d %d tenBit\n", __func__, i, h, src_height[i]-1, hpad[i], sample[0], sample[1]);

                } else {
                    YUVsample = dst_line[lastidx - 1];
                    memset(&dst_line[lastidx], YUVsample, hpad[i]);

                    if (h == 0)
                        av_log(NULL, AV_LOG_DEBUG, "%s: i %d hpad %d to %d memset size %d value %d\n", __func__, i, h, src_height[i]-1, hpad[i], YUVsample);
                }
            }

            src_line += src->linesize[i];
            dst_line += dst_stride[i];
        }

        /* Extend the height by cloning the last line */
        src_line = dst_line - dst_stride[i];
        for (h = 0; h < vpad[i]; h++) {
            memcpy(dst_line, src_line, dst_stride[i]);

            av_log(NULL, AV_LOG_DEBUG, "%s: h %d memcpy vpad size %d\n", __func__, h, dst_stride[i]);

            dst_line += dst_stride[i];
        }
    }

    return 0;
}

static int ni_to_avframe_copy(NetIntHvsplusContext *s, AVFrame *dst, const ni_packet_t *src, int nb_planes)
{
    int src_linesize[4], src_height[4], dst_height[4];
    int i, h;
    uint8_t *src_line, *dst_line;

    av_log(NULL, AV_LOG_DEBUG, "%s: dst width %d height %d nb w %d nb h %d format %s nb_planes %d\n", __func__,
           dst->width, dst->height, s->nb_width, s->nb_height, av_get_pix_fmt_name(dst->format), nb_planes);

    dst_height[0] = dst->height;
    dst_height[1] = dst->height / 2;
    dst_height[2] = dst_height[1];
    dst_height[3] = 0;

    switch (dst->format) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
        src_linesize[0] = FFALIGN(s->nb_width, 128);
        src_linesize[1] = FFALIGN(s->nb_width / 2, 128);
        src_linesize[2] = src_linesize[1];
        src_linesize[3] = 0;

        src_height[0] = s->nb_height;
        src_height[1] = FFALIGN(s->nb_height, 2) / 2;
        src_height[2] = src_height[1];
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_YUV420P10LE:
        src_linesize[0] = FFALIGN(s->nb_width * 2, 128);
        src_linesize[1] = FFALIGN(s->nb_width, 128);
        src_linesize[2] = src_linesize[1];
        src_linesize[3] = 0;

        src_height[0] = s->nb_height;
        src_height[1] = FFALIGN(s->nb_height, 2) / 2;
        src_height[2] = src_height[1];
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_NV12:
        src_linesize[0] = FFALIGN(dst->width, 128);
        src_linesize[1] = FFALIGN(dst->width, 128);
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = FFALIGN(dst->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_NV16:
        src_linesize[0] = FFALIGN(dst->width, 64);
        src_linesize[1] = FFALIGN(dst->width, 64);
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = dst->height;
        src_height[2] = 0;
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_YUYV422:
    case AV_PIX_FMT_UYVY422:
        src_linesize[0] = FFALIGN(dst->width, 16) * 2;
        src_linesize[1] = 0;
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_P010LE:
        src_linesize[0] = FFALIGN(dst->width * 2, 128);
        src_linesize[1] = FFALIGN(dst->width * 2, 128);
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = FFALIGN(dst->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_RGBA:
    case AV_PIX_FMT_BGRA:
    case AV_PIX_FMT_ABGR:
    case AV_PIX_FMT_ARGB:
    case AV_PIX_FMT_BGR0:
        src_linesize[0] = FFALIGN(dst->width, 16) * 4;
        src_linesize[1] = 0;
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;
        break;
    case AV_PIX_FMT_BGRP:
        src_linesize[0] = FFALIGN(dst->width, 32);
        src_linesize[1] = FFALIGN(dst->width, 32);
        src_linesize[2] = FFALIGN(dst->width, 32);
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = dst->height;
        src_height[2] = dst->height;
        src_height[3] = 0;
        break;
    default:
        av_log(NULL, AV_LOG_ERROR, "Error: Unsupported pixel format %s\n",
               av_get_pix_fmt_name(dst->format));
        return AVERROR(EINVAL);
    }
    av_log(NULL, AV_LOG_DEBUG, "%s: src_linesize %d %d %d src_height %d %d %d dst linesize %d %d %d dst_height %d %d %d\n", __func__,
           src_linesize[0], src_linesize[1], src_linesize[2],
           src_height[0], src_height[1], src_height[2],
           dst->linesize[0], dst->linesize[1], dst->linesize[2],
           dst_height[0], dst_height[1], dst_height[2]);

    src_line = src->p_data;
    for (i = 0; i < nb_planes; i++) {
        dst_line = dst->data[i];

        for (h = 0; h < src_height[i]; h++) {
            if (h < dst_height[i]) {
                memcpy(dst_line, src_line,
                       FFMIN(src_linesize[i], dst->linesize[i]));
                if (h == 0)
                    av_log(NULL, AV_LOG_DEBUG, "%s: i %d h %d to %d memcpy size %d\n", __func__, i, h, src_height[i]-1,  FFMIN(src_linesize[i], dst->linesize[i]));
                dst_line += FFMIN(src_linesize[i], dst->linesize[i]);
            }
            src_line += src_linesize[i];
        }
    }

    return 0;
}

static int ni_hwframe_pad(AVFilterContext *ctx, NetIntHvsplusContext *s, AVFrame *in,
                          int w, int h,
                          niFrameSurface1_t **filt_frame_surface)
{
    HwPadContext *pad_ctx = s->hwp_ctx;
    uint32_t ui32RgbaColor, scaler_format;
    ni_retcode_t retcode;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;

    frame_surface = (niFrameSurface1_t *)in->data[3];

    pAVHFWCtx = (AVHWFramesContext *)in->hw_frames_ctx->data;

    av_log(ctx, AV_LOG_DEBUG, "%s: in frame surface frameIdx %d sw_format %s w %d h %d\n", __func__,
           frame_surface->ui16FrameIdx, av_get_pix_fmt_name(pAVHFWCtx->sw_format), w, h);

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&pad_ctx->api_dst_frame.data.frame,
                                          w, h, 0);
    if (retcode != NI_RETCODE_SUCCESS)
        return AVERROR(ENOMEM);

    av_log(ctx, AV_LOG_DEBUG,
           "%s: inlink->w = %d;inlink->h = %d;outlink->w = %d;outlink->h = %d\n", __func__,
           in->width, in->height, s->nb_width, s->nb_height);
    av_log(ctx, AV_LOG_DEBUG,
           "%s: s->w=%d;s->h=%d;s->x=%d;s->y=%d;c=%02x:%02x:%02x:%02x\n", __func__, w,
           h, 0, 0, pad_ctx->rgba_color[0], pad_ctx->rgba_color[1],
           pad_ctx->rgba_color[2], pad_ctx->rgba_color[3]);

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(&pad_ctx->api_ctx,
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
        av_log(NULL, AV_LOG_ERROR, "Error: Can't allocate device input frame %d\n", retcode);
        return AVERROR(ENOMEM);
    }

    /* Scaler uses BGRA color, or ARGB in little-endian */
    ui32RgbaColor = (pad_ctx->rgba_color[3] << 24) | (pad_ctx->rgba_color[0] << 16) |
                    (pad_ctx->rgba_color[1] << 8) | pad_ctx->rgba_color[2];

    /* Allocate device destination frame. This will acquire a frame from the pool */
    retcode = ni_device_alloc_frame(&pad_ctx->api_ctx,
                                    FFALIGN(s->nb_width,2),
                                    FFALIGN(s->nb_height,2),
                                    scaler_format,
                                    NI_SCALER_FLAG_IO,    // output frame
                                    in->width,            // dst rectangle width
                                    in->height,           // dst rectangle height
                                    0, //s->x,                 // dst rectangle x
                                    0, //s->y,                 // dst rectangle y
                                    ui32RgbaColor,        // rgba color
                                    -1,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "Error: Can't allocate device output frame %d\n", retcode);
        return AVERROR(ENOMEM);
    }

    /* Set the new frame index */
    ni_device_session_read_hwdesc(
        &pad_ctx->api_ctx, &pad_ctx->api_dst_frame, NI_DEVICE_TYPE_SCALER);
    new_frame_surface =
        (niFrameSurface1_t *)pad_ctx->api_dst_frame.data.frame.p_data[3];

    new_frame_surface->ui16width = s->nb_width;
    new_frame_surface->ui16height = s->nb_height;

    *filt_frame_surface = new_frame_surface;

    return 0;
}

static int ni_hwframe_crop(AVFilterContext *ctx, NetIntHvsplusContext *s, AVFrame *in,
                          int w, int h,
                          niFrameSurface1_t **filt_frame_surface)
{
    AiContext *ai_ctx = s->ai_ctx;
    HwCropContext *crop_ctx = s->hwc_ctx;
    uint32_t scaler_format;
    ni_retcode_t retcode;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;

    frame_surface = (niFrameSurface1_t *) ai_ctx->api_dst_frame.data.frame.p_data[3]; //(niFrameSurface1_t *)in->data[3];
    if (frame_surface == NULL) {
        av_log(NULL, AV_LOG_ERROR, "Error: frame_surface is NULL\n");
        return AVERROR(EINVAL);
    }

    pAVHFWCtx = (AVHWFramesContext *)in->hw_frames_ctx->data;

    av_log(ctx, AV_LOG_DEBUG, "%s: in frame surface frameIdx %d sw_format %s w %d h %d\n", __func__,
           frame_surface->ui16FrameIdx, av_get_pix_fmt_name(pAVHFWCtx->sw_format), w, h);

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&crop_ctx->api_dst_frame.data.frame, s->nb_width, s->nb_height, // w, h,
                                          0);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "Error: Cannot allocate memory\n");
        return AVERROR(ENOMEM);
    }

    av_log(ctx, AV_LOG_DEBUG,
           "%s: inlink->w = %d;inlink->h = %d;outlink->w = %d;outlink->h = %d\n", __func__,
           s->nb_width, s->nb_height, w, h);

    av_log(ctx, AV_LOG_DEBUG, "%s: x:%d y:%d x+w:%d y+h:%d\n", __func__,
            0, 0, w, h);

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(&crop_ctx->api_ctx,
                                    FFALIGN(s->nb_width, 2),
                                    FFALIGN(s->nb_height, 2),
                                    scaler_format,
                                    0, // input frame
                                    w, // src rectangle width
                                    h, // src rectangle height
                                    0, // src rectangle x
                                    0, // src rectangle y
                                    frame_surface->ui32nodeAddress,
                                    frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "Error: Can't assign input frame %d\n", retcode);
        return AVERROR(ENOMEM);
    }

    /* Allocate device destination frame This will acquire a frame from the pool */
    retcode = ni_device_alloc_frame(&crop_ctx->api_ctx,
                        FFALIGN(w,2),
                        FFALIGN(h,2),
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
        av_log(NULL, AV_LOG_ERROR, "Error: Can't allocate device output frame %d\n", retcode);
        return AVERROR(ENOMEM);
    }

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(
        &crop_ctx->api_ctx, &crop_ctx->api_dst_frame, NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "%s: Error: Can't allocate device output frame %d\n", __func__, retcode);
        return AVERROR(ENOMEM);
    }

    new_frame_surface =
        (niFrameSurface1_t *)crop_ctx->api_dst_frame.data.frame.p_data[3];

    new_frame_surface->ui16width = w;
    new_frame_surface->ui16height = h;

    *filt_frame_surface = new_frame_surface;

    return 0;
}

static int filter_frame_internal(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    AVHWFramesContext *in_frames_context = NULL; //= (AVHWFramesContext *) in->hw_frames_ctx->data;
    NetIntHvsplusContext *s  = ctx->priv;
    AVFrame *out         = NULL;
    ni_retcode_t retval;
    int ret;
    AiContext *ai_ctx;
    ni_hvsplus_network_t *network = &s->network;
    int nb_planes;
    int64_t start_t;
    int hwframe = in->format == AV_PIX_FMT_NI_QUAD ? 1 : 0;

    av_log(ctx, AV_LOG_DEBUG, "%s: filter %p hwframe %d format %s\n", __func__, s, hwframe, av_get_pix_fmt_name(in->format));

    if (!s->initialized) {
        AVHWFramesContext *pAVHFWCtx;
        if (hwframe) {
            pAVHFWCtx = (AVHWFramesContext *) in->hw_frames_ctx->data;
        }
#if IS_FFMPEG_71_AND_ABOVE
        AVFilterLink *outlink = link->dst->outputs[0];
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           ctx->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        ret = config_input(ctx, in);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "Error: failed to config input\n");
            goto failed_out;
        }
        if (hwframe) {
            av_hwframe_ctx_init(s->out_frames_ref);
            AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
            AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
            ni_cpy_hwframe_ctx(pAVHFWCtx, out_frames_ctx);
            ni_device_session_copy(&s->ai_ctx->api_ctx, &out_ni_ctx->api_ctx);
        }
    }

    ai_ctx = s->ai_ctx;
    out = av_frame_alloc();
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto failed_out;
    }

    av_frame_copy_props(out, in);

    av_log(ctx, AV_LOG_DEBUG, "%s: out_width %d out_height %d in width %d height %d\n",
            __func__, s->out_width, s->out_height, in->width, in->height);

    if (hwframe) {
        niFrameSurface1_t *frame_surface;
        niFrameSurface1_t *hvsplus_surface;
        niFrameSurface1_t *out_surface;
        niFrameSurface1_t *frame_surface2;
        int ai_out_format;
        niFrameSurface1_t dst_surface = {0};

        in_frames_context = (AVHWFramesContext *) in->hw_frames_ctx->data;

        out->width = (s->need_padding) ? in->width : s->nb_width;
        out->height = (s->need_padding) ? in->height : s->nb_height;

        out->format = AV_PIX_FMT_NI_QUAD;

        /* Quadra 2D engine always outputs limited color range */
        out->color_range = AVCOL_RANGE_MPEG;

        if (s->need_padding) {
            ret = ni_hwframe_pad(ctx, s, in, s->nb_width, s->nb_height, //network->netw, network->neth,
                                   &frame_surface);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "Error run hwframe pad\n");
                goto failed_out;
            }

            av_log(ctx, AV_LOG_DEBUG, "filt frame surface frameIdx %d\n",
                    frame_surface->ui16FrameIdx);

            out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        } else {
            // To hvsplus
            frame_surface = (niFrameSurface1_t *)in->data[3];
        }

        out->data[3] = av_malloc(sizeof(niFrameSurface1_t));
        if (!out->data[3]) {
            av_log(ctx, AV_LOG_ERROR, "Error: ni hvsplus filter av_alloc returned NULL\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }

        memcpy(out->data[3], frame_surface, sizeof(niFrameSurface1_t));
        av_log(ctx, AV_LOG_DEBUG, "%s: input frame surface frameIdx %d ui16width %d ui16height %d\n",
               __func__, frame_surface->ui16FrameIdx, frame_surface->ui16width, frame_surface->ui16height);

        start_t = av_gettime();

        /* set output buffer */
        ai_out_format = ff_ni_ffmpeg_to_gc620_pix_fmt(in_frames_context->sw_format);

        av_log(ctx, AV_LOG_DEBUG, "%s: in sw_format %s ai_out_format %d\n", __func__,
               av_get_pix_fmt_name(in_frames_context->sw_format), ai_out_format);

#ifdef NI_MEASURE_LATENCY
        ff_ni_update_benchmark(NULL);
#endif

        do {
            if (s->channel_mode) {
                retval = ni_device_alloc_dst_frame(&(ai_ctx->api_ctx), &dst_surface, NI_DEVICE_TYPE_AI);
            } else {
                if (s->need_padding) {
                    av_log(ctx, AV_LOG_DEBUG, "%s: 1. Set output hw frame in Ai w %d h %d\n",
                            __func__, s->nb_width, s->nb_height);
                    retval = ni_device_alloc_frame(
                        &ai_ctx->api_ctx, FFALIGN(s->nb_width, 2), FFALIGN(s->nb_height,2),
                        ai_out_format, NI_AI_FLAG_IO, 0, 0,
                        0, 0, 0, -1, NI_DEVICE_TYPE_AI);
                } else {
                    av_log(ctx, AV_LOG_DEBUG, "%s: 1. Set output hw frame in Ai w %d h %d\n",
                            __func__, s->out_width, s->out_height);
                    retval = ni_device_alloc_frame(
                        &ai_ctx->api_ctx, FFALIGN(s->out_width, 2), FFALIGN(s->out_height,2),
                        ai_out_format, NI_AI_FLAG_IO, 0, 0,
                        0, 0, 0, -1, NI_DEVICE_TYPE_AI);
                }
            }

            if (retval < NI_RETCODE_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "Error: failed to alloc hw output frame\n");
                ret = AVERROR(ENOMEM);
                goto failed_out;
            }

            if (av_gettime() - start_t > s->ai_timeout * 1000000) {
                av_log(ctx, AV_LOG_ERROR, "Error: alloc hw output timeout\n");
                ret = AVERROR(ENOMEM);
                goto failed_out;
            }
        } while (retval != NI_RETCODE_SUCCESS);

        if (s->channel_mode) {
            // copy input hw frame to dst hw frame
            ni_frameclone_desc_t frame_clone_desc = {0};
            frame_clone_desc.ui16DstIdx = dst_surface.ui16FrameIdx;
            frame_clone_desc.ui16SrcIdx = frame_surface->ui16FrameIdx;
            if (in_frames_context->sw_format == AV_PIX_FMT_YUV420P) {
                // only support yuv420p
                if (s->need_padding) {
                    // offset Y size
                    frame_clone_desc.ui32Offset = NI_VPU_ALIGN128(s->nb_width) * NI_VPU_CEIL(s->nb_height, 2);
                    // copy U+V size
                    frame_clone_desc.ui32Size = NI_VPU_ALIGN128(s->nb_width / 2) * NI_VPU_CEIL(s->nb_height, 2);
                } else {
                    // offset Y size
                    frame_clone_desc.ui32Offset = NI_VPU_ALIGN128(s->out_width) * NI_VPU_CEIL(s->out_height, 2);
                    // copy U+V size
                    frame_clone_desc.ui32Size = NI_VPU_ALIGN128(s->out_width / 2) * NI_VPU_CEIL(s->out_height, 2);
                }
                retval = ni_device_clone_hwframe(&ai_ctx->api_ctx, &frame_clone_desc);
                if (retval != NI_RETCODE_SUCCESS) {
                    av_log(ctx, AV_LOG_ERROR, "Error: failed to clone hw input frame\n");
                    ret = AVERROR(ENOMEM);
                    goto failed_out;
                }
            } else {
                av_log(ctx, AV_LOG_ERROR, "Error: support yuv420p only, current fmt %d\n",
                    in_frames_context->sw_format);
                ret = AVERROR(EINVAL);
                goto failed_out;
            }
        }

        av_log(ctx, AV_LOG_DEBUG, "%s: 2. Set input hw frame in Ai w %d h %d\n",
                                __func__, frame_surface->ui16width, frame_surface->ui16height);

        /* set input buffer */
        retval = ni_device_alloc_frame(&ai_ctx->api_ctx, 0, 0, 0, 0, 0, 0, 0, 0,
                                    frame_surface->ui32nodeAddress,
                                    frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_AI);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Error: failed to alloc hw input frame\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }

        /* Set the new frame index */
        start_t = av_gettime();
        do{
            av_log(ctx, AV_LOG_DEBUG, "%s: 3. Read hw frame from Ai w %d h %d\n",
                                            __func__, out->width, out->height);
            retval = ni_device_session_read_hwdesc(
                &ai_ctx->api_ctx, &s->ai_ctx->api_dst_frame, NI_DEVICE_TYPE_AI);

            if (retval < NI_RETCODE_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "Error: failed to read hwdesc,retval=%d\n", retval);
                ret = AVERROR(EINVAL);
                goto failed_out;
            }
            if (av_gettime() - start_t > s->ai_timeout * 1000000) {
                av_log(ctx, AV_LOG_ERROR, "Error: alloc hw output timeout\n");
                ret = AVERROR(ENOMEM);
                goto failed_out;
            }
        } while (retval != NI_RETCODE_SUCCESS);

#ifdef NI_MEASURE_LATENCY
        ff_ni_update_benchmark("ni_quadra_hvsplus");
#endif

        if (s->need_padding) {

            hvsplus_surface = (niFrameSurface1_t *) ai_ctx->api_dst_frame.data.frame.p_data[3];

            ni_hwframe_buffer_recycle(frame_surface, frame_surface->device_handle);

            out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);

            memcpy(out->data[3], ai_ctx->api_dst_frame.data.frame.p_data[3], sizeof(niFrameSurface1_t));

            ret = ni_hwframe_crop(ctx, s, in, in->width, in->height, &frame_surface2);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "Error run hwframe crop\n");
                goto failed_out;
            }

            ni_hwframe_buffer_recycle(hvsplus_surface, hvsplus_surface->device_handle);

            av_log(ctx, AV_LOG_DEBUG, "filt frame surface frameIdx %d\n",
                    frame_surface2->ui16FrameIdx);
        } else {
            frame_surface2 = (niFrameSurface1_t *) ai_ctx->api_dst_frame.data.frame.p_data[3];
        }

        out_surface = (niFrameSurface1_t *) out->data[3];

        av_log(ctx, AV_LOG_DEBUG,"ai pre process, idx=%d\n", frame_surface2->ui16FrameIdx);

        out_surface->ui16FrameIdx = frame_surface2->ui16FrameIdx;
        out_surface->ui16session_ID = frame_surface2->ui16session_ID;
        out_surface->device_handle = frame_surface2->device_handle;
        out_surface->output_idx = frame_surface2->output_idx;
        out_surface->src_cpu = frame_surface2->src_cpu;
        out_surface->ui32nodeAddress = 0;
        out_surface->dma_buf_fd = 0;
        out_surface->ui16width = out->width;
        out_surface->ui16height = out->height;
        ff_ni_set_bit_depth_and_encoding_type(&out_surface->bit_depth,
                                            &out_surface->encoding_type,
                                            in_frames_context->sw_format);

        av_log(ctx, AV_LOG_DEBUG, "%s: need_padding %d 4. Read hw frame from Ai w %d %d h %d %d\n",
                                                        __func__, s->need_padding, out->width, s->out_width, out->height, s->out_height);

        out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t), ff_ni_frame_free, NULL, 0);

        if (!out->buf[0]) {
            av_log(ctx, AV_LOG_ERROR, "Error: ni hvsplus filter av_buffer_create returned NULL\n");
            ret = AVERROR(ENOMEM);
            av_log(NULL, AV_LOG_DEBUG, "Recycle trace ui16FrameIdx = [%d] DevHandle %d\n",
                    out_surface->ui16FrameIdx, out_surface->device_handle);
            retval = ni_hwframe_buffer_recycle(out_surface, out_surface->device_handle);
            if (retval != NI_RETCODE_SUCCESS) {
                av_log(NULL, AV_LOG_ERROR, "ERROR: Failed to recycle trace ui16FrameIdx = [%d] DevHandle %d\n",
                        out_surface->ui16FrameIdx, out_surface->device_handle);
            }
            goto failed_out;
        }

        /* Reference the new hw frames context */
        out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    } else {
        out->width = s->out_width;
        out->height = s->out_height;

        out->format = in->format;
        av_log(ctx, AV_LOG_DEBUG, "%s: format %s allocate frame %d x %d\n", __func__, av_get_pix_fmt_name(in->format), out->width, out->height);
        if (av_frame_get_buffer(out, 32) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error: Could not allocate the AVFrame buffers\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }

        start_t = av_gettime();
        // sw frame: step 1: allocate
        retval = ni_ai_frame_buffer_alloc(&ai_ctx->api_src_frame.data.frame,
                                          &network->raw);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Error: cannot allocate ai frame\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }
        nb_planes = av_pix_fmt_count_planes(in->format);
        if (s->channel_mode) {
            if (in->format != AV_PIX_FMT_YUV420P && in->format != AV_PIX_FMT_YUVJ420P) {
                av_log(ctx, AV_LOG_ERROR, "Error: support yuv420p and yuvj420p only, current fmt %d\n",
                        in->format);
                ret = AVERROR(EINVAL);
                goto failed_out;
            }
            nb_planes = 1; // only copy Y data
        }
        // sw frame: step 2: pad and setup frame
        retval = av_to_niframe_copy(s, &ai_ctx->api_src_frame.data.frame, in, nb_planes);
        if (retval < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error: hvsplus cannot copy frame\n");
            ret = AVERROR(EIO);
            goto failed_out;
        }

#ifdef NI_MEASURE_LATENCY
        ff_ni_update_benchmark(NULL);
#endif

        /* write frame */
        // sw frame: step 3: write a frame to AI
        do {
            retval = ni_device_session_write(
                &ai_ctx->api_ctx, &ai_ctx->api_src_frame, NI_DEVICE_TYPE_AI);
            if (retval < 0) {
                av_log(ctx, AV_LOG_ERROR,
                       "Error: failed to write ai session: retval %d\n", retval);
                ret = AVERROR(EIO);
                goto failed_out;
            }

            if (av_gettime() - start_t > s->ai_timeout * 1000000) {
                av_log(ctx, AV_LOG_ERROR, "Error: write sw frame to AI timeout\n");
                ret = AVERROR(ENOMEM);
                goto failed_out;
            }
        } while (retval == 0);
        // sw frame: step 4: alloc frame for read
        retval = ni_ai_packet_buffer_alloc(&ai_ctx->api_dst_frame.data.packet,
                                        &network->raw);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Error: failed to allocate ni packet\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }

        start_t = av_gettime();
        // sw frame: step 5: read a frame from AI
        do {
            retval = ni_device_session_read(&ai_ctx->api_ctx, &ai_ctx->api_dst_frame, NI_DEVICE_TYPE_AI);
            if (retval < 0) {
                av_log(NULL,AV_LOG_ERROR,"Error: read AI data retval %d\n", retval);
                ret = AVERROR(EIO);
                goto failed_out;
            } else if (retval > 0) {
                if (av_gettime() - start_t > s->ai_timeout * 1000000) {
                    av_log(ctx, AV_LOG_ERROR, "Error: read sw frame from AI timeout\n");
                    ret = AVERROR(ENOMEM);
                    goto failed_out;
                }
            }
        } while (retval == 0);
#ifdef NI_MEASURE_LATENCY
        ff_ni_update_benchmark("ni_quadra_hvsplus");
#endif
        nb_planes = av_pix_fmt_count_planes(out->format);
        if (s->channel_mode) {
            if (out->format != AV_PIX_FMT_YUV420P && out->format != AV_PIX_FMT_YUVJ420P) {
                av_log(ctx, AV_LOG_ERROR, "Error: support yuv420p and yuvj420p only, current fmt %d\n",
                        out->format);
                ret = AVERROR(EINVAL);
                goto failed_out;
            }
            nb_planes = 1; // only copy Y data
            // copy U/V data from the input sw frame
            memcpy(out->data[1], in->data[1], in->height * in->linesize[1] / 2);
            memcpy(out->data[2], in->data[2], in->height * in->linesize[2] / 2);
        }
        // sw frame: step 6: crop
        retval = ni_to_avframe_copy(s, out, &ai_ctx->api_dst_frame.data.packet, nb_planes);
        if (retval < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error: hvsplus cannot copy ai frame to avframe\n");
            ret = AVERROR(EIO);
            goto failed_out;
        }
    }

    av_frame_free(&in);
    return ff_filter_frame(link->dst->outputs[0], out);

failed_out:
    if (out)
        av_frame_free(&out);

    av_frame_free(&in);
    return ret;
}

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    int ret;

    if (in == NULL) {
        av_log(ctx, AV_LOG_ERROR, "Error: in frame is null\n");
        return AVERROR(EINVAL);
    }

    ret = filter_frame_internal(link, in);

    return ret;
}

#define OFFSET(x) offsetof(NetIntHvsplusContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_hvsplus_options[] = {
    { "devid",   "device to operate in swframe mode", OFFSET(devid),        AV_OPT_TYPE_INT, {.i64 = 0},  -1, INT_MAX,                  FLAGS },
    { "level",   "level of modification",             OFFSET(level),        AV_OPT_TYPE_INT, {.i64 = 2},  1,  2,                        FLAGS },
    { "width",   "Specify the output frame width.",   OFFSET(out_width),    AV_OPT_TYPE_INT, {.i64 = -1}, -1, NI_MAX_RESOLUTION_WIDTH,  FLAGS },
    { "height",  "Specify the output frame height.",  OFFSET(out_height),   AV_OPT_TYPE_INT, {.i64 = -1}, -1, NI_MAX_RESOLUTION_HEIGHT, FLAGS },
    { "mode",    "filter mode",                       OFFSET(channel_mode), AV_OPT_TYPE_INT, {.i64 = 0},  0,  1,                        FLAGS, "mode" },
        { "YUV",    "process channels Y, U, and V", 0, AV_OPT_TYPE_CONST, {.i64 = 0}, 0, 0, FLAGS, "mode" },
        { "Y_only", "process only channel Y",       0, AV_OPT_TYPE_CONST, {.i64 = 1}, 0, 0, FLAGS, "mode" },
    { "timeout", "Timeout for AI operations",         OFFSET(ai_timeout),   AV_OPT_TYPE_INT,  {.i64 = NI_DEFAULT_KEEP_ALIVE_TIMEOUT}, NI_MIN_KEEP_ALIVE_TIMEOUT, NI_MAX_KEEP_ALIVE_TIMEOUT, FLAGS, "AI_timeout" },
    NI_FILT_OPTION_KEEPALIVE10,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_hvsplus);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        //.config_props = config_input,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = output_config_props,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

AVFilter ff_vf_hvsplus_ni_quadra = {
    .name           = "ni_quadra_hvsplus",
    .description    = NULL_IF_CONFIG_SMALL("NETINT Quadra hvsplus v" NI_XCODER_REVISION),
    .init           = init,
    .uninit         = uninit,
    .priv_size      = sizeof(NetIntHvsplusContext),
    .priv_class     = &ni_hvsplus_class,
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
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
