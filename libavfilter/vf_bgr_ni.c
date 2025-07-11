/*
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

#include "libavutil/buffer.h"
#include "libavutil/hwcontext.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "libswscale/swscale.h"

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
#include "ni_device_api.h"
#include "ni_util.h"
#include "video.h"

#include "libavutil/avassert.h"

#if HAVE_IO_H
#define ACCESS(A,B)  _access(A,B)
#else
#define ACCESS(A,B)  access(A,B)
#endif


// used for OpenImage
#include <libavcodec/avcodec.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavformat/avformat.h>
#include <libavutil/bprint.h>
#include <libavutil/pixfmt.h>
#include <libavutil/time.h>
#include <libavutil/timecode.h>
#include <stdlib.h>

typedef struct _ni_roi_network_layer {
    int32_t width;
    int32_t height;
    int32_t channel;
    int32_t classes;
    int32_t component;
    int32_t output_number;
    float *output;
} ni_roi_network_layer_t;

typedef struct _ni_roi_network {
    int32_t netw;
    int32_t neth;
    ni_network_data_t raw;
    ni_roi_network_layer_t *layers;
} ni_roi_network_t;

typedef struct HwScaleContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
} HwScaleContext;

typedef struct AiContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_src_frame;
    ni_session_data_io_t api_dst_pkt;
} AiContext;

typedef struct HwFormatContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
} HwFormatContext;

typedef struct NetIntBgrContext {
    const AVClass *class;

    AVBufferRef *hwdevice;
    AVBufferRef *hwframe;

    AVBufferRef *hw_frames_ctx;

    /* roi */
    AVBufferRef *out_frames_ref;

    /* ai */
    int initialized;
    const char *nb_file; /* path to network binary */

    AiContext *ai_ctx;
    ni_roi_network_t network;
    HwScaleContext *hws_ctx;

    /* format conversion using 2D */
    HwFormatContext *format_ctx;
    /* bg */
    uint8_t *mask_data;
    int bg_frame_size;
    // AVFrame *bg_frame;
    AVFrame *alpha_enlarge_frame;

    //ni_frame_t *p_dl_frame;
    ni_session_data_io_t p_dl_frame;
    uint8_t aui8SmallMask[256 * 144];
    bool skipInference;
    int framecount;
    int skip;
    int skip_random_offset;
    int keep_alive_timeout; /* keep alive timeout setting */
    int buffer_limit;
} NetIntBgrContext;

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

static void cleanup_ai_context(AVFilterContext *ctx, NetIntBgrContext *s)
{
    ni_retcode_t retval;
    AiContext *ai_ctx = s->ai_ctx;

    if (ai_ctx) {
        ni_frame_buffer_free(&ai_ctx->api_src_frame.data.frame);
        ni_packet_buffer_free(&ai_ctx->api_dst_pkt.data.packet);

        retval =
            ni_device_session_close(&ai_ctx->api_ctx, 1, NI_DEVICE_TYPE_AI);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "%s: failed to close ai session. retval %d\n", __func__,
                   retval);
        }
        ni_device_session_context_clear(&ai_ctx->api_ctx);
        av_free(ai_ctx);
        s->ai_ctx = NULL;
    }
}

static int init_ai_context(AVFilterContext *ctx, NetIntBgrContext *s,
                           AVFrame *frame)
{
    ni_retcode_t retval;
    AiContext *ai_ctx;
    ni_roi_network_t *network = &s->network;
    int ret;
    int hwframe = frame->format == AV_PIX_FMT_NI_QUAD ? 1 : 0;

    if ((s->nb_file == NULL) || (ACCESS(s->nb_file, R_OK) != 0)) {
        av_log(ctx, AV_LOG_ERROR, "invalid network binary path\n");
        return AVERROR(EINVAL);
    }

    ai_ctx = av_mallocz(sizeof(AiContext));
    if (!ai_ctx) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate ai context\n");
        return AVERROR(ENOMEM);
    }
    s->ai_ctx = ai_ctx;
    ni_device_session_context_init(&ai_ctx->api_ctx);
    if (hwframe) {
        AVHWFramesContext *pAVHFWCtx;
        AVNIDeviceContext *pAVNIDevCtx;
        int cardno;

        pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
        pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
        cardno      = ni_get_cardno(frame);

        ai_ctx->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        ai_ctx->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];
        ai_ctx->api_ctx.hw_action     = NI_CODEC_HW_ENABLE;
        ai_ctx->api_ctx.hw_id         = cardno;
    }

    ai_ctx->api_ctx.device_type = NI_DEVICE_TYPE_AI;
    ai_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    retval = ni_device_session_open(&ai_ctx->api_ctx, NI_DEVICE_TYPE_AI);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to open ai session. retval %d\n",
               retval);
        ret = AVERROR(EIO);
        goto failed_out;
    }

    retval = ni_ai_config_network_binary(&ai_ctx->api_ctx, &network->raw,
                                         s->nb_file);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to configure ai session. retval %d\n",
               retval);
        ret = AVERROR(EIO);
        goto failed_out;
    }

    if (!hwframe) {
        retval = ni_ai_frame_buffer_alloc(&ai_ctx->api_src_frame.data.frame,
                                          &network->raw);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "failed to allocate ni frame\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }
    }

    retval = ni_ai_packet_buffer_alloc(&ai_ctx->api_dst_pkt.data.packet,
                                       &network->raw);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate ni packet\n");
        ret = AVERROR(ENOMEM);
        goto failed_out;
    }

    return 0;

failed_out:
    cleanup_ai_context(ctx, s);
    return ret;
}

static void ni_destroy_network(AVFilterContext *ctx,
                               ni_roi_network_t *network)
{
    if (network) {
        int i;

        for (i = 0; i < network->raw.output_num; i++) {
            if (network->layers && network->layers[i].output) {
                free(network->layers[i].output);
                network->layers[i].output = NULL;
            }
        }

        free(network->layers);
        network->layers = NULL;
    }
}

static int ni_create_network(AVFilterContext *ctx, ni_roi_network_t *network)
{
    int ret;
    int i;
    ni_network_data_t *ni_network = &network->raw;

    av_log(ctx, AV_LOG_VERBOSE, "network input number %d, output number %d\n",
           ni_network->input_num, ni_network->output_num);

    if (ni_network->input_num == 0 || ni_network->output_num == 0) {
        av_log(ctx, AV_LOG_ERROR, "invalid network layer\n");
        return AVERROR(EINVAL);
    }

    /* only support one input for now */
    if (ni_network->input_num != 1) {
        av_log(ctx, AV_LOG_ERROR,
               "network input layer number %d not supported\n",
               ni_network->input_num);
        return AVERROR(EINVAL);
    }

    /*
     * create network and its layers. i don't know whether this is platform
     * specific or not. maybe i shall add a create network api to do this.
     */
    network->layers =
        malloc(sizeof(ni_roi_network_layer_t) * ni_network->output_num);
    if (!network->layers) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate network layer memory\n");
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < ni_network->output_num; i++) {
        network->layers[i].width     = ni_network->linfo.out_param[i].sizes[0];
        network->layers[i].height    = ni_network->linfo.out_param[i].sizes[1];
        network->layers[i].channel   = ni_network->linfo.out_param[i].sizes[2];
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
                   "failed to allocate network layer %d output buffer\n", i);
            ret = AVERROR(ENOMEM);
            goto out;
        }

        av_log(ctx, AV_LOG_DEBUG,
               "network layer %d: w %d, h %d, ch %d, co %d, cl %d\n", i,
               network->layers[i].width, network->layers[i].height,
               network->layers[i].channel, network->layers[i].component,
               network->layers[i].classes);
    }

    network->netw = ni_network->linfo.in_param[0].sizes[0];
    network->neth = ni_network->linfo.in_param[0].sizes[1];

    return 0;
out:
    ni_destroy_network(ctx, network);
    return ret;
}

static av_cold int init_hwframe_scale(AVFilterContext *ctx, NetIntBgrContext *s,
                                      enum AVPixelFormat format, AVFrame *frame)
{
    ni_retcode_t retval;
    HwScaleContext *hws_ctx;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    int cardno;

    hws_ctx = av_mallocz(sizeof(HwScaleContext));
    if (!hws_ctx) {
        av_log(ctx, AV_LOG_ERROR, "could not allocate hwframe ctx\n");
        return AVERROR(ENOMEM);
    }
    s->hws_ctx = hws_ctx;
    ni_device_session_context_init(&hws_ctx->api_ctx);

    pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno      = ni_get_cardno(frame);

    hws_ctx->api_ctx.device_handle     = pAVNIDevCtx->cards[cardno];
    hws_ctx->api_ctx.blk_io_handle     = pAVNIDevCtx->cards[cardno];
    hws_ctx->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
    hws_ctx->api_ctx.scaler_operation  = NI_SCALER_OPCODE_SCALE;
    hws_ctx->api_ctx.hw_id             = cardno;
    hws_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    retval = ni_device_session_open(&hws_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "could not open scaler session\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&hws_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hws_ctx->api_ctx);
        goto out;
    }

#if IS_FFMPEG_61_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create scale frame pool on device */
    retval = ff_ni_build_frame_pool(&hws_ctx->api_ctx, s->network.netw,
                                    s->network.neth, format,
                                    DEFAULT_NI_FILTER_POOL_SIZE,
                                    s->buffer_limit);
    if (retval < 0) {
        av_log(ctx, AV_LOG_ERROR, "could not build frame pool\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&hws_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hws_ctx->api_ctx);
        goto out;
    }

    return 0;
out:
    av_free(hws_ctx);
    return ret;
}

static void cleanup_hwframe_scale(AVFilterContext *ctx, NetIntBgrContext *s)
{
    HwScaleContext *hws_ctx = s->hws_ctx;

    if (hws_ctx) {
        ni_frame_buffer_free(&hws_ctx->api_dst_frame.data.frame);
        ni_device_session_close(&hws_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hws_ctx->api_ctx);
        av_free(hws_ctx);
        s->hws_ctx = NULL;
    }
}

static av_cold int init_hwframe_format(AVFilterContext *ctx, NetIntBgrContext *s,
                                       AVFrame *frame)
{
    ni_retcode_t retval;
    HwFormatContext *format_ctx;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    int cardno;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    format_ctx = av_mallocz(sizeof(HwScaleContext));
    if (!format_ctx) {
        av_log(ctx, AV_LOG_ERROR,
               "Could not allocate hwframe ctx for format conversion\n");
        return AVERROR(ENOMEM);
    }
    s->format_ctx = format_ctx;
    ni_device_session_context_init(&format_ctx->api_ctx);

    pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno      = ni_get_cardno(frame);

    format_ctx->api_ctx.device_handle      = pAVNIDevCtx->cards[cardno];
    format_ctx->api_ctx.blk_io_handle      = pAVNIDevCtx->cards[cardno];
    format_ctx->api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
    format_ctx->api_ctx.scaler_operation   = NI_SCALER_OPCODE_SCALE;
    format_ctx->api_ctx.hw_id              = cardno;
    format_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    retval =
        ni_device_session_open(&format_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR,
               "Could not open scaler session for format conversion\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&format_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&format_ctx->api_ctx);
        goto out;
    }
#if IS_FFMPEG_71_AND_ABOVE
    pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
#endif

#if IS_FFMPEG_70_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create frame pool on device for format conversion through 2D engine.
     * It must be in P2P so that DSP can have access to it. */
    retval = ff_ni_build_frame_pool(&format_ctx->api_ctx, frame->width,
                                        frame->height, AV_PIX_FMT_RGBA,
                                        pool_size,
                                        s->buffer_limit);
    if (retval < 0) {
        av_log(ctx, AV_LOG_ERROR, "could not build frame pool\n");
        ret = AVERROR(EIO);
        ni_device_session_close(&format_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&format_ctx->api_ctx);
        goto out;
    }

    return 0;
out:
    av_free(format_ctx);
    return ret;
}

static void cleanup_hwframe_format(AVFilterContext *ctx, NetIntBgrContext *s)
{
    HwFormatContext *fmt_ctx = s->format_ctx;

    if (fmt_ctx) {
        ni_frame_buffer_free(&fmt_ctx->api_dst_frame.data.frame);
        ni_device_session_close(&fmt_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&fmt_ctx->api_ctx);
        av_free(fmt_ctx);
        s->format_ctx = NULL;
    }
}

static int init_hwframe_uploader(AVFilterContext *ctx, NetIntBgrContext *s,
                                 AVFrame *frame)
{
    int ret;
    AVHWFramesContext *hwframe_ctx;
    AVHWFramesContext *out_frames_ctx;
    AVNIFramesContext *ni_ctx;
    AVNIFramesContext *ni_ctx_output;
    int cardno   = ni_get_cardno(frame);
    char buf[64] = {0};

    snprintf(buf, sizeof(buf), "%d", cardno);

    ret = av_hwdevice_ctx_create(&s->hwdevice, AV_HWDEVICE_TYPE_NI_QUADRA, buf,
                                 NULL, 0);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to create AV HW device ctx\n");
        return ret;
    }

    s->hwframe = av_hwframe_ctx_alloc(s->hwdevice);
    if (!s->hwframe)
        return AVERROR(ENOMEM);

    hwframe_ctx            = (AVHWFramesContext *)s->hwframe->data;
    hwframe_ctx->format    = AV_PIX_FMT_NI_QUAD;
    hwframe_ctx->sw_format = AV_PIX_FMT_RGBA;
    hwframe_ctx->width     = ctx->inputs[0]->w;
    hwframe_ctx->height    = ctx->inputs[0]->h;

    ret = av_hwframe_ctx_init(s->hwframe);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to init AV HW device ctx\n");
        return ret;
    }

    // Work around a hwdownload session start timestamp issue
    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
    ni_ctx         = (AVNIFramesContext*) hwframe_ctx->hwctx;
    ni_ctx_output  = (AVNIFramesContext*) out_frames_ctx->hwctx;
    ni_ctx_output->api_ctx.session_timestamp =
        ni_ctx->api_ctx.session_timestamp;

    s->hw_frames_ctx = av_buffer_ref(s->hwframe);
    if (!s->hw_frames_ctx)
        return AVERROR(ENOMEM);

    return 0;
}

static int init_out_hwframe_ctx(AVFilterContext *ctx, AVHWFramesContext *in_frames_ctx)
{
    NetIntBgrContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];
    AVHWFramesContext *out_frames_ctx;
    AVNIFramesContext *out_ni_ctx;

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
        av_log(ctx, AV_LOG_ERROR, "bgrp not supported\n");
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

    out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
    ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx);
    ni_device_session_copy(&s->ai_ctx->api_ctx, &out_ni_ctx->api_ctx);

    out_frames_ctx->format            = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width             = inlink->w;
    out_frames_ctx->height            = inlink->h;
    out_frames_ctx->sw_format         = AV_PIX_FMT_RGBA;
    out_frames_ctx->initial_pool_size = NI_BGR_ID;

    av_hwframe_ctx_init(s->out_frames_ref);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntBgrContext *s            = ctx->priv;
    ni_roi_network_t *network = &s->network;

    av_buffer_unref(&s->hwframe);
    av_buffer_unref(&s->hwdevice);

    av_buffer_unref(&s->hw_frames_ctx);

    /* roi */
    av_buffer_unref(&s->out_frames_ref);

    /* ai */
    cleanup_ai_context(ctx, s);
    ni_destroy_network(ctx, network);

    /* bg */
    av_frame_free(&s->alpha_enlarge_frame);
    ni_frame_buffer_free(&s->p_dl_frame.data.frame);

    cleanup_hwframe_scale(ctx, s);
    cleanup_hwframe_format(ctx, s);

    free(s->mask_data);
}

static int ni_bgr_config_input(AVFilterContext *ctx, AVFrame *frame)
{
    NetIntBgrContext *s = ctx->priv;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    const AVPixFmtDescriptor *desc;

    if (s->initialized)
        return 0;

    pAVHFWCtx = (AVHWFramesContext *) frame->hw_frames_ctx->data;
    desc = av_pix_fmt_desc_get(pAVHFWCtx->sw_format);

    if ((frame->color_range == AVCOL_RANGE_JPEG) && !(desc->flags & AV_PIX_FMT_FLAG_RGB)) {
        av_log(ctx, AV_LOG_WARNING,
               "WARNING: Full color range input, limited color range output\n");
    }

#if !IS_FFMPEG_342_AND_ABOVE
    init_out_hwframe_ctx(ctx, (AVHWFramesContext *)frame->hw_frames_ctx->data);
#endif

    ret = init_hwframe_uploader(ctx, s, frame);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to initialize uploader session\n");
        return ret;
    }

    ret = init_ai_context(ctx, s, frame);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to initialize ai context\n");
        return ret;
    }

    ret = ni_create_network(ctx, &s->network);
    if (ret != 0) {
        goto fail_out;
    }

    ret = init_hwframe_scale(ctx, s, AV_PIX_FMT_BGRP, frame); // AV_PIX_FMT_RGBA
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "could not initialize hwframe scale context\n");
        goto fail_out;
    }

    /* Allocate a frame pool of type AV_PIX_FMT_RGBA for converting
     * the input frame to RGBA using 2D engine */
    ret = init_hwframe_format(ctx, s, frame);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "Could not initialize hwframe context for format conversion\n");
        goto fail_out;
    }

    s->mask_data = malloc(s->network.netw * s->network.neth * sizeof(uint8_t));

    if (!s->mask_data) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate sctx->mask_datamemory\n");
        return AVERROR(ENOMEM);
    }

    return 0;

fail_out:
    cleanup_ai_context(ctx, s);

    ni_destroy_network(ctx, &s->network);

    return ret;
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *outlink)
#else
static int config_output(AVFilterLink *outlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    NetIntBgrContext *s      = ctx->priv;
    int linesize[4];

    av_log(ctx, AV_LOG_DEBUG, "%s\n", __func__);

    av_buffer_unref(&s->hwframe);
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
#elif IS_FFMPEG_342_AND_ABOVE
    if (inlink->hw_frames_ctx == NULL) {
#else
    if (in->hw_frames_ctx == NULL) {
#endif
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_DEBUG, "inlink wxh %dx%d\n", inlink->w, inlink->h);

    /* roi */
    outlink->w = inlink->w;
    outlink->h = inlink->h;

    //prep download/upload buffer
    linesize[0] = FFALIGN(inlink->w, 16) * 4;
    linesize[1] = linesize[2] = linesize[3] = 0;
    if (ni_frame_buffer_alloc_pixfmt(&s->p_dl_frame.data.frame,
                                     NI_PIX_FMT_RGBA, inlink->w,
                                     inlink->h, linesize, 2 /* alignment*/,
                                     0 /*extra len*/)) {
        return AVERROR(ENOMEM);
    }
    s->alpha_enlarge_frame = av_frame_alloc();

    s->alpha_enlarge_frame->data[0] = s->p_dl_frame.data.frame.p_data[0];
    s->alpha_enlarge_frame->linesize[0] = FFALIGN(inlink->w, 16) * 4;
    s->alpha_enlarge_frame->width        = inlink->w;
    s->alpha_enlarge_frame->height       = inlink->h;

    av_log(ctx, AV_LOG_DEBUG, "outlink wxh %dx%d\n", outlink->w, outlink->h);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(outlink);
    init_out_hwframe_ctx(ctx, (AVHWFramesContext *)li->hw_frames_ctx->data);
    av_buffer_unref(&lo->hw_frames_ctx);
    lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
#if IS_FFMPEG_342_AND_ABOVE
    init_out_hwframe_ctx(ctx, (AVHWFramesContext *)inlink->hw_frames_ctx->data);
#endif

    av_buffer_unref(&outlink->hw_frames_ctx);

    outlink->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!outlink->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif

    return 0;
}

static int ni_get_mask2(AVFilterContext *ctx, ni_session_data_io_t *p_raw_mask_data)
{
    NetIntBgrContext *s = ctx->priv;
    uint8_t Y_MIN = 255;
    uint8_t Y_MAX = 0;
    uint8_t *p_mask_raw_start     = p_raw_mask_data->data.packet.p_data;

    int loop_length = 256 * 144;
    uint8_t *p_mask_raw = p_mask_raw_start;
    uint8_t ui8AiTensorL;
    uint8_t ui8AiTensorR;
    uint8_t *aui8SmallMask = s->aui8SmallMask;
    for (int i = 0; i < loop_length;i++) {
        ui8AiTensorL = *(p_mask_raw + 1);
        ui8AiTensorR = *p_mask_raw;

        if (ui8AiTensorL < ui8AiTensorR) {
            aui8SmallMask[i] = Y_MAX;
        } else {
            aui8SmallMask[i] = Y_MIN;
        }
        p_mask_raw+=2;
    }
    for (int i = 0; i < 30; i++) {
        av_log(ctx, AV_LOG_TRACE, "mask[%d] 0x%x\n",i, aui8SmallMask[i]);
    }
    return 0;
}

static int ni_upscale_alpha2RGBMTA(AVFilterContext *ctx)
{
    NetIntBgrContext *s = ctx->priv;
    uint8_t *aui8SmallMask = s->aui8SmallMask;
    uint8_t *p_rgbmta     = s->p_dl_frame.data.frame.p_data[0]; //rgbargbargba format
    uint32_t destWidth     = s->alpha_enlarge_frame->width;
    uint32_t destHeight    = s->alpha_enlarge_frame->height;

    uint16_t src_x, src_y;
    uint32_t dst_w_aligned = FFALIGN(destWidth, 16);
#ifdef ALPHAONLY
    for (uint32_t dst_y = 0; dst_y < destHeight; dst_y++) {
        uint8_t *p8_dst = p_rgbmta + (dst_y * dst_w_aligned * sizeof(uint32_t)) + 3; //3rd byte
        src_y = (uint16_t)floorf((((float)dst_y) / ((float)destHeight)) * ((float)144));
        for (uint32_t dst_x = 0; dst_x < destWidth; dst_x++) {
            src_x = (uint16_t)floorf((((float)dst_x) / ((float)destWidth)) *
            ((float) 256));

            *p8_dst     = aui8SmallMask[src_y * 256 + src_x];
            p8_dst += 4;
        }
    }
#else
    for (uint32_t dst_y = 0; dst_y < destHeight; dst_y++) {
        uint32_t *p32_dst = (uint32_t *)(p_rgbmta + dst_y * dst_w_aligned * sizeof(uint32_t));
        src_y = (uint16_t)floorf((((float)dst_y) / ((float)destHeight)) * ((float)144));
        for (uint32_t dst_x = 0; dst_x < destWidth; dst_x++) {
            src_x = (uint16_t)floorf((((float)dst_x) / ((float)destWidth)) * ((float)256));
            //need to read write
            if (aui8SmallMask[src_y * 256 + src_x]) {
            //do nothing
            } else {
                *p32_dst = 0;
            }
            p32_dst += 1;
        }
    }
#endif
    return 0;
}

static int ni_hwframe_scale(AVFilterContext *ctx, NetIntBgrContext *s, AVFrame *in,
                            int w, int h,
                            niFrameSurface1_t ** downscale_bgr_surface)
{
    HwScaleContext *scale_ctx = s->hws_ctx;
    int scaler_format;
    ni_retcode_t retcode;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;

    frame_surface = (niFrameSurface1_t *)in->data[3];

    av_log(ctx, AV_LOG_DEBUG, "in frame surface frameIdx %d\n",
           frame_surface->ui16FrameIdx);

    pAVHFWCtx = (AVHWFramesContext *)in->hw_frames_ctx->data;

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&scale_ctx->api_dst_frame.data.frame,
                                          w, h, 0);
    if (retcode != NI_RETCODE_SUCCESS)
        return AVERROR(ENOMEM);

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(
        &scale_ctx->api_ctx, FFALIGN(in->width, 2), FFALIGN(in->height, 2),
        scaler_format, 0, 0, 0, 0, 0, frame_surface->ui32nodeAddress,
        frame_surface->ui16FrameIdx, NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate device input frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Allocate hardware device destination frame. This acquires a frame from
     * the pool */
    retcode = ni_device_alloc_frame(
        &scale_ctx->api_ctx, FFALIGN(w, 2), FFALIGN(h, 2),
        ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_BGRP), NI_SCALER_FLAG_IO, 0, 0,
        0, 0, 0, -1, NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate device output frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Set the new frame index */
    ni_device_session_read_hwdesc(
        &scale_ctx->api_ctx, &scale_ctx->api_dst_frame, NI_DEVICE_TYPE_SCALER);
    new_frame_surface =
        (niFrameSurface1_t *)scale_ctx->api_dst_frame.data.frame.p_data[3];

    *downscale_bgr_surface = new_frame_surface;

    return 0;
}

static int ni_bgr_process(AVFilterContext *ctx, ni_session_data_io_t *p_dst_pkt)
{
    NetIntBgrContext *s = ctx->priv;
    int ret;

    //aui8SmallMask
    if (!s->skipInference) {
        ni_get_mask2(ctx, p_dst_pkt);
    }

    ret = ni_upscale_alpha2RGBMTA(ctx);

    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to get mask data.\n");
        return ret;
    }

    if (ret == 0) {
        av_log(ctx, AV_LOG_DEBUG,
               "the s->alpha_enlarge_frame->width: %d "
               "s->alpha_enlarge_frame->height: %d "
               "s->alpha_enlarge_frame->format: %d "
               "s->alpha_enlarge_frame->linesize[0]: %d \n",
               s->alpha_enlarge_frame->width, s->alpha_enlarge_frame->height,
               s->alpha_enlarge_frame->format,
               s->alpha_enlarge_frame->linesize[0]);
    } else {
        av_log(ctx, AV_LOG_ERROR, "failed to s->alpha_enlarge_frame\n");
        return ret;
    }

    return 0;
}

static int ni_hwframe_convert_format(AVFilterContext *ctx, NetIntBgrContext *s,
                                     AVFrame *in,
                                     niFrameSurface1_t **rgba_frame_surface) {
    HwFormatContext *format_ctx = s->format_ctx;
    ni_retcode_t retcode        = NI_RETCODE_SUCCESS;
    int input_format;
    int output_format = ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA);
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;

    frame_surface = (niFrameSurface1_t *)in->data[3];

    av_log(ctx, AV_LOG_DEBUG,
           "Input surface frame index for format conversion %d\n",
           frame_surface->ui16FrameIdx);

    pAVHFWCtx = (AVHWFramesContext *)in->hw_frames_ctx->data;

    input_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&format_ctx->api_dst_frame.data.frame,
                                          in->width, in->height, 0);
    if (retcode != NI_RETCODE_SUCCESS)
        return AVERROR(ENOMEM);

    /*
     * Allocate device input frame for format conversion.
     * This call won't actually allocate a frame, but sends the incoming
     * hardware frame index to the scaler manager.
     */
    retcode = ni_device_alloc_frame(&format_ctx->api_ctx, FFALIGN(in->width, 2),
                                    FFALIGN(in->height, 2), input_format, 0, 0,
                                    0, 0, 0, 0, frame_surface->ui16FrameIdx,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG,
               "Can't allocate device input frame for format conversion %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Allocate hardware device destination frame. This acquires a frame from
     * the pool that was allocated in the initialization stage.
     */
    retcode = ni_device_alloc_frame(&format_ctx->api_ctx, FFALIGN(in->width, 2),
                                    FFALIGN(in->height, 2), output_format,
                                    NI_SCALER_FLAG_IO, 0, 0, 0, 0, 0, -1,
                                    NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG,
               "Can't allocate device output frame for format conversion %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Set the new frame index */
    retcode = ni_device_session_read_hwdesc(&format_ctx->api_ctx,
                                            &format_ctx->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG,
               "Can't get the scaler output frame index %d\n", retcode);
        return AVERROR(ENOMEM);
    }
    new_frame_surface =
        (niFrameSurface1_t *)format_ctx->api_dst_frame.data.frame.p_data[3];
    new_frame_surface->bit_depth  = 1;
    new_frame_surface->encoding_type = 1;
    new_frame_surface->ui16width  = in->width;
    new_frame_surface->ui16height    = in->height;
    *rgba_frame_surface = new_frame_surface;

    return retcode;
}

static int ni_hwframe_converted_download(AVFilterContext *ctx, NetIntBgrContext *s,
                                         niFrameSurface1_t *hwdl_frame_surface)
{
    HwFormatContext *format_ctx = s->format_ctx;
    int bytesRead;

    av_log(ctx, AV_LOG_DEBUG,
           "Input surface frame index for HWDL conversion %d\n",
           hwdl_frame_surface->ui16FrameIdx);
    format_ctx->api_ctx.is_auto_dl = false;
    bytesRead = ni_device_session_hwdl(
        &format_ctx->api_ctx, &s->p_dl_frame, hwdl_frame_surface);
    if (bytesRead <=0) {
        av_log(ctx, AV_LOG_ERROR, "HWDL ERROR %d\n", bytesRead);
        return bytesRead;
    }
    return bytesRead;
}

static int ni_bgr_create_output(AVFilterContext* ctx, AVFrame* in, AVFrame** realout)
{
    NetIntBgrContext *s = ctx->priv;
    AVFrame *out;
    int ret = 0;

    av_log(ctx, AV_LOG_DEBUG, "ni_bgr_create_output\n");

    //set up the output
    out = av_frame_alloc();
    if (!out)
        return AVERROR(ENOMEM);

    av_frame_copy_props(out, in);
    out->width         = in->width;
    out->height        = in->height;
    out->format        = AV_PIX_FMT_NI_QUAD;
    out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);

    ret = av_hwframe_get_buffer(s->hw_frames_ctx, out, 0);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to get buffer\n");
        av_frame_free(&out);
        return ret;
    }

    av_frame_copy_props(s->alpha_enlarge_frame, in);
    s->alpha_enlarge_frame->format = AV_PIX_FMT_RGBA;
    ret = av_hwframe_transfer_data(out, // dst src flags
                                   s->alpha_enlarge_frame, 0);

    //There is a upload frame buffer transfer in hwupload filter
    //but all measurements show no harm in perf for now
    //we can probably skip that transfer if it ever gets down to it

    *realout = out;
    return ret;
}


static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    NetIntBgrContext *s = ctx->priv;
    int ret;
    int clean_scale_format = 0;

    /* ai roi */
    ni_roi_network_t *network;
    ni_retcode_t retval;
    AiContext *ai_ctx;
    niFrameSurface1_t *downscale_bgr_surface;
    niFrameSurface1_t *rgba_frame_surface;
    niFrameSurface1_t *logging_surface;
    niFrameSurface1_t *logging_surface_out;
    AVFrame *realout;

    s->framecount++;

#define CLEAN_SCALE 1
#define CLEAN_FORMAT 2

    av_log(ctx, AV_LOG_DEBUG, "entering %s\n", __func__);

    if (!s->initialized) {
#if IS_FFMPEG_71_AND_ABOVE
        AVFilterLink *outlink = link->dst->outputs[0];
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           ctx->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        s->skip_random_offset = 1;
        ret = ni_bgr_config_input(ctx, in);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed to config input\n");
            goto fail;
        }

#if !IS_FFMPEG_342_AND_ABOVE
        ret = config_output(ctx->outputs[0], in);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed to config output\n");
            goto fail;
        }
#endif

        s->initialized = 1;
    }

    ai_ctx  = s->ai_ctx;
    network = &s->network;

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    retval  = ni_ai_packet_buffer_alloc(&ai_ctx->api_dst_pkt.data.packet,
                                        &network->raw);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate packet\n");
        return AVERROR(EAGAIN);
    }

    if ((s->skip == 0) || ((s->framecount - 1) % (s->skip + 1) == 0)) {
        if (s->skip_random_offset) {
            s->framecount += ai_ctx->api_ctx.session_id % (s->skip + 1);
            av_log(ctx, AV_LOG_DEBUG, "BGR SPACE sid %u fc %u\n",
                   ai_ctx->api_ctx.session_id, s->framecount);
            s->skip_random_offset = 0;
        }
        s->skipInference = false;
    } else {
        s->skipInference = true;
        av_log(ctx, AV_LOG_DEBUG, "Inference skip, framecount %d\n", s->framecount);
    }
    //Main Job
    {
        if (!s->skipInference) {
            ret = ni_hwframe_scale(ctx, s, in, network->netw, network->neth,
                                    &downscale_bgr_surface);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "Error run hwframe scale\n");
                goto fail;
            }
            av_log(ctx, AV_LOG_DEBUG, "filt frame surface frameIdx %d\n",
                downscale_bgr_surface->ui16FrameIdx);
        }

        if (!s->skipInference) {
            /* allocate output buffer */
            retval = ni_device_alloc_frame(&ai_ctx->api_ctx, 0, 0, 0, 0, 0, 0, 0, 0,
                                            downscale_bgr_surface->ui32nodeAddress,
                                            downscale_bgr_surface->ui16FrameIdx,
                                            NI_DEVICE_TYPE_AI);
            if (retval != NI_RETCODE_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "failed to alloc hw input frame\n");
                ret =  AVERROR(ENOMEM);
                clean_scale_format |= CLEAN_SCALE;
                goto fail;
            }
        }
        ret = ni_hwframe_convert_format(ctx, s, in, &rgba_frame_surface);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                    "Error in runnig hwframe for format conversion\n");
            goto fail;
        }
        ret = ni_hwframe_converted_download(ctx, s, rgba_frame_surface);
        if (ret <= 0) {
            av_log(ctx, AV_LOG_ERROR,
                    "Error in downloading hwframe for format conversion\n");
            clean_scale_format |= CLEAN_FORMAT;
            goto fail;
        }
        ni_hwframe_buffer_recycle(rgba_frame_surface,
                                    rgba_frame_surface->device_handle);
        if (!s->skipInference) {
            do {
                retval = ni_device_session_read(
                    &ai_ctx->api_ctx, &ai_ctx->api_dst_pkt, NI_DEVICE_TYPE_AI);
                if (retval < 0) {
                    av_log(ctx, AV_LOG_ERROR, "read hwdesc retval %d\n", retval);
                    ret = AVERROR(EIO);
                    clean_scale_format |= CLEAN_SCALE;
                    goto fail;
                } else if (retval > 0) {
                    ret = ni_bgr_process(ctx, &ai_ctx->api_dst_pkt);
                    if (ret != 0) {
                        av_log(ctx, AV_LOG_ERROR,
                                "failed to process tensor\n");
                        clean_scale_format |= CLEAN_SCALE;
                        goto fail;
                    }
                }
                ni_usleep(100); //this loop should be in libxcoder side ideally
            } while (retval == 0);
            ni_hwframe_buffer_recycle(downscale_bgr_surface,
                                        downscale_bgr_surface->device_handle);
        } else { // use the last frame background shape if current frame is skipped
            ret = ni_bgr_process(ctx, &ai_ctx->api_dst_pkt);
            if (ret != 0) {
                av_log(ctx, AV_LOG_ERROR, "failed to process tensor\n");
                goto fail;
            }
        }

    }

    //output AVframe created
    ni_bgr_create_output(ctx, in, &realout);

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_bgr");
#endif

    //Logging for hwframe tracking
    logging_surface = (niFrameSurface1_t*)in->data[3];
    logging_surface_out = (niFrameSurface1_t*)realout->data[3];
    av_log(link->dst, AV_LOG_DEBUG,
        "vf_bgr_ni.c:IN trace ui16FrameIdx = [%d] --> out [%d]\n",
        logging_surface->ui16FrameIdx, logging_surface_out->ui16FrameIdx);

    av_frame_free(&in);
    return ff_filter_frame(ctx->outputs[0], realout);
fail:
    av_frame_free(&in);
    if (clean_scale_format & CLEAN_SCALE) {
        ni_hwframe_buffer_recycle(downscale_bgr_surface,
                                  downscale_bgr_surface->device_handle);
    }
    if (clean_scale_format & CLEAN_FORMAT) {
        ni_hwframe_buffer_recycle(rgba_frame_surface,
                                  rgba_frame_surface->device_handle);
    }
    return ret;
}

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    NetIntBgrContext *s = ctx->priv;
    AVFrame *frame = NULL;
    AVHWFramesContext *hwfc;
    AVNIFramesContext *f_hwctx;
    int ret = 0;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        if (s->initialized) {
            hwfc = (AVHWFramesContext *) s->hw_frames_ctx->data;
            f_hwctx = (AVNIFramesContext*) hwfc->hwctx;

            ret = ni_device_session_query_buffer_avail(&f_hwctx->api_ctx, NI_DEVICE_TYPE_UPLOAD);
        }

        if (ret == NI_RETCODE_ERROR_UNSUPPORTED_FW_VERSION) {
            av_log(ctx, AV_LOG_WARNING, "No backpressure support in FW\n");
        } else if (ret < 0) {
            av_log(ctx, AV_LOG_WARNING, "%s: query ret %d, ready %u inlink framequeue %lu available_frame %d outlink framequeue %lu frame_wanted %d - return NOT READY\n",
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

#define OFFSET(x) offsetof(NetIntBgrContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption ni_bgr_options[] = {
    { "nb",   "path to network binary file",      OFFSET(nb_file), AV_OPT_TYPE_STRING, .flags = FLAGS},
    { "skip", "frames to skip between inference", OFFSET(skip),    AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, FLAGS},
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL },
};

AVFILTER_DEFINE_CLASS(ni_bgr);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
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
    {NULL}
#endif
};

AVFilter ff_vf_bgr_ni_quadra = {
    .name           = "ni_quadra_bgr",
    .description    = NULL_IF_CONFIG_SMALL(
        "NETINT Quadra remove the background of the input video v" NI_XCODER_REVISION),
    .uninit         = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate       = activate,
#endif
    .priv_size      = sizeof(NetIntBgrContext),
    .priv_class     = &ni_bgr_class,
// only FFmpeg 3.4.2 and above have .flags_internal
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
