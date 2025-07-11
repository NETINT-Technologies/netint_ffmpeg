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

typedef struct OverlayContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
} OverlayContext;

typedef struct NetIntBgContext {
    const AVClass *class;

    AVBufferRef *hwdevice;
    AVBufferRef *hwframe;

    AVBufferRef *hw_frames_ctx;

    /* roi */
    AVBufferRef *out_frames_ref;

    /* ai */
    int initialized;
    const char *nb_file; /* path to network binary */
    const char *bg_img;  /* path to background img */
    int use_default_bg;  /* use_default_bg */

    AiContext *ai_ctx;
    ni_roi_network_t network;
    HwScaleContext *hws_ctx;

    /* overlay */
    OverlayContext *overlay_ctx;

    /* bg */
    uint8_t *mask_data;
    int bg_frame_size;
    // AVFrame *bg_frame;
    AVFrame *alpha_mask_frame;
    AVFrame *alpha_large_frame;
    AVFrame *alpha_mask_hwframe;
    AVFrame *alpha_enlarge_frame;
    int framecount;
    int skip;
    int keep_alive_timeout; /* keep alive timeout setting */
    bool is_p2p;
    int buffer_limit;
} NetIntBgContext;

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

static void cleanup_ai_context(AVFilterContext *ctx, NetIntBgContext *s)
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
    if (s->mask_data) {
        free(s->mask_data);
    }
}


static int init_ai_context(AVFilterContext *ctx, NetIntBgContext *s,
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

static void ni_destroy_network(ni_roi_network_t *network)
{
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

    network->layers =
        malloc(sizeof(ni_roi_network_layer_t) * ni_network->output_num);
    if (!network->layers) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate network layer memory\n");
        return AVERROR(ENOMEM);
    }
    memset(network->layers, 0,
           sizeof(ni_roi_network_layer_t) * ni_network->output_num);

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
    ni_destroy_network(network);
    return ret;
}

static av_cold int init_hwframe_scale(AVFilterContext *ctx, NetIntBgContext *s,
                                      enum AVPixelFormat format,
                                      AVFrame *frame)
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
                                    DEFAULT_NI_FILTER_POOL_SIZE, s->buffer_limit);
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

static void cleanup_hwframe_scale(NetIntBgContext *s)
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

static int init_hwframe_overlay(AVFilterContext *ctx, NetIntBgContext *s,
                                AVFrame *main_frame)
{
    ni_retcode_t retcode;
    OverlayContext *overlay_ctx;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;
    int main_cardno;
    int ret;

    overlay_ctx = av_mallocz(sizeof(OverlayContext));
    if (!overlay_ctx) {
        av_log(ctx, AV_LOG_ERROR, "could not allocate overlay ctx\n");
        return AVERROR(ENOMEM);
    }
    s->overlay_ctx = overlay_ctx;

    ni_device_session_context_init(&overlay_ctx->api_ctx);

    pAVHFWCtx   = (AVHWFramesContext *)main_frame->hw_frames_ctx->data;
    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    main_cardno = ni_get_cardno(main_frame);

    overlay_ctx->api_ctx.device_handle = pAVNIDevCtx->cards[main_cardno];
    overlay_ctx->api_ctx.blk_io_handle = pAVNIDevCtx->cards[main_cardno];

    overlay_ctx->api_ctx.hw_id             = main_cardno;
    overlay_ctx->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
    overlay_ctx->api_ctx.scaler_operation  = NI_SCALER_OPCODE_OVERLAY;
    overlay_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
    overlay_ctx->api_ctx.isP2P              = s->is_p2p;

    retcode =
        ni_device_session_open(&overlay_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't open device session on card %d\n",
               main_cardno);
        ret = AVERROR(EIO);
        ni_device_session_close(&overlay_ctx->api_ctx, 1,
                                NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&overlay_ctx->api_ctx);
        goto fail_out;
    }

    if (overlay_ctx->api_ctx.isP2P) {
        pool_size = 1;
    }
#if IS_FFMPEG_71_AND_ABOVE
    else {
        pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
    }
#endif

#if IS_FFMPEG_70_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create frame pool on device */
    ret = ff_ni_build_frame_pool(&overlay_ctx->api_ctx, main_frame->width,
                                 main_frame->height, pAVHFWCtx->sw_format,
                                 pool_size,
                                 s->buffer_limit);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "can not build overlay frame pool\n");
        ni_device_session_close(&overlay_ctx->api_ctx, 1,
                                NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&overlay_ctx->api_ctx);
        goto fail_out;
    }

    return 0;

fail_out:
    av_free(overlay_ctx);
    return ret;
}

static void cleanup_hwframe_overlay(NetIntBgContext *s)
{
    OverlayContext *overlay_ctx = s->overlay_ctx;

    if (overlay_ctx) {
        ni_frame_buffer_free(&overlay_ctx->api_dst_frame.data.frame);
        ni_device_session_close(&overlay_ctx->api_ctx, 1,
                                NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&overlay_ctx->api_ctx);
        av_free(overlay_ctx);
        s->overlay_ctx = NULL;
    }
}

static int init_hwframe_uploader(AVFilterContext *ctx, NetIntBgContext *s,
                                 AVFrame *frame)
{
    int ret;
    AVHWFramesContext *hwframe_ctx;

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

    s->hw_frames_ctx = av_buffer_ref(s->hwframe);
    if (!s->hw_frames_ctx)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntBgContext *s            = ctx->priv;
    ni_roi_network_t *network = &s->network;

    av_buffer_unref(&s->hwframe);
    av_buffer_unref(&s->hwdevice);

    av_buffer_unref(&s->hw_frames_ctx);

    /* roi */
    av_buffer_unref(&s->out_frames_ref);

    /* ai */
    cleanup_ai_context(ctx, s);
    ni_destroy_network(network);

    /* bg */
    av_frame_free(&s->alpha_mask_frame);
    av_frame_free(&s->alpha_large_frame);

    cleanup_hwframe_scale(s);

    cleanup_hwframe_overlay(s);
}

static AVFrame *import_bg_frame(AVFilterContext *ctx, int src_w, int src_h,
                                enum AVPixelFormat src_pixfmt,
                                const char *imageFileName) // ovleray network nb
{
    AVCodecContext *pCodecCtx;
    AVFormatContext *pFormatCtx = NULL;
    struct SwsContext *convert_ctx = NULL;
    AVFrame *pFrame, *dst;
    AVPacket *packet;
    int index;
    int ret;
#if LIBAVFILTER_VERSION_MAJOR >= 8
    // Clean warning as the return type of avcodec_find_decoder is of const
    // form FFmpeg-n5.0
    const
#endif
    AVCodec *dec, *pCodec;

    //FFmpeg3.4.2 requirement only
#if ((LIBAVFILTER_VERSION_MAJOR <= 6) && (LIBAVFILTER_VERSION_MINOR <= 107))
    av_register_all();
#endif

    if (avformat_open_input(&(pFormatCtx), imageFileName, NULL, NULL) != 0) {
        av_log(ctx, AV_LOG_ERROR, "Can't open image file '%s'\n",
               imageFileName);
        return NULL;
    }

    if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Can't find stream\n");
        return NULL;
    }

    av_dump_format(pFormatCtx, 0, imageFileName,
                   0); // when the fourth set to 1, it produce segment fault
                       // error av_dump_format(pFormatCtx, 0, imageFileName, 1)
                       // the last param is: is_output select whether the
                       // specified context is an input(0) or output(1)

    index = av_find_best_stream(pFormatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);

    dec = avcodec_find_decoder(pFormatCtx->streams[index]->codecpar->codec_id);
    pCodecCtx = avcodec_alloc_context3(dec);
    avcodec_parameters_to_context(pCodecCtx,
                                  pFormatCtx->streams[index]->codecpar);

    // Find the decoder for the video stream
    pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
    if (!pCodec) {
        av_log(ctx, AV_LOG_ERROR, "Codec not found\n");
        return NULL;
    }

    // Open codec
    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Could not open codec\n");
        return NULL;
    }

    pFrame = av_frame_alloc();
    if (!pFrame) {
        av_log(ctx, AV_LOG_ERROR, "Can't allocate memory for AVFrame\n");
        return NULL;
    }

    packet = av_packet_alloc();
    if (!packet) {
        av_log(ctx, AV_LOG_ERROR, "Can't allocate memory for AVPacket\n");
        return NULL;
    }

    while (av_read_frame(pFormatCtx, packet) >= 0) {
        if (packet->stream_index != index) {
            continue;
        }
        ret = avcodec_send_packet(pCodecCtx, packet);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "avcodec_send_packet failed");
            av_packet_unref(packet);
            return NULL;
        }
        av_packet_unref(packet);
        ret = avcodec_receive_frame(pCodecCtx, pFrame);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "avcodec_receive_frame failed");
            return NULL;
        }
    }

    av_packet_free(&packet);

    dst = av_frame_alloc();

    dst->format = (int)src_pixfmt;
    dst->width  = src_w;
    dst->height = src_h;

    av_frame_get_buffer(dst, 4);

    convert_ctx = sws_getContext(pFrame->width, pFrame->height,
                                 pCodecCtx->pix_fmt, dst->width, dst->height,
                                 dst->format, SWS_BICUBIC, NULL, NULL, NULL);

    sws_scale(convert_ctx, (const uint8_t *const *)pFrame->data,
              pFrame->linesize, 0, pFrame->height, dst->data, dst->linesize);
    sws_freeContext(convert_ctx);

    av_frame_free(&pFrame);
    avformat_close_input(&(pFormatCtx));
    avcodec_free_context(&pCodecCtx);

    return dst;
}

static AVFrame *create_bg_frame(AVFilterContext *ctx, int src_w, int src_h,
                                enum AVPixelFormat src_pixfmt)
{

    NetIntBgContext *s   = ctx->priv;

    int dst_w = src_w;
    int dst_h = src_h;
    int size_Y, size_UV;
    uint8_t *Y_value, *U_value, *V_value, *A_value;

    /*create bg frame*/
    AVFrame *dst = av_frame_alloc();

    dst->format = (int)src_pixfmt;
    dst->width  = dst_w;
    dst->height = dst_h;

    av_frame_get_buffer(dst, 4);

    av_log(ctx, AV_LOG_DEBUG, "create_frame function: dst_linesize: %d \n",
           dst->linesize[0]);

    s->bg_frame_size = s->network.netw * s->network.neth;

    /*copy value to dst(bg_frame) frame*/
    size_Y  = dst->width * dst->height;
    size_UV = dst->width * dst->height / 4;

    Y_value = av_mallocz(size_Y * sizeof(uint8_t));
    U_value = av_mallocz(size_UV * sizeof(uint8_t));
    V_value = av_mallocz(size_UV * sizeof(uint8_t));
    A_value = av_mallocz(s->bg_frame_size * sizeof(uint8_t));

    av_log(ctx, AV_LOG_DEBUG,
           "create_bg_frame dst->linesize[0] %d dst->linesize[1] %d "
           "dst->linesize[2] %d dst->linesize[3] %d\n",
           dst->linesize[0], dst->linesize[1], dst->linesize[2],
           dst->linesize[3]);

    for (int i = 0; i < size_Y; i++) {
        Y_value[i] = 149;
    }

    for (int i = 0; i < size_UV; i++) {
        U_value[i] = 43;
    }

    for (int i = 0; i < size_UV; i++) {
        V_value[i] = 21;
    }

    for (int i = 0; i < s->bg_frame_size; i++) {
        A_value[i] = 21;
    }

    // copy the mask_data to the alpha_mask_frame
    av_image_copy_plane(dst->data[0], dst->linesize[0], Y_value,
                        dst->linesize[0], dst->linesize[0], dst->height);

    av_image_copy_plane(dst->data[1], dst->linesize[1], U_value,
                        dst->linesize[1], dst->linesize[1], (dst->height) / 2);

    av_image_copy_plane(dst->data[2], dst->linesize[2], V_value,
                        dst->linesize[2], dst->linesize[2], (dst->height) / 2);

    av_image_copy_plane(dst->data[3], dst->linesize[3], A_value,
                        dst->linesize[3], dst->linesize[3], dst->height);

    av_free(Y_value);
    av_free(U_value);
    av_free(V_value);
    av_free(A_value);
    return dst;
}

static AVFrame *create_frame(AVFilterContext *ctx, int src_w, int src_h,
                             enum AVPixelFormat src_pixfmt)
{
    int dst_w                     = src_w;
    int dst_h                     = src_h;

    AVFrame *dst = av_frame_alloc();

    dst->format = (int)src_pixfmt;
    dst->width  = dst_w;
    dst->height = dst_h;
    av_frame_get_buffer(dst, 4);

    av_log(ctx, AV_LOG_DEBUG, "create_frame function: dst_linesize: %d \n",
           dst->linesize[0]);

    return dst;
}

static int ni_get_mask(AVFilterContext *ctx, uint8_t *mask_data,
                       ni_roi_network_t *network)
{
    uint8_t Y_MIN = 0;
    uint8_t Y_MAX = 255;
    int mask_size = network->netw * network->neth;
    int i;

    ni_roi_network_layer_t *l = &network->layers[0];

    av_log(ctx, AV_LOG_DEBUG,
           "network->netw: %d network->neth: %d mask_size %d \n", network->netw,
           network->neth, network->netw * network->neth);

    if (!mask_data) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate s->mask_data memory\n");
        return AVERROR(ENOMEM);
    }

    // nchw proprocessing
    /* for (int i = 0; i<mask_size;i++){
        if (l->output[i] > l->output[i+mask_size]) {
           mask_data[i] = Y_MAX;

        } else {
            mask_data[i] = Y_MIN;
        }
    } */

    // nhwc proprocessing
    for (i = 0; i < mask_size; i++) {
        if (l->output[2 * i] > l->output[2 * i + 1]) {
            mask_data[i] = Y_MAX;

        } else {
            mask_data[i] = Y_MIN;
        }
        // av_log(ctx, AV_LOG_INFO,
        //       "%d %d\n", i,  mask_data[i]);
    }

    av_log(ctx, AV_LOG_DEBUG, "lw=%d, lh=%d, ln=%d, lo=%d, nw=%d, nh=%d, \n",
           l->width, l->height, l->component, l->output_number, network->netw,
           network->neth);
    return 0;
}

static int get_alpha_mask_frame(AVFilterContext *ctx, uint8_t *mask_data)
{
    NetIntBgContext *s = ctx->priv;
    struct SwsContext *convert_ctx = NULL;

    /* Copy the alpha plane from mask_data */
    av_image_copy_plane(
        s->alpha_mask_frame->data[3], s->alpha_mask_frame->linesize[3],
        mask_data, s->alpha_mask_frame->linesize[3],
        s->alpha_mask_frame->linesize[3], s->alpha_mask_frame->height);

    av_log(ctx, AV_LOG_DEBUG,
           "get_alpha_mask_frame function: alpha_mask_frame->width: %d "
           "alpha_mask_frame->height:%d alpha_mask_frame->format:%d \n",
           s->alpha_mask_frame->width, s->alpha_mask_frame->height,
           s->alpha_mask_frame->format);

    convert_ctx = sws_getContext(
        s->alpha_mask_frame->width, s->alpha_mask_frame->height,
        s->alpha_mask_frame->format, s->alpha_large_frame->width,
        s->alpha_large_frame->height, s->alpha_large_frame->format,
        SWS_FAST_BILINEAR, NULL, NULL, NULL);

    // s->alpha_mask_frame is small frame (144x256) AV_PIX_FMT_YUVA420P,
    // s->alpha_large_frame is large frame (1280x720) AV_PIX_FMT_RGBA
    sws_scale(convert_ctx, (const uint8_t *const *)s->alpha_mask_frame->data,
              s->alpha_mask_frame->linesize, 0, s->alpha_mask_frame->height,
              s->alpha_large_frame->data, s->alpha_large_frame->linesize);

    sws_freeContext(convert_ctx);

    s->alpha_enlarge_frame = s->alpha_large_frame;
    return 0;
}

static int ni_bg_config_input(AVFilterContext *ctx, AVFrame *frame)
{
    NetIntBgContext *s = ctx->priv;
    enum AVPixelFormat alpha_mask_pixfmt = AV_PIX_FMT_YUVA420P;
    enum AVPixelFormat alpha_large_frame_format = AV_PIX_FMT_RGBA;
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

    ret = init_hwframe_overlay(ctx, s, frame);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "could not initialized hwframe overlay context\n");
        goto fail_out;
    }

    s->mask_data = malloc(s->network.netw * s->network.neth * sizeof(uint8_t));

    if (!s->mask_data) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate sctx->mask_datamemory\n");
        return AVERROR(ENOMEM);
    }

    // int(use_default_bg)==0 -> import_bg_frame don't use default_bg
    // int(use_default_bg)>0 -> create_bg_frame use default_bg

    if (s->use_default_bg == 0) {
        s->alpha_mask_frame =
            import_bg_frame(ctx, s->network.netw, s->network.neth,
                            alpha_mask_pixfmt, s->bg_img);
    } else {
        s->alpha_mask_frame = create_bg_frame(
            ctx, s->network.netw, s->network.neth, alpha_mask_pixfmt);
    }

    // create the new s->alpha_large_frame, which is the enlarged version of
    // s->alpha_mask_frame
    s->alpha_large_frame = create_frame(ctx, frame->width, frame->height,
                                        alpha_large_frame_format);

    av_log(ctx, AV_LOG_DEBUG,
           "ni_bg_config_input get_alpha_mask_frame function: "
           "alpha_mask_frame->width: %d alpha_mask_frame->height: %d "
           "s->alpha_mask_frame->format: %d alpha_large_frame->width :%d "
           "alpha_large_frame->height: %d alpha_large_frame->format: %d "
           "frame->width: %d frame->height: %d frame->format: %d "
           "frame->linesize[0]: %d\n",
           s->alpha_mask_frame->width, s->alpha_mask_frame->height,
           s->alpha_mask_frame->format, s->alpha_large_frame->width,
           s->alpha_large_frame->height, s->alpha_large_frame->format,
           frame->width, frame->height, frame->format, frame->linesize[0]);

    return 0;

fail_out:
    cleanup_ai_context(ctx, s);

    ni_destroy_network(&s->network);

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
    NetIntBgContext *s       = ctx->priv;

    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    AVNIFramesContext *out_ni_ctx;

    av_log(ctx, AV_LOG_DEBUG, "%s\n", __func__);

    av_buffer_unref(&s->hwframe);

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (inlink->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)inlink->hw_frames_ctx->data;
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

    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    av_log(ctx, AV_LOG_DEBUG, "inlink wxh %dx%d\n", inlink->w, inlink->h);

    /* roi */
    outlink->w = inlink->w;
    outlink->h = inlink->h;

    av_log(ctx, AV_LOG_DEBUG, "outlink wxh %dx%d\n", outlink->w, outlink->h);

    out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
    ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx);
    ni_device_session_copy(&s->ai_ctx->api_ctx, &out_ni_ctx->api_ctx);

    out_frames_ctx->format            = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width             = outlink->w;
    out_frames_ctx->height            = outlink->h;
    out_frames_ctx->sw_format         = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size = NI_BG_ID;

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

static int ni_hwframe_scale(AVFilterContext *ctx, NetIntBgContext *s, AVFrame *in,
                            int w, int h,
                            niFrameSurface1_t **filt_frame_surface)
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
        av_log(NULL, AV_LOG_DEBUG, "Can't allocate device input frame %d\n",
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
        av_log(NULL, AV_LOG_DEBUG, "Can't allocate device output frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Set the new frame index */
    ni_device_session_read_hwdesc(
        &scale_ctx->api_ctx, &scale_ctx->api_dst_frame, NI_DEVICE_TYPE_SCALER);
    new_frame_surface =
        (niFrameSurface1_t *)scale_ctx->api_dst_frame.data.frame.p_data[3];

    *filt_frame_surface = new_frame_surface;

    return 0;
}

static int ni_hwframe_overlay(AVFilterContext *ctx, NetIntBgContext *s,
                              AVFrame *frame, AVFrame *overlay,
                              AVFrame **output)
{
    AVHWFramesContext *main_frame_ctx;
    AVFilterLink *outlink;
    AVFrame *out;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    int flags, main_cardno;
    int main_scaler_format, ovly_scaler_format;
    ni_retcode_t retcode;
    uint16_t tempFIDOverlay     = 0;
    uint16_t tempFIDFrame       = 0;
    OverlayContext *overlay_ctx = s->overlay_ctx;

    outlink = ctx->outputs[0];

    main_frame_ctx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    main_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(main_frame_ctx->sw_format);
    main_cardno = ni_get_cardno(frame);

    if (overlay) {
        int ovly_cardno;
        AVHWFramesContext *ovly_frame_ctx;
        ovly_frame_ctx = (AVHWFramesContext *)overlay->hw_frames_ctx->data;
        ovly_scaler_format =
            ff_ni_ffmpeg_to_gc620_pix_fmt(ovly_frame_ctx->sw_format);
        ovly_cardno = ni_get_cardno(overlay);

        if (main_cardno != ovly_cardno) {
            av_log(ctx, AV_LOG_ERROR,
                   "Main/Overlay frames on different cards\n");
            return AVERROR(EINVAL);
        }
    } else {
        ovly_scaler_format = 0;
    }

    /* Allocate a ni_frame for the overlay output */
    retcode = ni_frame_buffer_alloc_hwenc(
        &overlay_ctx->api_dst_frame.data.frame, outlink->w, outlink->h, 0);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate overlay output\n");
        return AVERROR(ENOMEM);
    }

    if (overlay) {
        frame_surface  = (niFrameSurface1_t *)overlay->data[3];
        tempFIDOverlay = frame_surface->ui16FrameIdx;
        av_log(ctx, AV_LOG_INFO,
               "ovly_scaler_format %d, frameidx %d, wxh %dx%d\n",
               ovly_scaler_format, tempFIDOverlay, overlay->width,
               overlay->height);
    }

    /*
     * Allocate device input frame for overlay picture. This call won't actually
     * allocate a frame, but sends the incoming hardware frame index to the
     * scaler manager.
     */
    retcode = ni_device_alloc_frame(
        &overlay_ctx->api_ctx,
        overlay ? FFALIGN(overlay->width, 2) : 0,
        overlay ? FFALIGN(overlay->height, 2) : 0,
        ovly_scaler_format,
        0,
        overlay ? FFALIGN(overlay->width, 2) : 0,
        overlay ? FFALIGN(overlay->height, 2) : 0,
        0,
        0,
        frame_surface ? (int)frame_surface->ui32nodeAddress : 0,
        frame_surface ? frame_surface->ui16FrameIdx : 0,
        NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't assign frame for overlay input %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    frame_surface = (niFrameSurface1_t *)frame->data[3];
    if (frame_surface == NULL)
        return AVERROR(EINVAL);

    tempFIDFrame = frame_surface->ui16FrameIdx;

    av_log(ctx, AV_LOG_INFO, "main frame: format %d, frameidx %d, wxh %dx%d\n",
           main_scaler_format, tempFIDFrame, frame->width, frame->height);
    /*
     * Allocate device output frame from the pool. We also send down the frame
     * index of the background frame to the scaler manager.
     */
    flags   = NI_SCALER_FLAG_IO;
    retcode = ni_device_alloc_frame(&overlay_ctx->api_ctx,
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
    if (!out)
        return AVERROR(ENOMEM);

    av_frame_copy_props(out, frame);

    out->width  = outlink->w;
    out->height = outlink->h;
    out->format = AV_PIX_FMT_NI_QUAD;

    /* Quadra 2D engine always outputs limited color range */
    out->color_range = AVCOL_RANGE_MPEG;

    av_log(ctx, AV_LOG_INFO, "outlink wxh %dx%d\n", outlink->w, outlink->h);

    /* Reference the new hw frames context */
    out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    out->data[3]       = av_malloc(sizeof(niFrameSurface1_t));

    if (!out->data[3]) {
        av_frame_free(&out);
        return AVERROR(ENOMEM);
    }

    /* Copy the frame surface from the incoming frame */
    memcpy(out->data[3], frame->data[3], sizeof(niFrameSurface1_t));

    /* Set the new frame index */
    ni_device_session_read_hwdesc(&overlay_ctx->api_ctx,
                                  &overlay_ctx->api_dst_frame,
                                  NI_DEVICE_TYPE_SCALER);

    frame_surface = (niFrameSurface1_t *)out->data[3];
    new_frame_surface =
        (niFrameSurface1_t *)overlay_ctx->api_dst_frame.data.frame.p_data[3];
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

    frame_surface->ui16width  = out->width;
    frame_surface->ui16height = out->height;

    av_log(ctx, AV_LOG_INFO,
           "%s:IN trace ui16FrameIdx = [%d] and [%d] --> out [%d] \n", __func__,
           tempFIDFrame, tempFIDOverlay, frame_surface->ui16FrameIdx);

    out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t),
                                   ff_ni_frame_free, NULL, 0);

    *output = out;
    return 0;
}

static int ni_bg_process(AVFilterContext *ctx, ni_session_data_io_t *p_dst_pkt,
                         AVFrame *in)
{
    NetIntBgContext *s = ctx->priv;
    ni_retcode_t retval;
    ni_roi_network_t *network = &s->network;
    int ret;
    int i;

    for (i = 0; i < network->raw.output_num; i++) {
        retval = ni_network_layer_convert_output(
            network->layers[i].output,
            network->layers[i].output_number * sizeof(float),
            &p_dst_pkt->data.packet, &network->raw, i);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "failed to read layer %d output. retval %d\n", i, retval);
            return AVERROR(EIO);
        }
    }

    ret = ni_get_mask(ctx, s->mask_data, network);

    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to get mask data.\n");
        return ret;
    }

    av_log(ctx, AV_LOG_DEBUG, "s->mask_data %d\n", s->mask_data[2000]);

    ret = get_alpha_mask_frame(ctx, s->mask_data);

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

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    NetIntBgContext *s = ctx->priv;
    int ret;
    /* ai roi */
    ni_roi_network_t *network;
    ni_retcode_t retval;
    AiContext *ai_ctx;
    /* overlay */
    AVFrame *realout;

    av_log(ctx, AV_LOG_INFO, "entering %s\n", __func__);

    s->framecount++;

    if (!s->initialized) {
#if IS_FFMPEG_71_AND_ABOVE
        AVFilterLink *outlink = link->dst->outputs[0];
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           ctx->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        ret = ni_bg_config_input(ctx, in);
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

        AVHWFramesContext *in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx);
        ni_device_session_copy(&s->ai_ctx->api_ctx, &out_ni_ctx->api_ctx);

        s->initialized = 1;
    }

    ai_ctx  = s->ai_ctx;
    network = &s->network;

    retval  = ni_ai_packet_buffer_alloc(&ai_ctx->api_dst_pkt.data.packet,
                                       &network->raw);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate packet\n");
        ret = AVERROR(EAGAIN);
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    if (in->format == AV_PIX_FMT_NI_QUAD) {
        niFrameSurface1_t *filt_frame_surface;

        if ((s->skip == 0) || ((s->framecount-1) % (s->skip + 1) == 0)) {
            ret = ni_hwframe_scale(ctx, s, in, network->netw, network->neth,
                                   &filt_frame_surface);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "Error run hwframe scale\n");
                goto fail;
            }

            av_log(ctx, AV_LOG_DEBUG, "filt frame surface frameIdx %d\n",
                   filt_frame_surface->ui16FrameIdx);

            /* allocate output buffer */
            retval = ni_device_alloc_frame(&ai_ctx->api_ctx, 0, 0, 0, 0, 0, 0, 0, 0,
                                           filt_frame_surface->ui32nodeAddress,
                                           filt_frame_surface->ui16FrameIdx,
                                           NI_DEVICE_TYPE_AI);
            if (retval != NI_RETCODE_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "failed to alloc hw input frame\n");
                ret = AVERROR(ENOMEM);
                goto fail;
            }

            do {
                retval = ni_device_session_read(
                    &ai_ctx->api_ctx, &ai_ctx->api_dst_pkt, NI_DEVICE_TYPE_AI);
                if (retval < 0) {
                    av_log(ctx, AV_LOG_ERROR, "read hwdesc retval %d\n", retval);
                    ret = AVERROR(EIO);
                    goto fail;
                } else if (retval > 0) {
                    ret = ni_bg_process(ctx, &ai_ctx->api_dst_pkt, in);
                    if (ret != 0) {
                        av_log(ctx, AV_LOG_ERROR,
                               "failed to read roi from packet\n");
                        goto fail;
                    }
                }
                ni_usleep(100); //prevent spamming HW
            } while (retval == 0);

           ni_hwframe_buffer_recycle(filt_frame_surface,
                                     filt_frame_surface->device_handle);
        }
    }

    /* {
        char alpha_pic[512];
        static int frame_number = 0;
        int n;

        n=snprintf(alpha_pic, sizeof(alpha_pic),
    "./bg_test/results/alpha_enlarge_frame_raw_1280x720_%d.rgba",
    frame_number++); alpha_pic[n] = '\0'; save_raw_rgba_data(ctx,
    s->alpha_enlarge_frame, alpha_pic);

    } */

    s->alpha_mask_hwframe = av_frame_alloc();
    if (!s->alpha_mask_hwframe) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    av_log(ctx, AV_LOG_INFO, "get hw_frames_ctx\n");
    ret = av_hwframe_get_buffer(s->hw_frames_ctx, s->alpha_mask_hwframe, 0);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to get buffer\n");
        goto fail;
    }

    s->alpha_mask_hwframe->width  = in->width;
    s->alpha_mask_hwframe->height = in->height;

    ret = av_hwframe_transfer_data(s->alpha_mask_hwframe,
                                   s->alpha_enlarge_frame, 0);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error transferring data to the Quadra\n");
        goto fail;
    }

    {
        niFrameSurface1_t *frame_surface =
            (niFrameSurface1_t *)s->alpha_mask_hwframe->data[3];
        av_log(ctx, AV_LOG_DEBUG, "s->alpha_mask_hwframe frameindex %d\n",
               frame_surface->ui16FrameIdx);
    }

    ret = av_frame_copy_props(s->alpha_mask_hwframe, s->alpha_enlarge_frame);
    if (ret < 0)
        goto fail;

    ret = ni_hwframe_overlay(ctx, s, in, s->alpha_mask_hwframe, &realout);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to do overlay\n");
        goto fail;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_bg");
#endif

    av_frame_free(&in);
    av_frame_free(&s->alpha_mask_hwframe);

    return ff_filter_frame(ctx->outputs[0], realout);
fail:
    av_frame_free(&in);
    if (s->alpha_mask_hwframe)
        av_frame_free(&s->alpha_mask_hwframe);
    return ret;
}

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    NetIntBgContext *s = ctx->priv;
    AVFrame *frame = NULL;
    int ret = 0;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        // Consume from inlink framequeue only when outlink framequeue is empty
        // to prevent filter from exhausting all pre-allocated device buffers
        if (ff_inlink_check_available_frame(outlink))
            return FFERROR_NOT_READY;

        if (s->initialized) {
            ret = ni_device_session_query_buffer_avail(&s->overlay_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
        }

        if (ret == NI_RETCODE_ERROR_UNSUPPORTED_FW_VERSION) {
            av_log(ctx, AV_LOG_WARNING, "No backpressure support in FW\n");
        } else if (ret < 0) {
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

#define OFFSET(x) offsetof(NetIntBgContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)
static const AVOption ni_bg_options[] = {
    { "nb",             "path to network binary file",         OFFSET(nb_file),        AV_OPT_TYPE_STRING, .flags = FLAGS},
    { "bg_img",         "path to replacement background file", OFFSET(bg_img),         AV_OPT_TYPE_STRING, .flags = FLAGS},
    { "use_default_bg", "use bright green background image",   OFFSET(use_default_bg), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1,       FLAGS},
    { "skip",           "frames to skip between inference",    OFFSET(skip),           AV_OPT_TYPE_INT,  {.i64 = 0}, 0, INT_MAX, FLAGS},
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL },
};

AVFILTER_DEFINE_CLASS(ni_bg);

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

AVFilter ff_vf_bg_ni_quadra = {
    .name           = "ni_quadra_bg",
    .description    = NULL_IF_CONFIG_SMALL(
        "NETINT Quadra replace the background of the input video v" NI_XCODER_REVISION),
    .uninit = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate       = activate,
#endif
    .priv_size      = sizeof(NetIntBgContext),
    .priv_class     = &ni_bg_class,
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
