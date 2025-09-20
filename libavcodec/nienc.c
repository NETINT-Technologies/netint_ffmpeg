/*
 * NetInt XCoder H.264/HEVC Encoder common code
 * Copyright (c) 2018-2019 NetInt
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
#include "nienc.h"
#include "bytestream.h"
#include "libavcodec/h264.h"
#include "libavcodec/h264_sei.h"
#if ((LIBAVCODEC_VERSION_MAJOR > 61) || (LIBAVCODEC_VERSION_MAJOR == 61 && LIBAVCODEC_VERSION_MINOR >= 19))
#include "libavcodec/hevc/hevc.h"
#include "libavcodec/hevc/sei.h"
#include "libavutil/mem.h"
#else
#include "libavcodec/hevc.h"
#include "libavcodec/hevc_sei.h"
#endif

#include "libavcodec/put_bits.h"
#include "libavutil/avstring.h"
#include "libavutil/hdr_dynamic_metadata.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_ni_quad.h"
#include "libavutil/mastering_display_metadata.h"
#include "ni_av_codec.h"
#include "ni_util.h"
#include "put_bits.h"
#include "packet_internal.h"

#include <unistd.h>
#if ((LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134))
#include "encode.h"
#endif

static bool free_frames_isempty(XCoderEncContext *ctx);

static bool free_frames_isfull(XCoderEncContext *ctx);

static int deq_free_frames(XCoderEncContext *ctx);

static int enq_free_frames(XCoderEncContext *ctx, int idx);

static int recycle_index_2_avframe_index(XCoderEncContext *ctx, uint32_t recycleIndex);

static bool gop_params_check(AVDictionary *dict, AVCodecContext *avctx)
{
    XCoderEncContext *s = avctx->priv_data;
    AVDictionaryEntry *en = NULL;
    char *key;

    while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
        key = en->key;
        ni_gop_params_check_set(&s->api_param, key);
    }
    return ni_gop_params_check(&s->api_param);
}

static int xcoder_encoder_headers(AVCodecContext *avctx)
{
    // use a copy of encoder context, take care to restore original config
    // cropping setting
    XCoderEncContext *ctx = NULL;
    ni_xcoder_params_t *p_param = NULL;
    ni_packet_t *xpkt = NULL;
    int orig_conf_win_right;
    int orig_conf_win_bottom;
    int linesize_aligned, height_aligned;
    int ret, recv;

    ctx = av_malloc(sizeof(XCoderEncContext));
    if (!ctx) {
        return AVERROR(ENOMEM);
    }

    memcpy(ctx, (XCoderEncContext *)(avctx->priv_data),
           sizeof(XCoderEncContext));

    p_param = (ni_xcoder_params_t *)(ctx->api_ctx.p_session_config);

    orig_conf_win_right  = p_param->cfg_enc_params.conf_win_right;
    orig_conf_win_bottom = p_param->cfg_enc_params.conf_win_bottom;

    linesize_aligned = avctx->width;
    if (linesize_aligned < NI_MIN_WIDTH) {
        p_param->cfg_enc_params.conf_win_right +=
            (NI_MIN_WIDTH - avctx->width) / 2 * 2;
        linesize_aligned = NI_MIN_WIDTH;
    } else {
        if (avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
            avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
            linesize_aligned = FFALIGN(avctx->width, 4);
            p_param->cfg_enc_params.conf_win_right +=
                (linesize_aligned - avctx->width) / 2 * 2;
        } else {
            linesize_aligned = FFALIGN(avctx->width, 2);
            p_param->cfg_enc_params.conf_win_right +=
                (linesize_aligned - avctx->width) / 2 * 2;
        }
    }
    p_param->source_width = linesize_aligned;

    height_aligned = avctx->height;
    if (height_aligned < NI_MIN_HEIGHT) {
        p_param->cfg_enc_params.conf_win_bottom +=
            (NI_MIN_HEIGHT - avctx->height) / 2 * 2;
        height_aligned = NI_MIN_HEIGHT;
    } else {
        if (avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
            avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
            height_aligned = FFALIGN(avctx->height, 4);
            p_param->cfg_enc_params.conf_win_bottom +=
                (height_aligned - avctx->height) / 4 * 4;
        } else {
            height_aligned = FFALIGN(avctx->height, 2);
            p_param->cfg_enc_params.conf_win_bottom +=
                (height_aligned - avctx->height) / 2 * 2;
        }
    }
    p_param->source_height = height_aligned;

    // set enable_acq_limit=1 by default for FFmpeg>=7.0
#if IS_FFMPEG_61_AND_ABOVE_FOR_LIBAVCODEC
    p_param->cfg_enc_params.enable_acq_limit = 1;
#endif

    ctx->api_ctx.hw_id = ctx->dev_enc_idx;
    ff_xcoder_strncpy(ctx->api_ctx.blk_dev_name, ctx->dev_blk_name,
                      NI_MAX_DEVICE_NAME_LEN);
    ff_xcoder_strncpy(ctx->api_ctx.dev_xcoder_name, ctx->dev_xcoder,
                      MAX_CHAR_IN_DEVICE_NAME);

    ret = ni_device_session_open(&(ctx->api_ctx), NI_DEVICE_TYPE_ENCODER);

    ctx->dev_xcoder_name = ctx->api_ctx.dev_xcoder_name;
    ctx->blk_xcoder_name = ctx->api_ctx.blk_xcoder_name;
    ctx->dev_enc_idx = ctx->api_ctx.hw_id;

    switch (ret) {
    case NI_RETCODE_SUCCESS:
        av_log(avctx, AV_LOG_VERBOSE,
               "XCoder %s.%d (inst: %d) opened successfully\n",
               ctx->dev_xcoder_name, ctx->dev_enc_idx, ctx->api_ctx.session_id);
        break;
    case NI_RETCODE_INVALID_PARAM:
        av_log(avctx, AV_LOG_ERROR,
               "Failed to open encoder (status = %d), invalid parameter values "
               "given: %s\n", ret, ctx->api_ctx.param_err_msg);
        ret = AVERROR_EXTERNAL;
        goto end;
    default:
        av_log(avctx, AV_LOG_ERROR,
               "Failed to open encoder (status = %d), resource unavailable\n",
               ret);
        ret = AVERROR_EXTERNAL;
        goto end;
    }

    xpkt = &(ctx->api_pkt.data.packet);
    ni_packet_buffer_alloc(xpkt, NI_MAX_TX_SZ);

    while (1) {
        recv = ni_device_session_read(&(ctx->api_ctx), &(ctx->api_pkt),
                                    NI_DEVICE_TYPE_ENCODER);

        if (recv > 0) {
            av_freep(&avctx->extradata);
            avctx->extradata_size = recv - (int)(ctx->api_ctx.meta_size);
            avctx->extradata =
                av_mallocz(avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
            memcpy(avctx->extradata,
                   (uint8_t *)xpkt->p_data + ctx->api_ctx.meta_size,
                   avctx->extradata_size);
            av_log(avctx, AV_LOG_VERBOSE, "Xcoder encoder headers len: %d\n",
                   avctx->extradata_size);
            break;
        }
    }

end:
    // close and clean up the temporary session
    if (ret != 0) {
        ni_device_session_close(&(ctx->api_ctx), ctx->encoder_eof,
                               NI_DEVICE_TYPE_ENCODER);
    } else {
        ret = ni_device_session_close(&(ctx->api_ctx), ctx->encoder_eof,
                                     NI_DEVICE_TYPE_ENCODER);
    }
#ifdef _WIN32
    ni_device_close(ctx->api_ctx.device_handle);
#elif __linux__
    ni_device_close(ctx->api_ctx.device_handle);
    ni_device_close(ctx->api_ctx.blk_io_handle);
#endif
    ctx->api_ctx.device_handle = NI_INVALID_DEVICE_HANDLE;
    ctx->api_ctx.blk_io_handle = NI_INVALID_DEVICE_HANDLE;

    ni_packet_buffer_free(&(ctx->api_pkt.data.packet));

    ni_rsrc_free_device_context(ctx->rsrc_ctx);
    ctx->rsrc_ctx = NULL;

    p_param->cfg_enc_params.conf_win_right  = orig_conf_win_right;
    p_param->cfg_enc_params.conf_win_bottom = orig_conf_win_bottom;

    av_freep(&ctx);

    return ret;
}

static int xcoder_encoder_header_check_set(AVCodecContext *avctx)
{
    XCoderEncContext *ctx = avctx->priv_data;
    ni_xcoder_params_t *p_param;
    // set color metrics
    enum AVColorPrimaries color_primaries = avctx->color_primaries;
    enum AVColorTransferCharacteristic color_trc = avctx->color_trc;
    enum AVColorSpace color_space = avctx->colorspace;

    p_param = (ni_xcoder_params_t *)ctx->api_ctx.p_session_config;

    if (5 == p_param->dolby_vision_profile) {
// Check libavocodec version, since 4.2.1 symlinks 4.3.1
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 91))
        switch (avctx->codec_id) {
        case AV_CODEC_ID_HEVC:
            color_primaries = AVCOL_PRI_UNSPECIFIED;
            color_trc       = AVCOL_TRC_UNSPECIFIED;
            color_space     = AVCOL_SPC_UNSPECIFIED;
            p_param->cfg_enc_params.hrdEnable =
                p_param->cfg_enc_params.EnableAUD        = 1;
            p_param->cfg_enc_params.forced_header_enable = 1;
            p_param->cfg_enc_params.videoFullRange       = 1;
            break;
        case AV_CODEC_ID_AV1:
            av_log(avctx, AV_LOG_ERROR,
                   "dolbyVisionProfile is not supported on av1 encoder.\n");
            return -1;
        case AV_CODEC_ID_MJPEG:
            av_log(avctx, AV_LOG_ERROR,
                   "dolbyVisionProfile is not supported on jpeg encoder.\n");
            return -1;
        case AV_CODEC_ID_H264:
            av_log(avctx, AV_LOG_ERROR,
                   "dolbyVisionProfile is not supported on h264 encoder.\n");
            return -1;
        default:
            break;
        }
#else
        av_log(avctx, AV_LOG_ERROR,
               "dolbyVisionProfile is not supported on FFmpeg version 4.2.1 "
               "and earlier.\n");
        return -1;
#endif
    }

    if (avctx->codec_id != AV_CODEC_ID_MJPEG &&
        ((5 == p_param->dolby_vision_profile &&
          AV_CODEC_ID_HEVC == avctx->codec_id) ||
         color_primaries != AVCOL_PRI_UNSPECIFIED ||
         color_trc != AVCOL_TRC_UNSPECIFIED ||
         color_space != AVCOL_SPC_UNSPECIFIED)) {
        p_param->cfg_enc_params.colorDescPresent = 1;
        p_param->cfg_enc_params.colorPrimaries   = color_primaries;
        p_param->cfg_enc_params.colorTrc         = color_trc;
        p_param->cfg_enc_params.colorSpace       = color_space;

        av_log(avctx, AV_LOG_VERBOSE,
               "XCoder HDR color info color_primaries: %d "
               "color_trc: %d  color_space %d\n",
               color_primaries, color_trc, color_space);
    }
    if (avctx->color_range == AVCOL_RANGE_JPEG ||
        AV_PIX_FMT_YUVJ420P == avctx->pix_fmt ||
        AV_PIX_FMT_YUVJ420P == avctx->sw_pix_fmt) {
        p_param->cfg_enc_params.videoFullRange = 1;
    }

    return 0;
}

static int xcoder_setup_encoder(AVCodecContext *avctx)
{
    XCoderEncContext *s = avctx->priv_data;
    int i, ret = 0;
    uint32_t  xcoder_timeout;
    ni_xcoder_params_t *p_param       = &s->api_param;
    ni_xcoder_params_t *pparams       = NULL;
    ni_session_run_state_t prev_state = s->api_ctx.session_run_state;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder setup device encoder\n");

    if (ni_device_session_context_init(&(s->api_ctx)) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Error XCoder init encoder context failure\n");
        return AVERROR_EXTERNAL;
    }

    switch (avctx->codec_id) {
    case AV_CODEC_ID_HEVC:
        s->api_ctx.codec_format = NI_CODEC_FORMAT_H265;
        break;
    case AV_CODEC_ID_AV1:
        s->api_ctx.codec_format = NI_CODEC_FORMAT_AV1;
        break;
    case AV_CODEC_ID_MJPEG:
        s->api_ctx.codec_format = NI_CODEC_FORMAT_JPEG;
        break;
    default:
        s->api_ctx.codec_format = NI_CODEC_FORMAT_H264;
        break;
    }

    s->api_ctx.session_run_state = prev_state;
    s->av_rois = NULL;
    s->firstPktArrived = 0;
    s->spsPpsArrived = 0;
    s->spsPpsHdrLen = 0;
    s->p_spsPpsHdr = NULL;
    s->xcode_load_pixel = 0;
    s->reconfigCount = 0;
    s->latest_dts = 0;
    s->first_frame_pts = INT_MIN;

    if (SESSION_RUN_STATE_SEQ_CHANGE_DRAINING != s->api_ctx.session_run_state) {
        av_log(avctx, AV_LOG_INFO, "Session state: %d allocate frame fifo.\n",
               s->api_ctx.session_run_state);
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        s->fme_fifo = av_fifo_alloc2((size_t) 1, sizeof(AVFrame), 0);
#else
        s->fme_fifo = av_fifo_alloc(sizeof(AVFrame));
#endif
    } else {
        av_log(avctx, AV_LOG_INFO, "Session seq change, fifo size: %lu.\n",
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
               av_fifo_can_read(s->fme_fifo));
#else
               av_fifo_size(s->fme_fifo) / sizeof(AVFrame));
#endif
    }

    if (!s->fme_fifo) {
        return AVERROR(ENOMEM);
    }
    s->eos_fme_received = 0;

    //Xcoder User Configuration
    ret = ni_encoder_init_default_params(
        p_param, avctx->framerate.num,
        avctx->framerate.den, avctx->bit_rate,
        avctx->width, avctx->height, s->api_ctx.codec_format);
    switch (ret) {
    case NI_RETCODE_PARAM_ERROR_WIDTH_TOO_BIG:
        if (avctx->codec_id == AV_CODEC_ID_AV1 && avctx->width < NI_PARAM_MAX_WIDTH) {
            // AV1 resolution will be checked again when encoder session open (ni_validate_custom_template) since crop size may meet AV1 resolution constraint (E.g. AV1 tile encode)
            av_log(avctx, AV_LOG_ERROR, "AV1 Picture Width exceeds %d - picture needs to be cropped:\n",
                   NI_PARAM_AV1_MAX_WIDTH);
            ret = NI_RETCODE_SUCCESS;
        } else {
            av_log(avctx, AV_LOG_ERROR, "Invalid Picture Width: too big\n");
            return AVERROR_EXTERNAL;
        }
        break;
    case NI_RETCODE_PARAM_ERROR_WIDTH_TOO_SMALL:
        av_log(avctx, AV_LOG_ERROR, "Invalid Picture Width: too small\n");
        return AVERROR_EXTERNAL;
    case NI_RETCODE_PARAM_ERROR_HEIGHT_TOO_BIG:
        if (avctx->codec_id == AV_CODEC_ID_AV1) {
            // AV1 resolution will be checked again when encoder session open (ni_validate_custom_template) since crop size may meet AV1 resolution constraint (E.g. AV1 tile encode)
            av_log(avctx, AV_LOG_ERROR, "AV1 Picture Height exceeds %d - picture needs to be cropped:\n",
                   NI_PARAM_AV1_MAX_HEIGHT);
            ret = NI_RETCODE_SUCCESS;
        } else {
            av_log(avctx, AV_LOG_ERROR, "Invalid Picture Height: too big\n");
            return AVERROR_EXTERNAL;
        }
        break;
    case NI_RETCODE_PARAM_ERROR_HEIGHT_TOO_SMALL:
        av_log(avctx, AV_LOG_ERROR, "Invalid Picture Height: too small\n");
        return AVERROR_EXTERNAL;
    case NI_RETCODE_PARAM_ERROR_AREA_TOO_BIG:
        if (avctx->codec_id == AV_CODEC_ID_AV1) {
            // AV1 resolution will be checked again when encoder session open (ni_validate_custom_template) since crop size may meet AV1 resolution constraint (E.g. AV1 tile encode)
            av_log(avctx, AV_LOG_ERROR, "AV1 Picture Width x Height exceeds %d - picture needs to be cropped:\n",
                   NI_PARAM_AV1_MAX_AREA);
            ret = NI_RETCODE_SUCCESS;
        } else {
            av_log(avctx, AV_LOG_ERROR,
                   "Invalid Picture Width x Height: exceeds %d\n",
                   NI_MAX_RESOLUTION_AREA);
            return AVERROR_EXTERNAL;
        }
        break;
    case NI_RETCODE_PARAM_ERROR_PIC_WIDTH:
        av_log(avctx, AV_LOG_ERROR, "Invalid Picture Width\n");
        return AVERROR_EXTERNAL;
    case NI_RETCODE_PARAM_ERROR_PIC_HEIGHT:
        av_log(avctx, AV_LOG_ERROR, "Invalid Picture Height\n");
        return AVERROR_EXTERNAL;
    default:
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "Error setting preset or log.\n");
            av_log(avctx, AV_LOG_INFO, "Possible presets:");
            for (i = 0; g_xcoder_preset_names[i]; i++)
                av_log(avctx, AV_LOG_INFO, " %s", g_xcoder_preset_names[i]);
            av_log(avctx, AV_LOG_INFO, "\n");

            av_log(avctx, AV_LOG_INFO, "Possible log:");
            for (i = 0; g_xcoder_log_names[i]; i++)
                av_log(avctx, AV_LOG_INFO, " %s", g_xcoder_log_names[i]);
            av_log(avctx, AV_LOG_INFO, "\n");

            return AVERROR(EINVAL);
        }
        break;
    }

    av_log(avctx, AV_LOG_INFO, "pix_fmt is %d, sw_pix_fmt is %d resolution %dx%d\n", avctx->pix_fmt, avctx->sw_pix_fmt, avctx->width, avctx->height);
    if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
        av_log(avctx, AV_LOG_INFO, "sw_pix_fmt assigned to pix_fmt was %d, is now %d\n", avctx->pix_fmt, avctx->sw_pix_fmt);
        avctx->sw_pix_fmt = avctx->pix_fmt;
    } else {
        if ((avctx->height >= NI_MIN_HEIGHT) && (avctx->width >= NI_MIN_WIDTH)) {
            p_param->hwframes = 1;
        } else if (avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
                   avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
            av_log(avctx, AV_LOG_ERROR, "Invalid Picture Height or Width: too small\n");
            return AVERROR_EXTERNAL;
        }

        if (avctx->codec_id == AV_CODEC_ID_MJPEG) {
            if (avctx->sw_pix_fmt == AV_PIX_FMT_YUVJ420P) {
                av_log(avctx, AV_LOG_DEBUG, "Pixfmt %s supported in %s encoder\n",
                       av_get_pix_fmt_name(avctx->sw_pix_fmt), avctx->codec->name);
            } else if ((avctx->color_range == AVCOL_RANGE_JPEG || avctx->color_range == AVCOL_RANGE_UNSPECIFIED) &&
                      (avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P || avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10LE ||
                       avctx->sw_pix_fmt == AV_PIX_FMT_NV12 || avctx->sw_pix_fmt == AV_PIX_FMT_P010LE)) {
                av_log(avctx, AV_LOG_DEBUG, "Pixfmt %s supported in %s encoder when color_range is AVCOL_RANGE_JPEG\n",
                       av_get_pix_fmt_name(avctx->sw_pix_fmt), avctx->codec->name);
            } else {
                av_log(avctx, AV_LOG_ERROR, "Pixfmt %s not supported in %s encoder when color_range is %d\n",
                       av_get_pix_fmt_name(avctx->sw_pix_fmt), avctx->codec->name, avctx->color_range);
                return AVERROR_INVALIDDATA;
            }
        }
    }

    switch (avctx->sw_pix_fmt) {
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUVJ420P:
        s->api_ctx.pixel_format = NI_PIX_FMT_YUV420P;
        break;
    case AV_PIX_FMT_YUV420P10LE:
        s->api_ctx.pixel_format = NI_PIX_FMT_YUV420P10LE;
        break;
    case AV_PIX_FMT_NV12:
        s->api_ctx.pixel_format = NI_PIX_FMT_NV12;
        break;
    case AV_PIX_FMT_P010LE:
        s->api_ctx.pixel_format = NI_PIX_FMT_P010LE;
        break;
    case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
        s->api_ctx.pixel_format = NI_PIX_FMT_8_TILED4X4;
        break;
    case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
        s->api_ctx.pixel_format = NI_PIX_FMT_10_TILED4X4;
        break;
    case AV_PIX_FMT_ARGB:
        s->api_ctx.pixel_format = NI_PIX_FMT_ARGB;
        break;
    case AV_PIX_FMT_ABGR:
        s->api_ctx.pixel_format = NI_PIX_FMT_ABGR;
        break;
    case AV_PIX_FMT_RGBA:
        s->api_ctx.pixel_format = NI_PIX_FMT_RGBA;
        break;
    case AV_PIX_FMT_BGRA:
        s->api_ctx.pixel_format = NI_PIX_FMT_BGRA;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Pixfmt %s not supported in Quadra encoder\n",
               av_get_pix_fmt_name(avctx->sw_pix_fmt));
        return AVERROR_INVALIDDATA;
    }

    if (s->xcoder_opts) {
        AVDictionary *dict = NULL;
        AVDictionaryEntry *en = NULL;

        if (!av_dict_parse_string(&dict, s->xcoder_opts, "=", ":", 0)) {
            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                int parse_ret = ni_encoder_params_set_value(p_param, en->key, en->value);
                if (parse_ret != NI_RETCODE_SUCCESS) {
                    switch (parse_ret) {
                    case NI_RETCODE_PARAM_INVALID_NAME:
                        av_log(avctx, AV_LOG_ERROR, "Unknown option: %s.\n", en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_TOO_BIG:
                        av_log(avctx, AV_LOG_ERROR, "Invalid %s: too big\n", en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_TOO_SMALL:
                        av_log(avctx, AV_LOG_ERROR, "Invalid %s: too small\n", en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_OOR:
                        av_log(avctx, AV_LOG_ERROR, "Invalid %s: out of range\n",
                               en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_ZERO:
                        av_log(avctx, AV_LOG_ERROR,
                               "Error setting option %s to value 0\n", en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_INVALID_VALUE:
                        av_log(avctx, AV_LOG_ERROR, "Invalid value for %s: %s.\n",
                               en->key, en->value);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_WARNING_DEPRECATED:
                        av_log(avctx, AV_LOG_WARNING, "Parameter %s is deprecated\n",
                               en->key);
                        break;
                    default:
                        av_log(avctx, AV_LOG_ERROR, "Invalid %s: ret %d\n", en->key,
                               parse_ret);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    }
                }
            }
            av_dict_free(&dict);
        }
    }

    if (p_param->enable_vfr) {
        // in the vfr mode, if the initial framerate is out of [5-120]
        // think the initial framerate is incorrect, set it to default 30 fps
        if (p_param->cfg_enc_params.frame_rate < 5 ||
            p_param->cfg_enc_params.frame_rate > 120) {
            p_param->cfg_enc_params.frame_rate = 30;
            s->api_ctx.prev_fps                = 30;
        } else {
            s->api_ctx.prev_fps = p_param->cfg_enc_params.frame_rate;
        }
        s->api_ctx.last_change_framenum    = 0;
        s->api_ctx.fps_change_detect_count = 0;
    }

    av_log(avctx, AV_LOG_DEBUG, "p_param->hwframes = %d\n", p_param->hwframes);
    if (s->xcoder_gop) {
        AVDictionary *dict = NULL;
        AVDictionaryEntry *en = NULL;

        if (!av_dict_parse_string(&dict, s->xcoder_gop, "=", ":", 0)) {
            if (!gop_params_check(dict, avctx)) {
                av_dict_free(&dict);
                return AVERROR_EXTERNAL;
            }

            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                int parse_ret = ni_encoder_gop_params_set_value(p_param, en->key, en->value);
                if (parse_ret != NI_RETCODE_SUCCESS) {
                    switch (parse_ret) {
                    case NI_RETCODE_PARAM_INVALID_NAME:
                        av_log(avctx, AV_LOG_ERROR, "Unknown option: %s.\n", en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_TOO_BIG:
                        av_log(avctx, AV_LOG_ERROR,
                               "Invalid custom GOP parameters: %s too big\n", en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_TOO_SMALL:
                        av_log(avctx, AV_LOG_ERROR,
                               "Invalid custom GOP parameters: %s too small\n",
                               en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_OOR:
                        av_log(avctx, AV_LOG_ERROR,
                               "Invalid custom GOP parameters: %s out of range \n",
                               en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_ERROR_ZERO:
                        av_log(avctx, AV_LOG_ERROR,
                               "Invalid custom GOP paramaters: Error setting option %s "
                               "to value 0 \n",
                               en->key);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_INVALID_VALUE:
                        av_log(avctx, AV_LOG_ERROR,
                               "Invalid value for GOP param %s: %s.\n", en->key,
                               en->value);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    case NI_RETCODE_PARAM_WARNING_DEPRECATED:
                        av_log(avctx, AV_LOG_WARNING, "Parameter %s is deprecated\n",
                               en->key);
                        break;
                    default:
                        av_log(avctx, AV_LOG_ERROR, "Invalid %s: ret %d\n", en->key,
                               parse_ret);
                        av_dict_free(&dict);
                        return AVERROR_EXTERNAL;
                    }
                }
            }
            av_dict_free(&dict);
        }
    }
    if (s->nvme_io_size > 0 && s->nvme_io_size % 4096 != 0) {
        av_log(avctx, AV_LOG_ERROR, "Error XCoder iosize is not 4KB aligned!\n");
        return AVERROR_EXTERNAL;
    }

    s->api_ctx.p_session_config = &s->api_param;
    pparams = (ni_xcoder_params_t *)s->api_ctx.p_session_config;
    switch (pparams->cfg_enc_params.gop_preset_index) {
    /* dtsOffset is the max number of non-reference frames in a GOP
      * (derived from x264/5 algo) In case of IBBBP the first dts of the I
      * frame should be input_pts-(3*ticks_per_frame) In case of IBP the
      * first dts of the I frame should be input_pts-(1*ticks_per_frame)
      * thus we ensure pts>dts in all cases
      * */
    case 1:
    case 9:
    case 10:
        s->dtsOffset = 0;
        break;
    /* ts requires dts/pts of I fraem not same when there are B frames in
      * streams */
    case 3:
    case 4:
    case 7:
        s->dtsOffset = 1;
        break;
    case 5:
        s->dtsOffset = 2;
        break;
    case -1: // adaptive GOP
    case 8:
        s->dtsOffset = 3;
        break;
    default:
        s->dtsOffset = 7;
        break;
    }

    if (pparams->cfg_enc_params.custom_gop_params.custom_gop_size) {
        int dts_offset = 0;
        s->dtsOffset   = 0;
        bool has_b_frame = false;
        for (int idx = 0;
             idx < pparams->cfg_enc_params.custom_gop_params.custom_gop_size;
             idx++) {
            if (pparams->cfg_enc_params.custom_gop_params.pic_param[idx].poc_offset <
                idx + 1) {
                dts_offset = (idx + 1) -
                            pparams->cfg_enc_params.custom_gop_params.pic_param[idx].
                            poc_offset;
                if (s->dtsOffset < dts_offset) {
                    s->dtsOffset = dts_offset;
                }
            }

            if (!has_b_frame &&
                (pparams->cfg_enc_params.custom_gop_params.pic_param[idx].pic_type ==
                 PIC_TYPE_B)) {
                has_b_frame = true;
            }
        }

        if (has_b_frame && !s->dtsOffset) {
            s->dtsOffset = 1;
        }
    }
    av_log(avctx, AV_LOG_VERBOSE, "dts offset set to %ld\n", s->dtsOffset);

    s->total_frames_received = 0;
    s->gop_offset_count = 0;
    av_log(avctx, AV_LOG_INFO, "dts offset: %ld, gop_offset_count: %d\n",
           s->dtsOffset, s->gop_offset_count);

    //overwrite the nvme io size here with a custom value if it was provided
    if (s->nvme_io_size > 0) {
        s->api_ctx.max_nvme_io_size = s->nvme_io_size;
        av_log(avctx, AV_LOG_VERBOSE, "Custom NVME IO Size set to = %u\n",
               s->api_ctx.max_nvme_io_size);
        av_log(avctx, AV_LOG_INFO, "Encoder user specified NVMe IO Size set to: %u\n",
               s->api_ctx.max_nvme_io_size);
    }

    // overwrite keep alive timeout value here with a custom value if it was
    // provided
    // if xcoder option is set then overwrite the (legacy) decoder option
    xcoder_timeout = s->api_param.cfg_enc_params.keep_alive_timeout;
    if (xcoder_timeout != NI_DEFAULT_KEEP_ALIVE_TIMEOUT) {
        s->api_ctx.keep_alive_timeout = xcoder_timeout;
    } else {
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
    }
    av_log(avctx, AV_LOG_VERBOSE, "Custom NVME Keep Alive Timeout set to = %d\n",
           s->api_ctx.keep_alive_timeout);

    s->encoder_eof = 0;
    avctx->bit_rate = pparams->bitrate;

    s->api_ctx.src_bit_depth = 8;
    s->api_ctx.src_endian = NI_FRAME_LITTLE_ENDIAN;
    s->api_ctx.roi_len = 0;
    s->api_ctx.roi_avg_qp = 0;
    s->api_ctx.bit_depth_factor = 1;
    if (AV_PIX_FMT_YUV420P10BE == avctx->sw_pix_fmt ||
        AV_PIX_FMT_YUV420P10LE == avctx->sw_pix_fmt ||
        AV_PIX_FMT_P010LE == avctx->sw_pix_fmt ||
        AV_PIX_FMT_NI_QUAD_10_TILE_4X4 == avctx->sw_pix_fmt) {
        s->api_ctx.bit_depth_factor = 2;
        s->api_ctx.src_bit_depth = 10;
        if (AV_PIX_FMT_YUV420P10BE == avctx->sw_pix_fmt) {
            s->api_ctx.src_endian = NI_FRAME_BIG_ENDIAN;
        }
    }
    switch (avctx->sw_pix_fmt) {
        case AV_PIX_FMT_NV12:
        case AV_PIX_FMT_P010LE:
            pparams->cfg_enc_params.planar = NI_PIXEL_PLANAR_FORMAT_SEMIPLANAR;
            break;
        case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
        case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
            pparams->cfg_enc_params.planar = NI_PIXEL_PLANAR_FORMAT_TILED4X4;
            break;
        default:
            pparams->cfg_enc_params.planar = NI_PIXEL_PLANAR_FORMAT_PLANAR;
            break;
    }

    if (1) {
        s->freeHead = 0;
        s->freeTail = 0;
        for (i = 0; i < MAX_NUM_FRAMEPOOL_HWAVFRAME; i++) {
            s->sframe_pool[i] = av_frame_alloc();
            if (!s->sframe_pool[i]) {
                return AVERROR(ENOMEM);
            }
            s->aFree_Avframes_list[i] = i;
            s->freeTail++;
        }
        s->aFree_Avframes_list[i] = -1;
    }

    // init HDR SEI stuff
    s->api_ctx.sei_hdr_content_light_level_info_len =
        s->api_ctx.light_level_data_len =
        s->api_ctx.sei_hdr_mastering_display_color_vol_len =
        s->api_ctx.mdcv_max_min_lum_data_len = 0;
    s->api_ctx.p_master_display_meta_data = NULL;

    memset( &(s->api_fme), 0, sizeof(ni_session_data_io_t) );
    memset( &(s->api_pkt), 0, sizeof(ni_session_data_io_t) );

    s->api_pkt.data.packet.av1_buffer_index = 0;

    //validate encoded bitstream headers struct for encoder open
    if (xcoder_encoder_header_check_set(avctx) < 0) {
        return AVERROR_EXTERNAL;
    }

    // aspect ratio
    // Use the value passed in from FFmpeg if aspect ratio from xcoder-params have default values
    if ((p_param->cfg_enc_params.aspectRatioWidth == 0) && (p_param->cfg_enc_params.aspectRatioHeight == 1)) {
        p_param->cfg_enc_params.aspectRatioWidth  = avctx->sample_aspect_ratio.num;
        p_param->cfg_enc_params.aspectRatioHeight = avctx->sample_aspect_ratio.den;
    }

    // generate encoded bitstream headers in advance if configured to do so
    if ((avctx->codec_id != AV_CODEC_ID_MJPEG) &&
        (pparams->generate_enc_hdrs || (s->gen_global_headers == GEN_GLOBAL_HEADERS_ON) || // pparams->generate_enc_hdrs will be deprecated in v5.3.0
         ((avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) &&
          (s->gen_global_headers == GEN_GLOBAL_HEADERS_AUTO)))) {
        ret = xcoder_encoder_headers(avctx);
    }

    // original resolution this stream started with, this is used by encoder sequence change
    s->api_ctx.ori_width = avctx->width;
    s->api_ctx.ori_height = avctx->height;
    s->api_ctx.ori_bit_depth_factor = s->api_ctx.bit_depth_factor;
    s->api_ctx.ori_pix_fmt = s->api_ctx.pixel_format;

    av_log(avctx, AV_LOG_INFO, "xcoder_setup_encoder "
           "sw_pix_fmt %d ori_pix_fmt %d\n",
           avctx->sw_pix_fmt, s->api_ctx.ori_pix_fmt);

    s->api_ctx.ori_luma_linesize = 0;
    s->api_ctx.ori_chroma_linesize = 0;

    return ret;
}

av_cold int ff_xcoder_encode_init(AVCodecContext *avctx)
{
    XCoderEncContext *ctx = avctx->priv_data;
    AVHWFramesContext *avhwf_ctx;
    int ret;
    ni_log_set_level(ff_to_ni_log_level(av_log_get_level()));

    av_log(avctx, AV_LOG_VERBOSE, "XCoder encode init\n");

    if (ctx->api_ctx.session_run_state == SESSION_RUN_STATE_SEQ_CHANGE_DRAINING) {
        ctx->dev_enc_idx = ctx->orig_dev_enc_idx;
    } else {
        ctx->orig_dev_enc_idx = ctx->dev_enc_idx;
    }

    if ((ret = xcoder_setup_encoder(avctx)) < 0) {
        ff_xcoder_encode_close(avctx);
        return ret;
    }

    if (!avctx->hw_device_ctx) {
        if (avctx->hw_frames_ctx) {
            avhwf_ctx = (AVHWFramesContext *)avctx->hw_frames_ctx->data;
            avctx->hw_device_ctx = av_buffer_ref(avhwf_ctx->device_ref);
        }
    }

    return 0;
}

int ff_xcoder_encode_close(AVCodecContext *avctx)
{
    XCoderEncContext *ctx = avctx->priv_data;
    ni_retcode_t ret = NI_RETCODE_FAILURE;
    int i;

    for (i = 0; i < MAX_NUM_FRAMEPOOL_HWAVFRAME; i++) {
        av_frame_free(&(ctx->sframe_pool[i])); //any remaining stored AVframes that have not been unref will die here
        ctx->sframe_pool[i] = NULL;
    }

    ret = ni_device_session_close(&ctx->api_ctx, ctx->encoder_eof,
                                  NI_DEVICE_TYPE_ENCODER);
    if (NI_RETCODE_SUCCESS != ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to close Encoder Session (status = %d)\n", ret);
    }

    av_log(avctx, AV_LOG_VERBOSE, "XCoder encode close: session_run_state %d\n", ctx->api_ctx.session_run_state);
    if (ctx->api_ctx.session_run_state != SESSION_RUN_STATE_SEQ_CHANGE_DRAINING) {
        av_log(avctx, AV_LOG_VERBOSE, "XCoder encode close: close blk_io_handle %d device_handle %d\n", ctx->api_ctx.blk_io_handle, ctx->api_ctx.device_handle);
#ifdef _WIN32
        ni_device_close(ctx->api_ctx.device_handle);
#elif __linux__
        ni_device_close(ctx->api_ctx.device_handle);
        ni_device_close(ctx->api_ctx.blk_io_handle);
#endif
        ctx->api_ctx.device_handle = NI_INVALID_DEVICE_HANDLE;
        ctx->api_ctx.blk_io_handle = NI_INVALID_DEVICE_HANDLE;
        ctx->api_ctx.auto_dl_handle = NI_INVALID_DEVICE_HANDLE;
        ctx->api_ctx.sender_handle = NI_INVALID_DEVICE_HANDLE;
    }

    av_log(avctx, AV_LOG_VERBOSE, "XCoder encode close (status = %d)\n", ret);

    if (ctx->api_fme.data.frame.buffer_size
        || ctx->api_fme.data.frame.metadata_buffer_size
        || ctx->api_fme.data.frame.start_buffer_size) {
        ni_frame_buffer_free(&(ctx->api_fme.data.frame));
    }
    ni_packet_buffer_free(&(ctx->api_pkt.data.packet));
    if (AV_CODEC_ID_AV1 == avctx->codec_id &&
        ctx->api_pkt.data.packet.av1_buffer_index)
        ni_packet_buffer_free_av1(&(ctx->api_pkt.data.packet));

    av_log(avctx, AV_LOG_DEBUG, "fifo num frames: %lu\n",
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
           av_fifo_can_read(ctx->fme_fifo));
#else
           av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame));
#endif
    if (ctx->api_ctx.session_run_state != SESSION_RUN_STATE_SEQ_CHANGE_DRAINING) {
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        av_fifo_freep2(&ctx->fme_fifo);
#else
        if (ctx->fme_fifo)
            av_fifo_free(ctx->fme_fifo);
        ctx->fme_fifo = NULL;
#endif
        av_log(avctx, AV_LOG_DEBUG, " , freed.\n");
    } else {
        av_log(avctx, AV_LOG_DEBUG, " , kept.\n");
    }

    ni_device_session_context_clear(&ctx->api_ctx);

    ni_rsrc_free_device_context(ctx->rsrc_ctx);
    ctx->rsrc_ctx = NULL;

    ni_memfree(ctx->av_rois);
    av_freep(&ctx->p_spsPpsHdr);

    if (avctx->hw_device_ctx) {
        av_buffer_unref(&avctx->hw_device_ctx);
    }
    ctx->started = 0;

    return 0;
}

static int xcoder_encode_sequence_change(AVCodecContext *avctx, int width, int height, int bit_depth_factor)
{
    XCoderEncContext *ctx = avctx->priv_data;
    ni_retcode_t ret = NI_RETCODE_FAILURE;
    ni_xcoder_params_t *p_param = &ctx->api_param;
    ni_xcoder_params_t *pparams = (ni_xcoder_params_t *)ctx->api_ctx.p_session_config;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder encode sequence change: session_run_state %d\n", ctx->api_ctx.session_run_state);

    ret = ni_device_session_sequence_change(&ctx->api_ctx, width, height, bit_depth_factor, NI_DEVICE_TYPE_ENCODER);

    if (NI_RETCODE_SUCCESS != ret) {
        av_log(avctx, AV_LOG_ERROR, "Failed to send Sequence Change to Encoder Session (status = %d)\n", ret);
        return ret;
    }

    // update AvCodecContext
    if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
        av_log(avctx, AV_LOG_INFO, "sw_pix_fmt assigned to pix_fmt was %d, is now %d\n", avctx->pix_fmt, avctx->sw_pix_fmt);
        avctx->sw_pix_fmt = avctx->pix_fmt;
    } else {
        if ((avctx->height >= NI_MIN_HEIGHT) && (avctx->width >= NI_MIN_WIDTH)) {
            p_param->hwframes = 1;
        }
    }

    switch (avctx->sw_pix_fmt) {
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUVJ420P:
        case AV_PIX_FMT_YUV420P10LE:
        case AV_PIX_FMT_NV12:
        case AV_PIX_FMT_P010LE:
        case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
        case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
        case AV_PIX_FMT_ARGB:
        case AV_PIX_FMT_ABGR:
        case AV_PIX_FMT_RGBA:
        case AV_PIX_FMT_BGRA:
            break;
        case AV_PIX_FMT_YUV420P12:
        case AV_PIX_FMT_YUV422P:
        case AV_PIX_FMT_YUV422P10:
        case AV_PIX_FMT_YUV422P12:
        case AV_PIX_FMT_GBRP:
        case AV_PIX_FMT_GBRP10:
        case AV_PIX_FMT_GBRP12:
        case AV_PIX_FMT_YUV444P:
        case AV_PIX_FMT_YUV444P10:
        case AV_PIX_FMT_YUV444P12:
        case AV_PIX_FMT_GRAY8:
        case AV_PIX_FMT_GRAY10:
        case AV_PIX_FMT_GRAY12:
        default:
            return AVERROR_INVALIDDATA;
            break;
    }

    // update session context
    ctx->api_ctx.bit_depth_factor = bit_depth_factor;
    ctx->api_ctx.src_bit_depth = (bit_depth_factor == 1) ? 8 : 10;
    ctx->api_ctx.src_endian = (AV_PIX_FMT_YUV420P10BE == avctx->sw_pix_fmt) ? NI_FRAME_BIG_ENDIAN : NI_FRAME_LITTLE_ENDIAN;
    ctx->api_ctx.ready_to_close = 0;
    ctx->api_ctx.frame_num = 0; // need to reset frame_num because pkt_num is set to 1 when header received after sequnce change, and low delay mode compares frame_num and pkt_num
    ctx->api_ctx.pkt_num = 0; // also need to reset pkt_num because before header received, pkt_num > frame_num will also cause low delay mode stuck
    ctx->api_pkt.data.packet.end_of_stream = 0;

    switch (avctx->sw_pix_fmt) {
        case AV_PIX_FMT_NV12:
        case AV_PIX_FMT_P010LE:
            pparams->cfg_enc_params.planar = NI_PIXEL_PLANAR_FORMAT_SEMIPLANAR;
            break;
        case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
        case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
            pparams->cfg_enc_params.planar = NI_PIXEL_PLANAR_FORMAT_TILED4X4;
            break;
        default:
            pparams->cfg_enc_params.planar = NI_PIXEL_PLANAR_FORMAT_PLANAR;
            break;
    }
    return ret;
}

static int xcoder_encode_reset(AVCodecContext *avctx)
{
    av_log(avctx, AV_LOG_WARNING, "XCoder encode reset\n");
    ff_xcoder_encode_close(avctx);
    return ff_xcoder_encode_init(avctx);
}

// frame fifo operations
static int is_input_fifo_empty(XCoderEncContext *s)
{
    if (!s->fme_fifo) {
        return 1;
    }
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
    return av_fifo_can_read(s->fme_fifo) ? 0 : 1;
#else
    return av_fifo_size(s->fme_fifo) < sizeof(AVFrame);
#endif
}

static int enqueue_frame(AVCodecContext *avctx, const AVFrame *inframe)
{
    XCoderEncContext *ctx = avctx->priv_data;
    size_t nb_elems;
    int ret = 0;

    // expand frame buffer fifo if not enough space
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
    if (!av_fifo_can_write(ctx->fme_fifo)) {
#else
    if (av_fifo_space(ctx->fme_fifo) < sizeof(AVFrame)) {
#endif
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        if (av_fifo_can_read(ctx->fme_fifo) >= NI_MAX_FIFO_CAPACITY) {
            av_log(avctx, AV_LOG_ERROR, "Encoder frame buffer fifo capacity (%lu) reached maximum (%d)\n",
                   av_fifo_can_read(ctx->fme_fifo), NI_MAX_FIFO_CAPACITY);
            return AVERROR_EXTERNAL;
#else
        if (av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame) >= NI_MAX_FIFO_CAPACITY) {
            av_log(avctx, AV_LOG_ERROR, "Encoder frame buffer fifo capacity (%lu) reached maximum (%d)\n",
                   av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame), NI_MAX_FIFO_CAPACITY);
            return AVERROR_EXTERNAL;
#endif
        }

#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        ret = av_fifo_grow2(ctx->fme_fifo, (size_t) 1);
#else
        ret = av_fifo_grow(ctx->fme_fifo, sizeof(AVFrame));
#endif
        if (ret < 0) {
            av_log(avctx, AV_LOG_ERROR, "Cannot grow FIFO: out of memory\n");
            return ret;
        }

#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        nb_elems = av_fifo_can_read(ctx->fme_fifo) + av_fifo_can_write(ctx->fme_fifo);
#else
        nb_elems = (av_fifo_size(ctx->fme_fifo) + av_fifo_space(ctx->fme_fifo)) / sizeof(AVFrame);
#endif
        if ((nb_elems % 100) == 0) {
            av_log(avctx, AV_LOG_INFO, "Enc fifo being extended to: %lu\n", nb_elems);
        }
#if (LIBAVUTIL_VERSION_MAJOR < 59)
        av_assert0(0 == (av_fifo_size(ctx->fme_fifo) + av_fifo_space(ctx->fme_fifo)) % sizeof(AVFrame));
#endif
    }

    if (inframe == &ctx->buffered_fme) {
        // For FFmpeg-n4.4+ receive_packet interface the buffered_fme is fetched from
        // ff_alloc_get_frame rather than passed as function argument. So we need to
        // judge whether they are the same object. If they are the same NO need to do
        // any reference before queue operation.
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        av_fifo_write(ctx->fme_fifo, (void *)inframe, (size_t) 1);
#else
        av_fifo_generic_write(ctx->fme_fifo, (void *)inframe, sizeof(*inframe), NULL);
#endif
    } else {
        AVFrame temp_frame;
        memset(&temp_frame, 0, sizeof(AVFrame));
        // In case double free for external input frame and our buffered frame.
        av_frame_ref(&temp_frame, inframe);
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        av_fifo_write(ctx->fme_fifo, &temp_frame, 1);
#else
        av_fifo_generic_write(ctx->fme_fifo, &temp_frame, sizeof(*inframe), NULL);
#endif
    }

    av_log(avctx, AV_LOG_DEBUG, "fme queued, fifo num frames: %lu\n",
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
           av_fifo_can_read(ctx->fme_fifo));
#else
           av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame));
#endif
    return ret;
}

int ff_xcoder_send_frame(AVCodecContext *avctx, const AVFrame *frame)
{
    XCoderEncContext *ctx = avctx->priv_data;
    bool ishwframe;
    bool isnv12frame;
    bool alignment_2pass_wa;
    int format_in_use;
    int ret = 0;
    int sent;
    int i, j;
    int orig_avctx_width = avctx->width;
    int orig_avctx_height = avctx->height;
    ni_xcoder_params_t *p_param;
    int need_to_copy = 1;
    AVHWFramesContext *avhwf_ctx;
    AVNIFramesContext *nif_src_ctx;
    AVFrameSideData *side_data;
    AVNetintGeneralSideData *ni_side_data;
    const AVFrame *first_frame = NULL;
    // employ a ni_frame_t as a data holder to convert/prepare for side data
    // of the passed in frame
    ni_frame_t dec_frame    = {0};
    ni_aux_data_t *aux_data = NULL;
    // data buffer for various SEI: HDR mastering display color volume, HDR
    // content light level, close caption, User data unregistered, HDR10+ etc.
    int send_sei_with_idr;
    uint8_t mdcv_data[NI_MAX_SEI_DATA];
    uint8_t cll_data[NI_MAX_SEI_DATA];
    uint8_t cc_data[NI_MAX_SEI_DATA];
    uint8_t udu_data[NI_MAX_SEI_DATA];
    uint8_t hdrp_data[NI_MAX_SEI_DATA];
    ni_timecode_t timecode = {0};
    int timecode_found = 0;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder send frame\n");

    p_param = (ni_xcoder_params_t *) ctx->api_ctx.p_session_config;
    alignment_2pass_wa = ((p_param->cfg_enc_params.lookAheadDepth ||
                           p_param->cfg_enc_params.crf >= 0 ||
                           p_param->cfg_enc_params.crfFloat >= 0) &&
                          (avctx->codec_id == AV_CODEC_ID_HEVC ||
                           avctx->codec_id == AV_CODEC_ID_AV1));

    // leave encoder instance open to when the first frame buffer arrives so that
    // its stride size is known and handled accordingly.
    if (ctx->started == 0) {
        if (!is_input_fifo_empty(ctx)) {
            av_log(avctx, AV_LOG_VERBOSE, "first frame: use fme from fifo peek\n");
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
            av_fifo_peek(ctx->fme_fifo, &ctx->buffered_fme, (size_t) 1, NULL);
#else
            av_fifo_generic_peek(ctx->fme_fifo, &ctx->buffered_fme, sizeof(AVFrame),
                               NULL);
#endif
            ctx->buffered_fme.extended_data = ctx->buffered_fme.data;
            first_frame = &ctx->buffered_fme;

        } else if (frame) {
            av_log(avctx, AV_LOG_VERBOSE, "first frame: use input frame\n");
            first_frame = frame;
        } else {
            av_log(avctx, AV_LOG_ERROR, "first frame: NULL is unexpected!\n");
        }
    } else if (ctx->api_ctx.session_run_state == SESSION_RUN_STATE_SEQ_CHANGE_OPENING) {
        if (!is_input_fifo_empty(ctx)) {
            av_log(avctx, AV_LOG_VERBOSE, "first frame: use fme from fifo peek\n");
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
            av_fifo_peek(ctx->fme_fifo, &ctx->buffered_fme, (size_t) 1, NULL);
#else
            av_fifo_generic_peek(ctx->fme_fifo, &ctx->buffered_fme, sizeof(AVFrame),
                               NULL);
#endif
            ctx->buffered_fme.extended_data = ctx->buffered_fme.data;
            first_frame = &ctx->buffered_fme;
        } else {
            av_log(avctx, AV_LOG_ERROR, "No buffered frame - Sequence Change Fail");
            ret = AVERROR_EXTERNAL;
            return ret;
        }
    }

    if (first_frame && ctx->started == 0) {
        // if frame stride size is not as we expect it,
        // adjust using xcoder-params conf_win_right
        int linesize_aligned = first_frame->width;
        int height_aligned = first_frame->height;
        ishwframe = first_frame->format == AV_PIX_FMT_NI_QUAD;

        if (linesize_aligned < NI_MIN_WIDTH) {
            p_param->cfg_enc_params.conf_win_right +=
                (NI_MIN_WIDTH - first_frame->width) / 2 * 2;
            linesize_aligned = NI_MIN_WIDTH;
        } else {
            if (avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
                avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
                linesize_aligned = FFALIGN(first_frame->width, 4);
                p_param->cfg_enc_params.conf_win_right +=
                    (linesize_aligned - first_frame->width) / 2 * 2;
            } else {
                linesize_aligned = FFALIGN(first_frame->width, 2);
                p_param->cfg_enc_params.conf_win_right +=
                    (linesize_aligned - first_frame->width) / 2 * 2;
            }
        }
        p_param->source_width = linesize_aligned;

        if (height_aligned < NI_MIN_HEIGHT) {
            p_param->cfg_enc_params.conf_win_bottom +=
                (NI_MIN_HEIGHT - first_frame->height) / 2 * 2;
            height_aligned = NI_MIN_HEIGHT;
        } else {
            if (avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
                avctx->sw_pix_fmt == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
                height_aligned = FFALIGN(first_frame->height, 4);
                p_param->cfg_enc_params.conf_win_bottom +=
                    (height_aligned - first_frame->height) / 4 * 4;
            } else {
                height_aligned = FFALIGN(first_frame->height, 2);
                p_param->cfg_enc_params.conf_win_bottom +=
                    (height_aligned - first_frame->height) / 2 * 2;
            }
        }
        p_param->source_height = height_aligned;

        av_log(avctx, AV_LOG_DEBUG,
               "color primaries (%u %u) colorspace (%u %u) color_range (%u %u)\n",
               avctx->color_primaries, first_frame->color_primaries,
               avctx->colorspace, first_frame->colorspace,
               avctx->color_range, first_frame->color_range);

        if (avctx->color_primaries == AVCOL_PRI_UNSPECIFIED) {
            avctx->color_primaries = first_frame->color_primaries;
        }
        if (avctx->color_trc == AVCOL_TRC_UNSPECIFIED) {
            avctx->color_trc = first_frame->color_trc;
        }
        if (avctx->colorspace == AVCOL_SPC_UNSPECIFIED) {
            avctx->colorspace = first_frame->colorspace;
        }
        avctx->color_range = first_frame->color_range;

        if (xcoder_encoder_header_check_set(avctx) < 0) {
            return AVERROR_EXTERNAL;
        }

        av_log(avctx, AV_LOG_VERBOSE,
               "XCoder frame->linesize: %d/%d/%d frame width/height %dx%d"
               " conf_win_right %d  conf_win_bottom %d , color primaries %u trc %u "
               "space %u format %d\n",
               first_frame->linesize[0], first_frame->linesize[1],
               first_frame->linesize[2], first_frame->width, first_frame->height,
               p_param->cfg_enc_params.conf_win_right,
               p_param->cfg_enc_params.conf_win_bottom,
               first_frame->color_primaries, first_frame->color_trc,
               first_frame->colorspace, first_frame->format);

        if (SESSION_RUN_STATE_SEQ_CHANGE_OPENING != ctx->api_ctx.session_run_state) {
            // sequence change backup / restore encoder device handles, hw_id and
            // block device name, so no need to overwrite hw_id/blk_dev_name to user
            // set values
            ctx->api_ctx.hw_id = ctx->dev_enc_idx;
            ff_xcoder_strncpy(ctx->api_ctx.dev_xcoder_name, ctx->dev_xcoder,
                              MAX_CHAR_IN_DEVICE_NAME);

            ff_xcoder_strncpy(ctx->api_ctx.blk_dev_name, ctx->dev_blk_name,
                              NI_MAX_DEVICE_NAME_LEN);
        }

        // set enable_acq_limit=1 by default for FFmpeg>=7.0
#if IS_FFMPEG_61_AND_ABOVE_FOR_LIBAVCODEC
        p_param->cfg_enc_params.enable_acq_limit = 1;
#endif
        p_param->rootBufId = (ishwframe) ? ((niFrameSurface1_t*)((uint8_t*)first_frame->data[3]))->ui16FrameIdx : 0;
        if (ishwframe) {
            ctx->api_ctx.hw_action = NI_CODEC_HW_ENABLE;
            ctx->api_ctx.sender_handle = (ni_device_handle_t)(
                (int64_t)(((niFrameSurface1_t *)((uint8_t *)first_frame->data[3]))
                            ->device_handle));
        }

        if (first_frame->hw_frames_ctx && ctx->api_ctx.hw_id == -1 &&
            0 == strcmp(ctx->api_ctx.blk_dev_name, "")) {
            ctx->api_ctx.hw_id = ni_get_cardno(first_frame);
            av_log(avctx, AV_LOG_VERBOSE,
                   "ff_xcoder_send_frame: hw_id -1, empty blk_dev_name, collocated "
                   "to %d\n",
                   ctx->api_ctx.hw_id);
        }

        // AUD insertion has to be handled differently in the firmware
        // if it is global header
        if (p_param->cfg_enc_params.EnableAUD) {
            if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) {
                p_param->cfg_enc_params.EnableAUD = NI_ENABLE_AUD_FOR_GLOBAL_HEADER;
            }

            av_log(avctx, AV_LOG_VERBOSE,
                   "%s: EnableAUD %d global header flag %d\n", __FUNCTION__,
                   (p_param->cfg_enc_params.EnableAUD),
                   (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) ? 1 : 0);
        }

        // config linesize for zero copy (if input resolution is zero copy compatible)
        ni_encoder_frame_zerocopy_check(&ctx->api_ctx,
            p_param, first_frame->width, first_frame->height,
            first_frame->linesize, true);

        ret = ni_device_session_open(&ctx->api_ctx, NI_DEVICE_TYPE_ENCODER);

        // As the file handle may change we need to assign back
        ctx->dev_xcoder_name = ctx->api_ctx.dev_xcoder_name;
        ctx->blk_xcoder_name = ctx->api_ctx.blk_xcoder_name;
        ctx->dev_enc_idx = ctx->api_ctx.hw_id;

        switch (ret) {
        case NI_RETCODE_SUCCESS:
            av_log(avctx, AV_LOG_VERBOSE,
                   "XCoder %s.%d (inst: %d) opened successfully\n",
                   ctx->dev_xcoder_name, ctx->dev_enc_idx, ctx->api_ctx.session_id);
            break;
        case NI_RETCODE_INVALID_PARAM:
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to open encoder (status = %d), invalid parameter values "
                   "given: %s\n",
                   ret, ctx->api_ctx.param_err_msg);
            ret = AVERROR_EXTERNAL;
            return ret;
        default:
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to open encoder (status = %d), "
                   "resource unavailable\n",
                   ret);
            // for FFmpeg >= 6.1 sequence change session open fail: unlike previous
            // FFmpeg versions which terminate streams and codecs right away
            // by calling exit_program after submit_encode_frame returns error,
            // FFmpeg 6.1 calls enc_flush after error, which enters this function again,
            // but the buffered frame would have been unref'ed by then, and therefore
            // we must remove buffered frame upon session open fail to prevent
            // accessing or unref'ing invalid frame
            if (SESSION_RUN_STATE_SEQ_CHANGE_DRAINING !=
                ctx->api_ctx.session_run_state) {
                if (! is_input_fifo_empty(ctx)) {
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
                    av_fifo_drain2(ctx->fme_fifo, (size_t) 1);
                    av_log(avctx, AV_LOG_DEBUG, "fme popped, fifo num frames: %lu\n",
                           av_fifo_can_read(ctx->fme_fifo));
#else
                    av_fifo_drain(ctx->fme_fifo, sizeof(AVFrame));
                    av_log(avctx, AV_LOG_DEBUG, "fme popped, fifo num frames: %lu\n",
                           av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame));
#endif
                }
            }
            ret = AVERROR_EXTERNAL;
            return ret;
        }

        // set up ROI map if in ROI demo mode
        // Note: this is for demo purpose, and its direct access to QP map in
        //       session context is not the usual way to do ROI; the normal way is
        //       through side data of AVFrame in libavcodec, or aux data of ni_frame
        //       in libxcoder
        if (p_param->cfg_enc_params.roi_enable &&
            (1 == p_param->roi_demo_mode || 2 == p_param->roi_demo_mode)) {
            if (ni_set_demo_roi_map(&ctx->api_ctx) < 0) {
                return AVERROR(ENOMEM);
            }
        }
    } //end if (first_frame && ctx->started == 0)

    if (ctx->encoder_flushing) {
        if (! frame && is_input_fifo_empty(ctx)) {
            av_log(avctx, AV_LOG_DEBUG, "XCoder EOF: null frame && fifo empty\n");
            return AVERROR_EOF;
        }
    }

    if (! frame) {
        if (is_input_fifo_empty(ctx)) {
            ctx->eos_fme_received = 1;
            av_log(avctx, AV_LOG_DEBUG, "null frame, eos_fme_received = 1\n");
        } else {
            avctx->internal->draining = 0;
            av_log(avctx, AV_LOG_DEBUG, "null frame, but fifo not empty, clear draining = 0\n");
        }
    } else {
        av_log(avctx, AV_LOG_DEBUG, "XCoder send frame #%"PRIu64"\n",
               ctx->api_ctx.frame_num);

        // queue up the frame if fifo is NOT empty, or: sequence change ongoing !
        if (! is_input_fifo_empty(ctx) ||
            SESSION_RUN_STATE_SEQ_CHANGE_DRAINING == ctx->api_ctx.session_run_state) {
            ret = enqueue_frame(avctx, frame);
            if (ret < 0) {
                return ret;
            }

            if (SESSION_RUN_STATE_SEQ_CHANGE_DRAINING ==
                ctx->api_ctx.session_run_state) {
                av_log(avctx, AV_LOG_TRACE, "XCoder doing sequence change, frame #%"PRIu64" "
                       "queued and return 0 !\n", ctx->api_ctx.frame_num);
                return 0;
            }
        } else if (frame != &ctx->buffered_fme) {
            // For FFmpeg-n4.4+ receive_packet interface the buffered_fme is fetched from
            // ff_alloc_get_frame rather than passed as function argument. So we need to
            // judge whether they are the same object. If they are the same NO need to
            // reference.
            ret = av_frame_ref(&ctx->buffered_fme, frame);
        }
    }

resend:

    if (ctx->started == 0) {
        ctx->api_fme.data.frame.start_of_stream = 1;
        ctx->started = 1;
    } else if (ctx->api_ctx.session_run_state == SESSION_RUN_STATE_SEQ_CHANGE_OPENING) {
        ctx->api_fme.data.frame.start_of_stream = 1;
    } else {
        ctx->api_fme.data.frame.start_of_stream = 0;
    }

    if (is_input_fifo_empty(ctx)) {
        av_log(avctx, AV_LOG_DEBUG,
               "no frame in fifo to send, just send/receive ..\n");
        if (ctx->eos_fme_received) {
            av_log(avctx, AV_LOG_DEBUG,
                   "no frame in fifo to send, send eos ..\n");
        }
    } else {
        av_log(avctx, AV_LOG_DEBUG, "fifo peek fme\n");
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
        av_fifo_peek(ctx->fme_fifo, &ctx->buffered_fme, (size_t) 1, NULL);
#else
        av_fifo_generic_peek(ctx->fme_fifo, &ctx->buffered_fme, sizeof(AVFrame),
                             NULL);
#endif
        ctx->buffered_fme.extended_data = ctx->buffered_fme.data;
    }

    if (!ctx->eos_fme_received) {
        int8_t bit_depth = 1;
        ishwframe        = ctx->buffered_fme.format == AV_PIX_FMT_NI_QUAD;
        if (ishwframe) {
            // Superframe early cleanup of unused outputs
            niFrameSurface1_t *pOutExtra;
            if (ctx->buffered_fme.buf[1]) {
                // NOLINTNEXTLINE(clang-diagnostic-incompatible-pointer-types)
                pOutExtra= (niFrameSurface1_t *)ctx->buffered_fme.buf[1]->data;
                if (pOutExtra->ui16FrameIdx != 0) {
                    av_log(avctx, AV_LOG_DEBUG, "Unref unused index %d\n",
                           pOutExtra->ui16FrameIdx);
                } else {
                    av_log(avctx, AV_LOG_ERROR,
                           "ERROR: Should not be getting superframe with dead "
                           "outputs\n");
                }
                av_buffer_unref(&ctx->buffered_fme.buf[1]);
                if (ctx->buffered_fme.buf[2]) {
                    // NOLINTNEXTLINE(clang-diagnostic-incompatible-pointer-types)
                    pOutExtra = (niFrameSurface1_t *)ctx->buffered_fme.buf[2]->data;
                    if (pOutExtra->ui16FrameIdx != 0) {
                        av_log(avctx, AV_LOG_DEBUG, "Unref unused index %d\n",
                               pOutExtra->ui16FrameIdx);
                    } else {
                        av_log(
                            avctx, AV_LOG_ERROR,
                            "ERROR: Should not be getting superframe with dead "
                            "outputs\n");
                    }
                    av_buffer_unref(&ctx->buffered_fme.buf[2]);
                }
            }
            pOutExtra = (niFrameSurface1_t *)ctx->buffered_fme.data[3];
            if (ctx->api_ctx.pixel_format == NI_PIX_FMT_ARGB
                || ctx->api_ctx.pixel_format == NI_PIX_FMT_ABGR
                || ctx->api_ctx.pixel_format == NI_PIX_FMT_RGBA
                || ctx->api_ctx.pixel_format == NI_PIX_FMT_BGRA) {
                bit_depth = 1;
            } else {
                bit_depth = pOutExtra->bit_depth;
            }

            switch (bit_depth) {
            case 1:
            case 2:
                break;
            default:
                av_log(avctx, AV_LOG_ERROR, "ERROR: Unknown bit depth %d!\n", bit_depth);
                return AVERROR_INVALIDDATA;
            }
        } else {
            if (AV_PIX_FMT_YUV420P10BE == ctx->buffered_fme.format ||
                AV_PIX_FMT_YUV420P10LE == ctx->buffered_fme.format ||
                AV_PIX_FMT_P010LE == ctx->buffered_fme.format) {
                bit_depth = 2;
            }
        }

        if ((ctx->buffered_fme.height && ctx->buffered_fme.width &&
             (ctx->buffered_fme.height != avctx->height ||
              ctx->buffered_fme.width != avctx->width)) ||
            bit_depth != ctx->api_ctx.bit_depth_factor) {
            av_log(avctx, AV_LOG_INFO,
                   "ff_xcoder_send_frame resolution change %dx%d "
                   "-> %dx%d or bit depth change %d -> %d\n",
                   avctx->width, avctx->height, ctx->buffered_fme.width,
                   ctx->buffered_fme.height, ctx->api_ctx.bit_depth_factor,
                   bit_depth);

            ctx->api_ctx.session_run_state =
                SESSION_RUN_STATE_SEQ_CHANGE_DRAINING;
            ctx->eos_fme_received = 1;

            // have to queue this frame if not done so: an empty queue
            if (is_input_fifo_empty(ctx)) {
                av_log(avctx, AV_LOG_TRACE,
                       "resolution change when fifo empty, frame "
                       "#%" PRIu64 " being queued ..\n",
                       ctx->api_ctx.frame_num);
                // unref buffered frame (this buffered frame is taken from input
                // AVFrame) because we are going to send EOS (instead of sending
                // buffered frame)
                if (frame != &ctx->buffered_fme) {
                    // For FFmpeg-n4.4+ receive_packet interface the buffered_fme is fetched from
                    // ff_alloc_get_frame rather than passed as function argument. So we need to
                    // judge whether they are the same object. If they are the same do NOT unreference
                    // any of them because we need to enqueue it later.
                    av_frame_unref(&ctx->buffered_fme);
                }
                ret = enqueue_frame(avctx, frame);
                if (ret < 0) {
                    return ret;
                }
            }
        }
    }

    ctx->api_fme.data.frame.preferred_characteristics_data_len = 0;
    ctx->api_fme.data.frame.end_of_stream                      = 0;
    ctx->api_fme.data.frame.force_key_frame =
        ctx->api_fme.data.frame.use_cur_src_as_long_term_pic =
            ctx->api_fme.data.frame.use_long_term_ref = 0;

    ctx->api_fme.data.frame.sei_total_len =
        ctx->api_fme.data.frame.sei_cc_offset = ctx->api_fme.data.frame
                                                    .sei_cc_len =
            ctx->api_fme.data.frame.sei_hdr_mastering_display_color_vol_offset =
                ctx->api_fme.data.frame
                    .sei_hdr_mastering_display_color_vol_len =
                    ctx->api_fme.data.frame
                        .sei_hdr_content_light_level_info_offset =
                        ctx->api_fme.data.frame
                            .sei_hdr_content_light_level_info_len =
                            ctx->api_fme.data.frame.sei_hdr_plus_offset =
                                ctx->api_fme.data.frame.sei_hdr_plus_len = 0;

    ctx->api_fme.data.frame.roi_len      = 0;
    ctx->api_fme.data.frame.reconf_len   = 0;
    ctx->api_fme.data.frame.force_pic_qp = 0;

    if (SESSION_RUN_STATE_SEQ_CHANGE_DRAINING ==
            ctx->api_ctx.session_run_state ||
        (ctx->eos_fme_received && is_input_fifo_empty(ctx))) {
        av_log(avctx, AV_LOG_VERBOSE, "XCoder start flushing\n");
        ctx->api_fme.data.frame.end_of_stream = 1;
        ctx->encoder_flushing                 = 1;
    } else {
        format_in_use = ctx->buffered_fme.format;

        // extra data starts with metadata header, various aux data sizes
        // have been reset above
        ctx->api_fme.data.frame.extra_data_len =
            NI_APP_ENC_FRAME_META_DATA_SIZE;

        ctx->api_fme.data.frame.ni_pict_type    = 0;

        ret = ni_enc_prep_reconf_demo_data(&ctx->api_ctx, &dec_frame);
        if (ret < 0) {
            return ret;
        }

    // long term reference frame support
    side_data = av_frame_get_side_data(&ctx->buffered_fme,
                                       AV_FRAME_DATA_NETINT_LONG_TERM_REF);
    if (side_data && (side_data->size == sizeof(AVNetintLongTermRef))) {
        aux_data =
            ni_frame_new_aux_data(&dec_frame, NI_FRAME_AUX_DATA_LONG_TERM_REF,
                                  sizeof(ni_long_term_ref_t));
        if (aux_data) {
            memcpy(aux_data->data, side_data->data, side_data->size);
        }
    }

    // target bitrate reconfiguration support
    side_data = av_frame_get_side_data(&ctx->buffered_fme,
                                       AV_FRAME_DATA_NETINT_BITRATE);
    if (side_data && (side_data->size == sizeof(int32_t))) {
        aux_data = ni_frame_new_aux_data(&dec_frame, NI_FRAME_AUX_DATA_BITRATE,
                                         sizeof(int32_t));
        if (aux_data) {
            memcpy(aux_data->data, side_data->data, side_data->size);
        }
    }

    side_data = av_frame_get_side_data(&ctx->buffered_fme,
                                       AV_FRAME_DATA_NETINT_GENERAL_SIDE_DATA);
    if (side_data && side_data->size == sizeof(AVNetintGeneralSideData)) {
        ni_side_data = (AVNetintGeneralSideData *)(side_data->data);
        for (i = 0; i < ni_side_data->count && i < NI_GENERAL_SIDEDATA_MAX_COUNT; i++) {
            if (ni_side_data->size[i]) {
                aux_data = ni_frame_new_aux_data(&dec_frame, ni_side_data->type[i], ni_side_data->size[i]);
                if (aux_data) {
                    memcpy(aux_data->data, ni_side_data->data[i], ni_side_data->size[i]);
                }
            }
        }
    }

    // support VFR
    if (ctx->api_param.enable_vfr) {
        int cur_fps = 0, pre_fps = 0;

        pre_fps = ctx->api_ctx.prev_fps;

        if (ctx->buffered_fme.pts > ctx->api_ctx.prev_pts) {
            ctx->api_ctx.passed_time_in_timebase_unit += ctx->buffered_fme.pts - ctx->api_ctx.prev_pts;
            ctx->api_ctx.count_frame_num_in_sec++;
            //change the FrameRate for VFR
            //1. Only when the fps change, setting the new bitrate
            //2. The interval between two framerate chagne settings shall be greater than 1 seconds
            //   or at the start the transcoding
            if (ctx->api_ctx.passed_time_in_timebase_unit >= (avctx->time_base.den / avctx->time_base.num)) {
                //this is a workaround for small resolution vfr mode
                //when detect framerate change, the reconfig framerate will trigger bitrate params to reset
                //the cost related to bitrate estimate is all tuned with downsample flow
                //but for small resolution, the lookahead won't downsample
                int slow_down_vfr = 0;
                cur_fps = ctx->api_ctx.count_frame_num_in_sec;
                if (ctx->buffered_fme.width < 288 || ctx->buffered_fme.height < 256) {
                    slow_down_vfr = 1;
                }
                if ((ctx->api_ctx.frame_num != 0) && (pre_fps != cur_fps) && (slow_down_vfr ? (abs(cur_fps - pre_fps) > 2) : 1) &&
                    ((ctx->api_ctx.frame_num < ctx->api_param.cfg_enc_params.frame_rate) ||
                     (ctx->api_ctx.frame_num - ctx->api_ctx.last_change_framenum >= ctx->api_param.cfg_enc_params.frame_rate))) {
                    aux_data = ni_frame_new_aux_data(&dec_frame, NI_FRAME_AUX_DATA_FRAMERATE, sizeof(ni_framerate_t));
                    if (aux_data) {
                        ni_framerate_t *framerate = (ni_framerate_t *)aux_data->data;
                        framerate->framerate_num = cur_fps;
                        framerate->framerate_denom = 1;
                    }

                    ctx->api_ctx.last_change_framenum = ctx->api_ctx.frame_num;
                    ctx->api_ctx.prev_fps = cur_fps;
                }
                ctx->api_ctx.count_frame_num_in_sec = 0;
                ctx->api_ctx.passed_time_in_timebase_unit = 0;
            }
            ctx->api_ctx.prev_pts = ctx->buffered_fme.pts;
        } else if (ctx->buffered_fme.pts < ctx->api_ctx.prev_pts) {
            //error handle for the case that pts jump back
            //this may cause a little error in the bitrate setting, This little error is acceptable.
            //As long as the subsequent, PTS is normal, it will be repaired quickly.
            ctx->api_ctx.prev_pts = ctx->buffered_fme.pts;
        } else {
            //do nothing, when the pts of two adjacent frames are the same
            //this may cause a little error in the bitrate setting, This little error is acceptable.
            //As long as the subsequent, PTS is normal, it will be repaired quickly.
        }
    }

    // force pic qp demo mode: initial QP (200 frames) -> QP value specified by
    // ForcePicQpDemoMode (100 frames) -> initial QP (remaining frames)
    if (p_param->force_pic_qp_demo_mode) {
        if (ctx->api_ctx.frame_num >= 300) {
            ctx->api_fme.data.frame.force_pic_qp =
                p_param->cfg_enc_params.rc.intra_qp;
        } else if (ctx->api_ctx.frame_num >= 200) {
            ctx->api_fme.data.frame.force_pic_qp = p_param->force_pic_qp_demo_mode;
        }
    }

    // supply QP map if ROI enabled and if ROIs passed in
    // Note: ROI demo mode takes higher priority over side data !
    side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_REGIONS_OF_INTEREST);

    if (!p_param->roi_demo_mode && p_param->cfg_enc_params.roi_enable &&
        side_data) {
        aux_data = ni_frame_new_aux_data(
            &dec_frame, NI_FRAME_AUX_DATA_REGIONS_OF_INTEREST, side_data->size);
        if (aux_data) {
            memcpy(aux_data->data, side_data->data, side_data->size);
        }
    }

    // Note: when ROI demo modes enabled, supply ROI map for the specified range
    //       frames, and 0 map for others
    if (QUADRA && p_param->roi_demo_mode &&
        p_param->cfg_enc_params.roi_enable) {
        if (ctx->api_ctx.frame_num > 90 && ctx->api_ctx.frame_num < 300) {
            ctx->api_fme.data.frame.roi_len = ctx->api_ctx.roi_len;
        } else {
            ctx->api_fme.data.frame.roi_len = 0;
        }
        // when ROI enabled, always have a data buffer for ROI
        // Note: this is handled separately from ROI through side/aux data
        ctx->api_fme.data.frame.extra_data_len += ctx->api_ctx.roi_len;
    }

    if (!p_param->cfg_enc_params.enable_all_sei_passthru) {
        // SEI (HDR)
        // content light level info
        if (!(p_param->cfg_enc_params.HDR10CLLEnable)) { // not user set
            side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_CONTENT_LIGHT_LEVEL);

            if (side_data && side_data->size == sizeof(AVContentLightMetadata)) {
                aux_data = ni_frame_new_aux_data(
                    &dec_frame, NI_FRAME_AUX_DATA_CONTENT_LIGHT_LEVEL,
                    sizeof(ni_content_light_level_t));
                if (aux_data) {
                    memcpy(aux_data->data, side_data->data, side_data->size);
                }
            }
        } else if ((AV_CODEC_ID_H264 == avctx->codec_id ||
                    ctx->api_ctx.bit_depth_factor == 1) &&
                  ctx->api_ctx.light_level_data_len == 0) {
            // User input maxCLL so create SEIs for h264 and don't touch for (h265 &&
            // hdr10) since that is conveyed in config step
            // Quadra autoset only for hdr10 format with hevc
            aux_data = ni_frame_new_aux_data(&dec_frame,
                                            NI_FRAME_AUX_DATA_CONTENT_LIGHT_LEVEL,
                                            sizeof(ni_content_light_level_t));
            if (aux_data) {
                ni_content_light_level_t *cll =
                    (ni_content_light_level_t *)(aux_data->data);
                cll->max_cll  = p_param->cfg_enc_params.HDR10MaxLight;
                cll->max_fall = p_param->cfg_enc_params.HDR10AveLight;
            }
        }

        // mastering display color volume
        if (!(p_param->cfg_enc_params.HDR10Enable)) { // not user set
            side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_MASTERING_DISPLAY_METADATA);
            if (side_data && side_data->size == sizeof(AVMasteringDisplayMetadata)) {
                aux_data = ni_frame_new_aux_data(
                    &dec_frame, NI_FRAME_AUX_DATA_MASTERING_DISPLAY_METADATA,
                    sizeof(ni_mastering_display_metadata_t));
                if (aux_data) {
                    memcpy(aux_data->data, side_data->data, side_data->size);
                }
            }
        } else if ((AV_CODEC_ID_H264 == avctx->codec_id ||
                   ctx->api_ctx.bit_depth_factor == 1) &&
                  ctx->api_ctx.sei_hdr_mastering_display_color_vol_len == 0) {
            // User input masterDisplay so create SEIs for h264 and don't touch for (h265 &&
            // hdr10) since that is conveyed in config step
            // Quadra autoset only for hdr10 format with hevc
            aux_data = ni_frame_new_aux_data(&dec_frame,
                NI_FRAME_AUX_DATA_MASTERING_DISPLAY_METADATA,
                sizeof(ni_mastering_display_metadata_t));
            if (aux_data) {
                ni_mastering_display_metadata_t *mst_dsp =
                    (ni_mastering_display_metadata_t *)(aux_data->data);

                //X, Y display primaries for RGB channels and white point(WP) in units of 0.00002
                //and max, min luminance(L) values in units of 0.0001 nits
                //xy are denom = 50000 num = HDR10dx0/y
                mst_dsp->display_primaries[0][0].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->display_primaries[0][1].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->display_primaries[1][0].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->display_primaries[1][1].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->display_primaries[2][0].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->display_primaries[2][1].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->white_point[0].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->white_point[1].den = MASTERING_DISP_CHROMA_DEN;
                mst_dsp->min_luminance.den = MASTERING_DISP_LUMA_DEN;
                mst_dsp->max_luminance.den = MASTERING_DISP_LUMA_DEN;
                // ni_mastering_display_metadata_t has to be filled with R,G,B
                // values, in that order, while HDR10d is filled in order of G,B,R,
                // so do the conversion here.
                mst_dsp->display_primaries[0][0].num = p_param->cfg_enc_params.HDR10dx2;
                mst_dsp->display_primaries[0][1].num = p_param->cfg_enc_params.HDR10dy2;
                mst_dsp->display_primaries[1][0].num = p_param->cfg_enc_params.HDR10dx0;
                mst_dsp->display_primaries[1][1].num = p_param->cfg_enc_params.HDR10dy0;
                mst_dsp->display_primaries[2][0].num = p_param->cfg_enc_params.HDR10dx1;
                mst_dsp->display_primaries[2][1].num = p_param->cfg_enc_params.HDR10dy1;
                mst_dsp->white_point[0].num = p_param->cfg_enc_params.HDR10wx;
                mst_dsp->white_point[1].num = p_param->cfg_enc_params.HDR10wy;
                mst_dsp->min_luminance.num = p_param->cfg_enc_params.HDR10minluma;
                mst_dsp->max_luminance.num = p_param->cfg_enc_params.HDR10maxluma;
                mst_dsp->has_primaries = 1;
                mst_dsp->has_luminance = 1;
            }
        }
        // SEI (HDR10+)
        side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_DYNAMIC_HDR_PLUS);
        if (side_data && side_data->size == sizeof(AVDynamicHDRPlus)) {
            aux_data = ni_frame_new_aux_data(&dec_frame, NI_FRAME_AUX_DATA_HDR_PLUS,
                                            sizeof(ni_dynamic_hdr_plus_t));
            if (aux_data) {
                memcpy(aux_data->data, side_data->data, side_data->size);
            }
        } // hdr10+

        // SEI (close caption)
        side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_A53_CC);

        if (side_data && side_data->size > 0) {
            aux_data = ni_frame_new_aux_data(&dec_frame, NI_FRAME_AUX_DATA_A53_CC,
                                            side_data->size);
            if (aux_data) {
                memcpy(aux_data->data, side_data->data, side_data->size);
            }
        }

        // User data unregistered SEI
        side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_SEI_UNREGISTERED);
        if (ctx->udu_sei && side_data && side_data->size > 0) {
            aux_data = ni_frame_new_aux_data(&dec_frame, NI_FRAME_AUX_DATA_UDU_SEI,
                                             side_data->size);
            if (aux_data) {
                memcpy(aux_data->data, (uint8_t *)side_data->data, side_data->size);
            }
        }
    }
    if (ctx->api_ctx.force_frame_type) {
        switch (ctx->buffered_fme.pict_type) {
        case AV_PICTURE_TYPE_I:
            ctx->api_fme.data.frame.ni_pict_type = PIC_TYPE_IDR;
            break;
        case AV_PICTURE_TYPE_P:
            ctx->api_fme.data.frame.ni_pict_type = PIC_TYPE_P;
            break;
        default:
            ;
        }
    } else if (ctx->buffered_fme.pict_type == AV_PICTURE_TYPE_I) {
        ctx->api_fme.data.frame.force_key_frame = 1;
        ctx->api_fme.data.frame.ni_pict_type = PIC_TYPE_IDR;
    }

    av_log(avctx, AV_LOG_TRACE,
           "ff_xcoder_send_frame: #%" PRIu64 " ni_pict_type %d"
           " forced_header_enable %d intraPeriod %d\n",
           ctx->api_ctx.frame_num, ctx->api_fme.data.frame.ni_pict_type,
           p_param->cfg_enc_params.forced_header_enable,
           p_param->cfg_enc_params.intra_period);

    // whether should send SEI with this frame
    send_sei_with_idr = ni_should_send_sei_with_frame(
        &ctx->api_ctx, ctx->api_fme.data.frame.ni_pict_type, p_param);

    // prep for auxiliary data (various SEI, ROI) in encode frame, based on the
    // data returned in decoded frame
    ni_enc_prep_aux_data(&ctx->api_ctx, &ctx->api_fme.data.frame, &dec_frame,
                         ctx->api_ctx.codec_format, send_sei_with_idr,
                         mdcv_data, cll_data, cc_data, udu_data, hdrp_data);

    side_data = av_frame_get_side_data(&ctx->buffered_fme,
                                       AV_FRAME_DATA_NETINT_CUSTOM_SEI);
    if (side_data && side_data->size > 0) {
        int64_t local_pts = ctx->buffered_fme.pts;
        uint8_t *p_src_sei_data, *p_dst_sei_data;
        int sei_size;
        uint8_t sei_type;
        int size;
        ni_custom_sei_set_t *src_custom_sei_set, *dst_custom_sei_set;
        ni_custom_sei_t *p_src_custom_sei, *p_dst_custom_sei;

        // if one picture can be skipped, nienc will send that frame but will not
        // receive packet, therefore it will skip the free in receive packet as
        // well and cause memory leak. So check the last pkt_custom_sei_set has
        // been released or not.
        dst_custom_sei_set = ctx->api_ctx.pkt_custom_sei_set[local_pts % NI_FIFO_SZ];
        ni_memfree(dst_custom_sei_set);
        ctx->api_ctx.pkt_custom_sei_set[local_pts % NI_FIFO_SZ] = NULL;

        /* copy the whole SEI data */
        src_custom_sei_set = (ni_custom_sei_set_t *)side_data->data;
        dst_custom_sei_set = malloc(sizeof(ni_custom_sei_set_t)); // libxcoder uses malloc() and free()
        if (dst_custom_sei_set == NULL) {
            av_log(avctx, AV_LOG_ERROR, "failed to allocate memory for custom sei data\n");
            ret = AVERROR(ENOMEM);
            return ret;
        }
        memset(dst_custom_sei_set, 0, sizeof(ni_custom_sei_set_t));

        /* fill sei data */
        for (i = 0; i < src_custom_sei_set->count; i++) {
            int len;
            p_src_custom_sei = &src_custom_sei_set->custom_sei[i];
            sei_size = p_src_custom_sei->size;
            sei_type = p_src_custom_sei->type;
            p_src_sei_data = &p_src_custom_sei->data[0];

            // When timecode passthru is enabled, the picture timing / time code SEI
            // has been parsed by the decoder already and sent as special SEI type (240)
            // Here we just need to read it out as ni_timecode_t data and skip putting
            // the data in custom SEI FIFO so that it won't be appended directly
            if (ctx->timecode_passthru) {
                if (sei_type == 240) {
                    memcpy(&timecode, p_src_sei_data, sizeof(ni_timecode_t));
                    timecode_found = 1;
                    continue;
                }
            }

            p_dst_custom_sei = &dst_custom_sei_set->custom_sei[i];
            p_dst_sei_data = &p_dst_custom_sei->data[0];
            size = 0;

            // long start code
            p_dst_sei_data[size++] = 0x00;
            p_dst_sei_data[size++] = 0x00;
            p_dst_sei_data[size++] = 0x00;
            p_dst_sei_data[size++] = 0x01;

            if (AV_CODEC_ID_H264 == avctx->codec_id) {
                p_dst_sei_data[size++] = 0x06;   //nal type: SEI
            } else {
                p_dst_sei_data[size++] = 0x4e;   //nal type: SEI
                p_dst_sei_data[size++] = 0x01;
            }

            // SEI type
            p_dst_sei_data[size++] = sei_type;

            // original payload size
            len = sei_size;
            while (len >= 0) {
                p_dst_sei_data[size++] = len > 0xff ? 0xff : len;
                len -= 0xff;
            }

            // payload data
            for (j = 0; j < sei_size && size < NI_MAX_CUSTOM_SEI_DATA - 1; j++) {
                if (j >= 2 && !p_dst_sei_data[size - 2] && !p_dst_sei_data[size - 1] && p_src_sei_data[j] <= 0x03) {
                    /* insert 0x3 as emulation_prevention_three_byte */
                    p_dst_sei_data[size++] = 0x03;
                }
                p_dst_sei_data[size++] = p_src_sei_data[j];
            }

            if (j != sei_size) {
                av_log(avctx, AV_LOG_WARNING, "%s: trimming sei with RBSP size out of limit(%d), "
                       "idx=%u, type=%u, size=%d, custom_sei_loc=%d.\n", __func__,
                       NI_MAX_CUSTOM_SEI_DATA, i, sei_type, sei_size, p_src_custom_sei->location);
            }

            // trailing byte
            p_dst_sei_data[size++] = 0x80;

            p_dst_custom_sei->size = size;
            p_dst_custom_sei->type = sei_type;
            p_dst_custom_sei->location = p_src_custom_sei->location;
            av_log(avctx, AV_LOG_TRACE, "%s: custom sei idx %d type %u len %d loc %d.\n",
                   __func__, i, sei_type, size, p_dst_custom_sei->location);

            dst_custom_sei_set->count++;
        }

        ctx->api_ctx.pkt_custom_sei_set[local_pts % NI_FIFO_SZ] = dst_custom_sei_set;
        av_log(avctx, AV_LOG_TRACE, "%s: sei number %d pts %" PRId64 ".\n",
               __func__, dst_custom_sei_set->count, local_pts);
    }

    if (ctx->timecode_passthru) {
        // Check for time code from SW decoder
        side_data = av_frame_get_side_data(&ctx->buffered_fme, AV_FRAME_DATA_S12M_TIMECODE);
        if (side_data && side_data->size > 0) {
            uint32_t *tc_data = (uint32_t *)side_data->data;
            if ((tc_data[0] & 3) == 0) {
                av_log(avctx, AV_LOG_VERBOSE, "num_clock_ts provided in time code is 0, no SEI will be inserted");
            } else {
                if ((tc_data[0] & 3) > 1) {
                    av_log(avctx, AV_LOG_VERBOSE, "num_clock_ts provided in time code > 1, only the first time code will be inserted");
                }
                timecode.nuit_field_based_flag = 1;
                timecode.counting_type = 0;
                timecode.full_timestamp_flag = 1;
                timecode.discontinuity_flag = 0;
                timecode.cnt_dropped_flag = (tc_data[1] >> 30) & 1;
                timecode.n_frames = (tc_data[1] >> 24) & 0xf;
                timecode.n_frames += ((tc_data[1] >> 28) & 0x3) * 10;
                timecode.seconds_value = (tc_data[1] >> 16) & 0xf;
                timecode.seconds_value += ((tc_data[1] >> 20) & 0x7) * 10;
                timecode.minutes_value = (tc_data[1] >> 8) & 0xf;
                timecode.minutes_value += ((tc_data[1] >> 12) & 0x7) * 10;
                timecode.hours_value = tc_data[1] & 0xf;
                timecode.hours_value += ((tc_data[1] >> 4) & 0x3) * 10;
                timecode.time_offset_length = 0;
                timecode.time_offset_value = 0;

                /* Convert frame number from SMPTE timecode to actual value if framerate > 30FPS */
                if (av_cmp_q(avctx->framerate, (AVRational) {30, 1}) == 1) {
                    timecode.n_frames *= 2;
                    if (av_cmp_q(avctx->framerate, (AVRational) {50, 1}) == 0)
                        timecode.n_frames += (tc_data[1] >> 7) & 1;
                    else
                        timecode.n_frames += (tc_data[1] >> 23) & 1;
                }

                timecode_found = 1;
            }
        }
    }

    if (ctx->api_fme.data.frame.sei_total_len > NI_ENC_MAX_SEI_BUF_SIZE) {
        av_log(avctx, AV_LOG_ERROR, "ff_xcoder_send_frame: sei total length %u exceeds maximum sei size %u.\n",
               ctx->api_fme.data.frame.sei_total_len, NI_ENC_MAX_SEI_BUF_SIZE);
        ret = AVERROR(EINVAL);
        return ret;
    }

    // Maximum length of picture timing / time code SEI that's going to be inserted is 18 bytes
    // according to the H264/H265 specs. Temporarily add this maximum length to total extra data
    // length to make sure that the buffer allocated is large enough
    if (timecode_found) {
      ctx->api_fme.data.frame.sei_total_len += 18;
    }

    ctx->api_fme.data.frame.extra_data_len += ctx->api_fme.data.frame.sei_total_len;

    // data layout requirement: leave space for reconfig data if at least one of
    // reconfig, SEI or ROI is present
    // Note: ROI is present when enabled, so use encode config flag instead of
    //       frame's roi_len as it can be 0 indicating a 0'd ROI map setting !
    if (ctx->api_fme.data.frame.reconf_len ||
        ctx->api_fme.data.frame.sei_total_len ||
        p_param->cfg_enc_params.roi_enable) {
        ctx->api_fme.data.frame.extra_data_len +=
            sizeof(ni_encoder_change_params_t);
    }

    ctx->api_fme.data.frame.pts = ctx->buffered_fme.pts;
    ctx->api_fme.data.frame.dts = ctx->buffered_fme.pkt_dts;

    ctx->api_fme.data.frame.video_width = avctx->width;
    ctx->api_fme.data.frame.video_height = avctx->height;

    ishwframe = ctx->buffered_fme.format == AV_PIX_FMT_NI_QUAD;
    if (ctx->api_ctx.auto_dl_handle != 0 || (avctx->height < NI_MIN_HEIGHT) ||
        (avctx->width < NI_MIN_WIDTH)) {
        format_in_use          = avctx->sw_pix_fmt;
        ctx->api_ctx.hw_action = 0;
        ishwframe              = 0;
    }
    isnv12frame = (format_in_use == AV_PIX_FMT_NV12 || format_in_use == AV_PIX_FMT_P010LE);

    if (ishwframe) {
        ret = sizeof(niFrameSurface1_t);
    } else {
        ret = av_image_get_buffer_size(format_in_use,
                                     ctx->buffered_fme.width, ctx->buffered_fme.height, 1);
    }

#if FF_API_PKT_PTS
    // NOLINTNEXTLINE(clang-diagnostic-deprecated-declarations)
    av_log(avctx, AV_LOG_TRACE, "ff_xcoder_send_frame: pts=%" PRId64 ", pkt_dts=%" PRId64 ", pkt_pts=%" PRId64 "\n", ctx->buffered_fme.pts, ctx->buffered_fme.pkt_dts, ctx->buffered_fme.pkt_pts);
#endif
    av_log(avctx, AV_LOG_TRACE, "ff_xcoder_send_frame: frame->format=%d, frame->width=%d, frame->height=%d, frame->pict_type=%d, size=%d\n", format_in_use, ctx->buffered_fme.width, ctx->buffered_fme.height, ctx->buffered_fme.pict_type, ret);
    if (ret < 0) {
        return ret;
    }

    int dst_stride[NI_MAX_NUM_DATA_POINTERS]     = {0};
    int height_aligned[NI_MAX_NUM_DATA_POINTERS] = {0};
    int src_height[NI_MAX_NUM_DATA_POINTERS]     = {0};

    src_height[0] = ctx->buffered_fme.height;
    src_height[1] = ctx->buffered_fme.height / 2;
    src_height[2] = (isnv12frame) ? 0 : (ctx->buffered_fme.height / 2);
    if (avctx->sw_pix_fmt == AV_PIX_FMT_ARGB ||
        avctx->sw_pix_fmt == AV_PIX_FMT_RGBA ||
        avctx->sw_pix_fmt == AV_PIX_FMT_ABGR ||
        avctx->sw_pix_fmt == AV_PIX_FMT_BGRA) {
        src_height[0] = ctx->buffered_fme.height;
        src_height[1] = 0;
        src_height[2] = 0;
        alignment_2pass_wa = 0;
    }
    /* The main reason for the problem is that when using out=sw and noautoscale=0,
    * the incoming stream initially conforms to zerocopy, so the firmware will allocate memory according to zerocopy.
    * The resolution after this changes, but the software inserts autoscale,
    * so the encoder cannot detect the change in resolution and cannot reopen.
    * However, due to ffmpeg being 64 bit aligned, So the linesize is not consistent with the linesize we initially decoded,
    * so the encoding will take the path of non-zero copy. At this time, due to lookahead>0,
    * the software will send the firmware a size larger than the originally requested size,
    * so it will not be able to send it. Moreover, at this point,
    * the firmware is unable to perceive changes in linesize and respond accordingly.
    * For fix this, we can check the linesize and disable 2-pass workaround.
    */
    if (ctx->api_param.luma_linesize) {
        alignment_2pass_wa = false;
    }

    ni_get_min_frame_dim(ctx->buffered_fme.width,
                         ctx->buffered_fme.height,
                         ctx->api_ctx.pixel_format,
                         dst_stride, height_aligned);

    av_log(avctx, AV_LOG_TRACE,
           "ff_xcoder_send_frame frame->width %d "
           "ctx->api_ctx.bit_depth_factor %d dst_stride[0/1/2] %d/%d/%d sw_pix_fmt %d\n",
           ctx->buffered_fme.width, ctx->api_ctx.bit_depth_factor,
           dst_stride[0], dst_stride[1], dst_stride[2], avctx->sw_pix_fmt);

    if (alignment_2pass_wa && !ishwframe) {
        if (isnv12frame) {
            // for 2-pass encode output mismatch WA, need to extend (and
            // pad) CbCr plane height, because 1st pass assume input 32
            // align
            height_aligned[1] = FFALIGN(height_aligned[0], 32) / 2;
        } else {
            // for 2-pass encode output mismatch WA, need to extend (and
            // pad) Cr plane height, because 1st pass assume input 32 align
            height_aligned[2] = FFALIGN(height_aligned[0], 32) / 2;
        }
    }

    // alignment(16) extra padding for H.264 encoding
    if (ishwframe) {
        uint8_t *dsthw;
        const uint8_t *srchw;

        ni_frame_buffer_alloc_hwenc(
            &(ctx->api_fme.data.frame), ctx->buffered_fme.width,
            ctx->buffered_fme.height,
            (int)ctx->api_fme.data.frame.extra_data_len);
        if (!ctx->api_fme.data.frame.p_data[3]) {
            return AVERROR(ENOMEM);
        }
        dsthw       = ctx->api_fme.data.frame.p_data[3];
        srchw = (const uint8_t *)ctx->buffered_fme.data[3];
        av_log(avctx, AV_LOG_TRACE, "dst=%p src=%p len=%d\n", dsthw, srchw,
               ctx->api_fme.data.frame.data_len[3]);
        memcpy(dsthw, srchw, ctx->api_fme.data.frame.data_len[3]);
        av_log(avctx, AV_LOG_TRACE,
               "ctx->buffered_fme.data[3] %p memcpy to %p\n",
               ctx->buffered_fme.data[3], dsthw);
    } else {  // traditional yuv transfer
        av_log(avctx, AV_LOG_TRACE, "%s %s %d buffered_fme.data[0] %p data[3] %p wxh %u %u dst_stride[0] %d %d  linesize[0] %d %d data[1] %p %p data[2] %p %p data[3] %p buf[0] %p crop(t:b:l:r) %lu:%lu:%lu:%lu avctx(w:h:cw:ch) %u:%u:%u:%u\n",
          __FILE__, __FUNCTION__, __LINE__,
          ctx->buffered_fme.data[0], ctx->buffered_fme.data[3],
          ctx->buffered_fme.width, ctx->buffered_fme.height,
          dst_stride[0], dst_stride[1],
          ctx->buffered_fme.linesize[0], ctx->buffered_fme.linesize[1],
          ctx->buffered_fme.data[1], ctx->buffered_fme.data[0] + dst_stride[0] * ctx->buffered_fme.height,
          ctx->buffered_fme.data[2], ctx->buffered_fme.data[1] + dst_stride[1] * ctx->buffered_fme.height / 2,
          ctx->buffered_fme.data[3],
          ctx->buffered_fme.buf[0],
          ctx->buffered_fme.crop_top, ctx->buffered_fme.crop_bottom, ctx->buffered_fme.crop_left, ctx->buffered_fme.crop_right,
          avctx->width, avctx->height,
          avctx->coded_width, avctx->coded_height);

        // check input resolution zero copy compatible or not
        if (ni_encoder_frame_zerocopy_check(&ctx->api_ctx,
            p_param, ctx->buffered_fme.width, ctx->buffered_fme.height,
            (const int *)ctx->buffered_fme.linesize, false) == NI_RETCODE_SUCCESS) {
            need_to_copy = 0;
            // alloc metadata buffer etc. (if needed)
            ret = ni_encoder_frame_zerocopy_buffer_alloc(
                &(ctx->api_fme.data.frame), ctx->buffered_fme.width,
                ctx->buffered_fme.height, (const int *)ctx->buffered_fme.linesize, (const uint8_t **)ctx->buffered_fme.data,
                (int)ctx->api_fme.data.frame.extra_data_len);
            if (ret != NI_RETCODE_SUCCESS)
                return AVERROR(ENOMEM);
        } else {
            // if linesize changes (while resolution remains the same), copy to previously configured linesizes
            if (p_param->luma_linesize && p_param->chroma_linesize) {
                dst_stride[0] = p_param->luma_linesize;
                dst_stride[1] = p_param->chroma_linesize;
                dst_stride[2] = isnv12frame ? 0 : p_param->chroma_linesize;
            }
            ni_encoder_sw_frame_buffer_alloc(
                !isnv12frame, &(ctx->api_fme.data.frame), ctx->buffered_fme.width,
                height_aligned[0], dst_stride, (avctx->codec_id == AV_CODEC_ID_H264),
                (int)ctx->api_fme.data.frame.extra_data_len, alignment_2pass_wa);
        }
        av_log(avctx, AV_LOG_TRACE, "%p need_to_copy %d! pts = %ld\n", ctx->api_fme.data.frame.p_buffer, need_to_copy, ctx->buffered_fme.pts);
        if (!ctx->api_fme.data.frame.p_data[0]) {
            return AVERROR(ENOMEM);
        }

        // if this is indeed sw frame, do the YUV data layout, otherwise may need
        // to do frame download
        if (ctx->buffered_fme.format != AV_PIX_FMT_NI_QUAD) {
            av_log(
                avctx, AV_LOG_TRACE,
                "ff_xcoder_send_frame: fme.data_len[0]=%d, "
                "buf_fme->linesize=%d/%d/%d, dst alloc linesize = %d/%d/%d, "
                "src height = %d/%d/%d, dst height aligned = %d/%d/%d, "
                "force_key_frame=%d, extra_data_len=%d sei_size=%d "
                "(hdr_content_light_level %u hdr_mastering_display_color_vol %u "
                "hdr10+ %u cc %u udu %u prefC %u) roi_size=%u reconf_size=%u "
                "force_pic_qp=%u "
                "use_cur_src_as_long_term_pic %u use_long_term_ref %u\n",
                ctx->api_fme.data.frame.data_len[0],
                ctx->buffered_fme.linesize[0], ctx->buffered_fme.linesize[1],
                ctx->buffered_fme.linesize[2], dst_stride[0], dst_stride[1],
                dst_stride[2], src_height[0], src_height[1], src_height[2],
                height_aligned[0], height_aligned[1], height_aligned[2],
                ctx->api_fme.data.frame.force_key_frame,
                ctx->api_fme.data.frame.extra_data_len,
                ctx->api_fme.data.frame.sei_total_len,
                ctx->api_fme.data.frame.sei_hdr_content_light_level_info_len,
                ctx->api_fme.data.frame.sei_hdr_mastering_display_color_vol_len,
                ctx->api_fme.data.frame.sei_hdr_plus_len,
                ctx->api_fme.data.frame.sei_cc_len,
                ctx->api_fme.data.frame.sei_user_data_unreg_len,
                ctx->api_fme.data.frame.preferred_characteristics_data_len,
                (p_param->cfg_enc_params.roi_enable ? ctx->api_ctx.roi_len : 0),
                ctx->api_fme.data.frame.reconf_len,
                ctx->api_fme.data.frame.force_pic_qp,
                ctx->api_fme.data.frame.use_cur_src_as_long_term_pic,
                ctx->api_fme.data.frame.use_long_term_ref);

            // YUV part of the encoder input data layout
            if (need_to_copy) {
                ni_copy_frame_data(
                    (uint8_t **)(ctx->api_fme.data.frame.p_data),
                    ctx->buffered_fme.data, ctx->buffered_fme.width,
                    ctx->buffered_fme.height, ctx->api_ctx.bit_depth_factor,
                    ctx->api_ctx.pixel_format, p_param->cfg_enc_params.conf_win_right, dst_stride,
                    height_aligned, ctx->buffered_fme.linesize, src_height);
            }
        } else {
            ni_session_data_io_t *p_session_data;
            ni_session_data_io_t niframe;
            niFrameSurface1_t *src_surf;

            av_log(avctx, AV_LOG_DEBUG,
                   "ff_xcoder_send_frame:Autodownload to be run: hdl: %d w: %d h: %d\n",
                   ctx->api_ctx.auto_dl_handle, avctx->width, avctx->height);
            avhwf_ctx =
                (AVHWFramesContext *)ctx->buffered_fme.hw_frames_ctx->data;
            nif_src_ctx = (AVNIFramesContext*) avhwf_ctx->hwctx;

            src_surf = (niFrameSurface1_t *)ctx->buffered_fme.data[3];

            if (avctx->height < NI_MIN_HEIGHT || avctx->width < NI_MIN_WIDTH) {
                int bit_depth;
                int is_planar;

                p_session_data = &niframe;
                memset(&niframe, 0, sizeof(niframe));
                bit_depth = ((avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10LE) ||
                           (avctx->sw_pix_fmt == AV_PIX_FMT_P010LE))
                              ? 2
                              : 1;
                is_planar = (avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P) ||
                          (avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10LE);

                /* Allocate a minimal frame */
                ni_enc_frame_buffer_alloc(&niframe.data.frame, avctx->width,
                                    avctx->height, 0, /* alignment */
                                    1,                /* metadata */
                                    bit_depth, 0,     /* hw_frame_count */
                                    is_planar, ctx->api_ctx.pixel_format);
            } else {
                p_session_data = &(ctx->api_fme);
            }

            nif_src_ctx->api_ctx.is_auto_dl = true;
            ret = ni_device_session_hwdl(&nif_src_ctx->api_ctx, p_session_data,
                                       src_surf);
            ishwframe = false;
            if (ret <= 0) {
                av_log(avctx, AV_LOG_ERROR,
                       "nienc.c:ni_hwdl_frame() failed to retrieve frame\n");
                return AVERROR_EXTERNAL;
            }

            if ((avctx->height < NI_MIN_HEIGHT) ||
                (avctx->width < NI_MIN_WIDTH)) {
                int nb_planes = av_pix_fmt_count_planes(avctx->sw_pix_fmt);
                int ni_fmt = ctx->api_ctx.pixel_format;
                ni_expand_frame(&ctx->api_fme.data.frame,
                              &p_session_data->data.frame, dst_stride,
                              avctx->width, avctx->height, ni_fmt, nb_planes);

                ni_frame_buffer_free(&niframe.data.frame);
            }
        }
    } // end if hwframe else

    // auxiliary data part of the encoder input data layout
    ni_enc_copy_aux_data(&ctx->api_ctx, &ctx->api_fme.data.frame, &dec_frame,
                         ctx->api_ctx.codec_format, mdcv_data, cll_data,
                         cc_data, udu_data, hdrp_data, ishwframe, isnv12frame);

    if (timecode_found) {
        // Subtract the maximum length temporarily added earlier
        // ni_enc_insert_timecode will calculate the actual length and add to these values
        ctx->api_fme.data.frame.sei_total_len -= 18;
        ctx->api_fme.data.frame.extra_data_len -= 18;
        ni_enc_insert_timecode(&ctx->api_ctx, &ctx->api_fme.data.frame, &timecode);
    }

    ni_frame_buffer_free(&dec_frame);

    // end of encode input frame data layout

    } // end non seq change

    sent = ni_device_session_write(&ctx->api_ctx, &ctx->api_fme, NI_DEVICE_TYPE_ENCODER);

    av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_send_frame: size %d sent to xcoder\n", sent);

    // return EIO at error
    if (NI_RETCODE_ERROR_VPU_RECOVERY == sent) {
        sent = xcoder_encode_reset(avctx);
        if (sent < 0) {
            av_log(avctx, AV_LOG_ERROR, "ff_xcoder_send_frame(): VPU recovery failed:%d, returning EIO\n", sent);
            ret = AVERROR(EIO);
        }
    } else if (sent < 0) {
        av_log(avctx, AV_LOG_ERROR, "ff_xcoder_send_frame(): failure sent (%d) , "
               "returning EIO\n", sent);
        ret = AVERROR(EIO);

        // if rejected due to sequence change in progress, revert resolution
        // setting and will do it again next time.
        if (ctx->api_fme.data.frame.start_of_stream &&
            (avctx->width != orig_avctx_width ||
             avctx->height != orig_avctx_height)) {
            avctx->width = orig_avctx_width;
            avctx->height = orig_avctx_height;
        }
        return ret;
    } else {
        av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_send_frame(): sent (%d)\n", sent);
        if (sent == 0) {
            // case of sequence change in progress
            if (ctx->api_fme.data.frame.start_of_stream &&
                (avctx->width != orig_avctx_width ||
                 avctx->height != orig_avctx_height)) {
                avctx->width = orig_avctx_width;
                avctx->height = orig_avctx_height;
            }

            // when buffer_full, drop the frame and return EAGAIN if in strict timeout
            // mode, otherwise buffer the frame and it is to be sent out using encode2
            // API: queue the frame only if not done so yet, i.e. queue is empty
            // *and* it's a valid frame.
            if (ctx->api_ctx.status == NI_RETCODE_NVME_SC_WRITE_BUFFER_FULL) {
                ishwframe = ctx->buffered_fme.format == AV_PIX_FMT_NI_QUAD;
                if (ishwframe) {
                    // Do not queue frames to avoid FFmpeg stuck when multiple HW frames are queued up in nienc, causing decoder unable to acquire buffer, which led to FFmpeg stuck
                    av_log(avctx, AV_LOG_ERROR, "ff_xcoder_send_frame(): device WRITE_BUFFER_FULL cause HW frame drop! (approx. Frame num #%" PRIu64 "\n", ctx->api_ctx.frame_num);
                    av_frame_unref(&ctx->buffered_fme);
                    ret = 1;
                } else {
                    av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_send_frame(): Write buffer full, enqueue frame and return 0\n");
                    ret = 0;

                    if (frame && is_input_fifo_empty(ctx)) {
                        ret = enqueue_frame(avctx, frame);
                        if (ret < 0) {
                            return ret;
                        }
                    }
                }
            }
        } else {
            ishwframe = (ctx->buffered_fme.format == AV_PIX_FMT_NI_QUAD) &&
                        (ctx->api_ctx.auto_dl_handle == 0) &&
                        (avctx->height >= NI_MIN_HEIGHT) &&
                        (avctx->width >= NI_MIN_WIDTH);

            if (!ctx->eos_fme_received && ishwframe) {
                av_log(avctx, AV_LOG_TRACE, "AVframe_index = %d at head %d\n",
                       ctx->aFree_Avframes_list[ctx->freeHead], ctx->freeHead);
                av_frame_ref(
                    ctx->sframe_pool[ctx->aFree_Avframes_list[ctx->freeHead]],
                    &ctx->buffered_fme);
                av_log(avctx, AV_LOG_TRACE,
                       "AVframe_index = %d popped from free head %d\n",
                       ctx->aFree_Avframes_list[ctx->freeHead], ctx->freeHead);
                av_log(avctx, AV_LOG_TRACE,
                       "ctx->buffered_fme.data[3] %p sframe_pool[%d]->data[3] %p\n",
                       ctx->buffered_fme.data[3],
                       ctx->aFree_Avframes_list[ctx->freeHead],
                       ctx->sframe_pool[ctx->aFree_Avframes_list[ctx->freeHead]]
                           ->data[3]);
                if (ctx->sframe_pool[ctx->aFree_Avframes_list[ctx->freeHead]]
                        ->data[3]) {
                    av_log(avctx, AV_LOG_DEBUG,
                           "nienc.c sframe_pool[%d] trace ui16FrameIdx = [%u] sent\n",
                           ctx->aFree_Avframes_list[ctx->freeHead],
                           ((niFrameSurface1_t
                                 *)((uint8_t *)ctx
                                        ->sframe_pool
                                            [ctx->aFree_Avframes_list[ctx->freeHead]]
                                        ->data[3]))
                               ->ui16FrameIdx);
                    av_log(
                        avctx, AV_LOG_TRACE,
                        "ff_xcoder_send_frame: after ref sframe_pool, hw frame "
                        "av_buffer_get_ref_count=%d, data[3]=%p\n",
                        av_buffer_get_ref_count(
                            ctx->sframe_pool[ctx->aFree_Avframes_list[ctx->freeHead]]
                                ->buf[0]),
                        ctx->sframe_pool[ctx->aFree_Avframes_list[ctx->freeHead]]
                            ->data[3]);
                }
                if (deq_free_frames(ctx) != 0) {
                    ret = AVERROR_EXTERNAL;
                    return ret;
                }
            }

            // only if it's NOT sequence change flushing (in which case only the eos
            // was sent and not the first sc pkt) AND
            // only after successful sending will it be removed from fifo
            if (SESSION_RUN_STATE_SEQ_CHANGE_DRAINING !=
                ctx->api_ctx.session_run_state) {
                if (!is_input_fifo_empty(ctx)) {
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
                    av_fifo_drain2(ctx->fme_fifo, (size_t) 1);
                    av_log(avctx, AV_LOG_DEBUG, "fme popped, fifo num frames: %lu\n",
                           av_fifo_can_read(ctx->fme_fifo));
#else
                    av_fifo_drain(ctx->fme_fifo, sizeof(AVFrame));
                    av_log(avctx, AV_LOG_DEBUG, "fme popped, fifo num frames: %lu\n",
                           av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame));
#endif
                }
                av_frame_unref(&ctx->buffered_fme);
                ishwframe = (ctx->buffered_fme.format == AV_PIX_FMT_NI_QUAD) &&
                            (ctx->api_ctx.auto_dl_handle == 0);
                if (ishwframe) {
                    if (ctx->buffered_fme.buf[0])
                        av_log(avctx, AV_LOG_TRACE, "ff_xcoder_send_frame: after unref buffered_fme, hw frame av_buffer_get_ref_count=%d\n", av_buffer_get_ref_count(ctx->buffered_fme.buf[0]));
                    else
                        av_log(avctx, AV_LOG_TRACE, "ff_xcoder_send_frame: after unref buffered_fme, hw frame av_buffer_get_ref_count=0 (buf[0] is NULL)\n");
                }
            } else {
                av_log(avctx, AV_LOG_TRACE, "XCoder frame(eos) sent, sequence changing!"
                       " NO fifo pop !\n");
            }

            // pushing input pts in circular FIFO
            ctx->api_ctx.enc_pts_list[ctx->api_ctx.enc_pts_w_idx % NI_FIFO_SZ] = ctx->api_fme.data.frame.pts;
            ctx->api_ctx.enc_pts_w_idx++;

            // have another check before return: if no more frames in fifo to send and
            // we've got eos (NULL) frame from upper stream, flag for flushing
            if (ctx->eos_fme_received && is_input_fifo_empty(ctx)) {
                av_log(avctx, AV_LOG_DEBUG, "Upper stream EOS frame received, fifo "
                       "empty, start flushing ..\n");
                ctx->encoder_flushing = 1;
            }

            ret = 0;
        }
    }

    // try to flush encoder input fifo if it's not in seqchange draining state.
    // Sending a frame before seqchange done may lead to stuck because the new frame's
    // resolution could be different from that of the last sequence. Need to flush the
    // fifo because its size increases with seqchange.
    if (ret == 0 && frame && !is_input_fifo_empty(ctx) &&
        SESSION_RUN_STATE_SEQ_CHANGE_DRAINING != ctx->api_ctx.session_run_state) {
        av_log(avctx, AV_LOG_DEBUG, "try to flush encoder input fifo. Fifo num frames: %lu\n",
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
               av_fifo_can_read(ctx->fme_fifo));
#else
               av_fifo_size(ctx->fme_fifo) / sizeof(AVFrame));
#endif
        goto resend;
    }

    if (ctx->encoder_flushing) {
        av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_send_frame flushing ..\n");
        ret = ni_device_session_flush(&ctx->api_ctx, NI_DEVICE_TYPE_ENCODER);
    }

    av_log(avctx, AV_LOG_VERBOSE, "XCoder send frame return %d \n", ret);
    return ret;
}

static void get_luma_size(int width, int height, int bit_depth, int *luma_size, int *luma_size_4n)
{
    int width_4n, height_4n;

    width_4n = ((width + 15) / 16) * 4;
    width = FFALIGN(width, 64);
    height = FFALIGN(height, 64);
    height_4n = height / 4;

    *luma_size = FFALIGN(width * 4 * bit_depth / 8, 64) * height / 4;
    *luma_size_4n = FFALIGN(width_4n * 4 * bit_depth / 8, 64) * height_4n / 4;
}

static int xcoder_encode_reinit(AVCodecContext *avctx)
{
    int ret = 0;
    XCoderEncContext *ctx = avctx->priv_data;
    bool ishwframe;
    ni_device_handle_t device_handle = ctx->api_ctx.device_handle;
    ni_device_handle_t blk_io_handle = ctx->api_ctx.blk_io_handle;
    int hw_id = ctx->api_ctx.hw_id;
    char tmp_blk_dev_name[NI_MAX_DEVICE_NAME_LEN];
    int bit_depth = 1;
    int pix_fmt = AV_PIX_FMT_YUV420P;
    int stride, luma_size, luma_size_4n, ori_stride, ori_luma_size, ori_luma_size_4n;
    bool bIsSmallPicture = false;
    AVFrame temp_frame;
    ni_xcoder_params_t *p_param = &ctx->api_param;

    ff_xcoder_strncpy(tmp_blk_dev_name, ctx->api_ctx.blk_dev_name,
                      NI_MAX_DEVICE_NAME_LEN);

    // re-init avctx's resolution to the changed one that is
    // stored in the first frame of the fifo
#if (LIBAVUTIL_VERSION_MAJOR >= 59)
    av_fifo_peek(ctx->fme_fifo, &temp_frame, (size_t) 1, NULL);
#else
    av_fifo_generic_peek(ctx->fme_fifo, &temp_frame, sizeof(AVFrame), NULL);
#endif
    temp_frame.extended_data = temp_frame.data;

    ishwframe = temp_frame.format == AV_PIX_FMT_NI_QUAD;

    if (ishwframe) {
        bit_depth = (uint8_t)((niFrameSurface1_t*)((uint8_t*)temp_frame.data[3]))->bit_depth;
        av_log(avctx, AV_LOG_INFO, "xcoder_receive_packet hw frame bit depth "
               "changing %d -> %d\n",
               ctx->api_ctx.bit_depth_factor, bit_depth);

        switch (avctx->sw_pix_fmt) {
            case AV_PIX_FMT_YUV420P:
            case AV_PIX_FMT_YUVJ420P:
                if (bit_depth == 2) {
                    avctx->sw_pix_fmt = AV_PIX_FMT_YUV420P10LE;
                    pix_fmt = NI_PIX_FMT_YUV420P10LE;
                } else {
                    pix_fmt = NI_PIX_FMT_YUV420P;
                }
                break;
            case AV_PIX_FMT_YUV420P10LE:
                if (bit_depth == 1) {
                    avctx->sw_pix_fmt = AV_PIX_FMT_YUV420P;
                    pix_fmt = NI_PIX_FMT_YUV420P;
                } else {
                    pix_fmt = NI_PIX_FMT_YUV420P10LE;
                }
                break;
            case AV_PIX_FMT_NV12:
                if (bit_depth == 2) {
                    avctx->sw_pix_fmt = AV_PIX_FMT_P010LE;
                    pix_fmt = NI_PIX_FMT_P010LE;
                } else {
                    pix_fmt = NI_PIX_FMT_NV12;
                }
                break;
            case AV_PIX_FMT_P010LE:
                if (bit_depth == 1) {
                    avctx->sw_pix_fmt = AV_PIX_FMT_NV12;
                    pix_fmt = NI_PIX_FMT_NV12;
                } else {
                    pix_fmt = NI_PIX_FMT_P010LE;
                }
                break;
            case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
                if (bit_depth == 1) {
                    avctx->sw_pix_fmt = AV_PIX_FMT_NI_QUAD_8_TILE_4X4;
                    pix_fmt = NI_PIX_FMT_8_TILED4X4;
                } else {
                    pix_fmt = NI_PIX_FMT_10_TILED4X4;
                }
                break;
            case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
                if (bit_depth == 2) {
                    avctx->sw_pix_fmt = AV_PIX_FMT_NI_QUAD_10_TILE_4X4;
                    pix_fmt = NI_PIX_FMT_10_TILED4X4;
                } else {
                    pix_fmt = NI_PIX_FMT_8_TILED4X4;
                }
                break;
            case AV_PIX_FMT_ARGB:
                pix_fmt = NI_PIX_FMT_ARGB;
                break;
            case AV_PIX_FMT_ABGR:
                pix_fmt = NI_PIX_FMT_ABGR;
                break;
            case AV_PIX_FMT_RGBA:
                pix_fmt = NI_PIX_FMT_RGBA;
                break;
            case AV_PIX_FMT_BGRA:
                pix_fmt = NI_PIX_FMT_BGRA;
                break;
            default:
                pix_fmt = NI_PIX_FMT_NONE;
                break;
        }
    } else {
        switch (temp_frame.format) {
            case AV_PIX_FMT_YUV420P:
            case AV_PIX_FMT_YUVJ420P:
                pix_fmt = NI_PIX_FMT_YUV420P;
                bit_depth = 1;
                break;
            case AV_PIX_FMT_NV12:
                pix_fmt = NI_PIX_FMT_NV12;
                bit_depth = 1;
                break;
            case AV_PIX_FMT_YUV420P10LE:
                pix_fmt = NI_PIX_FMT_YUV420P10LE;
                bit_depth = 2;
                break;
            case AV_PIX_FMT_P010LE:
                pix_fmt = NI_PIX_FMT_P010LE;
                bit_depth = 2;
                break;
            default:
                pix_fmt = NI_PIX_FMT_NONE;
                break;
        }
    }

    ctx->eos_fme_received = 0;
    ctx->encoder_eof = 0;
    ctx->encoder_flushing = 0;
    ctx->firstPktArrived = 0;
    ctx->spsPpsArrived = 0;
    ctx->spsPpsHdrLen = 0;
    av_freep(&ctx->p_spsPpsHdr);
    ctx->seqChangeCount++;

    // check if resolution is zero copy compatible and set linesize according to new resolution
    if (ni_encoder_frame_zerocopy_check(&ctx->api_ctx,
        p_param, temp_frame.width, temp_frame.height,
        (const int *)temp_frame.linesize, true) == NI_RETCODE_SUCCESS) {
        stride = p_param->luma_linesize; // new sequence is zero copy compatible
    } else {
        stride = FFALIGN(temp_frame.width*bit_depth, 128);
    }

    if (ctx->api_ctx.ori_luma_linesize && ctx->api_ctx.ori_chroma_linesize) {
        ori_stride = ctx->api_ctx.ori_luma_linesize; // previous sequence was zero copy compatible
    } else {
        ori_stride = FFALIGN(ctx->api_ctx.ori_width*bit_depth, 128);
    }

    get_luma_size(temp_frame.width, temp_frame.height, bit_depth == 2 ? 10 : 8, &luma_size, &luma_size_4n);
    get_luma_size(ctx->api_ctx.ori_width, ctx->api_ctx.ori_height, ctx->api_ctx.ori_bit_depth_factor == 2 ? 10 : 8,
                  &ori_luma_size, &ori_luma_size_4n);

    if (pix_fmt == NI_PIX_FMT_ARGB
       || pix_fmt == NI_PIX_FMT_ABGR
       || pix_fmt == NI_PIX_FMT_RGBA
       || pix_fmt == NI_PIX_FMT_BGRA) {
        stride = temp_frame.width;
        ori_stride = ctx->api_ctx.ori_width;
        luma_size = luma_size_4n = temp_frame.width * temp_frame.height;
        ori_luma_size = ori_luma_size_4n = ctx->api_ctx.ori_width * ctx->api_ctx.ori_height;
    }

    if (ctx->api_param.cfg_enc_params.lookAheadDepth
        || ctx->api_param.cfg_enc_params.crf >= 0
        || ctx->api_param.cfg_enc_params.crfFloat >= 0) {
        av_log(avctx, AV_LOG_DEBUG, "xcoder_encode_reinit 2-pass "
               "lookaheadDepth  %d and/or CRF %d and/or CRFFloat %f\n",
               ctx->api_param.cfg_enc_params.lookAheadDepth,
               ctx->api_param.cfg_enc_params.crf,
               ctx->api_param.cfg_enc_params.crfFloat);
        if ((temp_frame.width < NI_2PASS_ENCODE_MIN_WIDTH) ||
            (temp_frame.height < NI_2PASS_ENCODE_MIN_HEIGHT)) {
            bIsSmallPicture = true;
        }
    } else {
        if ((temp_frame.width < NI_MIN_WIDTH) ||
           (temp_frame.height < NI_MIN_HEIGHT)) {
            bIsSmallPicture = true;
        }
    }

    if (ctx->api_param.cfg_enc_params.multicoreJointMode) {
        av_log(avctx, AV_LOG_DEBUG, "xcoder_encode_reinit multicore "
               "joint mode\n");
        if ((temp_frame.width < 256) ||
           (temp_frame.height < 256)) {
            bIsSmallPicture = true;
        }
    }

    if (ctx->api_param.cfg_enc_params.crop_width || ctx->api_param.cfg_enc_params.crop_height) {
        av_log(avctx, AV_LOG_DEBUG, "xcoder_encode_reinit needs to close and re-open "
               " due to crop width x height \n");
        bIsSmallPicture = true;
    }

    av_log(avctx, AV_LOG_INFO, "%s resolution "
           "changing %dx%d -> %dx%d "
           "format %d -> %d "
           "original stride %d height %d luma size %d/%d pix fmt %d "
           "new stride %d height %d luma size %d/%d pix fmt %d \n",
           __func__, avctx->width, avctx->height,
           temp_frame.width, temp_frame.height,
           avctx->pix_fmt, temp_frame.format,
           ori_stride, ctx->api_ctx.ori_height,
           ori_luma_size, ori_luma_size_4n, ctx->api_ctx.ori_pix_fmt,
           stride, temp_frame.height, luma_size, luma_size_4n, pix_fmt);

    avctx->width = temp_frame.width;
    avctx->height = temp_frame.height;
    avctx->pix_fmt = temp_frame.format;

    // fast sequence change without close / open only if new resolution < original resolution
    if ((ori_stride*ctx->api_ctx.ori_height < stride*temp_frame.height) ||
        (ori_luma_size < luma_size) || (ori_luma_size_4n < luma_size_4n) ||
        (ctx->api_ctx.ori_pix_fmt != pix_fmt) ||
        bIsSmallPicture ||
        (avctx->codec_id == AV_CODEC_ID_MJPEG) ||
        ctx->api_param.cfg_enc_params.disable_adaptive_buffers) {
        ff_xcoder_encode_close(avctx);
        ret = ff_xcoder_encode_init(avctx);
        // clear crop parameters upon sequence change because cropping values may not be compatible to new resolution
        // (except for Motion Constrained mode 2, for which we crop to 64x64 alignment)
        if (ctx->api_param.cfg_enc_params.motionConstrainedMode == MOTION_CONSTRAINED_QUALITY_MODE && avctx->codec_id == AV_CODEC_ID_HEVC) {
            ctx->api_param.cfg_enc_params.crop_width = (temp_frame.width / 64 * 64);
            ctx->api_param.cfg_enc_params.crop_height = (temp_frame.height / 64 * 64);
            ctx->api_param.cfg_enc_params.hor_offset = ctx->api_param.cfg_enc_params.ver_offset = 0;
            av_log(avctx, AV_LOG_DEBUG, "xcoder_encode_reinit sets "
                   "crop width x height to %d x %d for Motion Constrained mode 2\n",
                   ctx->api_param.cfg_enc_params.crop_width,
                   ctx->api_param.cfg_enc_params.crop_height);
        } else {
            ctx->api_param.cfg_enc_params.crop_width = ctx->api_param.cfg_enc_params.crop_height = 0;
            ctx->api_param.cfg_enc_params.hor_offset = ctx->api_param.cfg_enc_params.ver_offset = 0;
        }
    } else {
        if (avctx->codec_id == AV_CODEC_ID_AV1) {
            // AV1 8x8 alignment HW limitation is now worked around by FW cropping input resolution
            if (temp_frame.width % NI_PARAM_AV1_ALIGN_WIDTH_HEIGHT)
                av_log(avctx, AV_LOG_ERROR,
                       "resolution change: AV1 Picture Width not aligned to %d - picture will be cropped\n",
                       NI_PARAM_AV1_ALIGN_WIDTH_HEIGHT);

            if (temp_frame.height % NI_PARAM_AV1_ALIGN_WIDTH_HEIGHT)
                av_log(avctx, AV_LOG_ERROR,
                       "resolution change: AV1 Picture Height not aligned to %d - picture will be cropped\n",
                       NI_PARAM_AV1_ALIGN_WIDTH_HEIGHT);
        }
        ret = xcoder_encode_sequence_change(avctx, temp_frame.width, temp_frame.height, bit_depth);
    }

    // keep device handle(s) open during sequence change to fix mem bin buffer not recycled
    ctx->api_ctx.device_handle  = device_handle;
    ctx->api_ctx.blk_io_handle = blk_io_handle;
    ctx->api_ctx.hw_id = hw_id;
    ff_xcoder_strncpy(ctx->api_ctx.blk_dev_name, tmp_blk_dev_name,
                      NI_MAX_DEVICE_NAME_LEN);
    ctx->api_ctx.session_run_state = SESSION_RUN_STATE_SEQ_CHANGE_OPENING; // this state is referenced when sending first frame after sequence change

    return ret;
}

int ff_xcoder_receive_packet2(AVCodecContext *avctx, AVPacket *pkt)
{
    XCoderEncContext *ctx = avctx->priv_data;
    int i, ret = 0;
    int recv;
    AVFrame *frame = NULL;
    ni_packet_t *xpkt = &ctx->api_pkt.data.packet;
    bool av1_output_frame = 0;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder receive packet\n");

    if (ctx->encoder_eof) {
        av_log(avctx, AV_LOG_VERBOSE, "ff_xcoder_receive_packet2: EOS\n");
        return AVERROR_EOF;
    }

    if (ni_packet_buffer_alloc(xpkt, NI_MAX_TX_SZ)) {
        av_log(avctx, AV_LOG_ERROR,
               "ff_xcoder_receive_packet2: packet buffer size %d allocation failed\n",
               NI_MAX_TX_SZ);
        return AVERROR(ENOMEM);
    }

    if (avctx->codec_id == AV_CODEC_ID_MJPEG && (!ctx->spsPpsArrived)) {
        ctx->spsPpsArrived = 1;
        // for Jpeg, start pkt_num counter from 1, because unlike video codecs
        // (1st packet is header), there is no header for Jpeg
        ctx->api_ctx.pkt_num = 1;
    }

    while (1) {
        xpkt->recycle_index = -1;
        recv = ni_device_session_read(&ctx->api_ctx, &(ctx->api_pkt), NI_DEVICE_TYPE_ENCODER);

        av_log(avctx, AV_LOG_TRACE,
               "XCoder receive packet: xpkt.end_of_stream=%d, xpkt.data_len=%d, "
               "xpkt.frame_type=%d, recv=%d, encoder_flushing=%d, encoder_eof=%d\n",
               xpkt->end_of_stream, xpkt->data_len, xpkt->frame_type, recv,
               ctx->encoder_flushing, ctx->encoder_eof);

        if (recv <= 0) {
            ctx->encoder_eof = xpkt->end_of_stream;
            if (ctx->encoder_eof || xpkt->end_of_stream) {
                if (SESSION_RUN_STATE_SEQ_CHANGE_DRAINING ==
                    ctx->api_ctx.session_run_state) {
                    // after sequence change completes, reset codec state
                    av_log(avctx, AV_LOG_INFO, "ff_xcoder_receive_packet2 1: sequence "
                           "change completed, return AVERROR(EAGAIN) and will reopen "
                           "codec!\n");

                    ret = xcoder_encode_reinit(avctx);
                    av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_receive_packet2: xcoder_encode_reinit ret %d\n", ret);
                    if (ret >= 0) {
                        ret = AVERROR(EAGAIN);

                        ff_xcoder_send_frame(avctx, NULL);

                        ctx->api_ctx.session_run_state = SESSION_RUN_STATE_NORMAL;
                    }
                    break;
                }

                ret = AVERROR_EOF;
                av_log(avctx, AV_LOG_VERBOSE, "ff_xcoder_receive_packet2: got encoder_eof, return AVERROR_EOF\n");
                break;
            } else {
                bool bIsReset = false;
                if (NI_RETCODE_ERROR_VPU_RECOVERY == recv) {
                    xcoder_encode_reset(avctx);
                    bIsReset = true;
                } else if (NI_RETCODE_ERROR_INVALID_SESSION == recv) {
                    av_log(ctx, AV_LOG_ERROR, "encoder read retval %d\n", recv);
                    ret = AVERROR(EIO);
                    break;
                }
                ret = AVERROR(EAGAIN);
                if ((!ctx->encoder_flushing && !ctx->eos_fme_received) || bIsReset) { // if encode session was reset, can't read again with invalid session, must break out first
                    av_log(avctx, AV_LOG_TRACE, "ff_xcoder_receive_packet2: NOT encoder_"
                           "flushing, NOT eos_fme_received, return AVERROR(EAGAIN)\n");
                    break;
                }
            }
        } else {
            /* got encoded data back */
            uint8_t *p_src, *p_end;
            int64_t local_pts;
            ni_custom_sei_set_t *p_custom_sei_set;
            int meta_size = ctx->api_ctx.meta_size;
            uint32_t copy_len = 0;
            uint32_t data_len = 0;
            int total_custom_sei_size = 0;
            int custom_sei_count = 0;

            if (avctx->pix_fmt == AV_PIX_FMT_NI_QUAD && xpkt->recycle_index >= 0 &&
                avctx->height >= NI_MIN_HEIGHT && avctx->width >= NI_MIN_WIDTH &&
                xpkt->recycle_index < NI_GET_MAX_HWDESC_P2P_BUF_ID(ctx->api_ctx.ddr_config)) {
                int avframe_index =
                    recycle_index_2_avframe_index(ctx, xpkt->recycle_index);
                av_log(avctx, AV_LOG_VERBOSE, "UNREF trace ui16FrameIdx = [%d].\n",
                       xpkt->recycle_index);
                if (avframe_index >= 0 && ctx->sframe_pool[avframe_index]) {
                    av_frame_unref(ctx->sframe_pool[avframe_index]);
                    av_log(avctx, AV_LOG_DEBUG,
                           "AVframe_index = %d pushed to free tail %d\n",
                           avframe_index, ctx->freeTail);
                    enq_free_frames(ctx, avframe_index);
                    // enqueue the index back to free
                    xpkt->recycle_index = -1;
                } else {
                    av_log(avctx, AV_LOG_DEBUG,
                           "can't push to tail - avframe_index %d sframe_pool %p\n",
                           avframe_index, ctx->sframe_pool[avframe_index]);
                }
            }

            if (!ctx->spsPpsArrived) {
                ret = AVERROR(EAGAIN);
                ctx->spsPpsArrived = 1;
                ctx->spsPpsHdrLen = recv - meta_size;
                ctx->p_spsPpsHdr = av_malloc(ctx->spsPpsHdrLen);
                if (!ctx->p_spsPpsHdr) {
                    ret = AVERROR(ENOMEM);
                    break;
                }

                memcpy(ctx->p_spsPpsHdr, (uint8_t *)xpkt->p_data + meta_size,
                       xpkt->data_len - meta_size);

                // start pkt_num counter from 1 to get the real first frame
                ctx->api_ctx.pkt_num = 1;
                // for low-latency mode, keep reading until the first frame is back
                if (ctx->api_param.low_delay_mode) {
                    av_log(avctx, AV_LOG_TRACE, "XCoder receive packet: low delay mode,"
                           " keep reading until 1st pkt arrives\n");
                    continue;
                }
                break;
            }

            // handle pic skip
            if (xpkt->frame_type == 3) { // 0=I, 1=P, 2=B, 3=not coded / skip
                ret = AVERROR(EAGAIN);
                if (ctx->first_frame_pts == INT_MIN)
                    ctx->first_frame_pts = xpkt->pts;
                if (AV_CODEC_ID_AV1 == avctx->codec_id) {
                    ctx->latest_dts = xpkt->pts;
                } else if (ctx->total_frames_received < ctx->dtsOffset) {
                    // guess dts
#if LIBAVCODEC_VERSION_MAJOR < 62
                    ctx->latest_dts = ctx->first_frame_pts +
                                      ((ctx->gop_offset_count - ctx->dtsOffset) *
                                       avctx->ticks_per_frame);
#else
                    ctx->latest_dts = ctx->first_frame_pts +
                                      ctx->gop_offset_count - ctx->dtsOffset;
#endif
                    ctx->gop_offset_count++;
                } else {
                    // get dts from pts FIFO
                    ctx->latest_dts =
                        ctx->api_ctx
                            .enc_pts_list[ctx->api_ctx.enc_pts_r_idx % NI_FIFO_SZ];
                    ctx->api_ctx.enc_pts_r_idx++;
                }
                if (ctx->latest_dts > xpkt->pts) {
                    ctx->latest_dts = xpkt->pts;
                }
                ctx->total_frames_received++;

                if (!ctx->encoder_flushing && !ctx->eos_fme_received) {
                    av_log(avctx, AV_LOG_TRACE, "ff_xcoder_receive_packet2: skip"
                           " picture output, return AVERROR(EAGAIN)\n");
                    break;
                } else {
                    continue;
                }
            }

            // store av1 packets to be merged & sent along with future packet
            if (avctx->codec_id == AV_CODEC_ID_AV1) {
                av_log(
                    avctx, AV_LOG_TRACE,
                    "ff_xcoder_receive_packet2: AV1 xpkt buf %p size %d show_frame %d\n",
                    xpkt->p_data, xpkt->data_len, xpkt->av1_show_frame);
                if (!xpkt->av1_show_frame) {
                    // store AV1 packets
                    xpkt->av1_p_buffer[xpkt->av1_buffer_index] = xpkt->p_buffer;
                    xpkt->av1_p_data[xpkt->av1_buffer_index] = xpkt->p_data;
                    xpkt->av1_buffer_size[xpkt->av1_buffer_index] = xpkt->buffer_size;
                    xpkt->av1_data_len[xpkt->av1_buffer_index] = xpkt->data_len;
                    xpkt->av1_buffer_index++;
                    xpkt->p_buffer = NULL;
                    xpkt->p_data = NULL;
                    xpkt->buffer_size = 0;
                    xpkt->data_len = 0;
                    if (xpkt->av1_buffer_index >= MAX_AV1_ENCODER_GOP_NUM) {
                        av_log(avctx, AV_LOG_ERROR,
                               "ff_xcoder_receive_packet2: recv AV1 not shown frame "
                               "number %d >= %d, return AVERROR_EXTERNAL\n",
                               xpkt->av1_buffer_index, MAX_AV1_ENCODER_GOP_NUM);
                        ret = AVERROR_EXTERNAL;
                        break;
                    } else if (!ctx->encoder_flushing && !ctx->eos_fme_received) {
                        av_log(avctx, AV_LOG_TRACE,
                               "ff_xcoder_receive_packet2: recv AV1 not shown frame, "
                               "return AVERROR(EAGAIN)\n");
                        ret = AVERROR(EAGAIN);
                        break;
                    } else {
                        if (ni_packet_buffer_alloc(xpkt, NI_MAX_TX_SZ)) {
                            av_log(avctx, AV_LOG_ERROR,
                                   "ff_xcoder_receive_packet2: AV1 packet buffer size %d "
                                   "allocation failed during flush\n",
                                   NI_MAX_TX_SZ);
                            ret = AVERROR(ENOMEM);
                            break;
                        }
                        av_log(avctx, AV_LOG_TRACE,
                               "ff_xcoder_receive_packet2: recv AV1 not shown frame "
                               "during flush, continue..\n");
                        continue;
                    }
                } else {
                    // calculate length of previously received AV1 packets pending for merge
                    av1_output_frame = 1;
                    for (i = 0; i < xpkt->av1_buffer_index; i++) {
                        data_len += xpkt->av1_data_len[i] - meta_size;
                    }
                }
            }

            p_src = (uint8_t*)xpkt->p_data + meta_size;
            p_end = p_src + (xpkt->data_len - meta_size);
            local_pts = xpkt->pts;

            p_custom_sei_set = ctx->api_ctx.pkt_custom_sei_set[local_pts % NI_FIFO_SZ];
            if (p_custom_sei_set != NULL) {
                custom_sei_count = p_custom_sei_set->count;
                for (i = 0; i < p_custom_sei_set->count; i++) {
                    total_custom_sei_size += p_custom_sei_set->custom_sei[i].size;
                }
            }

            if (custom_sei_count) {
                // if HRD or custom sei enabled, search for pic_timing or custom SEI insertion point by
                // skipping non-VCL until video data is found.
                uint32_t nalu_type = 0;
                const uint8_t *p_start_code = p_src;
                uint32_t stc = -1;
                if (AV_CODEC_ID_HEVC == avctx->codec_id) {
                    do {
                        stc = -1;
                        p_start_code = avpriv_find_start_code(p_start_code, p_end, &stc);
                        nalu_type = (stc >> 1) & 0x3F;
                    } while (nalu_type > HEVC_NAL_RSV_VCL31);

                    // calc. length to copy
                    copy_len = p_start_code - 5 - p_src;
                } else if (AV_CODEC_ID_H264 == avctx->codec_id) {
                    do {
                        stc = -1;
                        p_start_code = avpriv_find_start_code(p_start_code, p_end, &stc);
                        nalu_type = stc & 0x1F;
                    } while (nalu_type > H264_NAL_IDR_SLICE);

                    // calc. length to copy
                    copy_len = p_start_code - 5 - p_src;
                } else {
                    av_log(avctx, AV_LOG_ERROR, "xcoder_receive packet: codec %d not "
                           "supported for SEI !\n", avctx->codec_id);
                }
            }

            if (avctx->codec_id == AV_CODEC_ID_MJPEG && !ctx->firstPktArrived) {
                // there is no header for Jpeg, so skip header copy
                ctx->firstPktArrived = 1;
                if (ctx->first_frame_pts == INT_MIN) {
                    ctx->first_frame_pts = xpkt->pts;
                }
            }

            if (!ctx->firstPktArrived) {
                int sizeof_spspps_attached_to_idr = ctx->spsPpsHdrLen;
                int seq_change_in_mp4_or_mov = (ctx->seqChangeCount > 0) && (avctx->flags & AV_CODEC_FLAG_MP4_MOV);
                if ((avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) &&
                    (avctx->codec_id != AV_CODEC_ID_AV1) &&
                    (!seq_change_in_mp4_or_mov)) {
                    sizeof_spspps_attached_to_idr = 0;
                }
                ctx->firstPktArrived = 1;
                if (ctx->first_frame_pts == INT_MIN) {
                    ctx->first_frame_pts = xpkt->pts;
                }

                data_len += xpkt->data_len - meta_size + sizeof_spspps_attached_to_idr + total_custom_sei_size;
                if (avctx->codec_id == AV_CODEC_ID_AV1)
                    av_log(avctx, AV_LOG_TRACE, "ff_xcoder_receive_packet2: AV1 first output pkt size %d\n", data_len);

#if (LIBAVCODEC_VERSION_MAJOR >= 59)
                ret = ff_get_encode_buffer(avctx, pkt, data_len, 0);
#else
                ret = ff_alloc_packet2(avctx, pkt, data_len, data_len);
#endif

                if (!ret) {
                    uint8_t *p_dst, *p_side_data;

                    // fill in AVC/HEVC sidedata
                    if ((avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER) &&
                        (avctx->extradata_size != ctx->spsPpsHdrLen ||
                         (memcmp(avctx->extradata, ctx->p_spsPpsHdr, ctx->spsPpsHdrLen) !=
                          0))) {
                        avctx->extradata_size = ctx->spsPpsHdrLen;
                        av_freep(&avctx->extradata);
                        avctx->extradata = av_mallocz(avctx->extradata_size +
                                                  AV_INPUT_BUFFER_PADDING_SIZE);
                        if (!avctx->extradata) {
                            av_log(avctx, AV_LOG_ERROR,
                                   "Cannot allocate AVC/HEVC header of size %d.\n",
                                   avctx->extradata_size);
                            return AVERROR(ENOMEM);
                        }
                        memcpy(avctx->extradata, ctx->p_spsPpsHdr, avctx->extradata_size);
                    }

                    p_side_data = av_packet_new_side_data(
                        pkt, AV_PKT_DATA_NEW_EXTRADATA, ctx->spsPpsHdrLen);
                    if (p_side_data) {
                        memcpy(p_side_data, ctx->p_spsPpsHdr, ctx->spsPpsHdrLen);
                    }

                    p_dst = pkt->data;
                    if (sizeof_spspps_attached_to_idr) {
                        memcpy(p_dst, ctx->p_spsPpsHdr, ctx->spsPpsHdrLen);
                        p_dst += ctx->spsPpsHdrLen;
                    }

                    if (custom_sei_count && avctx->codec_id != AV_CODEC_ID_AV1) {
                        // copy buf_period
                        memcpy(p_dst, p_src, copy_len);
                        p_dst += copy_len;

                        for (i = 0; i < custom_sei_count; i++) {
                            // copy custom sei
                            ni_custom_sei_t *p_custom_sei = &p_custom_sei_set->custom_sei[i];
                            if (p_custom_sei->location == NI_CUSTOM_SEI_LOC_AFTER_VCL) {
                                break;
                            }
                            memcpy(p_dst, &p_custom_sei->data[0], p_custom_sei->size);
                            p_dst += p_custom_sei->size;
                        }

                        // copy the IDR data
                        memcpy(p_dst, p_src + copy_len,
                               xpkt->data_len - meta_size - copy_len);
                        p_dst += xpkt->data_len - meta_size - copy_len;

                        // copy custom sei after slice
                        for (; i < custom_sei_count; i++) {
                            ni_custom_sei_t *p_custom_sei = &p_custom_sei_set->custom_sei[i];
                            memcpy(p_dst, &p_custom_sei->data[0], p_custom_sei->size);
                            p_dst += p_custom_sei->size;
                        }
                    } else {
                        // merge AV1 packets
                        if (avctx->codec_id == AV_CODEC_ID_AV1) {
                            for (i = 0; i < xpkt->av1_buffer_index; i++) {
                                memcpy(p_dst, (uint8_t *)xpkt->av1_p_data[i] + meta_size,
                                       xpkt->av1_data_len[i] - meta_size);
                                p_dst += (xpkt->av1_data_len[i] - meta_size);
                            }
                        }

                        memcpy(p_dst, (uint8_t*)xpkt->p_data + meta_size,
                               xpkt->data_len - meta_size);
                    }
                }
            } else {
                data_len += xpkt->data_len - meta_size + total_custom_sei_size;
                if (avctx->codec_id == AV_CODEC_ID_AV1)
                    av_log(avctx, AV_LOG_TRACE, "ff_xcoder_receive_packet2: AV1 output pkt size %d\n", data_len);

#if (LIBAVCODEC_VERSION_MAJOR >= 59)
                ret = ff_get_encode_buffer(avctx, pkt, data_len, 0);
#else
                ret = ff_alloc_packet2(avctx, pkt, data_len, data_len);
#endif

                if (!ret) {
                    uint8_t *p_dst = pkt->data;

                    if (custom_sei_count && avctx->codec_id != AV_CODEC_ID_AV1) {
                        // copy buf_period
                        memcpy(p_dst, p_src, copy_len);
                        p_dst += copy_len;

                        for (i = 0; i < custom_sei_count; i++) {
                            // copy custom sei
                            ni_custom_sei_t *p_custom_sei = &p_custom_sei_set->custom_sei[i];
                            if (p_custom_sei->location == NI_CUSTOM_SEI_LOC_AFTER_VCL) {
                                break;
                            }
                            memcpy(p_dst, &p_custom_sei->data[0], p_custom_sei->size);
                            p_dst += p_custom_sei->size;
                        }

                        // copy the packet data
                        memcpy(p_dst, p_src + copy_len,
                               xpkt->data_len - meta_size - copy_len);
                        p_dst += xpkt->data_len - meta_size - copy_len;

                        // copy custom sei after slice
                        for (; i < custom_sei_count; i++) {
                            ni_custom_sei_t *p_custom_sei = &p_custom_sei_set->custom_sei[i];
                            memcpy(p_dst, &p_custom_sei->data[0], p_custom_sei->size);
                            p_dst += p_custom_sei->size;
                        }
                    } else {
                        // merge AV1 packets
                        if (avctx->codec_id == AV_CODEC_ID_AV1) {
                            for (i = 0; i < xpkt->av1_buffer_index; i++) {
                                memcpy(p_dst, (uint8_t *)xpkt->av1_p_data[i] + meta_size,
                                       xpkt->av1_data_len[i] - meta_size);
                                p_dst += (xpkt->av1_data_len[i] - meta_size);
                            }
                        }

                        memcpy(p_dst, (uint8_t *)xpkt->p_data + meta_size,
                               xpkt->data_len - meta_size);
                    }
                }
            }

#if ((LIBXCODER_API_VERSION_MAJOR > 2) ||                                        \
      (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 66))
            if (ctx->api_param.cfg_enc_params.get_psnr_mode < 3 ||
                ctx->api_param.cfg_enc_params.enable_ssim) {
                ni_pkt_info pkt_info = {0};
                uint8_t* p_side_data = NULL;

                if (ctx->api_param.cfg_enc_params.get_psnr_mode < 3) {
                    pkt_info.psnr_y = xpkt->psnr_y;
                    pkt_info.psnr_u = xpkt->psnr_u;
                    pkt_info.psnr_v = xpkt->psnr_v;
                    pkt_info.average_psnr = xpkt->average_psnr;
                }
                if (ctx->api_param.cfg_enc_params.enable_ssim) {
                    pkt_info.ssim_y = xpkt->ssim_y;
                    pkt_info.ssim_u = xpkt->ssim_u;
                    pkt_info.ssim_v = xpkt->ssim_v;
                }

                p_side_data = av_packet_new_side_data(pkt, AV_PKT_DATA_PKT_INFO, sizeof(ni_pkt_info));
                if (p_side_data) {
                    memcpy(p_side_data, &pkt_info, sizeof(ni_pkt_info));
                } else {
                    av_log(avctx, AV_LOG_ERROR, "Cannot allocate PSNR/SSIM info size %lu.\n", sizeof(ni_pkt_info));
                    return AVERROR(ENOMEM);
                }
                av_log(avctx, AV_LOG_DEBUG,"Add PSNR Y:%lf U:%lf V:%lf Ave:%lf SSIM Y:%lf U:%lf V:%lf to packet side data\n",
                          pkt_info.psnr_y, pkt_info.psnr_u, pkt_info.psnr_v, pkt_info.average_psnr,
                          pkt_info.ssim_y, pkt_info.ssim_u, pkt_info.ssim_v);
            }
#else
            if (ctx->api_param.cfg_enc_params.get_psnr_mode < 3) {
                ni_pkt_info pkt_info;
                pkt_info.psnr_y = ctx->api_ctx.psnr_y;
                pkt_info.psnr_u = ctx->api_ctx.psnr_u;
                pkt_info.psnr_v = ctx->api_ctx.psnr_v;
                pkt_info.average_psnr = ctx->api_ctx.average_psnr;
                uint8_t* p_side_data = NULL;

                p_side_data = av_packet_new_side_data(pkt, AV_PKT_DATA_PKT_INFO, sizeof(ni_pkt_info));
                if (p_side_data) {
                    memcpy(p_side_data, &pkt_info, sizeof(ni_pkt_info));
                } else {
                    av_log(avctx, AV_LOG_ERROR, "Cannot allocate PSNR info size %lu.\n", sizeof(ni_pkt_info));
                    return AVERROR(ENOMEM);
                }
                av_log(avctx, AV_LOG_DEBUG,"Add PSNR Y:%lf U:%lf V:%lf Ava:%lf to packet side data\n",
                          ctx->api_ctx.psnr_y,ctx->api_ctx.psnr_u, ctx->api_ctx.psnr_v,ctx->api_ctx.average_psnr);
            }
#endif

            // free buffer
            if (custom_sei_count) {
                ni_memfree(p_custom_sei_set);
                ctx->api_ctx.pkt_custom_sei_set[local_pts % NI_FIFO_SZ] = NULL;
            }

            if (!ret) {
                if (xpkt->frame_type == 0) {
                    pkt->flags |= AV_PKT_FLAG_KEY;
                }

                pkt->pts = xpkt->pts;
                /* to ensure pts>dts for all frames, we assign a guess pts for the first 'dtsOffset' frames and then the pts from input stream
                 * is extracted from input pts FIFO.
                 * if GOP = IBBBP and PTSs = 0 1 2 3 4 5 .. then out DTSs = -3 -2 -1 0 1 ... and -3 -2 -1 are the guessed values
                 * if GOP = IBPBP and PTSs = 0 1 2 3 4 5 .. then out DTSs = -1 0 1 2 3 ... and -1 is the guessed value
                 * the number of guessed values is equal to dtsOffset
                 */
                if (AV_CODEC_ID_AV1 == avctx->codec_id) {
                    pkt->dts = pkt->pts;
                    av_log(avctx, AV_LOG_TRACE, "Packet dts (av1): %ld\n", pkt->dts);
                } else if (ctx->total_frames_received < ctx->dtsOffset) {
                    // guess dts
#if LIBAVCODEC_VERSION_MAJOR < 62
                    pkt->dts = ctx->first_frame_pts +
                               ((ctx->gop_offset_count - ctx->dtsOffset) *
                                avctx->ticks_per_frame);
#else
                    pkt->dts = ctx->first_frame_pts + ctx->gop_offset_count - ctx->dtsOffset;
#endif
                    ctx->gop_offset_count++;
                    av_log(avctx, AV_LOG_TRACE, "Packet dts (guessed): %ld\n",
                           pkt->dts);
                } else {
                    // get dts from pts FIFO
                    pkt->dts =
                        ctx->api_ctx
                            .enc_pts_list[ctx->api_ctx.enc_pts_r_idx % NI_FIFO_SZ];
                    ctx->api_ctx.enc_pts_r_idx++;
                    av_log(avctx, AV_LOG_TRACE, "Packet dts: %ld\n", pkt->dts);
                }
                if (ctx->total_frames_received >= 1) {
                    if (pkt->dts < ctx->latest_dts) {
                        av_log(NULL, AV_LOG_WARNING, "dts: %ld < latest_dts: %ld.\n",
                                pkt->dts, ctx->latest_dts);
                    }
                }
                if (pkt->pts < ctx->first_frame_pts) {
                    av_log(NULL, AV_LOG_WARNING, "pts %ld less than first frame pts %ld. Force it to first frame pts\n",
                            pkt->pts, ctx->first_frame_pts);
                    pkt->pts = ctx->first_frame_pts;
                }
                if (pkt->dts > pkt->pts) {
                    av_log(NULL, AV_LOG_WARNING, "dts: %ld, pts: %ld. Forcing dts = pts \n",
                            pkt->dts, pkt->pts);
                    pkt->dts = pkt->pts;
                    av_log(avctx, AV_LOG_TRACE, "Force dts to: %ld\n", pkt->dts);
                }
                ctx->total_frames_received++;
                ctx->latest_dts = pkt->dts;
                av_log(avctx, AV_LOG_DEBUG, "XCoder recv pkt #%" PRId64 ""
                       " pts %" PRId64 "  dts %" PRId64 "  size %d  st_index %d frame_type %u avg qp %u\n",
                       ctx->api_ctx.pkt_num - 1, pkt->pts, pkt->dts, pkt->size,
                       pkt->stream_index, xpkt->frame_type, xpkt->avg_frame_qp);

                enum AVPictureType pict_type = AV_PICTURE_TYPE_NONE;
                switch (xpkt->frame_type) {
                case 0:
                  pict_type = AV_PICTURE_TYPE_I;
                  break;
                case 1:
                  pict_type = AV_PICTURE_TYPE_P;
                  break;
                case 2:
                  pict_type = AV_PICTURE_TYPE_B;
                  break;
                default:
                  break;
                }

                int frame_qp = 0;
                switch (avctx->codec_id) {
                case AV_CODEC_ID_H264:
                case AV_CODEC_ID_HEVC:
                  frame_qp = xpkt->avg_frame_qp;
                  break;
                default:
                  break;
                }

                ff_side_data_set_encoder_stats(pkt, frame_qp * FF_QP2LAMBDA, NULL, 0, pict_type);
            }
            ctx->encoder_eof = xpkt->end_of_stream;
            if (ctx->encoder_eof &&
                SESSION_RUN_STATE_SEQ_CHANGE_DRAINING ==
                ctx->api_ctx.session_run_state) {
                // after sequence change completes, reset codec state
                av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_receive_packet2 2: sequence change "
                    "completed, return 0 and will reopen codec !\n");
                ret = xcoder_encode_reinit(avctx);
                av_log(avctx, AV_LOG_DEBUG, "ff_xcoder_receive_packet2: xcoder_encode_reinit ret %d\n", ret);
                if (ret >= 0) {
                    ff_xcoder_send_frame(avctx, NULL);
                    ctx->api_ctx.session_run_state = SESSION_RUN_STATE_NORMAL;
                }
            }
            break;
        }
    }

    if ((AV_CODEC_ID_AV1 == avctx->codec_id) && xpkt->av1_buffer_index &&
        av1_output_frame) {
        av_log(avctx, AV_LOG_TRACE,
               "ff_xcoder_receive_packet2: ni_packet_buffer_free_av1 %d packtes\n",
               xpkt->av1_buffer_index);
        ni_packet_buffer_free_av1(xpkt);
    }

    av_log(avctx, AV_LOG_VERBOSE, "ff_xcoder_receive_packet2: return %d\n", ret);
    return ret;
}

// for FFmpeg 4.4+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
int ff_xcoder_receive_packet(AVCodecContext *avctx, AVPacket *pkt)
{
    XCoderEncContext *ctx = avctx->priv_data;
    AVFrame *frame = &ctx->buffered_fme;
    int ret;

    ret = ff_encode_get_frame(avctx, frame);
    if (!ctx->encoder_flushing && ret >= 0 || ret == AVERROR_EOF) {
        ret = ff_xcoder_send_frame(avctx, (ret == AVERROR_EOF ? NULL : frame));
        if (ret < 0 && ret != AVERROR_EOF) {
            av_frame_unref(frame);
            return ret;
        }
    }
    // Once send_frame returns EOF go on receiving packets until EOS is met.
    return ff_xcoder_receive_packet2(avctx, pkt);
}
#else
int ff_xcoder_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                       const AVFrame *frame, int *got_packet)
{
    XCoderEncContext *ctx = avctx->priv_data;
    int ret;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder encode frame\n");

    if (!ctx->encoder_flushing) {
        ret = ff_xcoder_send_frame(avctx, frame);
        if (ret < 0) {
            return ret;
        }
    }

    ret = ff_xcoder_receive_packet2(avctx, pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        *got_packet = 0;
    } else if (ret < 0) {
        return ret;
    } else {
        *got_packet = 1;
    }

    return 0;
}
#endif

static bool free_frames_isempty(XCoderEncContext *ctx)
{
    return (ctx->freeHead == ctx->freeTail);
}

static bool free_frames_isfull(XCoderEncContext *ctx)
{
    return (ctx->freeHead == ((ctx->freeTail == MAX_NUM_FRAMEPOOL_HWAVFRAME) ? 0 : ctx->freeTail + 1));
}

static int deq_free_frames(XCoderEncContext *ctx)
{
    if (free_frames_isempty(ctx)) {
        return -1;
    }
    ctx->aFree_Avframes_list[ctx->freeHead] = -1;
    ctx->freeHead = (ctx->freeHead == MAX_NUM_FRAMEPOOL_HWAVFRAME) ? 0 : ctx->freeHead + 1;
    return 0;
}

static int enq_free_frames(XCoderEncContext *ctx, int idx)
{
    if (free_frames_isfull(ctx)) {
        return -1;
    }
    ctx->aFree_Avframes_list[ctx->freeTail] = idx;
    ctx->freeTail = (ctx->freeTail == MAX_NUM_FRAMEPOOL_HWAVFRAME) ? 0 : ctx->freeTail + 1;
    return 0;
}

static int recycle_index_2_avframe_index(XCoderEncContext *ctx, uint32_t recycleIndex)
{
    int i;
    for (i = 0; i < MAX_NUM_FRAMEPOOL_HWAVFRAME; i++) {
        if (ctx->sframe_pool[i]->data[3] &&
            ((niFrameSurface1_t *)(ctx->sframe_pool[i]->data[3]))->ui16FrameIdx == recycleIndex) {
            return i;
        }
    }
    return -1;
}

// Needed for hwframe on FFmpeg-n4.3+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 82)
const AVCodecHWConfigInternal *ff_ni_enc_hw_configs[] = {
    HW_CONFIG_ENCODER_FRAMES(NI_QUAD,  NI_QUADRA),
    HW_CONFIG_ENCODER_DEVICE(NV12, NI_QUADRA),
    HW_CONFIG_ENCODER_DEVICE(P010LE, NI_QUADRA),
    HW_CONFIG_ENCODER_DEVICE(YUV420P, NI_QUADRA),
    HW_CONFIG_ENCODER_DEVICE(YUV420P10LE, NI_QUADRA),
    NULL,
};
#endif
