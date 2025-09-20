/*
 * NetInt XCoder H.264/HEVC Decoder common code
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

/**
 * @file
 * XCoder decoder.
 */

#include "nidec.h"
#include "fftools/ffmpeg.h"
#if ((LIBAVCODEC_VERSION_MAJOR > 61) || (LIBAVCODEC_VERSION_MAJOR == 61 && LIBAVCODEC_VERSION_MINOR >= 19))
#include "libavcodec/hevc/hevc.h"
#include "libavcodec/hevc/sei.h"
#else
#include "libavcodec/hevc.h"
#include "libavcodec/hevc_sei.h"
#endif
#include "libavutil/hdr_dynamic_metadata.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_ni_quad.h"
#include "libavutil/imgutils.h"
#include "libavutil/mastering_display_metadata.h"
#include "libavutil/mem.h"
#if (LIBAVCODEC_VERSION_MAJOR >= 61)
#include "fftools/ffmpeg_sched.h"
#endif
#include <ni_av_codec.h>

#define USER_DATA_UNREGISTERED_SEI_PAYLOAD_TYPE 5
#define NETINT_SKIP_PROFILE 0

static int xcoder_decode_reset(AVCodecContext *avctx);

static int xcoder_dec_close(AVCodecContext *avctx,
                            XCoderDecContext *s);

static int xcoder_dec_init(AVCodecContext *avctx,
                           XCoderDecContext *s);

static int xcoder_dec_send(AVCodecContext *avctx,
                           XCoderDecContext *s,
                           AVPacket *pkt);

static int xcoder_dec_receive(AVCodecContext *avctx,
                              XCoderDecContext *s,
                              AVFrame *frame,
                              bool wait);

static int xcoder_dec_is_flushing(AVCodecContext *avctx,
                                  XCoderDecContext *s);

static int xcoder_dec_flush(AVCodecContext *avctx,
                            XCoderDecContext *s);

static int parse_symbolic_decoder_param(XCoderDecContext *s);

static int retrieve_frame(AVCodecContext *avctx, AVFrame *data,
                          int *got_frame, ni_frame_t *xfme);

static int xcoder_add_headers(AVCodecContext *avctx, AVPacket *pkt,
                          uint8_t *extradata, int extradata_size);

int ff_xcoder_decode_close(AVCodecContext *avctx)
{
#if LIBAVCODEC_VERSION_MAJOR >= 60
    int i;
#endif
    XCoderDecContext *s = avctx->priv_data;
    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode close\n");

    /* this call shall release resource based on s->api_ctx */
    xcoder_dec_close(avctx, s);

    av_packet_unref(&s->buffered_pkt);
    av_packet_unref(&s->lone_sei_pkt);

    av_freep(&s->extradata);
    s->extradata_size      = 0;
    s->got_first_key_frame = 0;

#if LIBAVCODEC_VERSION_MAJOR >= 60
    if (s->opaque_data_array) {
        for (i = 0; i < s->opaque_data_nb; i++)
            av_buffer_unref(&s->opaque_data_array[i].opaque_ref);
        av_freep(&s->opaque_data_array);
    }
#endif

    ni_rsrc_free_device_context(s->rsrc_ctx);
    s->rsrc_ctx = NULL;
    return 0;
}

static int xcoder_setup_decoder(AVCodecContext *avctx)
{
    XCoderDecContext *s = avctx->priv_data;
    ni_xcoder_params_t *p_param =
        &s->api_param; // dec params in union with enc params struct
    int min_resolution_width, min_resolution_height;

    int ret = 0;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder setup device decoder\n");

    if (ni_device_session_context_init(&(s->api_ctx)) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "Error XCoder init decoder context failure\n");
        return AVERROR_EXTERNAL;
    }

    min_resolution_width  = NI_MIN_RESOLUTION_WIDTH;
    min_resolution_height = NI_MIN_RESOLUTION_HEIGHT;

    // Check codec id or format as well as profile idc.
    switch (avctx->codec_id) {
        case AV_CODEC_ID_HEVC:
            s->api_ctx.codec_format = NI_CODEC_FORMAT_H265;
            switch (avctx->profile) {
                case FF_PROFILE_HEVC_MAIN:
                case FF_PROFILE_HEVC_MAIN_10:
                case FF_PROFILE_HEVC_MAIN_STILL_PICTURE:
                case FF_PROFILE_UNKNOWN:
                    break;
                case NETINT_SKIP_PROFILE:
                    av_log(avctx, AV_LOG_WARNING, "Warning: HEVC profile %d not supported, skip setting it\n", avctx->profile);
                    break;
                default:
                    av_log(avctx, AV_LOG_ERROR, "Error: profile %d not supported.\n", avctx->profile);
                    return AVERROR_INVALIDDATA;
            }
            break;
        case AV_CODEC_ID_VP9:
            s->api_ctx.codec_format = NI_CODEC_FORMAT_VP9;
            switch (avctx->profile) {
                case FF_PROFILE_VP9_0:
                case FF_PROFILE_VP9_2:
                case FF_PROFILE_UNKNOWN:
                    break;
                default:
                    av_log(avctx, AV_LOG_ERROR, "Error: profile %d not supported.\n", avctx->profile);
                    return AVERROR_INVALIDDATA;
            }
            break;
        case AV_CODEC_ID_MJPEG:
            s->api_ctx.codec_format = NI_CODEC_FORMAT_JPEG;
            min_resolution_width    = NI_MIN_RESOLUTION_WIDTH_JPEG;
            min_resolution_height   = NI_MIN_RESOLUTION_HEIGHT_JPEG;
            switch (avctx->profile) {
                case FF_PROFILE_MJPEG_HUFFMAN_BASELINE_DCT:
                case FF_PROFILE_UNKNOWN:
                    break;
                default:
                    av_log(avctx, AV_LOG_ERROR, "Error: profile %d not supported.\n", avctx->profile);
                    return AVERROR_INVALIDDATA;
            }
            break;
        default:
            s->api_ctx.codec_format = NI_CODEC_FORMAT_H264;
            switch (avctx->profile) {
                case FF_PROFILE_H264_BASELINE:
                case FF_PROFILE_H264_CONSTRAINED_BASELINE:
                case FF_PROFILE_H264_MAIN:
                case FF_PROFILE_H264_EXTENDED:
                case FF_PROFILE_H264_HIGH:
                case FF_PROFILE_H264_HIGH_10:
                case FF_PROFILE_UNKNOWN:
                    break;
                case NETINT_SKIP_PROFILE:
                    av_log(avctx, AV_LOG_WARNING, "Warning: H264 profile %d not supported, skip setting it.\n", avctx->profile);
                    break;
                default:
                    av_log(avctx, AV_LOG_ERROR, "Error: profile %d not supported.\n", avctx->profile);
                    return AVERROR_INVALIDDATA;
            }
            break;
    }

    if (avctx->width > NI_MAX_RESOLUTION_WIDTH ||
        avctx->height > NI_MAX_RESOLUTION_HEIGHT ||
        avctx->width * avctx->height > NI_MAX_RESOLUTION_AREA) {
        av_log(avctx, AV_LOG_ERROR,
               "Error XCoder resolution %dx%d not supported\n", avctx->width,
               avctx->height);
        av_log(avctx, AV_LOG_ERROR, "Max Supported Width: %d Height %d Area %d\n",
               NI_MAX_RESOLUTION_WIDTH, NI_MAX_RESOLUTION_HEIGHT,
               NI_MAX_RESOLUTION_AREA);
        return AVERROR_EXTERNAL;
    } else if (avctx->width < min_resolution_width ||
               avctx->height < min_resolution_height) {
        av_log(avctx, AV_LOG_ERROR,
               "Error XCoder resolution %dx%d not supported\n", avctx->width,
               avctx->height);
        av_log(avctx, AV_LOG_ERROR, "Min Supported Width: %d Height %d\n",
               min_resolution_width, min_resolution_height);
        return AVERROR_EXTERNAL;
    }

    s->offset = 0LL;

    s->draining = 0;

    s->api_ctx.pic_reorder_delay = avctx->has_b_frames;
    s->api_ctx.bit_depth_factor = 1;
    if (AV_PIX_FMT_YUV420P10BE == avctx->pix_fmt ||
        AV_PIX_FMT_YUV420P10LE == avctx->pix_fmt ||
        AV_PIX_FMT_P010LE == avctx->pix_fmt) {
        s->api_ctx.bit_depth_factor = 2;
    }
    av_log(avctx, AV_LOG_VERBOSE, "xcoder_setup_decoder: pix_fmt %u bit_depth_factor %u\n", avctx->pix_fmt, s->api_ctx.bit_depth_factor);

    //Xcoder User Configuration
    if (ni_decoder_init_default_params(p_param, avctx->framerate.num, avctx->framerate.den, avctx->bit_rate, avctx->width, avctx->height) < 0) {
        av_log(avctx, AV_LOG_INFO, "Error setting params\n");
        return AVERROR(EINVAL);
    }

    if (s->xcoder_opts) {
        AVDictionary *dict = NULL;
        AVDictionaryEntry *en = NULL;

        if (av_dict_parse_string(&dict, s->xcoder_opts, "=", ":", 0)) {
            av_log(avctx, AV_LOG_ERROR, "Xcoder options provided contain error(s)\n");
            av_dict_free(&dict);
            return AVERROR_EXTERNAL;
        } else {
            while ((en = av_dict_get(dict, "", en, AV_DICT_IGNORE_SUFFIX))) {
                int parse_ret = ni_decoder_params_set_value(p_param, en->key, en->value);
                if (parse_ret != NI_RETCODE_SUCCESS) {
                    switch (parse_ret) {
                        case NI_RETCODE_PARAM_INVALID_NAME:
                            av_log(avctx, AV_LOG_ERROR, "Unknown option: %s.\n", en->key);
                            av_dict_free(&dict);
                            return AVERROR_EXTERNAL;
                        case NI_RETCODE_PARAM_ERROR_TOO_BIG:
                            av_log(avctx, AV_LOG_ERROR,
                                   "Invalid %s: too big, max char len = %d\n", en->key,
                                   NI_MAX_PPU_PARAM_EXPR_CHAR);
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

        for (size_t i = 0; i < NI_MAX_NUM_OF_DECODER_OUTPUTS; i++) {
            if (p_param->dec_input_params.crop_mode[i] != NI_DEC_CROP_MODE_AUTO) {
                continue;
            }
            for (size_t j = 0; j < 4; j++) {
                if (strlen(p_param->dec_input_params.cr_expr[i][j])) {
                    av_log(avctx, AV_LOG_ERROR, "Setting crop parameters without setting crop mode to manual?\n");
                    return AVERROR_EXTERNAL;
                }
            }
        }
    }
    parse_symbolic_decoder_param(s);
    return 0;
}

int ff_xcoder_decode_init(AVCodecContext *avctx)
{
#if LIBAVCODEC_VERSION_MAJOR >= 60
    int i;
#endif
    int ret = 0;
    XCoderDecContext *s = avctx->priv_data;
    const AVPixFmtDescriptor *desc;
    ni_xcoder_params_t *p_param = &s->api_param;
    uint32_t xcoder_timeout;

    ni_log_set_level(ff_to_ni_log_level(av_log_get_level()));

    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode init\n");

    avctx->sw_pix_fmt = avctx->pix_fmt;

    desc = av_pix_fmt_desc_get(avctx->sw_pix_fmt);
    av_log(avctx, AV_LOG_VERBOSE, "width: %d height: %d sw_pix_fmt: %s\n",
           avctx->width, avctx->height, desc ? desc->name : "NONE");

    if (0 == avctx->width || 0 == avctx->height) {
        av_log(avctx, AV_LOG_ERROR, "Error probing input stream\n");
        return AVERROR_INVALIDDATA;
    }

    switch (avctx->pix_fmt) {
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUV420P10BE:
        case AV_PIX_FMT_YUV420P10LE:
        case AV_PIX_FMT_YUVJ420P:
        case AV_PIX_FMT_GRAY8:
            break;
        case AV_PIX_FMT_NONE:
            av_log(avctx, AV_LOG_WARNING, "Warning: pixel format is not specified\n");
            break;
        default:
            av_log(avctx, AV_LOG_ERROR, "Error: pixel format %s not supported.\n",
                   desc ? desc->name : "NONE");
            return AVERROR_INVALIDDATA;
    }

    av_log(avctx, AV_LOG_VERBOSE, "(avctx->field_order = %d)\n", avctx->field_order);
    if (avctx->field_order > AV_FIELD_PROGRESSIVE) { //AVFieldOrder with bottom or top coding order represents interlaced video
        av_log(avctx, AV_LOG_ERROR, "interlaced video not supported!\n");
        return AVERROR_INVALIDDATA;
    }

    if ((ret = xcoder_setup_decoder(avctx)) < 0) {
        return ret;
    }

    //--------reassign pix format based on user param------------//
    if (p_param->dec_input_params.semi_planar[0]) {
        if (avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10BE ||
            avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10LE ||
            avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P) {
            av_log(avctx, AV_LOG_VERBOSE, "XCoder decode init: YV12 forced to NV12\n");
            avctx->sw_pix_fmt = (avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P) ? AV_PIX_FMT_NV12 : AV_PIX_FMT_P010LE;
        }
    }
    if (p_param->dec_input_params.force_8_bit[0]) {
        if (avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10BE ||
            avctx->sw_pix_fmt == AV_PIX_FMT_YUV420P10LE ||
            avctx->sw_pix_fmt == AV_PIX_FMT_P010LE) {
            av_log(avctx, AV_LOG_VERBOSE, "XCoder decode init: 10Bit input forced to 8bit\n");
            avctx->sw_pix_fmt = (avctx->sw_pix_fmt == AV_PIX_FMT_P010LE) ? AV_PIX_FMT_NV12 : AV_PIX_FMT_YUV420P;
            s->api_ctx.bit_depth_factor = 1;
        }
    }
    if (p_param->dec_input_params.hwframes) { //need to set before open decoder
        s->api_ctx.hw_action = NI_CODEC_HW_ENABLE;
    } else {
        s->api_ctx.hw_action = NI_CODEC_HW_NONE;
    }

#if ((LIBAVCODEC_VERSION_MAJOR > 61) || (LIBAVCODEC_VERSION_MAJOR == 61 && LIBAVCODEC_VERSION_MINOR >= 19))
    if (p_param->dec_input_params.hwframes && p_param->dec_input_params.max_extra_hwframe_cnt == 255)
        p_param->dec_input_params.max_extra_hwframe_cnt = 0;
    if (p_param->dec_input_params.hwframes && (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1))
        p_param->dec_input_params.hwframes |= DEFAULT_FRAME_THREAD_QUEUE_SIZE << 4;
#endif
    //------reassign pix format based on user param done--------//

    if (s->custom_sei_type == USER_DATA_UNREGISTERED_SEI_PAYLOAD_TYPE ||
        p_param->dec_input_params.custom_sei_passthru == USER_DATA_UNREGISTERED_SEI_PAYLOAD_TYPE) {
        // use SW passthru only
        s->api_ctx.enable_user_data_sei_passthru = 0;
        p_param->dec_input_params.enable_user_data_sei_passthru = 0;
    } else {
        s->api_ctx.enable_user_data_sei_passthru = s->enable_user_data_sei_passthru;
    }

    av_log(avctx, AV_LOG_VERBOSE, "api_ctx %p api_ctx/s: user_data_sei_passthru = %d/%d, custom_sei_type = %d\n",
           &s->api_ctx, s->api_ctx.enable_user_data_sei_passthru, s->enable_user_data_sei_passthru, s->custom_sei_type);

#if LIBAVCODEC_VERSION_MAJOR < 62
    if (avctx->codec_id == AV_CODEC_ID_H264) {
        // reference h264_decode_init in h264dec.c
        if (avctx->ticks_per_frame == 1) {
            if (avctx->time_base.den < INT_MAX / 2) {
                avctx->time_base.den *= 2;
            } else
                avctx->time_base.num /= 2;
        }
        avctx->ticks_per_frame = 2;
    }
#endif

    s->started = 0;
    memset(&s->api_pkt, 0, sizeof(ni_packet_t));
    s->pkt_nal_bitmap = 0;
    s->svct_skip_next_packet = 0;

#if LIBAVCODEC_VERSION_MAJOR < 62
    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode init: time_base = %d/%d, frame rate = %d/%d, ticks_per_frame=%d\n", avctx->time_base.num, avctx->time_base.den, avctx->framerate.num, avctx->framerate.den, avctx->ticks_per_frame);
#else
    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode init: time_base = %d/%d, frame rate = %d/%d\n", avctx->time_base.num, avctx->time_base.den, avctx->framerate.num, avctx->framerate.den);
#endif
    // overwrite keep alive timeout value here with a custom value if it was
    // provided
    // if xcoder option is set then overwrite the (legacy) decoder option
    xcoder_timeout = s->api_param.dec_input_params.keep_alive_timeout;
    if (xcoder_timeout != NI_DEFAULT_KEEP_ALIVE_TIMEOUT) {
        s->api_ctx.keep_alive_timeout = xcoder_timeout;
    } else {
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
    }
    av_log(avctx, AV_LOG_VERBOSE, "Custom NVME Keep Alive Timeout set to %d\n",
           s->api_ctx.keep_alive_timeout);

    if (s->api_param.dec_input_params.decoder_low_delay != 0) {
        s->low_delay = s->api_param.dec_input_params.decoder_low_delay;
    } else {
        s->api_param.dec_input_params.decoder_low_delay = s->low_delay;
    }
    s->api_ctx.enable_low_delay_check = s->api_param.dec_input_params.enable_low_delay_check;
    if (avctx->has_b_frames && s->api_ctx.enable_low_delay_check) {
        // If has B frame, must set lowdelay to 0
        av_log(avctx, AV_LOG_WARNING,"Warning: decoder lowDelay mode "
               "is cancelled due to has_b_frames with enable_low_delay_check\n");
        s->low_delay = s->api_param.dec_input_params.decoder_low_delay = 0;
    }
    s->api_ctx.decoder_low_delay = s->low_delay;

    s->api_ctx.p_session_config = &s->api_param;

    if ((ret = xcoder_dec_init(avctx, s)) < 0) {
        goto done;
    }

    s->current_pts = NI_NOPTS_VALUE;

#if LIBAVCODEC_VERSION_MAJOR >= 60
    /* The size opaque pointers buffer is chosen by max buffered packets in FW (4) +
     * max output buffer in FW (24) + some extra room to be safe. If the delay of any
     * frame is larger than this, we assume that the frame is dropped so the buffered
     * opaque pointer can be overwritten when the opaque_data_array wraps around */
    s->opaque_data_nb = 30;
    s->opaque_data_pos = 0;
    if (!s->opaque_data_array) {
        s->opaque_data_array = av_calloc(s->opaque_data_nb, sizeof(OpaqueData));
        if (!s->opaque_data_array) {
            ret = AVERROR(ENOMEM);
            goto done;
        }
    }
    for (i = 0; i < s->opaque_data_nb; i++) {
        s->opaque_data_array[i].pkt_pos = -1;
    }
#endif

done:
    return ret;
}

// reset and restart when xcoder decoder resets
static int xcoder_decode_reset(AVCodecContext *avctx)
{
    XCoderDecContext *s = avctx->priv_data;
    int ret = NI_RETCODE_FAILURE;
    int64_t bcp_current_pts;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode reset\n");

    ni_device_session_close(&s->api_ctx, s->eos, NI_DEVICE_TYPE_DECODER);

    ni_device_session_context_clear(&s->api_ctx);

#ifdef _WIN32
    ni_device_close(s->api_ctx.device_handle);
#elif __linux__
    ni_device_close(s->api_ctx.device_handle);
    ni_device_close(s->api_ctx.blk_io_handle);
#endif
    s->api_ctx.device_handle = NI_INVALID_DEVICE_HANDLE;
    s->api_ctx.blk_io_handle = NI_INVALID_DEVICE_HANDLE;

    ni_packet_buffer_free(&(s->api_pkt.data.packet));
    bcp_current_pts = s->current_pts;
    ret = ff_xcoder_decode_init(avctx);
    s->current_pts = bcp_current_pts;
    s->api_ctx.session_run_state = SESSION_RUN_STATE_RESETTING;
    return ret;
}

static int xcoder_send_receive(AVCodecContext *avctx, XCoderDecContext *s,
                               AVFrame *frame, bool wait)
{
    int ret;

    /* send any pending data from buffered packet */
    while (s->buffered_pkt.size) {
        ret = xcoder_dec_send(avctx, s, &s->buffered_pkt);
        if (ret == AVERROR(EAGAIN))
            break;
        else if (ret < 0) {
            av_packet_unref(&s->buffered_pkt);
            return ret;
        }
        av_packet_unref(&s->buffered_pkt);
    }

    /* check for new frame */
    return xcoder_dec_receive(avctx, s, frame, wait);
}

int ff_xcoder_receive_frame(AVCodecContext *avctx, AVFrame *frame)
{
    XCoderDecContext *s = avctx->priv_data;
    const AVPixFmtDescriptor *desc;
    int ret;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder receive frame\n");
    /*
     * After we have buffered an input packet, check if the codec is in the
     * flushing state. If it is, we need to call xcoder_dec_flush.
     *
     * xcoder_dec_flush returns 0 if the flush cannot be performed on
     * the codec (because the user retains frames). The codec stays in the
     * flushing state.
     * For now we don't consider this case of user retaining the frame
     * (connected decoder-encoder case), so the return can only be 1
     * (flushed successfully), or < 0 (failure)
     *
     * xcoder_dec_flush returns 1 if the flush can actually be
     * performed on the codec. The codec leaves the flushing state and can
     * process again packets.
     *
     * xcoder_dec_flush returns a negative value if an error has
     * occurred.
     */
    if (xcoder_dec_is_flushing(avctx, s)) {
        if (!xcoder_dec_flush(avctx, s)) {
            return AVERROR(EAGAIN);
        }
    }

    // give priority to sending data to decoder
    if (s->buffered_pkt.size == 0) {
        ret = ff_decode_get_packet(avctx, &s->buffered_pkt);
        if (ret < 0) {
            av_log(avctx, AV_LOG_VERBOSE, "ff_decode_get_packet 1 rc: %s\n",
                   av_err2str(ret));
        } else {
            av_log(avctx, AV_LOG_DEBUG, "ff_decode_get_packet 1 rc: Success\n");
        }
    }

    /* flush buffered packet and check for new frame */
    ret = xcoder_send_receive(avctx, s, frame, false);
    if (NI_RETCODE_ERROR_VPU_RECOVERY == ret) {
        ret = xcoder_decode_reset(avctx);
        if (0 == ret) {
            return AVERROR(EAGAIN);
        } else {
            return ret;
        }
    } else if (ret != AVERROR(EAGAIN)) {
        return ret;
    }

    /* skip fetching new packet if we still have one buffered */
    if (s->buffered_pkt.size > 0) {
        return xcoder_send_receive(avctx, s, frame, true);
    }

    /* fetch new packet or eof */
    ret = ff_decode_get_packet(avctx, &s->buffered_pkt);
    if (ret < 0) {
        av_log(avctx, AV_LOG_VERBOSE, "ff_decode_get_packet 2 rc: %s\n",
               av_err2str(ret));
    } else {
        av_log(avctx, AV_LOG_DEBUG, "ff_decode_get_packet 2 rc: Success\n");
    }

    if (ret == AVERROR_EOF) {
        AVPacket null_pkt = {0};
        ret = xcoder_dec_send(avctx, s, &null_pkt);
        if (ret < 0) {
            return ret;
        }
    } else if (ret < 0) {
        return ret;
    } else {
        av_log(avctx, AV_LOG_VERBOSE, "width: %d  height: %d\n", avctx->width, avctx->height);
        desc = av_pix_fmt_desc_get(avctx->pix_fmt);
        av_log(avctx, AV_LOG_VERBOSE, "pix_fmt: %s\n", desc ? desc->name : "NONE");
    }

    /* crank decoder with new packet */
    return xcoder_send_receive(avctx, s, frame, true);
}

void ff_xcoder_decode_flush(AVCodecContext *avctx)
{
    XCoderDecContext *s = avctx->priv_data;
    ni_device_dec_session_flush(&s->api_ctx);
    s->draining = 0;
    s->flushing = 0;
    s->eos = 0;
}

static const char *const var_names[] = {
    "in_w", "iw",   ///< width  of the input video
    "in_h", "ih",   ///< height of the input video
    "out_w", "ow",  ///< width  of the cropped video
    "out_h", "oh",  ///< height of the cropped video
    "x",
    "y",
    NULL
};

enum var_name {
    VAR_IN_W, VAR_IW,
    VAR_IN_H, VAR_IH,
    VAR_OUT_W, VAR_OW,
    VAR_OUT_H, VAR_OH,
    VAR_X,
    VAR_Y,
    VAR_VARS_NB
};

static inline void ni_align_free(void *opaque, uint8_t *data)
{
    ni_buf_t *buf = (ni_buf_t *)opaque;
    if (buf) {
        ni_decoder_frame_buffer_pool_return_buf(buf, (ni_buf_pool_t *)buf->pool);
    }
}

static inline void ni_frame_free(void *opaque, uint8_t *data)
{
    if (data) {
        int ret;
        int num_buffers = opaque ? *((int*)opaque) : 1;
        for (int i = 0; i < num_buffers; i++) {
            niFrameSurface1_t* p_data3 = (niFrameSurface1_t*)(data + i * sizeof(niFrameSurface1_t));
            if (p_data3->ui16FrameIdx != 0) {
                av_log(NULL, AV_LOG_DEBUG, "Recycle trace ui16FrameIdx = [%d] DevHandle %d\n", p_data3->ui16FrameIdx, p_data3->device_handle);
                ret = ni_hwframe_buffer_recycle(p_data3, p_data3->device_handle);
                if (ret != NI_RETCODE_SUCCESS) {
                    av_log(NULL, AV_LOG_ERROR, "ERROR Failed to recycle trace ui16frameidx = [%d] DevHandle %d\n", p_data3->ui16FrameIdx, p_data3->device_handle);
                }
            }
        }
        ni_aligned_free(data);
        (void)data;  // suppress cppcheck
    }
}

static inline void __ni_free(void *opaque, uint8_t *data)
{
    free(data); // Free data allocated by libxcoder
}

static enum AVPixelFormat ni_supported_pixel_formats[] =
{
    AV_PIX_FMT_YUV420P, //0
    AV_PIX_FMT_YUV420P10LE,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_P010LE,
    AV_PIX_FMT_NONE, //convert RGB to unused
    AV_PIX_FMT_NONE,
    AV_PIX_FMT_NONE,
    AV_PIX_FMT_NONE,
    AV_PIX_FMT_NONE, //8
    AV_PIX_FMT_NONE,
    AV_PIX_FMT_NONE,
    AV_PIX_FMT_NONE,
    AV_PIX_FMT_NONE, //12
    AV_PIX_FMT_NI_QUAD_8_TILE_4X4,
    AV_PIX_FMT_NI_QUAD_10_TILE_4X4,
    AV_PIX_FMT_NONE, //15
};

static inline int ni_pix_fmt_2_ff_pix_fmt(ni_pix_fmt_t pix_fmt)
{
    return ni_supported_pixel_formats[pix_fmt];
}

static int parse_symbolic_decoder_param(XCoderDecContext *s)
{
    ni_decoder_input_params_t *pdec_param = &s->api_param.dec_input_params;
    int i, ret;
    double res;
    double var_values[VAR_VARS_NB];

    if (pdec_param == NULL) {
        return AVERROR_INVALIDDATA;
    }

    for (i = 0; i < NI_MAX_NUM_OF_DECODER_OUTPUTS; i++) {
        /*Set output width and height*/
        var_values[VAR_IN_W] = var_values[VAR_IW] = pdec_param->crop_whxy[i][0];
        var_values[VAR_IN_H] = var_values[VAR_IH] = pdec_param->crop_whxy[i][1];
        var_values[VAR_OUT_W] = var_values[VAR_OW] = pdec_param->crop_whxy[i][0];
        var_values[VAR_OUT_H] = var_values[VAR_OH] = pdec_param->crop_whxy[i][1];
        if (pdec_param->cr_expr[i][0][0] && pdec_param->cr_expr[i][1][0]) {
            if (av_expr_parse_and_eval(&res, pdec_param->cr_expr[i][0], var_names,
                                     var_values, NULL, NULL, NULL, NULL, NULL, 0,
                                     s) < 0) {
                return AVERROR_INVALIDDATA;
            }
            var_values[VAR_OUT_W] = var_values[VAR_OW] = (double)floor(res);
            if (av_expr_parse_and_eval(&res, pdec_param->cr_expr[i][1], var_names,
                                     var_values, NULL, NULL, NULL, NULL, NULL, 0,
                                     s) < 0) {
                return AVERROR_INVALIDDATA;
            }
            var_values[VAR_OUT_H] = var_values[VAR_OH] = (double)floor(res);
            /* evaluate again ow as it may depend on oh */
            ret = av_expr_parse_and_eval(&res, pdec_param->cr_expr[i][0], var_names,
                                       var_values, NULL, NULL, NULL, NULL, NULL,
                                       0, s);
            if (ret < 0) {
                return AVERROR_INVALIDDATA;
            }
            var_values[VAR_OUT_W] = var_values[VAR_OW] = (double)floor(res);
            pdec_param->crop_whxy[i][0] = (int)var_values[VAR_OUT_W];
            pdec_param->crop_whxy[i][1] = (int)var_values[VAR_OUT_H];
        }
        /*Set output crop offset X,Y*/
        if (pdec_param->cr_expr[i][2][0]) {
            ret = av_expr_parse_and_eval(&res, pdec_param->cr_expr[i][2], var_names,
                                       var_values, NULL, NULL, NULL, NULL, NULL,
                                       0, s);
            if (ret < 0) {
                return AVERROR_INVALIDDATA;
            }
            var_values[VAR_X] = res;
            pdec_param->crop_whxy[i][2] = floor(var_values[VAR_X]);
        }
        if (pdec_param->cr_expr[i][3][0]) {
            ret = av_expr_parse_and_eval(&res, pdec_param->cr_expr[i][3], var_names,
                                       var_values, NULL, NULL, NULL, NULL, NULL,
                                       0, s);
            if (ret < 0) {
                return AVERROR_INVALIDDATA;
            }
            var_values[VAR_Y] = res;
            pdec_param->crop_whxy[i][3] = floor(var_values[VAR_Y]);
        }
        /*Set output Scale*/
        /*Reset OW and OH to next lower even number*/
        var_values[VAR_OUT_W] = var_values[VAR_OW] =
            (double)(pdec_param->crop_whxy[i][0] -
                    (pdec_param->crop_whxy[i][0] % 2));
        var_values[VAR_OUT_H] = var_values[VAR_OH] =
            (double)(pdec_param->crop_whxy[i][1] -
                    (pdec_param->crop_whxy[i][1] % 2));
        if (pdec_param->sc_expr[i][0][0] && pdec_param->sc_expr[i][1][0]) {
            if (av_expr_parse_and_eval(&res, pdec_param->sc_expr[i][0], var_names,
                                     var_values, NULL, NULL, NULL, NULL, NULL, 0,
                                     s) < 0) {
                return AVERROR_INVALIDDATA;
            }
            pdec_param->scale_wh[i][0] = ceil(res);
            ret = av_expr_parse_and_eval(&res, pdec_param->sc_expr[i][1], var_names,
                                       var_values, NULL, NULL, NULL, NULL, NULL,
                                       0, s);
            if (ret < 0) {
                return AVERROR_INVALIDDATA;
            }
            pdec_param->scale_wh[i][1] = ceil(res);
        }
    }
    return 0;
}

static int xcoder_dec_init(AVCodecContext *avctx, XCoderDecContext *s)
{
    int ret = 0;
    ni_xcoder_params_t *p_param = &s->api_param;

    s->api_ctx.hw_id = s->dev_dec_idx;
    s->api_ctx.decoder_low_delay = 0;
    ff_xcoder_strncpy(s->api_ctx.blk_dev_name, s->dev_blk_name,
                      NI_MAX_DEVICE_NAME_LEN);
    ff_xcoder_strncpy(s->api_ctx.dev_xcoder_name, s->dev_xcoder,
                      MAX_CHAR_IN_DEVICE_NAME);

    ret = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_DECODER);
    if (ret != 0) {
        av_log(avctx, AV_LOG_ERROR, "Failed to open decoder (status = %d), "
               "resource unavailable\n", ret);
        ret = AVERROR_EXTERNAL;
        xcoder_dec_close(avctx, s);
    } else {
        s->dev_xcoder_name = s->api_ctx.dev_xcoder_name;
        s->blk_xcoder_name = s->api_ctx.blk_xcoder_name;
        s->dev_dec_idx = s->api_ctx.hw_id;
        av_log(avctx, AV_LOG_VERBOSE,
               "XCoder %s.%d (inst: %d) opened successfully\n",
               s->dev_xcoder_name, s->dev_dec_idx, s->api_ctx.session_id);

        if (p_param->dec_input_params.hwframes) {
            if (!avctx->hw_device_ctx) {
                char buf[64] = {0};
                av_log(avctx, AV_LOG_DEBUG,
                       "nidec.c:xcoder_dec_init() hwdevice_ctx_create\n");
                snprintf(buf, sizeof(buf), "%d", s->dev_dec_idx);
                ret = av_hwdevice_ctx_create(&avctx->hw_device_ctx, AV_HWDEVICE_TYPE_NI_QUADRA,
                                           buf, NULL, 0); // create with null device
                if (ret < 0) {
                    av_log(NULL, AV_LOG_ERROR, "Error creating a NI HW device\n");
                    return ret;
                }
            }
            if (!avctx->hw_frames_ctx) {
                avctx->hw_frames_ctx = av_hwframe_ctx_alloc(avctx->hw_device_ctx);

                if (!avctx->hw_frames_ctx) {
                    ret = AVERROR(ENOMEM);
                    return ret;
                }
            }
            s->frames = (AVHWFramesContext *)avctx->hw_frames_ctx->data;

            s->frames->format = AV_PIX_FMT_NI_QUAD;
            s->frames->width = avctx->width;
            s->frames->height = avctx->height;

            s->frames->sw_format = avctx->sw_pix_fmt;
            // Decoder has its own dedicated pool
            s->frames->initial_pool_size = -1;

            ret = av_hwframe_ctx_init(avctx->hw_frames_ctx);

            avctx->pix_fmt = AV_PIX_FMT_NI_QUAD;
            s->api_ctx.hw_action = NI_CODEC_HW_ENABLE;
        } else {
            // reassign in case above conditions alter value
            avctx->pix_fmt = avctx->sw_pix_fmt;
            s->api_ctx.hw_action = NI_CODEC_HW_NONE;
        }
    }

    return ret;
}

static int xcoder_dec_close(AVCodecContext *avctx, XCoderDecContext *s)
{
    ni_session_context_t *p_ctx = &s->api_ctx;

    if (p_ctx) {
        // dec params in union with enc params struct
        ni_retcode_t ret;
        ni_xcoder_params_t *p_param = &s->api_param;
        int suspended = 0;

        ret = ni_device_session_close(p_ctx, s->eos, NI_DEVICE_TYPE_DECODER);
        if (NI_RETCODE_SUCCESS != ret) {
            av_log(avctx, AV_LOG_ERROR,
                   "Failed to close Decode Session (status = %d)\n", ret);
        }
        ni_device_session_context_clear(p_ctx);

        if (p_param->dec_input_params.hwframes) {
            av_log(avctx, AV_LOG_VERBOSE,
                   "File BLK handle %d close suspended to frames Uninit\n",
                   p_ctx->blk_io_handle); // suspended_device_handle
            if (avctx->hw_frames_ctx) {
                AVHWFramesContext *ctx =
                    (AVHWFramesContext *)avctx->hw_frames_ctx->data;
                if (ctx) {
                    AVNIFramesContext *dst_ctx = (AVNIFramesContext*) ctx->hwctx;
                    if (dst_ctx) {
                        dst_ctx->suspended_device_handle = p_ctx->blk_io_handle;
                        suspended = 1;
                    }
                }
            }
        }

        if (suspended) {
#ifdef __linux__
            ni_device_close(p_ctx->device_handle);
#endif
        } else {
#ifdef _WIN32
            ni_device_close(p_ctx->device_handle);
#elif __linux__
            ni_device_close(p_ctx->device_handle);
            ni_device_close(p_ctx->blk_io_handle);
#endif
        }
        p_ctx->device_handle = NI_INVALID_DEVICE_HANDLE;
        p_ctx->blk_io_handle = NI_INVALID_DEVICE_HANDLE;
        ni_packet_t *xpkt = &(s->api_pkt.data.packet);
        ni_packet_buffer_free(xpkt);
    }

    if (s && s->timecode_passthru) {
        ff_h264_ps_uninit(&s->ps);
    }

    return 0;
}

// return 1 if need to prepend saved header to pkt data, 0 otherwise
static int xcoder_add_headers(AVCodecContext *avctx, AVPacket *pkt,
                              uint8_t *extradata, int extradata_size)
{
    XCoderDecContext *s = avctx->priv_data;
    int ret = 0;
    int vps_num, sps_num, pps_num;

    // check key frame packet only
    if (!(pkt->flags & AV_PKT_FLAG_KEY) || !pkt->data || !extradata ||
        !extradata_size) {
        return ret;
    }

    if (s->extradata_size == extradata_size &&
        memcmp(s->extradata, extradata, extradata_size) == 0) {
        av_log(avctx, AV_LOG_TRACE, "%s extradata unchanged.\n", __FUNCTION__);
        return ret;
    }

    if (AV_CODEC_ID_H264 != avctx->codec_id &&
        AV_CODEC_ID_HEVC != avctx->codec_id) {
        av_log(avctx, AV_LOG_DEBUG, "%s not AVC/HEVC codec: %d, skip!\n",
              __FUNCTION__, avctx->codec_id);
        return ret;
    }

    // extradata (headers) non-existing or changed: save/update it in the
    // session storage
    av_freep(&s->extradata);
    s->extradata_size      = 0;
    s->got_first_key_frame = 0;
    s->extradata           = av_malloc(extradata_size);
    if (!s->extradata) {
        av_log(avctx, AV_LOG_ERROR, "%s memory allocation failed !\n",
               __FUNCTION__);
        return ret;
    }

    memcpy(s->extradata, extradata, extradata_size);
    s->extradata_size = extradata_size;
    // prepend header by default (assuming no header found in the pkt itself)
    ret = 1;
    // and we've got the first key frame of this stream
    s->got_first_key_frame = 1;
    vps_num = sps_num = pps_num = 0;

    if (s->api_param.dec_input_params.skip_extra_headers &&
        (s->extradata_size > 0) &&
        s->extradata) {
        const uint8_t *ptr = s->extradata;
        const uint8_t *end = s->extradata + s->extradata_size;
        uint32_t stc;
        uint8_t nalu_type;

        while (ptr < end) {
            stc = -1;
            ptr = avpriv_find_start_code(ptr, end, &stc);
            if (ptr == end) {
                break;
            }

            if (AV_CODEC_ID_H264 == avctx->codec_id) {
                nalu_type = stc & 0x1f;

                if (H264_NAL_SPS == nalu_type) {
                    sps_num++;
                } else if(H264_NAL_PPS == nalu_type) {
                    pps_num++;
                }

                if (sps_num > H264_MAX_SPS_COUNT ||
                    pps_num > H264_MAX_PPS_COUNT) {
                    ret = 0;
                    av_log(avctx, AV_LOG_WARNING, "Drop extradata because of repeated SPS/PPS\n");
                    break;
                }
            } else if (AV_CODEC_ID_HEVC == avctx->codec_id) {
                nalu_type = (stc >> 1) & 0x3F;

                if (HEVC_NAL_VPS == nalu_type) {
                    vps_num++;
                } else if (HEVC_NAL_SPS == nalu_type) {
                    sps_num++;
                } else if (HEVC_NAL_PPS == nalu_type) {
                    pps_num++;
                }

                if (vps_num > HEVC_MAX_VPS_COUNT ||
                    sps_num > HEVC_MAX_SPS_COUNT ||
                    pps_num > HEVC_MAX_PPS_COUNT) {
                    ret = 0;
                    av_log(avctx, AV_LOG_WARNING, "Drop extradata because of repeated VPS/SPS/PPS\n");
                    break;
                }
            }
        }
    }

    return ret;
}
static int xcoder_dec_send(AVCodecContext *avctx, XCoderDecContext *s, AVPacket *pkt)
{
    /* call ni_decoder_session_write to send compressed video packet to the decoder
       instance */
    int need_draining = 0;
    size_t size;
    ni_packet_t *xpkt = &(s->api_pkt.data.packet);
    int ret;
    int sent;
    int send_size = 0;
    int new_packet = 0;
    int extra_prev_size = 0;
    int svct_skip_packet = s->svct_skip_next_packet;
#if LIBAVCODEC_VERSION_MAJOR >= 60
    OpaqueData *opaque_data;
#endif

    size = pkt->size;

    if (s->flushing) {
        av_log(avctx, AV_LOG_ERROR, "Decoder is flushing and cannot accept new "
               "buffer until all output buffers have been released\n");
        return AVERROR_EXTERNAL;
    }

    if (pkt->size == 0) {
        need_draining = 1;
    }

    if (s->draining && s->eos) {
        av_log(avctx, AV_LOG_VERBOSE, "Decoder is draining, eos\n");
        return AVERROR_EOF;
    }

    if (xpkt->data_len == 0) {
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 91)
        AVBSFContext *bsf = avctx->internal->bsf;
#else
        AVBSFContext *bsf = avctx->internal->filter.bsfs[0];
#endif
        uint8_t *extradata = bsf ? bsf->par_out->extradata : avctx->extradata;
        int extradata_size = bsf ? bsf->par_out->extradata_size : avctx->extradata_size;

        memset(xpkt, 0, sizeof(ni_packet_t));
        xpkt->pts = pkt->pts;
        xpkt->dts = pkt->dts;
        xpkt->flags = pkt->flags;
        xpkt->video_width = avctx->width;
        xpkt->video_height = avctx->height;
        xpkt->p_data = NULL;
        xpkt->data_len = pkt->size;
        xpkt->pkt_pos = pkt->pos;

        uint8_t* p_side_data;
#if LIBAVCODEC_VERSION_MAJOR >= 59
        size_t side_size = 0;
#else
        int side_size = 0;
#endif
        p_side_data = av_packet_get_side_data(pkt, AV_PKT_DATA_PPU_CONFIG, &side_size);
        if (p_side_data && side_size == sizeof(AVNIPpuConfig)) {
            AVNIPpuConfig *av_ppu_config = (AVNIPpuConfig *)p_side_data;
            ni_ppu_config_t ppu_config = {0};
            for (int idx = 0; idx < NI_MAX_NUM_OF_DECODER_OUTPUTS; idx++) {
                if (av_ppu_config->out_enabled[idx] == 1) {
                    ppu_config.ppu_set_enable += (0x01 << idx);
                    ppu_config.ppu_w[idx] = av_ppu_config->ppu_w[idx];
                    ppu_config.ppu_h[idx] = av_ppu_config->ppu_h[idx];
                }
            }
#if ((LIBXCODER_API_VERSION_MAJOR > 2) ||                                        \
      (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 77))
            if (av_ppu_config->ppu_reconfig_mode == 1)
            {
                ret = ni_dec_reconfig_ppu_params(&(s->api_ctx), &(s->api_param), &ppu_config);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "%s(): ni_dec_reconfig_ppu_params failed\n", __FUNCTION__);
                    goto fail;
                }
            }
            else
#endif
            {
                ret = ni_reconfig_ppu_output(&(s->api_ctx), &(s->api_param), &ppu_config);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "%s(): ni_reconfig_ppu_output failed\n", __FUNCTION__);
                    goto fail;
                }
            }
        }

        if (pkt->flags & AV_PKT_FLAG_KEY) {
            int ext_headers = 0;
            if (extradata_size > 0) {
                ext_headers = xcoder_add_headers(avctx, pkt, extradata, extradata_size);
                if (ext_headers) {
                    if (extradata_size > s->api_ctx.max_nvme_io_size * 2) {
                        av_log(avctx, AV_LOG_ERROR,
                            "xcoder_dec_send extradata_size %d "
                            "exceeding max size supported: %d\n",
                            extradata_size, s->api_ctx.max_nvme_io_size * 2);
                    } else {
                        av_log(avctx, AV_LOG_VERBOSE,
                            "xcoder_dec_send extradata_size %d "
                            "copied to pkt start.\n",
                            s->extradata_size);

                        s->api_ctx.prev_size = s->extradata_size;
                        memcpy(s->api_ctx.p_leftover, s->extradata, s->extradata_size);
                    }
                }
            }

            if (s->timecode_passthru && avctx->codec_id == AV_CODEC_ID_H264) {
                const uint8_t *data;
                const uint8_t *end;
                const uint8_t *next = NULL;
                uint32_t state;
                GetBitContext gb;
                if (ext_headers) {
                    data = extradata;
                    end = data + extradata_size;
                } else {
                    data = pkt->data;
                    end = data + pkt->size;
                }
                while (data < end) {
                    data = next ? next : avpriv_find_start_code(data, end, &state);
                    if (data == end) {
                        break;
                    }
                    next = NULL;
                    if ((state & 0x1f) == H264_NAL_SPS) {
                        uint8_t sps_data[4096] = {0}; // Max SPS size
                        int sps_len, i;
                        next = avpriv_find_start_code(data, end, &state);
                        // Remove any emulation prevention bytes so SPS can be parsed properly
                        for (sps_len = 0, i = 0; i < next - data; sps_len++, i++)
                        {
                            if (i >= 2 && data[i - 2] == 0 && data[i - 1] == 0 && data[i] == 3)
                            {
                                sps_len--;
                                continue;
                            }
                            sps_data[sps_len] = data[i];
                        }
                        ret = init_get_bits8(&gb, sps_data, sps_len);
                        if (ret != 0) {
                            av_log(avctx, AV_LOG_ERROR, "Failed to initialize getBitContext for SPS parsing.\n");
                        } else {
                            ff_h264_decode_seq_parameter_set(&gb, avctx, &s->ps, 0);
                        }
                    }
                }
            }
        }

        s->svct_skip_next_packet = 0;

        // Custom SEI passthru type specified from command line will be ignored when timecode passthru is enabled
        int custom_sei_type = s->custom_sei_type;
        if (s->timecode_passthru) {
            if (avctx->codec_id == AV_CODEC_ID_H264) {
                custom_sei_type = 1; // picture timing SEI
            } else if (avctx->codec_id == AV_CODEC_ID_HEVC) {
                custom_sei_type = 136; // time code SEI
            }
        }

        // If there was lone custom sei in the last packet and the firmware would
        // fail to recoginze it. So passthrough the custom sei here.
        if (s->lone_sei_pkt.size > 0) {
            // No need to check the return value here because the lone_sei_pkt was
            // parsed before. Here it is only to extract the SEI data.
            ni_dec_packet_parse(&s->api_ctx, &s->api_param, s->lone_sei_pkt.data,
                              s->lone_sei_pkt.size, xpkt, s->low_delay,
                              s->api_ctx.codec_format, s->pkt_nal_bitmap,
                              custom_sei_type, &s->svct_skip_next_packet,
                              &s->is_lone_sei_pkt);
        }

        ret = ni_dec_packet_parse(&s->api_ctx, &s->api_param, pkt->data, pkt->size, xpkt,
                                 s->low_delay, s->api_ctx.codec_format, s->pkt_nal_bitmap,
                                 custom_sei_type, &s->svct_skip_next_packet,
                                 &s->is_lone_sei_pkt);
        if (ret < 0) {
            goto fail;
        }

        if (s->timecode_passthru) {
            ni_timecode_t timecode = {0};
            GetBitContext gb;
            if (xpkt->p_custom_sei_set) {
                for (int i = 0; i < xpkt->p_custom_sei_set->count; i++) {
                    ni_custom_sei_t *custom_sei = &(xpkt->p_custom_sei_set->custom_sei[i]);
                    if (avctx->codec_id == AV_CODEC_ID_H264 && custom_sei->type == 1) {
                        ret = init_get_bits8(&gb, custom_sei->data, custom_sei->size);
                        if (ret != 0) {
                            av_log(avctx, AV_LOG_ERROR, "Failed to initialize getBitContext for SEI parsing.\n");
                        } else {
#if LIBAVCODEC_VERSION_MAJOR >= 60
                            const SPS *sps = s->ps.sps_list[0];
#else
                            const SPS *sps = NULL;
                            if (s->ps.sps_list[0]) {
                                sps = (const SPS *)s->ps.sps_list[0]->data;
                            }
#endif
                            if (!sps) {
                                av_log(avctx, AV_LOG_ERROR, "SPS is not parsed but timecode passthrough is enabled. "
                                        "Cannot parse picture timing SEI to get timecode. "
                                        "A dummy timecode will be inserted by NI encoder\n");
                            } else {
                                if (sps->nal_hrd_parameters_present_flag || sps->vcl_hrd_parameters_present_flag) {
                                    skip_bits_long(&gb, sps->cpb_removal_delay_length); // cpb_removal_delay
                                    skip_bits_long(&gb, sps->dpb_output_delay_length); // dpb_output_delay
                                }
                                if (sps->pic_struct_present_flag) {
                                    skip_bits(&gb, 4); // pic_struct
                                    // In H264 the number of timecodes present is determined by the pic_struct field
                                    // We only support progressive video which should have only 1 timecode
                                    // Therefore just parse the first timecode present and ignore any extra ones
                                    if (get_bits(&gb, 1)) { // clock_timestamp_flag
                                        skip_bits(&gb, 2); // ct_type
                                        timecode.nuit_field_based_flag = get_bits(&gb, 1);
                                        timecode.counting_type = get_bits(&gb, 5);
                                        timecode.full_timestamp_flag = get_bits(&gb, 1);
                                        timecode.discontinuity_flag = get_bits(&gb, 1);
                                        timecode.cnt_dropped_flag = get_bits(&gb, 1);
                                        timecode.n_frames = get_bits(&gb, 8);
                                        if (timecode.full_timestamp_flag) {
                                            timecode.seconds_value = get_bits(&gb, 6);
                                            timecode.minutes_value = get_bits(&gb, 6);
                                            timecode.hours_value = get_bits(&gb, 5);
                                        } else {
                                            timecode.seconds_flag = get_bits(&gb, 1);
                                            if (timecode.seconds_flag) {
                                                timecode.seconds_value = get_bits(&gb, 6);
                                                timecode.minutes_flag = get_bits(&gb, 1);
                                                if (timecode.minutes_flag) {
                                                    timecode.minutes_value = get_bits(&gb, 6);
                                                    timecode.hours_flag = get_bits(&gb, 1);
                                                    if (timecode.hours_flag) {
                                                        timecode.hours_value = get_bits(&gb, 5);
                                                    }
                                                }
                                            }
                                        }
                                        if (sps->time_offset_length > 0)
                                        {
                                            timecode.time_offset_value = get_bits_long(&gb, sps->time_offset_length);
                                        }
                                    }
                                }
                            }
                            // Replace the raw SEI data with parsed ni_timecode_t struct
                            memcpy(custom_sei->data, &timecode, sizeof(ni_timecode_t));
                            custom_sei->size = sizeof(ni_timecode_t);
                            // Use an SEI type that's not used by H264 or H265 to passthru the custom timecode data
                            custom_sei->type = 240;
                        }
                    } else if (avctx->codec_id == AV_CODEC_ID_HEVC && custom_sei->type == 136) {
                        ret = init_get_bits8(&gb, custom_sei->data, custom_sei->size);
                        if (ret != 0) {
                            av_log(avctx, AV_LOG_ERROR, "Failed to initialize getBitContext for SEI parsing.\n");
                        } else {
                            int num_clock_ts = get_bits(&gb, 2);
                            if (num_clock_ts == 0) {
                                av_log(avctx, AV_LOG_VERBOSE, "num_clock_ts provided in time code SEI is 0. "
                                    "A dummy timecode will be inserted by NI encoder\n");
                            } else {
                                if (num_clock_ts > 1) {
                                    av_log(avctx, AV_LOG_VERBOSE, "More than 1 timecode provided in time code SEI. "
                                        "Only the first one will be inserted by NI encoder");
                                }
                                if (get_bits(&gb, 1)) { // clock_timestamp_flag
                                    timecode.nuit_field_based_flag = get_bits(&gb, 1);
                                    timecode.counting_type = get_bits(&gb, 5);
                                    timecode.full_timestamp_flag = get_bits(&gb, 1);
                                    timecode.discontinuity_flag = get_bits(&gb, 1);
                                    timecode.cnt_dropped_flag = get_bits(&gb, 1);
                                    timecode.n_frames = get_bits(&gb, 9);
                                    if (timecode.full_timestamp_flag) {
                                        timecode.seconds_value = get_bits(&gb, 6);
                                        timecode.minutes_value = get_bits(&gb, 6);
                                        timecode.hours_value = get_bits(&gb, 5);
                                    } else {
                                        timecode.seconds_flag = get_bits(&gb, 1);
                                        if (timecode.seconds_flag) {
                                            timecode.seconds_value = get_bits(&gb, 6);
                                            timecode.minutes_flag = get_bits(&gb, 1);
                                            if (timecode.minutes_flag) {
                                                timecode.minutes_value = get_bits(&gb, 6);
                                                timecode.hours_flag = get_bits(&gb, 1);
                                                if (timecode.hours_flag) {
                                                    timecode.hours_value = get_bits(&gb, 5);
                                                }
                                            }
                                        }
                                    }
                                    timecode.time_offset_length = get_bits(&gb, 5);
                                    if (timecode.time_offset_length > 0) {
                                        timecode.time_offset_value = get_bits_long(&gb, timecode.time_offset_length);
                                    }
                                }
                            }
                            // Replace the raw SEI data with parsed ni_timecode_t struct
                            memcpy(custom_sei->data, &timecode, sizeof(ni_timecode_t));
                            custom_sei->size = sizeof(ni_timecode_t);
                            // Use an SEI type that's not used by H264 or H265 to passthru the custom timecode data
                            custom_sei->type = 240;
                        }
                    }
                }
            }
        }

        if (svct_skip_packet) {
            av_log(avctx, AV_LOG_TRACE, "xcoder_dec_send packet: pts:%" PRIi64 ","
                   " size:%d\n", pkt->pts, pkt->size);
            xpkt->data_len = 0;
            return pkt->size;
        }

        // If the current packet is a lone SEI, save it to be sent with the next
        // packet. And also check if getting the first packet containing key frame
        // in decoder low delay mode.
        if (s->is_lone_sei_pkt) {
            av_packet_ref(&s->lone_sei_pkt, pkt);
            xpkt->data_len = 0;
            ni_memfree(xpkt->p_custom_sei_set);
            if (s->low_delay && s->got_first_key_frame &&
                !(s->pkt_nal_bitmap & NI_GENERATE_ALL_NAL_HEADER_BIT)) {
                // Packets before the IDR is sent cannot be decoded. So
                // set packet num to zero here.
                s->api_ctx.decoder_low_delay = s->low_delay;
                s->api_ctx.pkt_num = 0;
                s->pkt_nal_bitmap |= NI_GENERATE_ALL_NAL_HEADER_BIT;
                av_log(avctx, AV_LOG_TRACE,
                       "xcoder_dec_send got first IDR in decoder low delay "
                       "mode, "
                       "delay time %dms, pkt_nal_bitmap %d\n",
                       s->low_delay, s->pkt_nal_bitmap);
            }
            av_log(avctx, AV_LOG_TRACE, "xcoder_dec_send pkt lone SEI, saved, "
                   "and return %d\n", pkt->size);
            return pkt->size;
        }

        // Send the previous saved lone SEI packet to the decoder
        if (s->lone_sei_pkt.size > 0) {
            av_log(avctx, AV_LOG_TRACE, "xcoder_dec_send copy over lone SEI "
                   "data size: %d\n", s->lone_sei_pkt.size);
            memcpy(s->api_ctx.p_leftover + s->api_ctx.prev_size,
                   s->lone_sei_pkt.data, s->lone_sei_pkt.size);
            s->api_ctx.prev_size += s->lone_sei_pkt.size;
            av_packet_unref(&s->lone_sei_pkt);
        }

        if (pkt->size + s->api_ctx.prev_size > 0) {
            ni_packet_buffer_alloc(xpkt, (pkt->size + s->api_ctx.prev_size));
            if (!xpkt->p_data) {
                ret = AVERROR(ENOMEM);
                goto fail;
            }
        }
        new_packet = 1;
    } else {
        send_size = xpkt->data_len;
    }

    av_log(avctx, AV_LOG_VERBOSE, "xcoder_dec_send: pkt->size=%d pkt->buf=%p\n", pkt->size, pkt->buf);

    if (s->started == 0) {
        xpkt->start_of_stream = 1;
        s->started = 1;
    }

    if (need_draining && !s->draining) {
        av_log(avctx, AV_LOG_VERBOSE, "Sending End Of Stream signal\n");
        xpkt->end_of_stream = 1;
        xpkt->data_len = 0;

        av_log(avctx, AV_LOG_TRACE, "ni_packet_copy before: size=%d, s->prev_size=%d, send_size=%d (end of stream)\n", pkt->size, s->api_ctx.prev_size, send_size);
        if (new_packet) {
            extra_prev_size = s->api_ctx.prev_size;
            send_size = ni_packet_copy(xpkt->p_data, pkt->data, pkt->size, s->api_ctx.p_leftover, &s->api_ctx.prev_size);
            // increment offset of data sent to decoder and save it
            xpkt->pos = (long long)s->offset;
            s->offset += pkt->size + extra_prev_size;
        }
        av_log(avctx, AV_LOG_TRACE, "ni_packet_copy after: size=%d, s->prev_size=%d, send_size=%d, xpkt->data_len=%d (end of stream)\n", pkt->size, s->api_ctx.prev_size, send_size, xpkt->data_len);

        if (send_size < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to copy pkt (status = %d)\n",
                   send_size);
            ret = AVERROR_EXTERNAL;
            goto fail;
        }
        xpkt->data_len += extra_prev_size;

        sent = 0;
        if (xpkt->data_len > 0) {
            sent = ni_device_session_write(&(s->api_ctx), &(s->api_pkt), NI_DEVICE_TYPE_DECODER);
        }
        if (sent < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to send eos signal (status = %d)\n",
                   sent);
            if (NI_RETCODE_ERROR_VPU_RECOVERY == sent) {
                ret = xcoder_decode_reset(avctx);
                if (0 == ret) {
                    ret = AVERROR(EAGAIN);
                }
            } else {
                ret = AVERROR(EIO);
            }
            goto fail;
        }
        av_log(avctx, AV_LOG_VERBOSE, "Queued eos (status = %d) ts=%llu\n",
               sent, xpkt->pts);
        s->draining = 1;

        ni_device_session_flush(&(s->api_ctx), NI_DEVICE_TYPE_DECODER);
    } else {
        av_log(avctx, AV_LOG_TRACE, "ni_packet_copy before: size=%d, s->prev_size=%d, send_size=%d\n", pkt->size, s->api_ctx.prev_size, send_size);
        if (new_packet) {
            extra_prev_size = s->api_ctx.prev_size;
            send_size = ni_packet_copy(xpkt->p_data, pkt->data, pkt->size, s->api_ctx.p_leftover, &s->api_ctx.prev_size);
            // increment offset of data sent to decoder and save it
            xpkt->pos = (long long)s->offset;
            s->offset += pkt->size + extra_prev_size;
        }
        av_log(avctx, AV_LOG_TRACE, "ni_packet_copy after: size=%d, s->prev_size=%d, send_size=%d, xpkt->data_len=%d\n", pkt->size, s->api_ctx.prev_size, send_size, xpkt->data_len);

        if (send_size < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to copy pkt (status = %d)\n", send_size);
            ret = AVERROR_EXTERNAL;
            goto fail;
        }
        xpkt->data_len += extra_prev_size;

        sent = 0;
        if (xpkt->data_len > 0) {
            sent = ni_device_session_write(&s->api_ctx, &(s->api_pkt), NI_DEVICE_TYPE_DECODER);
            av_log(avctx, AV_LOG_VERBOSE, "xcoder_dec_send pts=%" PRIi64 ", dts=%" PRIi64 ", pos=%" PRIi64 ", sent=%d\n", pkt->pts, pkt->dts, pkt->pos, sent);
        }
        if (sent < 0) {
            av_log(avctx, AV_LOG_ERROR, "Failed to send compressed pkt (status = %d)\n", sent);
            if (NI_RETCODE_ERROR_VPU_RECOVERY == sent) {
                ret = xcoder_decode_reset(avctx);
                if (0 == ret) {
                    ret = AVERROR(EAGAIN);
                }
            } else {
                ret = AVERROR(EIO);
            }
            goto fail;
        } else if (sent == 0) {
            av_log(avctx, AV_LOG_VERBOSE, "Queued input buffer size=0\n");
        } else if (sent < size) { /* partial sent; keep trying */
            av_log(avctx, AV_LOG_VERBOSE, "Queued input buffer size=%d\n", sent);
        }
    }

    if (xpkt->data_len == 0) {
        /* if this packet is done sending, free any sei buffer. */
        ni_memfree(xpkt->p_custom_sei_set);

#if LIBAVCODEC_VERSION_MAJOR >= 60
        /* save the opaque pointers from input packet to be copied to corresponding frame later */
        if (avctx->flags & AV_CODEC_FLAG_COPY_OPAQUE) {
            opaque_data = &s->opaque_data_array[s->opaque_data_pos];
            opaque_data->pkt_pos = pkt->pos;
            opaque_data->opaque = pkt->opaque;
            av_buffer_replace(&opaque_data->opaque_ref, pkt->opaque_ref);
            s->opaque_data_pos = (s->opaque_data_pos + 1) % s->opaque_data_nb;
        }
#endif
    }

    if (sent != 0) {
        //keep the current pkt to resend next time
        ni_packet_buffer_free(xpkt);
        return sent;
    } else {
        // special handling of return EAGAIN for FFmpeg-n6.1 and newer
#if ((LIBAVCODEC_VERSION_MAJOR > 60) || (LIBAVCODEC_VERSION_MAJOR == 60 && LIBAVCODEC_VERSION_MINOR >= 31))
        if (s->draining) {
            av_log(avctx, AV_LOG_WARNING, "%s draining, sent == 0, return 0!\n", __func__);
            return 0;
        } else {
            av_log(avctx, AV_LOG_VERBOSE, "%s NOT draining, sent == 0, return EAGAIN !\n", __func__);
            return AVERROR(EAGAIN);
        }
#else
        return AVERROR(EAGAIN);
#endif
    }

fail:
    ni_packet_buffer_free(xpkt);
    ni_memfree(xpkt->p_custom_sei_set);
    s->draining = 1;
    s->eos = 1;

    return ret;
}

static int retrieve_frame(AVCodecContext *avctx, AVFrame *data, int *got_frame,
                          ni_frame_t *xfme)
{
    XCoderDecContext *s = avctx->priv_data;
    ni_xcoder_params_t *p_param =
        &s->api_param; // dec params in union with enc params struct
    int num_extra_outputs = (p_param->dec_input_params.enable_out1 > 0) + (p_param->dec_input_params.enable_out2 > 0);
    uint32_t buf_size = xfme->data_len[0] + xfme->data_len[1] +
                        xfme->data_len[2] + xfme->data_len[3];
    uint8_t *buf = xfme->p_data[0];
    uint8_t *buf1, *buf2;
    bool is_hw, is_key_frame;
    int frame_planar;
    int stride = 0;
    int res = 0;
    AVHWFramesContext *ctx     = NULL;
    AVNIFramesContext *dst_ctx = NULL;
    AVFrame *frame = data;
    ni_aux_data_t *aux_data = NULL;
    AVFrameSideData *av_side_data = NULL;
    ni_session_data_io_t session_io_data1;
    ni_session_data_io_t session_io_data2;
    ni_session_data_io_t *p_session_data1 = &session_io_data1;
    ni_session_data_io_t *p_session_data2 = &session_io_data2;
    niFrameSurface1_t *p_data3;
    niFrameSurface1_t *p_data3_1;
    niFrameSurface1_t *p_data3_2;
#if LIBAVCODEC_VERSION_MAJOR >= 60
    OpaqueData *opaque_data;
    int i;
#endif

    av_log(avctx, AV_LOG_TRACE,
           "retrieve_frame: buf %p data_len [%d %d %d %d] buf_size %u\n", buf,
           xfme->data_len[0], xfme->data_len[1], xfme->data_len[2],
           xfme->data_len[3], buf_size);

    memset(p_session_data1, 0, sizeof(ni_session_data_io_t));
    memset(p_session_data2, 0, sizeof(ni_session_data_io_t));

    switch (avctx->sw_pix_fmt) {
    case AV_PIX_FMT_NV12:
    case AV_PIX_FMT_P010LE:
        frame_planar = NI_PIXEL_PLANAR_FORMAT_SEMIPLANAR;
        break;
    case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
    case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
        frame_planar = NI_PIXEL_PLANAR_FORMAT_TILED4X4;
        break;
    default:
        frame_planar = NI_PIXEL_PLANAR_FORMAT_PLANAR;
        break;
    }

    if (num_extra_outputs) {
        ni_frame_buffer_alloc(&(p_session_data1->data.frame), 1,
                             1, // width height does not matter//codec id does
                                // not matter//no metadata
                             1, 0, 1, 1, frame_planar);
        buf1 = p_session_data1->data.frame.p_data[0];
        if (num_extra_outputs > 1) {
            ni_frame_buffer_alloc(&(p_session_data2->data.frame), 1,
                                 1, // width height does not matter
                                 1, 0, 1, 1, frame_planar);
            buf2 = p_session_data2->data.frame.p_data[0];
        }
    }

    is_hw = xfme->data_len[3] > 0;
    if (is_hw) {
        if (frame->hw_frames_ctx) {
            ctx = (AVHWFramesContext*)frame->hw_frames_ctx->data;
            dst_ctx = (AVNIFramesContext*) ctx->hwctx;
        }

        // Note, the real first frame could be dropped due to AV_PKT_FLAG_DISCARD
        if ((dst_ctx != NULL) &&
            (dst_ctx->api_ctx.device_handle != s->api_ctx.device_handle)) {
            if (frame->hw_frames_ctx) {
                av_log(avctx, AV_LOG_VERBOSE,
                       "First frame, set hw_frame_context to copy decode sessions "
                       "threads\n");
                res = ni_device_session_copy(&s->api_ctx, &dst_ctx->api_ctx);
                if (NI_RETCODE_SUCCESS != res) {
                    return res;
                }
                av_log(avctx, AV_LOG_VERBOSE,
                       "retrieve_frame: blk_io_handle %d device_handle %d\n",
                       s->api_ctx.blk_io_handle, s->api_ctx.device_handle);
            }
        }
    }

    av_log(avctx, AV_LOG_VERBOSE, "decoding %" PRId64 " frame ...\n", s->api_ctx.frame_num);

    if (avctx->width <= 0) {
        av_log(avctx, AV_LOG_ERROR, "width is not set\n");
        return AVERROR_INVALIDDATA;
    }
    if (avctx->height <= 0) {
        av_log(avctx, AV_LOG_ERROR, "height is not set\n");
        return AVERROR_INVALIDDATA;
    }

    stride = s->api_ctx.active_video_width;

    av_log(avctx, AV_LOG_VERBOSE, "XFRAME SIZE: %d, STRIDE: %d\n", buf_size, stride);

    if (!is_hw && (stride == 0 || buf_size < stride * avctx->height)) {
        av_log(avctx, AV_LOG_ERROR, "Packet too small (%d)\n", buf_size);
        return AVERROR_INVALIDDATA;
    }

    is_key_frame = false;

    if (xfme->ni_pict_type & 0x10) // key frame marker VP9
        is_key_frame = true;

    switch (xfme->ni_pict_type & 0xF) {
    case DECODER_PIC_TYPE_IDR:
        is_key_frame = true;
    case PIC_TYPE_I:
        frame->pict_type = AV_PICTURE_TYPE_I;
        if(s->api_param.dec_input_params.enable_follow_iframe) {
            is_key_frame = true;
        }
        break;
    case PIC_TYPE_P:
        frame->pict_type = AV_PICTURE_TYPE_P;
        break;
    case PIC_TYPE_B:
        frame->pict_type = AV_PICTURE_TYPE_B;
        break;
    default:
        frame->pict_type = AV_PICTURE_TYPE_NONE;
    }

    if (avctx->codec_id == AV_CODEC_ID_MJPEG)
        is_key_frame = true;

#if (LIBAVCODEC_VERSION_MAJOR < 60)
    frame->key_frame = is_key_frame ? 1 : 0;
#else
    if (is_key_frame)
        frame->flags |= AV_FRAME_FLAG_KEY;
    else
        frame->flags &= ~AV_FRAME_FLAG_KEY;
#endif

    // lowdelay mode should close when frame is B frame
    if (frame->pict_type == AV_PICTURE_TYPE_B &&
        s->api_ctx.enable_low_delay_check &&
        s->low_delay) {
        av_log(avctx, AV_LOG_WARNING,
               "Warning: session %d decoder lowDelay mode "
               "is cancelled due to B frames with "
               "enable_low_delay_check, frame_num  %" PRId64 "\n",
               s->api_ctx.session_id, s->api_ctx.frame_num);
        s->low_delay = 0;
    }
    res = ff_decode_frame_props(avctx, frame);
    if (res < 0)
        return res;

#if FF_API_FRAME_PKT
FF_DISABLE_DEPRECATION_WARNINGS // FFmpeg-n6.1+ deprecates AVFrame->pkt_pos
    frame->pkt_pos = xfme->pkt_pos;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

#if LIBAVCODEC_VERSION_MAJOR >= 61 //7.0
    frame->duration = avctx->internal->last_pkt_props->duration;
#else
    frame->pkt_duration = avctx->internal->last_pkt_props->duration;
#endif

    if ((res = av_image_check_size(xfme->video_width, xfme->video_height, 0, avctx)) < 0)
        return res;

    if (is_hw) {
        frame->buf[0] = av_buffer_create(buf, buf_size, ni_frame_free, NULL, 0);
        if (num_extra_outputs) {
            frame->buf[1] =
                av_buffer_create(buf1, (int)(buf_size / 3), ni_frame_free, NULL, 0);
            buf1 = frame->buf[1]->data;
            memcpy(buf1, buf + sizeof(niFrameSurface1_t),
                   sizeof(niFrameSurface1_t)); // copy hwdesc to new buffer
            if (num_extra_outputs > 1) {
                frame->buf[2] = av_buffer_create(buf2, (int)(buf_size / 3),
                                               ni_frame_free, NULL, 0);
                buf2 = frame->buf[2]->data;
                memcpy(buf2, buf + 2 * sizeof(niFrameSurface1_t),
                       sizeof(niFrameSurface1_t));
            }
        }
    } else {
        frame->buf[0] = av_buffer_create(buf, buf_size, ni_align_free, xfme->dec_buf, 0);
    }
    av_log(avctx, AV_LOG_TRACE,
           "retrieve_frame: is_hw %d frame->buf[0] %p buf %p buf_size %u "
           "num_extra_outputs %d pkt_duration %ld\n",
           is_hw, frame->buf[0], buf, buf_size, num_extra_outputs,
#if LIBAVCODEC_VERSION_MAJOR >= 61 //7.0
           frame->duration
#else
           frame->pkt_duration
#endif
           );

    buf = frame->buf[0]->data;

    // retrieve side data if available
    ni_dec_retrieve_aux_data(xfme);

    // update avctx framerate with timing info
    if (xfme->vui_time_scale && xfme->vui_num_units_in_tick) {
        if (AV_CODEC_ID_H264 == avctx->codec_id) {
            int numticks;
#if LIBAVCODEC_VERSION_MAJOR < 62
            numticks = xfme->vui_num_units_in_tick * avctx->ticks_per_frame;
#else
            numticks = xfme->vui_num_units_in_tick * 2;
#endif
            av_reduce(&avctx->framerate.den, &avctx->framerate.num,
                      numticks,
                      xfme->vui_time_scale, 1 << 30);
        } else if (AV_CODEC_ID_H265 == avctx->codec_id) {
            av_reduce(&avctx->framerate.den, &avctx->framerate.num,
                     xfme->vui_num_units_in_tick, xfme->vui_time_scale, 1 << 30);
        }
    }

    if (xfme->vui_len > 0) {
        enum AVColorRange color_range = xfme->video_full_range_flag ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
        if ((avctx->color_range != color_range) ||
            (avctx->color_trc != xfme->color_trc) ||
            (avctx->colorspace != xfme->color_space) ||
            (avctx->color_primaries != xfme->color_primaries)) {
            avctx->color_range = frame->color_range = color_range;
            avctx->color_trc = frame->color_trc = xfme->color_trc;
            avctx->colorspace = frame->colorspace = xfme->color_space;
            avctx->color_primaries = frame->color_primaries = xfme->color_primaries;
        }

        if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
            if (frame->format == AV_PIX_FMT_YUVJ420P && color_range == AVCOL_RANGE_MPEG)
                frame->format = AV_PIX_FMT_YUV420P;
            else if (frame->format == AV_PIX_FMT_YUV420P && color_range == AVCOL_RANGE_JPEG)
                frame->format = AV_PIX_FMT_YUVJ420P;
        }
    }
    // User Data Unregistered SEI if available
    av_log(avctx, AV_LOG_VERBOSE, "#SEI# UDU (offset=%u len=%u)\n",
           xfme->sei_user_data_unreg_offset, xfme->sei_user_data_unreg_len);
    if (xfme->sei_user_data_unreg_offset) {
        if ((aux_data = ni_frame_get_aux_data(xfme, NI_FRAME_AUX_DATA_UDU_SEI))) {
            av_side_data = av_frame_new_side_data(
                frame, AV_FRAME_DATA_SEI_UNREGISTERED, aux_data->size);
            if (!av_side_data) {
                return AVERROR(ENOMEM);
            } else {
                memcpy(av_side_data->data, aux_data->data, aux_data->size);
            }
            av_log(avctx, AV_LOG_VERBOSE, "UDU SEI added (len=%d type=5)\n",
                   xfme->sei_user_data_unreg_len);
        } else {
            av_log(avctx, AV_LOG_ERROR, "UDU SEI dropped! (len=%d type=5)\n",
                   xfme->sei_user_data_unreg_len);
        }
    }

    // close caption data if available
    av_log(avctx, AV_LOG_VERBOSE, "#SEI# CC (offset=%u len=%u)\n",
           xfme->sei_cc_offset, xfme->sei_cc_len);
    if ((aux_data = ni_frame_get_aux_data(xfme, NI_FRAME_AUX_DATA_A53_CC))) {
        av_side_data =
            av_frame_new_side_data(frame, AV_FRAME_DATA_A53_CC, aux_data->size);

        if (!av_side_data) {
            return AVERROR(ENOMEM);
        } else {
            memcpy(av_side_data->data, aux_data->data, aux_data->size);
        }
    }

    // hdr10 sei data if available
    av_log(avctx, AV_LOG_VERBOSE, "#SEI# MDCV (offset=%u len=%u)\n",
           xfme->sei_hdr_mastering_display_color_vol_offset,
           xfme->sei_hdr_mastering_display_color_vol_len);
    if ((aux_data = ni_frame_get_aux_data(
             xfme, NI_FRAME_AUX_DATA_MASTERING_DISPLAY_METADATA))) {
        AVMasteringDisplayMetadata *mdm =
            av_mastering_display_metadata_create_side_data(frame);
        if (!mdm) {
            return AVERROR(ENOMEM);
        } else {
            memcpy(mdm, aux_data->data, aux_data->size);
        }
    }

    av_log(avctx, AV_LOG_VERBOSE, "#SEI# CLL (offset=%u len=%u)\n",
           xfme->sei_hdr_content_light_level_info_offset,
           xfme->sei_hdr_content_light_level_info_len);
    if ((aux_data = ni_frame_get_aux_data(
             xfme, NI_FRAME_AUX_DATA_CONTENT_LIGHT_LEVEL))) {
        AVContentLightMetadata *clm =
            av_content_light_metadata_create_side_data(frame);
        if (!clm) {
            return AVERROR(ENOMEM);
        } else {
            memcpy(clm, aux_data->data, aux_data->size);
        }
    }

    // hdr10+ sei data if available
    av_log(avctx, AV_LOG_VERBOSE, "#SEI# HDR10+ (offset=%u len=%u)\n",
           xfme->sei_hdr_plus_offset, xfme->sei_hdr_plus_len);
    if ((aux_data = ni_frame_get_aux_data(xfme, NI_FRAME_AUX_DATA_HDR_PLUS))) {
        AVDynamicHDRPlus *hdrp = av_dynamic_hdr_plus_create_side_data(frame);

        if (!hdrp) {
            return AVERROR(ENOMEM);
        } else {
            memcpy(hdrp, aux_data->data, aux_data->size);
        }
    } // hdr10+ sei

#if ((LIBXCODER_API_VERSION_MAJOR > 2) ||                                        \
      (LIBXCODER_API_VERSION_MAJOR == 2 && LIBXCODER_API_VERSION_MINOR>= 75))
    // save error_ratio to side data
    if (xfme->error_ratio > 0) {
        av_side_data =
            av_frame_new_side_data(frame, AV_FRAME_DATA_NETINT_ERROR_RATIO, sizeof(uint32_t));
        if (!av_side_data) {
            return AVERROR(ENOMEM);
        } else {
            memcpy(av_side_data->data, &xfme->error_ratio, sizeof(uint32_t));
        }
    }
#endif

    // remember to clean up auxiliary data of ni_frame after their use
    ni_frame_wipe_aux_data(xfme);

    if (xfme->p_custom_sei_set) {
        AVBufferRef *sei_ref = av_buffer_create((uint8_t *)xfme->p_custom_sei_set,
                                               sizeof(ni_custom_sei_set_t),
                                               __ni_free, NULL, 0);
        if (!sei_ref ||
            !av_frame_new_side_data_from_buf(frame, AV_FRAME_DATA_NETINT_CUSTOM_SEI,
                                           sei_ref)) {
            return AVERROR(ENOMEM);
        }
        xfme->p_custom_sei_set = NULL;
    }

    frame->pkt_dts = xfme->dts;
    frame->pts     = xfme->pts;
    if (xfme->pts != NI_NOPTS_VALUE) {
        s->current_pts = frame->pts;
    }
#if !((LIBAVCODEC_VERSION_MAJOR > 60) || (LIBAVCODEC_VERSION_MAJOR == 60 && LIBAVCODEC_VERSION_MINOR >= 31))
    else {
        // FFmpeg >= n6.1 shouldn't set pts for NOPTS case
        if (!s->api_ctx.ready_to_close) {
            s->current_pts = s->current_pts == NI_NOPTS_VALUE ? 0 : s->current_pts + frame->pkt_duration;
            frame->pts = s->current_pts;
        }
    }
#endif

    if (is_hw) {
        p_data3 = (niFrameSurface1_t*)(xfme->p_buffer + xfme->data_len[0] + xfme->data_len[1] + xfme->data_len[2]);
        frame->data[3] = xfme->p_buffer + xfme->data_len[0] + xfme->data_len[1] + xfme->data_len[2];

        av_log(avctx, AV_LOG_DEBUG, "retrieve_frame: OUT0 data[3] trace ui16FrameIdx = [%d], device_handle=%d bitdep=%d, WxH %d x %d\n",
               p_data3->ui16FrameIdx,
               p_data3->device_handle,
               p_data3->bit_depth,
               p_data3->ui16width,
               p_data3->ui16height);

        if (num_extra_outputs) {
            p_data3_1 = (niFrameSurface1_t*)buf1;
            av_log(avctx, AV_LOG_DEBUG, "retrieve_frame: OUT1 data[3] trace ui16FrameIdx = [%d], device_handle=%d bitdep=%d, WxH %d x %d\n",
                   p_data3_1->ui16FrameIdx,
                   p_data3_1->device_handle,
                   p_data3_1->bit_depth,
                   p_data3_1->ui16width,
                   p_data3_1->ui16height);
            if (num_extra_outputs > 1) {
                p_data3_2 = (niFrameSurface1_t*)buf2;
                av_log(avctx, AV_LOG_DEBUG, "retrieve_frame: OUT2 data[3] trace ui16FrameIdx = [%d], device_handle=%d bitdep=%d, WxH %d x %d\n",
                       p_data3_2->ui16FrameIdx,
                       p_data3_2->device_handle,
                       p_data3_2->bit_depth,
                       p_data3_2->ui16width,
                       p_data3_2->ui16height);
            }
        }
    }
    av_log(avctx, AV_LOG_VERBOSE, "retrieve_frame: frame->buf[0]=%p, frame->data=%p, frame->pts=%" PRId64 ", frame size=%d, s->current_pts=%" PRId64 ", frame->pkt_duration=%" PRId64 " sei size %d offset %u\n",
           frame->buf[0], frame->data, frame->pts, buf_size, s->current_pts,
#if LIBAVCODEC_VERSION_MAJOR >= 61 //7.0
           frame->duration,
#else
           frame->pkt_duration,
#endif
           xfme->sei_cc_len, xfme->sei_cc_offset);

    /* av_buffer_ref(avpkt->buf); */
    if (!frame->buf[0])
        return AVERROR(ENOMEM);

    if (!is_hw &&
        ((res = av_image_fill_arrays(
              frame->data, frame->linesize, buf, avctx->sw_pix_fmt,
              (int)(s->api_ctx.active_video_width / s->api_ctx.bit_depth_factor),
              s->api_ctx.active_video_height, 1)) < 0)) {
        av_buffer_unref(&frame->buf[0]);
        return res;
    }

    av_log(avctx, AV_LOG_VERBOSE, "retrieve_frame: success av_image_fill_arrays "
           "return %d\n", res);

    if (!is_hw) {
        frame->linesize[1] = frame->linesize[2] = (((frame->width / ((frame_planar == 0) ? 1 : 2) * s->api_ctx.bit_depth_factor) + 127) / 128) * 128;
        frame->linesize[2] = (frame_planar == 0) ? 0 : frame->linesize[1];
        frame->data[2] = (frame_planar == 0) ? 0 : frame->data[1] + (frame->linesize[1] * frame->height / 2);
    }

    frame->crop_top = xfme->crop_top;
    frame->crop_bottom = frame->height - xfme->crop_bottom; // ppu auto crop should have cropped out padding, crop_bottom should be 0
    frame->crop_left = xfme->crop_left;
    frame->crop_right = frame->width - xfme->crop_right; // ppu auto crop should have cropped out padding, crop_right should be 0

    if (is_hw && frame->hw_frames_ctx && dst_ctx != NULL) {
        av_log(avctx, AV_LOG_TRACE,
               "retrieve_frame: hw_frames_ctx av_buffer_get_ref_count=%d\n",
               av_buffer_get_ref_count(frame->hw_frames_ctx));
        dst_ctx->split_ctx.enabled = (num_extra_outputs >= 1) ? 1 : 0;
        dst_ctx->split_ctx.w[0]    = p_data3->ui16width;
        dst_ctx->split_ctx.h[0]    = p_data3->ui16height;
        dst_ctx->split_ctx.f[0]    = (int)p_data3->encoding_type;
        dst_ctx->split_ctx.f8b[0]  = (int)p_data3->bit_depth;
        dst_ctx->split_ctx.w[1] =
            (num_extra_outputs >= 1) ? p_data3_1->ui16width : 0;
        dst_ctx->split_ctx.h[1] =
            (num_extra_outputs >= 1) ? p_data3_1->ui16height : 0;
        dst_ctx->split_ctx.f[1] =
            (num_extra_outputs >= 1) ? p_data3_1->encoding_type : 0;
        dst_ctx->split_ctx.f8b[1] =
            (num_extra_outputs >= 1) ? p_data3_1->bit_depth : 0;
        dst_ctx->split_ctx.w[2] =
            (num_extra_outputs == 2) ? p_data3_2->ui16width : 0;
        dst_ctx->split_ctx.h[2] =
            (num_extra_outputs == 2) ? p_data3_2->ui16height : 0;
        dst_ctx->split_ctx.f[2] =
            (num_extra_outputs == 2) ? p_data3_2->encoding_type : 0;
        dst_ctx->split_ctx.f8b[2] =
            (num_extra_outputs == 2) ? p_data3_2->bit_depth : 0;
    }

#if LIBAVCODEC_VERSION_MAJOR >= 60
    /* retrive the opaque pointers saved earlier by matching the pkt_pos between output
     * frame and input packet, assuming that the pkt_pos of every input packet is unique */
    if (avctx->flags & AV_CODEC_FLAG_COPY_OPAQUE) {
        opaque_data = NULL;
        for (i = 0; i < s->opaque_data_nb; i++) {
            if (s->opaque_data_array[i].pkt_pos == (int64_t)xfme->pkt_pos) {
                opaque_data = &s->opaque_data_array[i];
                break;
            }
        }
        /* copy the pointers over to AVFrame if a matching entry found, otherwise it's unexpected so don't do anything */
        if (opaque_data) {
            frame->opaque = opaque_data->opaque;
            av_buffer_replace(&frame->opaque_ref, opaque_data->opaque_ref);
            av_buffer_unref(&opaque_data->opaque_ref);
            opaque_data->pkt_pos = -1;
        }
    }
#endif

    *got_frame = 1;
    return buf_size;
}

static int xcoder_dec_receive(AVCodecContext *avctx, XCoderDecContext *s,
                              AVFrame *frame, bool wait)
{
    /* call xcode_dec_receive to get a decoded YUV frame from the decoder
       instance */
    int ret = 0;
    int got_frame = 0;
    ni_session_data_io_t session_io_data;
    ni_session_data_io_t * p_session_data = &session_io_data;
    int alloc_mem, height, actual_width, cropped_width, cropped_height;
    bool bSequenceChange = 0;
    int frame_planar;

    if (s->draining && s->eos) {
        return AVERROR_EOF;
    }

#if ((LIBAVCODEC_VERSION_MAJOR > 60) || (LIBAVCODEC_VERSION_MAJOR == 60 && LIBAVCODEC_VERSION_MINOR >= 31))
read_op:
#endif
    memset(p_session_data, 0, sizeof(ni_session_data_io_t));

#if ((LIBAVCODEC_VERSION_MAJOR > 60) || (LIBAVCODEC_VERSION_MAJOR == 60 && LIBAVCODEC_VERSION_MINOR >= 31))
    if (s->draining) {
        s->api_ctx.burst_control = 0;
    } else
#endif
    if (s->api_ctx.frame_num % 2 == 0) {
        s->api_ctx.burst_control = (s->api_ctx.burst_control == 0 ? 1 : 0); //toggle
    }
    if (s->api_ctx.burst_control) {
        av_log(avctx, AV_LOG_DEBUG, "xcoder_dec_receive burst return%" PRId64 " frame\n", s->api_ctx.frame_num);
        return AVERROR(EAGAIN);
    }

    // if active video resolution has been obtained we just use it as it's the
    // exact size of frame to be returned, otherwise we use what we are told by
    // upper stream as the initial setting and it will be adjusted.
    height =
        (int)(s->api_ctx.active_video_height > 0 ? s->api_ctx.active_video_height
                                                : avctx->height);
    actual_width =
        (int)(s->api_ctx.actual_video_width > 0 ? s->api_ctx.actual_video_width
                                               : avctx->width);

    // allocate memory only after resolution is known (buffer pool set up)
    alloc_mem = (s->api_ctx.active_video_width > 0 &&
                 s->api_ctx.active_video_height > 0 ? 1 : 0);
    switch (avctx->sw_pix_fmt) {
    case AV_PIX_FMT_NV12:
    case AV_PIX_FMT_P010LE:
        frame_planar = NI_PIXEL_PLANAR_FORMAT_SEMIPLANAR;
        break;
    case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
    case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
        frame_planar = NI_PIXEL_PLANAR_FORMAT_TILED4X4;
        break;
    default:
        frame_planar = NI_PIXEL_PLANAR_FORMAT_PLANAR;
        break;
    }

    if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
        ret = ni_decoder_frame_buffer_alloc(
            s->api_ctx.dec_fme_buf_pool, &(p_session_data->data.frame), alloc_mem,
            actual_width, height, (avctx->codec_id == AV_CODEC_ID_H264),
            s->api_ctx.bit_depth_factor, frame_planar);
    } else {
        ret = ni_frame_buffer_alloc(&(p_session_data->data.frame), actual_width,
                                   height, (avctx->codec_id == AV_CODEC_ID_H264),
                                   1, s->api_ctx.bit_depth_factor, 3,
                                   frame_planar);
    }

    if (NI_RETCODE_SUCCESS != ret) {
        return AVERROR_EXTERNAL;
    }

    if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
        ret = ni_device_session_read(&s->api_ctx, p_session_data, NI_DEVICE_TYPE_DECODER);
    } else {
        ret = ni_device_session_read_hwdesc(&s->api_ctx, p_session_data, NI_DEVICE_TYPE_DECODER);
    }

    if (ret == 0) {
        s->eos = p_session_data->data.frame.end_of_stream;
        if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
            ni_decoder_frame_buffer_free(&(p_session_data->data.frame));
        } else {
            ni_frame_buffer_free(&(p_session_data->data.frame));
        }

        if (s->eos) {
            return AVERROR_EOF;
        } else if (s->draining) {
            av_log(avctx, AV_LOG_ERROR, "ERROR: %s draining ret == 0 but not EOS\n", __func__);
            return AVERROR_EXTERNAL;
        }
        return AVERROR(EAGAIN);
    } else if (ret > 0) {
        int dec_ff_pix_fmt;

        if (p_session_data->data.frame.flags & AV_PKT_FLAG_DISCARD) {
            av_log(avctx, AV_LOG_DEBUG,
                   "Current frame is dropped when AV_PKT_FLAG_DISCARD is set\n");
            if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
                ni_decoder_frame_buffer_free(&(p_session_data->data.frame));
            } else {
                // recycle frame mem bin buffer of all PPU outputs & free p_buffer
                int num_outputs = (s->api_param.dec_input_params.enable_out1 > 0) +
                                (s->api_param.dec_input_params.enable_out2 > 0) + 1;
                ni_frame_free(&num_outputs, p_session_data->data.frame.p_buffer);
            }
#if ((LIBAVCODEC_VERSION_MAJOR > 60) || (LIBAVCODEC_VERSION_MAJOR == 60 && LIBAVCODEC_VERSION_MINOR >= 31))
            // FFmpeg-n6.1 doesn't allow egain in draing stage
            if (s->draining) {
                goto read_op;
            }
#endif
            return AVERROR(EAGAIN);
        }

        av_log(avctx, AV_LOG_VERBOSE, "Got output buffer pts=%lld "
                                     "dts=%lld eos=%d sos=%d\n",
               p_session_data->data.frame.pts, p_session_data->data.frame.dts,
               p_session_data->data.frame.end_of_stream, p_session_data->data.frame.start_of_stream);

        s->eos = p_session_data->data.frame.end_of_stream;

        // update ctxt resolution if change has been detected
        frame->width = cropped_width = p_session_data->data.frame.video_width; // ppu auto crop reports wdith as cropped width
        frame->height = cropped_height = p_session_data->data.frame.video_height; // ppu auto crop reports heigth as cropped height

        if (cropped_width != avctx->width || cropped_height != avctx->height) {
            av_log(avctx, AV_LOG_WARNING, "xcoder_dec_receive: resolution "
                   "changed: %dx%d to %dx%d\n", avctx->width, avctx->height,
                   cropped_width, cropped_height);
            avctx->width = cropped_width;
            avctx->height = cropped_height;
            bSequenceChange = 1;
        }

        dec_ff_pix_fmt = ni_pix_fmt_2_ff_pix_fmt(s->api_ctx.pixel_format);

        // If the codec is Jpeg or color range detected is a full range,
        // yuv420p from xxx_ni_quadra_dec means a full range.
        // Change it to yuvj420p so that FFmpeg can process it as a full range.
        if ((avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) &&
            (dec_ff_pix_fmt == AV_PIX_FMT_YUV420P) &&
            ((avctx->codec_id == AV_CODEC_ID_MJPEG) ||
             (avctx->color_range == AVCOL_RANGE_JPEG))) {
            avctx->sw_pix_fmt = avctx->pix_fmt = dec_ff_pix_fmt = AV_PIX_FMT_YUVJ420P;
            avctx->color_range = AVCOL_RANGE_JPEG;
        }

        if (avctx->sw_pix_fmt != dec_ff_pix_fmt) {
            av_log(avctx, AV_LOG_VERBOSE, "update sw_pix_fmt from %d to %d\n",
                   avctx->sw_pix_fmt, dec_ff_pix_fmt);
            avctx->sw_pix_fmt = dec_ff_pix_fmt;
            if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
                avctx->pix_fmt = avctx->sw_pix_fmt;
            }
            bSequenceChange = 1;
        }

        frame->format = avctx->pix_fmt;

        av_log(avctx, AV_LOG_VERBOSE, "xcoder_dec_receive: frame->format %d, sw_pix_fmt = %d\n", frame->format, avctx->sw_pix_fmt);

        if (avctx->pix_fmt == AV_PIX_FMT_NI_QUAD) {
            AVNIFramesContext *ni_hwf_ctx;

            if (bSequenceChange) {
                AVHWFramesContext *ctx;
                AVNIFramesContext *dst_ctx;

                av_buffer_unref(&avctx->hw_frames_ctx);
                avctx->hw_frames_ctx = av_hwframe_ctx_alloc(avctx->hw_device_ctx);
                if (!avctx->hw_frames_ctx) {
                    ret = AVERROR(ENOMEM);
                    return ret;
                }

                s->frames = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
                s->frames->format = AV_PIX_FMT_NI_QUAD;
                s->frames->width = avctx->width;
                s->frames->height = avctx->height;
                s->frames->sw_format = avctx->sw_pix_fmt;
                s->frames->initial_pool_size = -1; //Decoder has its own dedicated pool
                ret = av_hwframe_ctx_init(avctx->hw_frames_ctx);
                if (ret < 0) {
                    return ret;
                }

                ctx = (AVHWFramesContext*)avctx->hw_frames_ctx->data;
                dst_ctx = (AVNIFramesContext*) ctx->hwctx;
                av_log(avctx, AV_LOG_VERBOSE, "xcoder_dec_receive: sequence change, set hw_frame_context to copy decode sessions threads\n");
                ret = ni_device_session_copy(&s->api_ctx, &dst_ctx->api_ctx);
                if (NI_RETCODE_SUCCESS != ret) {
                    return ret;
                }
            }
            frame->hw_frames_ctx = av_buffer_ref(avctx->hw_frames_ctx);

            /* Set the hw_id/card number in AVNIFramesContext */
            ni_hwf_ctx = (AVNIFramesContext*)((AVHWFramesContext*)frame->hw_frames_ctx->data)->hwctx;
            ni_hwf_ctx->hw_id = s->dev_dec_idx;
        }
        if (s->api_ctx.frame_num == 1) {
            av_log(avctx, AV_LOG_DEBUG, "NI:%s:out\n",
                   (frame_planar == 0)   ? "semiplanar"
                   : (frame_planar == 2) ? "tiled"
                                        : "planar");
        }
        retrieve_frame(avctx, frame, &got_frame, &(p_session_data->data.frame));
        av_log(avctx, AV_LOG_VERBOSE, "xcoder_dec_receive: got_frame=%d, frame->width=%d, frame->height=%d, crop top %" SIZE_SPECIFIER " bottom %" SIZE_SPECIFIER " left %" SIZE_SPECIFIER " right %" SIZE_SPECIFIER ", frame->format=%d, frame->linesize=%d/%d/%d\n", got_frame, frame->width, frame->height, frame->crop_top, frame->crop_bottom, frame->crop_left, frame->crop_right, frame->format, frame->linesize[0], frame->linesize[1], frame->linesize[2]);

#if FF_API_PKT_PTS
        FF_DISABLE_DEPRECATION_WARNINGS
        frame->pkt_pts = frame->pts;
        FF_ENABLE_DEPRECATION_WARNINGS
#endif
        frame->best_effort_timestamp = frame->pts;

        av_log(avctx, AV_LOG_VERBOSE, "xcoder_dec_receive: pkt_timebase= %d/%d, frame_rate=%d/%d, frame->pts=%" PRId64 ", frame->pkt_dts=%" PRId64 "\n", avctx->pkt_timebase.num, avctx->pkt_timebase.den, avctx->framerate.num, avctx->framerate.den, frame->pts, frame->pkt_dts);

        // release buffer ownership and let frame owner return frame buffer to
        // buffer pool later
        p_session_data->data.frame.dec_buf = NULL;

        ni_memfree(p_session_data->data.frame.p_custom_sei_set);
    } else {
        av_log(avctx, AV_LOG_ERROR, "Failed to get output buffer (status = %d)\n",
               ret);

        if (NI_RETCODE_ERROR_VPU_RECOVERY == ret) {
            av_log(avctx, AV_LOG_WARNING, "xcoder_dec_receive VPU recovery, need to reset ..\n");
            if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
                ni_decoder_frame_buffer_free(&(p_session_data->data.frame));
            } else {
                ni_frame_buffer_free(&(p_session_data->data.frame));
            }
            return ret;
        } else if (ret == NI_RETCODE_ERROR_INVALID_SESSION ||
                  ret == NI_RETCODE_ERROR_NVME_CMD_FAILED) {
            if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
                ni_decoder_frame_buffer_free(&(p_session_data->data.frame));
            } else {
                ni_frame_buffer_free(&(p_session_data->data.frame));
            }
            return AVERROR_EOF;
        }
        if (avctx->pix_fmt != AV_PIX_FMT_NI_QUAD) {
            ni_decoder_frame_buffer_free(&(p_session_data->data.frame));
        } else {
            ni_frame_buffer_free(&(p_session_data->data.frame));
        }
        return AVERROR(EIO);
    }

    ret = 0;

    return ret;
}

static int xcoder_dec_is_flushing(AVCodecContext *avctx, XCoderDecContext *s)
{
    return s->flushing;
}

static int xcoder_dec_flush(AVCodecContext *avctx, XCoderDecContext *s)
{
    s->draining = 0;
    s->flushing = 0;
    s->eos = 0;

    /* Future: for now, always return 1 to indicate the codec has been flushed
       and it leaves the flushing state and can process again ! will consider
       case of user retaining frames in HW "surface" usage */
    return 1;
}
