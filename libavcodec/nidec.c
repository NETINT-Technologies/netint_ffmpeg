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
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_ni_quad.h"
#include "libavutil/mem.h"
#if (LIBAVCODEC_VERSION_MAJOR >= 61)
#include "fftools/ffmpeg_sched.h"
#endif

#define USER_DATA_UNREGISTERED_SEI_PAYLOAD_TYPE 5
#define NETINT_SKIP_PROFILE 0

int xcoder_decode_close(AVCodecContext *avctx) {
#if LIBAVCODEC_VERSION_MAJOR >= 60
    int i;
#endif
    XCoderDecContext *s = avctx->priv_data;
    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode close\n");

    /* this call shall release resource based on s->api_ctx */
    ff_xcoder_dec_close(avctx, s);

    av_packet_unref(&s->buffered_pkt);
    av_packet_unref(&s->lone_sei_pkt);

    free(s->extradata);
    s->extradata           = NULL;
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

static int xcoder_setup_decoder(AVCodecContext *avctx) {
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

int xcoder_decode_init(AVCodecContext *avctx) {
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

    if (avctx->codec_id == AV_CODEC_ID_H264) {
#if (LIBAVCODEC_VERSION_MAJOR < 60 || (LIBAVCODEC_VERSION_MAJOR == 60 && LIBAVCODEC_VERSION_MINOR < 3))
        // reference h264_decode_init in h264dec.c
        if (avctx->ticks_per_frame == 1) {
            if (avctx->time_base.den < INT_MAX / 2) {
                avctx->time_base.den *= 2;
            } else
                avctx->time_base.num /= 2;
        }
#endif
        avctx->ticks_per_frame = 2;
    }

    s->started = 0;
    memset(&s->api_pkt, 0, sizeof(ni_packet_t));
    s->pkt_nal_bitmap = 0;
    s->svct_skip_next_packet = 0;
    av_log(avctx, AV_LOG_VERBOSE, "XCoder decode init: time_base = %d/%d, frame rate = %d/%d, ticks_per_frame=%d\n", avctx->time_base.num, avctx->time_base.den, avctx->framerate.num, avctx->framerate.den, avctx->ticks_per_frame);

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

    if ((ret = ff_xcoder_dec_init(avctx, s)) < 0) {
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
int xcoder_decode_reset(AVCodecContext *avctx) {
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
    ret = xcoder_decode_init(avctx);
    s->current_pts = bcp_current_pts;
    s->api_ctx.session_run_state = SESSION_RUN_STATE_RESETTING;
    return ret;
}

static int xcoder_send_receive(AVCodecContext *avctx, XCoderDecContext *s,
                               AVFrame *frame, bool wait) {
    int ret;

    /* send any pending data from buffered packet */
    while (s->buffered_pkt.size) {
        ret = ff_xcoder_dec_send(avctx, s, &s->buffered_pkt);
        if (ret == AVERROR(EAGAIN))
            break;
        else if (ret < 0) {
            av_packet_unref(&s->buffered_pkt);
            return ret;
        }
        av_packet_unref(&s->buffered_pkt);
    }

    /* check for new frame */
    return ff_xcoder_dec_receive(avctx, s, frame, wait);
}

int xcoder_receive_frame(AVCodecContext *avctx, AVFrame *frame) {
    XCoderDecContext *s = avctx->priv_data;
    const AVPixFmtDescriptor *desc;
    int ret;

    av_log(avctx, AV_LOG_VERBOSE, "XCoder receive frame\n");
    /*
     * After we have buffered an input packet, check if the codec is in the
     * flushing state. If it is, we need to call ff_xcoder_dec_flush.
     *
     * ff_xcoder_dec_flush returns 0 if the flush cannot be performed on
     * the codec (because the user retains frames). The codec stays in the
     * flushing state.
     * For now we don't consider this case of user retaining the frame
     * (connected decoder-encoder case), so the return can only be 1
     * (flushed successfully), or < 0 (failure)
     *
     * ff_xcoder_dec_flush returns 1 if the flush can actually be
     * performed on the codec. The codec leaves the flushing state and can
     * process again packets.
     *
     * ff_xcoder_dec_flush returns a negative value if an error has
     * occurred.
     */
    if (ff_xcoder_dec_is_flushing(avctx, s)) {
        if (!ff_xcoder_dec_flush(avctx, s)) {
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
        ret = ff_xcoder_dec_send(avctx, s, &null_pkt);
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

void xcoder_decode_flush(AVCodecContext *avctx) {
    XCoderDecContext *s = avctx->priv_data;
    ni_device_dec_session_flush(&s->api_ctx);
    s->draining = 0;
    s->flushing = 0;
    s->eos = 0;
}
