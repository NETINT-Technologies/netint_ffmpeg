/*
 * XCoder HEVC Decoder
 * Copyright (c) 2018 NetInt
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
// Needed for hwframe on FFmpeg-n4.3+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 82)
#include "hwconfig.h"
#else
#include "hwaccel.h"
#endif

#if (LIBAVCODEC_VERSION_MAJOR > 61)
static const enum AVPixelFormat ni_quadra_dec_h265_pix_fmts[] = {
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_YUV420P10LE,
    AV_PIX_FMT_P010LE,
    AV_PIX_FMT_NI_QUAD,
    AV_PIX_FMT_NONE
};
#endif

static const AVCodecHWConfigInternal *ff_ni_quad_hw_configs[] = {
    &(const AVCodecHWConfigInternal) {
        .public = {
            .pix_fmt = AV_PIX_FMT_NI_QUAD,
            .methods = AV_CODEC_HW_CONFIG_METHOD_HW_FRAMES_CTX |
                       AV_CODEC_HW_CONFIG_METHOD_AD_HOC |
                       AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX,
            .device_type = AV_HWDEVICE_TYPE_NI_QUADRA,
        },
        .hwaccel = NULL,
    },
    NULL
};

static const AVOption dec_options[] = {
    NI_DEC_OPTIONS,
    NI_DEC_OPTION_SEI_PASSTHRU,
    NI_DEC_OPTION_LOW_DELAY,
    {NULL}};

static const AVClass h265_xcoderdec_class = {
    .class_name = "h265_ni_quadra_dec",
    .item_name  = av_default_item_name,
    .option     = dec_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
const FFCodec
#else
AVCodec
#endif
ff_h265_ni_quadra_decoder = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
    .p.name         = "h265_ni_quadra_dec",
#if (LIBAVCODEC_VERSION_MAJOR > 59)
    CODEC_LONG_NAME("H.265 NETINT Quadra decoder v" NI_XCODER_REVISION),
#else
    .p.long_name    = NULL_IF_CONFIG_SMALL("H.265 NETINT Quadra decoder v" NI_XCODER_REVISION),
#endif
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_HEVC,
    .p.priv_class   = &h265_xcoderdec_class,
    .p.capabilities = AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE,
#if (LIBAVCODEC_VERSION_MAJOR > 61)
    CODEC_PIXFMTS_ARRAY(ni_quadra_dec_h265_pix_fmts),
#else
    .p.pix_fmts     = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12,
                                                    AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_P010LE,
                                                    AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE },
#endif
    FF_CODEC_RECEIVE_FRAME_CB(ff_xcoder_receive_frame),
    .p.wrapper_name = "libxcoder_quadra",
#else
    .name           = "h265_ni_quadra_dec",
    .long_name      = NULL_IF_CONFIG_SMALL("H.265 NETINT Quadra decoder v" NI_XCODER_REVISION),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_HEVC,
    .priv_class     = &h265_xcoderdec_class,
    .capabilities   = AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE,
    .pix_fmts       = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12,
                                                    AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_P010LE,
                                                    AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE },
    .receive_frame  = ff_xcoder_receive_frame,
    .wrapper_name   = "libxcoder_quadra",
    .caps_internal  = FF_CODEC_CAP_SETS_PKT_DTS,
#endif
    .priv_data_size = sizeof(XCoderDecContext),
    .init           = ff_xcoder_decode_init,
    .close          = ff_xcoder_decode_close,
    .hw_configs     = ff_ni_quad_hw_configs,
    .bsfs           = "hevc_mp4toannexb",
    .flush          = ff_xcoder_decode_flush,
};
