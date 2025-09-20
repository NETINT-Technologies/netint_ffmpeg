/*
 * XCoder VP9 Decoder
 * Copyright (c) 2020 NetInt
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
#include "profiles.h"

static const enum AVPixelFormat ni_quadra_vp9_pix_fmts[] = {
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_NV12,
    AV_PIX_FMT_YUV420P10LE,
    AV_PIX_FMT_P010LE,
    AV_PIX_FMT_NI_QUAD,
    AV_PIX_FMT_NONE
};

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
    {NULL}};

static const AVClass vp9_xcoderdec_class = {
    .class_name = "vp9_ni_quadra_dec",
    .item_name  = av_default_item_name,
    .option     = dec_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
const FFCodec
#else
AVCodec
#endif
ff_vp9_ni_quadra_decoder = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
    .p.name         = "vp9_ni_quadra_dec",
#if (LIBAVCODEC_VERSION_MAJOR > 59)
    CODEC_LONG_NAME("VP9 NETINT Quadra decoder v" NI_XCODER_REVISION),
#else
    .p.long_name    = NULL_IF_CONFIG_SMALL("VP9 NETINT Quadra decoder v" NI_XCODER_REVISION),
#endif
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_VP9,
    .p.priv_class   = &vp9_xcoderdec_class,
    .p.capabilities = AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE,
#if (LIBAVCODEC_VERSION_MAJOR > 61)
    CODEC_PIXFMTS_ARRAY(ni_quadra_dec_vp9_pix_fmts),
#else
    .p.pix_fmts     = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12,
                                                    AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_P010LE,
                                                    AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE },
#endif
    .p.profiles     = NULL_IF_CONFIG_SMALL(ff_vp9_profiles),
    .p.wrapper_name = "libxcoder_quadra",
    FF_CODEC_RECEIVE_FRAME_CB(ff_xcoder_receive_frame),
#else
    .name           = "vp9_ni_quadra_dec",
    .long_name      = NULL_IF_CONFIG_SMALL("VP9 NETINT Quadra decoder v" NI_XCODER_REVISION),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_VP9,
    .priv_class     = &vp9_xcoderdec_class,
    .capabilities   = AV_CODEC_CAP_AVOID_PROBING | AV_CODEC_CAP_DELAY | AV_CODEC_CAP_HARDWARE,
    .pix_fmts       = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12,
                                                    AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_P010LE,
                                                    AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE },
    .profiles       = NULL_IF_CONFIG_SMALL(ff_vp9_profiles),
    .receive_frame  = ff_xcoder_receive_frame,
    .wrapper_name   = "libxcoder_quadra",
    .caps_internal  = FF_CODEC_CAP_SETS_PKT_DTS,
#endif
    .priv_data_size = sizeof(XCoderDecContext),
    .init           = ff_xcoder_decode_init,
    .close          = ff_xcoder_decode_close,
    .hw_configs     = ff_ni_quad_hw_configs,
    .flush          = ff_xcoder_decode_flush,
};
