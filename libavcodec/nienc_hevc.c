/*
 * NetInt XCoder HEVC Encoder
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

static const AVOption enc_options[] = {
    NI_ENC_OPTIONS,
    NI_ENC_OPTION_GEN_GLOBAL_HEADERS,
    NI_ENC_OPTION_UDU_SEI,
    {NULL}
};

static const AVClass h265_xcoderenc_class = {
    .class_name = "h265_ni_quadra_enc",
    .item_name  = av_default_item_name,
    .option     = enc_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
FFCodec
#else
AVCodec
#endif
ff_h265_ni_quadra_encoder = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
    .p.name           = "h265_ni_quadra_enc",
#if (LIBAVCODEC_VERSION_MAJOR > 59)
    CODEC_LONG_NAME("H.265 NETINT Quadra encoder v" NI_XCODER_REVISION),
#else
    .p.long_name      = NULL_IF_CONFIG_SMALL("H.265 NETINT Quadra encoder v" NI_XCODER_REVISION),
#endif
    .p.type           = AVMEDIA_TYPE_VIDEO,
    .p.id             = AV_CODEC_ID_H265,
    .p.priv_class     = &h265_xcoderenc_class,
    .p.capabilities   = AV_CODEC_CAP_DELAY,
    .p.pix_fmts       = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P,
                                                      AV_PIX_FMT_YUV420P10, AV_PIX_FMT_NV12,
                                                      AV_PIX_FMT_P010LE, AV_PIX_FMT_NI_QUAD,
                                                      AV_PIX_FMT_NONE },
    FF_CODEC_RECEIVE_PACKET_CB(ff_xcoder_receive_packet),
#else
    .name             = "h265_ni_quadra_enc",
    .long_name        = NULL_IF_CONFIG_SMALL("H.265 NETINT Quadra encoder v" NI_XCODER_REVISION),
    .type             = AVMEDIA_TYPE_VIDEO,
    .id               = AV_CODEC_ID_H265,
    .priv_class       = &h265_xcoderenc_class,
    .capabilities     = AV_CODEC_CAP_DELAY,
    .pix_fmts         = (const enum AVPixelFormat[]){ AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P,
                                                      AV_PIX_FMT_YUV420P10, AV_PIX_FMT_NV12,
                                                      AV_PIX_FMT_P010LE, AV_PIX_FMT_NI_QUAD,
                                                      AV_PIX_FMT_NONE },
// FFmpeg-n4.4+ has no more .send_frame.
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    .receive_packet   = ff_xcoder_receive_packet,
#else
    .send_frame       = xcoder_send_frame,
    .receive_packet   = xcoder_receive_packet,
    .encode2          = xcoder_encode_frame,
#endif
#endif

    .init             = xcoder_encode_init,
    .close            = xcoder_encode_close,
    .priv_data_size   = sizeof(XCoderEncContext),
// Needed for hwframe on FFmpeg-n4.3+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 82)
    .hw_configs       = ff_ni_enc_hw_configs,
#endif
};
