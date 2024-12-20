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

#define OFFSETENC(x) offsetof(XCoderH265EncContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption enc_options[] = {
    {"xcoder",
     "Select which XCoder card to use.",
     OFFSETENC(dev_xcoder),
     AV_OPT_TYPE_STRING,
     {.str = NI_BEST_MODEL_LOAD_STR},
     CHAR_MIN,
     CHAR_MAX,
     VE,
     "xcoder"},

    {"bestmodelload",
     "Pick the least model load XCoder/encoder available.",
     0,
     AV_OPT_TYPE_CONST,
     {.str = NI_BEST_MODEL_LOAD_STR},
     0,
     0,
     VE,
     "xcoder"},

    {"bestload",
     "Pick the least real load XCoder/encoder available.",
     0,
     AV_OPT_TYPE_CONST,
     {.str = NI_BEST_REAL_LOAD_STR},
     0,
     0,
     VE,
     "xcoder"},

    {"enc",
     "Select which encoder to use by index. First is 0, second is 1, and so "
     "on.",
     OFFSETENC(dev_enc_idx),
     AV_OPT_TYPE_INT,
     {.i64 = BEST_DEVICE_LOAD},
     -1,
     INT_MAX,
     VE,
     "enc"},

    {"ni_enc_idx",
     "Select which encoder to use by index. First is 0, second is 1, and so "
     "on.",
     OFFSETENC(dev_enc_idx),
     AV_OPT_TYPE_INT,
     {.i64 = BEST_DEVICE_LOAD},
     -1,
     INT_MAX,
     VE,
     "ni_enc_idx"},

    {"ni_enc_name",
     "Select which encoder to use by NVMe block device name, e.g. "
     "/dev/nvme0n1.",
     OFFSETENC(dev_blk_name),
     AV_OPT_TYPE_STRING,
     {0},
     0,
     0,
     VE,
     "ni_enc_name"},

    {"encname",
     "Select which encoder to use by NVMe block device name, e.g. "
     "/dev/nvme0n1.",
     OFFSETENC(dev_blk_name),
     AV_OPT_TYPE_STRING,
     {0},
     0,
     0,
     VE,
     "encname"},

    {"iosize",
     "Specify a custom NVMe IO transfer size (multiples of 4096 only).",
     OFFSETENC(nvme_io_size),
     AV_OPT_TYPE_INT,
     {.i64 = BEST_DEVICE_LOAD},
     -1,
     INT_MAX,
     VE,
     "iosize"},

    {"xcoder-params",
     "Set the XCoder configuration using a :-separated list of key=value "
     "parameters.",
     OFFSETENC(xcoder_opts),
     AV_OPT_TYPE_STRING,
     {0},
     0,
     0,
     VE},

    {"xcoder-gop",
     "Set the XCoder custom gop using a :-separated list of key=value "
     "parameters.",
     OFFSETENC(xcoder_gop),
     AV_OPT_TYPE_STRING,
     {0},
     0,
     0,
     VE},

    {"keep_alive_timeout",
     "Specify a custom session keep alive timeout in seconds.",
     OFFSETENC(keep_alive_timeout),
     AV_OPT_TYPE_INT,
     {.i64 = NI_DEFAULT_KEEP_ALIVE_TIMEOUT},
     NI_MIN_KEEP_ALIVE_TIMEOUT,
     NI_MAX_KEEP_ALIVE_TIMEOUT,
     VE,
     "keep_alive_timeout"},

    {NULL}};

static const AVClass av1_xcoderenc_class = {
    .class_name = "av1_ni_quadra_enc",
    .item_name  = av_default_item_name,
    .option     = enc_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
FFCodec
#else
AVCodec
#endif
ff_av1_ni_quadra_encoder = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
    .p.name = "av1_ni_quadra_enc",
#if (LIBAVCODEC_VERSION_MAJOR > 59)
    CODEC_LONG_NAME("AV1 NETINT Quadra encoder v" NI_XCODER_REVISION),
#else
    .p.long_name = NULL_IF_CONFIG_SMALL("AV1 NETINT Quadra encoder v" NI_XCODER_REVISION),
#endif
    .p.type = AVMEDIA_TYPE_VIDEO,
    .p.id   = AV_CODEC_ID_AV1,
    .p.priv_class     = &av1_xcoderenc_class,
    .p.capabilities   = AV_CODEC_CAP_DELAY,
    .p.pix_fmts =
        (const enum AVPixelFormat[]){AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P,
                                     AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_NV12,
                                     AV_PIX_FMT_P010LE, AV_PIX_FMT_NI_QUAD,
                                     AV_PIX_FMT_NONE},
    FF_CODEC_RECEIVE_PACKET_CB(ff_xcoder_receive_packet),
#else
    .name = "av1_ni_quadra_enc",
    .long_name =
        NULL_IF_CONFIG_SMALL("AV1 NETINT Quadra encoder v" NI_XCODER_REVISION),
    .type = AVMEDIA_TYPE_VIDEO,
    .id   = AV_CODEC_ID_AV1,
    .priv_class     = &av1_xcoderenc_class,
    .capabilities   = AV_CODEC_CAP_DELAY,
    .pix_fmts =
        (const enum AVPixelFormat[]){AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P,
                                     AV_PIX_FMT_YUV420P10LE, AV_PIX_FMT_NV12,
                                     AV_PIX_FMT_P010LE, AV_PIX_FMT_NI_QUAD,
                                     AV_PIX_FMT_NONE},
// FFmpeg-n4.4+ has no more .send_frame;
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    .receive_packet = ff_xcoder_receive_packet,
#else
    .send_frame     = xcoder_send_frame,
    .receive_packet = xcoder_receive_packet,
    .encode2        = xcoder_encode_frame,
#endif
#endif
    .init = xcoder_encode_init,
    .close          = xcoder_encode_close,
    .priv_data_size = sizeof(XCoderH265EncContext),
// Needed for hwframe on FFmpeg-n4.3+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 82)
    .hw_configs = ff_ni_enc_hw_configs,
#endif
};
