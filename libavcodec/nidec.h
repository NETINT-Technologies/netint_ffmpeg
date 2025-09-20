/*
 * NetInt XCoder H.264/HEVC Decoder common code header
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

#ifndef AVCODEC_NIDEC_H
#define AVCODEC_NIDEC_H

#include <stdbool.h>
#include <ni_rsrc_api.h>
#include <ni_device_api.h>
#include <ni_util.h>

#include "avcodec.h"
#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
#include "codec_internal.h"
#endif
#include "decode.h"
#include "internal.h"

#include "libavutil/internal.h"
#include "libavutil/frame.h"
#include "libavutil/buffer.h"
#include "libavutil/pixdesc.h"
#include "libavutil/opt.h"

#include "nicodec.h"

#if LIBAVCODEC_VERSION_MAJOR >= 60
typedef struct OpaqueData {
    int64_t pkt_pos;
    void *opaque;
    AVBufferRef *opaque_ref;
} OpaqueData;
#endif

typedef struct XCoderDecContext {
    AVClass *avclass;

    /* from the command line, which resource allocation method we use */
    char *dev_xcoder;
    char *dev_xcoder_name;          /* dev name of the xcoder card to use */
    char *blk_xcoder_name;          /* blk name of the xcoder card to use */
    int dev_dec_idx;                /* user-specified decoder index */
    char *dev_blk_name;             /* user-specified decoder block device name */
    int keep_alive_timeout;         /* keep alive timeout setting */
    ni_device_context_t *rsrc_ctx;  /* resource management context */

    ni_session_context_t api_ctx;
    ni_xcoder_params_t api_param;
    ni_session_data_io_t api_pkt;

    AVPacket buffered_pkt;
    AVPacket lone_sei_pkt;

    // stream header copied/saved from AVCodecContext.extradata
    int got_first_key_frame;
    uint8_t *extradata;
    int extradata_size;

    int64_t current_pts;
    unsigned long long offset;
    int svct_skip_next_packet;

    int started;
    int draining;
    int flushing;
    int is_lone_sei_pkt;
    int eos;
    AVHWFramesContext    *frames;

#if LIBAVCODEC_VERSION_MAJOR >= 60
    /* for temporarily storing the opaque pointers when AV_CODEC_FLAG_COPY_OPAQUE is set */
    OpaqueData *opaque_data_array;
    int opaque_data_nb;
    int opaque_data_pos;
#endif

    /* below are all command line options */
    char *xcoder_opts;
    int enable_user_data_sei_passthru;
    int custom_sei_type;
    int low_delay;
    int pkt_nal_bitmap;
    int timecode_passthru;

    H264ParamSets ps;
} XCoderDecContext;

#define OFFSETDEC(x) offsetof(XCoderDecContext, x)
#define VD           AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM

// Common Netint decoder options
#define NI_DEC_OPTIONS\
    { "xcoder", "Select which XCoder card to use.", OFFSETDEC(dev_xcoder), \
      AV_OPT_TYPE_STRING, {.str = NI_BEST_MODEL_LOAD_STR}, CHAR_MIN, CHAR_MAX, VD, "xcoder"}, \
    {     "bestmodelload", "Pick the least model load XCoder/decoder available.", 0,\
          AV_OPT_TYPE_CONST, {.str = NI_BEST_MODEL_LOAD_STR}, 0, 0, VD, "xcoder"},\
    {     "bestload", "Pick the least real load XCoder/decoder available.", 0,\
          AV_OPT_TYPE_CONST, {.str = NI_BEST_REAL_LOAD_STR}, 0, 0, VD, "xcoder"},\
    \
    { "dec", "Select which decoder to use by index. First is 0, second is 1, and so on.", \
      OFFSETDEC(dev_dec_idx), AV_OPT_TYPE_INT, {.i64 = BEST_DEVICE_LOAD}, -1, INT_MAX, VD, "dec"}, \
    \
    { "ni_dec_idx", "Select which decoder to use by index. First is 0, second is 1, and so on.", \
      OFFSETDEC(dev_dec_idx), AV_OPT_TYPE_INT, {.i64 = BEST_DEVICE_LOAD}, -1, INT_MAX, VD, "ni_dec_idx"}, \
    \
    { "ni_dec_name", "Select which decoder to use by NVMe block device name, e.g. /dev/nvme0n1.", \
      OFFSETDEC(dev_blk_name), AV_OPT_TYPE_STRING, {0}, 0, 0, VD, "ni_dec_name"}, \
    \
    { "decname", "Select which decoder to use by NVMe block device name, e.g. /dev/nvme0n1.", \
      OFFSETDEC(dev_blk_name), AV_OPT_TYPE_STRING, {0}, 0, 0, VD, "decname"}, \
    \
    { "xcoder-params", "Set the XCoder configuration using a :-separated list of key=value parameters.", \
      OFFSETDEC(xcoder_opts), AV_OPT_TYPE_STRING, {0}, 0, 0, VD}, \
    \
    { "keep_alive_timeout", "Specify a custom session keep alive timeout in seconds.", \
      OFFSETDEC(keep_alive_timeout), AV_OPT_TYPE_INT, {.i64 = NI_DEFAULT_KEEP_ALIVE_TIMEOUT}, \
      NI_MIN_KEEP_ALIVE_TIMEOUT, NI_MAX_KEEP_ALIVE_TIMEOUT, VD, "keep_alive_timeout"}

#define NI_DEC_OPTION_SEI_PASSTHRU\
    { "user_data_sei_passthru", "Enable user data unregistered SEI passthrough.", \
      OFFSETDEC(enable_user_data_sei_passthru), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VD, "user_data_sei_passthru"}, \
    \
    { "custom_sei_passthru", "Specify a custom SEI type to passthrough.", \
      OFFSETDEC(custom_sei_type), AV_OPT_TYPE_INT, {.i64 = -1}, -1, 254, VD, "custom_sei_passthru"}, \
    \
    { "timecode_passthru", "Enable passthrough of time code in picture timing / time code SEI if present.", \
      OFFSETDEC(timecode_passthru), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VD, "timecode_passthru"}

#define NI_DEC_OPTION_LOW_DELAY\
    { "low_delay", "Enable low delay decoding mode for 1 in, 1 out decoding sequence. " \
      "Set 1 to enable low delay mode. Should be used only for streams that are in sequence.", \
      OFFSETDEC(low_delay), AV_OPT_TYPE_INT, {.i64 = 0}, 0, 1, VD, "low_delay"}

int ff_xcoder_decode_close(AVCodecContext *avctx);

int ff_xcoder_decode_init(AVCodecContext *avctx);

int ff_xcoder_receive_frame(AVCodecContext *avctx, AVFrame *frame);

void ff_xcoder_decode_flush(AVCodecContext *avctx);

#endif /* AVCODEC_NIDEC_H */
