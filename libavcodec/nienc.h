/*
 * NetInt XCoder H.264/HEVC Encoder common code header
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

#ifndef AVCODEC_NIENC_H
#define AVCODEC_NIENC_H

#include <ni_rsrc_api.h>
#include <ni_device_api.h>
#include <ni_util.h>

#include "libavutil/internal.h"

#include "avcodec.h"
#if (LIBAVCODEC_VERSION_MAJOR > 59 || (LIBAVCODEC_VERSION_MAJOR == 59 && LIBAVCODEC_VERSION_MINOR >= 37))
#include "codec_internal.h"
#endif
#include "internal.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/fifo.h"

// Needed for hwframe on FFmpeg-n4.3+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 82)
#include "hwconfig.h"
#endif
#include "nicodec.h"

#define IS_FFMPEG_70_AND_ABOVE_FOR_LIBAVCODEC                                                \
    (LIBAVCODEC_VERSION_MAJOR  >= 61)

#define IS_FFMPEG_61_AND_ABOVE_FOR_LIBAVCODEC                                                \
    (LIBAVCODEC_VERSION_MAJOR  >= 60)

typedef struct XCoderEncContext {
    AVClass *avclass;

    /* from the command line, which resource allocation method we use */
    char *dev_xcoder;
    char *dev_xcoder_name;          /* dev name of the xcoder card to use */
    char *blk_xcoder_name;          /* blk name of the xcoder card to use */
    int dev_enc_idx;                /* user-specified encoder index */
    char *dev_blk_name;             /* user-specified encoder block device name */
    int nvme_io_size;               /* custom nvme io size */
    int keep_alive_timeout;         /* keep alive timeout setting */
    ni_device_context_t *rsrc_ctx;  /* resource management context */
    uint64_t xcode_load_pixel; /* xcode load in pixels by this encode task */

#if LIBAVUTIL_VERSION_MAJOR >= 59 //7.0
    AVFifo *fme_fifo;
#else
    // frame fifo, to be used for sequence change frame buffering
    AVFifoBuffer *fme_fifo;
#endif
    int eos_fme_received;
    AVFrame buffered_fme; // buffered frame for sequence change handling

    ni_session_data_io_t  api_pkt; /* used for receiving bitstream from xcoder */
    ni_session_data_io_t   api_fme; /* used for sending YUV data to xcoder */
    ni_session_context_t api_ctx;
    ni_xcoder_params_t api_param;

    int started;
    uint8_t *p_spsPpsHdr;
    int spsPpsHdrLen;
    int spsPpsArrived;
    int firstPktArrived;
    int64_t dtsOffset;
    int gop_offset_count;/*this is a counter to guess the pts only dtsOffset times*/
    uint64_t total_frames_received;
    int64_t first_frame_pts;
    int64_t latest_dts;

    int encoder_flushing;
    int encoder_eof;

    // ROI
    int roi_side_data_size;
    AVRegionOfInterest *av_rois;  // last passed in AVRegionOfInterest
    int nb_rois;

    /* backup copy of original values of -enc command line option */
    int  orig_dev_enc_idx;

    AVFrame *sframe_pool[MAX_NUM_FRAMEPOOL_HWAVFRAME];
    int aFree_Avframes_list[MAX_NUM_FRAMEPOOL_HWAVFRAME + 1];
    int freeHead;
    int freeTail;

    /* below are all command line options */
    char *xcoder_opts;
    char *xcoder_gop;
    int gen_global_headers;
    int udu_sei;
    int timecode_passthru;

    int reconfigCount;
    int seqChangeCount;
    // actual enc_change_params is in ni_session_context !

} XCoderEncContext;

#define OFFSETENC(x) offsetof(XCoderEncContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM

// Common Netint encoder options
#define NI_ENC_OPTIONS\
    { "xcoder", "Select which XCoder card to use.", OFFSETENC(dev_xcoder), \
      AV_OPT_TYPE_STRING, { .str = NI_BEST_MODEL_LOAD_STR }, CHAR_MIN, CHAR_MAX, VE, "xcoder" }, \
    {     "bestmodelload", "Pick the least model load XCoder/encoder available.", 0, AV_OPT_TYPE_CONST, \
          { .str = NI_BEST_MODEL_LOAD_STR }, 0, 0, VE, "xcoder" }, \
    {     "bestload", "Pick the least real load XCoder/encoder available.", 0, AV_OPT_TYPE_CONST, \
          { .str = NI_BEST_REAL_LOAD_STR }, 0, 0, VE, "xcoder" }, \
    \
    { "enc", "Select which encoder to use by index. First is 0, second is 1, and so on.", \
      OFFSETENC(dev_enc_idx), AV_OPT_TYPE_INT, { .i64 = BEST_DEVICE_LOAD }, -1, INT_MAX, VE }, \
    \
    { "ni_enc_idx", "Select which encoder to use by index. First is 0, second is 1, and so on.", \
      OFFSETENC(dev_enc_idx), AV_OPT_TYPE_INT, { .i64 = BEST_DEVICE_LOAD }, -1, INT_MAX, VE }, \
    \
    { "ni_enc_name", "Select which encoder to use by NVMe block device name, e.g. /dev/nvme0n1.", \
      OFFSETENC(dev_blk_name), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE }, \
    \
    { "encname", "Select which encoder to use by NVMe block device name, e.g. /dev/nvme0n1.", \
      OFFSETENC(dev_blk_name), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE }, \
    \
    { "iosize", "Specify a custom NVMe IO transfer size (multiples of 4096 only).", \
      OFFSETENC(nvme_io_size), AV_OPT_TYPE_INT, { .i64 = BEST_DEVICE_LOAD }, -1, INT_MAX, VE }, \
    \
    { "xcoder-params", "Set the XCoder configuration using a :-separated list of key=value parameters.", \
      OFFSETENC(xcoder_opts), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE }, \
    \
    { "xcoder-gop", "Set the XCoder custom gop using a :-separated list of key=value parameters.", \
      OFFSETENC(xcoder_gop), AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE }, \
    \
    { "keep_alive_timeout", "Specify a custom session keep alive timeout in seconds.", \
      OFFSETENC(keep_alive_timeout), AV_OPT_TYPE_INT, { .i64 = NI_DEFAULT_KEEP_ALIVE_TIMEOUT }, \
      NI_MIN_KEEP_ALIVE_TIMEOUT, NI_MAX_KEEP_ALIVE_TIMEOUT, VE }

// "gen_global_headers" encoder options
#define NI_ENC_OPTION_GEN_GLOBAL_HEADERS \
    { "gen_global_headers", "Generate SPS and PPS headers during codec initialization.", \
      OFFSETENC(gen_global_headers), AV_OPT_TYPE_INT, { .i64 = GEN_GLOBAL_HEADERS_AUTO }, \
      GEN_GLOBAL_HEADERS_AUTO, GEN_GLOBAL_HEADERS_ON, VE, "gen_global_headers" }, \
    {     "auto", NULL, 0, AV_OPT_TYPE_CONST, \
          { .i64 = GEN_GLOBAL_HEADERS_AUTO }, 0, 0, VE, "gen_global_headers" }, \
    {     "off", NULL, 0, AV_OPT_TYPE_CONST, \
          { .i64 = GEN_GLOBAL_HEADERS_OFF }, 0, 0, VE, "gen_global_headers" }, \
    {     "on", NULL, 0, AV_OPT_TYPE_CONST, \
          { .i64 = GEN_GLOBAL_HEADERS_ON }, 0, 0, VE, "gen_global_headers" }

#define NI_ENC_OPTION_UDU_SEI \
    { "udu_sei", "Pass through user data unregistered SEI if available", OFFSETENC(udu_sei), \
      AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, VE }

#define NI_ENC_OPTION_TIMECODE_PASSTHRU \
    { "timecode_passthru", "Enable passthrough of time code in picture timing / time code SEI if present.", \
      OFFSETENC(timecode_passthru), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, VE, "timecode_passthru"}

av_cold int ff_xcoder_encode_init(AVCodecContext *avctx);

int ff_xcoder_encode_close(AVCodecContext *avctx);
// for FFmpeg 4.4+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
int ff_xcoder_receive_packet(AVCodecContext *avctx, AVPacket *pkt);
int ff_xcoder_receive_packet2(AVCodecContext *avctx, AVPacket *pkt);
int ff_xcoder_send_frame(AVCodecContext *avctx, const AVFrame *frame);
#else
int ff_xcoder_send_frame(AVCodecContext *avctx, const AVFrame *frame);
int ff_xcoder_receive_packet2(AVCodecContext *avctx, AVPacket *pkt);
int ff_xcoder_encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                                  const AVFrame *frame, int *got_packet);
#endif

// Needed for hwframe on FFmpeg-n4.3+
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 82)
extern const AVCodecHWConfigInternal *ff_ni_enc_hw_configs[];
#endif

#endif /* AVCODEC_NIENC_H */
