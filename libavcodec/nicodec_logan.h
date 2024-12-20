/*
 * XCoder Codec Lib Wrapper
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
 * XCoder codec lib wrapper header.
 */

#ifndef AVCODEC_NICODEC_LOGAN_H
#define AVCODEC_NICODEC_LOGAN_H

#include <stdbool.h>
#include <time.h>
#include "avcodec.h"
#include "libavutil/fifo.h"
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_ni_logan.h"

#include <ni_device_api_logan.h>

/* enum for specifying hardware accelbrate index */
enum {
  HW_FRAMES_OFF = 0,
  HW_FRAMES_ON = 1
};

typedef struct XCoderLoganDecContext {
  AVClass *avclass;

  char *dev_xcoder;         /* from the user command, which device allocation method we use */
  char *dev_xcoder_name;    /* dev name of the xcoder card to use */
  char *blk_xcoder_name;    /* blk name of the xcoder card to use */
  int  dev_dec_idx;         /* index of the decoder on the xcoder card */
  char *dev_dec_name;       /* name of the decoder on the xcoder card */
  int  keep_alive_timeout;    /* keep alive timeout setting */
  int  set_high_priority;   /*set_high_priority*/
  ni_logan_device_context_t *rsrc_ctx;  /* resource management context */

  ni_logan_session_context_t api_ctx;
  ni_logan_decoder_params_t  api_param;
  ni_logan_session_data_io_t api_pkt;

  AVPacket buffered_pkt;
  AVPacket seq_hdr_pkt;

  // stream header copied/saved from AVCodecContext.extradata
  int got_first_key_frame;
  uint8_t *extradata;
  int extradata_size;

  int64_t current_pts;
  unsigned long long offset;

  int started;
  int draining;
  int flushing;
  int eos;
  int vpu_reset;
  AVHWFramesContext    *hwfc;

  /* below are all command line options */
  char *xcoder_opts;
  int enable_user_data_sei_passthru;
  int enable_check_packet;  // check source packet. Skip SEI payloads after VCL
  int custom_sei;
  int low_delay;
  int pkt_nal_bitmap;
  int hwFrames;

} XCoderLoganDecContext;

typedef struct XCoderLoganEncContext {
  AVClass *avclass;

  char *dev_xcoder;         /* from the user command, which device allocation method we use */
  char *dev_xcoder_name;    /* dev name of the xcoder card to use */
  char *blk_xcoder_name;    /* blk name of the xcoder card to use */
  int  dev_enc_idx;         /* index of the encoder on the xcoder card */
  char *dev_enc_name;       /* name of the encoder on the xcoder card */
  uint8_t d_serial_number[20]; /*Serial number of card (dec) in use*/
  uint8_t e_serial_number[20]; /*Serial number of card (enc) in use*/
  int  keep_alive_timeout;    /* keep alive timeout setting */
  int  set_high_priority;   /*set_high_priority*/
  ni_logan_device_context_t *rsrc_ctx;  /* resource management context */
  unsigned long xcode_load_pixel; /* xcode load in pixels by this encode task */

#if LIBAVUTIL_VERSION_MAJOR >= 59 // 7.0
  AVFifo *fme_fifo;
#else
  // frame fifo, to be used for sequence change frame buffering
  AVFifoBuffer *fme_fifo;
#endif
  int fme_fifo_capacity;
  int eos_fme_received;
  AVFrame buffered_fme;

  ni_logan_session_data_io_t  api_pkt; /* used for receiving bitstream from xcoder */
  ni_logan_session_data_io_t   api_fme; /* used for sending YUV data to xcoder */
  ni_logan_session_context_t api_ctx;
  ni_logan_encoder_params_t  api_param;

  int started;
  uint8_t *p_spsPpsHdr;
  int spsPpsHdrLen;
  int spsPpsArrived;
  int firstPktArrived;
  int dts_offset;
  uint64_t total_frames_received;
  int64_t first_frame_pts;
  int64_t latest_dts;
  int vpu_reset;
  int encoder_flushing;
  int encoder_eof;

  // ROI
  int roi_side_data_size;
  AVRegionOfInterest *av_rois;  // last passed in AVRegionOfInterest
  int nb_rois;
  ni_logan_enc_avc_roi_custom_map_t *avc_roi_map; // actual AVC/HEVC map(s)
  uint8_t *hevc_sub_ctu_roi_buf;
  ni_logan_enc_hevc_roi_custom_map_t *hevc_roi_map;

  /* backup copy of original values of -enc command line option */
  int  orig_dev_enc_idx;

  // for hw trancoding
  // refer the hw frame when sending to encoder,
  // unrefer the hw frame after received the encoded packet.
  // Then it can recycle the HW frame buffer
  AVFrame *sframe_pool[LOGAN_MAX_NUM_FRAMEPOOL_HWAVFRAME];
  int aFree_Avframes_list[LOGAN_MAX_NUM_FRAMEPOOL_HWAVFRAME+1];
  int freeHead;
  int freeTail;

 /* below are all command line options */
  char *xcoder_opts;
  char *xcoder_gop;

  int reconfigCount;
  // actual enc_change_params is in ni_logan_session_context !

  // low delay mode flags
  int gotPacket; /* used to stop receiving packets when a packet is already received */
  int sentFrame; /* used to continue receiving packets when a frame is sent and a packet is not yet received */
} XCoderLoganEncContext;

int ff_xcoder_logan_dec_close(AVCodecContext *avctx,
                              XCoderLoganDecContext *s);

int ff_xcoder_logan_dec_init(AVCodecContext *avctx,
                             XCoderLoganDecContext *s);

int ff_xcoder_logan_dec_send(AVCodecContext *avctx,
                             XCoderLoganDecContext *s,
                             AVPacket *pkt);

int ff_xcoder_logan_dec_receive(AVCodecContext *avctx,
                                XCoderLoganDecContext *s,
                                AVFrame *frame,
                                bool wait);

int ff_xcoder_logan_dec_is_flushing(AVCodecContext *avctx,
                                    XCoderLoganDecContext *s);

int ff_xcoder_logan_dec_flush(AVCodecContext *avctx,
                              XCoderLoganDecContext *s);

int retrieve_logan_frame(AVCodecContext *avctx, AVFrame *data, int *got_frame,
                         ni_logan_frame_t *xfme);
int ff_xcoder_logan_add_headers(AVCodecContext *avctx, AVPacket *pkt,
                                uint8_t* extradata, int extradata_size);
#endif /* AVCODEC_NICODEC_LOGAN_H */
