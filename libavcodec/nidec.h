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

#define OFFSETDEC(x) offsetof(XCoderH264DecContext, x)
#define VD           AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_DECODING_PARAM

int xcoder_decode_close(AVCodecContext *avctx);

int xcoder_decode_init(AVCodecContext *avctx);

int xcoder_decode_reset(AVCodecContext *avctx);

int xcoder_receive_frame(AVCodecContext *avctx, AVFrame *frame);

void xcoder_decode_flush(AVCodecContext *avctx);

#endif /* AVCODEC_NIDEC_H */
