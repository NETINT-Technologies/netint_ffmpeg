/*
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

#ifndef AVUTIL_HWCONTEXT_NI_QUAD_H
#define AVUTIL_HWCONTEXT_NI_QUAD_H

#include "hwcontext.h"
#include <ni_device_api.h>
#include <ni_rsrc_api.h>
#include <ni_util.h>

#define IS_FFMPEG_61_AND_ABOVE_FOR_LIBAVUTIL                                   \
    ((LIBAVUTIL_VERSION_MAJOR > 58) ||                                         \
     (LIBAVUTIL_VERSION_MAJOR == 58 && LIBAVUTIL_VERSION_MINOR >= 29))

#define IS_FFMPEG_70_AND_ABOVE_FOR_LIBAVUTIL                                   \
    ((LIBAVUTIL_VERSION_MAJOR > 59) ||                                         \
     (LIBAVUTIL_VERSION_MAJOR == 59 && LIBAVUTIL_VERSION_MINOR >= 8))

enum
{
    NI_MEMTYPE_VIDEO_MEMORY_NONE,
    NI_MEMTYPE_VIDEO_MEMORY_DECODER_TARGET,
    NI_MEMTYPE_VIDEO_MEMORY_HWUPLOAD_TARGET,
};

typedef enum _ni_filter_poolsize_code {
    NI_DECODER_ID       = -1,
    NI_SCALE_ID         = -2,
    NI_PAD_ID           = -3,
    NI_CROP_ID          = -4,
    NI_OVERLAY_ID       = -5,
    NI_ROI_ID           = -6,
    NI_BG_ID            = -7,
    NI_STACK_ID         = -8,
    NI_ROTATE_ID        = -9,
    NI_DRAWBOX_ID       = -10,
    NI_BGR_ID           = -11,
    NI_DRAWTEXT_ID      = -12,
    NI_AI_PREPROCESS_ID = -13,
    NI_DELOGO_ID        = -14,
    NI_MERGE_ID         = -15,
    NI_FLIP_ID          = -16,
    NI_HVSPLUS_ID       = -17,
    NI_AI_ALIGN_ID      = -18,
} ni_filter_poolsize_code;

/**
* This struct is allocated as AVHWDeviceContext.hwctx
*/
typedef struct AVNIDeviceContext {
    int uploader_ID;
    ni_device_handle_t uploader_handle;

    ni_device_handle_t cards[NI_MAX_DEVICE_CNT];
} AVNIDeviceContext;

/**
* This struct is allocated as AVHWFramesContext.hwctx
*/
typedef struct AVNIFramesContext {
    niFrameSurface1_t *surfaces;
    int               nb_surfaces;
    int               keep_alive_timeout;
    int               frame_type;
    AVRational        framerate;                  /* used for modelling hwupload */
    int               hw_id;
    ni_session_context_t api_ctx; // for down/uploading frames
    ni_split_context_t   split_ctx;
    ni_device_handle_t   suspended_device_handle;
    int                  uploader_device_id; // same one passed to libxcoder session open

    // Accessed only by hwcontext_ni_quad.c
    niFrameSurface1_t    *surfaces_internal;
    int                  nb_surfaces_used;
    niFrameSurface1_t    **surface_ptrs;
    ni_session_data_io_t src_session_io_data; // for upload frame to be sent up
} AVNIFramesContext;

static inline int ni_get_cardno(const AVFrame *frame) {
    AVNIFramesContext* ni_hwf_ctx;
    ni_hwf_ctx = (AVNIFramesContext*)((AVHWFramesContext*)frame->hw_frames_ctx->data)->hwctx;
    return ni_hwf_ctx->hw_id;
}

// copy hwctx specific data from one AVHWFramesContext to another
static inline void ni_cpy_hwframe_ctx(AVHWFramesContext *in_frames_ctx,
                                      AVHWFramesContext *out_frames_ctx)
{
    memcpy(out_frames_ctx->hwctx, in_frames_ctx->hwctx, sizeof(AVNIFramesContext));
}

#endif /* AVUTIL_HWCONTEXT_NI_H */
