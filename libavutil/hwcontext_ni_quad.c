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

#include "config.h"

#include <fcntl.h>
#include <string.h>
#if HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "avassert.h"
#include "buffer.h"
#include "common.h"
#include "hwcontext.h"
#include "hwcontext_internal.h"
#include "hwcontext_ni_quad.h"
#include "libavutil/imgutils.h"
#include "mem.h"
#include "pixdesc.h"
#include "pixfmt.h"

static enum AVPixelFormat supported_pixel_formats[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUYV422, AV_PIX_FMT_UYVY422,
    AV_PIX_FMT_NV12,    AV_PIX_FMT_ARGB,    AV_PIX_FMT_RGBA,
    AV_PIX_FMT_ABGR,    AV_PIX_FMT_BGRA,    AV_PIX_FMT_YUV420P10LE,
    AV_PIX_FMT_NV16,    AV_PIX_FMT_BGR0,    AV_PIX_FMT_P010LE,
    AV_PIX_FMT_BGRP
};

static inline void ni_frame_free(void *opaque, uint8_t *data)
{
    if (data) {
        niFrameSurface1_t* p_data3 = (niFrameSurface1_t*)data;
        if (p_data3->ui16FrameIdx != 0) {
            ni_hwframe_buffer_recycle(p_data3, p_data3->device_handle);
        }
        ni_aligned_free(p_data3);
    }
}

static int ni_device_create(AVHWDeviceContext *ctx, const char *device,
                            AVDictionary *opts, int flags)
{
    AVNIDeviceContext *ni_hw_ctx;
    char *blk_name;
    int i, module_id = 0, ret = 0;
    ni_device_handle_t fd;
    uint32_t max_io_size = NI_INVALID_IO_SIZE;
    ni_device_t *p_ni_devices = NULL;

    p_ni_devices = av_calloc(1, sizeof(ni_device_t));
    if(p_ni_devices == NULL) {
        av_log(ctx, AV_LOG_ERROR, "could not allocate memory for p_ni_devices in %s", __func__);
        return AVERROR_UNKNOWN;
    }

    ni_hw_ctx = (AVNIDeviceContext *)ctx->hwctx;
    ni_hw_ctx->uploader_handle = NI_INVALID_DEVICE_HANDLE;
    ni_hw_ctx->uploader_ID = -2; // -1 is load balance by pixel rate,
                                 // default -2 invalid

    if (device) {
        /* parse device string and fail if incorrect */
        av_log(ctx, AV_LOG_VERBOSE, "%s %s\n", __func__, device);
        ni_hw_ctx->uploader_ID = atoi(device);
        av_log(ctx, AV_LOG_DEBUG, "%s: given uploader ID %d\n", __func__,
               ni_hw_ctx->uploader_ID);
        if (ni_hw_ctx->uploader_ID < -1) {
            av_log(ctx, AV_LOG_ERROR, "%s: uploader ID %d must be >= -1.\n",
                   __func__, ni_hw_ctx->uploader_ID);
            ret =  AVERROR_UNKNOWN;
            LRETURN;
        }
    }

    for (i = 0; i < NI_MAX_DEVICE_CNT; i++) {
        ni_hw_ctx->cards[i] = NI_INVALID_DEVICE_HANDLE;
    }

    /* Scan all cards on the host, only look at NETINT cards */
    if (ni_rsrc_list_all_devices(p_ni_devices) == NI_RETCODE_SUCCESS) {
        // Note: this only checks for Netint encoders
        for (i = 0; i < p_ni_devices->xcoder_cnt[NI_DEVICE_TYPE_ENCODER]; i++) {
            blk_name =
                &(p_ni_devices->xcoders[NI_DEVICE_TYPE_ENCODER][i].blk_name[0]);
            // cone-to-one correspondence between card index and module_id
            module_id = p_ni_devices->xcoders[NI_DEVICE_TYPE_ENCODER][i].module_id;
            av_log(ctx, AV_LOG_DEBUG, "%s blk name %s\n", __func__, blk_name);
            fd = ni_device_open(blk_name, &max_io_size);
            if (fd != NI_INVALID_DEVICE_HANDLE) {
                ni_hw_ctx->cards[module_id] = fd;
            }
        }
    } else {
        ret = AVERROR_UNKNOWN;
    }
END:
    av_freep(&p_ni_devices);
    return ret;
}

static void ni_device_uninit(AVHWDeviceContext *ctx)
{
    AVNIDeviceContext *ni_hw_ctx;
    int i;

    ni_hw_ctx = (AVNIDeviceContext *)ctx->hwctx;

    av_log(ctx, AV_LOG_VERBOSE, "%s\n", __func__);

    if (ni_hw_ctx->uploader_handle != NI_INVALID_DEVICE_HANDLE) {
        ni_device_close(ni_hw_ctx->uploader_handle);
        ni_hw_ctx->uploader_handle = NI_INVALID_DEVICE_HANDLE;
    }

    for (i = 0; i < NI_MAX_DEVICE_CNT; i++) {
        ni_device_handle_t fd = ni_hw_ctx->cards[i];
        if (fd != NI_INVALID_DEVICE_HANDLE) {
            ni_hw_ctx->cards[i] = NI_INVALID_DEVICE_HANDLE;
            ni_device_close(fd);
        } else {
            break;
        }
    }

    return;
}

static int ni_frames_get_constraints(AVHWDeviceContext *ctx,
                                     const void *hwconfig,
                                     AVHWFramesConstraints *constraints)
{
    int i;
    int num_pix_fmts_supported;

    num_pix_fmts_supported = FF_ARRAY_ELEMS(supported_pixel_formats);

    constraints->valid_sw_formats = av_malloc_array(num_pix_fmts_supported + 1,
                                                    sizeof(*constraints->valid_sw_formats));
    if (!constraints->valid_sw_formats) {
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < num_pix_fmts_supported; i++) {
        constraints->valid_sw_formats[i] = supported_pixel_formats[i];
    }
    constraints->valid_sw_formats[num_pix_fmts_supported] = AV_PIX_FMT_NONE;

    constraints->valid_hw_formats = av_malloc_array(2, sizeof(*constraints->valid_hw_formats));
    if (!constraints->valid_hw_formats) {
        return AVERROR(ENOMEM);
    }

    constraints->valid_hw_formats[0] = AV_PIX_FMT_NI_QUAD;
    constraints->valid_hw_formats[1] = AV_PIX_FMT_NONE;

    return 0;
}

static int ni_get_buffer(AVHWFramesContext *ctx, AVFrame *frame)
{
    int ret = 0;
    uint8_t *buf;
    uint32_t buf_size;
    ni_frame_t *xfme;
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) ctx->hwctx;
    ni_session_data_io_t dst_session_io_data;
    ni_session_data_io_t * p_dst_session_data = &dst_session_io_data;
    bool isnv12frame = (ctx->sw_format == AV_PIX_FMT_NV12 ||
                        ctx->sw_format == AV_PIX_FMT_P010LE);

    av_log(ctx, AV_LOG_TRACE, "hwcontext_ni.c:ni_get_buffer()\n");

    // alloc dest avframe buff
    memset(p_dst_session_data, 0, sizeof(dst_session_io_data));
    ret = ni_frame_buffer_alloc(&p_dst_session_data->data.frame, ctx->width,
                                ctx->height, 0, 1, // codec type does not matter, metadata exists
                                f_hwctx->api_ctx.bit_depth_factor, 1, !isnv12frame);
    if (ret != 0) {
        return AVERROR(ENOMEM);
    }

    xfme = &p_dst_session_data->data.frame;
    buf_size = xfme->data_len[0] + xfme->data_len[1] +
               xfme->data_len[2] + xfme->data_len[3];
    buf = xfme->p_data[0];
    memset(buf, 0, buf_size);
    frame->buf[0] = av_buffer_create(buf, buf_size, ni_frame_free, NULL, 0);
    if (!frame->buf[0]) {
        return AVERROR(ENOMEM);
    }
    frame->data[3] = xfme->p_buffer + xfme->data_len[0] + xfme->data_len[1] +
                     xfme->data_len[2];
    frame->format = AV_PIX_FMT_NI_QUAD;
    frame->width = ctx->width;
    frame->height = ctx->height;

    return 0;
}

static int ni_transfer_get_formats(AVHWFramesContext *ctx,
                                   enum AVHWFrameTransferDirection dir,
                                   enum AVPixelFormat **formats)
{
    enum AVPixelFormat *fmts;

    fmts = av_malloc_array(2, sizeof(*fmts));
    if (!fmts) {
        return AVERROR(ENOMEM);
    }

    fmts[0] = ctx->sw_format;
    fmts[1] = AV_PIX_FMT_NONE;

    *formats = fmts;

    return 0;
}

static void ni_frames_uninit(AVHWFramesContext *ctx)
{
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) ctx->hwctx;
    int dev_dec_idx = f_hwctx->uploader_device_id; //Supplied by init_hw_device ni=<name>:<id> or ni_hwupload=<id>

    av_log(ctx, AV_LOG_DEBUG, "%s: only close if upload instance, poolsize=%d "
                              "devid=%d\n",
                              __func__, ctx->initial_pool_size, dev_dec_idx);

    if (dev_dec_idx != -2 && ctx->initial_pool_size >= 0) {
        if (f_hwctx->src_session_io_data.data.frame.buffer_size
            || f_hwctx->src_session_io_data.data.frame.metadata_buffer_size
            || f_hwctx->src_session_io_data.data.frame.start_buffer_size) {
            av_log(ctx, AV_LOG_DEBUG, "%s:free upload src frame buffer\n",
                 __func__);
            ni_frame_buffer_free(&f_hwctx->src_session_io_data.data.frame);
        }
        av_log(ctx, AV_LOG_VERBOSE, "SessionID = %d!\n", f_hwctx->api_ctx.session_id);
        if (f_hwctx->api_ctx.session_id != NI_INVALID_SESSION_ID) {
            ni_device_session_close(&f_hwctx->api_ctx, 1, NI_DEVICE_TYPE_UPLOAD);
        }
        ni_device_session_context_clear(&f_hwctx->api_ctx);

        //only upload frames init allocates these ones
        av_freep(&f_hwctx->surface_ptrs);
        av_freep(&f_hwctx->surfaces_internal);
    } else {
        ni_device_session_context_clear(&f_hwctx->api_ctx);
    }

    if (f_hwctx->suspended_device_handle != NI_INVALID_DEVICE_HANDLE) {
        av_log(ctx, AV_LOG_DEBUG, "%s: close file handle, =%d\n",
               __func__, f_hwctx->suspended_device_handle);
        ni_device_close(f_hwctx->suspended_device_handle);
        f_hwctx->suspended_device_handle = NI_INVALID_DEVICE_HANDLE;
    }
}

static AVBufferRef *ni_pool_alloc(void *opaque,
#if LIBAVUTIL_VERSION_MAJOR >= 58 || LIBAVUTIL_VERSION_MAJOR >= 57 && LIBAVUTIL_VERSION_MINOR >= 17
                                  size_t size)
#else
                                  int size)
#endif
{
    AVHWFramesContext *ctx = (AVHWFramesContext*)opaque;
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) ctx->hwctx;

    if (f_hwctx->nb_surfaces_used < f_hwctx->nb_surfaces) {
        f_hwctx->nb_surfaces_used++;
        return av_buffer_create((uint8_t*) (f_hwctx->surfaces_internal + f_hwctx->nb_surfaces_used - 1),
                                sizeof(*f_hwctx->surfaces), NULL, NULL, 0);
    }

    return NULL;
}

static int ni_init_surface(AVHWFramesContext *ctx, niFrameSurface1_t *surf)
{
    /* Fill with dummy values. This data is never used. */
    surf->ui16FrameIdx    = 0;
    surf->ui16session_ID  = 0;
    surf->ui32nodeAddress = 0;
    surf->device_handle   = 0;
    surf->bit_depth       = 0;
    surf->encoding_type   = 0;
    surf->output_idx      = 0;
    surf->src_cpu         = 0;

    return 0;
}

static int ni_init_pool(AVHWFramesContext *ctx)
{
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) ctx->hwctx;
    int i, ret;

    av_log(ctx, AV_LOG_VERBOSE, "ctx->initial_pool_size = %d\n", ctx->initial_pool_size);

    if (ctx->initial_pool_size <= 0) {
        av_log(ctx, AV_LOG_ERROR, "NI requires a fixed frame pool size\n");
        return AVERROR(EINVAL);
    }

    f_hwctx->surfaces_internal = av_calloc(ctx->initial_pool_size,
                                           sizeof(*f_hwctx->surfaces_internal));
    if (!f_hwctx->surfaces_internal) {
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < ctx->initial_pool_size; i++) {
        ret = ni_init_surface(ctx, &f_hwctx->surfaces_internal[i]);
        if (ret < 0) {
            return ret;
        }
    }

#if IS_FFMPEG_70_AND_ABOVE_FOR_LIBAVUTIL
    ffhwframesctx(ctx)->pool_internal =
        av_buffer_pool_init2(sizeof(niFrameSurface1_t), ctx, ni_pool_alloc, NULL);
    if (!ffhwframesctx(ctx)->pool_internal) {
#else
    ctx->internal->pool_internal =
        av_buffer_pool_init2(sizeof(niFrameSurface1_t), ctx, ni_pool_alloc, NULL);
    if (!ctx->internal->pool_internal) {
#endif
        return AVERROR(ENOMEM);
    }

    f_hwctx->surfaces = f_hwctx->surfaces_internal;
    f_hwctx->nb_surfaces = ctx->initial_pool_size;

    return 0;
}

static int ni_init_internal_session(AVHWFramesContext *ctx)
{
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) ctx->hwctx;
    ni_log_set_level(ff_to_ni_log_level(av_log_get_level()));
    av_log(ctx, AV_LOG_INFO, "hwcontext_ni:ni_init_internal_session()\n");
    if (ni_device_session_context_init(&(f_hwctx->api_ctx)) < 0) {
        av_log(ctx, AV_LOG_ERROR, "ni init context failure\n");
        return -1;
    }

    return 0;
}

static void init_split_rsrc(AVNIFramesContext *f_hwctx, int w, int h)
{
    int i;
    ni_split_context_t* p_split_ctx = &f_hwctx->split_ctx;
    memset(p_split_ctx, 0, sizeof(ni_split_context_t));
    for (i = 0; i < 3; i++) {
        p_split_ctx->w[i] = w;
        p_split_ctx->h[i] = h;
        p_split_ctx->f[i] = -1;
    }
}

static int ni_frames_init(AVHWFramesContext *ctx) //hwupload runs this on hwupload_config_output
{
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) ctx->hwctx;
    AVNIDeviceContext *device_hwctx = (AVNIDeviceContext*) ctx->device_ctx->hwctx;
    int linesize_aligned,height_aligned;
    int pool_size,ret;

    av_log(ctx, AV_LOG_INFO, "%s: Enter, supplied poolsize = %d, devid=%d\n",
           __func__, ctx->initial_pool_size, device_hwctx->uploader_ID);

    f_hwctx->suspended_device_handle = NI_INVALID_DEVICE_HANDLE;
    f_hwctx->uploader_device_id = -2; // -1 is load balance by pixel rate,
                                      // default -2 invalid
    pool_size = ctx->initial_pool_size;
    if (device_hwctx->uploader_ID < -1) {
        if (pool_size > -1) { // ffmpeg does not specify init_hw_device for decoder
                              // - so decoder device_hwctx->uploader_ID is always -1
            av_log(ctx, AV_LOG_INFO, "%s no uploader device selected!\n",
                   __func__);
            return AVERROR(EINVAL);
        }
    }

    ret = ni_init_internal_session(ctx);
    if (ret < 0) {
        return AVERROR(EINVAL);
    }

    init_split_rsrc(f_hwctx, ctx->width, ctx->height);
    if (pool_size <= -1) { // None upload init returns here
        av_log(ctx, AV_LOG_INFO, "%s: poolsize code %d, this code recquires no host pool\n",
               __func__, pool_size);
        return ret;
    } else if (pool_size == 0) {
        pool_size = ctx->initial_pool_size = 3;
        av_log(ctx, AV_LOG_INFO, "%s: Pool_size autoset to %d\n", __func__, pool_size);
    }

    /*Kept in AVNIFramesContext for future reference, the AVNIDeviceContext data member gets overwritten*/
    f_hwctx->uploader_device_id = device_hwctx->uploader_ID;

    if ((ctx->width & 0x1) || (ctx->height & 0x1)) {
        av_log(ctx, AV_LOG_ERROR, "Odd resolution %dx%d not permitted\n",
               ctx->width, ctx->height);
        return AVERROR(EINVAL);
    }

    linesize_aligned = NI_VPU_CEIL(ctx->width, 2);
    ctx->width = linesize_aligned;

    height_aligned = ctx->height;
    ctx->height = NI_VPU_CEIL(height_aligned, 2);

    f_hwctx->api_ctx.active_video_width = ctx->width;
    f_hwctx->api_ctx.active_video_height = ctx->height;

    switch (ctx->sw_format) {
        case AV_PIX_FMT_YUV420P:
            f_hwctx->api_ctx.bit_depth_factor = 1;
            f_hwctx->api_ctx.src_bit_depth = 8;
            f_hwctx->api_ctx.pixel_format = NI_PIX_FMT_YUV420P;
            break;
        case AV_PIX_FMT_YUV420P10LE:
            f_hwctx->api_ctx.bit_depth_factor = 2;
            f_hwctx->api_ctx.src_bit_depth = 10;
            f_hwctx->api_ctx.src_endian = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format = NI_PIX_FMT_YUV420P10LE;
            break;
        case AV_PIX_FMT_NV12:
            f_hwctx->api_ctx.bit_depth_factor = 1;
            f_hwctx->api_ctx.src_bit_depth = 8;
            f_hwctx->api_ctx.pixel_format = NI_PIX_FMT_NV12;
            break;
        case AV_PIX_FMT_P010LE:
            f_hwctx->api_ctx.bit_depth_factor = 2;
            f_hwctx->api_ctx.src_bit_depth = 10;
            f_hwctx->api_ctx.pixel_format = NI_PIX_FMT_P010LE;
            f_hwctx->api_ctx.src_endian = NI_FRAME_LITTLE_ENDIAN;
            break;
        case AV_PIX_FMT_RGBA:
            f_hwctx->api_ctx.bit_depth_factor = 4;
            f_hwctx->api_ctx.src_bit_depth    = 32;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_RGBA;
            break;
        case AV_PIX_FMT_BGRA:
            f_hwctx->api_ctx.bit_depth_factor = 4;
            f_hwctx->api_ctx.src_bit_depth    = 32;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_BGRA;
            break;
        case AV_PIX_FMT_ABGR:
            f_hwctx->api_ctx.bit_depth_factor = 4;
            f_hwctx->api_ctx.src_bit_depth    = 32;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_ABGR;
            break;
        case AV_PIX_FMT_ARGB:
            f_hwctx->api_ctx.bit_depth_factor = 4;
            f_hwctx->api_ctx.src_bit_depth    = 32;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_ARGB;
            break;
        case AV_PIX_FMT_BGR0:
            f_hwctx->api_ctx.bit_depth_factor = 4;
            f_hwctx->api_ctx.src_bit_depth    = 32;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_BGR0;
            break;
        case AV_PIX_FMT_BGRP:
            f_hwctx->api_ctx.bit_depth_factor = 1;
            f_hwctx->api_ctx.src_bit_depth    = 24;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_BGRP;
            break;
        case AV_PIX_FMT_YUYV422:
            f_hwctx->api_ctx.bit_depth_factor = 1;
            f_hwctx->api_ctx.src_bit_depth    = 8;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_YUYV422;
            break;
        case AV_PIX_FMT_UYVY422:
            f_hwctx->api_ctx.bit_depth_factor = 1;
            f_hwctx->api_ctx.src_bit_depth    = 8;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_UYVY422;
            break;
        case AV_PIX_FMT_NV16:
            f_hwctx->api_ctx.bit_depth_factor = 1;
            f_hwctx->api_ctx.src_bit_depth    = 8;
            f_hwctx->api_ctx.src_endian       = NI_FRAME_LITTLE_ENDIAN;
            f_hwctx->api_ctx.pixel_format     = NI_PIX_FMT_NV16;
            break;
        default:
            av_log(ctx, AV_LOG_ERROR, "Pixel format %s not supported by device.\n", av_get_pix_fmt_name(ctx->sw_format));
            return AVERROR(EINVAL);
    }

    if (ctx->width > NI_MAX_RESOLUTION_WIDTH ||
        ctx->height > NI_MAX_RESOLUTION_HEIGHT ||
        ctx->width * ctx->height > NI_MAX_RESOLUTION_AREA) {
        av_log(ctx, AV_LOG_ERROR, "Error XCoder resolution %dx%d not supported\n",
               ctx->width, ctx->height);
        av_log(ctx, AV_LOG_ERROR, "Max Supported Width: %d Height %d Area %d\n",
               NI_MAX_RESOLUTION_WIDTH, NI_MAX_RESOLUTION_HEIGHT, NI_MAX_RESOLUTION_AREA);
        return AVERROR_EXTERNAL;
    } else if (f_hwctx->uploader_device_id >= -1) {
        // leave it to ni_device_session_open to handle uploader session open
        // based on api_ctx.hw_id set to proper value
    } else {
        av_log(ctx, AV_LOG_ERROR, "Error XCoder command line options");
        return AVERROR(EINVAL);
    }

    av_log(ctx, AV_LOG_VERBOSE,
           "pixel sw_format=%d width = %d height = %d outformat=%d "
           "uploader_device_id=%d\n",
           ctx->sw_format, ctx->width, ctx->height, ctx->format,
           f_hwctx->uploader_device_id);

    f_hwctx->api_ctx.hw_id = f_hwctx->uploader_device_id;
    f_hwctx->api_ctx.keep_alive_timeout = f_hwctx->keep_alive_timeout;
    if (0 == f_hwctx->api_ctx.keep_alive_timeout) {
        f_hwctx->api_ctx.keep_alive_timeout = NI_DEFAULT_KEEP_ALIVE_TIMEOUT;
    }

    f_hwctx->api_ctx.framerate.framerate_num = f_hwctx->framerate.num;
    f_hwctx->api_ctx.framerate.framerate_denom = f_hwctx->framerate.den;

    ret = ni_device_session_open(&f_hwctx->api_ctx, NI_DEVICE_TYPE_UPLOAD);
    if (ret != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Error Something wrong in xcoder open\n");
        ni_frames_uninit(ctx);
        return AVERROR_EXTERNAL;
    } else {
        av_log(ctx, AV_LOG_VERBOSE,
               "XCoder %s.%d (inst: %d) opened successfully\n",
               f_hwctx->api_ctx.dev_xcoder_name, f_hwctx->api_ctx.hw_id,
               f_hwctx->api_ctx.session_id);
#ifndef _WIN32
        // replace device_handle with blk_io_handle
        ni_device_close(f_hwctx->api_ctx.device_handle);
        f_hwctx->api_ctx.device_handle = f_hwctx->api_ctx.blk_io_handle;
#endif
        // save blk_io_handle for track
        device_hwctx->uploader_handle = f_hwctx->api_ctx.blk_io_handle;
    }
    memset(&f_hwctx->src_session_io_data, 0, sizeof(ni_session_data_io_t));

    // enable buffer_limit for FFmpeg>=7.0 by default
#if IS_FFMPEG_61_AND_ABOVE_FOR_LIBAVUTIL
    ret = ni_device_session_init_framepool(&f_hwctx->api_ctx, pool_size, NI_UPLOADER_FLAG_LM);
#else
    ret = ni_device_session_init_framepool(&f_hwctx->api_ctx, pool_size, 0);
#endif

    if (ret < 0) {
        return ret;
    }

    if (!ctx->pool) {
        ret = ni_init_pool(ctx);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error creating an internal frame pool\n");
            return ret;
        }
    }
    return 0;
}

static int ni_to_avframe_copy(AVHWFramesContext *hwfc, AVFrame *dst,
                              const ni_frame_t *src)
{
    int src_linesize[4], src_height[4];
    int i, h, nb_planes;
    uint8_t *src_line, *dst_line;

    nb_planes = av_pix_fmt_count_planes(hwfc->sw_format);

    switch (hwfc->sw_format) {
    case AV_PIX_FMT_YUV420P:
        src_linesize[0] = FFALIGN(dst->width, 128);
        src_linesize[1] = FFALIGN(dst->width / 2, 128);
        src_linesize[2] = src_linesize[1];
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = FFALIGN(dst->height, 2) / 2;
        src_height[2] = src_height[1];
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_YUV420P10LE:
        src_linesize[0] = FFALIGN(dst->width * 2, 128);
        src_linesize[1] = FFALIGN(dst->width, 128);
        src_linesize[2] = src_linesize[1];
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = FFALIGN(dst->height, 2) / 2;
        src_height[2] = src_height[1];
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_NV12:
        src_linesize[0] = FFALIGN(dst->width, 128);
        src_linesize[1] = FFALIGN(dst->width, 128);
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = FFALIGN(dst->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_NV16:
        src_linesize[0] = FFALIGN(dst->width, 64);
        src_linesize[1] = FFALIGN(dst->width, 64);
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = dst->height;
        src_height[2] = 0;
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_YUYV422:
    case AV_PIX_FMT_UYVY422:
        src_linesize[0] = FFALIGN(dst->width, 16) * 2;
        src_linesize[1] = 0;
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_P010LE:
        src_linesize[0] = FFALIGN(dst->width * 2, 128);
        src_linesize[1] = FFALIGN(dst->width * 2, 128);
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = FFALIGN(dst->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_RGBA:
    case AV_PIX_FMT_BGRA:
    case AV_PIX_FMT_ABGR:
    case AV_PIX_FMT_ARGB:
    case AV_PIX_FMT_BGR0:
        src_linesize[0] = FFALIGN(dst->width, 16) * 4;
        src_linesize[1] = 0;
        src_linesize[2] = 0;
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;
        break;

    case AV_PIX_FMT_BGRP:
        src_linesize[0] = FFALIGN(dst->width, 32);
        src_linesize[1] = FFALIGN(dst->width, 32);
        src_linesize[2] = FFALIGN(dst->width, 32);
        src_linesize[3] = 0;

        src_height[0] = dst->height;
        src_height[1] = dst->height;
        src_height[2] = dst->height;
        src_height[3] = 0;
        break;

    default:
        av_log(hwfc, AV_LOG_ERROR, "Unsupported pixel format %s\n",
               av_get_pix_fmt_name(hwfc->sw_format));
        return AVERROR(EINVAL);
    }

    for (i = 0; i < nb_planes; i++) {
        dst_line = dst->data[i];
        src_line = src->p_data[i];

        for (h = 0; h < src_height[i]; h++) {
            memcpy(dst_line, src_line,
                   FFMIN(src_linesize[i], dst->linesize[i]));
            dst_line += dst->linesize[i];
            src_line += src_linesize[i];
        }
    }

    return 0;
}

static int av_to_niframe_copy(AVHWFramesContext *hwfc, const int dst_stride[4],
                              ni_frame_t *dst, const AVFrame *src) {
    int src_height[4], hpad[4], vpad[4];
    int i, j, h, nb_planes;
    uint8_t *src_line, *dst_line, YUVsample, *sample, *dest;
    uint16_t lastidx;
    bool tenBit;

    nb_planes = av_pix_fmt_count_planes(hwfc->sw_format);

    switch (src->format) {
    case AV_PIX_FMT_YUV420P:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - src->linesize[1], 0);
        hpad[2] = FFMAX(dst_stride[2] - src->linesize[2], 0);
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = FFALIGN(src->height, 2) / 2;
        src_height[3] = 0;

        vpad[0] = FFALIGN(src_height[0], 2) - src_height[0];
        vpad[1] = FFALIGN(src_height[1], 2) - src_height[1];
        vpad[2] = FFALIGN(src_height[2], 2) - src_height[2];
        vpad[3] = 0;

        tenBit = false;
        break;

    case AV_PIX_FMT_YUV420P10LE:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - src->linesize[1], 0);
        hpad[2] = FFMAX(dst_stride[2] - src->linesize[2], 0);
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = FFALIGN(src->height, 2) / 2;
        src_height[3] = 0;

        vpad[0] = FFALIGN(src_height[0], 2) - src_height[0];
        vpad[1] = FFALIGN(src_height[1], 2) - src_height[1];
        vpad[2] = FFALIGN(src_height[2], 2) - src_height[2];
        vpad[3] = 0;

        tenBit = true;
        break;

    case AV_PIX_FMT_NV12:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - src->linesize[1], 0);
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = FFALIGN(src_height[0], 2) - src_height[0];
        vpad[1] = FFALIGN(src_height[1], 2) - src_height[1];
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;
    case AV_PIX_FMT_NV16:
        hpad[0] = 0;
        hpad[1] = 0;
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = src->height;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;

    case AV_PIX_FMT_P010LE:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - src->linesize[1], 0);
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = FFALIGN(src->height, 2) / 2;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = FFALIGN(src_height[0], 2) - src_height[0];
        vpad[1] = FFALIGN(src_height[1], 2) - src_height[1];
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = true;
        break;

    case AV_PIX_FMT_RGBA:
    case AV_PIX_FMT_BGRA:
    case AV_PIX_FMT_ABGR:
    case AV_PIX_FMT_ARGB:
    case AV_PIX_FMT_BGR0:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = 0;
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;

    case AV_PIX_FMT_BGRP:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = FFMAX(dst_stride[1] - src->linesize[1], 0);
        hpad[2] = FFMAX(dst_stride[2] - src->linesize[2], 0);
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = src->height;
        src_height[2] = src->height;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;

    case AV_PIX_FMT_YUYV422:
    case AV_PIX_FMT_UYVY422:
        hpad[0] = FFMAX(dst_stride[0] - src->linesize[0], 0);
        hpad[1] = 0;
        hpad[2] = 0;
        hpad[3] = 0;

        src_height[0] = src->height;
        src_height[1] = 0;
        src_height[2] = 0;
        src_height[3] = 0;

        vpad[0] = 0;
        vpad[1] = 0;
        vpad[2] = 0;
        vpad[3] = 0;

        tenBit = false;
        break;

    default:
        av_log(hwfc, AV_LOG_ERROR, "Pixel format %s not supported\n",
               av_get_pix_fmt_name(src->format));
        break;
    }

    for (i = 0; i < nb_planes; i++) {
        dst_line = dst->p_data[i];
        src_line = src->data[i];

        for (h = 0; h < src_height[i]; h++) {
            memcpy(dst_line, src_line, FFMIN(src->linesize[i], dst_stride[i]));

            if (hpad[i]) {
                lastidx = src->linesize[i];

                if (tenBit) {
                    sample = &src_line[lastidx - 2];
                    dest   = &dst_line[lastidx];

                    /* two bytes per sample */
                    for (j = 0; j < hpad[i] / 2; j++) {
                        memcpy(dest, sample, 2);
                        dest += 2;
                    }
                } else {
                    YUVsample = dst_line[lastidx - 1];
                    memset(&dst_line[lastidx], YUVsample, hpad[i]);
                }
            }

            src_line += src->linesize[i];
            dst_line += dst_stride[i];
        }

        /* Extend the height by cloning the last line */
        src_line = dst_line - dst_stride[i];
        for (h = 0; h < vpad[i]; h++) {
            memcpy(dst_line, src_line, dst_stride[i]);
            dst_line += dst_stride[i];
        }
    }

    return 0;
}

static int ni_hwdl_frame(AVHWFramesContext *hwfc, AVFrame *dst,
                         const AVFrame *src)
{
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) hwfc->hwctx;
    ni_session_data_io_t session_io_data;
    ni_session_data_io_t *p_session_data = &session_io_data;
    niFrameSurface1_t *src_surf = (niFrameSurface1_t *)src->data[3];
    int ret;
    int pixel_format;

    memset(&session_io_data, 0, sizeof(ni_session_data_io_t));

    av_log(hwfc, AV_LOG_VERBOSE,
           "%s handle %d trace ui16FrameIdx = [%d] SID %d\n", __func__,
           src_surf->device_handle, src_surf->ui16FrameIdx,
           src_surf->ui16session_ID);

    av_log(hwfc, AV_LOG_DEBUG, "%s hwdl processed h/w = %d/%d\n", __func__,
           src->height, src->width);

    switch (hwfc->sw_format) {
    case AV_PIX_FMT_YUV420P:
        pixel_format = NI_PIX_FMT_YUV420P;
        break;
    case AV_PIX_FMT_YUV420P10LE:
        pixel_format = NI_PIX_FMT_YUV420P10LE;
        break;
    case AV_PIX_FMT_NV12:
        pixel_format = NI_PIX_FMT_NV12;
        break;
    case AV_PIX_FMT_NV16:
        pixel_format = NI_PIX_FMT_NV16;
        break;
    case AV_PIX_FMT_YUYV422:
        pixel_format = NI_PIX_FMT_YUYV422;
        break;
    case AV_PIX_FMT_UYVY422:
        pixel_format = NI_PIX_FMT_UYVY422;
        break;
    case AV_PIX_FMT_P010LE:
        pixel_format = NI_PIX_FMT_P010LE;
        break;
    case AV_PIX_FMT_RGBA:
        pixel_format = NI_PIX_FMT_RGBA;
        break;
    case AV_PIX_FMT_BGRA:
        pixel_format = NI_PIX_FMT_BGRA;
        break;
    case AV_PIX_FMT_ABGR:
        pixel_format = NI_PIX_FMT_ABGR;
        break;
    case AV_PIX_FMT_ARGB:
        pixel_format = NI_PIX_FMT_ARGB;
        break;
    case AV_PIX_FMT_BGR0:
        pixel_format = NI_PIX_FMT_BGR0;
        break;
    case AV_PIX_FMT_BGRP:
        pixel_format = NI_PIX_FMT_BGRP;
        break;
    default:
        av_log(hwfc, AV_LOG_ERROR, "Pixel format %s not supported\n",
               av_get_pix_fmt_name(hwfc->sw_format));
        return AVERROR(EINVAL);
    }

    ret = ni_frame_buffer_alloc_dl(&(p_session_data->data.frame), src->width,
                                   src->height, pixel_format);
    if (ret != NI_RETCODE_SUCCESS) {
        av_log(hwfc, AV_LOG_ERROR, "%s Cannot allocate ni_frame\n", __func__);
        return AVERROR(ENOMEM);
    }

    f_hwctx->api_ctx.is_auto_dl = false;
    ret = ni_device_session_hwdl(&f_hwctx->api_ctx, p_session_data, src_surf);
    if (ret <= 0) {
        av_log(hwfc, AV_LOG_DEBUG, "%s failed to retrieve frame\n", __func__);
        ni_frame_buffer_free(&p_session_data->data.frame);
        return AVERROR_EXTERNAL;
    }

    ret = ni_to_avframe_copy(hwfc, dst, &p_session_data->data.frame);
    if (ret < 0) {
        av_log(hwfc, AV_LOG_ERROR, "Can't copy frame %d\n", ret);
        ni_frame_buffer_free(&p_session_data->data.frame);
        return ret;
    }

    dst->format = hwfc->sw_format;

    av_frame_copy_props(dst, src);
    ni_frame_buffer_free(&p_session_data->data.frame);

    return 0;
}

static void ff_ni_set_bit_depth_and_encoding_type(int8_t *p_bit_depth,
                                           int8_t *p_enc_type,
                                           enum AVPixelFormat pix_fmt)
{
    // bit depth is 1 for 8-bit format, 2 for 10-bit format
    // encoding type should be 1 for planar or packed, 0 for semi-planar
    switch (pix_fmt)
    {
        case AV_PIX_FMT_YUV420P:
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type  = 1;   // planar
            break;

        case AV_PIX_FMT_YUV420P10LE:
            *p_bit_depth = 2;   // 10-bits per component
            *p_enc_type  = 1;   // planar
            break;

        case AV_PIX_FMT_NV12:
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type  = 0;   // semi-planar
            break;
        case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type = NI_PIXEL_PLANAR_FORMAT_TILED4X4;   // semi-planar
            break;
        case AV_PIX_FMT_P010LE:
            *p_bit_depth = 2;   // 10-bits per component
            *p_enc_type  = 0;   // semi-planar
            break;

        case AV_PIX_FMT_YUYV422:
        case AV_PIX_FMT_UYVY422:
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type  = 1;   // packed
            break;

        case AV_PIX_FMT_NV16:
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type  = 0;   // semi-planar
            break;

        case AV_PIX_FMT_BGRP:
        case AV_PIX_FMT_RGBA:
        case AV_PIX_FMT_BGRA:
        case AV_PIX_FMT_ABGR:
        case AV_PIX_FMT_ARGB:
        case AV_PIX_FMT_BGR0:
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type  = 1;   // packed or planar
            break;

        default:
            av_log(NULL, AV_LOG_WARNING, "WARNING: unexpected pix format %s\n",
                   av_get_pix_fmt_name(pix_fmt));

            // use default values if we've supported a new pixel format
            *p_bit_depth = 1;   // 8-bits per component
            *p_enc_type  = 1;   // planar or packed
            break;
    }
}

static int ni_hwup_frame(AVHWFramesContext *hwfc, AVFrame *dst, const AVFrame *src)
{
    AVNIFramesContext *f_hwctx = (AVNIFramesContext*) hwfc->hwctx;
    ni_session_data_io_t *p_src_session_data;
    niFrameSurface1_t *dst_surf;
    int ret = 0;
    int dst_stride[4];
    int pixel_format;
    int need_to_copy = 1;
    size_t crop_right = 0, crop_bottom = 0;

    dst_surf = (niFrameSurface1_t *)dst->data[3];

    if (dst_surf == NULL || dst->hw_frames_ctx == NULL) {
        av_log(hwfc, AV_LOG_ERROR, "Invalid hw frame\n");
        return AVERROR(EINVAL);
    }

    p_src_session_data = &f_hwctx->src_session_io_data;

    switch (src->format) {
    /* 8-bit YUV420 planar */
    case AV_PIX_FMT_YUV420P:
        dst_stride[0] = FFALIGN(src->width, 128);
        dst_stride[1] = FFALIGN((src->width / 2), 128);
        dst_stride[2] = dst_stride[1];
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_YUV420P;
        break;

    /* 10-bit YUV420 planar, little-endian, least significant bits */
    case AV_PIX_FMT_YUV420P10LE:
        dst_stride[0] = FFALIGN(src->width * 2, 128);
        dst_stride[1] = FFALIGN(src->width, 128);
        dst_stride[2] = dst_stride[1];
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_YUV420P10LE;
        break;

    /* 8-bit YUV420 semi-planar */
    case AV_PIX_FMT_NV12:
        dst_stride[0] = FFALIGN(src->width, 128);
        dst_stride[1] = dst_stride[0];
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_NV12;
        break;

    /* 8-bit yuv422 semi-planar */
    case AV_PIX_FMT_NV16:
        dst_stride[0] = FFALIGN(src->width, 64);
        dst_stride[1] = dst_stride[0];
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_NV16;
        break;

    /*8-bit yuv422 planar */
    case AV_PIX_FMT_YUYV422:
        dst_stride[0] = FFALIGN(src->width, 16) * 2;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_YUYV422;
        break;

    case AV_PIX_FMT_UYVY422:
        dst_stride[0] = FFALIGN(src->width, 16) * 2;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_UYVY422;
        break;

    /* 10-bit YUV420 semi-planar, little endian, most significant bits */
    case AV_PIX_FMT_P010LE:
        dst_stride[0] = FFALIGN(src->width * 2, 128);
        dst_stride[1] = dst_stride[0];
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_P010LE;
        break;

    /* 32-bit RGBA packed */
    case AV_PIX_FMT_RGBA:
        /* RGBA for the scaler has a 16-byte width/64-byte stride alignment */
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_RGBA;
        break;

    case AV_PIX_FMT_BGRA:
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_BGRA;
        break;

    case AV_PIX_FMT_ABGR:
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_ABGR;
        break;

    case AV_PIX_FMT_ARGB:
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_ARGB;
        break;

    case AV_PIX_FMT_BGR0:
        dst_stride[0] = FFALIGN(src->width, 16) * 4;
        dst_stride[1] = 0;
        dst_stride[2] = 0;
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_BGR0;
        break;

    /* 24-bit BGR planar not supported for hwupload */
    /*
    case AV_PIX_FMT_BGRP:
        dst_stride[0] = FFALIGN(src->width, 32);
        dst_stride[1] = FFALIGN(src->width, 32);
        dst_stride[2] = FFALIGN(src->width, 32);
        dst_stride[3] = 0;

        pixel_format = NI_PIX_FMT_BGRP;
        break;
    */
    default:
        av_log(hwfc, AV_LOG_ERROR, "Pixel format %s not supported by device %s\n",
#if IS_FFMPEG_70_AND_ABOVE_FOR_LIBAVUTIL
               av_get_pix_fmt_name(src->format), ffhwframesctx(hwfc)->hw_type->name);
#else
               av_get_pix_fmt_name(src->format), hwfc->internal->hw_type->name);
#endif
        return AVERROR(EINVAL);
    }

    // check input resolution zero copy compatible or not
    if (ni_uploader_frame_zerocopy_check(&f_hwctx->api_ctx,
        src->width, src->height,
        (const int *)src->linesize, pixel_format) == NI_RETCODE_SUCCESS) {
        need_to_copy = 0;
        p_src_session_data->data.frame.extra_data_len =
            NI_APP_ENC_FRAME_META_DATA_SIZE;
        // alloc metadata buffer etc. (if needed)
        ret = ni_encoder_frame_zerocopy_buffer_alloc(
            &p_src_session_data->data.frame, src->width,
            src->height, (const int *)src->linesize, (const uint8_t **)src->data,
            (int)p_src_session_data->data.frame.extra_data_len);
        if (ret != NI_RETCODE_SUCCESS) {
            return AVERROR(ENOMEM);
        }
    } else {
        // allocate only once per upload Session when we have frame info
        p_src_session_data->data.frame.extra_data_len =
            NI_APP_ENC_FRAME_META_DATA_SIZE;

        ret = ni_frame_buffer_alloc_pixfmt(&p_src_session_data->data.frame,
                                           pixel_format, src->width,
                                           src->height, dst_stride,
                                           1, // force to av_codec_id_h264 for max compat
                                           (int)p_src_session_data->data.frame.extra_data_len);
        if (ret < 0) {
            av_log(hwfc, AV_LOG_ERROR, "Cannot allocate ni_frame %d\n", ret);
            return ret;
        }
    }

    if (need_to_copy) {
        ret = av_to_niframe_copy(hwfc, dst_stride, &p_src_session_data->data.frame, src);
        if (ret < 0) {
            av_log(hwfc, AV_LOG_ERROR, "%s can't copy frame\n", __func__);
            return AVERROR(EINVAL);
        }
    }

    ret = ni_device_session_hwup(&f_hwctx->api_ctx, p_src_session_data, dst_surf);
    if (ret < 0) {
        av_log(hwfc, AV_LOG_ERROR, "%s failed to upload frame %d\n",
               __func__, ret);
        return AVERROR_EXTERNAL;
    }

    dst_surf->ui16width = f_hwctx->split_ctx.w[0] = src->width;
    dst_surf->ui16height = f_hwctx->split_ctx.h[0] = src->height;
    dst_surf->ui32nodeAddress = 0; // always 0 offset for upload
    ff_ni_set_bit_depth_and_encoding_type(&dst_surf->bit_depth,
                                          &dst_surf->encoding_type,
                                          src->format);
    av_log(hwfc, AV_LOG_VERBOSE, "%s trace ui16FrameIdx = [%u] hdl %d SID%d\n",
           __func__, dst_surf->ui16FrameIdx, dst_surf->device_handle,
           dst_surf->ui16session_ID);

    // Update frames context
    f_hwctx->split_ctx.f[0] = (int)dst_surf->encoding_type;

    /* Set the hw_id/card number in AVNIFramesContext */
    ((AVNIFramesContext*)((AVHWFramesContext*)dst->hw_frames_ctx->data)->hwctx)->hw_id = f_hwctx->api_ctx.hw_id;

    // only FFmpeg 3.4.2 and above have crop_*
#if (LIBAVUTIL_VERSION_MAJOR > 55 ||                                           \
     (LIBAVUTIL_VERSION_MAJOR == 55 && LIBAVUTIL_VERSION_MINOR >= 78))
    crop_right  = dst->crop_right;
    crop_bottom = dst->crop_bottom;
#endif

    av_frame_copy_props(dst, src); // should get the metadata right
    av_log(hwfc, AV_LOG_DEBUG, "%s Upload frame w/h %d/%d crop r/b %lu/%lu\n",
           __func__, dst->width, dst->height, crop_right, crop_bottom);

    return ret;
}

static int ni_transfer_data_to(AVHWFramesContext *hwfc, AVFrame *dst,
                               const AVFrame *src)
{
    int err;
    niFrameSurface1_t *dst_surf;

    if (src->width > hwfc->width || src->height > hwfc->height) {
        return AVERROR(EINVAL);
    }

    /* should check against MAX frame size */
    err = ni_hwup_frame(hwfc, dst, src);
    if (err) {
        return err;
    }

    dst_surf = (niFrameSurface1_t *)(dst->data[3]);

    av_log(hwfc, AV_LOG_VERBOSE,
           "hwcontext.c:ni_hwup_frame() dst_surf FID %d %d\n",
           dst_surf->ui16FrameIdx, dst_surf->ui16session_ID);

    return 0;
}

static int ni_transfer_data_from(AVHWFramesContext *hwfc, AVFrame *dst,
                                 const AVFrame *src)
{
    if (dst->width > hwfc->width || dst->height > hwfc->height) {
        av_log(hwfc, AV_LOG_ERROR, "Invalid frame dimensions\n");
        return AVERROR(EINVAL);
    }

    return ni_hwdl_frame(hwfc, dst, src);
}

const HWContextType ff_hwcontext_type_ni_quadra = {
    // QUADRA
    .type = AV_HWDEVICE_TYPE_NI_QUADRA,
    .name = "NI_QUADRA",

    .device_hwctx_size = sizeof(AVNIDeviceContext),
    .frames_hwctx_size = sizeof(AVNIFramesContext),

    .device_create = ni_device_create,
    .device_uninit = ni_device_uninit,

    .frames_get_constraints = ni_frames_get_constraints,

    .frames_init   = ni_frames_init,
    .frames_uninit = ni_frames_uninit,

    .frames_get_buffer = ni_get_buffer,

    .transfer_get_formats = ni_transfer_get_formats,
    .transfer_data_to     = ni_transfer_data_to,
    .transfer_data_from   = ni_transfer_data_from,

    .pix_fmts =
        (const enum AVPixelFormat[]){AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE},
};
