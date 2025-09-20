/*
 * Copyright (c) 2023 NetInt
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

#include "nifilter.h"
#include "avfilter.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#endif
#include "libavutil/opt.h"
#include "libavutil/macros.h"
#include "filters.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#ifndef _WIN32
#include <ni_p2p_ioctl.h>
#include <sys/ioctl.h>
#endif

typedef struct NetIntP2PXferContext {
    const AVClass *class;

    int fd;
    int card_no;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
    ni_frame_t niframe;

    AVBufferRef *out_frames_ref;

    uint64_t dest_addr;

    int keep_alive_timeout;
    int session_opened;
    bool initialized;
    AVFrame *out;
    int count;
} NetIntP2PXferContext;

AVFilter ff_vf_p2pxfer_ni;

#ifndef _WIN32
static int val(char c)
{
    if ((c >= '0') && (c <= '9'))
        return c - '0';
    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;
    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    return -1;
}

static int get_bdf_by_block_name(
    AVFilterContext *ctx,
    const char *block_name,
    int *domain,
    int *bus,
    int *device,
    int *function)
{
    const char *base_name = &block_name[5]; // skip the "/dev/"
    char sysfile[128];
    char line[128];
    FILE *fp;
    int ret = -1;

    // /sys/block/nvme0n1/device/device/uevent file has
    snprintf(sysfile, 128, "/sys/block/%s/device/device/uevent", base_name);

    av_log(ctx, AV_LOG_DEBUG, "sysfile %s\n",sysfile);
    fp = fopen(sysfile,"r");

    if (!fp) {
        av_log(ctx, AV_LOG_ERROR, "Can't get PCIe device\n");
        return -1;
    }

    // PCI_SLOT_NAME=0000:0b:00.0
    while (fgets(line, 128, fp) != NULL) {
        if (strncmp("PCI_SLOT_NAME=", line, 14) == 0) {
            *domain = (val(line[14]) << 12) + (val(line[15]) << 8) + (val(line[16]) << 4) + (val(line[17]));
            *bus = (val(line[19]) << 4) + val(line[20]);
            *device = (val(line[22]) << 4) + val(line[23]);
            *function = val(line[25]);
            av_log(ctx, AV_LOG_DEBUG, "%04x:%02x:%02x.%x\n",*domain,*bus,*device,*function);
            ret = 0;
            break;
        }
    }

    fclose(fp);

    return ret;
}

static int import_dma_buf(
    AVFilterLink *link,
    int domain,
    int bus,
    int dev,
    int fnc,
    uint64_t *dma_addr)
{
    AVFilterContext *ctx = link->dst;
    NetIntP2PXferContext *s = ctx->priv;
    niFrameSurface1_t *frame_surface;
    struct netint_iocmd_import_dmabuf uimp;
    int ret;

    frame_surface = (niFrameSurface1_t *) s->niframe.p_data[3];

    uimp.fd = frame_surface->dma_buf_fd;
    uimp.flags = 0;
    uimp.domain = domain;
    uimp.bus = bus;
    uimp.dev = dev;
    uimp.fn = fnc;

    ret = ioctl(s->api_ctx.netint_fd, NETINT_IOCTL_IMPORT_DMABUF, &uimp);

    *dma_addr = uimp.dma_addr[0];

    return ret;
}

static int cleanup_dma_buf(
    AVFilterContext *ctx,
    int domain,
    int bus,
    int dev,
    int fnc)
{
    NetIntP2PXferContext *s = ctx->priv;
    niFrameSurface1_t *frame_surface;
    struct netint_iocmd_import_dmabuf uimp;

    frame_surface = (niFrameSurface1_t *) s->niframe.p_data[3];

    uimp.fd = frame_surface->dma_buf_fd;
    uimp.flags = 1;
    uimp.domain = domain;
    uimp.bus = bus;
    uimp.dev = dev;
    uimp.fn = fnc;

    return ioctl(s->api_ctx.netint_fd, NETINT_IOCTL_IMPORT_DMABUF, &uimp);
}
#endif

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] =
        { AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE };
    AVFilterFormats *formats;

    /* This filter only sends and receives hw frames */
    formats = ff_make_format_list(pix_fmts);

    if (!formats)
        return AVERROR(ENOMEM);

    return ff_set_common_formats(ctx, formats);
}

static av_cold int preinit(AVFilterContext *ctx)
{
    NetIntP2PXferContext *s = ctx->priv;

#ifdef _WIN32
    av_log(ctx, AV_LOG_ERROR, "MS Windows not supported\n");
    return AVERROR(EINVAL);
#else
    /* Check for the NetInt kernel device driver */
    s->fd = open("/dev/netint", O_RDWR);

    if (s->fd < 0) {
        switch (errno) {
        case EACCES:
            av_log(ctx, AV_LOG_ERROR, "No permission to access /dev/netint\n");
            break;
        case ENOENT:
        case ENODEV:
            av_log(ctx, AV_LOG_ERROR, "NetInt kernel driver not installed\n");
            break;
        default:
            perror("p2pxfer");
            break;
        }

        return AVERROR(errno);
    }

    return 0;
#endif
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntP2PXferContext *s = ctx->priv;

    /* Close hwupload P2P session on target device */
    if (s->fd > 0)
        close(s->fd);

#ifndef _WIN32
    if (s->initialized) {
        if (cleanup_dma_buf(ctx,0,0,0,0) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Can't clean up DMA buffer\n");
        }
    }
#endif

    if (s->session_opened) {
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_UPLOAD);
        ni_device_session_context_clear(&s->api_ctx);
    }

    av_frame_free(&s->out);
}

static int config_input(AVFilterLink *inlink)
{
#ifndef _WIN32
    AVFilterContext *ctx = inlink->dst;
    NetIntP2PXferContext *s = ctx->priv;
    AVHWFramesContext *frames_ctx = NULL;
    AVNIDeviceContext *device_ctx = frames_ctx->device_ctx->hwctx;
    ni_retcode_t retcode = 0;
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    frames_ctx = (AVHWFramesContext *) li->hw_frames_ctx->data;
#else
    if (inlink->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    frames_ctx = (AVHWFramesContext *) inlink->hw_frames_ctx->data;
#endif

    if (frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No incoming hw frames context\n");
        return 0;
    }

    /* Check the incoming frame context */
    av_log(ctx, AV_LOG_DEBUG, "w: %d; h: %d; fmt: %s; sw_fmt: %s\n",
           inlink->w, inlink->h,
           av_get_pix_fmt_name(inlink->format),
           av_get_pix_fmt_name(frames_ctx->sw_format));

    /* Check for existence of target device */
    if (s->card_no >= 0) {
        retcode = ni_device_session_context_init(&s->api_ctx);

        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Session context init failure\n");
            goto fail;
        }

        s->api_ctx.device_handle      = device_ctx->cards[s->card_no];
        s->api_ctx.blk_io_handle      = device_ctx->cards[s->card_no];
        s->api_ctx.hw_id              = s->card_no;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

        retcode = ni_uploader_set_frame_format(&s->api_ctx, inlink->w, inlink->h,
            ff_ni_ffmpeg_to_libxcoder_pix_fmt(frames_ctx->sw_format), 1);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't set frame format\n");
            goto fail;
        }

        /* Open hwupload P2P session on target device */
        retcode = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_UPLOAD);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't open target device\n");
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_UPLOAD);
            ni_device_session_context_clear(&s->api_ctx);
            goto fail;
        }
        s->session_opened = 1;

        /* Create a P2P frame pool for the uploader session of size 1 */
        retcode = ni_device_session_init_framepool(&s->api_ctx, 1, 1);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR, "Can't create frame pool\n");
            goto fail;
        }

        /* Allocate the host memory for a hw ni_frame */
        retcode = ni_frame_buffer_alloc_hwenc(&s->niframe,
                                              inlink->w, inlink->h, 0);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "Can't allocate host memory\n");
            goto fail;
        }

        /* Acquire a hw frame from the upload session */
        retcode = ni_device_session_acquire(&s->api_ctx, &s->niframe);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR, "Can't acquire frame\n");
            goto fail;
        }
    } else {
        av_log(ctx, AV_LOG_ERROR, "Invalid card number\n");
        retcode = -1;
        goto fail;
    }

fail:
    return retcode;
#else
    /* Windows not supported */
    return AVERROR(EINVAL);
#endif
}

static int config_output(AVFilterLink *outlink)
{
#ifndef _WIN32
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    NetIntP2PXferContext *s = ctx->priv;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    int ret=0;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[0]);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *) li->hw_frames_ctx->data;
#else
    in_frames_ctx = (AVHWFramesContext *) ctx->inputs[0]->hw_frames_ctx->data;
#endif

    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    outlink->w = inlink->w;
    outlink->h = inlink->h;

    out_frames_ctx = (AVHWFramesContext *) s->out_frames_ref->data;

    out_frames_ctx->format = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width  = outlink->w;
    out_frames_ctx->height = outlink->h;
    out_frames_ctx->sw_format = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size = NI_PAD_ID; // repurposed
    ((AVNIFramesContext*)out_frames_ctx->hwctx)->hw_id = s->card_no;

    av_log(ctx, AV_LOG_DEBUG, "%s: w=%d; h=%d\n", __func__,
           outlink->w, outlink->h);

    ret = av_hwframe_ctx_init(s->out_frames_ref);
    if (ret < 0)
        return ret;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(ctx->outputs[0]);
    av_buffer_unref(&lo->hw_frames_ctx);

    lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);
#else
    av_buffer_unref(&ctx->outputs[0]->hw_frames_ctx);

    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!ctx->outputs[0]->hw_frames_ctx)
        return AVERROR(ENOMEM);
#endif
    return 0;
#else
    return AVERROR(EINVAL);
#endif
}

static int filter_frame(AVFilterLink *link, AVFrame *frame)
{
#ifndef _WIN32
    AVFilterContext *ctx = link->dst;
    NetIntP2PXferContext *s = ctx->priv;
    niFrameSurface1_t *frame_surface;
    niFrameSurface1_t *in_frame_surface,*out_frame_surface;
    ni_device_info_t *ni_dev_info;
    uint16_t ui16FrameIdx;
    uint32_t ui32FrameSize;
    int linesize[3];
    AVFrame *out = NULL;
    int retcode,domain,bus,dev,func;

    s->count++;

    /* Extract buffer id from incoming frame */
    frame_surface = (niFrameSurface1_t *) frame->data[3];

    if (frame_surface == NULL) {
        return AVERROR(EINVAL);
    }

    ui16FrameIdx = frame_surface->ui16FrameIdx;
    av_log(ctx, AV_LOG_DEBUG, "Incoming frame idx %d\n", ui16FrameIdx);

    /* Get size of data to transfer */
    linesize[0] = FFALIGN(link->w, 128);
    linesize[1] = FFALIGN(link->w/2, 128);
    linesize[2] = FFALIGN(link->w/2, 128);

    ui32FrameSize = ni_calculate_total_frame_size(&s->api_ctx, linesize);

    if (!s->initialized) {
        s->initialized = 1;

        /* Call Netint driver to import the dma buf */
        ni_dev_info = ni_rsrc_get_device_info(NI_DEVICE_TYPE_SCALER,
                                              ni_get_cardno(frame));

        if (ni_dev_info == NULL) {
            av_log(ctx, AV_LOG_ERROR, "Can't get device info\n");
            return AVERROR(EINVAL);
        }

        if (get_bdf_by_block_name(ctx,ni_dev_info->blk_name,&domain,&bus,&dev,&func) != 0) {
            av_log(ctx, AV_LOG_ERROR, "Can't get bdf\n");
            retcode = AVERROR(EINVAL);
            goto fail;
        }

        ni_memfree(ni_dev_info);

        av_log(ctx, AV_LOG_DEBUG, "Import DMA buf: src domain %d, bus %d, dev %d, func %d\n",
               domain,bus,dev,func);

        if (import_dma_buf(link,domain,bus,dev,func,&s->dest_addr) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Can't import DMA buffer\n");
            return AVERROR(EINVAL);
        }

        av_log(ctx, AV_LOG_DEBUG, "Instruct card to send to address %lx, size = %u\n", s->dest_addr, ui32FrameSize);
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    /* Send NVME command to source card */
    retcode = ni_p2p_send(&s->api_ctx, frame_surface, s->dest_addr, ui32FrameSize);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "P2P transfer failed\n");
        goto fail;
    }

    if (!s->initialized) {
        /* Clean up dma-buf */
        if (cleanup_dma_buf(ctx, domain, bus, dev, func) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Can't clean up DMA buffer\n");
            return AVERROR(EINVAL);
        }
    }

    if (s->count == 1) {

        /* Generate new hardware frame for destination */
        out = av_frame_alloc();
        if (!out) {
            av_log(ctx, AV_LOG_ERROR, "Can't allocate out frame\n");
            retcode = AVERROR(ENOMEM);
            goto fail;
        }

        av_frame_copy_props(out, frame);

        out->width = link->w;
        out->height = link->h;
        out->format = AV_PIX_FMT_NI_QUAD;

        out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        out->data[3] = av_malloc(sizeof(niFrameSurface1_t));

        if (!out->data[3]) {
            av_log(ctx, AV_LOG_ERROR, "Can't allocate frame surface\n");
            retcode = AVERROR(ENOMEM);
            goto fail;
        }

        /* Copy the frame surface from the destination frame */
        out_frame_surface = (niFrameSurface1_t *) out->data[3];
        in_frame_surface  = (niFrameSurface1_t *) s->niframe.p_data[3];

        memcpy(out_frame_surface, in_frame_surface, sizeof(niFrameSurface1_t));

        out_frame_surface->ui32nodeAddress = 0;

        out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t),
                                       ff_ni_frame_free, NULL, 0);

        av_log(ctx, AV_LOG_DEBUG,
               "%s:IN trace ui16FrameIdx = [%d] --> out [%d] \n", __func__,
               ui16FrameIdx, out_frame_surface->ui16FrameIdx);

        av_frame_free(&s->out);
        s->out = av_frame_clone(out);
    } else {
        out = av_frame_clone(s->out);
    }

    av_frame_free(&frame);

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_p2pxfer");
#endif

    /* For now it's a pass-thru */
    return ff_filter_frame(link->dst->outputs[0], out);

fail:
    av_frame_free(&frame);
    av_frame_free(&out);
    return retcode;
#else
    return AVERROR(EINVAL);
#endif
}

#define OFFSET(x) offsetof(NetIntP2PXferContext, x)
#define FLAGS (AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM)

static const AVOption ni_p2pxfer_options[] = {
    { "card_no", "target hardware id", OFFSET(card_no), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 255, FLAGS },
    NI_FILT_OPTION_KEEPALIVE,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_p2pxfer);

static const AVFilterPad inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_input,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_output,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_p2pxfer_ni_quadra = {
    .name = "ni_quadra_p2pxfer",
    .description = NULL_IF_CONFIG_SMALL("NETINT Quadra P2P transfer v" NI_XCODER_REVISION),
    .priv_size = sizeof(NetIntP2PXferContext),
    .priv_class = &ni_p2pxfer_class,
    .preinit = preinit,
    .uninit = uninit,
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs = inputs,
    .outputs = outputs,
    .query_formats = query_formats
#endif
};
