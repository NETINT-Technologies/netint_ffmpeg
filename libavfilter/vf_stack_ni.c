/*
 * Copyright (c) 2015 Paul B. Mahol
 * Copyright (c) 2022 NETINT
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

#include "libavutil/avstring.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"

#include "version.h"
#include "filters.h"
#include "formats.h"
#if !(LIBAVFILTER_VERSION_MAJOR > 10 || (LIBAVFILTER_VERSION_MAJOR == 10 && LIBAVFILTER_VERSION_MINOR >= 4))
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
#include "framesync.h"
#include "video.h"
#include "nifilter.h"

#define MAX_INPUTS 8
#define MAX_XSTACK_INPUTS 50

typedef struct StackItem {
    int x, y;
    int w;
    int h;
} StackItem;

typedef struct NetIntStackContext {
    const AVClass *class;
    const AVPixFmtDescriptor *desc;
    int nb_inputs;
    char *layout;
    char *size;
    int shortest;
    uint8_t fillcolor[4];
    char *fillcolor_str;
    int fillcolor_enable;
    int sync;

    StackItem *items;
    AVFrame **frames;
    FFFrameSync fs;

    // The rest of this structure is NETINT HW specific

    enum AVPixelFormat out_format;
    AVBufferRef *out_frames_ref;

    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
    ni_scaler_params_t params;

    int initialized;
    int session_opened;
    int keep_alive_timeout;
    bool is_p2p;
    int buffer_limit;

    ni_frame_config_t frame_in[MAX_XSTACK_INPUTS];
    ni_frame_config_t frame_out;
} NetIntStackContext;

static int process_frame(FFFrameSync *fs);

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] =
        {AV_PIX_FMT_NI_QUAD, AV_PIX_FMT_NONE};
    AVFilterFormats *formats;

    formats = ff_make_format_list(pix_fmts);

    if (!formats)
        return AVERROR(ENOMEM);

    return ff_set_common_formats(ctx, formats);
}

static av_cold int init(AVFilterContext *ctx)
{
    NetIntStackContext *s = ctx->priv;
    int i, ret;

    // NOLINTNEXTLINE(bugprone-sizeof-expression)
    s->frames = av_calloc(s->nb_inputs, sizeof(*s->frames));
    if (!s->frames)
        return AVERROR(ENOMEM);

    s->items = av_calloc(s->nb_inputs, sizeof(*s->items));
    if (!s->items)
        return AVERROR(ENOMEM);

    if ((strcmp(s->fillcolor_str, "none") != 0) &&
        av_parse_color(s->fillcolor, s->fillcolor_str, -1, ctx) >= 0) {
        s->fillcolor_enable = 1;
    } else {
        s->fillcolor_enable = 0;
    }
    if (!s->layout) {
        if (s->nb_inputs == 2) {
            s->layout = av_strdup("0_0|w0_0");
            if (!s->layout)
                return AVERROR(ENOMEM);
        } else {
            av_log(ctx, AV_LOG_ERROR, "No layout specified.\n");
            return AVERROR(EINVAL);
        }
    }

    for (i = 0; i < s->nb_inputs; i++) {
        AVFilterPad pad = { 0 };

        pad.type = AVMEDIA_TYPE_VIDEO;
        pad.name = av_asprintf("input%d", i);
        if (!pad.name)
            return AVERROR(ENOMEM);

#if (LIBAVFILTER_VERSION_MAJOR >= 8)
        if ((ret = ff_append_inpad(ctx, &pad)) < 0) {
#else
        if ((ret = ff_insert_inpad(ctx, i, &pad)) < 0) {
#endif
            av_freep(&pad.name);
            return ret;
        }
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntStackContext *s = ctx->priv;
    int i;

    ff_framesync_uninit(&s->fs);
    av_freep(&s->frames);
    av_freep(&s->items);

    for (i = 0; i < ctx->nb_inputs; i++)
        av_freep(&ctx->input_pads[i].name);

    if (s->api_dst_frame.data.frame.p_buffer)
        ni_frame_buffer_free(&s->api_dst_frame.data.frame);

    if (s->session_opened) {
        /* Close operation will free the device frames */
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
    }

    av_buffer_unref(&s->out_frames_ref);
}

static int init_out_pool(AVFilterContext *ctx)
{
    NetIntStackContext *s = ctx->priv;
    AVHWFramesContext *out_frames_ctx;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[0]);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#else
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#endif

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    /* Don't check return code, this will intentionally fail */
    // av_hwframe_ctx_init(s->out_frames_ref);

    if (s->api_ctx.isP2P) {
        pool_size = 1;
    }
#if IS_FFMPEG_71_AND_ABOVE
    else {
        pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
    }
#endif

#if IS_FFMPEG_61_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create frame pool on device */
    return ff_ni_build_frame_pool(&s->api_ctx, out_frames_ctx->width,
                                  out_frames_ctx->height,
                                  s->out_format, pool_size,
                                  s->buffer_limit);
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    NetIntStackContext *s = ctx->priv;
    // AVRational frame_rate = ctx->inputs[0]->frame_rate;
    // AVRational sar = ctx->inputs[0]->sample_aspect_ratio;
    int height, width;
    FFFrameSyncIn *in;
    int i, ret;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *in_frames_ctx0;
    AVHWFramesContext *out_frames_ctx;
    char *arg, *p, *saveptr = NULL;
    char *arg2, *p2, *saveptr2 = NULL;
    char *arg3, *p3, *saveptr3 = NULL;
    int inw, inh, size;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[0]);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#else
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
#endif
    s->desc = av_pix_fmt_desc_get(outlink->format);
    if (!s->desc)
        return AVERROR_BUG;

    if (s->size == NULL) {
        for (i = 0; i < s->nb_inputs; i++) {
            AVFilterLink *inlink = ctx->inputs[i];
            StackItem *item = &s->items[i];

            item->w = FFALIGN(inlink->w,2);
            item->h = FFALIGN(inlink->h,2);
        }
    } else {
        p = s->size;
        for (i = 0; i < s->nb_inputs; i++) {
            StackItem *item = &s->items[i];

            if (!(arg = av_strtok(p, "|", &saveptr)))
                return AVERROR(EINVAL);

            p = NULL;

            p2 = arg;
            inw = inh = 0;

            for (int j = 0; j < 2; j++) {
                if (!(arg2 = av_strtok(p2, "_", &saveptr2)))
                    return AVERROR(EINVAL);

                p2 = NULL;
                p3 = arg2;
                while ((arg3 = av_strtok(p3, "+", &saveptr3))) {
                    p3 = NULL;
                    if (sscanf(arg3, "%d", &size) == 1) {
                        if (size < 0)
                            return AVERROR(EINVAL);

                        if (!j)
                            inw += size;
                        else
                            inh += size;
                    } else {
                        return AVERROR(EINVAL);
                    }
                }
            }

            item->w = FFALIGN(inw,2);
            item->h = FFALIGN(inh,2);
        }
    }

    width = 0;
    height = 0;
    p = s->layout;
    saveptr = NULL;
    saveptr2 = NULL;
    saveptr3 = NULL;
    for (i = 0; i < s->nb_inputs; i++) {
        StackItem *item = &s->items[i];

        if (!(arg = av_strtok(p, "|", &saveptr)))
            return AVERROR(EINVAL);

        p = NULL;

        p2 = arg;
        inw = inh = 0;

        for (int j = 0; j < 2; j++) {
            if (!(arg2 = av_strtok(p2, "_", &saveptr2)))
                return AVERROR(EINVAL);

            p2 = NULL;
            p3 = arg2;
            while ((arg3 = av_strtok(p3, "+", &saveptr3))) {
                p3 = NULL;
                if (sscanf(arg3, "w%d", &size) == 1) {
                    if (size == i || size < 0 || size >= s->nb_inputs)
                        return AVERROR(EINVAL);

                    if (!j)
                        inw += s->items[size].w;
                    else
                        inh += s->items[size].w;
                } else if (sscanf(arg3, "h%d", &size) == 1) {
                    if (size == i || size < 0 || size >= s->nb_inputs)
                        return AVERROR(EINVAL);

                    if (!j)
                        inw += s->items[size].h;
                    else
                        inh += s->items[size].h;
                } else if (sscanf(arg3, "%d", &size) == 1) {
                    if (size < 0)
                        return AVERROR(EINVAL);

                    if (!j)
                        inw += size;
                    else
                        inh += size;
                } else {
                    return AVERROR(EINVAL);
                }
            }
        }

        item->x = FFALIGN(inw,2);
        item->y = FFALIGN(inh,2);

        width  = FFMAX(width,  item->w + inw);
        height = FFMAX(height, item->h + inh);
    }

    if ((s->sync < 0) || (s->sync >= s->nb_inputs)) {
        av_log(ctx, AV_LOG_ERROR,
               "Cannot sync to %d, valid range is 0 to %d!!!, Defaulting to 0.\n",
               s->sync, s->nb_inputs - 1);
        s->sync = 0;
    }

    outlink->w          = width;
    outlink->h          = height;
    outlink->sample_aspect_ratio = ctx->inputs[s->sync]->sample_aspect_ratio;
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(outlink);
    FilterLink *lt = ff_filter_link(ctx->inputs[s->sync]);
    lo->frame_rate = lt->frame_rate;
    in_frames_ctx0 = (AVHWFramesContext *)li->hw_frames_ctx->data;
#else
    outlink->frame_rate = ctx->inputs[s->sync]->frame_rate;
    in_frames_ctx0 = (AVHWFramesContext *)ctx->inputs[0]->hw_frames_ctx->data;
#endif
    if (in_frames_ctx0->sw_format == AV_PIX_FMT_BGRP) {
        av_log(ctx, AV_LOG_ERROR, "bgrp not supported\n");
        return AVERROR(EINVAL);
    }
    if (in_frames_ctx0->sw_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
        in_frames_ctx0->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(ctx, AV_LOG_ERROR, "tile4x4 not supported\n");
        return AVERROR(EINVAL);
    }

    for (i = 1; i < s->nb_inputs; i++) {
#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *lii = ff_filter_link(ctx->inputs[i]);
        if (lii->hw_frames_ctx == NULL) {
            av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
            return AVERROR(EINVAL);
        }
        in_frames_ctx = (AVHWFramesContext *)lii->hw_frames_ctx->data;
#else
        if (ctx->inputs[i]->hw_frames_ctx == NULL) {
            av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
            return AVERROR(EINVAL);
        }
        in_frames_ctx = (AVHWFramesContext *)ctx->inputs[i]->hw_frames_ctx->data;
#endif
        if (in_frames_ctx0->sw_format != in_frames_ctx->sw_format) {
            av_log(ctx, AV_LOG_ERROR,
                   "All inputs must have the same pixel format!!!\n");
            return AVERROR(EINVAL);
        }

        if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
            av_log(ctx, AV_LOG_ERROR, "bgrp not supported\n");
            return AVERROR(EINVAL);
        }
        if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
            in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
            av_log(ctx, AV_LOG_ERROR, "tile4x4 not supported\n");
            return AVERROR(EINVAL);
        }
    }

    for (i = 1; i < s->nb_inputs; i++) {
        AVFilterLink *inlink = ctx->inputs[i];
#if IS_FFMPEG_71_AND_ABOVE
        li = ff_filter_link(inlink);
        lo = ff_filter_link(outlink);
        if (lo->frame_rate.num != li->frame_rate.num ||
            lo->frame_rate.den != li->frame_rate.den) {
            av_log(ctx, AV_LOG_WARNING,
                    "Video inputs have different frame rates, output will be VFR\n");
            lo->frame_rate = av_make_q(1, 0);
            break;
        }
#else
        if (outlink->frame_rate.num != inlink->frame_rate.num ||
            outlink->frame_rate.den != inlink->frame_rate.den) {
            av_log(ctx, AV_LOG_WARNING,
                    "Video inputs have different frame rates, output will be VFR\n");
            outlink->frame_rate = av_make_q(1, 0);
            break;
        }
#endif
    }

    if ((ret = ff_framesync_init(&s->fs, ctx, s->nb_inputs)) < 0)
        return ret;

    in = s->fs.in;
    s->fs.opaque = s;
    s->fs.on_event = process_frame;

    /* Give each input a unique sync priority */
    for (i = 0; i < s->nb_inputs; i++) {
        AVFilterLink *inlink = ctx->inputs[i];

        in[i].time_base = inlink->time_base;
        in[i].sync   = i+1;
        in[i].before = EXT_STOP;
        in[i].after  = s->shortest ? EXT_STOP : EXT_INFINITY;
    }

    /* Give the sync input highest priority */
    in[s->sync].sync = s->nb_inputs + 1;

    ret = ff_framesync_configure(&s->fs);
    outlink->time_base = s->fs.time_base;

    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx0->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    out_frames_ctx->format    = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width     = outlink->w;
    out_frames_ctx->height    = outlink->h;
    out_frames_ctx->sw_format = in_frames_ctx0->sw_format;
    out_frames_ctx->initial_pool_size =
        NI_STACK_ID; // Repurposed as identity code

    s->out_format = out_frames_ctx->sw_format;

    av_hwframe_ctx_init(s->out_frames_ref);

#if IS_FFMPEG_71_AND_ABOVE
    lo = ff_filter_link(ctx->outputs[0]);
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
    return ret;
}

static int process_frame(FFFrameSync *fs)
{
    AVFilterContext *ctx = fs->parent;
    AVFilterLink *outlink = ctx->outputs[0];
    NetIntStackContext *s = fs->opaque;
    AVFrame **in = s->frames;
    AVFrame *out = NULL;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    int i, p, ret;

    AVFilterLink *inlink0 = ctx->inputs[0];
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    ni_retcode_t retcode;
    int scaler_format;
    uint16_t outFrameIdx;
    int num_cfg_inputs = MAX_INPUTS;

    for (i = 0; i < s->nb_inputs; i++) {
        if ((ret = ff_framesync_get_frame(&s->fs, i, &in[i], 0)) < 0)
            return ret;
    }

    if (!s->initialized) {
        int cardno, tmp_cardno;
        cardno = ni_get_cardno(in[0]);

        for (i = 1; i < s->nb_inputs; i++) {
            tmp_cardno = ni_get_cardno(in[i]);
            if (tmp_cardno != cardno) {
                // All inputs must be on the same Quadra device
                return AVERROR(EINVAL);
            }
        }
#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *li = ff_filter_link(inlink0);
        if (li->hw_frames_ctx == NULL) {
            av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
            return AVERROR(EINVAL);
        }
        pAVHFWCtx = (AVHWFramesContext *) li->hw_frames_ctx->data;
#else
        if (inlink0->hw_frames_ctx == NULL) {
            av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
            return AVERROR(EINVAL);
        }
        pAVHFWCtx = (AVHWFramesContext *) inlink0->hw_frames_ctx->data;
#endif
        pAVNIDevCtx = (AVNIDeviceContext *) pAVHFWCtx->device_ctx->hwctx;

        retcode = ni_device_session_context_init(&s->api_ctx);
        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni stack filter session context init failure\n");
            goto fail;
        }

        s->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

        s->api_ctx.hw_id             = cardno;
        s->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
        s->api_ctx.scaler_operation  = NI_SCALER_OPCODE_STACK;
        s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
        s->api_ctx.isP2P = s->is_p2p;

        av_log(ctx, AV_LOG_INFO,
               "Open scaler session to card %d, hdl %d, blk_hdl %d\n", cardno,
               s->api_ctx.device_handle, s->api_ctx.blk_io_handle);

        retcode =
            ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can't open device session on card %d\n", cardno);
            ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->api_ctx);
            goto fail;
        }

        s->session_opened = 1;

#if IS_FFMPEG_71_AND_ABOVE
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           ctx->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        retcode = init_out_pool(ctx);

        if (retcode < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", retcode);
            goto fail;
        }

        if (s->nb_inputs < MAX_INPUTS) {
            s->params.nb_inputs = s->nb_inputs;
        } else {
            s->params.nb_inputs = MAX_INPUTS;
        }
        retcode = ni_scaler_set_params(&s->api_ctx, &(s->params));
        if (retcode < 0)
            goto fail;

        AVHWFramesContext *out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        AVNIFramesContext *out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(pAVHFWCtx, out_frames_ctx);
        ni_device_session_copy(&s->api_ctx, &out_ni_ctx->api_ctx);

        s->initialized = 1;
    }

    out = av_frame_alloc();
    if (!out) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    av_frame_copy_props(out, in[s->sync]);

    out->width  = outlink->w;
    out->height = outlink->h;

    out->format = AV_PIX_FMT_NI_QUAD;

    /* Reference the new hw frames context */
    out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);

    out->data[3] = av_malloc(sizeof(niFrameSurface1_t));
    if (!out->data[3]) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Copy the frame surface from the incoming frame */
    memcpy(out->data[3], in[0]->data[3], sizeof(niFrameSurface1_t));

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          outlink->w,
                                          outlink->h,
                                          0);
    if (retcode != NI_RETCODE_SUCCESS) {
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    /* Allocate hardware device destination frame. This acquires a frame
     * from the pool
     */
    retcode = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);
    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR,
               "Can't acquire output frame %d\n",retcode);
        retcode = AVERROR(ENOMEM);
        goto fail;
    }

    frame_surface = (niFrameSurface1_t *)out->data[3];
    new_frame_surface = (niFrameSurface1_t *)s->api_dst_frame.data.frame.p_data[3];
    frame_surface->ui16FrameIdx   = new_frame_surface->ui16FrameIdx;
    frame_surface->ui16session_ID = new_frame_surface->ui16session_ID;
    frame_surface->device_handle  = new_frame_surface->device_handle;
    frame_surface->output_idx     = new_frame_surface->output_idx;
    frame_surface->src_cpu        = new_frame_surface->src_cpu;
    frame_surface->dma_buf_fd     = 0;

    ff_ni_set_bit_depth_and_encoding_type(&frame_surface->bit_depth,
                                          &frame_surface->encoding_type,
                                          s->out_format);

    /* Remove ni-split specific assets */
    frame_surface->ui32nodeAddress = 0;

    frame_surface->ui16width  = out->width;
    frame_surface->ui16height = out->height;

    av_log(ctx, AV_LOG_DEBUG,
           "vf_stack_ni.c:OUT trace ui16FrameIdx = [%d]\n",
           frame_surface->ui16FrameIdx);

    out->pts = av_rescale_q(s->fs.pts, s->fs.time_base, outlink->time_base);
    out->sample_aspect_ratio = outlink->sample_aspect_ratio;

    outFrameIdx = frame_surface->ui16FrameIdx;

    if (s->fillcolor_enable == 1) {
        s->frame_out.options = NI_SCALER_FLAG_FCE;
    }

    i = 0;
    for (p = s->nb_inputs; p > 0; p -= MAX_INPUTS) {
        int start = i;
        int end = i + MAX_INPUTS;

        if (end > s->nb_inputs) {
            num_cfg_inputs = p;
            end = s->nb_inputs;
        }

        for ( ; i < end; i++) {
            AVFilterLink *inlink = ctx->inputs[i];
#if IS_FFMPEG_71_AND_ABOVE
            FilterLink *li = ff_filter_link(inlink);
            pAVHFWCtx = (AVHWFramesContext *) li->hw_frames_ctx->data;
#else
            pAVHFWCtx = (AVHWFramesContext *) inlink->hw_frames_ctx->data;
#endif
            scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

            frame_surface = (niFrameSurface1_t *) in[i]->data[3];
            if (frame_surface == NULL) {
                return AVERROR(EINVAL);
            }

            s->frame_in[i].picture_width  = FFALIGN(in[i]->width, 2);
            s->frame_in[i].picture_height = FFALIGN(in[i]->height, 2);
            s->frame_in[i].picture_format = scaler_format;
            s->frame_in[i].session_id     = frame_surface->ui16session_ID;
            s->frame_in[i].output_index   = frame_surface->output_idx;
            s->frame_in[i].frame_index    = frame_surface->ui16FrameIdx;

            // Where to place the input into the output
            s->frame_in[i].rectangle_x    = s->items[i].x;
            s->frame_in[i].rectangle_y    = s->items[i].y;
            s->frame_in[i].rectangle_width = s->items[i].w;
            s->frame_in[i].rectangle_height = s->items[i].h;

            av_log(ctx, AV_LOG_DEBUG,
                   "vf_stack_ni.c:IN %d, ui16FrameIdx = [%d]\n",
                   i, frame_surface->ui16FrameIdx);
        }

        scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(s->out_format);

        s->frame_out.picture_width  = FFALIGN(outlink->w, 2);
        s->frame_out.picture_height = FFALIGN(outlink->h, 2);
        s->frame_out.picture_format = scaler_format;
        s->frame_out.frame_index    = outFrameIdx;
        s->frame_out.options        |= NI_SCALER_FLAG_IO;
        if (s->frame_out.options & NI_SCALER_FLAG_FCE) {
            s->frame_out.rgba_color = (s->fillcolor[3] << 24) | (s->fillcolor[0] << 16) |
                                      (s->fillcolor[1] << 8) | s->fillcolor[2];
        } else {
            s->frame_out.rgba_color = 0;
        }

        /*
         * Config device frame parameters
         */
        retcode = ni_device_multi_config_frame(&s->api_ctx, &s->frame_in[start],
                                               num_cfg_inputs, &s->frame_out);

        if (retcode != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_DEBUG,
                   "Can't transfer config frames %d\n", retcode);
            retcode = AVERROR(ENOMEM);
            goto fail;
        }

        /* Only fill the output frame once each process_frame */
        s->frame_out.options &= ~NI_SCALER_FLAG_FCE;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_xstack");
#endif

    out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t),
                                   ff_ni_frame_free, NULL, 0);

    return ff_filter_frame(outlink, out);

fail:
    av_frame_free(&out);
    return retcode;
}

static int activate(AVFilterContext *ctx)
{
    NetIntStackContext *s = ctx->priv;
    return ff_framesync_activate(&s->fs);
}

#define OFFSET(x) offsetof(NetIntStackContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_xstack_options[] = {
    { "inputs",   "set number of inputs",            OFFSET(nb_inputs),     AV_OPT_TYPE_INT,    {.i64=2},    2, MAX_XSTACK_INPUTS, .flags = FLAGS },
    { "layout",   "set custom layout",               OFFSET(layout),        AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, .flags = FLAGS },
    { "size",     "set custom size",                 OFFSET(size),          AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, .flags = FLAGS },
    { "fill",     "set the color for unused pixels", OFFSET(fillcolor_str), AV_OPT_TYPE_STRING, {.str = "black"}, .flags = FLAGS },
    { "sync",     "input to sync to",                OFFSET(sync),          AV_OPT_TYPE_INT,    {.i64=0},    0, MAX_XSTACK_INPUTS - 1, .flags = FLAGS },
    { "shortest", "force termination when the shortest input terminates", OFFSET(shortest), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, .flags = FLAGS },
    NI_FILT_OPTION_IS_P2P,
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_xstack);

static const AVFilterPad outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .config_props  = config_output,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_xstack_ni_quadra = {
    .name          = "ni_quadra_xstack",
    .description   = NULL_IF_CONFIG_SMALL("NETINT Quadra stack video inputs into custom layout v" NI_XCODER_REVISION),
    .priv_size     = sizeof(NetIntStackContext),
    .priv_class    = &ni_xstack_class,
    .init          = init,
    .uninit        = uninit,
    .activate      = activate,
    .flags         = AVFILTER_FLAG_DYNAMIC_INPUTS,
    .flags_internal= FF_FILTER_FLAG_HWFRAME_AWARE,
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .outputs       = outputs,
    .query_formats = query_formats,
#endif
};
