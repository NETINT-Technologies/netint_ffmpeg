/*
 * Copyright (c) 2022 NetInt
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

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "version.h"
#include "filters.h"
#include "formats.h"
#if !(LIBAVFILTER_VERSION_MAJOR > 10 || (LIBAVFILTER_VERSION_MAJOR == 10 && LIBAVFILTER_VERSION_MINOR >= 4))
#include "internal.h"
#else
#include "libavutil/mem.h"
#include "fftools/ffmpeg_sched.h"
#endif
#if HAVE_IO_H
#include <io.h>
#endif
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/parseutils.h"
#include "libavutil/pixdesc.h"
#include "libswscale/swscale.h"
#include "ni_device_api.h"
#include "ni_util.h"
#include "nifilter.h"
#include "video.h"

#define NI_NUM_FRAMES_IN_QUEUE 8

typedef struct _ni_roi_network_layer {
    int32_t width;
    int32_t height;
    int32_t channel;
    int32_t classes;
    int32_t component;
    int32_t mask[3];
    float biases[12];
    int32_t output_number;
    float *output;
} ni_roi_network_layer_t;

typedef struct _ni_roi_network {
    int32_t netw;
    int32_t neth;
    ni_network_data_t raw;
    ni_roi_network_layer_t *layers;
} ni_roi_network_t;

typedef struct box {
    float x, y, w, h;
} box;

typedef struct detection {
    box bbox;
    float objectness;
    int classes;
    int color;
    float *prob;
    int prob_class;
    float max_prob;
} detection;

typedef struct detetion_cache {
    detection *dets;
    int capacity;
    int dets_num;
} detection_cache;

struct roi_box {
    int left;
    int right;
    int top;
    int bottom;
    int color;
    float objectness;
    int cls;
    float prob;
};

typedef struct HwScaleContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
} HwScaleContext;

typedef struct AiContext {
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_src_frame;
    ni_session_data_io_t api_dst_pkt;
} AiContext;

typedef struct NetIntRoiContext {
    const AVClass *class;
    const char *nb_file;  /* path to network binary */
    AVRational qp_offset; /* default qp offset. */
    int initialized;
    int devid;
    float obj_thresh;
    float nms_thresh;

    AiContext *ai_ctx;

    AVBufferRef *out_frames_ref;

    ni_roi_network_t network;
    detection_cache det_cache;
    struct SwsContext *img_cvt_ctx;
    AVFrame rgb_picture;

    HwScaleContext *hws_ctx;
    int keep_alive_timeout; /* keep alive timeout setting */
    int buffer_limit;
} NetIntRoiContext;

/* class */
// int g_masks[2][3] = { { 3, 4, 5 }, { 1, 2, 3 } };
// float g_biases[] = { 10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319 };

/* human face */
static int g_masks[2][3] = {{3, 4, 5}, {0, 1, 2}};
static float g_biases[] = {10, 16, 25, 37, 49, 71, 85, 118, 143, 190, 274, 283};

static int entry_index(ni_roi_network_layer_t *l, int batch, int location,
                       int entry)
{
    int n   = location / (l->width * l->height);
    int loc = location % (l->width * l->height);
    return batch * l->output_number +
           n * l->width * l->height * (4 + l->classes + 1) +
           entry * l->width * l->height + loc;
}

static inline float sigmoid(float x)
{
    return (float)(1.0 / (1.0 + (float)exp((double)(-x))));
}

/*
 * nw: network input width
 * nh: network input height
 * lw: layer width
 * lh: layer height
 */
static box get_yolo_box(float *x, float *biases, int n, int index, int col,
                        int row, int lw, int lh, int nw, int nh, int stride)
{
    box b;

    b.x = (float)((float)col + sigmoid(x[index + 0 * stride])) / (float)lw;
    b.y = (float)((float)row + sigmoid(x[index + 1 * stride])) / (float)lh;
    b.w = (float)exp((double)x[index + 2 * stride]) * biases[2 * n] / (float)nw;
    b.h = (float)exp((double)x[index + 3 * stride]) * biases[2 * n + 1] /
          (float)nh;

    b.x -= (float)(b.w / 2.0);
    b.y -= (float)(b.h / 2.0);

    return b;
}

static int get_yolo_detections(void *ctx, ni_roi_network_layer_t *l, int netw,
                               int neth, float thresh,
                               detection_cache *det_cache, int *dets_num)
{
    int i, n, k;
    float *predictions = l->output;
    float max_prob;
    int prob_class;
    // This snippet below is not necessary
    // Need to comment it in order to batch processing >= 2 images
    // if (l.batch == 2) avg_flipped_yolo(l);
    int count       = 0;
    detection *dets = det_cache->dets;
    int row;
    int col;
    int obj_index;
    float objectness;
    int class_index;
    double prob;
    int box_index;
    box bbox;

    *dets_num = 0;

    av_log(ctx, AV_LOG_TRACE,
           "pic %dx%d, comp=%d, class=%d, net %dx%d, thresh=%f\n", l->width,
           l->height, l->component, l->classes, netw, neth, thresh);
    for (i = 0; i < l->width * l->height; ++i) {
        row = i / l->width;
        col = i % l->width;
        for (n = 0; n < l->component; ++n) {
            obj_index = entry_index(l, 0, n * l->width * l->height + i, 4);
            objectness = predictions[obj_index];
            objectness       = sigmoid(objectness);

            prob_class = -1;
            max_prob   = thresh;
            for (k = 0; k < l->classes; k++) {
                class_index =
                    entry_index(l, 0, n * l->width * l->height + i, 4 + 1 + k);
                prob = objectness * sigmoid(predictions[class_index]);
                if (prob >= max_prob) {
                    prob_class = k;
                    max_prob   = (float)prob;
                }
            }

            if (prob_class >= 0) {
                box_index =
                    entry_index(l, 0, n * l->width * l->height + i, 0);

                if (det_cache->dets_num >= det_cache->capacity) {
                    dets =
                        realloc(det_cache->dets,
                                sizeof(detection) * (det_cache->capacity + 10));
                    if (!dets) {
                        av_log(ctx, AV_LOG_ERROR,
                               "failed to realloc detections capacity %d\n",
                               det_cache->capacity);
                        return AVERROR(ENOMEM);
                    }
                    det_cache->dets = dets;
                    det_cache->capacity += 10;
                    if (det_cache->capacity >= 100) {
                        av_log(ctx, AV_LOG_WARNING, "too many detections %d\n",
                               det_cache->dets_num);
                    }
                }

                av_log(ctx, AV_LOG_TRACE, "max_prob %f, class %d\n", max_prob,
                       prob_class);
                bbox = get_yolo_box(predictions, l->biases, l->mask[n],
                                    box_index, col, row, l->width, l->height,
                                    netw, neth, l->width * l->height);

                dets[det_cache->dets_num].max_prob   = max_prob;
                dets[det_cache->dets_num].prob_class = prob_class;
                dets[det_cache->dets_num].bbox       = bbox;
                dets[det_cache->dets_num].objectness = objectness;
                dets[det_cache->dets_num].classes    = l->classes;
                dets[det_cache->dets_num].color      = n;

                av_log(ctx, AV_LOG_TRACE, "%d, x %f, y %f, w %f, h %f\n",
                       det_cache->dets_num, dets[det_cache->dets_num].bbox.x,
                       dets[det_cache->dets_num].bbox.y,
                       dets[det_cache->dets_num].bbox.w,
                       dets[det_cache->dets_num].bbox.h);
                det_cache->dets_num++;
                count++;
            }
        }
    }
    *dets_num = count;
    return 0;
}

static int nms_comparator(const void *pa, const void *pb)
{
    detection *a = (detection *)pa;
    detection *b = (detection *)pb;

    if (a->prob_class > b->prob_class)
        return 1;
    else if (a->prob_class < b->prob_class)
        return -1;
    else {
        if (a->max_prob < b->max_prob)
            return 1;
        else if (a->max_prob > b->max_prob)
            return -1;
    }
    return 0;
}

static float overlap(float x1, float w1, float x2, float w2)
{
    float l1    = x1 - w1 / 2;
    float l2    = x2 - w2 / 2;
    float left  = l1 > l2 ? l1 : l2;
    float r1    = x1 + w1 / 2;
    float r2    = x2 + w2 / 2;
    float right = r1 < r2 ? r1 : r2;
    return right - left;
}

static float box_intersection(box a, box b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    float area;

    if (w < 0 || h < 0)
        return 0;

    area = w * h;
    return area;
}

static float box_union(box a, box b)
{
    float i = box_intersection(a, b);
    float u = a.w * a.h + b.w * b.h - i;
    return u;
}

static float box_iou(box a, box b)
{
    // return box_intersection(a, b)/box_union(a, b);

    float I = box_intersection(a, b);
    float U = box_union(a, b);
    if (I == 0 || U == 0)
        return 0;

    return I / U;
}

static int nms_sort(void *ctx, detection *dets, int dets_num, float nms_thresh)
{
    int i, j;
    box boxa, boxb;
    int class;

    for (i = 0; i < (dets_num - 1); i++) {
        class = dets[i].prob_class;
        if (dets[i].max_prob == 0)
            continue;

        if (dets[i].prob_class != dets[i + 1].prob_class)
            continue;

        boxa = dets[i].bbox;
        for (j = i + 1; j < dets_num && dets[j].prob_class == class; j++) {
            if (dets[j].max_prob == 0)
                continue;

            boxb = dets[j].bbox;
            if (box_iou(boxa, boxb) > nms_thresh)
                dets[j].max_prob = 0;
        }
    }

    return 0;
}

static int resize_coords(void *ctx, detection *dets, int dets_num,
                         uint32_t img_width, uint32_t img_height,
                         struct roi_box **roi_box, int *roi_num)
{
    int i;
    int left, right, top, bot;
    struct roi_box *rbox;
    int rbox_num = 0;

    if (dets_num == 0) {
        return 0;
    }

    rbox = malloc(sizeof(struct roi_box) * dets_num);
    if (!rbox)
        return AVERROR(ENOMEM);

    for (i = 0; i < dets_num; i++) {
        av_log(ctx, AV_LOG_TRACE, "index %d, max_prob %f, class %d\n", i,
               dets[i].max_prob, dets[i].prob_class);
        if (dets[i].max_prob == 0)
            continue;

        top   = (int)floor(dets[i].bbox.y * img_height + 0.5);
        left  = (int)floor(dets[i].bbox.x * img_width + 0.5);
        right = (int)floor((dets[i].bbox.x + dets[i].bbox.w) * img_width + 0.5);
        bot = (int)floor((dets[i].bbox.y + dets[i].bbox.h) * img_height + 0.5);

        if (top < 0)
            top = 0;

        if (left < 0)
            left = 0;

        if (right > img_width)
            right = img_width;

        if (bot > img_height)
            bot = img_height;

        av_log(ctx, AV_LOG_DEBUG, "top %d, left %d, right %d, bottom %d\n", top,
               left, right, bot);

        rbox[rbox_num].left       = left;
        rbox[rbox_num].right      = right;
        rbox[rbox_num].top        = top;
        rbox[rbox_num].bottom     = bot;
        rbox[rbox_num].cls        = dets[i].prob_class;
        rbox[rbox_num].objectness = dets[i].objectness;
        rbox[rbox_num].color      = dets[i].color;
        rbox[rbox_num].prob       = dets[i].max_prob;
        rbox_num++;
    }

    if (rbox_num == 0) {
        free(rbox);
        *roi_num = rbox_num;
        *roi_box = NULL;
    } else {
        *roi_num = rbox_num;
        *roi_box = rbox;
    }

    return 0;
}

static int ni_get_detections(void *ctx, ni_roi_network_t *network,
                             detection_cache *det_cache, uint32_t img_width,
                             uint32_t img_height, float obj_thresh,
                             float nms_thresh, struct roi_box **roi_box,
                             int *roi_num)
{
    int i;
    int ret;
    int dets_num    = 0;
    detection *dets = NULL;

    *roi_box = NULL;
    *roi_num = 0;

    for (i = 0; i < network->raw.output_num; i++) {
        ret = get_yolo_detections(ctx, &network->layers[i], network->netw,
                                  network->neth, obj_thresh, det_cache,
                                  &dets_num);
        if (ret != 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "failed to get yolo detection at layer %d\n", i);
            return ret;
        }
        av_log(ctx, AV_LOG_TRACE, "layer %d, yolo detections: %d\n", i,
               dets_num);
    }

    if (det_cache->dets_num == 0)
        return 0;

    dets     = det_cache->dets;
    dets_num = det_cache->dets_num;
    for (i = 0; i < dets_num; i++) {
        av_log(ctx, AV_LOG_TRACE,
               "orig dets %d: x %f,y %f,w %f,h %f,c %d,p %f\n", i,
               dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h,
               dets[i].prob_class, dets[i].max_prob);
    }

    qsort(dets, dets_num, sizeof(detection), nms_comparator);
    for (i = 0; i < dets_num; i++) {
        av_log(ctx, AV_LOG_TRACE,
               "sorted dets %d: x %f,y %f,w %f,h %f,c %d,p %f\n", i,
               dets[i].bbox.x, dets[i].bbox.y, dets[i].bbox.w, dets[i].bbox.h,
               dets[i].prob_class, dets[i].max_prob);
    }

    nms_sort(ctx, dets, dets_num, nms_thresh);
    ret = resize_coords(ctx, dets, dets_num, img_width, img_height, roi_box,
                        roi_num);
    if (ret != 0) {
        av_log(ctx, AV_LOG_ERROR, "cannot resize coordinates\n");
        return ret;
    }

    return 0;
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_NI_QUAD,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NONE,
    };

    AVFilterFormats *formats;

    formats = ff_make_format_list(pix_fmts);
    if (!formats)
        return AVERROR(ENOMEM);

    return ff_set_common_formats(ctx, formats);
}

static void cleanup_ai_context(AVFilterContext *ctx, NetIntRoiContext *s)
{
    ni_retcode_t retval;
    AiContext *ai_ctx = s->ai_ctx;

    if (ai_ctx) {
        ni_frame_buffer_free(&ai_ctx->api_src_frame.data.frame);
        ni_packet_buffer_free(&ai_ctx->api_dst_pkt.data.packet);

        retval =
            ni_device_session_close(&ai_ctx->api_ctx, 1, NI_DEVICE_TYPE_AI);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "%s: failed to close ai session. retval %d\n", __func__,
                   retval);
        }
        if (!s->hws_ctx) {
#ifdef _WIN32
            if (ai_ctx->api_ctx.device_handle != NI_INVALID_DEVICE_HANDLE) {
                ni_device_close(ai_ctx->api_ctx.device_handle);
            }
#elif __linux__
            if (ai_ctx->api_ctx.device_handle != NI_INVALID_DEVICE_HANDLE) {
                ni_device_close(ai_ctx->api_ctx.device_handle);
            }
            if (ai_ctx->api_ctx.blk_io_handle != NI_INVALID_DEVICE_HANDLE) {
                ni_device_close(ai_ctx->api_ctx.blk_io_handle);
            }
#endif
            ai_ctx->api_ctx.device_handle = NI_INVALID_DEVICE_HANDLE;
            ai_ctx->api_ctx.blk_io_handle = NI_INVALID_DEVICE_HANDLE;
        }
        ni_device_session_context_clear(&ai_ctx->api_ctx);
        av_free(ai_ctx);
        s->ai_ctx = NULL;
    }
}

static int init_ai_context(AVFilterContext *ctx, NetIntRoiContext *s,
                           AVFrame *frame)
{
    ni_retcode_t retval;
    AiContext *ai_ctx;
    ni_roi_network_t *network = &s->network;
    int ret;
    int hwframe = frame->format == AV_PIX_FMT_NI_QUAD ? 1 : 0;
    int cardno;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;

#if HAVE_IO_H
    if ((s->nb_file == NULL) || (_access(s->nb_file, R_OK) != 0)) {
#else
    if ((s->nb_file == NULL) || (access(s->nb_file, R_OK) != 0)) {
#endif
        av_log(ctx, AV_LOG_ERROR, "invalid network binary path\n");
        return AVERROR(EINVAL);
    }

    ai_ctx = av_mallocz(sizeof(AiContext));
    if (!ai_ctx) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate ai context\n");
        return AVERROR(ENOMEM);
    }

    retval = ni_device_session_context_init(&ai_ctx->api_ctx);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "ai session context init failure\n");
        return AVERROR(EIO);
    }

    if (hwframe) {

        pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
        pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
        cardno      = ni_get_cardno(frame);

        ai_ctx->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        ai_ctx->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];
        ai_ctx->api_ctx.hw_action     = NI_CODEC_HW_ENABLE;
        ai_ctx->api_ctx.hw_id         = cardno;
    } else {
        ai_ctx->api_ctx.hw_id = s->devid;
    }

    ai_ctx->api_ctx.device_type = NI_DEVICE_TYPE_AI;
    ai_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;
    retval = ni_device_session_open(&ai_ctx->api_ctx, NI_DEVICE_TYPE_AI);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to open ai session. retval %d\n",
               retval);
        return AVERROR(EIO);
    }

    retval = ni_ai_config_network_binary(&ai_ctx->api_ctx, &network->raw,
                                         s->nb_file);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to configure ai session. retval %d\n",
               retval);
        ret = AVERROR(EIO);
        goto failed_out;
    }

    if (!hwframe) {
        retval = ni_ai_frame_buffer_alloc(&ai_ctx->api_src_frame.data.frame,
                                          &network->raw);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "failed to allocate ni frame\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }
    }

    retval = ni_ai_packet_buffer_alloc(&ai_ctx->api_dst_pkt.data.packet,
                                       &network->raw);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate ni packet\n");
        ret = AVERROR(ENOMEM);
        goto failed_out;
    }

    s->ai_ctx = ai_ctx;
    return 0;

failed_out:
    cleanup_ai_context(ctx, s);
    return ret;
}

static void ni_destroy_network(AVFilterContext *ctx, ni_roi_network_t *network)
{
    int i;
    if (network) {

        if (network->layers) {
            for (i = 0; i < network->raw.output_num; i++) {
                if (network->layers[i].output) {
                    free(network->layers[i].output);
                    network->layers[i].output = NULL;
                }
            }

            free(network->layers);
            network->layers = NULL;
        }
    }
}

static int ni_create_network(AVFilterContext *ctx, ni_roi_network_t *network)
{
    int ret;
    int i;
    ni_network_data_t *ni_network = &network->raw;

    av_log(ctx, AV_LOG_VERBOSE, "network input number %d, output number %d\n",
           ni_network->input_num, ni_network->output_num);

    if (ni_network->input_num == 0 || ni_network->output_num == 0) {
        av_log(ctx, AV_LOG_ERROR, "invalid network layer\n");
        return AVERROR(EINVAL);
    }

    /* only support one input for now */
    if (ni_network->input_num != 1) {
        av_log(ctx, AV_LOG_ERROR,
               "network input layer number %d not supported\n",
               ni_network->input_num);
        return AVERROR(EINVAL);
    }

    network->layers =
        malloc(sizeof(ni_roi_network_layer_t) * ni_network->output_num);
    if (!network->layers) {
        av_log(ctx, AV_LOG_ERROR, "cannot allocate network layer memory\n");
        return AVERROR(ENOMEM);
    }
    memset(network->layers, 0,
           sizeof(ni_roi_network_layer_t) * ni_network->output_num);

    for (i = 0; i < ni_network->output_num; i++) {
        network->layers[i].width     = ni_network->linfo.out_param[i].sizes[0];
        network->layers[i].height    = ni_network->linfo.out_param[i].sizes[1];
        network->layers[i].channel   = ni_network->linfo.out_param[i].sizes[2];
        network->layers[i].component = 3;
        network->layers[i].classes =
            (network->layers[i].channel / network->layers[i].component) -
            (4 + 1);
        network->layers[i].output_number =
            ni_ai_network_layer_dims(&ni_network->linfo.out_param[i]);
        av_assert0(network->layers[i].output_number ==
                   network->layers[i].width * network->layers[i].height *
                       network->layers[i].channel);

        network->layers[i].output =
            malloc(network->layers[i].output_number * sizeof(float));
        if (!network->layers[i].output) {
            av_log(ctx, AV_LOG_ERROR,
                   "failed to allocate network layer %d output buffer\n", i);
            ret = AVERROR(ENOMEM);
            goto out;
        }
        memcpy(network->layers[i].mask, &g_masks[i][0],
               sizeof(network->layers[i].mask));
        memcpy(network->layers[i].biases, &g_biases[0],
               sizeof(network->layers[i].biases));

        av_log(ctx, AV_LOG_DEBUG,
               "network layer %d: w %d, h %d, ch %d, co %d, cl %d\n", i,
               network->layers[i].width, network->layers[i].height,
               network->layers[i].channel, network->layers[i].component,
               network->layers[i].classes);
    }

    network->netw = ni_network->linfo.in_param[0].sizes[0];
    network->neth = ni_network->linfo.in_param[0].sizes[1];

    return 0;
out:
    ni_destroy_network(ctx, network);
    return ret;
}

static av_cold int init_hwframe_scale(AVFilterContext *ctx, NetIntRoiContext *s,
                                      enum AVPixelFormat format, AVFrame *frame)
{
    ni_retcode_t retval;
    HwScaleContext *hws_ctx;
    int ret;
    AVHWFramesContext *pAVHFWCtx;
    AVNIDeviceContext *pAVNIDevCtx;
    int cardno;
    int pool_size = DEFAULT_NI_FILTER_POOL_SIZE;

    hws_ctx = av_mallocz(sizeof(HwScaleContext));
    if (!hws_ctx) {
        av_log(ctx, AV_LOG_ERROR, "could not allocate hwframe ctx\n");
        return AVERROR(ENOMEM);
    }

    retval = ni_device_session_context_init(&hws_ctx->api_ctx);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "hw scaler session context init failure\n");
        return AVERROR(EIO);
    }

    pAVHFWCtx   = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    pAVNIDevCtx = (AVNIDeviceContext *)pAVHFWCtx->device_ctx->hwctx;
    cardno      = ni_get_cardno(frame);

    hws_ctx->api_ctx.device_handle     = pAVNIDevCtx->cards[cardno];
    hws_ctx->api_ctx.blk_io_handle     = pAVNIDevCtx->cards[cardno];
    hws_ctx->api_ctx.device_type       = NI_DEVICE_TYPE_SCALER;
    hws_ctx->api_ctx.scaler_operation  = NI_SCALER_OPCODE_SCALE;
    hws_ctx->api_ctx.hw_id             = cardno;
    hws_ctx->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    retval = ni_device_session_open(&hws_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "could not open scaler session\n");
        ret = AVERROR(EIO);
        goto out;
    }

#if IS_FFMPEG_71_AND_ABOVE
    pool_size += ctx->extra_hw_frames > 0 ? ctx->extra_hw_frames : 0;
#endif
#if IS_FFMPEG_61_AND_ABOVE
    s->buffer_limit = 1;
#endif
    /* Create scale frame pool on device */
    retval = ff_ni_build_frame_pool(&hws_ctx->api_ctx, s->network.netw,
                                    s->network.neth, format,
                                    pool_size,
                                    s->buffer_limit);
    if (retval < 0) {
        av_log(ctx, AV_LOG_ERROR, "could not build frame pool\n");
        ni_device_session_close(&hws_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hws_ctx->api_ctx);
        ret = AVERROR(EIO);
        goto out;
    }

    s->hws_ctx = hws_ctx;
    return 0;
out:
    av_free(hws_ctx);
    return ret;
}

static void cleanup_hwframe_scale(AVFilterContext *ctx, NetIntRoiContext *s)
{
    HwScaleContext *hws_ctx = s->hws_ctx;

    if (hws_ctx) {
        ni_frame_buffer_free(&hws_ctx->api_dst_frame.data.frame);
        ni_device_session_close(&hws_ctx->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&hws_ctx->api_ctx);

        av_free(hws_ctx);
        s->hws_ctx = NULL;
    }
}

static av_cold int init(AVFilterContext *ctx)
{
    NetIntRoiContext *s = ctx->priv;

    s->det_cache.dets_num = 0;
    s->det_cache.capacity = 20;
    s->det_cache.dets     = malloc(sizeof(detection) * s->det_cache.capacity);
    if (!s->det_cache.dets) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate detection cache\n");
        return AVERROR(ENOMEM);
    }

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntRoiContext *s       = ctx->priv;
    ni_roi_network_t *network = &s->network;

    cleanup_ai_context(ctx, s);

    ni_destroy_network(ctx, network);

    if (s->det_cache.dets) {
        free(s->det_cache.dets);
        s->det_cache.dets = NULL;
    }

    av_buffer_unref(&s->out_frames_ref);
    s->out_frames_ref = NULL;

    av_frame_unref(&s->rgb_picture);
    sws_freeContext(s->img_cvt_ctx);
    s->img_cvt_ctx = NULL;

    cleanup_hwframe_scale(ctx, s);
}

static int config_input(AVFilterContext *ctx, AVFrame *frame)
{
    NetIntRoiContext *s = ctx->priv;
    int ret;
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    AVNIFramesContext *out_ni_ctx;

    if (s->initialized)
        return 0;

    ret = init_ai_context(ctx, s, frame);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to initialize ai context\n");
        return ret;
    }

    ret = ni_create_network(ctx, &s->network);
    if (ret != 0) {
        goto fail_out;
    }

    if (frame->format != AV_PIX_FMT_NI_QUAD) {
        memset(&s->rgb_picture, 0, sizeof(s->rgb_picture));
        s->rgb_picture.width  = s->network.netw;
        s->rgb_picture.height = s->network.neth;
        s->rgb_picture.format = AV_PIX_FMT_RGB24;
        if (av_frame_get_buffer(&s->rgb_picture, 32)) {
            av_log(ctx, AV_LOG_ERROR, "Out of memory for RGB pack data!\n");
            goto fail_out;
        }

        s->img_cvt_ctx = sws_getContext(frame->width, frame->height,
                                        frame->format, s->network.netw,
                                        s->network.neth, s->rgb_picture.format,
                                        SWS_BICUBIC, NULL, NULL, NULL);
        if (!s->img_cvt_ctx) {
            av_log(ctx, AV_LOG_ERROR,
                   "could not create SwsContext for conversion and scaling\n");
            ret = AVERROR(ENOMEM);
            goto fail_out;
        }
    } else {
        ret = init_hwframe_scale(ctx, s, AV_PIX_FMT_BGRP, frame);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "could not initialize hwframe scale context\n");
            goto fail_out;
        }

        in_frames_ctx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
        out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
        out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
        ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx);
        ni_device_session_copy(&s->ai_ctx->api_ctx, &out_ni_ctx->api_ctx);
    }

    s->initialized = 1;
    return 0;

fail_out:
    cleanup_ai_context(ctx, s);

    ni_destroy_network(ctx, &s->network);

    av_frame_unref(&s->rgb_picture);
    if (s->img_cvt_ctx) {
        sws_freeContext(s->img_cvt_ctx);
        s->img_cvt_ctx = NULL;
    }
    return ret;
}

static int output_config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    NetIntRoiContext *s = ctx->priv;
    AVNIFramesContext *out_ni_ctx;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if ((li->hw_frames_ctx == NULL) && (inlink->format == AV_PIX_FMT_NI_QUAD)) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }

    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_DEBUG, "sw frame\n");
        return 0;
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#else
    if ((inlink->hw_frames_ctx == NULL) && (inlink->format == AV_PIX_FMT_NI_QUAD)) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }

    if (inlink->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_DEBUG, "sw frame\n");
        return 0;
    }
    in_frames_ctx = (AVHWFramesContext *)ctx->inputs[0]->hw_frames_ctx->data;
#endif
    outlink->w = inlink->w;
    outlink->h = inlink->h;

    if (in_frames_ctx->sw_format == AV_PIX_FMT_BGRP) {
        av_log(ctx, AV_LOG_ERROR, "bgrp not supported\n");
        return AVERROR(EINVAL);
    }
    if (in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_8_TILE_4X4 ||
        in_frames_ctx->sw_format == AV_PIX_FMT_NI_QUAD_10_TILE_4X4) {
        av_log(ctx, AV_LOG_ERROR, "tile4x4 not supported\n");
        return AVERROR(EINVAL);
    }

    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    out_ni_ctx = (AVNIFramesContext *)out_frames_ctx->hwctx;
    ni_cpy_hwframe_ctx(in_frames_ctx, out_frames_ctx);
    ni_device_session_copy(&s->ai_ctx->api_ctx, &out_ni_ctx->api_ctx);

    out_frames_ctx->format            = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width             = outlink->w;
    out_frames_ctx->height            = outlink->h;
    out_frames_ctx->sw_format         = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size = NI_ROI_ID;

    av_hwframe_ctx_init(s->out_frames_ref);

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
}

static int ni_read_roi(AVFilterContext *ctx, ni_session_data_io_t *p_dst_pkt,
                       AVFrame *out, int pic_width, int pic_height)
{
    NetIntRoiContext *s = ctx->priv;
    ni_retcode_t retval;
    ni_roi_network_t *network = &s->network;
    AVFrameSideData *sd;
    AVFrameSideData *sd_roi_extra;
    AVRegionOfInterest *roi;
    AVRegionOfInterestNetintExtra *roi_extra;
    struct roi_box *roi_box = NULL;
    int roi_num = 0;
    int ret;
    int i;
    int width, height;

    for (i = 0; i < network->raw.output_num; i++) {
        retval = ni_network_layer_convert_output(
            network->layers[i].output,
            network->layers[i].output_number * sizeof(float),
            &p_dst_pkt->data.packet, &network->raw, i);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "failed to read layer %d output. retval %d\n", i, retval);
            return AVERROR(EIO);
        }
    }

    width  = pic_width;
    height = pic_height;

    s->det_cache.dets_num = 0;
    ret = ni_get_detections(ctx, network, &s->det_cache, width, height,
                            s->obj_thresh, s->nms_thresh, &roi_box, &roi_num);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to get roi.\n");
        return ret;
    }

    if (roi_num == 0) {
        av_log(ctx, AV_LOG_DEBUG, "no roi available\n");
        return 0;
    }

    sd = av_frame_new_side_data(out, AV_FRAME_DATA_REGIONS_OF_INTEREST,
                                (int)(roi_num * sizeof(AVRegionOfInterest)));
    sd_roi_extra = av_frame_new_side_data(
        out, AV_FRAME_DATA_NETINT_REGIONS_OF_INTEREST_EXTRA,
        (int)(roi_num * sizeof(AVRegionOfInterestNetintExtra)));
    if (!sd || !sd_roi_extra) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate roi sidedata\n");
        free(roi_box);
        return AVERROR(ENOMEM);
    }

    roi = (AVRegionOfInterest *)sd->data;
    roi_extra = (AVRegionOfInterestNetintExtra *)sd_roi_extra->data;
    for (i = 0; i < roi_num; i++) {
        roi[i].self_size = sizeof(*roi);
        roi[i].top       = roi_box[i].top;
        roi[i].bottom    = roi_box[i].bottom;
        roi[i].left      = roi_box[i].left;
        roi[i].right     = roi_box[i].right;
        roi[i].qoffset   = s->qp_offset;
        roi_extra[i].self_size = sizeof(*roi_extra);
        roi_extra[i].cls       = roi_box[i].cls;
        roi_extra[i].prob      = roi_box[i].prob;
        av_log(ctx, AV_LOG_DEBUG,
               "roi %d: top %d, bottom %d, left %d, right %d, qpo %d/%d\n", i,
               roi[i].top, roi[i].bottom, roi[i].left, roi[i].right,
               roi[i].qoffset.num, roi[i].qoffset.den);
    }

    free(roi_box);
    return 0;
}

static int ni_recreate_frame(ni_frame_t *ni_frame, AVFrame *frame)
{
    uint8_t *p_data = ni_frame->p_data[0];
    int i;
    uint8_t *r_data ;
    uint8_t *g_data ;
    uint8_t *b_data ;
    uint8_t *fdata  ;
    int x, y;
    int fpos;
    int ppos;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    av_log(NULL, AV_LOG_DEBUG,
           "linesize %d/%d/%d, data %p/%p/%p, pixel %dx%d\n",
           frame->linesize[0], frame->linesize[1], frame->linesize[2],
           frame->data[0], frame->data[1], frame->data[2], frame->width,
           frame->height);

    if (frame->format == AV_PIX_FMT_GBRP) {
        /* GBRP -> BGRP */
        for (i = 0; i < frame->height; i++) {
            memcpy((void *)(p_data + i * frame->linesize[1]),
                   frame->data[1] + i * frame->linesize[1], frame->linesize[1]);
        }

        p_data += frame->height * frame->linesize[1];
        for (i = 0; i < frame->height; i++) {
            memcpy((void *)(p_data + i * frame->linesize[0]),
                   frame->data[0] + i * frame->linesize[0], frame->linesize[0]);
        }

        p_data += frame->height * frame->linesize[0];
        for (i = 0; i < frame->height; i++) {
            memcpy((void *)(p_data + i * frame->linesize[2]),
                   frame->data[2] + i * frame->linesize[2], frame->linesize[2]);
        }
    } else if (frame->format == AV_PIX_FMT_RGB24) {
        /* RGB24 -> BGRP */
        r_data = p_data + frame->width * frame->height * 2;
        g_data = p_data + frame->width * frame->height * 1;
        b_data = p_data + frame->width * frame->height * 0;
        fdata  = frame->data[0];

        av_log(NULL, AV_LOG_DEBUG,
               "%s(): rgb24 to bgrp, pix %dx%d, linesize %d\n", __func__,
               frame->width, frame->height, frame->linesize[0]);

        for (y = 0; y < frame->height; y++) {
            for (x = 0; x < frame->width; x++) {
                fpos  = y * frame->linesize[0];
                ppos  = y * frame->width;
                r = fdata[fpos + x * 3 + 0];
                g = fdata[fpos + x * 3 + 1];
                b = fdata[fpos + x * 3 + 2];

                r_data[ppos + x] = r;
                g_data[ppos + x] = g;
                b_data[ppos + x] = b;
            }
        }
    }
    return 0;
}

static int ni_hwframe_scale(AVFilterContext *ctx, NetIntRoiContext *s,
                            AVFrame *in, int w, int h,
                            niFrameSurface1_t **filt_frame_surface)
{
    HwScaleContext *scale_ctx = s->hws_ctx;
    int scaler_format;
    ni_retcode_t retcode;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    AVHWFramesContext *pAVHFWCtx;

    frame_surface = (niFrameSurface1_t *)in->data[3];

    av_log(ctx, AV_LOG_DEBUG, "in frame surface frameIdx %d\n",
           frame_surface->ui16FrameIdx);

    pAVHFWCtx = (AVHWFramesContext *)in->hw_frames_ctx->data;

    scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(pAVHFWCtx->sw_format);

    retcode = ni_frame_buffer_alloc_hwenc(&scale_ctx->api_dst_frame.data.frame,
                                          w, h, 0);
    if (retcode != NI_RETCODE_SUCCESS)
        return AVERROR(ENOMEM);

    /*
     * Allocate device input frame. This call won't actually allocate a frame,
     * but sends the incoming hardware frame index to the scaler manager
     */
    retcode = ni_device_alloc_frame(
        &scale_ctx->api_ctx, FFALIGN(in->width, 2), FFALIGN(in->height, 2),
        scaler_format, 0, 0, 0, 0, 0, frame_surface->ui32nodeAddress,
        frame_surface->ui16FrameIdx, NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(NULL, AV_LOG_DEBUG, "Can't allocate device input frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Allocate hardware device destination frame. This acquires a frame from
     * the pool */
    retcode = ni_device_alloc_frame(
        &scale_ctx->api_ctx, FFALIGN(w, 2), FFALIGN(h, 2),
        ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_BGRP), NI_SCALER_FLAG_IO, 0, 0,
        0, 0, 0, -1, NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(NULL, AV_LOG_DEBUG, "Can't allocate device output frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    /* Set the new frame index */
    ni_device_session_read_hwdesc(
        &scale_ctx->api_ctx, &scale_ctx->api_dst_frame, NI_DEVICE_TYPE_SCALER);
    new_frame_surface =
        (niFrameSurface1_t *)scale_ctx->api_dst_frame.data.frame.p_data[3];

    *filt_frame_surface = new_frame_surface;

    return 0;
}

static int filter_frame(AVFilterLink *link, AVFrame *in)
{
    AVFilterContext *ctx = link->dst;
    NetIntRoiContext *s  = ctx->priv;
    AVFrame *out         = NULL;
    ni_roi_network_t *network;
    ni_retcode_t retval;
    int ret;
    AiContext *ai_ctx;

    if (in == NULL) {
        av_log(ctx, AV_LOG_WARNING, "in frame is null\n");
        return AVERROR(EINVAL);
    }

    if (!s->initialized) {
#if IS_FFMPEG_71_AND_ABOVE
        AVFilterLink *outlink = link->dst->outputs[0];
        if (!((av_strstart(outlink->dst->filter->name, "ni_quadra", NULL)) || (av_strstart(outlink->dst->filter->name, "hwdownload", NULL)))) {
           ctx->extra_hw_frames = (DEFAULT_FRAME_THREAD_QUEUE_SIZE > 1) ? DEFAULT_FRAME_THREAD_QUEUE_SIZE : 0;
        }
#endif
        ret = config_input(ctx, in);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed to config input\n");
            goto failed_out;
        }
    }

    ai_ctx  = s->ai_ctx;
    network = &s->network;
    retval  = ni_ai_packet_buffer_alloc(&ai_ctx->api_dst_pkt.data.packet,
                                       &network->raw);
    if (retval != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate packet\n");
        ret = AVERROR(EAGAIN);
        goto failed_out;
    }

    out = av_frame_clone(in);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto failed_out;
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    if (in->format == AV_PIX_FMT_NI_QUAD) {
        niFrameSurface1_t *filt_frame_surface;

        ret = ni_hwframe_scale(ctx, s, in, network->netw, network->neth,
                               &filt_frame_surface);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Error run hwframe scale\n");
            goto failed_out;
        }

        av_log(ctx, AV_LOG_DEBUG, "filt frame surface frameIdx %d\n",
               filt_frame_surface->ui16FrameIdx);

        /* allocate output buffer */
        retval = ni_device_alloc_frame(&ai_ctx->api_ctx, 0, 0, 0, 0, 0, 0, 0, 0,
                                       filt_frame_surface->ui32nodeAddress,
                                       filt_frame_surface->ui16FrameIdx,
                                       NI_DEVICE_TYPE_AI);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "failed to alloc hw input frame\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }

        do {
            retval = ni_device_session_read(
                &ai_ctx->api_ctx, &ai_ctx->api_dst_pkt, NI_DEVICE_TYPE_AI);
            if (retval < 0) {
                av_log(ctx, AV_LOG_ERROR, "read hwdesc retval %d\n", retval);
                ret = AVERROR(EIO);
                goto failed_out;
            } else if (retval > 0) {
                ret = ni_read_roi(ctx, &ai_ctx->api_dst_pkt, out, out->width,
                                  out->height);
                if (ret != 0) {
                    av_log(ctx, AV_LOG_ERROR,
                           "failed to read roi from packet\n");
                    goto failed_out;
                }
            }
        } while (retval == 0);

        ni_hwframe_buffer_recycle(filt_frame_surface,
                                  filt_frame_surface->device_handle);

        av_buffer_unref(&out->hw_frames_ctx);
        /* Reference the new hw frames context */
        out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    } else {
        ret = sws_scale(s->img_cvt_ctx, (const uint8_t *const *)in->data,
                        in->linesize, 0, in->height, s->rgb_picture.data,
                        s->rgb_picture.linesize);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "failed to do sws scale\n");
            goto failed_out;
        }

        retval = ni_ai_frame_buffer_alloc(&ai_ctx->api_src_frame.data.frame,
                                          &network->raw);
        if (retval != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "cannot allocate ai frame\n");
            ret = AVERROR(ENOMEM);
            goto failed_out;
        }

        ret = ni_recreate_frame(&ai_ctx->api_src_frame.data.frame,
                                &s->rgb_picture);
        if (ret != 0) {
            av_log(ctx, AV_LOG_ERROR, "cannot re-create ai frame\n");
            goto failed_out;
        }

        /* write frame */
        do {
            retval = ni_device_session_write(
                &ai_ctx->api_ctx, &ai_ctx->api_src_frame, NI_DEVICE_TYPE_AI);
            if (retval < 0) {
                av_log(ctx, AV_LOG_ERROR,
                       "failed to write ai session: retval %d\n", retval);
                ret = AVERROR(EIO);
                goto failed_out;
            }
        } while (retval == 0);

        /* read roi result */
        do {
            retval = ni_device_session_read(
                &ai_ctx->api_ctx, &ai_ctx->api_dst_pkt, NI_DEVICE_TYPE_AI);
            if (retval < 0) {
                av_log(ctx, AV_LOG_ERROR, "read hwdesc retval %d\n", retval);
                ret = AVERROR(EIO);
                goto failed_out;
            } else if (retval > 0) {
                ret = ni_read_roi(ctx, &ai_ctx->api_dst_pkt, out, out->width,
                                  out->height);
                if (ret != 0) {
                    av_log(ctx, AV_LOG_ERROR,
                           "failed to read roi from packet\n");
                    goto failed_out;
                }
            }
        } while (retval == 0);
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_roi");
#endif

    av_frame_free(&in);
    return ff_filter_frame(link->dst->outputs[0], out);

failed_out:
    if (out)
        av_frame_free(&out);

    av_frame_free(&in);
    return ret;
}

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    AVFrame *frame = NULL;
    int ret = 0;
    NetIntRoiContext *s = inlink->dst->priv;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        if (s->initialized) {
            if (s->hws_ctx) {
                ret = ni_device_session_query_buffer_avail(&s->hws_ctx->api_ctx, NI_DEVICE_TYPE_SCALER);
            }
        }

        if (ret == NI_RETCODE_ERROR_UNSUPPORTED_FW_VERSION) {
            av_log(ctx, AV_LOG_WARNING, "No backpressure support in FW\n");
        } else if (ret < 0) {
            av_log(ctx, AV_LOG_WARNING, "%s: query ret %d, ready %u inlink framequeue %u available_frame %d outlink framequeue %u frame_wanted %d - return NOT READY\n",
                __func__, ret, ctx->ready, ff_inlink_queued_frames(inlink), ff_inlink_check_available_frame(inlink), ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));
            return FFERROR_NOT_READY;
        }

        ret = ff_inlink_consume_frame(inlink, &frame);
        if (ret < 0)
            return ret;

        ret = filter_frame(inlink, frame);
        if (ret >= 0) {
            ff_filter_set_ready(ctx, 300);
        }
        return ret;
    }

    // We did not get a frame from input link, check its status
    FF_FILTER_FORWARD_STATUS(inlink, outlink);

    // We have no frames yet from input link and no EOF, so request some.
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}
#endif

#define OFFSET(x) offsetof(NetIntRoiContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_FILTERING_PARAM)

static const AVOption ni_roi_options[] = {
    { "nb",         "path to network binary file",              OFFSET(nb_file),    AV_OPT_TYPE_STRING,   {.str = NULL}, 0,        0,        FLAGS },
    { "qpoffset",   "qp offset ratio",                          OFFSET(qp_offset),  AV_OPT_TYPE_RATIONAL, {.dbl = 0},    -1.0,     1.0,      FLAGS, "range" },
    { "devid",      "device to operate in swframe mode",        OFFSET(devid),      AV_OPT_TYPE_INT,      {.i64 = 0},    -1,       INT_MAX,  FLAGS, "range" },
    { "obj_thresh", "objectness threshold",                     OFFSET(obj_thresh), AV_OPT_TYPE_FLOAT,    {.dbl = 0.25}, -FLT_MAX, FLT_MAX,  FLAGS, "range" },
    { "nms_thresh", "yolov4 non-maximum suppression threshold", OFFSET(nms_thresh), AV_OPT_TYPE_FLOAT,    {.dbl = 0.45}, -FLT_MAX, FLT_MAX,  FLAGS, "range" },
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
{NULL}};

AVFILTER_DEFINE_CLASS(ni_roi);

static const AVFilterPad inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = output_config_props,
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    {NULL}
#endif
};

AVFilter ff_vf_roi_ni_quadra = {
    .name           = "ni_quadra_roi",
    .description    = NULL_IF_CONFIG_SMALL("NETINT Quadra video roi v" NI_XCODER_REVISION),
    .init           = init,
    .uninit         = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate       = activate,
#endif
    .priv_size      = sizeof(NetIntRoiContext),
    .priv_class     = &ni_roi_class,
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
    FILTER_QUERY_FUNC(query_formats),
#else
    .inputs         = inputs,
    .outputs        = outputs,
    .query_formats  = query_formats,
#endif
};
