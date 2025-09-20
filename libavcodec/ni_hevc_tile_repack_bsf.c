/*
 * NetInt HEVC tile repack BSF common source code
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

/**
 * @file
 *
 * This bitstream filter repacks HEVC tiles into one packet containing
 * just one frame.
 */

#include "version.h"

#include "libavutil/avassert.h"
#include "libavutil/opt.h"

#include "avcodec.h"

#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 91)
#include "bsf_internal.h"
#else
#include "bsf.h"
#endif

#if (LIBAVCODEC_VERSION_MAJOR > 59 || LIBAVCODEC_VERSION_MAJOR >= 59 && LIBAVCODEC_VERSION_MINOR >= 37)
#include "startcode.h"
#else
#include "internal.h"
#endif

#if ((LIBAVCODEC_VERSION_MAJOR > 61) || (LIBAVCODEC_VERSION_MAJOR == 61 && LIBAVCODEC_VERSION_MINOR >= 19))
#include "hevc/hevc.h"
#include "libavutil/mem.h"
#else
#include "hevc.h"
#endif

typedef struct HEVCRepackContext {
    AVPacket *buffer_pkt;
    AVPacket **tile_pkt;
    int tile_pos;
    int tile_num;
} HEVCRepackContext;

static int hevc_tile_repack_filter(AVBSFContext *ctx, AVPacket *out) {
    HEVCRepackContext *s = ctx->priv_data;
    int ret;
    int tile_idx;
    int *side_data;
    int i;

    av_log(ctx, AV_LOG_DEBUG, "tile_pos %d, tile_num %d\n", s->tile_pos,
           s->tile_num);

    if (s->tile_pos < s->tile_num) {
        if (!s->buffer_pkt->data) {
            ret = ff_bsf_get_packet_ref(ctx, s->buffer_pkt);
            if (ret < 0) {
                av_log(ctx, AV_LOG_INFO, "failed to get packet ref: 0x%x\n",
                       ret);
                return ret;
            }
        }

        side_data = (int *)av_packet_get_side_data(
            s->buffer_pkt, AV_PKT_DATA_SLICE_ADDR, NULL);
        if (!side_data) {
            av_log(ctx, AV_LOG_ERROR, "failed to get packet side data\n");
            return AVERROR(EINVAL);
        }

        tile_idx = *side_data;
        if (tile_idx >= s->tile_num) {
            av_log(ctx, AV_LOG_ERROR,
                   "tile index %d exceeds maximum tile number %d\n", tile_idx,
                   s->tile_num);
            return AVERROR(EINVAL);
        }

        if (s->tile_pkt[tile_idx]->buf) {
            av_log(ctx, AV_LOG_ERROR, "duplicated tile index %d\n", tile_idx);
            return AVERROR(EINVAL);
        }

        s->tile_pkt[tile_idx]->buf = av_buffer_ref(s->buffer_pkt->buf);
        if (!s->tile_pkt[tile_idx]->buf) {
            av_log(ctx, AV_LOG_ERROR,
                   "failed to get buffer for tile index %d\n", tile_idx);
            return AVERROR(ENOMEM);
        }
        s->tile_pkt[tile_idx]->data = s->buffer_pkt->data;
        s->tile_pkt[tile_idx]->size = s->buffer_pkt->size;

        av_log(ctx, AV_LOG_DEBUG, "tile %d, data actual size %d\n", tile_idx,
               s->buffer_pkt->size);

        if (s->tile_pos == 0) {
            s->tile_pkt[0]->pts          = s->buffer_pkt->pts;
            s->tile_pkt[0]->dts          = s->buffer_pkt->dts;
            s->tile_pkt[0]->pos          = s->buffer_pkt->pos;
            s->tile_pkt[0]->flags        = s->buffer_pkt->flags;
            s->tile_pkt[0]->stream_index = s->buffer_pkt->stream_index;

            s->tile_pkt[0]->side_data       = NULL;
            s->tile_pkt[0]->side_data_elems = 0;

            for (i = 0; i < s->tile_pkt[0]->side_data_elems; i++) {
                enum AVPacketSideDataType type =
                    s->buffer_pkt->side_data[i].type;
                if (type != AV_PKT_DATA_SLICE_ADDR) {
                    int size          = s->buffer_pkt->side_data[i].size;
                    uint8_t *src_data = s->buffer_pkt->side_data[i].data;
                    uint8_t *dst_data =
                        av_packet_new_side_data(s->tile_pkt[0], type, size);

                    if (!dst_data) {
                        av_packet_free_side_data(s->tile_pkt[0]);
                        return AVERROR(ENOMEM);
                    }

                    memcpy(dst_data, src_data, size);
                }
            }
        } else {
            if (s->buffer_pkt->pts != s->tile_pkt[0]->pts ||
                s->buffer_pkt->dts != s->tile_pkt[0]->dts ||
                s->buffer_pkt->flags != s->tile_pkt[0]->flags ||
                s->buffer_pkt->stream_index != s->tile_pkt[0]->stream_index) {
                av_log(ctx, AV_LOG_ERROR, "packet metadata does not match\n");
                return AVERROR(EINVAL);
            }
        }
        s->tile_pos++;
        av_packet_unref(s->buffer_pkt);
    }

    if (s->tile_pos == s->tile_num) {
        int new_size = 0;
        int found;
        const uint8_t *ptr;
        const uint8_t *end;
        const uint8_t *p_offset;
        uint32_t nalu_type;
        uint32_t stc;
        uint8_t *data;
        AVBufferRef *buf;

        /* max payload size */
        for (i = 0; i < s->tile_num; i++) {
            new_size += s->tile_pkt[i]->size;
        }

        buf = av_buffer_alloc(new_size);
        if (!buf) {
            av_log(ctx, AV_LOG_ERROR, "failed to allocate new packet data\n");
            return AVERROR(ENOMEM);
        }

        data = buf->data;
        memcpy(data, s->tile_pkt[0]->data, s->tile_pkt[0]->size);
        new_size = s->tile_pkt[0]->size;
        av_log(ctx, AV_LOG_DEBUG, "tile %d size %d\n", 0, new_size);
        av_buffer_unref(&s->tile_pkt[0]->buf);
        s->tile_pkt[0]->buf = NULL;

        for (i = 1; i < s->tile_num; i++) {
            ptr = s->tile_pkt[i]->data;
            end = s->tile_pkt[i]->data + s->tile_pkt[i]->size;

            stc   = -1;
            found = 0;
            ptr   = (const uint8_t *) avpriv_find_start_code(ptr, end, &stc);
            while (ptr < end) {
                av_log(ctx, AV_LOG_DEBUG, "tile %d, %02x %02x %02x %02x %02x\n",
                       i, *(ptr - 4), *(ptr - 3), *(ptr - 2), *(ptr - 1), *ptr);

                if (found) {
                    memcpy(data + new_size, p_offset, ptr - 4 - p_offset);
                    new_size += (int)(ptr - 4 - p_offset);
                    found = 0;
                }

                nalu_type = (stc >> 1) & 0x3F;
                if (nalu_type <= HEVC_NAL_RSV_VCL31) {
                    p_offset = ptr - 4;
                    found    = 1;
                }

                stc = -1;
                ptr = (const uint8_t *) avpriv_find_start_code(ptr, end, &stc);
            }

            if (found) {
                memcpy(data + new_size, p_offset, end - p_offset);
                new_size += (int)(end - p_offset);
                av_log(ctx, AV_LOG_DEBUG, "tile %d size %d\n", i,
                       (int)(end - p_offset));
            }
            av_buffer_unref(&s->tile_pkt[i]->buf);
            s->tile_pkt[i]->buf = NULL;
        }

        out->buf  = buf;
        out->data = data;
        out->size = new_size;

        av_log(ctx, AV_LOG_DEBUG, "repacket new size %d\n", new_size);

        s->tile_pos = 0;
        return 0;
    } else {
        return AVERROR(EAGAIN);
    }
}

static int hevc_tile_repack_init(AVBSFContext *ctx) {
    HEVCRepackContext *s = ctx->priv_data;
    int ret;
    int i;

    av_log(ctx, AV_LOG_INFO, "number of tiles %d\n", s->tile_num);
    if (s->tile_num <= 0) {
        return AVERROR(EINVAL);
    }

    s->buffer_pkt = av_packet_alloc();
    if (!s->buffer_pkt) {
        return AVERROR(ENOMEM);
    }

    s->tile_pkt = av_malloc(sizeof(AVPacket *) * s->tile_num);
    if (!s->tile_pkt) {
        ret = AVERROR(ENOMEM);
        goto fail_alloc_tile_pkt;
    }
    memset(s->tile_pkt, 0, sizeof(AVPacket *) * s->tile_num);

    for (i = 0; i < s->tile_num; i++) {
        s->tile_pkt[i] = av_packet_alloc();
        if (!s->tile_pkt[i]) {
            ret = AVERROR(ENOMEM);
            goto fail_alloc_pkts;
        }
    }

    return 0;

fail_alloc_pkts:
    for (i -= 1; i >= 0; i--) {
        av_packet_free(&s->tile_pkt[i]);
    }
    av_freep(&s->tile_pkt);

fail_alloc_tile_pkt:
    av_packet_free(&s->buffer_pkt);

    return ret;
}

static void hevc_tile_repack_flush(AVBSFContext *ctx) {
    HEVCRepackContext *s = ctx->priv_data;
    int i;

    av_packet_unref(s->buffer_pkt);

    for (i = 0; i < s->tile_num; i++) {
        av_packet_unref(s->tile_pkt[i]);
    }
}

static void hevc_tile_repack_close(AVBSFContext *ctx) {
    HEVCRepackContext *s = ctx->priv_data;
    int i;

    av_packet_free(&s->buffer_pkt);

    for (i = 0; i < s->tile_num; i++) {
        av_packet_free(&s->tile_pkt[i]);
    }
    av_freep(&s->tile_pkt);
}

static const enum AVCodecID hevc_tile_repack_codec_ids[] = {
    AV_CODEC_ID_HEVC,
    AV_CODEC_ID_NONE,
};

#define OFFSET(x) offsetof(HEVCRepackContext, x)
#define FLAGS (AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_BSF_PARAM)
static const AVOption options[] = {
    {"tile_num",
     "specify number of tiles",
     OFFSET(tile_num),
     AV_OPT_TYPE_INT,
     {.i64 = 0},
     0,
     INT_MAX,
     FLAGS},
    {NULL},
};

static const AVClass tile_repack_class = {
    .class_name = "hevc_tile_repack_bsf",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const
#if (LIBAVCODEC_VERSION_MAJOR > 59 || LIBAVCODEC_VERSION_MAJOR >= 59 && LIBAVCODEC_VERSION_MINOR >= 37)
FFBitStreamFilter
#else
AVBitStreamFilter
#endif
ff_hevc_tile_repack_bsf = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || LIBAVCODEC_VERSION_MAJOR >= 59 && LIBAVCODEC_VERSION_MINOR >= 37)
    .p.name           = "hevc_tile_repack",
    .p.codec_ids      = hevc_tile_repack_codec_ids,
    .p.priv_class     = &tile_repack_class,
#else
    .name           = "hevc_tile_repack",
    .codec_ids      = hevc_tile_repack_codec_ids,
    .priv_class     = &tile_repack_class,
#endif
    .priv_data_size = sizeof(HEVCRepackContext),
    .init           = hevc_tile_repack_init,
    .flush          = hevc_tile_repack_flush,
    .close          = hevc_tile_repack_close,
    .filter         = hevc_tile_repack_filter,
};
