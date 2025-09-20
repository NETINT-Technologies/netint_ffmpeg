/*
 * NetInt AV1 tile repack BSF common source code
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

/**
 * @file
 *
 * This bitstream filter repacks AV1 tiles into one packet containing
 * just one frame.
 */

#include "libavutil/opt.h"

#include "avcodec.h"
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 91)
#include "bsf_internal.h"
#else
#include "bsf.h"
#endif
#include "cbs.h"
#include "cbs_av1.h"
#include "internal.h"
#include "ni_av1_rbsp.h"
#if ((LIBAVCODEC_VERSION_MAJOR > 61) || (LIBAVCODEC_VERSION_MAJOR == 61 && LIBAVCODEC_VERSION_MINOR >= 19))
#include "libavutil/mem.h"
#endif
typedef struct AV1RepackContext {
    AVPacket *buffer_pkt;
    AVPacket **tile_pkt;
    CodedBitstreamContext *cbc;
    CodedBitstreamFragment temporal_unit;
    PutBitContext stream;
    AV1TileInfo tileinfo[MAX_NUM_TILE_PER_FRAME];

    int tile_pos;
    int tile_num;
} AV1RepackContext;

static int ni_av1_log2(int blksize, int target)
{
    int k;
    for (k = 0; (blksize << k) < target; k++);
    return k;
}

static int av1_rawtotile_encode_sequence_header_obu(AVBSFContext *ctx, AV1RawSequenceHeader *current, AV1TileInfo *tileinfo)
{
    AV1RepackContext *s = ctx->priv_data;

    current->max_frame_width_minus_1 = tileinfo->width - 1;
    current->max_frame_height_minus_1 = tileinfo->height - 1;
    current->frame_width_bits_minus_1 = ni_av1_log2(1, tileinfo->width) - 1;
    current->frame_height_bits_minus_1 = ni_av1_log2(1, tileinfo->height) - 1;

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: width %d height %d\n",
            __FILE__, __LINE__, __func__, tileinfo->width, tileinfo->height);

    return netint_av1_write_sequence_header_obu(s->cbc, &s->stream, current);
}

static int av1_rawtotile_encode_temporal_delimiter_obu(AVBSFContext *ctx)
{
    AV1RepackContext *s = ctx->priv_data;

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: \n", __FILE__, __LINE__, __func__);

    return netint_av1_write_temporal_delimiter_obu(s->cbc, &s->stream);
}

#ifdef NOFRAMEOBU
static int av1_rawtotile_encode_frame_header_obu(AVBSFContext *ctx, AV1RawSequenceHeader *seq, AV1RawFrameHeader *current, AV1TileInfo *tileinfo)
{
    int uniform_tile_spacing_flag = 1;
    AV1RepackContext *s = ctx->priv_data;

    current->frame_width_minus_1 = tileinfo->width - 1;
    current->frame_height_minus_1 = tileinfo->height - 1;
    current->tile_cols_log2 = ni_av1_log2(1, tileinfo->column);
    current->tile_rows_log2 = ni_av1_log2(1, tileinfo->row);
    current->tile_size_bytes_minus1 = TILE_SIZE_BYTES - 1;
    for(int i = 1; i < tileinfo->column * tileinfo->row; i++)
    {
        if((tileinfo[i].x_w != tileinfo[i - 1].x_w) || (tileinfo[i].y_h != tileinfo[i - 1].y_h))
        {
            uniform_tile_spacing_flag = 0;
            break;
        }
    }
    if(!uniform_tile_spacing_flag)
    {
        for(int i = 0; i < tileinfo->column; i++)
            current->width_in_sbs_minus_1[i] = (tileinfo[i].x_w + 63) / 64 - 1;
        for(int i = 0; i < tileinfo->row; i++)
            current->height_in_sbs_minus_1[i] = (tileinfo[i * tileinfo->column].y_h + 63) / 64 - 1;
        current->uniform_tile_spacing_flag = 0;
    }

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: width %d height %d tile_size_bytes_minus1 %d\n",
            __FILE__, __LINE__, __func__, tileinfo->width, tileinfo->height, current->tile_size_bytes_minus1);

    return netint_av1_write_frame_header_obu(s->cbc, &s->stream, seq, current);
}

static int av1_rawtotile_encode_tile_group_obu(AVBSFContext *ctx, AV1RawSequenceHeader *seq, AV1RawTileGroup *current)
{
    AV1RepackContext *s = ctx->priv_data;

     current->tg_end = s->tile_num - 1;

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: tile_start_and_end_present_flag %d tg_start %d tg_end %d\n",
            __FILE__, __LINE__, __func__, current->tile_start_and_end_present_flag, current->tg_start, current->tg_end);

    return netint_av1_write_tile_group_obu(s->cbc, &s->stream, seq, current);
}
#else
static int av1_rawtotile_encode_frame_obu(
    AVBSFContext *ctx, AV1RawSequenceHeader *seq, AV1RawTileGroup *tile_current, AV1RawFrameHeader *frame_current, AV1TileInfo *tileinfo)
{
    AV1RepackContext *s = ctx->priv_data;
    AV1RawFrame current;

    frame_current->frame_width_minus_1 = tileinfo->width - 1;
    frame_current->frame_height_minus_1 = tileinfo->height - 1;
    frame_current->tile_cols_log2 = ni_av1_log2(1, tileinfo->column);
    frame_current->tile_rows_log2 = ni_av1_log2(1, tileinfo->row);
    frame_current->tile_size_bytes_minus1 = TILE_SIZE_BYTES - 1;

    memcpy(&current.header, frame_current, sizeof(AV1RawFrameHeader));
    memcpy(&current.tile_group, tile_current, sizeof(AV1RawTileGroup));

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: width %d height %d tile_size_bytes_minus1 %d\n",
            __FILE__, __LINE__, __func__, tileinfo->width, tileinfo->height, frame_current->tile_size_bytes_minus1);

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: tile_start_and_end_present_flag %d tg_start %d tg_end %d\n",
            __FILE__, __LINE__, __func__, tile_current->tile_start_and_end_present_flag, tile_current->tg_start, tile_current->tg_end);

    return netint_av1_write_frame_obu(s->cbc, &s->stream, seq, &current);
}
#endif

// called from tile_repack_bsf()
static int av1_tile_repack_filter(AVBSFContext *ctx, AVPacket *out) {
    AV1RepackContext *s = ctx->priv_data;
    CodedBitstreamFragment *td = &s->temporal_unit;
    CodedBitstreamAV1Context *priv = s->cbc->priv_data;
    AVPacket *tile_pkt;
    AV1RawOBU *obu;
#ifndef NOFRAMEOBU
    AV1RawOBU *frame_obu;
#endif
    AV1RawSequenceHeader *prev_sequence_header_obu;
    PutBitContext pbc_tmp;
    int start_pos, end_pos, data_pos, add_trailing_bits = 0;
    size_t obu_size;
    int ret;
    int tile_idx;
    int *side_data;
    int i, j, new_size;
    int tile_group_index;

    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: tile_pos %d, tile_num %d\n", __FILE__, __LINE__, __func__, s->tile_pos, s->tile_num);

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

        // copy tile info
        memcpy(&s->tileinfo[tile_idx], s->buffer_pkt->data, sizeof(AV1TileInfo));

#ifdef PLOG
        for (i = 0; i < s->tileinfo[tile_idx].num_obu; i++) {
            av_log(ctx, AV_LOG_INFO, "### %s line %d %s: tile_idx %d tile_pos %d unit %d type %d unit_size %d obu_size %d column %d row %d raw_data_size %d\n",
                __FILE__, __LINE__, __func__, tile_idx, s->tile_pos, i, s->tileinfo[tile_idx].type[i], s->tileinfo[tile_idx].unit_size[i],
                s->tileinfo[tile_idx].obu_size[i], s->tileinfo[tile_idx].column, s->tileinfo[tile_idx].row);
            for(j = 0; j < s->tileinfo[tile_idx].num_tile_group; j++)
            {
                av_log(ctx, AV_LOG_INFO, "raw_data_size[%d] %d\n", j, s->tileinfo[tile_idx].tile_raw_data_size[j]);
            }
        }
#endif

        s->tile_pkt[tile_idx]->data = s->buffer_pkt->data + sizeof(AV1TileInfo);
        s->tile_pkt[tile_idx]->size = s->buffer_pkt->size - sizeof(AV1TileInfo);

        av_log(ctx, AV_LOG_DEBUG, "== tile %d, data actual size %d data pos %ld\n", tile_idx,
               s->buffer_pkt->size, s->buffer_pkt->pos);

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

                    av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: side data type %d size %d added\n",
                            __FILE__, __LINE__, __func__, type, size);

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
#ifndef NOFRAMEOBU
        int frame_header_index = -1;
#endif
        tile_group_index = 0;

        tile_pkt = s->tile_pkt[0];

        priv->tile_cols = s->tileinfo[0].column;
        priv->tile_rows = s->tileinfo[0].row;
        priv->frame_width = s->tileinfo[0].width;
        priv->frame_height = s->tileinfo[0].height;

        // read headers and save in ctx->priv_data in cbs_av1_read_unit()
        ret = ff_cbs_read_packet(s->cbc, td, tile_pkt);
        if (ret < 0) {
            av_log(ctx, AV_LOG_INFO, "Failed to parse temporal unit.\n");
            goto end;
        }

        for (i = 0; i < td->nb_units; i++) {

            CodedBitstreamUnit *unit = &td->units[i];
            obu = (AV1RawOBU *) unit->content;
            obu->header.obu_has_size_field = 1;

            netint_av1_write_obu_header(s->cbc, &s->stream, &obu->header);

            av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: obu_has_size_field %d\n",
                   __FILE__, __LINE__, __func__, obu->header.obu_has_size_field);

            if (obu->header.obu_has_size_field) {
                pbc_tmp = s->stream;
                // Add space for the size field to fill later.
                put_bits32(&s->stream, 0);
                put_bits32(&s->stream, 0);
            }

            start_pos = put_bits_count(&s->stream);

            switch(unit->type) {
            case AV1_OBU_SEQUENCE_HEADER:
                av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: AV1_OBU_SEQUENCE_HEADER \n",
                       __FILE__, __LINE__, __func__);
                av1_rawtotile_encode_sequence_header_obu(ctx, &obu->obu.sequence_header, &s->tileinfo[0]);
                prev_sequence_header_obu = &obu->obu.sequence_header;
                priv->sequence_header = &obu->obu.sequence_header;
                add_trailing_bits = 1;
                break;
            case AV1_OBU_TEMPORAL_DELIMITER:
                av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: AV1_OBU_TEMPORAL_DELIMITER \n",
                       __FILE__, __LINE__, __func__);
                av1_rawtotile_encode_temporal_delimiter_obu(ctx);
                add_trailing_bits = 0;
                break;
            case AV1_OBU_FRAME_HEADER:
                av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: AV1_OBU_FRAME_HEADER \n",
                       __FILE__, __LINE__, __func__);
#ifdef NOFRAMEOBU
                av1_rawtotile_encode_frame_header_obu(ctx, prev_sequence_header_obu, &obu->obu.frame_header, &s->tileinfo[0]);
                add_trailing_bits = 1;
#else
                frame_header_index = i;
#endif
                break;
            case AV1_OBU_TILE_GROUP:
                av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: AV1_OBU_TILE_GROUP \n",
                       __FILE__, __LINE__, __func__);
#ifdef NOFRAMEOBU
                av1_rawtotile_encode_tile_group_obu(ctx, prev_sequence_header_obu, &obu->obu.tile_group);
                add_trailing_bits = 0;
#else
                // Need to encode OBU_FRAME_HEADER + OBU_TILE_GROUP as OBU_FRAME
                if(frame_header_index < 0) {
                    av_log(ctx, AV_LOG_INFO, "AV1_OBU_FRAME_HEADER must be received first to repack\n");
                    goto end;
                }

                frame_obu = td->units[frame_header_index].content;

                av1_rawtotile_encode_frame_obu(ctx, prev_sequence_header_obu,
                     &obu->obu.tile_group, &frame_obu->obu.frame_header, &tileinfo[0]);
#endif

                break;

            case AV1_OBU_METADATA:
            case AV1_OBU_FRAME:
            case AV1_OBU_REDUNDANT_FRAME_HEADER:
                continue;
            case AV1_OBU_TILE_LIST:
            default:
                av_log(ctx, AV_LOG_INFO, "Large scale tiles are unsupported.\n");
                goto end;
            }


            if(unit->type != AV1_OBU_TILE_GROUP) {
               av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: calling netint_av1_update_obu_data_length 1 \n",
                       __FILE__, __LINE__, __func__);
               obu_size = netint_av1_update_obu_data_length(s->cbc, &s->stream, start_pos, obu, &pbc_tmp, add_trailing_bits);
            } else {
                uint8_t *rawdata;
                int tile_size, last_tile;

                for(j = 0; j < s->tile_num; j++) {
                    end_pos = put_bits_count(&s->stream);
                    tile_size = s->tileinfo[j].tile_raw_data_size[tile_group_index];
                    last_tile = (j == (s->tile_num - 1));

                    // Don't add the size info for the last tile
                    if(!last_tile)
                        netint_av1_write_le32(s->cbc, &s->stream, "tile_size_minus_1", tile_size - 1);

                    data_pos = put_bits_count(&s->stream) / 8;
                    flush_put_bits(&s->stream);

                    if (tile_size > 0) {
                        rawdata = s->tile_pkt[j]->data + s->tileinfo[j].tile_raw_data_pos[tile_group_index];

                        memcpy(s->stream.buf + data_pos, rawdata, tile_size);
                        skip_put_bytes(&s->stream, tile_size);

                        av_log(ctx, AV_LOG_DEBUG, "### %s line %d %s: writing obu size plus raw data for tile %d last data %x %x\n",
                                __FILE__, __LINE__, __func__, j, rawdata[tile_size-1], rawdata[tile_size]);
                    }
#ifdef PLOG
                    av_log(ctx, AV_LOG_INFO, "### %s line %d %s: end_pos %d data_pos %d tile_size %d\n",
                            __FILE__, __LINE__, __func__, end_pos, data_pos, tile_size);
                    for(int k = 0; k < s->tileinfo[tile_idx].num_tile_group; k++)
                    {
                        av_log(ctx, AV_LOG_INFO, "raw_data_pos[%d] %d, raw_data_size[%d] %d", k, s->tileinfo[tile_idx].tile_raw_data_pos[k], k, s->tileinfo[tile_idx].tile_raw_data_size[k]);
                    }
#endif
                }
                tile_group_index++;
                obu_size = netint_av1_update_obu_data_length(s->cbc, &s->stream, start_pos, obu, &pbc_tmp, add_trailing_bits);
            }
        }
        for(i = 0; i < s->tile_num; i++)
        {
            av_buffer_unref(&s->tile_pkt[i]->buf);
            s->tile_pkt[i]->buf = NULL;
        }

#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
        ff_cbs_fragment_reset(td);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
        ff_cbs_fragment_reset(s->cbc, td);
#else
        ff_cbs_fragment_uninit(s->cbc, td);
#endif

        new_size = put_bits_count(&s->stream) / 8;
        ret = av_new_packet(out, new_size);
        if (ret < 0) {
            return ret;
        }

        av_packet_copy_props(out, s->buffer_pkt);

        netint_av1_bitstream_fetch(&s->stream, out, new_size);
        netint_av1_bitstream_reset(&s->stream);

        s->tile_pos = 0;
        goto end;
    } else {
        return AVERROR(EAGAIN);
    }

end:
    return 0;
}

static const CodedBitstreamUnitType decompose_unit_types[] = {
    AV1_OBU_TEMPORAL_DELIMITER,
    AV1_OBU_SEQUENCE_HEADER,
    AV1_OBU_FRAME_HEADER,
    AV1_OBU_TILE_GROUP,
    AV1_OBU_FRAME,
};

static int av1_tile_repack_init(AVBSFContext *ctx) {
    AV1RepackContext *s = ctx->priv_data;
    CodedBitstreamFragment *td = &s->temporal_unit;
    void *pb_buf;
    int i, ret;

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

#if 1 // moved from rawtotile
    s->buffer_pkt = av_packet_alloc();
    if (!s->buffer_pkt)
        return AVERROR(ENOMEM);

    ret = ff_cbs_init(&s->cbc, AV_CODEC_ID_AV1, ctx);
    if (ret < 0)
        return ret;

    s->cbc->decompose_unit_types    = (CodedBitstreamUnitType*)decompose_unit_types;
    s->cbc->nb_decompose_unit_types = FF_ARRAY_ELEMS(decompose_unit_types);

#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_reset(td);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_reset(s->cbc, td);
#else
    ff_cbs_fragment_uninit(s->cbc, td);
#endif

    pb_buf = av_mallocz(MAX_PUT_BUF_SIZE);
    if (!pb_buf) {
        return AVERROR(ENOMEM);
    }
    init_put_bits(&s->stream, pb_buf, MAX_PUT_BUF_SIZE);
#endif

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

static void av1_tile_repack_flush(AVBSFContext *ctx) {
    AV1RepackContext *s = ctx->priv_data;
    int i;

#ifdef PLOG
    av_log(ctx, AV_LOG_INFO, "### %s line %d %s: \n", __FILE__, __LINE__, __func__);
#endif

    av_packet_unref(s->buffer_pkt);

    for (i = 0; i < s->tile_num; i++) {
        av_packet_unref(s->tile_pkt[i]);
    }

    av_packet_unref(s->buffer_pkt);

#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_reset(&s->temporal_unit);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_reset(s->cbc, &s->temporal_unit);
#else
    ff_cbs_fragment_uninit(s->cbc, &s->temporal_unit);
#endif
}

static void av1_tile_repack_close(AVBSFContext *ctx) {
    AV1RepackContext *s = ctx->priv_data;
    int i;

#ifdef PLOG
    av_log(ctx, AV_LOG_INFO, "### %s line %d %s: \n", __FILE__, __LINE__, __func__);
#endif

    av_packet_free(&s->buffer_pkt);

    for (i = 0; i < s->tile_num; i++) {
        av_packet_free(&s->tile_pkt[i]);
    }
    av_freep(&s->tile_pkt);

#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_reset(&s->temporal_unit);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_reset(s->cbc, &s->temporal_unit);
#else
    ff_cbs_fragment_uninit(s->cbc, &s->temporal_unit);
#endif

    ff_cbs_close(&s->cbc);
}

static const enum AVCodecID av1_tile_repack_codec_ids[] = {
    AV_CODEC_ID_AV1,
    AV_CODEC_ID_NONE,
};

#define OFFSET(x) offsetof(AV1RepackContext, x)
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
    .class_name = "av1_tile_repack_bsf",
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
ff_av1_tile_repack_bsf = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || LIBAVCODEC_VERSION_MAJOR >= 59 && LIBAVCODEC_VERSION_MINOR >= 37)
    .p.name         = "av1_tile_repack",
    .p.codec_ids    = av1_tile_repack_codec_ids,
    .p.priv_class     = &tile_repack_class,
#else
    .name           = "av1_tile_repack",
    .codec_ids      = av1_tile_repack_codec_ids,
    .priv_class     = &tile_repack_class,
#endif
    .priv_data_size = sizeof(AV1RepackContext),
    .init           = av1_tile_repack_init,
    .flush          = av1_tile_repack_flush,
    .close          = av1_tile_repack_close,
    .filter         = av1_tile_repack_filter,
};
