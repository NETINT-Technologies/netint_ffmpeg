/*
 * NetInt HEVC frame split BSF common source code
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
 * This bitstream filter splits HEVC stream into packets containing just one
 * frame and re-encoding them with tile flags so that the splited packets can
 * be decoded independently.
 */

#include "version.h"

#include "libavutil/avassert.h"

#include "avcodec.h"
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 91)
#include "bsf_internal.h"
#else
#include "bsf.h"
#endif
#include "cbs.h"
#include "cbs_h265.h"
#include "ni_hevc_extradata.h"
#include "ni_hevc_rbsp.h"
#if ((LIBAVCODEC_VERSION_MAJOR > 61) || (LIBAVCODEC_VERSION_MAJOR == 61 && LIBAVCODEC_VERSION_MINOR >= 19))
#include "libavutil/mem.h"
#endif

struct tile_format {
    int log2_ctb_size;
    int num_tile_columns;
    int num_tile_rows;
    int ctb_width;
    int ctb_height;
    int width;
    int height;

    int *column_width;
    int *row_height;
    int *col_idx;
    int *row_idx;
};

typedef struct HEVCFSplitContext {
    AVPacket *buffer_pkt;
    CodedBitstreamContext *cbc;
    CodedBitstreamFragment temporal_unit;

    struct tile_format *tiles[HEVC_MAX_PPS_COUNT];
    struct tile_format **this_tile;
    ni_bitstream_t *streams;

    int tile_enabled;
    int num_tiles;
    int unit_offset;
    int nb_slices;
} HEVCFSplitContext;

static int slice_addr_to_idx(HEVCFSplitContext *s, int slice_addr, int hid) {
    int x, y;
    struct tile_format *format = s->tiles[hid];

    av_assert0(format);

    x = slice_addr % format->ctb_width;
    y = slice_addr / format->ctb_width;

    return format->col_idx[x] + format->num_tile_columns * format->row_idx[y];
}

static void slice_geo(HEVCFSplitContext *s, int slice_idx, int *width,
                      int *height, int hid) {
    int x, y;
    struct tile_format *format = s->tiles[hid];

    av_assert0(format);

    x = slice_idx % format->num_tile_columns;
    y = slice_idx / format->num_tile_columns;

    *width  = format->column_width[x];
    *height = format->row_height[y];
}

static int hevc_frame_resolve_tiles(HEVCFSplitContext *s, AVBSFContext *ctx,
                                    const H265RawSPS *sps,
                                    const H265RawPPS *pps) {
    int i, j;
    int log2_ctb_size;
    int limit;
    struct tile_format *format;
    int *index_buffer;
    uint32_t index_len;

    if (pps->pps_pic_parameter_set_id >= HEVC_MAX_PPS_COUNT) {
        av_log(ctx, AV_LOG_ERROR, "pps id %d exceeds maximus\n",
               pps->pps_pic_parameter_set_id);
        return AVERROR(EINVAL);
    }

    if (!s->tiles[pps->pps_pic_parameter_set_id]) {
        format = av_mallocz(sizeof(struct tile_format));
        if (!format) {
            av_log(ctx, AV_LOG_ERROR, "failed to allocate tile format\n");
            return AVERROR(ENOMEM);
        }
        s->tiles[pps->pps_pic_parameter_set_id] = format;
    } else {
        format = s->tiles[pps->pps_pic_parameter_set_id];
    }

    if (!pps->tiles_enabled_flag) {
        av_log(ctx, AV_LOG_ERROR, "tile enabled flags invalid\n");
        return AVERROR(ENODEV);
    }

    log2_ctb_size = (sps->log2_min_luma_coding_block_size_minus3 + 3) +
                    sps->log2_diff_max_min_luma_coding_block_size;
    format->ctb_width =
        (sps->pic_width_in_luma_samples + (1 << log2_ctb_size) - 1) >>
        log2_ctb_size;
    format->ctb_height =
        (sps->pic_height_in_luma_samples + (1 << log2_ctb_size) - 1) >>
        log2_ctb_size;
    format->num_tile_columns = pps->num_tile_columns_minus1 + 1;
    format->num_tile_rows    = pps->num_tile_rows_minus1 + 1;
    format->log2_ctb_size    = log2_ctb_size;
    format->width            = sps->pic_width_in_luma_samples;
    format->height           = sps->pic_height_in_luma_samples;

    av_log(
        ctx, AV_LOG_DEBUG,
        "ctb_size %d, ctb_width %d, ctb_height %d, uniform_spacing_flag %d\n",
        log2_ctb_size, format->ctb_width, format->ctb_height,
        pps->uniform_spacing_flag);

    index_len = sizeof(int) * ((pps->num_tile_columns_minus1 + 1) +
                               (pps->num_tile_rows_minus1 + 1) +
                               format->ctb_width + format->ctb_height);

    index_buffer = av_mallocz(index_len);
    if (!index_buffer) {
        av_log(ctx, AV_LOG_ERROR, "failed to allocate index buffer\n");
        return AVERROR(ENOMEM);
    }

    format->column_width = index_buffer;
    format->row_height =
        format->column_width + (pps->num_tile_columns_minus1 + 1);
    format->col_idx = format->row_height + (pps->num_tile_rows_minus1 + 1);
    format->row_idx = format->col_idx + format->ctb_width;

    if (pps->uniform_spacing_flag) {
        for (i = 0, limit = 0; i < pps->num_tile_columns_minus1; i++) {
            format->column_width[i] =
                (((i + 1) * format->ctb_width) /
                     (pps->num_tile_columns_minus1 + 1) -
                 (i * format->ctb_width) / (pps->num_tile_columns_minus1 + 1))
                << log2_ctb_size;
            limit += format->column_width[i];
        }
        format->column_width[i] = sps->pic_width_in_luma_samples - limit;

        for (i = 0, limit = 0; i < pps->num_tile_rows_minus1; i++) {
            format->row_height[i] =
                (((i + 1) * format->ctb_height) /
                     (pps->num_tile_rows_minus1 + 1) -
                 (i * format->ctb_height) / (pps->num_tile_rows_minus1 + 1))
                << log2_ctb_size;
            limit += format->row_height[i];
        }
        format->row_height[i] = sps->pic_height_in_luma_samples - limit;
    } else {
        for (i = 0, limit = 0; i < pps->num_tile_columns_minus1; i++) {
            format->column_width[i] = (pps->column_width_minus1[i] + 1)
                                      << log2_ctb_size;
            limit += format->column_width[i];
        }
        format->column_width[i] = sps->pic_width_in_luma_samples - limit;

        for (i = 0, limit = 0; i < pps->num_tile_rows_minus1; i++) {
            format->row_height[i] = (pps->row_height_minus1[i] + 1)
                                    << log2_ctb_size;
            limit += format->row_height[i];
        }
        format->row_height[i] = sps->pic_height_in_luma_samples - limit;
    }

    limit =
        (format->column_width[0] + (1 << log2_ctb_size) - 1) >> log2_ctb_size;
    for (i = j = 0; i < format->ctb_width; i++) {
        if (i >= limit) {
            j++;
            limit += ((format->column_width[j] + (1 << log2_ctb_size) - 1) >>
                      log2_ctb_size);
        }
        format->col_idx[i] = j;
    }

    limit = (format->row_height[0] + (1 << log2_ctb_size) - 1) >> log2_ctb_size;
    for (i = j = 0; i < format->ctb_height; i++) {
        if (i >= limit) {
            j++;
            limit += ((format->row_height[j] + (1 << log2_ctb_size) - 1) >>
                      log2_ctb_size);
        }
        format->row_idx[i] = j;
    }

    /* dump index buffer */
    for (i = 0; i < pps->num_tile_columns_minus1 + 1; i++) {
        av_log(ctx, AV_LOG_DEBUG, "column_width: %d %d\n", i,
               format->column_width[i]);
    }

    for (i = 0; i < pps->num_tile_rows_minus1 + 1; i++) {
        av_log(ctx, AV_LOG_DEBUG, "row_width: %d %d\n", i,
               format->row_height[i]);
    }

    for (i = 0; i < format->ctb_width; i++) {
        av_log(ctx, AV_LOG_DEBUG, "column_idx: %d %d\n", i, format->col_idx[i]);
    }

    for (i = 0; i < format->ctb_height; i++) {
        av_log(ctx, AV_LOG_DEBUG, "row_idx: %d %d\n", i, format->row_idx[i]);
    }

    return 0;
}

static int hevc_frame_init_tiles(HEVCFSplitContext *s, AVBSFContext *ctx) {
    CodedBitstreamH265Context *priv = s->cbc->priv_data;
    const H265RawSPS *sps;
    const H265RawPPS *pps;
    int i, ret;

    for (i = 0; i < s->temporal_unit.nb_units; i++) {
        CodedBitstreamUnit *unit = &s->temporal_unit.units[i];
        if (unit->type == HEVC_NAL_PPS) {
            pps = (H265RawPPS *)unit->content;
            if (!pps) {
                av_log(ctx, AV_LOG_ERROR, "invalid pps\n");
                return AVERROR(EINVAL);
            }
            break;
        }
    }

    if (i >= s->temporal_unit.nb_units) {
        av_log(ctx, AV_LOG_ERROR, "cannot find valid header\n");
        return AVERROR(ENODEV);
    }

    sps = priv->sps[pps->pps_seq_parameter_set_id];
    if (!sps) {
        av_log(ctx, AV_LOG_ERROR, "invalid pps data\n");
        return AVERROR(EINVAL);
    }

    if (!pps->tiles_enabled_flag) {
        av_log(ctx, AV_LOG_ERROR, "tile is disabled\n");
        return AVERROR(ENODEV);
    }

    s->tile_enabled = !!pps->tiles_enabled_flag;
    s->num_tiles =
        (pps->num_tile_columns_minus1 + 1) * (pps->num_tile_rows_minus1 + 1);

    ret = hevc_frame_resolve_tiles(s, ctx, sps, pps);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to resolve tiles\n");
        return ret;
    }

    s->this_tile = &s->tiles[pps->pps_pic_parameter_set_id];
    if (!s->this_tile) {
        av_log(ctx, AV_LOG_ERROR, "invalid tile format\n");
        return AVERROR(EINVAL);
    }

    return 0;
}

static int hevc_frame_encode_vps(HEVCFSplitContext *s, AVBSFContext *ctx,
                                 CodedBitstreamUnit *unit, int tile_idx) {
    ni_bitstream_t *stream = &s->streams[tile_idx];
    H265RawVPS *vps        = unit->content;
    int ret;

    ni_write_nal_header(stream, HEVC_NAL_VPS, 0, 1);
    ret = ni_hevc_encode_nal_vps(stream, vps);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to encode vps for tile %d\n",
               tile_idx);
        return ret;
    }

    return 0;
}

static int hevc_frame_tiles_encode_vps(HEVCFSplitContext *s, AVBSFContext *ctx,
                                       CodedBitstreamUnit *unit) {
    int i, ret = 0;

    for (i = 0; i < s->num_tiles; i++) {
        ret = hevc_frame_encode_vps(s, ctx, unit, i);
        if (ret)
            break;
    }

    return ret;
}

static int hevc_frame_encode_sps(HEVCFSplitContext *s, AVBSFContext *ctx,
                                 CodedBitstreamUnit *unit, int tile_idx,
                                 int hid) {
    int ret, width, height;
    ni_bitstream_t *stream = &s->streams[tile_idx];
    H265RawSPS *sps        = unit->content;

    slice_geo(s, tile_idx, &width, &height, hid);

    av_log(ctx, AV_LOG_DEBUG, "tile_idx %d, pixel %dx%d\n", tile_idx, width,
           height);

    ni_write_nal_header(stream, HEVC_NAL_SPS, 0, 1);
    ret = ni_hevc_encode_nal_sps(stream, sps, width, height);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to encode sps for tile %d\n",
               tile_idx);
        return ret;
    }

    return 0;
}

static int hevc_frame_tiles_encode_sps(HEVCFSplitContext *s, AVBSFContext *ctx,
                                       CodedBitstreamUnit *unit, int hid) {
    int i, ret = 0;

    for (i = 0; i < s->num_tiles; i++) {
        ret = hevc_frame_encode_sps(s, ctx, unit, i, hid);
        if (ret)
            break;
    }

    return ret;
}

static int hevc_frame_encode_pps(HEVCFSplitContext *s, AVBSFContext *ctx,
                                 CodedBitstreamUnit *unit, int tile_idx) {
    ni_bitstream_t *stream = &s->streams[tile_idx];
    H265RawPPS *pps        = unit->content;
    int ret                = 0;

    ni_write_nal_header(stream, HEVC_NAL_PPS, 0, 1);
    ret = ni_hevc_encode_nal_pps(stream, pps, 1, 1, 1);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to encode pps for tile %d\n",
               tile_idx);
    }

    return ret;
}

static int hevc_frame_tiles_encode_pps(HEVCFSplitContext *s, AVBSFContext *ctx,
                                       CodedBitstreamUnit *unit) {
    int i, ret = 0;

    for (i = 0; i < s->num_tiles; i++) {
        ret = hevc_frame_encode_pps(s, ctx, unit, i);
        if (ret)
            break;
    }

    return ret;
}

static int hevc_frame_encode_slice_header(HEVCFSplitContext *s,
                                          AVBSFContext *ctx,
                                          CodedBitstreamUnit *unit,
                                          int tile_idx, int hid) {
    H265RawSlice *slice             = unit->content;
    ni_bitstream_t *stream          = &s->streams[tile_idx];
    CodedBitstreamH265Context *priv = s->cbc->priv_data;
    H265RawPPS *pps                 = priv->pps[hid];
    H265RawSPS *sps                 = priv->sps[pps->pps_seq_parameter_set_id];
    int i, ret;

    ni_write_nal_header(stream, slice->header.nal_unit_header.nal_unit_type, 0,
                        1);

    ret = ni_hevc_encode_nal_slice_header(stream, &slice->header, sps, pps, -1,
                                          -1, 1 /* disable tile */, 0, 0,
                                          1 /* independent */);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to encode slice header for tile %d\n",
               tile_idx);
        return ret;
    }

    av_assert0((slice->data_bit_start % 8) == 0);

    for (i = 0; i < slice->data_size; i++) {
        ni_put_bits(stream, 8, slice->data[i]);
    }

    return 0;
}

static int hevc_frame_init_bitstream(HEVCFSplitContext *s) {
    int i;

    s->streams = av_mallocz(s->num_tiles * sizeof(ni_bitstream_t));
    if (!s->streams) {
        return AVERROR(ENOMEM);
    }

    for (i = 0; i < s->num_tiles; i++) {
        ni_bitstream_init(&s->streams[i]);
    }

    return 0;
}

static int hevc_frame_parse_tiles(HEVCFSplitContext *s, AVBSFContext *ctx,
                                  H265RawSPS *sps, H265RawPPS *pps) {
    struct tile_format *format = *s->this_tile;
    int log2_ctb_size;

    if (!pps->tiles_enabled_flag) {
        av_log(ctx, AV_LOG_ERROR, "tile_enabled_flag unset\n");
        return AVERROR(ENODEV);
    }

    if (sps->pic_width_in_luma_samples != format->width ||
        sps->pic_height_in_luma_samples != format->height) {
        av_log(ctx, AV_LOG_ERROR, "pixel size not match\n");
        return AVERROR(ENODEV);
    }

    if (pps->num_tile_columns_minus1 + 1 != format->num_tile_columns ||
        pps->num_tile_rows_minus1 + 1 != format->num_tile_rows) {
        av_log(ctx, AV_LOG_ERROR, "tiles partition not match\n");
        return AVERROR(ENODEV);
    }

    log2_ctb_size = (sps->log2_min_luma_coding_block_size_minus3 + 3) +
                    sps->log2_diff_max_min_luma_coding_block_size;
    if (log2_ctb_size != format->log2_ctb_size) {
        av_log(ctx, AV_LOG_ERROR, "ctb size not match\n");
        return AVERROR(ENODEV);
    }

    return 0;
}

static int hevc_frame_split_filter(AVBSFContext *ctx, AVPacket *out) {
    HEVCFSplitContext *s       = ctx->priv_data;
    CodedBitstreamFragment *td = &s->temporal_unit;
    int i, ret, tile_idx = 0, new_size, *slice_addr;
    H265RawSlice *slice;
    ni_bitstream_t *stream = &s->streams[0];

    if (!s->tile_enabled) {
        av_assert0(s->tile_enabled);
        goto passthrough;
    }

    if (s->buffer_pkt->data) {
        av_assert0(s->nb_slices > 0);
        goto slice_split;
    }

    ret = ff_bsf_get_packet_ref(ctx, s->buffer_pkt);
    if (ret < 0) {
        av_log(ctx, AV_LOG_DEBUG, "failed to get packet ref: 0x%x\n", ret);
        return ret;
    }

    ret = ff_cbs_read_packet(s->cbc, td, s->buffer_pkt);
    if (ret < 0) {
        av_log(ctx, AV_LOG_WARNING, "Failed to parse temporal unit.\n");
        goto passthrough;
    }

    s->nb_slices   = 0;
    s->unit_offset = 0;
    for (i = 0; i < td->nb_units; i++) {
        CodedBitstreamUnit *unit = &td->units[i];
        av_log(ctx, AV_LOG_DEBUG, "query index %d, unit type %d\n", i,
               unit->type);

        if (unit->type == HEVC_NAL_VPS) {
            ret = hevc_frame_tiles_encode_vps(s, ctx, unit);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "failed to re-encode vps\n");
                return ret;
            }
        } else if (unit->type == HEVC_NAL_SPS) {
            if (i < td->nb_units - 1 && td->units[i + 1].type == HEVC_NAL_PPS) {
                H265RawPPS *pps = td->units[i + 1].content;
                H265RawSPS *sps = unit->content;
                if (pps->pps_seq_parameter_set_id ==
                    sps->sps_seq_parameter_set_id) {
                    ret = hevc_frame_parse_tiles(s, ctx, sps, pps);
                    if (ret < 0) {
                        av_log(ctx, AV_LOG_ERROR, "failed to parse tiles\n");
                        return ret;
                    }

                    ret = hevc_frame_tiles_encode_sps(
                        s, ctx, unit, pps->pps_pic_parameter_set_id);
                    if (ret < 0) {
                        av_log(ctx, AV_LOG_ERROR, "failed to re-encode sps\n");
                        return ret;
                    }
                } else {
                    av_log(ctx, AV_LOG_ERROR,
                           "seq_parameter_set_id mismatch: %d, %d\n",
                           pps->pps_seq_parameter_set_id,
                           sps->sps_seq_parameter_set_id);
                    return AVERROR(EINVAL);
                }
            } else {
                av_log(ctx, AV_LOG_ERROR, "failed to find PPS after SPS\n");
                return AVERROR(EINVAL);
            }
        } else if (unit->type == HEVC_NAL_PPS) {
            ret = hevc_frame_tiles_encode_pps(s, ctx, unit);
            if (ret < 0) {
                av_log(ctx, AV_LOG_ERROR, "failed to re-encode pps\n");
                return ret;
            }
        } else if (unit->type <= HEVC_NAL_RSV_VCL31) {
            if (s->nb_slices == 0) {
                s->unit_offset = i;
            }
            s->nb_slices++;
        }
    }

    if (s->nb_slices == 0) {
        ret = AVERROR(EAGAIN);
        goto end;
    }

slice_split:
    for (i = s->unit_offset; i < td->nb_units; i++) {
        CodedBitstreamUnit *unit = &td->units[i];
        if (unit->type <= HEVC_NAL_RSV_VCL31) {
            slice = unit->content;
            tile_idx =
                slice_addr_to_idx(s, slice->header.slice_segment_address,
                                  slice->header.slice_pic_parameter_set_id);
            av_assert0(tile_idx >= 0 && tile_idx < s->num_tiles);
            av_log(ctx, AV_LOG_DEBUG, "slice_seg_addr %d, tile_idx %d\n",
                   slice->header.slice_segment_address, tile_idx);

            ret = hevc_frame_encode_slice_header(
                s, ctx, unit, tile_idx,
                slice->header.slice_pic_parameter_set_id);
            if (ret) {
                av_log(ctx, AV_LOG_ERROR, "failed to re-encode slice header\n");
                av_assert0(0);
            }

            stream = &s->streams[tile_idx];
            break;
        }
    }

    new_size = (int)(ni_bitstream_count(stream) / 8);
    ret      = av_new_packet(out, new_size);
    if (ret < 0) {
        return ret;
    }

    av_packet_copy_props(out, s->buffer_pkt);

    slice_addr = (int *)av_packet_new_side_data(out, AV_PKT_DATA_SLICE_ADDR,
                                                sizeof(*slice_addr));
    av_assert0(slice_addr);
    *slice_addr = tile_idx;

    ni_bitstream_fetch(stream, out->data, new_size);
    ni_bitstream_reset(stream);

    s->unit_offset++;
    if (s->unit_offset < td->nb_units) {
        // To be continued...
        return ret;
    }

end:
    av_packet_unref(s->buffer_pkt);
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_reset(td);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_reset(s->cbc, td);
#else
    ff_cbs_fragment_uninit(s->cbc, td);
#endif
    return ret;

passthrough:
    av_packet_move_ref(out, s->buffer_pkt);
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_reset(td);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_reset(s->cbc, td);
#else
    ff_cbs_fragment_uninit(s->cbc, td);
#endif

    return ret;
}

static int hevc_frame_split_init(AVBSFContext *ctx) {
    HEVCFSplitContext *s       = ctx->priv_data;
    CodedBitstreamFragment *td = &s->temporal_unit;
    int ret;

    s->buffer_pkt = av_packet_alloc();
    if (!s->buffer_pkt)
        return AVERROR(ENOMEM);

    ret = ff_cbs_init(&s->cbc, AV_CODEC_ID_HEVC, ctx);
    if (ret < 0)
        return ret;

    s->cbc->decompose_unit_types    = NULL;
    s->cbc->nb_decompose_unit_types = 0;
    //    s->cbc->decompose_unit_types    =
    //    (CodedBitstreamUnitType*)decompose_unit_types;
    //    s->cbc->nb_decompose_unit_types =
    //    FF_ARRAY_ELEMS(decompose_unit_types);

    /* extradata is a must */
    //    if (!ctx->par_in->extradata_size)
    //        return AVERROR(ENODEV);
    if (!ctx->par_in->extradata_size)
        return 0;

    ret = ff_cbs_read_extradata(s->cbc, td, ctx->par_in);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to parse extradata.\n");
        goto fail_out;
    }

    ret = hevc_frame_init_tiles(s, ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to initialize tiles\n");
        goto fail_out;
    }

    ret = hevc_frame_init_bitstream(s);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to initialize bitstream\n");
        goto fail_out;
    }

#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_reset(td);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_reset(s->cbc, td);
#else
    ff_cbs_fragment_uninit(s->cbc, td);
#endif
    return 0;

fail_out:
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_free(&s->temporal_unit);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_free(s->cbc, &s->temporal_unit);
#else
    ff_cbs_fragment_uninit(s->cbc, &s->temporal_unit);
#endif
    ff_cbs_close(&s->cbc);
    return ret;
}

static void hevc_frame_split_flush(AVBSFContext *ctx) {
    int i;
    HEVCFSplitContext *s = ctx->priv_data;

    for (i = 0; i < s->num_tiles; i++) {
        ni_bitstream_reset(&s->streams[i]);
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

static void hevc_frame_split_close(AVBSFContext *ctx) {
    HEVCFSplitContext *s = ctx->priv_data;
    int i;

    for (i = 0; i < s->num_tiles; i++) {
        ni_bitstream_deinit(&s->streams[i]);
    }

    for (i = 0; i < HEVC_MAX_PPS_COUNT; i++) {
        if (s->tiles[i]) {
            av_freep(&s->tiles[i]->column_width);
            av_freep(&s->tiles[i]);
        }
    }

    av_freep(&s->streams);
    av_packet_free(&s->buffer_pkt);
#if (LIBAVCODEC_VERSION_MAJOR >= 59 || LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 134)
    ff_cbs_fragment_free(&s->temporal_unit);
#elif (LIBAVCODEC_VERSION_MAJOR >= 58 && LIBAVCODEC_VERSION_MINOR >= 54)
    ff_cbs_fragment_free(s->cbc, &s->temporal_unit);
#else
    ff_cbs_fragment_uninit(s->cbc, &s->temporal_unit);
#endif
    ff_cbs_close(&s->cbc);
}

static const enum AVCodecID hevc_frame_split_codec_ids[] = {
    AV_CODEC_ID_HEVC,
    AV_CODEC_ID_NONE,
};

const
#if (LIBAVCODEC_VERSION_MAJOR > 59 || LIBAVCODEC_VERSION_MAJOR >= 59 && LIBAVCODEC_VERSION_MINOR >= 37)
FFBitStreamFilter
#else
AVBitStreamFilter
#endif
ff_hevc_frame_split_bsf = {
#if (LIBAVCODEC_VERSION_MAJOR > 59 || LIBAVCODEC_VERSION_MAJOR >= 59 && LIBAVCODEC_VERSION_MINOR >= 37)
    .p.name         = "hevc_frame_split",
    .p.codec_ids    = hevc_frame_split_codec_ids,
#else
    .name           = "hevc_frame_split",
    .codec_ids      = hevc_frame_split_codec_ids,
#endif
    .priv_data_size = sizeof(HEVCFSplitContext),
    .init           = hevc_frame_split_init,
    .flush          = hevc_frame_split_flush,
    .close          = hevc_frame_split_close,
    .filter         = hevc_frame_split_filter,
};
