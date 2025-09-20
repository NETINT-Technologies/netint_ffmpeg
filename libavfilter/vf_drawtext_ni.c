/*
 * Copyright (c) 2011 Stefano Sabatini
 * Copyright (c) 2010 S.N. Hemanth Meenakshisundaram
 * Copyright (c) 2003 Gustavo Sverzut Barbieri <gsbarbieri@yahoo.com.br>
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
 * NETINT drawtext filter, based on the original vf_drawtext.c
 *
 */

#include "config.h"

#if HAVE_SYS_TIME_H
#include <sys/time.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#if HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <fenv.h>

#if CONFIG_LIBFONTCONFIG
#include <fontconfig/fontconfig.h>
#endif

#include "libavutil/avstring.h"
#include "libavutil/bprint.h"
#include "libavutil/common.h"
#include "libavutil/file.h"
#include "libavutil/eval.h"
#include "libavutil/opt.h"
#include "libavutil/random_seed.h"
#include "libavutil/parseutils.h"
#include "libavutil/timecode.h"
#include "libavutil/time_internal.h"
#include "libavutil/tree.h"
#include "libavutil/lfg.h"
#include "libavutil/version.h"
#include "nifilter.h"
#include "filters.h"
#include "drawutils.h"
#include "formats.h"
#if !IS_FFMPEG_71_AND_ABOVE
#include "internal.h"
#else
#include "libavutil/mem.h"
#endif

#include "video.h"

#if CONFIG_LIBFRIBIDI
#include <fribidi.h>
#endif

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_GLYPH_H
#include FT_STROKER_H

#define MAX_TEXT_NUM 32

static const char *const var_names[] = {
    "dar",
    "hsub", "vsub",
    "line_h", "lh",           ///< line height, same as max_glyph_h
    "main_h", "h", "H",       ///< height of the input video
    "main_w", "w", "W",       ///< width  of the input video
    "max_glyph_a", "ascent",  ///< max glyph ascent
    "max_glyph_d", "descent", ///< min glyph descent
    "max_glyph_h",            ///< max glyph height
    "max_glyph_w",            ///< max glyph width
    "n",                      ///< number of frame
    "sar",
    "t",                      ///< timestamp expressed in seconds
    "text_h", "th",           ///< height of the rendered text
    "text_w", "tw",           ///< width  of the rendered text
    "x",
    "y",
    "pict_type",
#if FF_API_FRAME_PKT
    "pkt_pos",
    "pkt_size",
#endif
    "pkt_duration",
    NULL
};

static const char *const fun2_names[] = {
    "rand"
};

static double drand(void *opaque, double min, double max)
{
    return min + (max-min) / UINT_MAX * av_lfg_get(opaque);
}

typedef double (*eval_func2)(void *, double a, double b);

static const eval_func2 fun2[] = {
    drand,
    NULL
};

enum var_name {
    VAR_DAR,
    VAR_HSUB, VAR_VSUB,
    VAR_LINE_H, VAR_LH,
    VAR_MAIN_H, VAR_h, VAR_H,
    VAR_MAIN_W, VAR_w, VAR_W,
    VAR_MAX_GLYPH_A, VAR_ASCENT,
    VAR_MAX_GLYPH_D, VAR_DESCENT,
    VAR_MAX_GLYPH_H,
    VAR_MAX_GLYPH_W,
    VAR_N,
    VAR_SAR,
    VAR_T,
    VAR_TEXT_H, VAR_TH,
    VAR_TEXT_W, VAR_TW,
    VAR_X,
    VAR_Y,
    VAR_PICT_TYPE,
#if FF_API_FRAME_PKT
    VAR_PKT_POS,
    VAR_PKT_SIZE,
#endif
    VAR_PKT_DURATION,
    VAR_VARS_NB
};

enum expansion_mode {
    EXP_NONE,
    EXP_NORMAL,
    EXP_STRFTIME,
};

typedef struct NetIntDrawTextContext {
    const AVClass *class;
    int exp_mode;                   ///< expansion mode to use for the text
    int reinit;                     ///< tells if the filter is being reinited
    int text_num;                   ///< number of the text
#if CONFIG_LIBFONTCONFIG
    uint8_t *font[MAX_TEXT_NUM];    ///< font to be used
#endif
    uint8_t *fontfile[MAX_TEXT_NUM];///< font to be used
    uint8_t *text[MAX_TEXT_NUM];    ///< text to be drawn
    uint8_t *text_last_updated[MAX_TEXT_NUM];
    AVBPrint expanded_text;         ///< used to contain the expanded text
    uint8_t *fontcolor_expr[MAX_TEXT_NUM];        ///< fontcolor expression to evaluate
    AVBPrint expanded_fontcolor;    ///< used to contain the expanded fontcolor spec
    int ft_load_flags;              ///< flags used for loading fonts, see FT_LOAD_*
    FT_Vector *positions;           ///< positions for each element in the text
    size_t nb_positions;            ///< number of elements of positions array
    char *textfile;                 ///< file with text to be drawn
    int x[MAX_TEXT_NUM];            ///< x position to start drawing one text
    int y[MAX_TEXT_NUM];            ///< y position to start drawing one text
    int x_bak[MAX_TEXT_NUM];        ///< x position of last uploaded overlay frame
    int y_bak[MAX_TEXT_NUM];        ///< y position of last uploaded overlay frame
    int x_start;                    ///< x position for text canvas start in one frame
    int y_start;                    ///< y position for text canvas start in one frame
    int x_end;                      ///< x position for text canvas end in one frame
    int y_end;                      ///< y position for text canvas end in one frame
    int max_glyph_w;                ///< max glyph width
    int max_glyph_h;                ///< max glyph height
    int shadowx, shadowy;
    int borderw;                    ///< border width
    char *fontsize_expr[MAX_TEXT_NUM];            ///< expression for fontsize
    AVExpr *fontsize_pexpr[MAX_TEXT_NUM];         ///< parsed expressions for fontsize
    unsigned int fontsize[MAX_TEXT_NUM];          ///< font size to use
    unsigned int default_fontsize;  ///< default font size to use

    int line_spacing;               ///< lines spacing in pixels
    short int draw_box;             ///< draw box around text - true or false
    char *boxborderw[MAX_TEXT_NUM]; ///< box border width (padding)
                                    ///  allowed formats: "all", "vert|oriz", "top|right|bottom|left"
    int bb_top[MAX_TEXT_NUM];       ///< the size of the top box border
    int bb_right[MAX_TEXT_NUM];     ///< the size of the right box border
    int bb_bottom[MAX_TEXT_NUM];    ///< the size of the bottom box border
    int bb_left[MAX_TEXT_NUM];      ///< the size of the left box border
    int use_kerning[MAX_TEXT_NUM];  ///< font kerning is used - true/false
    int tabsize[MAX_TEXT_NUM];      ///< tab size
    int fix_bounds;                 ///< do we let it go out of frame bounds - t/f
    int optimize_upload;
    FFDrawContext dc;
    FFDrawColor fontcolor[MAX_TEXT_NUM];  ///< foreground color
    FFDrawColor shadowcolor;        ///< shadow color
    FFDrawColor bordercolor;        ///< border color
    FFDrawColor boxcolor[MAX_TEXT_NUM];   ///< background color

    FT_Library library;             ///< freetype font library handle
    FT_Face face[MAX_TEXT_NUM];     ///< freetype font face handle
    FT_Stroker stroker;             ///< freetype stroker handle
    struct AVTreeNode *glyphs;      ///< rendered glyphs, stored using the UTF-32 char code
    char *x_expr[MAX_TEXT_NUM];     ///< expression for x position
    char *y_expr[MAX_TEXT_NUM];     ///< expression for y position
    AVExpr *x_pexpr[MAX_TEXT_NUM];  //< parsed expressions for x
    AVExpr *y_pexpr[MAX_TEXT_NUM];  ///< parsed expressions for y
    int64_t basetime;               ///< base pts time in the real world for display
    double var_values[VAR_VARS_NB];
    char   *a_expr;
    AVExpr *a_pexpr;
    int alpha;
    AVLFG  prng;                    ///< random
    char       *tc_opt_string;      ///< specified timecode option string
    AVRational  tc_rate;            ///< frame rate for timecode
    AVTimecode  tc;                 ///< timecode context
    int tc24hmax;                   ///< 1 if timecode is wrapped to 24 hours, 0 otherwise
    int reload;                     ///< reload text file for each frame
    int start_number;               ///< starting frame number for n/frame_num var
#if CONFIG_LIBFRIBIDI
    int text_shaping;               ///< 1 to shape the text before drawing it
#endif
    AVDictionary *metadata;

    // NI overlay related
    ni_session_context_t api_ctx;
    ni_session_data_io_t api_dst_frame;
    int session_opened;

    // NI HW frame upload related
    AVBufferRef *hwdevice;
    AVBufferRef *hwframe;
    AVBufferRef *hw_frames_ctx;

    // NI watermark related
    ni_scaler_multi_watermark_params_t scaler_watermark_paras;
    int watermark_width0;
    int watermark_width1;
    int watermark_height0;
    int watermark_height1;

    // NI ovly inplace crop related
    ni_session_context_t crop_api_ctx;
    ni_session_data_io_t crop_api_dst_frame;
    uint16_t ui16CropFrameIdx;
    int crop_session_opened;

    int keep_alive_timeout; /* keep alive timeout setting */
    int buffer_limit;

#if IS_FFMPEG_342_AND_ABOVE
    int initialized;
#else
    int input_config_initialized;
    int ni_config_initialized;
#endif
    int main_has_alpha;
    int use_watermark;
    AVBufferRef *out_frames_ref;

    // contains data downloaded from the input HW frame
    ni_session_data_io_t dl_frame;
    // contains text portion of overlaying frame
    ni_session_data_io_t txt_frame;

    AVFrame *up_frame;
    AVFrame *keep_overlay;
    int upload_drawtext_frame;
    int filtered_frame_count;
    int framerate;
    int initiated_upload_width;
    int initiated_upload_height;
} NetIntDrawTextContext;

static const enum AVPixelFormat alpha_pix_fmts[] = {
    AV_PIX_FMT_RGBA, AV_PIX_FMT_ARGB, AV_PIX_FMT_ABGR,
    AV_PIX_FMT_BGRA, AV_PIX_FMT_NONE
};

#define OFFSET(x) offsetof(NetIntDrawTextContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM

static const AVOption ni_drawtext_options[] = {
    { "fontfile",    "set font file",        OFFSET(fontfile[0]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff0",         "set font file",        OFFSET(fontfile[0]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff1",         "set font file",        OFFSET(fontfile[1]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff2",         "set font file",        OFFSET(fontfile[2]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff3",         "set font file",        OFFSET(fontfile[3]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff4",         "set font file",        OFFSET(fontfile[4]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff5",         "set font file",        OFFSET(fontfile[5]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff6",         "set font file",        OFFSET(fontfile[6]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff7",         "set font file",        OFFSET(fontfile[7]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff8",         "set font file",        OFFSET(fontfile[8]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff9",         "set font file",        OFFSET(fontfile[9]),           AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff10",        "set font file",        OFFSET(fontfile[10]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff11",        "set font file",        OFFSET(fontfile[11]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff12",        "set font file",        OFFSET(fontfile[12]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff13",        "set font file",        OFFSET(fontfile[13]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff14",        "set font file",        OFFSET(fontfile[14]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff15",        "set font file",        OFFSET(fontfile[15]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff16",        "set font file",        OFFSET(fontfile[16]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff17",        "set font file",        OFFSET(fontfile[17]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff18",        "set font file",        OFFSET(fontfile[18]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff19",        "set font file",        OFFSET(fontfile[19]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff20",        "set font file",        OFFSET(fontfile[20]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff21",        "set font file",        OFFSET(fontfile[21]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff22",        "set font file",        OFFSET(fontfile[22]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff23",        "set font file",        OFFSET(fontfile[23]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff24",        "set font file",        OFFSET(fontfile[24]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff25",        "set font file",        OFFSET(fontfile[25]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff26",        "set font file",        OFFSET(fontfile[26]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff27",        "set font file",        OFFSET(fontfile[27]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff28",        "set font file",        OFFSET(fontfile[28]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff29",        "set font file",        OFFSET(fontfile[29]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff30",        "set font file",        OFFSET(fontfile[30]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "ff31",        "set font file",        OFFSET(fontfile[31]),          AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "text",        "set text",             OFFSET(text[0]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t0",          "set text",             OFFSET(text[0]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t1",          "set text",             OFFSET(text[1]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t2",          "set text",             OFFSET(text[2]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t3",          "set text",             OFFSET(text[3]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t4",          "set text",             OFFSET(text[4]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t5",          "set text",             OFFSET(text[5]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t6",          "set text",             OFFSET(text[6]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t7",          "set text",             OFFSET(text[7]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t8",          "set text",             OFFSET(text[8]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t9",          "set text",             OFFSET(text[9]),               AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t10",         "set text",             OFFSET(text[10]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t11",         "set text",             OFFSET(text[11]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t12",         "set text",             OFFSET(text[12]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t13",         "set text",             OFFSET(text[13]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t14",         "set text",             OFFSET(text[14]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t15",         "set text",             OFFSET(text[15]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t16",         "set text",             OFFSET(text[16]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t17",         "set text",             OFFSET(text[17]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t18",         "set text",             OFFSET(text[18]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t19",         "set text",             OFFSET(text[19]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t20",         "set text",             OFFSET(text[20]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t21",         "set text",             OFFSET(text[21]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t22",         "set text",             OFFSET(text[22]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t23",         "set text",             OFFSET(text[23]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t24",         "set text",             OFFSET(text[24]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t25",         "set text",             OFFSET(text[25]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t26",         "set text",             OFFSET(text[26]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t27",         "set text",             OFFSET(text[27]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t28",         "set text",             OFFSET(text[28]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t29",         "set text",             OFFSET(text[29]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t30",         "set text",             OFFSET(text[30]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "t31",         "set text",             OFFSET(text[31]),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "textfile",    "set text file",        OFFSET(textfile),              AV_OPT_TYPE_STRING, {.str=NULL},  0, 0, FLAGS },
    { "fontcolor",   "set foreground color", OFFSET(fontcolor[0].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc0",         "set foreground color", OFFSET(fontcolor[0].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc1",         "set foreground color", OFFSET(fontcolor[1].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc2",         "set foreground color", OFFSET(fontcolor[2].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc3",         "set foreground color", OFFSET(fontcolor[3].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc4",         "set foreground color", OFFSET(fontcolor[4].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc5",         "set foreground color", OFFSET(fontcolor[5].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc6",         "set foreground color", OFFSET(fontcolor[6].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc7",         "set foreground color", OFFSET(fontcolor[7].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc8",         "set foreground color", OFFSET(fontcolor[8].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc9",         "set foreground color", OFFSET(fontcolor[9].rgba),     AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc10",        "set foreground color", OFFSET(fontcolor[10].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc11",        "set foreground color", OFFSET(fontcolor[11].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc12",        "set foreground color", OFFSET(fontcolor[12].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc13",        "set foreground color", OFFSET(fontcolor[13].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc14",        "set foreground color", OFFSET(fontcolor[14].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc15",        "set foreground color", OFFSET(fontcolor[15].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc16",        "set foreground color", OFFSET(fontcolor[16].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc17",        "set foreground color", OFFSET(fontcolor[17].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc18",        "set foreground color", OFFSET(fontcolor[18].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc19",        "set foreground color", OFFSET(fontcolor[19].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc20",        "set foreground color", OFFSET(fontcolor[20].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc21",        "set foreground color", OFFSET(fontcolor[21].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc22",        "set foreground color", OFFSET(fontcolor[22].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc23",        "set foreground color", OFFSET(fontcolor[23].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc24",        "set foreground color", OFFSET(fontcolor[24].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc25",        "set foreground color", OFFSET(fontcolor[25].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc26",        "set foreground color", OFFSET(fontcolor[26].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc27",        "set foreground color", OFFSET(fontcolor[27].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc28",        "set foreground color", OFFSET(fontcolor[28].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc29",        "set foreground color", OFFSET(fontcolor[29].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc30",        "set foreground color", OFFSET(fontcolor[30].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fc31",        "set foreground color", OFFSET(fontcolor[31].rgba),    AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "fontcolor_expr", "set foreground color expression", OFFSET(fontcolor_expr[0]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr0",    "set foreground color expression", OFFSET(fontcolor_expr[0]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr1",    "set foreground color expression", OFFSET(fontcolor_expr[1]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr2",    "set foreground color expression", OFFSET(fontcolor_expr[2]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr3",    "set foreground color expression", OFFSET(fontcolor_expr[3]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr4",    "set foreground color expression", OFFSET(fontcolor_expr[4]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr5",    "set foreground color expression", OFFSET(fontcolor_expr[5]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr6",    "set foreground color expression", OFFSET(fontcolor_expr[6]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr7",    "set foreground color expression", OFFSET(fontcolor_expr[7]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr8",    "set foreground color expression", OFFSET(fontcolor_expr[8]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr9",    "set foreground color expression", OFFSET(fontcolor_expr[9]),   AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr10",   "set foreground color expression", OFFSET(fontcolor_expr[10]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr11",   "set foreground color expression", OFFSET(fontcolor_expr[11]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr12",   "set foreground color expression", OFFSET(fontcolor_expr[12]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr13",   "set foreground color expression", OFFSET(fontcolor_expr[13]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr14",   "set foreground color expression", OFFSET(fontcolor_expr[14]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr15",   "set foreground color expression", OFFSET(fontcolor_expr[15]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr16",   "set foreground color expression", OFFSET(fontcolor_expr[16]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr17",   "set foreground color expression", OFFSET(fontcolor_expr[17]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr18",   "set foreground color expression", OFFSET(fontcolor_expr[18]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr19",   "set foreground color expression", OFFSET(fontcolor_expr[19]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr20",   "set foreground color expression", OFFSET(fontcolor_expr[20]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr21",   "set foreground color expression", OFFSET(fontcolor_expr[21]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr22",   "set foreground color expression", OFFSET(fontcolor_expr[22]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr23",   "set foreground color expression", OFFSET(fontcolor_expr[23]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr24",   "set foreground color expression", OFFSET(fontcolor_expr[24]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr25",   "set foreground color expression", OFFSET(fontcolor_expr[25]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr26",   "set foreground color expression", OFFSET(fontcolor_expr[26]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr27",   "set foreground color expression", OFFSET(fontcolor_expr[27]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr28",   "set foreground color expression", OFFSET(fontcolor_expr[28]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr29",   "set foreground color expression", OFFSET(fontcolor_expr[29]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr30",   "set foreground color expression", OFFSET(fontcolor_expr[30]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "fc_expr31",   "set foreground color expression", OFFSET(fontcolor_expr[31]),  AV_OPT_TYPE_STRING,  {.str=NULL}, 0, 0, FLAGS },
    { "boxcolor",    "set box color",        OFFSET(boxcolor[0].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc0",         "set box color",        OFFSET(boxcolor[0].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc1",         "set box color",        OFFSET(boxcolor[1].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc2",         "set box color",        OFFSET(boxcolor[2].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc3",         "set box color",        OFFSET(boxcolor[3].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc4",         "set box color",        OFFSET(boxcolor[4].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc5",         "set box color",        OFFSET(boxcolor[5].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc6",         "set box color",        OFFSET(boxcolor[6].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc7",         "set box color",        OFFSET(boxcolor[7].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc8",         "set box color",        OFFSET(boxcolor[8].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc9",         "set box color",        OFFSET(boxcolor[9].rgba),   AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc10",        "set box color",        OFFSET(boxcolor[10].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc11",        "set box color",        OFFSET(boxcolor[11].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc12",        "set box color",        OFFSET(boxcolor[12].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc13",        "set box color",        OFFSET(boxcolor[13].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc14",        "set box color",        OFFSET(boxcolor[14].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc15",        "set box color",        OFFSET(boxcolor[15].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc16",        "set box color",        OFFSET(boxcolor[16].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc17",        "set box color",        OFFSET(boxcolor[17].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc18",        "set box color",        OFFSET(boxcolor[18].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc19",        "set box color",        OFFSET(boxcolor[19].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc20",        "set box color",        OFFSET(boxcolor[20].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc21",        "set box color",        OFFSET(boxcolor[21].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc22",        "set box color",        OFFSET(boxcolor[22].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc23",        "set box color",        OFFSET(boxcolor[23].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc24",        "set box color",        OFFSET(boxcolor[24].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc25",        "set box color",        OFFSET(boxcolor[25].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc26",        "set box color",        OFFSET(boxcolor[26].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc27",        "set box color",        OFFSET(boxcolor[27].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc28",        "set box color",        OFFSET(boxcolor[28].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc29",        "set box color",        OFFSET(boxcolor[29].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc30",        "set box color",        OFFSET(boxcolor[30].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bc31",        "set box color",        OFFSET(boxcolor[31].rgba),  AV_OPT_TYPE_COLOR,  {.str="white"}, 0, 0, FLAGS },
    { "bordercolor", "set border color",     OFFSET(bordercolor.rgba),   AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "shadowcolor", "set shadow color",     OFFSET(shadowcolor.rgba),   AV_OPT_TYPE_COLOR,  {.str="black"}, 0, 0, FLAGS },
    { "box",         "set box",              OFFSET(draw_box),           AV_OPT_TYPE_BOOL,   {.i64=0},     0,        1       , FLAGS },
    { "boxborderw",  "set box borders width", OFFSET(boxborderw[0]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb0",         "set box borders width", OFFSET(boxborderw[0]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb1",         "set box borders width", OFFSET(boxborderw[1]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb2",         "set box borders width", OFFSET(boxborderw[2]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb3",         "set box borders width", OFFSET(boxborderw[3]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb4",         "set box borders width", OFFSET(boxborderw[4]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb5",         "set box borders width", OFFSET(boxborderw[5]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb6",         "set box borders width", OFFSET(boxborderw[6]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb7",         "set box borders width", OFFSET(boxborderw[7]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb8",         "set box borders width", OFFSET(boxborderw[8]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb9",         "set box borders width", OFFSET(boxborderw[9]),        AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb10",        "set box borders width", OFFSET(boxborderw[10]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb11",        "set box borders width", OFFSET(boxborderw[11]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb12",        "set box borders width", OFFSET(boxborderw[12]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb13",        "set box borders width", OFFSET(boxborderw[13]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb14",        "set box borders width", OFFSET(boxborderw[14]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb15",        "set box borders width", OFFSET(boxborderw[15]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb16",        "set box borders width", OFFSET(boxborderw[16]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb17",        "set box borders width", OFFSET(boxborderw[17]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb18",        "set box borders width", OFFSET(boxborderw[18]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb19",        "set box borders width", OFFSET(boxborderw[19]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb20",        "set box borders width", OFFSET(boxborderw[20]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb21",        "set box borders width", OFFSET(boxborderw[21]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb22",        "set box borders width", OFFSET(boxborderw[22]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb23",        "set box borders width", OFFSET(boxborderw[23]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb24",        "set box borders width", OFFSET(boxborderw[24]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb25",        "set box borders width", OFFSET(boxborderw[25]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb26",        "set box borders width", OFFSET(boxborderw[26]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb27",        "set box borders width", OFFSET(boxborderw[27]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb28",        "set box borders width", OFFSET(boxborderw[28]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb29",        "set box borders width", OFFSET(boxborderw[29]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb30",        "set box borders width", OFFSET(boxborderw[30]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "bb31",        "set box borders width", OFFSET(boxborderw[31]),       AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "line_spacing",  "set line spacing in pixels", OFFSET(line_spacing),   AV_OPT_TYPE_INT,    {.i64=0},     INT_MIN,  INT_MAX,FLAGS },
    { "fontsize",     "set font size",        OFFSET(fontsize_expr[0]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs0",          "set font size",        OFFSET(fontsize_expr[0]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs1",          "set font size",        OFFSET(fontsize_expr[1]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs2",          "set font size",        OFFSET(fontsize_expr[2]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs3",          "set font size",        OFFSET(fontsize_expr[3]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs4",          "set font size",        OFFSET(fontsize_expr[4]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs5",          "set font size",        OFFSET(fontsize_expr[5]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs6",          "set font size",        OFFSET(fontsize_expr[6]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs7",          "set font size",        OFFSET(fontsize_expr[7]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs8",          "set font size",        OFFSET(fontsize_expr[8]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs9",          "set font size",        OFFSET(fontsize_expr[9]),      AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs10",         "set font size",        OFFSET(fontsize_expr[10]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs11",         "set font size",        OFFSET(fontsize_expr[11]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs12",         "set font size",        OFFSET(fontsize_expr[12]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs13",         "set font size",        OFFSET(fontsize_expr[13]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs14",         "set font size",        OFFSET(fontsize_expr[14]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs15",         "set font size",        OFFSET(fontsize_expr[15]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs16",         "set font size",        OFFSET(fontsize_expr[16]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs17",         "set font size",        OFFSET(fontsize_expr[17]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs18",         "set font size",        OFFSET(fontsize_expr[18]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs19",         "set font size",        OFFSET(fontsize_expr[19]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs20",         "set font size",        OFFSET(fontsize_expr[20]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs21",         "set font size",        OFFSET(fontsize_expr[21]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs22",         "set font size",        OFFSET(fontsize_expr[22]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs23",         "set font size",        OFFSET(fontsize_expr[23]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs24",         "set font size",        OFFSET(fontsize_expr[24]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs25",         "set font size",        OFFSET(fontsize_expr[25]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs26",         "set font size",        OFFSET(fontsize_expr[26]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs27",         "set font size",        OFFSET(fontsize_expr[27]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs28",         "set font size",        OFFSET(fontsize_expr[28]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs29",         "set font size",        OFFSET(fontsize_expr[29]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs30",         "set font size",        OFFSET(fontsize_expr[30]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "fs31",         "set font size",        OFFSET(fontsize_expr[31]),     AV_OPT_TYPE_STRING, {.str="36"},  0, 0 , FLAGS },
    { "x",            "set x expression",     OFFSET(x_expr[0]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y",            "set y expression",     OFFSET(y_expr[0]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x0",           "set x expression",     OFFSET(x_expr[0]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y0",           "set y expression",     OFFSET(y_expr[0]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x1",           "set x expression",     OFFSET(x_expr[1]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y1",           "set y expression",     OFFSET(y_expr[1]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x2",           "set x expression",     OFFSET(x_expr[2]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y2",           "set y expression",     OFFSET(y_expr[2]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x3",           "set x expression",     OFFSET(x_expr[3]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y3",           "set y expression",     OFFSET(y_expr[3]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x4",           "set x expression",     OFFSET(x_expr[4]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y4",           "set y expression",     OFFSET(y_expr[4]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x5",           "set x expression",     OFFSET(x_expr[5]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y5",           "set y expression",     OFFSET(y_expr[5]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x6",           "set x expression",     OFFSET(x_expr[6]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y6",           "set y expression",     OFFSET(y_expr[6]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x7",           "set x expression",     OFFSET(x_expr[7]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y7",           "set y expression",     OFFSET(y_expr[7]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x8",           "set x expression",     OFFSET(x_expr[8]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y8",           "set y expression",     OFFSET(y_expr[8]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x9",           "set x expression",     OFFSET(x_expr[9]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y9",           "set y expression",     OFFSET(y_expr[9]),             AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x10",          "set x expression",     OFFSET(x_expr[10]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y10",          "set y expression",     OFFSET(y_expr[10]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x11",          "set x expression",     OFFSET(x_expr[11]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y11",          "set y expression",     OFFSET(y_expr[11]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x12",          "set x expression",     OFFSET(x_expr[12]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y12",          "set y expression",     OFFSET(y_expr[12]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x13",          "set x expression",     OFFSET(x_expr[13]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y13",          "set y expression",     OFFSET(y_expr[13]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x14",          "set x expression",     OFFSET(x_expr[14]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y14",          "set y expression",     OFFSET(y_expr[14]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x15",          "set x expression",     OFFSET(x_expr[15]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y15",          "set y expression",     OFFSET(y_expr[15]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x16",          "set x expression",     OFFSET(x_expr[16]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y16",          "set y expression",     OFFSET(y_expr[16]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x17",          "set x expression",     OFFSET(x_expr[17]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y17",          "set y expression",     OFFSET(y_expr[17]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x18",          "set x expression",     OFFSET(x_expr[18]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y18",          "set y expression",     OFFSET(y_expr[18]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x19",          "set x expression",     OFFSET(x_expr[19]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y19",          "set y expression",     OFFSET(y_expr[19]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x20",          "set x expression",     OFFSET(x_expr[20]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y20",          "set y expression",     OFFSET(y_expr[20]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x21",          "set x expression",     OFFSET(x_expr[21]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y21",          "set y expression",     OFFSET(y_expr[21]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x22",          "set x expression",     OFFSET(x_expr[22]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y22",          "set y expression",     OFFSET(y_expr[22]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x23",          "set x expression",     OFFSET(x_expr[23]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y23",          "set y expression",     OFFSET(y_expr[23]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x24",          "set x expression",     OFFSET(x_expr[24]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y24",          "set y expression",     OFFSET(y_expr[24]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x25",          "set x expression",     OFFSET(x_expr[25]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y25",          "set y expression",     OFFSET(y_expr[25]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x26",          "set x expression",     OFFSET(x_expr[26]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y26",          "set y expression",     OFFSET(y_expr[26]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x27",          "set x expression",     OFFSET(x_expr[27]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y27",          "set y expression",     OFFSET(y_expr[27]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x28",          "set x expression",     OFFSET(x_expr[28]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y28",          "set y expression",     OFFSET(y_expr[28]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x29",          "set x expression",     OFFSET(x_expr[29]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y29",          "set y expression",     OFFSET(y_expr[29]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x30",          "set x expression",     OFFSET(x_expr[30]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y30",          "set y expression",     OFFSET(y_expr[30]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "x31",          "set x expression",     OFFSET(x_expr[31]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "y31",          "set y expression",     OFFSET(y_expr[31]),            AV_OPT_TYPE_STRING, {.str="0"},   0, 0, FLAGS },
    { "shadowx",     "set shadow x offset",  OFFSET(shadowx),            AV_OPT_TYPE_INT,    {.i64=0},     INT_MIN,  INT_MAX , FLAGS },
    { "shadowy",     "set shadow y offset",  OFFSET(shadowy),            AV_OPT_TYPE_INT,    {.i64=0},     INT_MIN,  INT_MAX , FLAGS },
    { "borderw",     "set border width",     OFFSET(borderw),            AV_OPT_TYPE_INT,    {.i64=0},     INT_MIN,  INT_MAX , FLAGS },
    { "tabsize",     "set tab size",         OFFSET(tabsize[0]),         AV_OPT_TYPE_INT,    {.i64=4},     0,        INT_MAX , FLAGS },
    { "basetime",    "set base time",        OFFSET(basetime),           AV_OPT_TYPE_INT64,  {.i64=AV_NOPTS_VALUE}, INT64_MIN, INT64_MAX , FLAGS },
#if CONFIG_LIBFONTCONFIG
    { "font",      "Font name",            OFFSET(font[0]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f0",        "Font name",            OFFSET(font[0]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f1",        "Font name",            OFFSET(font[1]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f2",        "Font name",            OFFSET(font[2]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f3",        "Font name",            OFFSET(font[3]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f4",        "Font name",            OFFSET(font[4]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f5",        "Font name",            OFFSET(font[5]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f6",        "Font name",            OFFSET(font[6]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f7",        "Font name",            OFFSET(font[7]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f8",        "Font name",            OFFSET(font[8]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f9",        "Font name",            OFFSET(font[9]),               AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f10",       "Font name",            OFFSET(font[10]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f11",       "Font name",            OFFSET(font[11]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f12",       "Font name",            OFFSET(font[12]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f13",       "Font name",            OFFSET(font[13]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f14",       "Font name",            OFFSET(font[14]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f15",       "Font name",            OFFSET(font[15]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f16",       "Font name",            OFFSET(font[16]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f17",       "Font name",            OFFSET(font[17]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f18",       "Font name",            OFFSET(font[18]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f19",       "Font name",            OFFSET(font[19]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f20",       "Font name",            OFFSET(font[20]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f21",       "Font name",            OFFSET(font[21]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f22",       "Font name",            OFFSET(font[22]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f23",       "Font name",            OFFSET(font[23]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f24",       "Font name",            OFFSET(font[24]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f25",       "Font name",            OFFSET(font[25]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f26",       "Font name",            OFFSET(font[26]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f27",       "Font name",            OFFSET(font[27]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f28",       "Font name",            OFFSET(font[28]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f29",       "Font name",            OFFSET(font[29]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f30",       "Font name",            OFFSET(font[30]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
    { "f31",       "Font name",            OFFSET(font[31]),              AV_OPT_TYPE_STRING, { .str = "Sans" },           .flags = FLAGS  },
#endif
    { "expansion", "set the expansion mode", OFFSET(exp_mode), AV_OPT_TYPE_INT, {.i64=EXP_NORMAL}, 0, 2, FLAGS, "expansion" },
        { "none",     "set no expansion",                    OFFSET(exp_mode), AV_OPT_TYPE_CONST, {.i64=EXP_NONE},     0, 0, FLAGS, "expansion" },
        { "normal",   "set normal expansion",                OFFSET(exp_mode), AV_OPT_TYPE_CONST, {.i64=EXP_NORMAL},   0, 0, FLAGS, "expansion" },
        { "strftime", "set strftime expansion (deprecated)", OFFSET(exp_mode), AV_OPT_TYPE_CONST, {.i64=EXP_STRFTIME}, 0, 0, FLAGS, "expansion" },
    { "timecode",        "set initial timecode",             OFFSET(tc_opt_string), AV_OPT_TYPE_STRING,   {.str=NULL}, 0, 0, FLAGS },
    { "tc24hmax",        "set 24 hours max (timecode only)", OFFSET(tc24hmax),      AV_OPT_TYPE_BOOL,     {.i64=0},    0,        1, FLAGS },
    { "timecode_rate",   "set rate (timecode only)",         OFFSET(tc_rate),       AV_OPT_TYPE_RATIONAL, {.dbl=0},    0,  INT_MAX, FLAGS },
    { "r",               "set rate (timecode only)",         OFFSET(tc_rate),       AV_OPT_TYPE_RATIONAL, {.dbl=0},    0,  INT_MAX, FLAGS },
    { "rate",            "set rate (timecode only)",         OFFSET(tc_rate),       AV_OPT_TYPE_RATIONAL, {.dbl=0},    0,  INT_MAX, FLAGS },
    { "reload",     "reload text file for each frame",                       OFFSET(reload),     AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    {  "alpha",       "apply alpha while rendering", OFFSET(a_expr),      AV_OPT_TYPE_STRING, { .str = "1"     },          .flags = FLAGS  },
    { "fix_bounds", "check and fix text coords to avoid clipping", OFFSET(fix_bounds), AV_OPT_TYPE_BOOL, {.i64=0}, 0, 1, FLAGS },
    { "start_number", "start frame number for n/frame_num variable", OFFSET(start_number), AV_OPT_TYPE_INT, {.i64=0}, 0, INT_MAX, FLAGS },

#if CONFIG_LIBFRIBIDI
    { "text_shaping", "attempt to shape text before drawing", OFFSET(text_shaping), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS },
#endif

    /* FT_LOAD_* flags */
    { "ft_load_flags", "set font loading flags for libfreetype", OFFSET(ft_load_flags), AV_OPT_TYPE_FLAGS, { .i64 = FT_LOAD_DEFAULT }, 0, INT_MAX, FLAGS, "ft_load_flags" },
        { "default",                     NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_DEFAULT },                     .flags = FLAGS, .unit = "ft_load_flags" },
        { "no_scale",                    NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_NO_SCALE },                    .flags = FLAGS, .unit = "ft_load_flags" },
        { "no_hinting",                  NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_NO_HINTING },                  .flags = FLAGS, .unit = "ft_load_flags" },
        { "render",                      NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_RENDER },                      .flags = FLAGS, .unit = "ft_load_flags" },
        { "no_bitmap",                   NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_NO_BITMAP },                   .flags = FLAGS, .unit = "ft_load_flags" },
        { "vertical_layout",             NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_VERTICAL_LAYOUT },             .flags = FLAGS, .unit = "ft_load_flags" },
        { "force_autohint",              NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_FORCE_AUTOHINT },              .flags = FLAGS, .unit = "ft_load_flags" },
        { "crop_bitmap",                 NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_CROP_BITMAP },                 .flags = FLAGS, .unit = "ft_load_flags" },
        { "pedantic",                    NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_PEDANTIC },                    .flags = FLAGS, .unit = "ft_load_flags" },
        { "ignore_global_advance_width", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_IGNORE_GLOBAL_ADVANCE_WIDTH }, .flags = FLAGS, .unit = "ft_load_flags" },
        { "no_recurse",                  NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_NO_RECURSE },                  .flags = FLAGS, .unit = "ft_load_flags" },
        { "ignore_transform",            NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_IGNORE_TRANSFORM },            .flags = FLAGS, .unit = "ft_load_flags" },
        { "monochrome",                  NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_MONOCHROME },                  .flags = FLAGS, .unit = "ft_load_flags" },
        { "linear_design",               NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_LINEAR_DESIGN },               .flags = FLAGS, .unit = "ft_load_flags" },
        { "no_autohint",                 NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FT_LOAD_NO_AUTOHINT },                 .flags = FLAGS, .unit = "ft_load_flags" },
    { "optimize_upload", "Decrease the drawtext frame uploading frequency", OFFSET(optimize_upload), AV_OPT_TYPE_BOOL, {.i64=1}, 0, 1, FLAGS},
    { "use_watermark", "Use performance optimizations", OFFSET(use_watermark), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS }, // Deprecated language
    { "perf_optimization", "Use performance optimizations", OFFSET(use_watermark),       AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS },
    NI_FILT_OPTION_KEEPALIVE,
    NI_FILT_OPTION_BUFFER_LIMIT,
    { NULL }
};

AVFILTER_DEFINE_CLASS(ni_drawtext);

#undef __FTERRORS_H__
#define FT_ERROR_START_LIST {
#define FT_ERRORDEF(e, v, s) { (e), (s) },
#define FT_ERROR_END_LIST { 0, NULL } };

static const struct ft_error {
    int err;
    const char *err_msg;
} ft_errors[] =
#include FT_ERRORS_H

#define FT_ERRMSG(e) ft_errors[e].err_msg

typedef struct Glyph {
    FT_Glyph glyph;
    FT_Glyph border_glyph;
    uint32_t code;
    unsigned int fontsize;
    FT_Bitmap bitmap; ///< array holding bitmaps of font
    FT_Bitmap border_bitmap; ///< array holding bitmaps of font border
    FT_BBox bbox;
    int advance;
    int bitmap_left;
    int bitmap_top;
} Glyph;

static int glyph_cmp(const void *key, const void *b)
{
    const Glyph *a = key, *bb = b;
    int64_t diff = (int64_t)a->code - (int64_t)bb->code;

    if (diff != 0)
        return diff > 0 ? 1 : -1;
    else
        return FFDIFFSIGN((int64_t)a->fontsize, (int64_t)bb->fontsize);
}

/**
 * Load glyphs corresponding to the UTF-32 codepoint code.
 */
static int load_glyph(AVFilterContext *ctx, Glyph **glyph_ptr, uint32_t code, int index)
{
    NetIntDrawTextContext *s = ctx->priv;
    FT_BitmapGlyph bitmapglyph;
    Glyph *glyph, dummy = { 0 };
    struct AVTreeNode *node = NULL;
    int ret;

    /* load glyph into s->face->glyph */
    if (FT_Load_Char(s->face[index], code, s->ft_load_flags))
        return AVERROR(EINVAL);

    /* if glyph has already insert into s->glyphs, return directly */
    dummy.code = code;
    dummy.fontsize = s->fontsize[index];
    glyph = av_tree_find(s->glyphs, &dummy, glyph_cmp, NULL);
    if (glyph) {
        if (glyph_ptr)
            *glyph_ptr = glyph;
        return 0;
    }

    glyph = av_mallocz(sizeof(*glyph));
    if (!glyph) {
        ret = AVERROR(ENOMEM);
        goto error;
    }
    glyph->code  = code;
    glyph->fontsize = s->fontsize[index];

    if (FT_Get_Glyph(s->face[index]->glyph, &glyph->glyph)) {
        ret = AVERROR(EINVAL);
        goto error;
    }
    if (s->borderw) {
        glyph->border_glyph = glyph->glyph;
        if (FT_Glyph_StrokeBorder(&glyph->border_glyph, s->stroker, 0, 0) ||
            FT_Glyph_To_Bitmap(&glyph->border_glyph, FT_RENDER_MODE_NORMAL, 0, 1)) {
            ret = AVERROR_EXTERNAL;
            goto error;
        }
        bitmapglyph = (FT_BitmapGlyph) glyph->border_glyph;
        glyph->border_bitmap = bitmapglyph->bitmap;
    }
    if (FT_Glyph_To_Bitmap(&glyph->glyph, FT_RENDER_MODE_NORMAL, 0, 1)) {
        ret = AVERROR_EXTERNAL;
        goto error;
    }
    bitmapglyph = (FT_BitmapGlyph) glyph->glyph;

    glyph->bitmap      = bitmapglyph->bitmap;
    glyph->bitmap_left = bitmapglyph->left;
    glyph->bitmap_top  = bitmapglyph->top;
    glyph->advance     = s->face[index]->glyph->advance.x >> 6;

    /* measure text height to calculate text_height (or the maximum text height) */
    FT_Glyph_Get_CBox(glyph->glyph, ft_glyph_bbox_pixels, &glyph->bbox);

    /* cache the newly created glyph */
    if (!(node = av_tree_node_alloc())) {
        ret = AVERROR(ENOMEM);
        goto error;
    }
    av_tree_insert(&s->glyphs, glyph, glyph_cmp, &node);

    if (glyph_ptr)
        *glyph_ptr = glyph;
    return 0;

error:
    if (glyph)
        av_freep(&glyph->glyph);

    av_freep(&glyph);
    av_freep(&node);
    return ret;
}

// Convert a string formatted as "n1|n2|...|nN" into an integer array
static int string_to_array(const char *source, int *result, int result_size)
{
    int counter = 0, size = strlen(source) + 1;
    char *saveptr, *curval, *dup = av_malloc(size);
    if (!dup)
        return 0;
    av_strlcpy(dup, source, size);
    if (result_size > 0 && (curval = av_strtok(dup, "|", &saveptr))) {
        do {
            result[counter++] = atoi(curval);
        } while ((curval = av_strtok(NULL, "|", &saveptr)) && counter < result_size);
    }
    av_free(dup);
    return counter;
}

// convert FFmpeg AV_PIX_FMT_ to NI_PIX_FMT_
static int ff_to_ni_pix_fmt(int ff_av_pix_fmt)
{
    int pixel_format;

    switch (ff_av_pix_fmt) {
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
        av_log(NULL, AV_LOG_ERROR, "Pixel %d format not supported.\n",
               ff_av_pix_fmt);
        return AVERROR(EINVAL);
    }
    return pixel_format;
}

static av_cold int set_fontsize(AVFilterContext *ctx, unsigned int fontsize, int index)
{
    int err;
    NetIntDrawTextContext *s = ctx->priv;

    if ((err = FT_Set_Pixel_Sizes(s->face[index], 0, fontsize))) {
        av_log(ctx, AV_LOG_ERROR, "Could not set font size to %d pixels: %s\n",
               fontsize, FT_ERRMSG(err));
        return AVERROR(EINVAL);
    }

    s->fontsize[index] = fontsize;

    return 0;
}

static av_cold int parse_fontsize(AVFilterContext *ctx, int index)
{
    NetIntDrawTextContext *s = ctx->priv;
    int err;

    if (s->fontsize_pexpr[index])
        return 0;

    if (s->fontsize_expr[index] == NULL)
        return AVERROR(EINVAL);

    if ((err = av_expr_parse(&s->fontsize_pexpr[index], s->fontsize_expr[index], var_names,
                            NULL, NULL, fun2_names, fun2, 0, ctx)) < 0)
        return err;

    return 0;
}

static av_cold int update_fontsize(AVFilterContext *ctx, int index)
{
    NetIntDrawTextContext *s = ctx->priv;
    unsigned int fontsize = s->default_fontsize;
    int err;
    double size, roundedsize;

    // if no fontsize specified use the default
    if (s->fontsize_expr[index] != NULL) {
        if ((err = parse_fontsize(ctx, index)) < 0)
        return err;

        size = av_expr_eval(s->fontsize_pexpr[index], s->var_values, &s->prng);

        if (!isnan(size)) {
            roundedsize = round(size);
            // test for overflow before cast
            if (!(roundedsize > INT_MIN && roundedsize < INT_MAX)) {
                av_log(ctx, AV_LOG_ERROR, "fontsize overflow\n");
                return AVERROR(EINVAL);
            }

            fontsize = (int)roundedsize;
        }
    }

    if (fontsize == 0)
        fontsize = 1;

    // no change
    if (fontsize == s->fontsize[index])
        return 0;

    return set_fontsize(ctx, fontsize, index);
}

static int load_font_file(AVFilterContext *ctx, const char *path, int index, int text_index)
{
    NetIntDrawTextContext *s = ctx->priv;
    int err;

    err = FT_New_Face(s->library, path, index, &s->face[text_index]);
    if (err) {
#if !CONFIG_LIBFONTCONFIG
        av_log(ctx, AV_LOG_ERROR, "Could not load font \"%s\": %s\n",
               s->fontfile[text_index], FT_ERRMSG(err));
#endif
        return AVERROR(EINVAL);
    }
    return 0;
}

#if CONFIG_LIBFONTCONFIG
static int load_font_fontconfig(AVFilterContext *ctx, int text_index)
{
    NetIntDrawTextContext *s = ctx->priv;
    FcConfig *fontconfig;
    FcPattern *pat, *best;
    FcResult result = FcResultMatch;
    FcChar8 *filename;
    int index;
    double size;
    int err = AVERROR(ENOENT);
    int parse_err;

    fontconfig = FcInitLoadConfigAndFonts();
    if (!fontconfig) {
        av_log(ctx, AV_LOG_ERROR, "impossible to init fontconfig\n");
        return AVERROR_UNKNOWN;
    }
    pat = FcNameParse(s->fontfile[text_index] ? s->fontfile[text_index] :
                          (uint8_t *)(intptr_t)"default");
    if (!pat) {
        av_log(ctx, AV_LOG_ERROR, "could not parse fontconfig pat");
        return AVERROR(EINVAL);
    }

    FcPatternAddString(pat, FC_FAMILY, s->font[text_index]);

    parse_err = parse_fontsize(ctx, text_index);
    if (!parse_err) {
        double size = av_expr_eval(s->fontsize_pexpr[text_index], s->var_values, &s->prng);

        if (isnan(size)) {
            av_log(ctx, AV_LOG_ERROR, "impossible to find font information");
            return AVERROR(EINVAL);
        }

        FcPatternAddDouble(pat, FC_SIZE, size);
    }

    FcDefaultSubstitute(pat);

    if (!FcConfigSubstitute(fontconfig, pat, FcMatchPattern)) {
        av_log(ctx, AV_LOG_ERROR, "could not substitute fontconfig options"); /* very unlikely */
        FcPatternDestroy(pat);
        return AVERROR(ENOMEM);
    }

    best = FcFontMatch(fontconfig, pat, &result);
    FcPatternDestroy(pat);

    if (!best || result != FcResultMatch) {
        av_log(ctx, AV_LOG_ERROR,
               "Cannot find a valid font for the family %s\n",
               s->font[text_index]);
        goto fail;
    }

    if (
        FcPatternGetInteger(best, FC_INDEX, 0, &index   ) != FcResultMatch ||
        FcPatternGetDouble (best, FC_SIZE,  0, &size    ) != FcResultMatch) {
        av_log(ctx, AV_LOG_ERROR, "impossible to find font information");
        return AVERROR(EINVAL);
    }

    if (FcPatternGetString(best, FC_FILE, 0, &filename) != FcResultMatch) {
        av_log(ctx, AV_LOG_ERROR, "No file path for %s\n",
               s->font[text_index]);
        goto fail;
    }

    av_log(ctx, AV_LOG_INFO, "Using \"%s\"\n", filename);
    if (parse_err)
        s->default_fontsize = size + 0.5;

    err = load_font_file(ctx, filename, index, text_index);
    if (err)
        return err;
    FcConfigDestroy(fontconfig);
fail:
    FcPatternDestroy(best);
    return err;
}
#endif

static int load_font(AVFilterContext *ctx, int index)
{
    NetIntDrawTextContext *s = ctx->priv;
    int err;

    /* load the face, and set up the encoding, which is by default UTF-8 */
    if (s->fontfile[index]) {
        err = load_font_file(ctx, s->fontfile[index], 0, index);
        if (!err)
            return 0;
    }
#if CONFIG_LIBFONTCONFIG
    err = load_font_fontconfig(ctx, index);
    if (!err)
        return 0;
#endif
    return err;
}

static int load_textfile(AVFilterContext *ctx)
{
    NetIntDrawTextContext *s = ctx->priv;
    int err;
    uint8_t *textbuf;
    uint8_t *tmp;
    size_t textbuf_size;

    if ((err = av_file_map(s->textfile, &textbuf, &textbuf_size, 0, ctx)) < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "The text file '%s' could not be read or is empty\n",
               s->textfile);
        return err;
    }

    if (textbuf_size > SIZE_MAX - 1 || !(tmp = av_realloc(s->text[0], textbuf_size + 1))) {
        av_file_unmap(textbuf, textbuf_size);
        return AVERROR(ENOMEM);
    }
    s->text[0] = tmp;
    memcpy(s->text[0], textbuf, textbuf_size);
    s->text[0][textbuf_size] = 0;
    av_file_unmap(textbuf, textbuf_size);

    return 0;
}

static inline int is_newline(uint32_t c)
{
    return c == '\n' || c == '\r' || c == '\f' || c == '\v';
}

#if CONFIG_LIBFRIBIDI
static int shape_text(AVFilterContext *ctx)
{
    NetIntDrawTextContext *s = ctx->priv;
    uint8_t *tmp;
    int ret = AVERROR(ENOMEM);
    static const FriBidiFlags flags = FRIBIDI_FLAGS_DEFAULT |
                                      FRIBIDI_FLAGS_ARABIC;
    FriBidiChar *unicodestr = NULL;
    FriBidiStrIndex len;
    FriBidiParType direction = FRIBIDI_PAR_LTR;
    FriBidiStrIndex line_start = 0;
    FriBidiStrIndex line_end = 0;
    FriBidiLevel *embedding_levels = NULL;
    FriBidiArabicProp *ar_props = NULL;
    FriBidiCharType *bidi_types = NULL;
    FriBidiStrIndex i,j;

    len = strlen(s->text[0]);
    if (!(unicodestr = av_malloc_array(len, sizeof(*unicodestr)))) {
        goto out;
    }
    len = fribidi_charset_to_unicode(FRIBIDI_CHAR_SET_UTF8,
                                     s->text[0], len, unicodestr);

    bidi_types = av_malloc_array(len, sizeof(*bidi_types));
    if (!bidi_types) {
        goto out;
    }

    fribidi_get_bidi_types(unicodestr, len, bidi_types);

    embedding_levels = av_malloc_array(len, sizeof(*embedding_levels));
    if (!embedding_levels) {
        goto out;
    }

    if (!fribidi_get_par_embedding_levels(bidi_types, len, &direction,
                                          embedding_levels)) {
        goto out;
    }

    ar_props = av_malloc_array(len, sizeof(*ar_props));
    if (!ar_props) {
        goto out;
    }

    fribidi_get_joining_types(unicodestr, len, ar_props);
    fribidi_join_arabic(bidi_types, len, embedding_levels, ar_props);
    fribidi_shape(flags, embedding_levels, len, ar_props, unicodestr);

    for (line_end = 0, line_start = 0; line_end < len; line_end++) {
        if (is_newline(unicodestr[line_end]) || line_end == len - 1) {
            if (!fribidi_reorder_line(flags, bidi_types,
                                      line_end - line_start + 1, line_start,
                                      direction, embedding_levels, unicodestr,
                                      NULL)) {
                goto out;
            }
            line_start = line_end + 1;
        }
    }

    /* Remove zero-width fill chars put in by libfribidi */
    for (i = 0, j = 0; i < len; i++)
        if (unicodestr[i] != FRIBIDI_CHAR_FILL)
            unicodestr[j++] = unicodestr[i];
    len = j;

    if (!(tmp = av_realloc(s->text[0], (len * 4 + 1) * sizeof(*s->text[0])))) {
        /* Use len * 4, as a unicode character can be up to 4 bytes in UTF-8 */
        goto out;
    }

    s->text[0] = tmp;
    len = fribidi_unicode_to_charset(FRIBIDI_CHAR_SET_UTF8,
                                     unicodestr, len, s->text[0]);
    ret = 0;

out:
    av_free(unicodestr);
    av_free(embedding_levels);
    av_free(ar_props);
    av_free(bidi_types);
    return ret;
}
#endif

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

#if IS_FFMPEG_342_AND_ABOVE
static int config_output(AVFilterLink *outlink)
#else
static int config_output(AVFilterLink *outlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = ctx->inputs[0];
    NetIntDrawTextContext *s   = ctx->priv;

    AVHWFramesContext *in_frames_ctx;
    AVHWFramesContext *out_frames_ctx;
    int ni_pix_fmt;

    av_log(ctx, AV_LOG_DEBUG, "%s inlink wxh %dx%d\n", __func__,
           inlink->w, inlink->h);

    outlink->w = inlink->w;
    outlink->h = inlink->h;

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)ctx->inputs[0]->hw_frames_ctx->data;
#else
    if (in->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif

    av_log(ctx, AV_LOG_INFO, "vf_drawtext_ni.c %s in_frames_ctx->sw_format: %d "
           "%s\n", __func__, in_frames_ctx->sw_format,
           av_get_pix_fmt_name(in_frames_ctx->sw_format));
    if ((ni_pix_fmt = ff_to_ni_pix_fmt(in_frames_ctx->sw_format)) < 0) {
        return AVERROR(EINVAL);
    }


    s->out_frames_ref = av_hwframe_ctx_alloc(in_frames_ctx->device_ref);
    if (!s->out_frames_ref)
        return AVERROR(ENOMEM);

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;
    out_frames_ctx->format            = AV_PIX_FMT_NI_QUAD;
    out_frames_ctx->width             = outlink->w;
    out_frames_ctx->height            = outlink->h;
    out_frames_ctx->sw_format         = in_frames_ctx->sw_format;
    out_frames_ctx->initial_pool_size = NI_DRAWTEXT_ID;

    av_hwframe_ctx_init(s->out_frames_ref);
#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *lo = ff_filter_link(ctx->outputs[0]);
    av_buffer_unref(&lo->hw_frames_ctx);
    lo->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!lo->hw_frames_ctx)
        return AVERROR(ENOMEM);

    //The upload will be per frame if frame rate is not specified/determined
    if (li->frame_rate.den)
        s->framerate = (li->frame_rate.num + li->frame_rate.den - 1) / li->frame_rate.den;
#else
    av_buffer_unref(&ctx->outputs[0]->hw_frames_ctx);
    ctx->outputs[0]->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
    if (!ctx->outputs[0]->hw_frames_ctx)
        return AVERROR(ENOMEM);

    //The upload will be per frame if frame rate is not specified/determined
    if (inlink->frame_rate.den)
        s->framerate = (inlink->frame_rate.num + inlink->frame_rate.den - 1) / inlink->frame_rate.den;
#endif

    if (s->framerate == 0)
        s->framerate = 1;
    av_log(ctx, AV_LOG_INFO, "overlay frame upload frequency %d\n", s->framerate);

    return 0;
}

static int glyph_enu_free(void *opaque, void *elem)
{
    Glyph *glyph = elem;

    FT_Done_Glyph(glyph->glyph);
    FT_Done_Glyph(glyph->border_glyph);
    av_free(elem);
    return 0;
}


static av_cold int init(AVFilterContext *ctx)
{
    int i, err;
    NetIntDrawTextContext *s = ctx->priv;
    Glyph *glyph;

    for (i = 0; i < s->text_num; i++) {
        av_expr_free(s->fontsize_pexpr[i]);
        s->fontsize_pexpr[i] = NULL;

        s->fontsize[i] = 0;
    }
    for (i = 0; i < MAX_TEXT_NUM; i++) {
        s->text_last_updated[0] = NULL;
        s->x_bak[i] = 0;
        s->y_bak[i] = 0;
    }
    s->default_fontsize = 16;
    s->upload_drawtext_frame = 1;
    s->keep_overlay = NULL;
    s->filtered_frame_count=0;
    s->framerate = 0;

    if (!s->fontfile && !CONFIG_LIBFONTCONFIG) {
        av_log(ctx, AV_LOG_ERROR, "No font filename provided\n");
        return AVERROR(EINVAL);
    }

    if (s->textfile) {
        if (s->text[0]) {
            av_log(ctx, AV_LOG_ERROR,
                   "Both text and text file provided. Please provide only one\n");
            return AVERROR(EINVAL);
        }
        if ((err = load_textfile(ctx)) < 0)
            return err;
    }

    s->text_num = 0;
    for (i = 0; i < MAX_TEXT_NUM; i++) {
        if (!s->text[i]) {
            break;
        }
        s->text_num++;
    }

    if (s->reload && !s->textfile)
        av_log(ctx, AV_LOG_WARNING, "No file to reload\n");

    if (s->tc_opt_string) {
        int ret = av_timecode_init_from_string(&s->tc, s->tc_rate,
                                               s->tc_opt_string, ctx);
        if (ret < 0)
            return ret;
        if (s->tc24hmax)
            s->tc.flags |= AV_TIMECODE_FLAG_24HOURSMAX;
        if (!s->text[0])
            s->text[0] = av_strdup("");
    }

    if (!s->text_num) {
        av_log(ctx, AV_LOG_ERROR,
               "Either text, a valid file or a timecode must be provided\n");
        return AVERROR(EINVAL);
    }

#if CONFIG_LIBFRIBIDI
    if (s->text_shaping)
        if ((err = shape_text(ctx)) < 0)
            return err;
#endif

    if ((err = FT_Init_FreeType(&(s->library)))) {
        av_log(ctx, AV_LOG_ERROR,
               "Could not load FreeType: %s\n", FT_ERRMSG(err));
        return AVERROR(EINVAL);
    }

    for (i = 0; i < s->text_num; i++) {
        if ((err = load_font(ctx, i)) < 0)
            return err;

        if ((err = update_fontsize(ctx, i)) < 0)
            return err;
    }

    if (s->borderw) {
        if (FT_Stroker_New(s->library, &s->stroker)) {
            av_log(ctx, AV_LOG_ERROR, "Coult not init FT stroker\n");
            return AVERROR_EXTERNAL;
        }
        FT_Stroker_Set(s->stroker, s->borderw << 6, FT_STROKER_LINECAP_ROUND,
                       FT_STROKER_LINEJOIN_ROUND, 0);
    }

    for (i = 0; i < s->text_num; i++) {
        s->use_kerning[i] = FT_HAS_KERNING(s->face[i]);

        /* load the fallback glyph with code 0 */
        load_glyph(ctx, NULL, 0, i);

        /* set the tabsize in pixels */
        if ((err = load_glyph(ctx, &glyph, ' ', i)) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Could not set tabsize.\n");
            return err;
        }
        if (i > 0) {
            s->tabsize[i] = s->tabsize[0];
        }
        s->tabsize[i] *= glyph->advance;

        if (s->exp_mode == EXP_STRFTIME &&
            (strchr(s->text[i], '%') || strchr(s->text[i], '\\')))
            av_log(ctx, AV_LOG_WARNING, "expansion=strftime is deprecated.\n");
    }

    av_bprint_init(&s->expanded_text, 0, AV_BPRINT_SIZE_UNLIMITED);
    av_bprint_init(&s->expanded_fontcolor, 0, AV_BPRINT_SIZE_UNLIMITED);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    NetIntDrawTextContext *s = ctx->priv;
    int i;
    // NI HW frame related uninit
    av_frame_free(&s->keep_overlay);
    ni_frame_buffer_free(&s->dl_frame.data.frame);
    ni_frame_buffer_free(&s->txt_frame.data.frame);
    av_frame_free(&s->up_frame);

    if (s->api_dst_frame.data.frame.p_buffer) {
        ni_frame_buffer_free(&s->api_dst_frame.data.frame);
    }

    if (s->crop_api_dst_frame.data.frame.p_buffer) {
        ni_frame_buffer_free(&s->crop_api_dst_frame.data.frame);
    }

    if (s->session_opened) {
        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
    }

    if (s->crop_session_opened) {
        ni_device_session_close(&s->crop_api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->crop_api_ctx);
    }

    av_buffer_unref(&s->hwframe);
    av_buffer_unref(&s->hwdevice);
    av_buffer_unref(&s->hw_frames_ctx);

    av_buffer_unref(&s->out_frames_ref);

    for (i = 0; i < s->text_num; i++) {
        av_expr_free(s->x_pexpr[i]);
        av_expr_free(s->y_pexpr[i]);
        av_expr_free(s->fontsize_pexpr[i]);
        av_free(s->text_last_updated[i]);
        s->text_last_updated[i] = NULL;

        s->x_pexpr[i] = s->y_pexpr[i] = s->fontsize_pexpr[i] = NULL;
    }
    av_expr_free(s->a_pexpr);
    s->a_pexpr = NULL;

    av_freep(&s->positions);
    s->nb_positions = 0;

    av_tree_enumerate(s->glyphs, NULL, NULL, glyph_enu_free);
    av_tree_destroy(s->glyphs);
    s->glyphs = NULL;

    for (i = 0; i < s->text_num; i++) {
        FT_Done_Face(s->face[i]);
    }
    FT_Stroker_Done(s->stroker);
    FT_Done_FreeType(s->library);

    av_bprint_finalize(&s->expanded_text, NULL);
    av_bprint_finalize(&s->expanded_fontcolor, NULL);
}

#if IS_FFMPEG_342_AND_ABOVE
static int config_input(AVFilterLink *inlink)
#else
static int config_input(AVFilterLink *inlink, AVFrame *in)
#endif
{
    AVFilterContext *ctx = inlink->dst;
    NetIntDrawTextContext *s = ctx->priv;
    char *expr;
    int i, ret, flags;
    AVHWFramesContext *in_frames_ctx;

#if !IS_FFMPEG_342_AND_ABOVE
    AVFilterLink *outlink = ctx->outputs[0];
    if (s->input_config_initialized)
        return 0;
#endif

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(ctx->inputs[0]);
    if (li->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)li->hw_frames_ctx->data;
#elif IS_FFMPEG_342_AND_ABOVE
    if (ctx->inputs[0]->hw_frames_ctx == NULL) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext *)ctx->inputs[0]->hw_frames_ctx->data;
#else
    in_frames_ctx = (AVHWFramesContext *)in->hw_frames_ctx->data;
#endif

    av_log(ctx, AV_LOG_INFO, "vf_drawtext_ni.c: inlink->format %d "
           "in_frames_ctx->sw_format %d %s\n",
           inlink->format, in_frames_ctx->sw_format,
           av_get_pix_fmt_name(in_frames_ctx->sw_format));

    switch (in_frames_ctx->sw_format) {
    case AV_PIX_FMT_BGRP:
    case AV_PIX_FMT_NI_QUAD_8_TILE_4X4:
    case AV_PIX_FMT_NI_QUAD_10_TILE_4X4:
        av_log(ctx, AV_LOG_ERROR, "Error vf_drawtext_ni.c: frame pixel format "
               "not supported !\n");
        return AVERROR(EINVAL);
    default:
        break;
    }

    s->main_has_alpha = ff_fmt_is_in(in_frames_ctx->sw_format, alpha_pix_fmts);

// only FFmpeg 3.4.2 and above have flags
#if IS_FFMPEG_342_AND_ABOVE
    flags = FF_DRAW_PROCESS_ALPHA;
#else
    flags = 0;
#endif
    if (ff_draw_init(&s->dc, AV_PIX_FMT_RGBA, flags)
        < 0) {
        av_log(ctx, AV_LOG_ERROR, "Error vf_drawtext_ni.c: frame pixel format "
               "not supported !\n");
        return AVERROR(EINVAL);
    } else {
        av_log(ctx, AV_LOG_INFO, "%s ff_draw_init success main_has_alpha: %d\n",
               __func__, s->main_has_alpha);
    }

    for (i = 0; i < s->text_num; i++) {
        ff_draw_color(&s->dc, &s->fontcolor[i],   s->fontcolor[i].rgba);
        ff_draw_color(&s->dc, &s->boxcolor[i],    s->boxcolor[i].rgba);
    }
    ff_draw_color(&s->dc, &s->shadowcolor, s->shadowcolor.rgba);
    ff_draw_color(&s->dc, &s->bordercolor, s->bordercolor.rgba);

    s->var_values[VAR_w]     = s->var_values[VAR_W]     = s->var_values[VAR_MAIN_W] = inlink->w;
    s->var_values[VAR_h]     = s->var_values[VAR_H]     = s->var_values[VAR_MAIN_H] = inlink->h;
    s->var_values[VAR_SAR]   = inlink->sample_aspect_ratio.num ? av_q2d(inlink->sample_aspect_ratio) : 1;
    s->var_values[VAR_DAR]   = (double)inlink->w / inlink->h * s->var_values[VAR_SAR];
    s->var_values[VAR_HSUB]  = 1 << s->dc.hsub_max;
    s->var_values[VAR_VSUB]  = 1 << s->dc.vsub_max;
    s->var_values[VAR_X]     = NAN;
    s->var_values[VAR_Y]     = NAN;
    s->var_values[VAR_T]     = NAN;

    av_lfg_init(&s->prng, av_get_random_seed());

    for (i = 0; i < s->text_num; i++) {
        av_expr_free(s->x_pexpr[i]);
        av_expr_free(s->y_pexpr[i]);

        s->x_pexpr[i] = s->y_pexpr[i] = NULL;

        if ((ret = av_expr_parse(&s->x_pexpr[i], expr = s->x_expr[i], var_names,
                                NULL, NULL, fun2_names, fun2, 0, ctx)) < 0 ||
            (ret = av_expr_parse(&s->y_pexpr[i], expr = s->y_expr[i], var_names,
                                NULL, NULL, fun2_names, fun2, 0, ctx)) < 0 ) {
            av_log(ctx, AV_LOG_ERROR, "Failed to parse expression: %s \n", expr);
            return AVERROR(EINVAL);
        }
    }

    av_expr_free(s->a_pexpr);
    s->a_pexpr = NULL;

    if (ret = av_expr_parse(&s->a_pexpr, expr = s->a_expr, var_names,
                                NULL, NULL, fun2_names, fun2, 0, ctx) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to parse expression: %s \n", expr);
        return AVERROR(EINVAL);
    }

#if !IS_FFMPEG_342_AND_ABOVE
    ret = config_output(outlink, in);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed config_output\n");
        return AVERROR(EINVAL);
    }

    s->input_config_initialized = 1;
#endif

    // prep download/upload buffer
    if (ni_frame_buffer_alloc_dl(&(s->dl_frame.data.frame),
                                 inlink->w, inlink->h, NI_PIX_FMT_RGBA)) {
        return AVERROR(ENOMEM);
    }
    s->up_frame = av_frame_alloc();

    return 0;
}

#if IS_FFMPEG_43_AND_ABOVE
static int command(AVFilterContext *ctx, const char *cmd, const char *arg, char *res, int res_len, int flags)
{
    NetIntDrawTextContext *old = ctx->priv;
    NetIntDrawTextContext *new = NULL;
    int ret;

    if (!strcmp(cmd, "reinit")) {
        new = av_mallocz(sizeof(NetIntDrawTextContext));
        if (!new)
            return AVERROR(ENOMEM);

        new->class = &ni_drawtext_class;
        ret = av_opt_copy(new, old);
        if (ret < 0)
            goto fail;

        ctx->priv = new;
        ret = av_set_options_string(ctx, arg, "=", ":");
        if (ret < 0) {
            ctx->priv = old;
            goto fail;
        }

        ret = init(ctx);
        if (ret < 0) {
            uninit(ctx);
            ctx->priv = old;
            goto fail;
        }

        new->reinit = 1;
        new->initialized = 0;
        new->out_frames_ref = av_buffer_ref(old->out_frames_ref);

        ctx->priv = old;
        // NETINT/FFmpeg-patch:
        // fix memory leak for ffmpeg while using this function
        av_opt_free(old);
        uninit(ctx);
        av_freep(&old);

        ctx->priv = new;
        return config_input(ctx->inputs[0]);
    } else {
        return AVERROR(ENOSYS);
    }

fail:
    av_log(ctx, AV_LOG_ERROR, "Failed to process command. Continuing with existing parameters.\n");
    av_freep(&new);
    return ret;
}
#endif

static int func_pict_type(AVFilterContext *ctx, AVBPrint *bp,
                          char *fct, unsigned argc, char **argv, int tag)
{
    NetIntDrawTextContext *s = ctx->priv;

    av_bprintf(bp, "%c", av_get_picture_type_char(s->var_values[VAR_PICT_TYPE]));
    return 0;
}

static int func_pts(AVFilterContext *ctx, AVBPrint *bp,
                    char *fct, unsigned argc, char **argv, int tag)
{
    NetIntDrawTextContext *s = ctx->priv;
    const char *fmt;
    double pts = s->var_values[VAR_T];
    int ret;

    fmt = argc >= 1 ? argv[0] : "flt";
    if (argc >= 2) {
        int64_t delta;
        if ((ret = av_parse_time(&delta, argv[1], 1)) < 0) {
            av_log(ctx, AV_LOG_ERROR, "Invalid delta '%s'\n", argv[1]);
            return ret;
        }
        pts += (double)delta / AV_TIME_BASE;
    }
    if (!strcmp(fmt, "flt")) {
        av_bprintf(bp, "%.6f", pts);
    } else if (!strcmp(fmt, "hms")) {
        if (isnan(pts)) {
            av_bprintf(bp, " ??:??:??.???");
        } else {
            int64_t ms = llrint(pts * 1000);
            char sign = ' ';
            if (ms < 0) {
                sign = '-';
                ms = -ms;
            }
            if (argc >= 3) {
                if (!strcmp(argv[2], "24HH")) {
                    ms %= 24 * 60 * 60 * 1000;
                } else {
                    av_log(ctx, AV_LOG_ERROR, "Invalid argument '%s'\n", argv[2]);
                    return AVERROR(EINVAL);
                }
            }
            av_bprintf(bp, "%c%02d:%02d:%02d.%03d", sign,
                       (int)(ms / (60 * 60 * 1000)),
                       (int)(ms / (60 * 1000)) % 60,
                       (int)(ms / 1000) % 60,
                       (int)(ms % 1000));
        }
    } else if (!strcmp(fmt, "localtime") ||
               !strcmp(fmt, "gmtime")) {
        struct tm tm;
        time_t ms = (time_t)pts;
        const char *timefmt = argc >= 3 ? argv[2] : "%Y-%m-%d %H:%M:%S";
        if (!strcmp(fmt, "localtime"))
            localtime_r(&ms, &tm);
        else
            gmtime_r(&ms, &tm);
        av_bprint_strftime(bp, timefmt, &tm);
    } else {
        av_log(ctx, AV_LOG_ERROR, "Invalid format '%s'\n", fmt);
        return AVERROR(EINVAL);
    }
    return 0;
}

static int func_frame_num(AVFilterContext *ctx, AVBPrint *bp,
                          char *fct, unsigned argc, char **argv, int tag)
{
    NetIntDrawTextContext *s = ctx->priv;

    av_bprintf(bp, "%d", (int)s->var_values[VAR_N]);
    return 0;
}

static int func_metadata(AVFilterContext *ctx, AVBPrint *bp,
                         char *fct, unsigned argc, char **argv, int tag)
{
    NetIntDrawTextContext *s = ctx->priv;
    AVDictionaryEntry *e = av_dict_get(s->metadata, argv[0], NULL, 0);

    if (e && e->value)
        av_bprintf(bp, "%s", e->value);
    else if (argc >= 2)
        av_bprintf(bp, "%s", argv[1]);
    return 0;
}

static int func_strftime(AVFilterContext *ctx, AVBPrint *bp,
                         char *fct, unsigned argc, char **argv, int tag)
{
    const char *fmt = argc ? argv[0] : "%Y-%m-%d %H:%M:%S";
    time_t now;
    struct tm tm;

    time(&now);
    if (tag == 'L')
        localtime_r(&now, &tm);
    else
        tm = *gmtime_r(&now, &tm);
    av_bprint_strftime(bp, fmt, &tm);
    return 0;
}

static int func_eval_expr(AVFilterContext *ctx, AVBPrint *bp,
                          char *fct, unsigned argc, char **argv, int tag)
{
    NetIntDrawTextContext *s = ctx->priv;
    double res;
    int ret;

    ret = av_expr_parse_and_eval(&res, argv[0], var_names, s->var_values,
                                 NULL, NULL, fun2_names, fun2,
                                 &s->prng, 0, ctx);
    if (ret < 0)
        av_log(ctx, AV_LOG_ERROR,
               "Expression '%s' for the expr text expansion function is not valid\n",
               argv[0]);
    else
        av_bprintf(bp, "%f", res);

    return ret;
}

static int func_eval_expr_int_format(AVFilterContext *ctx, AVBPrint *bp,
                          char *fct, unsigned argc, char **argv, int tag)
{
    NetIntDrawTextContext *s = ctx->priv;
    double res;
    int intval;
    int ret;
    unsigned int positions = 0;
    char fmt_str[30] = "%";

    /*
     * argv[0] expression to be converted to `int`
     * argv[1] format: 'x', 'X', 'd' or 'u'
     * argv[2] positions printed (optional)
     */

    ret = av_expr_parse_and_eval(&res, argv[0], var_names, s->var_values,
                                 NULL, NULL, fun2_names, fun2,
                                 &s->prng, 0, ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "Expression '%s' for the expr text expansion function is not valid\n",
               argv[0]);
        return ret;
    }

    if (!strchr("xXdu", argv[1][0])) {
        av_log(ctx, AV_LOG_ERROR, "Invalid format '%c' specified,"
                " allowed values: 'x', 'X', 'd', 'u'\n", argv[1][0]);
        return AVERROR(EINVAL);
    }

    if (argc == 3) {
        ret = sscanf(argv[2], "%u", &positions);
        if (ret != 1) {
            av_log(ctx, AV_LOG_ERROR, "expr_int_format(): Invalid number of positions"
                    " to print: '%s'\n", argv[2]);
            return AVERROR(EINVAL);
        }
    }

    feclearexcept(FE_ALL_EXCEPT);
    intval = res;
#if defined(FE_INVALID) && defined(FE_OVERFLOW) && defined(FE_UNDERFLOW)
    if ((ret = fetestexcept(FE_INVALID|FE_OVERFLOW|FE_UNDERFLOW))) {
        av_log(ctx, AV_LOG_ERROR, "Conversion of floating-point result to int failed. Control register: 0x%08x. Conversion result: %d\n", ret, intval);
        return AVERROR(EINVAL);
    }
#endif

    if (argc == 3)
        av_strlcatf(fmt_str, sizeof(fmt_str), "0%u", positions);
    av_strlcatf(fmt_str, sizeof(fmt_str), "%c", argv[1][0]);

    av_log(ctx, AV_LOG_DEBUG, "Formatting value %f (expr '%s') with spec '%s'\n",
            res, argv[0], fmt_str);

    av_bprintf(bp, fmt_str, intval);

    return 0;
}

static const struct drawtext_function {
    const char *name;
    unsigned argc_min, argc_max;
    int tag;                            /**< opaque argument to func */
    int (*func)(AVFilterContext *, AVBPrint *, char *, unsigned, char **, int);
} functions[] = {
    { "expr",      1, 1, 0,   func_eval_expr },
    { "e",         1, 1, 0,   func_eval_expr },
    { "expr_int_format", 2, 3, 0, func_eval_expr_int_format },
    { "eif",       2, 3, 0,   func_eval_expr_int_format },
    { "pict_type", 0, 0, 0,   func_pict_type },
    { "pts",       0, 3, 0,   func_pts      },
    { "gmtime",    0, 1, 'G', func_strftime },
    { "localtime", 0, 1, 'L', func_strftime },
    { "frame_num", 0, 0, 0,   func_frame_num },
    { "n",         0, 0, 0,   func_frame_num },
    { "metadata",  1, 2, 0,   func_metadata },
};

static int eval_function(AVFilterContext *ctx, AVBPrint *bp, char *fct,
                         unsigned argc, char **argv)
{
    unsigned i;

    for (i = 0; i < FF_ARRAY_ELEMS(functions); i++) {
        if (strcmp(fct, functions[i].name))
            continue;
        if (argc < functions[i].argc_min) {
            av_log(ctx, AV_LOG_ERROR, "%%{%s} requires at least %d arguments\n",
                   fct, functions[i].argc_min);
            return AVERROR(EINVAL);
        }
        if (argc > functions[i].argc_max) {
            av_log(ctx, AV_LOG_ERROR, "%%{%s} requires at most %d arguments\n",
                   fct, functions[i].argc_max);
            return AVERROR(EINVAL);
        }
        break;
    }
    if (i >= FF_ARRAY_ELEMS(functions)) {
        av_log(ctx, AV_LOG_ERROR, "%%{%s} is not known\n", fct);
        return AVERROR(EINVAL);
    }
    return functions[i].func(ctx, bp, fct, argc, argv, functions[i].tag);
}

static int expand_function(AVFilterContext *ctx, AVBPrint *bp, char **rtext)
{
    const char *text = *rtext;
    char *argv[16] = { NULL };
    unsigned argc = 0, i;
    int ret;

    if (*text != '{') {
        av_log(ctx, AV_LOG_ERROR, "Stray %% near '%s'\n", text);
        return AVERROR(EINVAL);
    }
    text++;
    while (1) {
        if (!(argv[argc++] = av_get_token(&text, ":}"))) {
            ret = AVERROR(ENOMEM);
            goto end;
        }
        if (!*text) {
            av_log(ctx, AV_LOG_ERROR, "Unterminated %%{} near '%s'\n", *rtext);
            ret = AVERROR(EINVAL);
            goto end;
        }
        if (argc == FF_ARRAY_ELEMS(argv))
            av_freep(&argv[--argc]); /* error will be caught later */
        if (*text == '}')
            break;
        text++;
    }

    if ((ret = eval_function(ctx, bp, argv[0], argc - 1, argv + 1)) < 0)
        goto end;
    ret = 0;
    *rtext = (char *)text + 1;

end:
    for (i = 0; i < argc; i++)
        av_freep(&argv[i]);
    return ret;
}

static int expand_text(AVFilterContext *ctx, char *text, AVBPrint *bp)
{
    int ret;

    av_bprint_clear(bp);
    while (*text) {
        if (*text == '\\' && text[1]) {
            av_bprint_chars(bp, text[1], 1);
            text += 2;
        } else if (*text == '%') {
            text++;
            if ((ret = expand_function(ctx, bp, &text)) < 0)
                return ret;
        } else {
            av_bprint_chars(bp, *text, 1);
            text++;
        }
    }
    if (!av_bprint_is_complete(bp))
        return AVERROR(ENOMEM);
    return 0;
}

static int draw_glyphs(NetIntDrawTextContext *s, ni_frame_t *frame,
                       int width, int height,
                       FFDrawColor *color,
                       int x, int y, int borderw, int index)
{
    char *text = s->expanded_text.str;
    uint32_t code = 0;
    int i, x1, y1;
    uint8_t *p;
    Glyph *glyph = NULL;
    int dst_linesize[NI_MAX_NUM_DATA_POINTERS] = {0};

    dst_linesize[0] = frame->data_len[0] / height;
    dst_linesize[1] = dst_linesize[2] = frame->data_len[1] / (height / 2);

    for (i = 0, p = text; *p; i++) {
        FT_Bitmap bitmap;
        Glyph dummy = { 0 };
#if IS_FFMPEG_43_AND_ABOVE
        GET_UTF8(code, *p ? *p++ : 0, code = 0xfffd; goto continue_on_invalid;);
continue_on_invalid:
#else
        GET_UTF8(code, *p++, continue;);
#endif

        /* skip new line chars, just go to new line */
        if (code == '\n' || code == '\r' || code == '\t')
            continue;

        dummy.code = code;
        dummy.fontsize = s->fontsize[index];
        glyph = av_tree_find(s->glyphs, &dummy, glyph_cmp, NULL);

        bitmap = borderw ? glyph->border_bitmap : glyph->bitmap;

        if (glyph->bitmap.pixel_mode != FT_PIXEL_MODE_MONO &&
            glyph->bitmap.pixel_mode != FT_PIXEL_MODE_GRAY)
            return AVERROR(EINVAL);

        x1 = s->positions[i].x + s->x[index] + x - borderw;
        y1 = s->positions[i].y + s->y[index] + y - borderw;

        ff_blend_mask(&s->dc, color,
                      frame->p_data, dst_linesize, width, height,
                      bitmap.buffer, bitmap.pitch,
                      bitmap.width, bitmap.rows,
                      bitmap.pixel_mode == FT_PIXEL_MODE_MONO ? 0 : 3,
                      0, x1, y1);
    }

    return 0;
}


static void update_color_with_alpha(NetIntDrawTextContext *s, FFDrawColor *color, const FFDrawColor incolor)
{
    *color = incolor;
    color->rgba[3] = (color->rgba[3] * s->alpha) / 255;
    ff_draw_color(&s->dc, color, color->rgba);
}

static void update_alpha(NetIntDrawTextContext *s)
{
    double alpha = av_expr_eval(s->a_pexpr, s->var_values, &s->prng);

    if (isnan(alpha))
        return;

    if (alpha >= 1.0)
        s->alpha = 255;
    else if (alpha <= 0)
        s->alpha = 0;
    else
        s->alpha = 256 * alpha;
}

static void update_canvas_size(NetIntDrawTextContext *s, int x, int y, int w, int h)
{
    if (s->x_start == 0 && s->x_end == -1 &&
        s->y_start == 0 && s->y_end == -1) {
        s->x_start = x;
        s->y_start = y;
        s->x_end = x + w;
        s->y_end = y + h;
        return;
    }
    if (x < s->x_start)
        s->x_start = x;
    if (y < s->y_start)
        s->y_start = y;
    if (x + w > s->x_end)
        s->x_end = x + w;
    if (y + h > s->y_end)
        s->y_end = y + h;
}

static void update_watermark_internal(ni_scaler_watermark_params_t *multi_watermark_params, int x, int y, int w, int h)
{
    if (w == 0 || h == 0) {
        return;
    }
    if (multi_watermark_params->ui32Valid) {
        uint32_t x_end = multi_watermark_params->ui32StartX + multi_watermark_params->ui32Width;
        uint32_t y_end = multi_watermark_params->ui32StartY + multi_watermark_params->ui32Height;
        multi_watermark_params->ui32StartX = FFMIN(multi_watermark_params->ui32StartX, x);
        multi_watermark_params->ui32StartY = FFMIN(multi_watermark_params->ui32StartY, y);
        x_end = FFMAX(x_end, x + w);
        y_end = FFMAX(y_end, y + h);
        multi_watermark_params->ui32Width = x_end - multi_watermark_params->ui32StartX;
        multi_watermark_params->ui32Height = y_end - multi_watermark_params->ui32StartY;
    } else {
        multi_watermark_params->ui32Valid = 1;
        multi_watermark_params->ui32StartX = x;
        multi_watermark_params->ui32StartY = y;
        multi_watermark_params->ui32Width = w;
        multi_watermark_params->ui32Height = h;
    }
}

static void update_signal_watermark(int x0, int y0, int w0, int h0,
                                   int x1, int y1, int w1, int h1,
                                   NetIntDrawTextContext *s, int index)
{
    int inter_x_start = FFMAX(x0, x1);
    int inter_y_start = FFMAX(y0, y1);
    int inter_x_end = FFMIN(x0 + w0, x1 + w1);
    int inter_y_end = FFMIN(y0 + h0, y1 + h1);
    if (inter_x_start >= inter_x_end || inter_y_start >= inter_y_end) {
        return;
    } else {
        av_log(s, AV_LOG_DEBUG, "index %d, x0 %d y0 %d w0 %d h0 %d\n", index,
           x0, y0, w0, h0);
        av_log(s, AV_LOG_DEBUG, "index %d, xstart %d ystart %d xend %d yend %d\n", index,
           inter_x_start, inter_y_start, inter_x_end, inter_y_end);
        update_watermark_internal(&(s->scaler_watermark_paras.multi_watermark_params[index]),
                                  inter_x_start, inter_y_start,
                                  inter_x_end - inter_x_start, inter_y_end - inter_y_start);
    }
}

static void update_watermark(NetIntDrawTextContext *s, int x, int y, int w, int h)
{
    int frame_width = s->watermark_width0 + s->watermark_width1;
    int frame_height = (s->watermark_height0 * 2) + s->watermark_height1;
    if (x < 0) {
        w = FFMAX(w + x, 0);
        x = 0;
    }
    if (y < 0) {
        h = FFMAX(h + y, 0);
        y = 0;
    }
    if (x + w > frame_width) {
        x = FFMIN(x, frame_width);
        w = frame_width - x;
    }
    if (y + h > frame_height) {
        y = FFMIN(y, frame_height);
        h = frame_height - y;
    }

    for (int watermark_idx = 0; watermark_idx < NI_MAX_SUPPORT_WATERMARK_NUM; watermark_idx++) {
        update_signal_watermark(x, y, w, h,
                                s->watermark_width0 * (watermark_idx % 2),
                                s->watermark_height0 * (watermark_idx / 2),
                                watermark_idx % 2 ? s->watermark_width1 : s->watermark_width0,
                                watermark_idx > 3 ? s->watermark_height1 : s->watermark_height0,
                                s, watermark_idx);
    }
}

static void check_and_expand_canvas_size(NetIntDrawTextContext *s, int min_filter_width, int min_filter_heigth)
{
    int x_distance = s->x_end - s->x_start;
    int y_distance = s->y_end - s->y_start;

    if (x_distance < min_filter_width) {
        if (s->x_start - 0 >= min_filter_width - x_distance) {
            s->x_start -= min_filter_width - x_distance;
        }
        else {
            s->x_end += min_filter_width - x_distance;
        }
    }

    if (y_distance < min_filter_heigth) {
        if (s->y_start - 0 >= min_filter_heigth - y_distance) {
            s->y_start -= min_filter_heigth - y_distance;
        }
        else {
            s->y_end += min_filter_heigth - y_distance;
        }
    }
}

static int draw_text(AVFilterContext *ctx, ni_frame_t *frame,
                     int width, int height, int64_t pts)
{
    NetIntDrawTextContext *s = ctx->priv;
    AVFilterLink *inlink = ctx->inputs[0];

    uint32_t code = 0, prev_code = 0;
    int x = 0, y = 0, i = 0, j = 0, ret;
    int max_text_line_w = 0, len;
    int box_w, box_h;
    char *text;
    uint8_t *p;
    int y_min = 32000, y_max = -32000;
    int x_min = 32000, x_max = -32000;
    FT_Vector delta;
    Glyph *glyph = NULL, *prev_glyph = NULL;
    Glyph dummy = { 0 };

    time_t now = time(0);
    struct tm ltime;
    AVBPrint *bp = &s->expanded_text;

    FFDrawColor fontcolor;
    FFDrawColor shadowcolor;
    FFDrawColor bordercolor;
    FFDrawColor boxcolor;
    unsigned int dst_linesize[NI_MAX_NUM_DATA_POINTERS] = {0};
    dst_linesize[0] = frame->data_len[0] / height;
    dst_linesize[1] = dst_linesize[2] = frame->data_len[1] / height / 2;

    av_bprint_clear(bp);

    if (s->basetime != AV_NOPTS_VALUE)
        now = pts * av_q2d(ctx->inputs[0]->time_base) + s->basetime/1000000;

    s->upload_drawtext_frame = 0;

    for (i = 0; i < s->text_num; i++) {
        switch (s->exp_mode) {
        case EXP_NONE:
            av_bprintf(bp, "%s", s->text[i]);
            break;
        case EXP_NORMAL:
            if ((ret = expand_text(ctx, s->text[i], &s->expanded_text)) < 0)
                return ret;
            break;
        case EXP_STRFTIME:
            localtime_r(&now, &ltime);
            av_bprint_strftime(bp, s->text[i], &ltime);
            break;
        }
        if (s->text_last_updated[i] == NULL) {
            s->upload_drawtext_frame = 1;
        } else {
            if (strcmp(s->text_last_updated[i], bp->str))
                s->upload_drawtext_frame = 1;
        }
        s->text_last_updated[i] = av_realloc(s->text_last_updated[i], bp->len+1);
        strcpy(s->text_last_updated[i], bp->str);

        if (s->tc_opt_string) {
            char tcbuf[AV_TIMECODE_STR_SIZE];
#if IS_FFMPEG_71_AND_ABOVE
            FilterLink *li = ff_filter_link(inlink);
            av_timecode_make_string(&s->tc, tcbuf, li->frame_count_out);
#elif IS_FFMPEG_342_AND_ABOVE
            av_timecode_make_string(&s->tc, tcbuf, inlink->frame_count_out);
#else
            av_timecode_make_string(&s->tc, tcbuf, inlink->frame_count);
#endif
            av_bprint_clear(bp);
            av_bprintf(bp, "%s%s", s->text[i], tcbuf);
        }

        if (!av_bprint_is_complete(bp))
            return AVERROR(ENOMEM);
        text = s->expanded_text.str;
        if ((len = s->expanded_text.len) > s->nb_positions) {
            if (!(s->positions =
                av_realloc(s->positions, len*sizeof(*s->positions))))
                return AVERROR(ENOMEM);
            s->nb_positions = len;
        }

        if (s->fontcolor_expr[i]) {
            /* If expression is set, evaluate and replace the static value */
            av_bprint_clear(&s->expanded_fontcolor);
            if ((ret = expand_text(ctx, s->fontcolor_expr[i], &s->expanded_fontcolor)) < 0)
                return ret;
            if (!av_bprint_is_complete(&s->expanded_fontcolor))
                return AVERROR(ENOMEM);
            av_log(s, AV_LOG_DEBUG, "Evaluated fontcolor is '%s'\n", s->expanded_fontcolor.str);
            ret = av_parse_color(s->fontcolor[i].rgba, s->expanded_fontcolor.str, -1, s);
            if (ret)
                return ret;
            ff_draw_color(&s->dc, &s->fontcolor[i], s->fontcolor[i].rgba);
        }

        x = 0;
        y = 0;
        max_text_line_w = 0;

        if ((ret = update_fontsize(ctx, i)) < 0)
            return ret;

        /* load and cache glyphs */
        for (j = 0, p = text; *p; j++) {
#if IS_FFMPEG_43_AND_ABOVE
            GET_UTF8(code, *p ? *p++ : 0, code = 0xfffd; goto continue_on_invalid;);
continue_on_invalid:
#else
            GET_UTF8(code, *p++, continue;);
#endif

            /* get glyph */
            dummy.code = code;
            dummy.fontsize = s->fontsize[i];
            glyph = av_tree_find(s->glyphs, &dummy, glyph_cmp, NULL);
            if (!glyph) {
                ret = load_glyph(ctx, &glyph, code, i);
                if (ret < 0)
                    return ret;
            }

            y_min = FFMIN(glyph->bbox.yMin, y_min);
            y_max = FFMAX(glyph->bbox.yMax, y_max);
            x_min = FFMIN(glyph->bbox.xMin, x_min);
            x_max = FFMAX(glyph->bbox.xMax, x_max);
        }
        s->max_glyph_h = y_max - y_min;
        s->max_glyph_w = x_max - x_min;

        /* compute and save position for each glyph */
        glyph = NULL;
        for (j = 0, p = text; *p; j++) {
#if IS_FFMPEG_43_AND_ABOVE
            GET_UTF8(code, *p ? *p++ : 0, code = 0xfffd; goto continue_on_invalid2;);
continue_on_invalid2:
#else
            GET_UTF8(code, *p++, continue;);
#endif

            /* skip the \n in the sequence \r\n */
            if (prev_code == '\r' && code == '\n')
                continue;

            prev_code = code;
            if (is_newline(code)) {

                max_text_line_w = FFMAX(max_text_line_w, x);
                y += s->max_glyph_h + s->line_spacing;
                x = 0;
                continue;
            }

            /* get glyph */
            prev_glyph = glyph;
            dummy.code = code;
            dummy.fontsize = s->fontsize[i];
            glyph = av_tree_find(s->glyphs, &dummy, glyph_cmp, NULL);

            /* kerning */
            if (s->use_kerning[i] && prev_glyph && glyph->code) {
                FT_Get_Kerning(s->face[i], prev_glyph->code, glyph->code,
                               ft_kerning_default, &delta);
                x += delta.x >> 6;
            }

            /* save position */
            s->positions[j].x = x + glyph->bitmap_left;
            s->positions[j].y = y - glyph->bitmap_top + y_max;
            if (code == '\t')
                x  = (x / s->tabsize[i] + 1)*s->tabsize[i];
            else
                x += glyph->advance;
        }

        max_text_line_w = FFMAX(x, max_text_line_w);

        s->var_values[VAR_TW] = s->var_values[VAR_TEXT_W] = max_text_line_w;
        s->var_values[VAR_TH] = s->var_values[VAR_TEXT_H] = y + s->max_glyph_h;

        s->var_values[VAR_MAX_GLYPH_W] = s->max_glyph_w;
        s->var_values[VAR_MAX_GLYPH_H] = s->max_glyph_h;
        s->var_values[VAR_MAX_GLYPH_A] = s->var_values[VAR_ASCENT ] = y_max;
        s->var_values[VAR_MAX_GLYPH_D] = s->var_values[VAR_DESCENT] = y_min;

        s->var_values[VAR_LINE_H] = s->var_values[VAR_LH] = s->max_glyph_h;

        s->x[i] = s->var_values[VAR_X] = av_expr_eval(s->x_pexpr[i], s->var_values, &s->prng);
        s->y[i] = s->var_values[VAR_Y] = av_expr_eval(s->y_pexpr[i], s->var_values, &s->prng);
        /* It is necessary if x is expressed from y  */
        s->x[i] = s->var_values[VAR_X] = av_expr_eval(s->x_pexpr[i], s->var_values, &s->prng);

        update_alpha(s);
        update_color_with_alpha(s, &fontcolor  , s->fontcolor[i]);
        update_color_with_alpha(s, &shadowcolor, s->shadowcolor);
        update_color_with_alpha(s, &bordercolor, s->bordercolor);
        update_color_with_alpha(s, &boxcolor   , s->boxcolor[i]);

        box_w = max_text_line_w;
        box_h = y + s->max_glyph_h;

        if (s->draw_box && s->boxborderw[i]) {
            int bbsize[4];
            int count;
            count = string_to_array(s->boxborderw[i], bbsize, 4);
            if (count == 1) {
                s->bb_top[i] = s->bb_right[i] = s->bb_bottom[i] = s->bb_left[i] = bbsize[0];
            } else if (count == 2) {
                s->bb_top[i] = s->bb_bottom[i] = bbsize[0];
                s->bb_right[i] = s->bb_left[i] = bbsize[1];
            } else if (count == 3) {
                s->bb_top[i] = bbsize[0];
                s->bb_right[i] = s->bb_left[i] = bbsize[1];
                s->bb_bottom[i] = bbsize[2];
            } else if (count == 4) {
                s->bb_top[i] = bbsize[0];
                s->bb_right[i] = bbsize[1];
                s->bb_bottom[i] = bbsize[2];
                s->bb_left[i] = bbsize[3];
            }
        } else {
            s->bb_top[i] = s->bb_right[i] = s->bb_bottom[i] = s->bb_left[i] = 0;
        }

        if (s->fix_bounds) {

            /* calculate footprint of text effects */
            int borderoffset  = s->borderw  ? FFMAX(s->borderw, 0) : 0;

            int offsetleft = FFMAX3(FFMAX(s->bb_left[i], 0), borderoffset,
                                    (s->shadowx < 0 ? FFABS(s->shadowx) : 0));
            int offsettop = FFMAX3(FFMAX(s->bb_top[i], 0), borderoffset,
                                    (s->shadowy < 0 ? FFABS(s->shadowy) : 0));
            int offsetright = FFMAX3(FFMAX(s->bb_right[i], 0), borderoffset,
                                    (s->shadowx > 0 ? s->shadowx : 0));
            int offsetbottom = FFMAX3(FFMAX(s->bb_bottom[i], 0), borderoffset,
                                    (s->shadowy > 0 ? s->shadowy : 0));


            if (s->x[i] - offsetleft < 0) s->x[i] = offsetleft;
            if (s->y[i] - offsettop < 0)  s->y[i] = offsettop;

            if (s->x[i] + box_w + offsetright > width)
                s->x[i] = FFMAX(width - box_w - offsetright, 0);
            if (s->y[i] + box_h + offsetbottom > height)
                s->y[i] = FFMAX(height - box_h - offsetbottom, 0);
        }
        if (s->x[i] != s->x_bak[i] || s->y[i] != s->y_bak[i]) {
            s->x_bak[i] = s->x[i];
            s->y_bak[i] = s->y[i];
            s->upload_drawtext_frame = 1;
        }
        /* draw box */
        if (s->draw_box)
            ff_blend_rectangle(&s->dc, &boxcolor,
                               frame->p_data, dst_linesize, width, height,
                               s->x[i] - s->bb_left[i], s->y[i] - s->bb_top[i],
                               box_w + s->bb_left[i] + s->bb_right[i], box_h + s->bb_top[i] + s->bb_bottom[i]);

        if (s->shadowx || s->shadowy) {
            if ((ret = draw_glyphs(s, frame, width, height,
                                   &shadowcolor, s->shadowx, s->shadowy, 0, i)) < 0)
                return ret;
        }

        if (s->borderw) {
            if ((ret = draw_glyphs(s, frame, width, height,
                                &bordercolor, 0, 0, s->borderw, i)) < 0)
                return ret;
        }
        if ((ret = draw_glyphs(s, frame, width, height,
                            &fontcolor, 0, 0, 0, i)) < 0)
            return ret;

        update_canvas_size(s, s->x[i] - s->bb_left[i], s->y[i] - s->bb_top[i],
                           box_w + s->bb_left[i] + s->bb_right[i],  box_h + s->bb_top[i] + s->bb_bottom[i]);
        update_watermark(s, s->x[i] - s->bb_left[i], s->y[i] - s->bb_top[i],
                         box_w + s->bb_left[i] + s->bb_right[i], box_h + s->bb_top[i] + s->bb_bottom[i]);
    }
    return 0;
}

static int init_hwframe_uploader(AVFilterContext *ctx, NetIntDrawTextContext *s,
                                 AVFrame *frame, int txt_w, int txt_h)
{
    int ret;
    AVHWFramesContext *hwframe_ctx;
    AVHWFramesContext *out_frames_ctx;
    AVHWFramesContext *main_frame_ctx;
    AVNIDeviceContext *pAVNIDevCtx;
    AVNIFramesContext *f_hwctx, *f_hwctx_output;
    int cardno   = ni_get_cardno(frame);
    char buf[64] = {0};

    main_frame_ctx = (AVHWFramesContext *)frame->hw_frames_ctx->data;

    out_frames_ctx = (AVHWFramesContext *)s->out_frames_ref->data;

    av_log(ctx, AV_LOG_INFO, "%s out_frames_ctx->sw_format %d %s txt %dx%d\n",
           __func__, out_frames_ctx->sw_format,
           av_get_pix_fmt_name(out_frames_ctx->sw_format), txt_w, txt_h);

    snprintf(buf, sizeof(buf), "%d", cardno);

    ret = av_hwdevice_ctx_create(&s->hwdevice, AV_HWDEVICE_TYPE_NI_QUADRA, buf,
                                 NULL, 0);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to create AV HW device ctx\n");
        return ret;
    }

    s->hwframe = av_hwframe_ctx_alloc(s->hwdevice);
    if (!s->hwframe)
        return AVERROR(ENOMEM);

    hwframe_ctx            = (AVHWFramesContext *)s->hwframe->data;
    hwframe_ctx->format    = AV_PIX_FMT_NI_QUAD;
    hwframe_ctx->sw_format = AV_PIX_FMT_RGBA;
    hwframe_ctx->width     = txt_w;
    hwframe_ctx->height    = txt_h;

    ret = av_hwframe_ctx_init(s->hwframe);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to init AV HW device ctx\n");
        return ret;
    }

    // Work around a hwdownload session start timestamp issue
    f_hwctx        = (AVNIFramesContext*) hwframe_ctx->hwctx;
    f_hwctx_output = (AVNIFramesContext*) out_frames_ctx->hwctx;
    f_hwctx_output->api_ctx.session_timestamp =
        f_hwctx->api_ctx.session_timestamp;

    s->hw_frames_ctx = av_buffer_ref(s->hwframe);
    if (!s->hw_frames_ctx)
        return AVERROR(ENOMEM);

    // set up a scaler session for the in-place overlay
    ret = ni_device_session_context_init(&s->api_ctx);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR,
               "ni overlay filter session context init failure\n");
        return ret;
    }

    pAVNIDevCtx = (AVNIDeviceContext *)main_frame_ctx->device_ctx->hwctx;
    s->api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
    s->api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

    s->api_ctx.hw_id              = cardno;
    s->api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
    s->api_ctx.scaler_operation   = s->use_watermark ?
                                    NI_SCALER_OPCODE_WATERMARK : NI_SCALER_OPCODE_IPOVLY;
    s->api_ctx.keep_alive_timeout = s->keep_alive_timeout;

    av_log(ctx, AV_LOG_DEBUG, "%s open overlay session\n", __func__);
    ret = ni_device_session_open(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
    if (ret != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't open scaler session on card %d\n",
               cardno);

        ni_device_session_close(&s->api_ctx, 1, NI_DEVICE_TYPE_SCALER);
        ni_device_session_context_clear(&s->api_ctx);
        return ret;
    }

    s->session_opened = 1;

    ni_cpy_hwframe_ctx(main_frame_ctx, out_frames_ctx);
    ni_device_session_copy(&s->api_ctx, &f_hwctx_output->api_ctx);
#if IS_FFMPEG_61_AND_ABOVE
    s->buffer_limit = 1;
#endif
    if (s->use_watermark) {
        // init the out pool for the overlay session when use watermark
        ret = ff_ni_build_frame_pool(&s->api_ctx, frame->width, frame->height,
                                        main_frame_ctx->sw_format, 4,
                                        s->buffer_limit);

        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                    "Internal output allocation failed rc = %d\n", ret);
            return ret;
        }
    }

    // if background frame has no alpha, set up an extra intermediate scaler
    // session for the crop operation
    if (!s->main_has_alpha && !s->use_watermark) {
        ret = ni_device_session_context_init(&s->crop_api_ctx);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "ni drawtext filter (crop) session context init failure\n");
            return ret;
        }

        s->crop_api_ctx.device_handle = pAVNIDevCtx->cards[cardno];
        s->crop_api_ctx.blk_io_handle = pAVNIDevCtx->cards[cardno];

        s->crop_api_ctx.hw_id              = cardno;
        s->crop_api_ctx.device_type        = NI_DEVICE_TYPE_SCALER;
        s->crop_api_ctx.scaler_operation   = NI_SCALER_OPCODE_CROP;
        s->crop_api_ctx.keep_alive_timeout = s->keep_alive_timeout;

        av_log(ctx, AV_LOG_DEBUG, "%s open crop session\n", __func__);
        ret = ni_device_session_open(&s->crop_api_ctx, NI_DEVICE_TYPE_SCALER);
        if (ret != NI_RETCODE_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR,
                   "Can't open crop session on card %d\n", cardno);

            ni_device_session_close(&s->crop_api_ctx, 1, NI_DEVICE_TYPE_SCALER);
            ni_device_session_context_clear(&s->crop_api_ctx);
            return ret;
        }

        s->crop_session_opened = 1;

        // init the out pool for the crop session, make it rgba
        ret = ff_ni_build_frame_pool(&s->crop_api_ctx, txt_w, txt_h,
                                     AV_PIX_FMT_RGBA, 1, 0);

        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR,
                   "Internal output allocation failed rc = %d\n", ret);
            return ret;
        }
    }

    return 0;
}

static int ni_drawtext_config_input(AVFilterContext *ctx, AVFrame *frame,
                                    int txt_w, int txt_h)
{
    NetIntDrawTextContext *s = ctx->priv;
    int ret;

#if IS_FFMPEG_342_AND_ABOVE
    if (s->initialized)
#else
    if (s->ni_config_initialized)
#endif
        return 0;

    ret = init_hwframe_uploader(ctx, s, frame, txt_w, txt_h);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "failed to initialize uploader session\n");
        return ret;
    }

#if IS_FFMPEG_342_AND_ABOVE
    s->initialized = 1;
#else
    s->ni_config_initialized = 1;
#endif
    return 0;
}

static int overlay_intersects_background(
    const AVFilterContext *ctx,
    int overlay_width,
    int overlay_height,
    const AVFrame *main)
{
    const NetIntDrawTextContext *s = (NetIntDrawTextContext *)ctx->priv;

    if (s->x_start >= main->width)
        return 0;

    if (s->y_start >= main->height)
        return 0;

    if (s->x_start + overlay_width <= 0)
        return 0;

    if (s->y_start + overlay_height <= 0)
        return 0;

    return 1;
}

static void calculate_dst_rectangle(
    int *px,
    int *py,
    int *pw,
    int *ph,
    int bgnd_x,
    int bgnd_y,
    int bgnd_w,
    int bgnd_h,
    int ovly_x,
    int ovly_y,
    int ovly_w,
    int ovly_h)
{
    *px = FFMAX(0, ovly_x);
    *py = FFMAX(0, ovly_y);

    if (ovly_x > 0) {
        *pw = FFMIN(bgnd_w - ovly_x, ovly_w);
    } else {
        *pw = FFMIN(ovly_w + ovly_x, bgnd_w);
    }

    if (ovly_y > 0) {
        *ph = FFMIN(bgnd_h - ovly_y, ovly_h);
    } else {
        *ph = FFMIN(ovly_h + ovly_y, bgnd_h);
    }
}

static void init_watermark(NetIntDrawTextContext *s, int width, int height)
{
    s->watermark_width0 = width / 2;
    s->watermark_width1 = width - s->watermark_width0;
    s->watermark_height0 = height / 3;
    s->watermark_height1 = height - (2 * s->watermark_height0);
    for (int watermark_idx = 0; watermark_idx < NI_MAX_SUPPORT_WATERMARK_NUM; watermark_idx++) {
        s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32StartX = 0;
        s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32StartY = 0;
        s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32Width = 0;
        s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32Height = 0;
        s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32Valid = 0;
    }
}

static void calculate_src_rectangle(
    int *px,
    int *py,
    int *pw,
    int *ph,
    int bgnd_x,
    int bgnd_y,
    int bgnd_w,
    int bgnd_h,
    int ovly_x,
    int ovly_y,
    int ovly_w,
    int ovly_h)

{
    *px = (ovly_x > 0) ? 0 : -ovly_x;
    *py = (ovly_y > 0) ? 0 : -ovly_y;

    if (ovly_x > 0) {
        *pw = FFMIN(bgnd_w - ovly_x, ovly_w);
    } else {
        *pw = FFMIN(ovly_w + ovly_x, bgnd_w);
    }

    if (ovly_y > 0) {
        *ph = FFMIN(bgnd_h - ovly_y, ovly_h);
    } else {
        *ph = FFMIN(ovly_h + ovly_y, bgnd_h);
    }
}

static int do_intermediate_crop_and_overlay(
    AVFilterContext *ctx,
    AVFrame *overlay,
    AVFrame *frame) {
    NetIntDrawTextContext *s = (NetIntDrawTextContext *)ctx->priv;
    AVHWFramesContext    *main_frame_ctx;
    niFrameSurface1_t    *frame_surface;
    ni_retcode_t          retcode;
    uint16_t              ui16FrameIdx;
    int                   main_scaler_format, ovly_scaler_format;
    int                   flags;
    int                   crop_x,crop_y,crop_w,crop_h;
    int                   src_x,src_y,src_w,src_h;

    main_frame_ctx = (AVHWFramesContext *) frame->hw_frames_ctx->data;
    main_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(main_frame_ctx->sw_format);

    ovly_scaler_format = ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA);

    // Allocate a ni_frame_t for the intermediate crop operation
    retcode = ni_frame_buffer_alloc_hwenc(&s->crop_api_dst_frame.data.frame,
                                          overlay->width,
                                          overlay->height,
                                          0);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't allocate interim crop frame\n");
        return AVERROR(ENOMEM);
    }

    calculate_dst_rectangle(&crop_x, &crop_y, &crop_w, &crop_h,
                            0, 0, frame->width, frame->height,
                            FFALIGN(s->x_start,2), FFALIGN(s->y_start,2),
                            overlay->width, overlay->height);

    frame_surface = (niFrameSurface1_t *)frame->data[3];

    // Assign a device input frame. Send incoming frame index to crop session
    retcode = ni_device_alloc_frame(
        &s->crop_api_ctx,
        FFALIGN(frame->width, 2),
        FFALIGN(frame->height, 2),
        main_scaler_format,
        0,
        crop_w,
        crop_h,
        crop_x,
        crop_y,
        0,
        frame_surface->ui16FrameIdx,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't assign input crop frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    // Allocate destination frame. This acquires a frame from the pool
    retcode = ni_device_alloc_frame(
        &s->crop_api_ctx,
        FFALIGN(overlay->width, 2),
        FFALIGN(overlay->height, 2),
        ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA),
        NI_SCALER_FLAG_IO,
        crop_w,
        crop_h,
        0,
        0,
        0,
        -1,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't allocate output crop frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    retcode = ni_device_session_read_hwdesc(&s->crop_api_ctx,
                                            &s->crop_api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "No cropped output frame %d\n", retcode);
        return AVERROR(ENOMEM);
    }

    // Get the acquired frame
    frame_surface = (niFrameSurface1_t *)
        s->crop_api_dst_frame.data.frame.p_data[3];
    s->ui16CropFrameIdx = frame_surface->ui16FrameIdx;

    av_log(ctx, AV_LOG_DEBUG, "%s intrim crop frame idx [%u]\n",
           __func__, s->ui16CropFrameIdx);

    // Overlay the icon over the intermediate cropped frame

    // Allocate a ni_frame_t for the intermediate overlay
    retcode = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          overlay->width,
                                          overlay->height,
                                          0);

    if (retcode < 0) {
        av_log(ctx, AV_LOG_ERROR, "Can't allocate interim ovly frame\n");
        return AVERROR(ENOMEM);
    }

    frame_surface = (niFrameSurface1_t *)overlay->data[3];
    ui16FrameIdx = frame_surface->ui16FrameIdx;

    calculate_src_rectangle(&src_x, &src_y, &src_w, &src_h,
                            0, 0, frame->width, frame->height,
                            FFALIGN(s->x_start,2), FFALIGN(s->y_start,2),
                            overlay->width, overlay->height);

    /* Assign input frame to intermediate overlay session */
    retcode = ni_device_alloc_frame(
        &s->api_ctx,
        FFALIGN(overlay->width, 2),
        FFALIGN(overlay->height, 2),
        ovly_scaler_format,
        0,
        src_w,
        src_h,
        src_x,
        src_y,
        0,
        ui16FrameIdx,
        NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't assign input overlay frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    // In-place overlay frame. Send down frame index of background frame
    /* Configure output, Premultiply alpha */
    flags = NI_SCALER_FLAG_IO | NI_SCALER_FLAG_PA;

    retcode = ni_device_alloc_frame(
       &s->api_ctx,
       FFALIGN(overlay->width, 2),
       FFALIGN(overlay->height, 2),
       ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA),
       flags,
       crop_w,
       crop_h,
       0,
       0,
       0,
       s->ui16CropFrameIdx,
       NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_DEBUG, "Can't overlay frame for output %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    retcode = ni_device_session_read_hwdesc(&s->api_ctx,
                                            &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

    if (retcode != NI_RETCODE_SUCCESS) {
        av_log(ctx, AV_LOG_ERROR, "Can't acquire intermediate frame %d\n",
               retcode);
        return AVERROR(ENOMEM);
    }

    return NI_RETCODE_SUCCESS;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    NetIntDrawTextContext *s = ctx->priv;
    niFrameSurface1_t *logging_surface, *logging_surface_out;
    uint8_t *p_dst, *p_src;
    int y, txt_img_width, txt_img_height;
    int ret;

    AVHWFramesContext *main_frame_ctx, *ovly_frame_ctx;
    niFrameSurface1_t *frame_surface, *new_frame_surface;
    AVFrame *overlay = NULL;
    AVFrame *out = NULL;
    uint16_t main_frame_idx = 0;
    uint16_t ovly_frame_idx = 0;
    int main_scaler_format, ovly_scaler_format;
    int flags;
    int src_x, src_y, src_w, src_h;
    int dst_x, dst_y, dst_w, dst_h;
    int start_row, stop_row, start_col, stop_col;
    int dl_frame_linesize0, text_frame_linesize0;
    int ovly_width, ovly_height;

    av_log(ctx, AV_LOG_DEBUG, "ni_drawtext %s %dx%d is_hw_frame %d\n",
           __func__, frame->width, frame->height,
           AV_PIX_FMT_NI_QUAD == frame->format);

    if (s->reload) {
        if ((ret = load_textfile(ctx)) < 0) {
            av_frame_free(&frame);
            return ret;
        }
#if CONFIG_LIBFRIBIDI
        if (s->text_shaping)
            if ((ret = shape_text(ctx)) < 0) {
                av_frame_free(&frame);
                return ret;
            }
#endif
    }

#if !IS_FFMPEG_342_AND_ABOVE
    if (!s->input_config_initialized) {
        ret = config_input(inlink, frame);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed ni_drawtext config input\n");
            goto fail;
        }
    }
#endif

#if IS_FFMPEG_71_AND_ABOVE
    FilterLink *li = ff_filter_link(inlink);
    s->var_values[VAR_N] = li->frame_count_out + s->start_number;
#elif IS_FFMPEG_342_AND_ABOVE
    s->var_values[VAR_N] = inlink->frame_count_out + s->start_number;
#else
    s->var_values[VAR_N] = inlink->frame_count + s->start_number;
#endif
    s->var_values[VAR_T] = frame->pts == AV_NOPTS_VALUE ?
        NAN : frame->pts * av_q2d(inlink->time_base);

    s->var_values[VAR_PICT_TYPE] = frame->pict_type;
#if FF_API_FRAME_PKT
FF_DISABLE_DEPRECATION_WARNINGS // FFmpeg-n6.1+ deprecates AVFrame->pkt_pos and AVFrame->pkt_size
    s->var_values[VAR_PKT_POS] = frame->pkt_pos;
    s->var_values[VAR_PKT_SIZE] = frame->pkt_size;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
#if LIBAVUTIL_VERSION_MAJOR >= 59 //7.0
    s->var_values[VAR_PKT_DURATION] = frame->duration * av_q2d(inlink->time_base);
#else
    s->var_values[VAR_PKT_DURATION] = frame->pkt_duration * av_q2d(inlink->time_base);
#endif
    s->metadata = frame->metadata;
    s->x_start = 0;
    s->x_end = -1;
    s->y_start = 0;
    s->y_end = -1;
    init_watermark(s, frame->width, frame->height);

    main_frame_ctx = (AVHWFramesContext *)frame->hw_frames_ctx->data;
    av_log(ctx, AV_LOG_DEBUG, "%s HW frame, sw_format %d %s, before drawtext "
           "var_text_WxH %dx%d\n",
           __func__, main_frame_ctx->sw_format,
           av_get_pix_fmt_name(main_frame_ctx->sw_format),
           (int)s->var_values[VAR_TEXT_W], (int)s->var_values[VAR_TEXT_H]);

    memset(s->dl_frame.data.frame.p_buffer, 0,
           s->dl_frame.data.frame.buffer_size);

    draw_text(ctx, &(s->dl_frame.data.frame), frame->width, frame->height,
              frame->pts);
    check_and_expand_canvas_size(s, NI_MIN_RESOLUTION_WIDTH_SCALER, NI_MIN_RESOLUTION_HEIGHT_SCALER);

    av_log(ctx, AV_LOG_DEBUG, "n:%d t:%f text_w:%d text_h:%d x:%d y:%d "
           "shadowx:%d shadowy:%d\n",
           (int)s->var_values[VAR_N], s->var_values[VAR_T],
           (int)s->var_values[VAR_TEXT_W], (int)s->var_values[VAR_TEXT_H],
           s->x_start, s->y_start, s->shadowx, s->shadowy);

    txt_img_width = FFALIGN(s->x_end - s->x_start, 2);
    txt_img_height = FFALIGN(s->y_end - s->y_start, 2);

    if (s->use_watermark) {
        ovly_width = frame->width;
        ovly_height = frame->height;
    } else {
        ovly_width = txt_img_width;
        ovly_height = txt_img_height;
    }
    // If overlay does not intersect the background, pass
    // the frame through the drawtext filter.
    if (!overlay_intersects_background(ctx, txt_img_width, txt_img_height,
                                       frame)) {
        return ff_filter_frame(outlink, frame);
    }

    if (s->use_watermark) {
#if IS_FFMPEG_71_AND_ABOVE
        FilterLink *li = ff_filter_link(inlink);
        int frame_count = li->frame_count_out;
#elif IS_FFMPEG_342_AND_ABOVE
        int frame_count = inlink->frame_count_out;
#else
        int frame_count = inlink->frame_count;
#endif
        for (int watermark_idx = 0; watermark_idx < NI_MAX_SUPPORT_WATERMARK_NUM; watermark_idx++) {
            if (s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32Valid) {
                av_log(ctx, AV_LOG_DEBUG, "frame %d index %d, x %d, y %d, w %d, h %d\n",
                    frame_count, watermark_idx,
                    s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32StartX,
                    s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32StartY,
                    s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32Width,
                    s->scaler_watermark_paras.multi_watermark_params[watermark_idx].ui32Height);
            }
        }
    }
#if IS_FFMPEG_342_AND_ABOVE
    if (!s->initialized) {
#else
    if (!s->ni_config_initialized) {
#endif
        if (s->initiated_upload_width == 0) {
            s->initiated_upload_width = frame->width > ovly_width ? ovly_width + 4 : frame->width;
            s->initiated_upload_height = frame->height > ovly_height ? ovly_height + 4 : frame->height;
        }
        ret = ni_drawtext_config_input(ctx, frame, s->initiated_upload_width, s->initiated_upload_height);
        if (ret) {
            av_log(ctx, AV_LOG_ERROR, "failed ni_drawtext config input\n");
            goto fail;
        }
    }
    txt_img_width = ovly_width = s->initiated_upload_width;
    txt_img_height = ovly_height = s->initiated_upload_height;

    // Clear the contents of up_frame to avoid accumulating old data
    av_frame_free(&s->up_frame);
    s->up_frame = av_frame_alloc();
    if (!s->up_frame) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    if (s->use_watermark) {
        if (ni_scaler_set_watermark_params(&s->api_ctx,
                                           &s->scaler_watermark_paras.multi_watermark_params[0])) {
            av_log(ctx, AV_LOG_ERROR, "failed ni_drawtext set_watermark_params\n");
            goto fail;
        }
        // wrap the dl_frame ni_frame into AVFrame up_frame
        // for RGBA format, only need to copy the first data
        // in some situation, like linesize[0] == align64(width*4)
        // it will use zero copy, and it need to keep data[1] and data[2] be null
        // for watermark, it uploads the whole frame
        s->up_frame->data[0] = s->dl_frame.data.frame.p_data[0];
        s->up_frame->linesize[0] = FFALIGN(ovly_width, 16) * 4;
    } else {
        av_log(ctx, AV_LOG_DEBUG, "%s alloc txt_frame %dx%d\n", __func__,
            txt_img_width, txt_img_height);
        if (ni_frame_buffer_alloc_dl(&(s->txt_frame.data.frame),
                                    txt_img_width, txt_img_height,
                                    NI_PIX_FMT_RGBA)) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        p_dst = s->txt_frame.data.frame.p_buffer;
        memset(p_dst, 0, s->txt_frame.data.frame.buffer_size);

        start_row = s->y_start;
        stop_row  = start_row + txt_img_height;
        dl_frame_linesize0 = FFALIGN(frame->width, 16);
        text_frame_linesize0 = FFALIGN(txt_img_width, 16);
        // if overlay intersects at the main top/bottom, only copy the overlaying
        // portion
        if (start_row < 0) {
            p_dst += -1 * start_row * text_frame_linesize0 * 4;
            start_row = 0;
        }
        if (stop_row > frame->height) {
            stop_row = frame->height;
        }

        // if overlay intersects at the main left/right, only copy the overlaying
        // portion
        start_col = s->x_start;
        stop_col  = start_col + txt_img_width;
        if (start_col < 0) {
            p_dst += (-4 * start_col);
            start_col = 0;
        }
        if (stop_col > frame->width) {
            stop_col = frame->width;
        }

        for (y = start_row; y < stop_row; y++) {
            p_src = s->dl_frame.data.frame.p_buffer +
                (y * dl_frame_linesize0 + start_col) * 4;

            memcpy(p_dst, p_src, (stop_col - start_col) * 4);
            p_dst += text_frame_linesize0 * 4;
        }
        // wrap the txt ni_frame into AVFrame up_frame
        // for RGBA format, only need to copy the first data
        // in some situation, like linesize[0] == align64(width*4)
        // it will use zero copy, and it need to keep data[1] and data[2] be null
        // for inplace overlay, it updates the clip include text
        s->up_frame->data[0] = s->txt_frame.data.frame.p_data[0];
        s->up_frame->linesize[0] = text_frame_linesize0 * 4;
    }

    if (s->optimize_upload == 0) //Force uploading drawtext frame by every frame
        s->upload_drawtext_frame = 1;

    s->filtered_frame_count++;
    if (s->filtered_frame_count  == s->framerate || s->keep_overlay == NULL) {
        s->upload_drawtext_frame = 1;
        s->filtered_frame_count = 0;
    }

    if (s->upload_drawtext_frame) {
        av_frame_free(&s->keep_overlay);
        s->keep_overlay = NULL;
        s->keep_overlay = overlay = av_frame_alloc();
        if (!overlay) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        av_frame_copy_props(overlay, frame);
        overlay->width         = ovly_width;
        overlay->height        = ovly_height;
        overlay->format        = AV_PIX_FMT_NI_QUAD;
        overlay->color_range   = AVCOL_RANGE_MPEG;
        overlay->hw_frames_ctx = s->out_frames_ref;

        ret = av_hwframe_get_buffer(s->hw_frames_ctx, overlay, 0);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "failed to get buffer\n");
            av_frame_free(&overlay);
            return ret;
        }

        av_frame_copy_props(s->up_frame, frame);
        s->up_frame->format = AV_PIX_FMT_RGBA;
        s->up_frame->width  = ovly_width;
        s->up_frame->height = ovly_height;
        ret = av_hwframe_transfer_data(overlay, // dst src flags
                                       s->up_frame, 0);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "upload failed, ret = %d\n", ret);
            return ret;
        }
    }
    else {
        overlay = s->keep_overlay;
    }
    // logging
    logging_surface = (niFrameSurface1_t*)frame->data[3];
    logging_surface_out = (niFrameSurface1_t*)overlay->data[3];
    av_log(ctx, AV_LOG_DEBUG,
           "vf_drawtext_ni:IN ui16FrameIdx = [%d] uploaded overlay = [%d]\n",
           logging_surface->ui16FrameIdx, logging_surface_out->ui16FrameIdx);

    // do the in place overlay
    main_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(main_frame_ctx->sw_format);

    frame_surface = (niFrameSurface1_t *) frame->data[3];
    if (frame_surface == NULL) {
        av_frame_free(&overlay);
        return AVERROR(EINVAL);
    }

    main_frame_idx = frame_surface->ui16FrameIdx;

    frame_surface = (niFrameSurface1_t *) overlay->data[3];
    if (frame_surface == NULL) {
        av_frame_free(&overlay);
        return AVERROR(EINVAL);
    }

    ovly_frame_idx = frame_surface->ui16FrameIdx;
    ovly_frame_ctx = (AVHWFramesContext *)overlay->hw_frames_ctx->data;
    ovly_scaler_format =
        ff_ni_ffmpeg_to_gc620_pix_fmt(ovly_frame_ctx->sw_format);

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark(NULL);
#endif

    // for rgba over yuv, do an intermediate crop and overlay
    if (!s->main_has_alpha && !s->use_watermark) {
        ret = do_intermediate_crop_and_overlay(ctx, overlay, frame);
        if (ret < 0) {
            av_frame_free(&overlay);
            return ret;
        }

        // Allocate a ni_frame for the overlay output
        ret = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          outlink->w,
                                          outlink->h,
                                          0);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR, "Can't allocate inplace overlay frame\n");
            return AVERROR(ENOMEM);
        }

        calculate_src_rectangle(&src_x, &src_y, &src_w, &src_h,
                                0, 0, frame->width, frame->height,
                                FFALIGN(s->x_start,2),FFALIGN(s->y_start,2),
                                overlay->width, overlay->height);

        // Assign an input frame for overlay picture. Send the
        // incoming hardware frame index to the scaler manager.
        ret = ni_device_alloc_frame(
            &s->api_ctx,
            overlay->width,  // ovly width
            overlay->height, // ovly height
            ff_ni_ffmpeg_to_gc620_pix_fmt(AV_PIX_FMT_RGBA), // ovly pix fmt
            0,                           // flags
            src_w,                       // src rect width
            src_h,                       // src rect height
            0,                           // src rect x
            0,                           // src rect y
            0,                           // n/a
            s->ui16CropFrameIdx,         // ovly frame idx
            NI_DEVICE_TYPE_SCALER);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR, "Can't assign input overlay frame %d\n",
                   ret);
            return AVERROR(ENOMEM);
        }

        calculate_dst_rectangle(&dst_x, &dst_y, &dst_w, &dst_h,
                                0, 0, frame->width, frame->height,
                                FFALIGN(s->x_start,2), FFALIGN(s->y_start, 2),
                                overlay->width, overlay->height);

        // Allocate device output frame from the pool. We also send down the
        // frame index of the background frame to the scaler manager.

        /* configure the output, premultiply alpha*/
        flags = NI_SCALER_FLAG_IO | NI_SCALER_FLAG_PA;

        ret = ni_device_alloc_frame(
            &s->api_ctx,
            FFALIGN(frame->width, 2),       // main width
            FFALIGN(frame->height, 2),      // main height
            main_scaler_format,             // main pix fmt
            flags,                          // flags
            dst_w,                          // dst rect width
            dst_h,                          // dst rect height
            dst_x,                          // dst rect x
            dst_y,                          // dst rect y
            0,                              // n/a
            main_frame_idx,                 // main frame idx
            NI_DEVICE_TYPE_SCALER);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR, "Can't allocate overlay output %d\n",
                   ret);
            return AVERROR(ENOMEM);
        }

        // Set the new frame index
        ret = ni_device_session_read_hwdesc(&s->api_ctx,
                                            &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR,
                   "Can't acquire output overlay frame %d\n", ret);
            return AVERROR(ENOMEM);
        }
    } else {
        // we can perform an in-place overlay immediately for rgba over rgba,
        // or use watermark, it overlay rgab over yuv/rgba

        av_log(ctx, AV_LOG_DEBUG, "%s overlay %s main %s\n", __func__,
               av_get_pix_fmt_name(ovly_frame_ctx->sw_format),
               av_get_pix_fmt_name(main_frame_ctx->sw_format));

        /* Allocate ni_frame for the overlay output */
        ret = ni_frame_buffer_alloc_hwenc(&s->api_dst_frame.data.frame,
                                          outlink->w,
                                          outlink->h,
                                          0);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR, "Cannot allocate in-place frame\n");
            return AVERROR(ENOMEM);
        }

        if (!s->use_watermark) {
            calculate_src_rectangle(&src_x, &src_y, &src_w, &src_h,
                        0, 0, frame->width, frame->height,
                        FFALIGN(s->x_start,2), FFALIGN(s->y_start,2),
                        overlay->width, overlay->height);
        }

        /*
         * Assign input frame for overlay picture. Sends the
         * incoming hardware frame index to the scaler manager.
         */
        ret = ni_device_alloc_frame(
            &s->api_ctx,
            overlay->width,          // overlay width
            overlay->height,        // overlay height
            ovly_scaler_format,                     // overlay pix fmt
            0,                                      // flags
            s->use_watermark ? ovly_width : src_w,  // src rect width
            s->use_watermark ? ovly_height : src_h, // src rect height
            s->use_watermark ? 0 : src_x,           // src rect x
            s->use_watermark ? 0 : src_y,           // src rect y
            0,                                  // n/a
            ovly_frame_idx,                     // overlay frame idx
            NI_DEVICE_TYPE_SCALER);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR,
                   "Can't assign frame for overlay input %d\n", ret);
            return AVERROR(ENOMEM);
        }

        if (!s->use_watermark) {
            /* Configure the output, Premultiply alpha */
            flags = NI_SCALER_FLAG_IO | NI_SCALER_FLAG_PA;

            calculate_dst_rectangle(&dst_x, &dst_y, &dst_w, &dst_h,
                                    0, 0, frame->width, frame->height,
                                    FFALIGN(s->x_start,2), FFALIGN(s->y_start,2),
                                    overlay->width, overlay->height);
        }
        ret = ni_device_alloc_frame(
            &s->api_ctx,
            FFALIGN(frame->width, 2),       // main width
            FFALIGN(frame->height, 2),      // main height
            main_scaler_format,             // main pix fmt
            s->use_watermark ? NI_SCALER_FLAG_IO : flags,     // flags
            s->use_watermark ? frame->width : dst_w,          // dst rect width
            s->use_watermark ? frame->height : dst_h,         // dst rect height
            s->use_watermark ? 0 : dst_x,                     // dst rect x
            s->use_watermark ? 0 : dst_y,                     // dst rect y
            0,                              // n/a
            main_frame_idx,                 // main frame idx
            NI_DEVICE_TYPE_SCALER);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR,
                   "Can't allocate frame for output ovly %d\n", ret);
            return AVERROR(ENOMEM);
        }

        ret = ni_device_session_read_hwdesc(&s->api_ctx, &s->api_dst_frame,
                                            NI_DEVICE_TYPE_SCALER);

        if (ret != NI_RETCODE_SUCCESS) {
            av_frame_free(&overlay);
            av_log(ctx, AV_LOG_ERROR,
                   "Can't acquire output frame of overlay %d\n", ret);
            return AVERROR(ENOMEM);
        }
    }

#ifdef NI_MEASURE_LATENCY
    ff_ni_update_benchmark("ni_quadra_drawtext");
#endif

    if (s->use_watermark) {
        out = av_frame_alloc();
        if (!out) {
            return AVERROR(ENOMEM);
        }

        av_frame_copy_props(out,frame);

        out->width = outlink->w;
        out->height = outlink->h;
        out->format = AV_PIX_FMT_NI_QUAD;

        /* Quadra 2D engine always outputs limited color range */
        out->color_range = AVCOL_RANGE_MPEG;

        /* Reference the new hw frames context */
        out->hw_frames_ctx = av_buffer_ref(s->out_frames_ref);
        // av_hwframe_get_buffer(s->hw_frames_ctx, out, 0);
        out->data[3] = av_malloc(sizeof(niFrameSurface1_t));

        if (!out->data[3]) {
            av_frame_free(&out);
            return AVERROR(ENOMEM);
        }

        /* Copy the frame surface from the incoming frame */
        memcpy(out->data[3], frame->data[3], sizeof(niFrameSurface1_t));
        frame_surface = (niFrameSurface1_t *) out->data[3];
        new_frame_surface = (niFrameSurface1_t *) s->api_dst_frame.data.frame.p_data[3];
        frame_surface->ui16FrameIdx   = new_frame_surface->ui16FrameIdx;
        frame_surface->ui16session_ID = new_frame_surface->ui16session_ID;
        frame_surface->device_handle  = new_frame_surface->device_handle;
        frame_surface->output_idx     = new_frame_surface->output_idx;
        frame_surface->src_cpu        = new_frame_surface->src_cpu;
        frame_surface->dma_buf_fd     = 0;

        ff_ni_set_bit_depth_and_encoding_type(&frame_surface->bit_depth,
                                            &frame_surface->encoding_type,
                                            main_frame_ctx->sw_format);

        /* Remove ni-split specific assets */
        frame_surface->ui32nodeAddress = 0;

        frame_surface->ui16width = out->width;
        frame_surface->ui16height = out->height;

        out->buf[0] = av_buffer_create(out->data[3], sizeof(niFrameSurface1_t), ff_ni_frame_free, NULL, 0);

        av_log(ctx, AV_LOG_DEBUG,
            "%s:IN trace ui16FrameIdx = [%d] over [%d] --> out [%d]\n",
            __func__, ovly_frame_idx, main_frame_idx, frame_surface->ui16FrameIdx);

        av_frame_free(&frame);
        return ff_filter_frame(outlink, out);
    } else {
        frame->color_range = AVCOL_RANGE_MPEG;

        if (!s->main_has_alpha) {
            av_log(ctx, AV_LOG_DEBUG,
                "%s:IN trace ui16FrameIdx = [%d] and [%d] and [%d] --> out [%d]\n",
                __func__, main_frame_idx, ovly_frame_idx, s->ui16CropFrameIdx,
                main_frame_idx);
        } else {
            av_log(ctx, AV_LOG_DEBUG,
            "%s:IN trace ui16FrameIdx = [%d] and [%d] --> out [%d]\n",
            __func__, main_frame_idx, ovly_frame_idx, main_frame_idx);
        }

        if (!s->main_has_alpha) {
            ni_hwframe_buffer_recycle((niFrameSurface1_t *)
                                    s->crop_api_dst_frame.data.frame.p_data[3],
                                    (int32_t)s->crop_api_ctx.device_handle);
        }

        return ff_filter_frame(outlink, frame);
    }
fail:
    return ret;
}

#if IS_FFMPEG_61_AND_ABOVE
static int activate(AVFilterContext *ctx)
{
    AVFilterLink  *inlink = ctx->inputs[0];
    AVFilterLink  *outlink = ctx->outputs[0];
    AVFrame *frame = NULL;
    int ret = 0;
    NetIntDrawTextContext *s = inlink->dst->priv;

    // Forward the status on output link to input link, if the status is set, discard all queued frames
    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    if (ff_inlink_check_available_frame(inlink)) {
        if (s->initialized) {
            ret = ni_device_session_query_buffer_avail(&s->api_ctx, NI_DEVICE_TYPE_SCALER);
        }

        if (ret == NI_RETCODE_ERROR_UNSUPPORTED_FW_VERSION) {
            av_log(ctx, AV_LOG_WARNING, "No backpressure support in FW\n");
        } else if (ret < 0) {
            av_log(ctx, AV_LOG_WARNING, "%s: query ret %d, inlink framequeue %lu available_frame %d outlink framequeue %lu frame_wanted %d - return NOT READY\n",
                __func__, ret, ff_inlink_queued_frames(inlink), ff_inlink_check_available_frame(inlink), ff_inlink_queued_frames(outlink), ff_outlink_frame_wanted(outlink));
            return FFERROR_NOT_READY;
        }

        ret = ff_inlink_consume_frame(inlink, &frame);
        if (ret < 0)
            return ret;

        ret = filter_frame(inlink, frame);
        if (ret >= 0) {
            ff_filter_set_ready(ctx, 100);
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

static const AVFilterPad inputs[] = {
    {
        .name           = "default",
        .type           = AVMEDIA_TYPE_VIDEO,
        .filter_frame   = filter_frame,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props   = config_input,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

static const AVFilterPad outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
#if IS_FFMPEG_342_AND_ABOVE
        .config_props = config_output,
#endif
    },
#if (LIBAVFILTER_VERSION_MAJOR < 8)
    { NULL }
#endif
};

AVFilter ff_vf_drawtext_ni_quadra = {
    .name            = "ni_quadra_drawtext",
    .description     = NULL_IF_CONFIG_SMALL("NETINT Quadra draw text on top of video frames using libfreetype library v" NI_XCODER_REVISION),
    .priv_size       = sizeof(NetIntDrawTextContext),
    .priv_class      = &ni_drawtext_class,
    .init            = init,
    .uninit          = uninit,
#if IS_FFMPEG_61_AND_ABOVE
    .activate        = activate,
#endif
#if (LIBAVFILTER_VERSION_MAJOR >= 8)
    FILTER_QUERY_FUNC(query_formats),
    FILTER_INPUTS(inputs),
    FILTER_OUTPUTS(outputs),
#else
    .query_formats   = query_formats,
    .inputs          = inputs,
    .outputs         = outputs,
#endif
#if IS_FFMPEG_43_AND_ABOVE
    .process_command = command,
#endif
    .flags           = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    // only FFmpeg 3.4.2 and above have .flags_internal
#if IS_FFMPEG_342_AND_ABOVE
    .flags_internal  = FF_FILTER_FLAG_HWFRAME_AWARE
#endif
};
