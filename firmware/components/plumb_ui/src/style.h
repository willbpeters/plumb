/* Palette, text and drawing helpers shared by every screen.
 * Colours are the reviewed screen studies'. */
#ifndef PL_UI_STYLE_H
#define PL_UI_STYLE_H

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"
#include "../fonts/pl_fonts.h"

#define PL_UI_BG     lv_color_hex(0x0A0B0D)
#define PL_UI_FG     lv_color_hex(0xF2EFE9)
#define PL_UI_DIM    lv_color_hex(0x6E747B)
#define PL_UI_FAINT  lv_color_hex(0x2A2E34)
#define PL_UI_STEEL  lv_color_hex(0x8E959D)
#define PL_UI_ACCENT lv_color_hex(0xE8B44A)

/* UTF-8, as escapes: C sources stay ASCII (MSVC, C4819). Kept as separate
 * string literals so a following hex digit cannot join the escape. */
#define PL_UI_DEG    "\xC2\xB0"
#define PL_UI_MIDDOT "\xC2\xB7"
#define PL_UI_DASH   "\xE2\x80\x94"

typedef enum {
    PL_UI_ALIGN_LEFT,
    PL_UI_ALIGN_CENTER,
    PL_UI_ALIGN_RIGHT
} pl_ui_align;

int32_t pl_ui_px(float v);

/* Draw `text` with its baseline at `baseline`, anchored at x by `align`.
 * Counts any glyph the font lacks (pl_ui_missing_glyphs). */
void pl_ui_text(lv_layer_t *layer, const lv_font_t *font, lv_color_t color,
                lv_opa_t opa, const char *text, int32_t x, int32_t baseline,
                pl_ui_align align, int32_t letter_space);
int32_t pl_ui_text_width(const lv_font_t *font, const char *text,
                         int32_t letter_space);
uint32_t pl_ui_count_missing(const lv_font_t *font, const char *text);

void pl_ui_line(lv_layer_t *layer, lv_color_t color, int32_t width,
                float x1, float y1, float x2, float y2, bool round,
                int32_t dash, int32_t gap);
/* A straight bar from (x1,y1) to (x2,y2), rotated about (cx,cy). */
void pl_ui_rotated_bar(lv_layer_t *layer, lv_color_t color, int32_t width,
                       float x1, float y1, float x2, float y2,
                       float cx, float cy, float deg_cw);
void pl_ui_triangle(lv_layer_t *layer, lv_color_t color, float ax, float ay,
                    float bx, float by, float cx, float cy);
void pl_ui_dot(lv_layer_t *layer, lv_color_t color, float cx, float cy,
               int32_t radius);
void pl_ui_arc(lv_layer_t *layer, lv_color_t color, int32_t radius,
               int32_t width, int32_t start_deg, int32_t end_deg, bool rounded);

/* The faint arrow at the left edge, and optionally the word TARGET: left is
 * the target. The path screen omits the word, because its target line and
 * trail run through where the word sits. */
void pl_ui_target_mark(lv_layer_t *layer, bool with_word);
/* What a metric without a reading shows: a faint dash pair and its title. */
void pl_ui_no_reading(lv_layer_t *layer, const char *title);

#endif /* PL_UI_STYLE_H */
