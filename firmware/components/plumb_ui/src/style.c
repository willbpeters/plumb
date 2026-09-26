/* See style.h. */

#include "style.h"

#include <math.h>

#include "geometry.h"
#include "plumb/ui.h"

static uint32_t s_missing_glyphs;

/* All our strings are well-formed UTF-8, from the macros in style.h. */
static uint32_t next_codepoint(const char **text)
{
    const unsigned char *s = (const unsigned char *)*text;
    uint32_t c;
    if (s[0] < 0x80u) {
        c = s[0];
        *text += 1;
    } else if ((s[0] & 0xE0u) == 0xC0u) {
        c = ((uint32_t)(s[0] & 0x1Fu) << 6) | (uint32_t)(s[1] & 0x3Fu);
        *text += 2;
    } else if ((s[0] & 0xF0u) == 0xE0u) {
        c = ((uint32_t)(s[0] & 0x0Fu) << 12) | ((uint32_t)(s[1] & 0x3Fu) << 6)
            | (uint32_t)(s[2] & 0x3Fu);
        *text += 3;
    } else {
        c = 0xFFFDu;
        *text += 1;
    }
    return c;
}

int32_t pl_ui_px(float v)
{
    return (int32_t)lroundf(v);
}

uint32_t pl_ui_count_missing(const lv_font_t *font, const char *text)
{
    uint32_t missing = 0;
    const char *p = text;
    while (*p != '\0') {
        lv_font_glyph_dsc_t glyph;
        const uint32_t c = next_codepoint(&p);
        if (!lv_font_get_glyph_dsc(font, &glyph, c, 0)) {
            missing++;
        }
    }
    return missing;
}

uint32_t pl_ui_missing_glyphs(void)
{
    return s_missing_glyphs;
}

int32_t pl_ui_text_width(const lv_font_t *font, const char *text,
                         int32_t letter_space)
{
    lv_point_t size;
    lv_text_get_size(&size, text, font, letter_space, 0, LV_COORD_MAX,
                     LV_TEXT_FLAG_NONE);
    return size.x;
}

void pl_ui_text(lv_layer_t *layer, const lv_font_t *font, lv_color_t color,
                lv_opa_t opa, const char *text, int32_t x, int32_t baseline,
                pl_ui_align align, int32_t letter_space)
{
    lv_draw_label_dsc_t dsc;
    lv_area_t area;
    const int32_t line_height = lv_font_get_line_height(font);
    const int32_t width = pl_ui_text_width(font, text, letter_space);
    const int32_t top = baseline - (line_height - font->base_line);

    s_missing_glyphs += pl_ui_count_missing(font, text);

    if (align == PL_UI_ALIGN_CENTER) {
        x -= width / 2;
    } else if (align == PL_UI_ALIGN_RIGHT) {
        x -= width;
    }
    area.x1 = x;
    area.y1 = top;
    area.x2 = x + width;
    area.y2 = top + line_height;

    lv_draw_label_dsc_init(&dsc);
    dsc.text = text;
    /* Draw tasks run after this returns, and callers pass stack buffers. */
    dsc.text_local = 1;
    dsc.font = font;
    dsc.color = color;
    dsc.opa = opa;
    dsc.letter_space = letter_space;
    dsc.align = LV_TEXT_ALIGN_LEFT;
    lv_draw_label(layer, &dsc, &area);
}

void pl_ui_line(lv_layer_t *layer, lv_color_t color, int32_t width,
                float x1, float y1, float x2, float y2, bool round,
                int32_t dash, int32_t gap)
{
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = LV_OPA_COVER;
    dsc.width = width;
    dsc.p1.x = pl_ui_px(x1);
    dsc.p1.y = pl_ui_px(y1);
    dsc.p2.x = pl_ui_px(x2);
    dsc.p2.y = pl_ui_px(y2);
    dsc.round_start = (uint8_t)(round ? 1 : 0);
    dsc.round_end = (uint8_t)(round ? 1 : 0);
    dsc.dash_width = dash;
    dsc.dash_gap = gap;
    lv_draw_line(layer, &dsc);
}

void pl_ui_rotated_bar(lv_layer_t *layer, lv_color_t color, int32_t width,
                       float x1, float y1, float x2, float y2,
                       float cx, float cy, float deg_cw)
{
    float ax, ay, bx, by;
    pl_ui_rotate_cw(cx, cy, deg_cw, x1, y1, &ax, &ay);
    pl_ui_rotate_cw(cx, cy, deg_cw, x2, y2, &bx, &by);
    pl_ui_line(layer, color, width, ax, ay, bx, by, false, 0, 0);
}

void pl_ui_triangle(lv_layer_t *layer, lv_color_t color, float ax, float ay,
                    float bx, float by, float cx, float cy)
{
    lv_draw_triangle_dsc_t dsc;
    lv_draw_triangle_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = LV_OPA_COVER;
    dsc.p[0].x = pl_ui_px(ax);
    dsc.p[0].y = pl_ui_px(ay);
    dsc.p[1].x = pl_ui_px(bx);
    dsc.p[1].y = pl_ui_px(by);
    dsc.p[2].x = pl_ui_px(cx);
    dsc.p[2].y = pl_ui_px(cy);
    lv_draw_triangle(layer, &dsc);
}

void pl_ui_dot(lv_layer_t *layer, lv_color_t color, float cx, float cy,
               int32_t radius)
{
    lv_draw_rect_dsc_t dsc;
    lv_area_t area;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color = color;
    dsc.bg_opa = LV_OPA_COVER;
    dsc.radius = LV_RADIUS_CIRCLE;
    area.x1 = pl_ui_px(cx) - radius;
    area.y1 = pl_ui_px(cy) - radius;
    area.x2 = pl_ui_px(cx) + radius;
    area.y2 = pl_ui_px(cy) + radius;
    lv_draw_rect(layer, &dsc, &area);
}

void pl_ui_arc(lv_layer_t *layer, lv_color_t color, int32_t radius,
               int32_t width, int32_t start_deg, int32_t end_deg, bool rounded)
{
    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = LV_OPA_COVER;
    dsc.width = width;
    dsc.center.x = 120;
    dsc.center.y = 120;
    dsc.radius = (uint16_t)radius;
    dsc.start_angle = start_deg;
    dsc.end_angle = end_deg;
    dsc.rounded = (uint8_t)(rounded ? 1 : 0);
    lv_draw_arc(layer, &dsc);
}

void pl_ui_target_mark(lv_layer_t *layer, bool with_word)
{
    pl_ui_triangle(layer, PL_UI_FAINT, 14.0f, 120.0f, 24.0f, 114.0f, 24.0f, 126.0f);
    if (with_word) {
        pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "TARGET",
                   30, 124, PL_UI_ALIGN_LEFT, 2);
    }
}

void pl_ui_no_reading(lv_layer_t *layer, const char *title)
{
    pl_ui_text(layer, &pl_font_hero, PL_UI_FAINT, LV_OPA_COVER,
               PL_UI_DASH " " PL_UI_DASH, 120, 150, PL_UI_ALIGN_CENTER, 0);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, title,
               120, 178, PL_UI_ALIGN_CENTER, 2);
}
