/* Tempo: the backswing bar grows right, then the through bar left, both at
 * one speed, so length is duration. UI spec 4.2. No "ideal" marker. */

#include <math.h>
#include <stdio.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define FADE_MS 300u
#define BACK_Y 80.0f
#define THRU_Y 118.0f

static uint32_t ms(float seconds)
{
    return seconds > 0.0f ? (uint32_t)lroundf(seconds * 1000.0f) : 0u;
}

/* How far through [start, start + span] the clock is, 0..1. A span of 0 is a
 * step, not a division by zero. */
static float progress(uint32_t t_ms, uint32_t start, uint32_t span)
{
    if (t_ms <= start) {
        return 0.0f;
    }
    if (span == 0u) {
        return 1.0f;
    }
    return pl_ui_clamp01((float)(t_ms - start) / (float)span);
}

static void draw_ratio(lv_layer_t *layer, const pl_ui_result *result, lv_opa_t opa)
{
    char number[16];
    const char *suffix = " : 1";
    int32_t w_number, w_suffix, x0;

    snprintf(number, sizeof(number), "%.2f",
             (double)(result->backswing_s / result->downswing_s));
    w_number = pl_ui_text_width(&pl_font_hero, number, -1);
    w_suffix = pl_ui_text_width(&pl_font_word, suffix, 0);
    x0 = 120 - (w_number + w_suffix) / 2;
    pl_ui_text(layer, &pl_font_hero, PL_UI_FG, opa, number, x0, 184,
               PL_UI_ALIGN_LEFT, -1);
    pl_ui_text(layer, &pl_font_word, PL_UI_DIM, opa, suffix, x0 + w_number, 184,
               PL_UI_ALIGN_LEFT, 0);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, opa, "BACK : THROUGH",
               120, 208, PL_UI_ALIGN_CENTER, 2);
}

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    float back_px, thru_px, back, thru, fade;
    uint32_t back_ms, thru_ms;

    if (!result->tempo_valid || result->downswing_s <= 0.0f) {
        pl_ui_no_reading(layer, "TEMPO");
        return;
    }
    pl_ui_tempo_bars(result->backswing_s, result->downswing_s, &back_px, &thru_px);
    back_ms = ms(result->backswing_s);
    thru_ms = ms(result->downswing_s);

    pl_ui_line(layer, PL_UI_FAINT, 1, 120.0f, 44.0f, 120.0f, 128.0f, false, 2, 4);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "BALL",
               128, 42, PL_UI_ALIGN_LEFT, 2);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "BACK",
               128, 66, PL_UI_ALIGN_LEFT, 2);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "THROUGH",
               112, 104, PL_UI_ALIGN_RIGHT, 2);

    back = back_px * (back_ms == 0u ? 1.0f : pl_ui_clamp01((float)t_ms / (float)back_ms));
    if (back >= 1.0f) {
        pl_ui_line(layer, PL_UI_ACCENT, 15, 120.0f, BACK_Y, 120.0f + back, BACK_Y,
                   true, 0, 0);
    }
    thru = thru_px * progress(t_ms, back_ms, thru_ms);
    if (thru >= 1.0f) {
        pl_ui_line(layer, PL_UI_ACCENT, 15, 120.0f, THRU_Y, 120.0f - thru, THRU_Y,
                   true, 0, 0);
    }

    fade = progress(t_ms, back_ms + thru_ms, FADE_MS);
    if (fade > 0.0f) {
        draw_ratio(layer, result, (lv_opa_t)lroundf(fade * 255.0f));
    }
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    if (!result->tempo_valid || result->downswing_s <= 0.0f) {
        return 0u;
    }
    return ms(result->backswing_s) + ms(result->downswing_s) + FADE_MS;
}

const pl_ui_screen pl_ui_screen_tempo = { draw, duration_ms };
