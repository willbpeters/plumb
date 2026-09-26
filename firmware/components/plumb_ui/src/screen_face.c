/* Face angle: a blade from above, heel nearest the golfer, the face lit.
 * UI spec 4.1. Drawn rotation is 5x and clamped; the number is exact. */

#include <math.h>
#include <stdio.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define PIVOT_X 120.0f
#define PIVOT_Y 100.0f
#define SWING_MS 850u

static void draw_head(lv_layer_t *layer, float rotation_cw)
{
    /* Body, 17 px wide, then the 5 px face on its target side, then the
     * hosel stub at the heel. Each is a straight bar rotated about the pivot. */
    pl_ui_rotated_bar(layer, PL_UI_STEEL, 17, 120.5f, 52.0f, 120.5f, 148.0f,
                      PIVOT_X, PIVOT_Y, rotation_cw);
    pl_ui_rotated_bar(layer, PL_UI_ACCENT, 5, 109.5f, 52.0f, 109.5f, 148.0f,
                      PIVOT_X, PIVOT_Y, rotation_cw);
    pl_ui_rotated_bar(layer, PL_UI_STEEL, 7, 129.0f, 141.5f, 143.0f, 141.5f,
                      PIVOT_X, PIVOT_Y, rotation_cw);
}

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    char hero[24];
    const char *label;

    if (!result->face_valid) {
        pl_ui_no_reading(layer, "FACE ANGLE");
        return;
    }

    /* Where square sits, so the gap to the face is the error, unread. */
    pl_ui_line(layer, PL_UI_FAINT, 2, 109.0f, 44.0f, 109.0f, 156.0f, false, 4, 6);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "TOE",
               148, 42, PL_UI_ALIGN_LEFT, 2);
    /* Beside the heel end of the head, not the studies' y=163: at 163 the
     * hero number, whose glyphs start about y=150, drew over it. */
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "HEEL",
               150, 147, PL_UI_ALIGN_LEFT, 2);
    pl_ui_target_mark(layer);

    draw_head(layer, pl_ui_ease_out((float)t_ms / (float)SWING_MS)
                         * pl_ui_face_rotation_cw_deg(result->face_angle_deg));

    snprintf(hero, sizeof(hero), "%.1f" PL_UI_DEG,
             (double)fabsf(result->face_angle_deg));
    switch (pl_ui_face_side_of(result->face_angle_deg)) {
    case PL_UI_FACE_OPEN:
        label = "OPEN " PL_UI_MIDDOT " REL. ADDRESS";
        break;
    case PL_UI_FACE_CLOSED:
        label = "CLOSED " PL_UI_MIDDOT " REL. ADDRESS";
        break;
    default:
        label = "SQUARE " PL_UI_MIDDOT " REL. ADDRESS";
        break;
    }
    pl_ui_text(layer, &pl_font_hero, PL_UI_FG, LV_OPA_COVER, hero,
               120, 196, PL_UI_ALIGN_CENTER, -1);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, label,
               120, 216, PL_UI_ALIGN_CENTER, 2);
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    return result->face_valid ? SWING_MS : 0u;
}

const pl_ui_screen pl_ui_screen_face = { draw, duration_ms };
