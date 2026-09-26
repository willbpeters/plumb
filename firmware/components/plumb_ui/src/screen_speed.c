/* Impact speed: clubhead speed at impact, as a ring filling anticlockwise
 * from three o'clock, toward the target. UI spec 4.4. Clubhead, not ball. */

#include <math.h>
#include <stdio.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define FILL_MS 800u
#define RING_RADIUS 88
#define RING_WIDTH 5

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    char hero[16];
    int32_t sweep;

    if (!result->speed_valid) {
        pl_ui_no_reading(layer, "IMPACT SPEED");
        return;
    }

    pl_ui_arc(layer, PL_UI_FAINT, RING_RADIUS, RING_WIDTH, 0, 360, false);
    sweep = (int32_t)lroundf(PL_UI_SPEED_MAX_SWEEP_DEG
                             * pl_ui_speed_fraction(result->impact_speed_mps)
                             * pl_ui_ease_out((float)t_ms / (float)FILL_MS));
    if (sweep >= 1) {
        /* LVGL angles run clockwise from three o'clock, so anticlockwise by
         * `sweep` is the span from 360 - sweep round to 360. */
        pl_ui_arc(layer, PL_UI_ACCENT, RING_RADIUS, RING_WIDTH, 360 - sweep, 360, true);
    }

    snprintf(hero, sizeof(hero), "%.2f", (double)result->impact_speed_mps);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "IMPACT SPEED",
               120, 92, PL_UI_ALIGN_CENTER, 2);
    pl_ui_text(layer, &pl_font_hero, PL_UI_FG, LV_OPA_COVER, hero,
               120, 158, PL_UI_ALIGN_CENTER, -1);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "M / S",
               120, 184, PL_UI_ALIGN_CENTER, 2);
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    return result->speed_valid ? FILL_MS : 0u;
}

const pl_ui_screen pl_ui_screen_speed = { draw, duration_ms };
