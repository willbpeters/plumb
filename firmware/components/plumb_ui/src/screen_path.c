/* Path: the head retraces the stroke right to left across the target line.
 * UI spec 4.3. The shape is the message: arc is amplified 4x and scaled by
 * the stroke's travel, and no millimetre figure is ever shown.
 *
 * Drawn as a straight segment. The studies wrote it as a quadratic, but its
 * control point was the chord midpoint, which is a straight line; the result
 * carries a direction and one arc magnitude, not a curve. */

#include <math.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define TRACE_MS 1100u
#define TEXT_MS 350u

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    float x0, y0, x1, y1, u, x, y, text;
    const char *words;

    if (!result->path_valid) {
        pl_ui_no_reading(layer, "PATH");
        return;
    }

    pl_ui_line(layer, PL_UI_FAINT, 2, 26.0f, PL_UI_PATH_BASELINE_Y, 214.0f,
               PL_UI_PATH_BASELINE_Y, false, 4, 6);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "OUTSIDE",
               200, 64, PL_UI_ALIGN_RIGHT, 2);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER,
               "INSIDE " PL_UI_MIDDOT " YOU", 200, 188, PL_UI_ALIGN_RIGHT, 2);
    pl_ui_target_mark(layer, false);

    pl_ui_path_ends(result->path_dir, result->path_arc_m, result->path_travel_m,
                    &x0, &y0, &x1, &y1);
    pl_ui_line(layer, PL_UI_FAINT, 2, x0, y0, x1, y1, false, 0, 0);

    u = pl_ui_ease_in_out((float)t_ms / (float)TRACE_MS);
    x = x0 + (x1 - x0) * u;
    y = y0 + (y1 - y0) * u;
    if (fabsf(x - x0) >= 1.0f) {
        pl_ui_line(layer, PL_UI_ACCENT, 5, x0, y0, x, y, true, 0, 0);
    }
    pl_ui_dot(layer, PL_UI_FG, x, y, 7);

    text = t_ms <= TRACE_MS ? 0.0f
         : pl_ui_clamp01((float)(t_ms - TRACE_MS) / (float)TEXT_MS);
    if (text > 0.0f) {
        words = result->path_dir == PL_PATH_OUT_TO_IN   ? "OUT TO IN"
              : result->path_dir == PL_PATH_IN_TO_OUT   ? "IN TO OUT"
                                                        : "STRAIGHT";
        pl_ui_text(layer, &pl_font_word, PL_UI_FG, (lv_opa_t)lroundf(text * 255.0f),
                   words, 120, 216, PL_UI_ALIGN_CENTER, 1);
    }
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    return result->path_valid ? TRACE_MS + TEXT_MS : 0u;
}

const pl_ui_screen pl_ui_screen_path = { draw, duration_ms };
