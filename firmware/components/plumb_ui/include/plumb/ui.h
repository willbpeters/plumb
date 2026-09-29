/* plumb_ui: the result screens. Pure LVGL, no hardware (CLAUDE.md, stack).
 *
 * The UI takes a results struct and draws it; it never computes a metric.
 * Invariant 8 -- nothing renders between arm and follow-through -- is the app
 * task's to enforce, by not calling lv_timer_handler() in that window. The UI
 * starts motion only in pl_ui_show_result(), which the app calls after DONE.
 */
#ifndef PLUMB_UI_H
#define PLUMB_UI_H

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"

typedef enum {
    PL_PATH_STRAIGHT = 0,
    PL_PATH_IN_TO_OUT = 1,
    PL_PATH_OUT_TO_IN = 2
} pl_path_dir;

typedef struct {
    bool face_valid;
    float face_angle_deg;       /* + open, relative to address (invariant 3) */

    bool tempo_valid;
    float backswing_s;
    float downswing_s;          /* tempo ratio = backswing_s / downswing_s */

    bool path_valid;
    pl_path_dir path_dir;
    float path_arc_m;           /* shapes the drawing; never shown */
    float path_travel_m;        /* scales it */

    bool speed_valid;
    float impact_speed_mps;     /* clubhead, not ball */
} pl_ui_result;

enum {
    PL_UI_SCREEN_FACE = 0,
    PL_UI_SCREEN_TEMPO,
    PL_UI_SCREEN_PATH,
    PL_UI_SCREEN_SPEED,
    PL_UI_SCREEN_COUNT
};
#define PL_UI_IDLE (-1)

void pl_ui_init(lv_display_t *display);
void pl_ui_show_result(const pl_ui_result *result);
void pl_ui_next(void);
void pl_ui_prev(void);
void pl_ui_idle(void);
int pl_ui_current_screen(void);

/* Diagnostic: glyphs asked for and not in the font, since init. A missing
 * glyph renders as a silent blank, so this is how it gets noticed. */
uint32_t pl_ui_missing_glyphs(void);

#endif /* PLUMB_UI_H */
