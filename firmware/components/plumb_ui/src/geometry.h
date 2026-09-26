/* Pure display maths for plumb_ui. No LVGL, so it is testable on its own.
 *
 * Every constant here is a DISPLAY range or amplification, not a measurement
 * threshold; invariant 5 does not apply to them. The numbers shown on screen
 * are never amplified or clamped, only the drawings.
 */
#ifndef PL_UI_GEOMETRY_H
#define PL_UI_GEOMETRY_H

#include "plumb/ui.h"

/* 1.8 deg across a 110 px head moves the toe three pixels: invisible. */
#define PL_UI_FACE_GAIN 5.0f
/* Past this a 12 deg reading would spin the head sideways. */
#define PL_UI_FACE_CLAMP_DEG 45.0f
/* Where %.1f stops printing 0.0: a display resolution, not a verdict. */
#define PL_UI_SQUARE_DEG 0.05f

/* Same speed both ways, so bar length IS duration. */
#define PL_UI_TEMPO_PX_PER_S 118.0f
/* Longest bar that stays inside the aperture with its round cap. */
#define PL_UI_TEMPO_MAX_PX 90.0f

#define PL_UI_ARC_GAIN 4.0f
#define PL_UI_PATH_SPAN_PX 152.0f
#define PL_UI_PATH_CLAMP_PX 50.0f
#define PL_UI_PATH_BASELINE_Y 122.0f
#define PL_UI_CENTRE_X 120.0f

/* Typical putts land a third to two-thirds round. */
#define PL_UI_SPEED_FULL_SCALE_MPS 3.0f
#define PL_UI_SPEED_MAX_SWEEP_DEG 270.0f

typedef enum {
    PL_UI_FACE_SQUARE = 0,
    PL_UI_FACE_OPEN = 1,
    PL_UI_FACE_CLOSED = 2
} pl_ui_face_side;

float pl_ui_clamp01(float u);
float pl_ui_ease_out(float u);      /* cubic, u clamped to 0..1 */
float pl_ui_ease_in_out(float u);   /* smoothstep, u clamped to 0..1 */

/* On-screen rotation of the head, degrees, positive CLOCKWISE on screen.
 * Open turns the toe toward the target (left), so open is negative. */
float pl_ui_face_rotation_cw_deg(float face_deg);
pl_ui_face_side pl_ui_face_side_of(float face_deg);

/* Rotate (x, y) about (cx, cy) by deg_cw, clockwise on a y-down screen. */
void pl_ui_rotate_cw(float cx, float cy, float deg_cw, float x, float y,
                     float *out_x, float *out_y);

void pl_ui_tempo_bars(float backswing_s, float downswing_s,
                      float *back_px, float *thru_px);

/* The stroke runs right to left: (x0, y0) is where it starts, on the right. */
void pl_ui_path_ends(pl_path_dir dir, float arc_m, float travel_m,
                     float *x0, float *y0, float *x1, float *y1);

float pl_ui_speed_fraction(float mps);

#endif /* PL_UI_GEOMETRY_H */
