/* See geometry.h. */

#include "geometry.h"

#include <math.h>

#define PL_UI_PI 3.14159265358979f

float pl_ui_clamp01(float u)
{
    if (u < 0.0f) {
        return 0.0f;
    }
    if (u > 1.0f) {
        return 1.0f;
    }
    return u;
}

float pl_ui_ease_out(float u)
{
    const float v = 1.0f - pl_ui_clamp01(u);
    return 1.0f - v * v * v;
}

float pl_ui_ease_in_out(float u)
{
    const float v = pl_ui_clamp01(u);
    return v * v * (3.0f - 2.0f * v);
}

float pl_ui_face_rotation_cw_deg(float face_deg)
{
    float drawn = PL_UI_FACE_GAIN * face_deg;
    if (drawn > PL_UI_FACE_CLAMP_DEG) {
        drawn = PL_UI_FACE_CLAMP_DEG;
    }
    if (drawn < -PL_UI_FACE_CLAMP_DEG) {
        drawn = -PL_UI_FACE_CLAMP_DEG;
    }
    return -drawn;
}

pl_ui_face_side pl_ui_face_side_of(float face_deg)
{
    if (fabsf(face_deg) < PL_UI_SQUARE_DEG) {
        return PL_UI_FACE_SQUARE;
    }
    return face_deg > 0.0f ? PL_UI_FACE_OPEN : PL_UI_FACE_CLOSED;
}

void pl_ui_rotate_cw(float cx, float cy, float deg_cw, float x, float y,
                     float *out_x, float *out_y)
{
    const float a = deg_cw * (PL_UI_PI / 180.0f);
    const float c = cosf(a);
    const float s = sinf(a);
    const float dx = x - cx;
    const float dy = y - cy;
    *out_x = cx + dx * c - dy * s;
    *out_y = cy + dx * s + dy * c;
}

void pl_ui_tempo_bars(float backswing_s, float downswing_s,
                      float *back_px, float *thru_px)
{
    float back = PL_UI_TEMPO_PX_PER_S * (backswing_s > 0.0f ? backswing_s : 0.0f);
    float thru = PL_UI_TEMPO_PX_PER_S * (downswing_s > 0.0f ? downswing_s : 0.0f);
    const float longest = back > thru ? back : thru;
    if (longest > PL_UI_TEMPO_MAX_PX) {
        /* One factor for both, so the ratio on screen stays exact. */
        const float scale = PL_UI_TEMPO_MAX_PX / longest;
        back *= scale;
        thru *= scale;
    }
    *back_px = back;
    *thru_px = thru;
}

void pl_ui_path_ends(pl_path_dir dir, float arc_m, float travel_m,
                     float *x0, float *y0, float *x1, float *y1)
{
    float h = 0.0f;
    if (dir != PL_PATH_STRAIGHT && travel_m > 0.0f && arc_m > 0.0f) {
        h = PL_UI_ARC_GAIN * arc_m * (PL_UI_PATH_SPAN_PX / travel_m);
        if (h > PL_UI_PATH_CLAMP_PX) {
            h = PL_UI_PATH_CLAMP_PX;
        }
    }
    *x0 = PL_UI_CENTRE_X + 0.5f * PL_UI_PATH_SPAN_PX;
    *x1 = PL_UI_CENTRE_X - 0.5f * PL_UI_PATH_SPAN_PX;
    if (dir == PL_PATH_IN_TO_OUT) {
        /* Starts inside (below, the golfer's side), finishes outside. */
        *y0 = PL_UI_PATH_BASELINE_Y + h;
        *y1 = PL_UI_PATH_BASELINE_Y - h;
    } else {
        /* Out-to-in starts outside (above); straight has h = 0. */
        *y0 = PL_UI_PATH_BASELINE_Y - h;
        *y1 = PL_UI_PATH_BASELINE_Y + h;
    }
}

float pl_ui_speed_fraction(float mps)
{
    return pl_ui_clamp01(mps / PL_UI_SPEED_FULL_SCALE_MPS);
}
