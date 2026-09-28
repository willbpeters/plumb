/* The stroke pipeline: parent spec section 7, one sample at a time.
 * Translation of analysis/plumb/pipeline.py -- the reasoning behind every step
 * is in its comments and is not repeated here. What IS here is what the Python
 * never had to decide: where the samples live.
 *
 * The Python keeps every sample from address onward in a list. The device
 * cannot, so the per-sample path data sits in a ring of PL_PIPELINE_TRACK_MAX
 * samples. A stroke whose path window (backswing onset to the end of the
 * follow-through hold) outruns the ring reports path as UNAVAILABLE, never as a
 * number computed from part of the stroke. Face angle and tempo do not touch
 * the ring and cannot be affected by its size.
 *
 * No threshold has a value in this file or in pipeline.c. They all arrive in
 * pl_pipeline_config (invariant 5): the Python's defaults are placeholders for
 * the synthetic harness, to be replaced from the Phase 2 corpus.
 *
 * One pipeline instance is one stroke, as in the Python: DONE is terminal, and
 * the caller initialises a fresh one for the next stroke. The pivot
 * calibration, which is the only thing that persists across strokes, lives
 * outside it.
 *
 * Pure C99, no allocation, no globals.
 */

#ifndef PLUMB_PIPELINE_H
#define PLUMB_PIPELINE_H

#include <stdint.h>

#include "plumb/pivot.h"
#include "plumb/real.h"

/* Stillness window, in samples. 0.5 s at the 1793.6 Hz step is 896. */
#ifndef PL_PIPELINE_STILL_MAX
#define PL_PIPELINE_STILL_MAX 1024
#endif

/* Path window, in samples: 48 bytes each in single precision. 2720 is 3.0 s at
 * this board's 906.86 Hz, 130 KB. Measured 2026-09-28 on the synthetic
 * harness: the slowest strokes generated (1.0 s backswing, tempo 1.5) need
 * 2318 samples at 896.8 Hz from onset to DONE. Will's call; see HANDOFF.md. */
#ifndef PL_PIPELINE_TRACK_MAX
#define PL_PIPELINE_TRACK_MAX 2720
#endif

/* Attitudes held through the impact spike, whose middle is the impact instant.
 * A putt's impulse is a few milliseconds -- four or five samples. A run longer
 * than this is not a putt, and face angle is reported unavailable. */
#ifndef PL_PIPELINE_IMPACT_MAX
#define PL_PIPELINE_IMPACT_MAX 64
#endif

typedef enum {
    PL_STATE_IDLE = 0,
    PL_STATE_ADDRESS,
    PL_STATE_BACKSWING,
    PL_STATE_DOWNSWING,
    PL_STATE_IMPACT,
    PL_STATE_FOLLOWTHROUGH,
    PL_STATE_DONE
} pl_state;

/* Values match plumb_ui's pl_path_dir, which this component does not include:
 * the algorithm knows nothing about the screen. */
typedef enum {
    PL_DIRECTION_STRAIGHT = 0,
    PL_DIRECTION_IN_TO_OUT = 1,
    PL_DIRECTION_OUT_TO_IN = 2
} pl_direction;

/* Field for field, analysis/plumb/pipeline.py Thresholds. */
typedef struct {
    pl_real stillness_window_s;
    pl_real stillness_gyro_std_rad;
    pl_real onset_gyro_rad;
    pl_real backswing_gyro_rad;
    pl_real backswing_hold_s;
    pl_real transition_gyro_rad;
    pl_real impact_accel_mps2;
    pl_real followthrough_gyro_rad;
    pl_real followthrough_hold_s;
    pl_real accel_gain_static;
    pl_real accel_gain_stroke;
    pl_real path_straight_arc_m;
    pl_real pivot_max_residual;
} pl_thresholds;

typedef struct {
    pl_thresholds th;
    /* The rate the samples are actually taken at, measured by the caller
     * (Will's decision, 2026-09-27) -- not the datasheet's nominal. */
    pl_real sample_rate_hz;
    pl_real gyro_rad_per_count;
    pl_real accel_mps2_per_count;
    pl_real lever_arm_m;
    /* Device calibration, parent spec 8.1: true = (raw - offset) * gain.
     * Zero and one for an uncalibrated unit. */
    pl_real accel_offset_mps2[3];
    pl_real accel_gain[3];
} pl_pipeline_config;

typedef struct {
    int face_valid;
    pl_real face_angle_deg;     /* relative to address (invariant 3) */
    pl_real backswing_s;
    pl_real downswing_s;
    pl_real tempo_ratio;
    int path_valid;
    pl_real path_arc_m;
    pl_real path_travel_m;
    pl_direction path_direction;
    /* Clubhead speed arriving at the ball (parent spec 1.2.1). Unavailable
     * without a pivot solution: the lever arm alone would read ~39% low. */
    int speed_valid;
    pl_real impact_speed_mps;
} pl_stroke_result;

typedef struct {
    pl_pipeline_config cfg;
    pl_real dt;
    int still_window;

    pl_state state;
    int n;

    /* IDLE: the last still_window samples, as a ring. */
    pl_real still_omega[PL_PIPELINE_STILL_MAX][3];
    pl_real still_accel[PL_PIPELINE_STILL_MAX][3];
    int still_count;
    int still_head;

    int address_captured;
    pl_real bias[3];
    pl_real g0[3];
    pl_real q[4];
    pl_real prev_omega[3];
    int has_prev_omega;
    int hold;

    pl_real last_quiet_n;
    pl_real prev_magnitude;
    int has_prev_magnitude;
    pl_real backswing_direction[2];
    int has_backswing_direction;
    pl_real last_same_sign_n;

    pl_real i_backswing_start;
    pl_real i_transition;
    int i_impact;
    int i_motion_end;
    int has_motion_end;

    /* IMPACT: attitudes through the spike, from its first sample. */
    pl_real impact_q[PL_PIPELINE_IMPACT_MAX][4];
    int impact_start_n;
    int impact_length;
    int impact_track_n;         /* first track sample after the spike began */
    /* The last downswing sample before the spike, for impact speed. */
    pl_real pre_impact_omega[3];
    int has_pre_impact_omega;
    int i_speed_n;
    pl_real q_impact[4];
    int has_q_impact;

    /* Path: per-sample R v dt and R [omega]x dt, slot n % TRACK_MAX. */
    pl_real track_v[PL_PIPELINE_TRACK_MAX][3];
    pl_real track_m[PL_PIPELINE_TRACK_MAX][9];
    int track_first_n;
    int track_last_n;
    int has_track;

    pl_pivot_estimator pivot;
    pl_pivot_calibration *pivot_calibration;    /* NULL: this stroke alone */
    pl_pivot_solution pivot_solution;
    int has_pivot_solution;
} pl_pipeline;

/* Returns 0 if the config cannot be run: a stillness window that is empty or
 * larger than PL_PIPELINE_STILL_MAX at this sample rate. `calibration` may be
 * NULL; if not, it must outlive the stroke, and the stroke is folded into it
 * at DONE. */
int pl_pipeline_init(pl_pipeline *p, const pl_pipeline_config *cfg,
                     pl_pivot_calibration *calibration);

/* One sample, in raw counts. Returns 1 and fills `out` on the sample the
 * stroke completes; 0 otherwise. */
int pl_pipeline_step(pl_pipeline *p, const int16_t gyro_counts[3],
                     const int16_t accel_counts[3], pl_stroke_result *out);

#endif /* PLUMB_PIPELINE_H */
