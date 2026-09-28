/* Pivot-offset estimation, for the path metric. Translation of
 * analysis/plumb/pivot.py -- read its docstring for the reasoning; this file
 * repeats only what a caller needs.
 *
 * The putter swings about the hands, not about the sensor, so the face travels
 * on `r + d` where `d` runs from the pivot to the sensor. `d` enters linearly
 * in the integrated world velocity (v = R [omega]x d - v0), so one stroke is an
 * over-determined system in three unknowns, accumulated a sample at a time and
 * solved once the stroke is over. Summing strokes' normal equations is the
 * per-golfer calibration of parent spec 8.4.
 *
 * Pure C99, no allocation, no globals: builds for host and device from one
 * source, like quat.c.
 */

#ifndef PLUMB_PIVOT_H
#define PLUMB_PIVOT_H

#include "plumb/real.h"

/* Below this many samples there is no averaging to speak of. */
#define PL_PIVOT_MINIMUM_SAMPLES 50

/* How many standard errors a direction must clear to be reported. A chosen
 * false-alarm rate, not a fitted threshold -- see pivot.py. */
#define PL_PIVOT_SIGNIFICANCE_SIGMAS 3.0

/* Relative cutoff for treating an eigenvalue as exactly zero. A numerical
 * guard only; the significance test makes the real decision. */
#define PL_PIVOT_SINGULAR_VALUE_CUTOFF 1e-12

/* One stroke's evidence, or a session's. Matrices are row-major 3x3.
 * `noise` is already included in `ata` and is subtracted by the solve. */
typedef struct {
    pl_real ata[9];
    pl_real atb[3];
    pl_real btb;
    int samples;
    pl_real noise[9];
    pl_real noise_variance;
} pl_pivot_normal;

typedef struct {
    pl_real offset[3];          /* d, pivot to sensor, body frame, metres */
    int rank;                   /* 3 if the stroke excited every direction */
    pl_real residual_fraction;  /* unexplained share of the measured velocity */
    int samples;
} pl_pivot_solution;

typedef struct {
    pl_real dt;
    pl_real velocity[3];
    pl_real previous[3];
    int has_previous;
    pl_real mm[9];
    pl_real my[3];
    pl_real yy;
    pl_real sum_m[9];
    pl_real sum_y[3];
    int samples;
    pl_real noise_cov[9];
    int noise_samples;
} pl_pivot_estimator;

typedef struct {
    pl_pivot_normal eq;
    int strokes;
} pl_pivot_calibration;

void pl_pivot_normal_empty(pl_pivot_normal *eq);

/* acc += other, elementwise. */
void pl_pivot_normal_add(pl_pivot_normal *acc, const pl_pivot_normal *other);

/* Least-squares offset, corrected for noise in the design matrix. Returns 1
 * and fills `out`, or 0 when the evidence shows no pivot -- never a small
 * number standing in for "unknown". */
int pl_pivot_solve_normal(const pl_pivot_normal *eq, pl_pivot_solution *out);

void pl_pivot_init(pl_pivot_estimator *est, pl_real dt);

/* Gyro noise covariance per sample, (rad/s)^2, row-major, measured over
 * `samples` samples of stillness -- the same window that nulls the bias. */
void pl_pivot_set_noise(pl_pivot_estimator *est, const pl_real covariance[9],
                        int samples);

/* One sample. `a_body` has gravity already removed; `rotation` is the
 * attitude as a row-major matrix, body to the address frame. */
void pl_pivot_update(pl_pivot_estimator *est, const pl_real omega[3],
                     const pl_real a_body[3], const pl_real rotation[9]);

/* This stroke's equations, centred to eliminate v0. */
void pl_pivot_normal_equations(const pl_pivot_estimator *est,
                               pl_pivot_normal *out);

int pl_pivot_solve(const pl_pivot_estimator *est, pl_pivot_solution *out);

void pl_pivot_calibration_init(pl_pivot_calibration *cal);

/* Add one completed stroke's evidence; strokes too short to carry any are
 * ignored, as in the Python. */
void pl_pivot_calibration_fold(pl_pivot_calibration *cal,
                               const pl_pivot_estimator *est);

int pl_pivot_calibration_solve(const pl_pivot_calibration *cal,
                               pl_pivot_solution *out);

#endif /* PLUMB_PIVOT_H */
