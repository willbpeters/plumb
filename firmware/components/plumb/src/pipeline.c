/* See plumb/pipeline.h. Translation of analysis/plumb/pipeline.py.
 *
 * Each function below is the Python method of the same name, in the same
 * order, and the reasoning for every step lives in the Python's comments --
 * where the translation had to depart from the Python (the rings, the
 * two-pass path), the reason is here.
 *
 * Arithmetic follows the Python's order wherever NumPy's does not get in the
 * way, for the reason quat.c gives. Where it does -- NumPy's pairwise mean and
 * BLAS-backed cov -- the C sums in order, and the harness measures the result.
 */

#include "plumb/pipeline.h"

#include <stddef.h>

#include "plumb/quat.h"

#define DEGREES_PER_RADIAN (180.0 / 3.14159265358979323846)

static pl_real norm3(const pl_real v[3])
{
    return PL_SQRT(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* np.cross, component for component. */
static void cross(const pl_real a[3], const pl_real b[3], pl_real out[3])
{
    const pl_real x = a[1] * b[2] - a[2] * b[1];
    const pl_real y = a[2] * b[0] - a[0] * b[2];
    const pl_real z = a[0] * b[1] - a[1] * b[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static void enter(pl_pipeline *p, pl_state state)
{
    p->state = state;
    p->hold = 0;
}

int pl_pipeline_init(pl_pipeline *p, const pl_pipeline_config *cfg,
                     pl_pivot_calibration *calibration, const pl_track *track)
{
    int i;

    p->cfg = *cfg;
    p->dt = (pl_real)1.0 / cfg->sample_rate_hz;
    p->still_window = (int)(cfg->th.stillness_window_s / p->dt);
    if (p->still_window < 2 || p->still_window > PL_PIPELINE_STILL_MAX) {
        return 0;
    }

    p->state = PL_STATE_IDLE;
    p->n = 0;
    p->still_count = 0;
    p->still_head = 0;
    p->address_captured = 0;
    for (i = 0; i < 3; i++) {
        p->bias[i] = (pl_real)0.0;
        p->g0[i] = (pl_real)0.0;
        p->prev_omega[i] = (pl_real)0.0;
    }
    pl_quat_identity(p->q);
    p->has_prev_omega = 0;
    p->hold = 0;

    p->last_quiet_n = (pl_real)0.0;
    p->prev_magnitude = (pl_real)0.0;
    p->has_prev_magnitude = 0;
    p->backswing_direction[0] = (pl_real)0.0;
    p->backswing_direction[1] = (pl_real)0.0;
    p->has_backswing_direction = 0;
    p->last_same_sign_n = (pl_real)0.0;

    p->unsettled = 0;
    p->abandon_reason = PL_ABANDON_NONE;
    p->abandoned_from = PL_STATE_IDLE;

    p->i_backswing_start = (pl_real)0.0;
    p->i_transition = (pl_real)0.0;
    p->i_impact = 0;
    p->i_motion_end = 0;
    p->has_motion_end = 0;

    p->impact_start_n = 0;
    p->impact_length = 0;
    p->impact_track_n = 0;
    for (i = 0; i < 3; i++) {
        p->pre_impact_omega[i] = (pl_real)0.0;
    }
    p->has_pre_impact_omega = 0;
    p->i_speed_n = 0;
    pl_quat_identity(p->q_impact);
    p->has_q_impact = 0;

    p->track_v = track->v;
    p->track_m = track->m;
    p->track_first_n = 0;
    p->track_last_n = 0;
    p->has_track = 0;

    pl_pivot_init(&p->pivot, p->dt);
    p->pivot_calibration = calibration;
    p->has_pivot_solution = 0;
    return 1;
}

/* The stillness window: a ring of the last still_window samples. The Python's
 * list, trimmed to the same length, holds the same samples. It runs from IDLE
 * through DOWNSWING -- address capture and the rest test are the same test on
 * the same window. Returns whether the window is full. */
static int still_push(pl_pipeline *p, const pl_real omega[3],
                      const pl_real accel[3])
{
    const int window = p->still_window;
    int i;
    for (i = 0; i < 3; i++) {
        p->still_omega[p->still_head][i] = omega[i];
        p->still_accel[p->still_head][i] = accel[i];
    }
    p->still_head = (p->still_head + 1) % window;
    if (p->still_count < window) {
        p->still_count++;
    }
    return p->still_count >= window;
}

/* Means and ddof-0 variances of a full window, oldest first as the Python's
 * array is ordered; still_head points at the oldest sample. */
static void still_stats(const pl_pipeline *p, pl_real mean_w[3],
                        pl_real mean_a[3], pl_real var[3])
{
    const int window = p->still_window;
    const pl_real count = (pl_real)window;
    int i, k, slot;

    for (i = 0; i < 3; i++) {
        mean_w[i] = (pl_real)0.0;
        mean_a[i] = (pl_real)0.0;
        var[i] = (pl_real)0.0;
    }
    for (k = 0; k < window; k++) {
        slot = (p->still_head + k) % window;
        for (i = 0; i < 3; i++) {
            mean_w[i] = mean_w[i] + p->still_omega[slot][i];
            mean_a[i] = mean_a[i] + p->still_accel[slot][i];
        }
    }
    for (i = 0; i < 3; i++) {
        mean_w[i] = mean_w[i] / count;
        mean_a[i] = mean_a[i] / count;
    }
    for (k = 0; k < window; k++) {
        slot = (p->still_head + k) % window;
        for (i = 0; i < 3; i++) {
            const pl_real d = p->still_omega[slot][i] - mean_w[i];
            var[i] = var[i] + d * d;
        }
    }
    for (i = 0; i < 3; i++) {
        var[i] = var[i] / count;
    }
}

/* The address test: every axis's std (ddof 0, as ndarray.std) under the
 * stillness threshold. */
static int still_test(const pl_pipeline *p, const pl_real var[3])
{
    int i;
    for (i = 0; i < 3; i++) {
        if (!(PL_SQRT(var[i]) < p->cfg.th.stillness_gyro_std_rad)) {
            return 0;
        }
    }
    return 1;
}

/* Wait for stillness, then capture the bias, g0 and the gyro's noise from the
 * same window. */
static void step_idle(pl_pipeline *p, const pl_real omega[3],
                      const pl_real accel[3])
{
    const int window = p->still_window;
    const pl_real count = (pl_real)window;
    pl_real mean_w[3], mean_a[3], var[3], cov[9];
    int i, j, k, slot;

    if (!still_push(p, omega, accel)) {
        return;
    }
    still_stats(p, mean_w, mean_a, var);
    if (!still_test(p, var)) {
        return;
    }

    for (i = 0; i < 9; i++) {
        cov[i] = (pl_real)0.0;
    }
    for (k = 0; k < window; k++) {
        slot = (p->still_head + k) % window;
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                cov[3 * i + j] = cov[3 * i + j]
                    + (p->still_omega[slot][i] - mean_w[i])
                    * (p->still_omega[slot][j] - mean_w[j]);
            }
        }
    }
    for (i = 0; i < 9; i++) {
        cov[i] = cov[i] / (count - (pl_real)1.0);     /* np.cov, ddof 1 */
    }

    for (i = 0; i < 3; i++) {
        p->bias[i] = mean_w[i];
        p->g0[i] = mean_a[i];
    }
    pl_pivot_set_noise(&p->pivot, cov, window);
    p->address_captured = 1;
    pl_quat_identity(p->q);
    enter(p, PL_STATE_ADDRESS);
}

static void step_address(pl_pipeline *p, const pl_real omega[3])
{
    const pl_real onset = p->cfg.th.onset_gyro_rad;
    pl_real corrected[3], magnitude;
    int i;

    for (i = 0; i < 3; i++) {
        corrected[i] = omega[i] - p->bias[i];
    }
    /* Perpendicular to the shaft only (invariant 1). */
    magnitude = PL_SQRT(corrected[0] * corrected[0]
                        + corrected[1] * corrected[1]);

    if (magnitude < onset) {
        p->last_quiet_n = (pl_real)p->n;
    } else if (p->has_prev_magnitude && p->prev_magnitude < onset) {
        const pl_real rise = magnitude - p->prev_magnitude;
        if (rise > (pl_real)0.0) {
            p->last_quiet_n = (pl_real)p->n - (pl_real)1.0
                              - p->prev_magnitude / rise;
        }
    }
    p->prev_magnitude = magnitude;
    p->has_prev_magnitude = 1;

    if (magnitude > p->cfg.th.backswing_gyro_rad) {
        p->hold += 1;
        if ((pl_real)p->hold * p->dt >= p->cfg.th.backswing_hold_s) {
            p->i_backswing_start = p->last_quiet_n;
            enter(p, PL_STATE_BACKSWING);
        }
    } else {
        p->hold = 0;
    }
}

static void integrate(pl_pipeline *p, const pl_real omega[3],
                      const pl_real accel[3])
{
    const pl_real dt = p->dt;
    const int in_stroke = p->state == PL_STATE_BACKSWING
                          || p->state == PL_STATE_DOWNSWING
                          || p->state == PL_STATE_IMPACT
                          || p->state == PL_STATE_FOLLOWTHROUGH;
    pl_real corrected[3], rate[3], gain, r_body[3], v_face[3], rotation[9];
    pl_real s[9], rv[3];
    int moving, slot, i, j;

    for (i = 0; i < 3; i++) {
        corrected[i] = omega[i] - p->bias[i];
    }
    moving = norm3(corrected) > p->cfg.th.onset_gyro_rad;
    gain = (in_stroke || moving) ? p->cfg.th.accel_gain_stroke
                                 : p->cfg.th.accel_gain_static;

    /* Trapezoidal: one sample of latency, not lookahead. */
    if (!p->has_prev_omega) {
        for (i = 0; i < 3; i++) {
            p->prev_omega[i] = corrected[i];
        }
        p->has_prev_omega = 1;
    }
    for (i = 0; i < 3; i++) {
        rate[i] = (pl_real)0.5 * (corrected[i] + p->prev_omega[i]);
        p->prev_omega[i] = corrected[i];
    }
    pl_quat_integrate(p->q, rate, dt, p->q);

    if (gain > (pl_real)0.0 && norm3(accel) > (pl_real)0.0) {
        pl_real measured[3], reference[3], predicted[3], conj[4], error[3];
        const pl_real accel_norm = norm3(accel);
        const pl_real g0_norm = norm3(p->g0);
        for (i = 0; i < 3; i++) {
            measured[i] = accel[i] / accel_norm;
            reference[i] = p->g0[i] / g0_norm;
        }
        pl_quat_conjugate(p->q, conj);
        pl_quat_rotate(conj, reference, predicted);
        /* measured x predicted: the sign that turns toward gravity. */
        cross(measured, predicted, error);
        for (i = 0; i < 3; i++) {
            error[i] = gain * error[i];
        }
        pl_quat_integrate(p->q, error, dt, p->q);
    }

    /* Face-point increment, and the part that waits for the pivot offset,
     * into the ring slot for this sample. */
    r_body[0] = (pl_real)0.0;
    r_body[1] = (pl_real)0.0;
    r_body[2] = -p->cfg.lever_arm_m;
    cross(corrected, r_body, v_face);
    pl_quat_to_matrix(p->q, rotation);
    slot = p->n % PL_PIPELINE_TRACK_MAX;
    for (i = 0; i < 3; i++) {
        rv[i] = rotation[3 * i] * v_face[0] + rotation[3 * i + 1] * v_face[1]
              + rotation[3 * i + 2] * v_face[2];
        p->track_v[slot][i] = rv[i] * dt;
    }
    /* skew(corrected), as pivot.c builds it */
    s[0] = (pl_real)0.0;   s[1] = -corrected[2]; s[2] = corrected[1];
    s[3] = corrected[2];   s[4] = (pl_real)0.0;  s[5] = -corrected[0];
    s[6] = -corrected[1];  s[7] = corrected[0];  s[8] = (pl_real)0.0;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            p->track_m[slot][3 * i + j] =
                (rotation[3 * i] * s[j] + rotation[3 * i + 1] * s[3 + j]
                 + rotation[3 * i + 2] * s[6 + j]) * dt;
        }
    }
    if (!p->has_track) {
        p->track_first_n = p->n;
        p->has_track = 1;
    }
    p->track_last_n = p->n;

    /* The pivot fit, before impact only. */
    if (p->state == PL_STATE_BACKSWING || p->state == PL_STATE_DOWNSWING) {
        pl_real conj[4], gravity_body[3], a_body[3];
        pl_quat_conjugate(p->q, conj);
        pl_quat_rotate(conj, p->g0, gravity_body);
        for (i = 0; i < 3; i++) {
            a_body[i] = accel[i] - gravity_body[i];
        }
        pl_pivot_update(&p->pivot, corrected, a_body, rotation);
    }
}

static void step_backswing(pl_pipeline *p, const pl_real omega[3])
{
    const pl_real x = omega[0] - p->bias[0];
    const pl_real y = omega[1] - p->bias[1];
    pl_real along;

    if (!p->has_backswing_direction) {
        const pl_real magnitude = PL_SQRT(x * x + y * y);
        if (magnitude > (pl_real)0.0) {
            p->backswing_direction[0] = x / magnitude;
            p->backswing_direction[1] = y / magnitude;
            p->has_backswing_direction = 1;
            p->last_same_sign_n = (pl_real)p->n;
        }
        return;
    }

    along = x * p->backswing_direction[0] + y * p->backswing_direction[1];
    if (along > (pl_real)0.0) {
        p->last_same_sign_n = (pl_real)p->n;
    } else if (-along > p->cfg.th.transition_gyro_rad) {
        p->i_transition = p->last_same_sign_n + (pl_real)0.5;
        enter(p, PL_STATE_DOWNSWING);
    }
}

static void step_downswing(pl_pipeline *p, const pl_real accel[3])
{
    int i;
    if (norm3(accel) > p->cfg.th.impact_accel_mps2) {
        p->impact_start_n = p->n;
        p->impact_length = 1;
        for (i = 0; i < 4; i++) {
            p->impact_q[0][i] = p->q[i];
        }
        /* The Python's len(face_track): the track holds this sample already. */
        p->impact_track_n = p->n + 1;
        enter(p, PL_STATE_IMPACT);
        return;
    }
    /* Overwritten by every sample that is not the spike's first, so on the
     * trigger it holds the last one before it: the speed the face arrived
     * with, before the gyro reads the collision too. */
    for (i = 0; i < 3; i++) {
        p->pre_impact_omega[i] = p->prev_omega[i];
    }
    p->has_pre_impact_omega = 1;
    p->i_speed_n = p->n;
}

/* The middle of the above-threshold run. Attitudes are stored only while the
 * run fits PL_PIPELINE_IMPACT_MAX; its length is counted regardless, so the
 * impact INSTANT -- and with it tempo -- never depends on the capacity. */
static void step_impact(pl_pipeline *p, const pl_real accel[3])
{
    int middle, i;

    if (norm3(accel) > p->cfg.th.impact_accel_mps2) {
        if (p->impact_length < PL_PIPELINE_IMPACT_MAX) {
            for (i = 0; i < 4; i++) {
                p->impact_q[p->impact_length][i] = p->q[i];
            }
        }
        p->impact_length += 1;
        return;
    }
    middle = p->impact_length / 2;
    p->i_impact = p->impact_start_n + middle;
    if (middle < PL_PIPELINE_IMPACT_MAX) {
        for (i = 0; i < 4; i++) {
            p->q_impact[i] = p->impact_q[middle][i];
        }
        p->has_q_impact = 1;
    }
    enter(p, PL_STATE_FOLLOWTHROUGH);
}

/* One track sample's increment, with the pivot term when there is one:
 * the Python's `v + m @ d`. */
static void track_increment(const pl_pipeline *p, int n, const pl_real *d,
                            pl_real out[3])
{
    const int slot = n % PL_PIPELINE_TRACK_MAX;
    const pl_real *v = p->track_v[slot];
    const pl_real *m = p->track_m[slot];
    int i;
    for (i = 0; i < 3; i++) {
        out[i] = d == NULL ? v[i]
                           : v[i] + (m[3 * i] * d[0] + m[3 * i + 1] * d[1]
                                     + m[3 * i + 2] * d[2]);
    }
}

static void horizontal_of(const pl_real track[3], const pl_real g_hat[3],
                          pl_real out[3])
{
    const pl_real up = track[0] * g_hat[0] + track[1] * g_hat[1]
                     + track[2] * g_hat[2];
    int i;
    for (i = 0; i < 3; i++) {
        out[i] = track[i] - up * g_hat[i];
    }
}

/* Path over the motion only, in the ground plane. The Python builds the whole
 * cumulative track and slices it; the ring holds only the window, so this
 * sums from the window's first sample instead of from address. Every quantity
 * used -- travel, peak-to-peak lateral, the before/after means' difference --
 * is a difference of track points, so the origin cancels.
 *
 * Two passes rather than storing the track: the first finds the travel, which
 * defines "lateral"; the second recomputes the same running sum and measures
 * lateral against it. Same additions in the same order, so the same numbers. */
static void compute_path(pl_pipeline *p, pl_stroke_result *out)
{
    const pl_real *d = NULL;
    pl_real g_hat[3], track[3], inc[3], h[3], h_first[3], h_last[3];
    pl_real travel[3], forward[3], normal[3], distance, g0_norm;
    pl_real lowest = (pl_real)0.0, highest = (pl_real)0.0;
    pl_real before_sum = (pl_real)0.0, after_sum = (pl_real)0.0;
    int lo, hi, impact, before_count = 0, after_count = 0, n, i;

    out->path_valid = 0;
    out->path_arc_m = (pl_real)0.0;
    out->path_travel_m = (pl_real)0.0;
    out->path_direction = PL_DIRECTION_STRAIGHT;

    if (p->has_pivot_solution && p->pivot_solution.residual_fraction
                                     <= p->cfg.th.pivot_max_residual) {
        d = p->pivot_solution.offset;
    }

    lo = (int)PL_FLOOR(p->i_backswing_start);
    if (lo < p->track_first_n) {
        lo = p->track_first_n;
    }
    hi = p->track_last_n;
    if (p->has_motion_end && p->i_motion_end < hi) {
        hi = p->i_motion_end;
    }
    /* The ring holds samples (track_last_n - TRACK_MAX, track_last_n]. */
    if (hi < lo || lo <= p->track_last_n - PL_PIPELINE_TRACK_MAX) {
        return;
    }
    out->path_valid = 1;

    g0_norm = norm3(p->g0);
    for (i = 0; i < 3; i++) {
        g_hat[i] = p->g0[i] / g0_norm;
        track[i] = (pl_real)0.0;
    }
    for (n = lo; n <= hi; n++) {
        track_increment(p, n, d, inc);
        for (i = 0; i < 3; i++) {
            track[i] = track[i] + inc[i];
        }
        if (n == lo) {
            horizontal_of(track, g_hat, h_first);
        }
    }
    horizontal_of(track, g_hat, h_last);

    for (i = 0; i < 3; i++) {
        travel[i] = h_last[i] - h_first[i];
    }
    distance = norm3(travel);
    out->path_travel_m = distance;
    if (distance == (pl_real)0.0) {
        return;
    }
    for (i = 0; i < 3; i++) {
        forward[i] = travel[i] / distance;
    }
    cross(g_hat, forward, normal);

    impact = p->impact_track_n - lo;    /* index into the window */
    for (i = 0; i < 3; i++) {
        track[i] = (pl_real)0.0;
    }
    for (n = lo; n <= hi; n++) {
        pl_real lateral;
        track_increment(p, n, d, inc);
        for (i = 0; i < 3; i++) {
            track[i] = track[i] + inc[i];
        }
        horizontal_of(track, g_hat, h);
        lateral = h[0] * normal[0] + h[1] * normal[1] + h[2] * normal[2];
        if (n == lo || lateral < lowest) {
            lowest = lateral;
        }
        if (n == lo || lateral > highest) {
            highest = lateral;
        }
        if (n - lo < impact) {
            before_sum = before_sum + lateral;
            before_count++;
        } else {
            after_sum = after_sum + lateral;
            after_count++;
        }
    }
    out->path_arc_m = highest - lowest;

    if (out->path_arc_m < p->cfg.th.path_straight_arc_m) {
        out->path_direction = PL_DIRECTION_STRAIGHT;
    } else {
        const pl_real after = after_count ? after_sum / (pl_real)after_count
                                          : (pl_real)0.0;
        const pl_real before = before_count ? before_sum / (pl_real)before_count
                                            : (pl_real)0.0;
        out->path_direction = (after - before > (pl_real)0.0)
                              ? PL_DIRECTION_IN_TO_OUT
                              : PL_DIRECTION_OUT_TO_IN;
    }
}

/* |omega x (d + r)|: the face is a point of a rigid body rotating about the
 * pivot, and d + r runs from the pivot to it. Rotation preserves length, so
 * the body frame is enough. Runs after compute_path, which solved the pivot. */
static void compute_speed(const pl_pipeline *p, pl_stroke_result *out)
{
    pl_real face[3], v[3];
    const pl_real *d = p->pivot_solution.offset;

    out->speed_valid = 0;
    out->impact_speed_mps = (pl_real)0.0;
    if (!p->has_pre_impact_omega || !p->has_pivot_solution
        || p->pivot_solution.residual_fraction > p->cfg.th.pivot_max_residual) {
        return;
    }
    face[0] = (pl_real)0.0 + d[0];
    face[1] = (pl_real)0.0 + d[1];
    face[2] = -p->cfg.lever_arm_m + d[2];
    cross(p->pre_impact_omega, face, v);
    out->impact_speed_mps = norm3(v);
    out->speed_valid = 1;
}

static void compute(pl_pipeline *p, pl_stroke_result *out, int with_path)
{
    const pl_real backswing = (p->i_transition - p->i_backswing_start) * p->dt;
    const pl_real downswing = ((pl_real)p->i_impact - p->i_transition) * p->dt;

    out->backswing_s = backswing;
    out->downswing_s = downswing;
    out->tempo_ratio = backswing / downswing;

    /* Twist about measured gravity, relative to address (invariant 3). */
    out->face_valid = p->has_q_impact;
    out->face_angle_deg = p->has_q_impact
        ? pl_quat_twist_angle(p->q_impact, p->g0) * (pl_real)DEGREES_PER_RADIAN
        : (pl_real)0.0;

    /* The pivot first: speed needs it whether or not there is a path. */
    if (p->pivot_calibration != NULL) {
        pl_pivot_calibration_fold(p->pivot_calibration, &p->pivot);
        p->has_pivot_solution = pl_pivot_calibration_solve(
            p->pivot_calibration, &p->pivot_solution);
    } else {
        p->has_pivot_solution = pl_pivot_solve(&p->pivot, &p->pivot_solution);
    }

    if (with_path) {
        compute_path(p, out);
    } else {
        out->path_valid = 0;
        out->path_arc_m = (pl_real)0.0;
        out->path_travel_m = (pl_real)0.0;
        out->path_direction = PL_DIRECTION_STRAIGHT;
    }
    compute_speed(p, out);
}

static int step_followthrough(pl_pipeline *p, const pl_real omega[3],
                              pl_stroke_result *out)
{
    pl_real corrected[3];
    int i;
    for (i = 0; i < 3; i++) {
        corrected[i] = omega[i] - p->bias[i];
    }
    if (norm3(corrected) < p->cfg.th.followthrough_gyro_rad) {
        if (p->hold == 0) {
            p->i_motion_end = p->n;
            p->has_motion_end = 1;
        }
        p->hold += 1;
        if ((pl_real)p->hold * p->dt >= p->cfg.th.followthrough_hold_s) {
            enter(p, PL_STATE_DONE);
            compute(p, out, 1);
            return 1;
        }
    } else {
        p->hold = 0;
    }
    return 0;
}

static void abandon(pl_pipeline *p, pl_abandon_reason reason)
{
    p->abandon_reason = reason;
    p->abandoned_from = p->state;
    enter(p, PL_STATE_ABANDONED);
}

/* The ways out of a stroke parent spec 7.1 does not have: back at rest
 * before impact, and the timeout. The reasoning is in pipeline.py,
 * _check_exits. Returns 0 to carry on, 1 with a result in `out`, 2 when the
 * stroke was abandoned. */
static int check_exits(pl_pipeline *p, const pl_real omega[3],
                       const pl_real accel[3], pl_stroke_result *out)
{
    if (p->state == PL_STATE_ADDRESS || p->state == PL_STATE_BACKSWING
        || p->state == PL_STATE_DOWNSWING) {
        if (still_push(p, omega, accel)) {
            pl_real mean_w[3], mean_a[3], var[3];
            still_stats(p, mean_w, mean_a, var);
            if (!still_test(p, var)) {
                p->unsettled = 1;
            } else if (p->unsettled) {
                abandon(p, PL_ABANDON_REST);
                return 2;
            }
        }
    }

    if ((p->state == PL_STATE_BACKSWING || p->state == PL_STATE_DOWNSWING
         || p->state == PL_STATE_FOLLOWTHROUGH)
        && ((pl_real)p->n - p->i_backswing_start) * p->dt
           > p->cfg.th.stroke_timeout_s) {
        if (p->state == PL_STATE_FOLLOWTHROUGH) {
            enter(p, PL_STATE_DONE);
            compute(p, out, 0);
            return 1;
        }
        abandon(p, PL_ABANDON_TIMEOUT);
        return 2;
    }
    return 0;
}

int pl_pipeline_step(pl_pipeline *p, const int16_t gyro_counts[3],
                     const int16_t accel_counts[3], pl_stroke_result *out)
{
    pl_real omega[3], accel[3];
    int i;

    for (i = 0; i < 3; i++) {
        omega[i] = (pl_real)gyro_counts[i] * p->cfg.gyro_rad_per_count;
        accel[i] = ((pl_real)accel_counts[i] * p->cfg.accel_mps2_per_count
                    - p->cfg.accel_offset_mps2[i]) * p->cfg.accel_gain[i];
    }
    p->n += 1;

    if (p->state == PL_STATE_IDLE) {
        step_idle(p, omega, accel);
        return 0;
    }
    if (p->state == PL_STATE_DONE || p->state == PL_STATE_ABANDONED) {
        return 0;
    }

    integrate(p, omega, accel);

    switch (check_exits(p, omega, accel, out)) {
    case 1:
        return 1;
    case 2:
        return 0;
    default:
        break;
    }

    switch (p->state) {
    case PL_STATE_ADDRESS:
        step_address(p, omega);
        break;
    case PL_STATE_BACKSWING:
        step_backswing(p, omega);
        break;
    case PL_STATE_DOWNSWING:
        step_downswing(p, accel);
        break;
    case PL_STATE_IMPACT:
        step_impact(p, accel);
        break;
    case PL_STATE_FOLLOWTHROUGH:
        return step_followthrough(p, omega, out);
    default:
        break;
    }
    return 0;
}
