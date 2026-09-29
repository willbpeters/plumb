/* Differential harness: run the ported C against cases the Python generates.
 *
 * The conventions require every algorithm to be proven in Python and then
 * "re-verified against the same corpus" after the port. This is the program
 * that makes that possible. It reads cases on stdin, one per line, and writes
 * the C answers on stdout; analysis/tests/test_c_port.py generates the cases,
 * computes the Python answers, and compares.
 *
 * Why it matters more here than on most projects: there is no JTAG on this
 * board (parent spec 14.3). A translation error found after the port is found
 * with printf, on hardware, against no reference. Found here it is a diff.
 *
 * Deliberately dependency-free and line-oriented so it builds with whatever
 * compiler is to hand -- MSVC on this machine, xtensa-gcc on the device,
 * anything on a CI box.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "plumb/pipeline.h"
#include "plumb/pivot.h"
#include "plumb/quat.h"
#include "plumb/session.h"

/* pivupdate carries fifteen numbers at up to ~25 characters each. */
#define LINE_MAX_CHARS 1024

static void print_reals(const pl_real *values, int count)
{
    int i;
    for (i = 0; i < count; i++) {
        /* 17 significant digits round-trips a double exactly, so the harness
         * compares numbers rather than their decimal shadows. */
        printf("%s%.17g", i ? " " : "", (double)values[i]);
    }
    printf("\n");
}

/* Parse exactly `count` numbers after the op name, each as a double and then
 * converted once to pl_real, as the quaternion ops do. 0 if the line is short. */
static int parse_reals(const char *line, pl_real *out, int count)
{
    const char *at = line + strcspn(line, " \t");
    int i;
    for (i = 0; i < count; i++) {
        char *end;
        const double value = strtod(at, &end);
        if (end == at) {
            return 0;
        }
        out[i] = (pl_real)value;
        at = end;
    }
    return 1;
}

/* "0" for no solution, else 1, offset[3], rank, residual_fraction, samples. */
static void print_solution(int solved, const pl_pivot_solution *solution)
{
    pl_real row[7];
    if (!solved) {
        printf("0\n");
        return;
    }
    row[0] = (pl_real)1.0;
    row[1] = solution->offset[0];
    row[2] = solution->offset[1];
    row[3] = solution->offset[2];
    row[4] = (pl_real)solution->rank;
    row[5] = solution->residual_fraction;
    row[6] = (pl_real)solution->samples;
    print_reals(row, 7);
}

/* Static: the track ring makes these ~260 KB each in double, too big for a
 * stack. */
static pl_pipeline pipeline;
static pl_session session;
static pl_real track_v[PL_PIPELINE_TRACK_MAX][3];
static pl_real track_m[PL_PIPELINE_TRACK_MAX][9];
static const pl_track track = {track_v, track_m};

#define CONFIG_VALUES 24

/* rate, gyro and accel scale, lever arm, the fourteen thresholds in
 * declaration order, accel offset[3] and gain[3]. */
static void config_from(const pl_real *values, pl_pipeline_config *cfg)
{
    pl_thresholds *th = &cfg->th;
    int i;
    cfg->sample_rate_hz = values[0];
    cfg->gyro_rad_per_count = values[1];
    cfg->accel_mps2_per_count = values[2];
    cfg->lever_arm_m = values[3];
    th->stillness_window_s = values[4];
    th->stillness_gyro_std_rad = values[5];
    th->onset_gyro_rad = values[6];
    th->backswing_gyro_rad = values[7];
    th->backswing_hold_s = values[8];
    th->transition_gyro_rad = values[9];
    th->impact_accel_mps2 = values[10];
    th->followthrough_gyro_rad = values[11];
    th->followthrough_hold_s = values[12];
    th->accel_gain_static = values[13];
    th->accel_gain_stroke = values[14];
    th->path_straight_arc_m = values[15];
    th->pivot_max_residual = values[16];
    th->stroke_timeout_s = values[17];
    for (i = 0; i < 3; i++) {
        cfg->accel_offset_mps2[i] = values[18 + i];
        cfg->accel_gain[i] = values[21 + i];
    }
}

static int parse_counts(const char *line, int16_t gyro[3], int16_t accel[3])
{
    pl_real values[6];
    int i;
    if (!parse_reals(line, values, 6)) {
        return 0;
    }
    for (i = 0; i < 3; i++) {
        gyro[i] = (int16_t)values[i];
        accel[i] = (int16_t)values[3 + i];
    }
    return 1;
}

static void print_stroke(const pl_stroke_result *r)
{
    pl_real row[11];
    row[0] = (pl_real)r->face_valid;
    row[1] = r->face_angle_deg;
    row[2] = r->backswing_s;
    row[3] = r->downswing_s;
    row[4] = r->tempo_ratio;
    row[5] = (pl_real)r->path_valid;
    row[6] = r->path_arc_m;
    row[7] = r->path_travel_m;
    row[8] = (pl_real)r->path_direction;
    row[9] = (pl_real)r->speed_valid;
    row[10] = r->impact_speed_mps;
    print_reals(row, 11);
}

int main(void)
{
    /* The pivot estimator and session calibration, carried between lines in
     * pl_real exactly as the device carries them through a stroke. */
    pl_pivot_estimator pivot;
    pl_pivot_calibration calibration;
    char line[LINE_MAX_CHARS];
    /* Attitude carried between lines by the seq* ops, in pl_real, exactly as
     * the device will carry it from sample to sample. Every other op starts
     * from inputs parsed as double; this one never goes back to double, so
     * rounding compounds across a stroke the way it will on hardware. */
    pl_real state[4] = {1, 0, 0, 0};

    pl_pivot_init(&pivot, (pl_real)0.0);
    pl_pivot_calibration_init(&calibration);

    while (fgets(line, sizeof(line), stdin) != NULL) {
        char op[32];
        double a[4], b[4], v[3], dt;
        pl_real qa[4], qb[4], qv[3], out4[4], out3[3], out9[9];
        int i;

        if (sscanf(line, "%31s", op) != 1) {
            continue;
        }

        if (strcmp(op, "identity") == 0) {
            pl_quat_identity(out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "multiply") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &b[0], &b[1], &b[2], &b[3]) != 9) {
                fprintf(stderr, "bad multiply: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; qb[i] = (pl_real)b[i]; }
            pl_quat_multiply(qa, qb, out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "conjugate") == 0
                   || strcmp(op, "normalize") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3]) != 5) {
                fprintf(stderr, "bad %s: %s", op, line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            if (op[0] == 'c') {
                pl_quat_conjugate(qa, out4);
            } else {
                pl_quat_normalize(qa, out4);
            }
            print_reals(out4, 4);

        } else if (strcmp(op, "rotate") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &v[0], &v[1], &v[2]) != 8) {
                fprintf(stderr, "bad rotate: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_rotate(qa, qv, out3);
            print_reals(out3, 3);

        } else if (strcmp(op, "integrate") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &v[0], &v[1], &v[2], &dt) != 9) {
                fprintf(stderr, "bad integrate: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_integrate(qa, qv, (pl_real)dt, out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "twist") == 0) {
            pl_real angle;
            if (sscanf(line, "%31s %lf %lf %lf %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3],
                       &v[0], &v[1], &v[2]) != 8) {
                fprintf(stderr, "bad twist: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            angle = pl_quat_twist_angle(qa, qv);
            print_reals(&angle, 1);

        } else if (strcmp(op, "axisangle") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &v[0], &v[1], &v[2], &dt) != 5) {
                fprintf(stderr, "bad axisangle: %s", line);
                return 1;
            }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_from_axis_angle(qv, (pl_real)dt, out4);
            print_reals(out4, 4);

        } else if (strcmp(op, "tomatrix") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &a[0], &a[1], &a[2], &a[3]) != 5) {
                fprintf(stderr, "bad tomatrix: %s", line);
                return 1;
            }
            for (i = 0; i < 4; i++) { qa[i] = (pl_real)a[i]; }
            pl_quat_to_matrix(qa, out9);
            print_reals(out9, 9);

        } else if (strcmp(op, "seqreset") == 0) {
            pl_quat_identity(state);

        } else if (strcmp(op, "seqstep") == 0) {
            if (sscanf(line, "%31s %lf %lf %lf %lf", op,
                       &v[0], &v[1], &v[2], &dt) != 5) {
                fprintf(stderr, "bad seqstep: %s", line);
                return 1;
            }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            pl_quat_integrate(state, qv, (pl_real)dt, state);

        } else if (strcmp(op, "seqtwist") == 0) {
            pl_real result[5];
            if (sscanf(line, "%31s %lf %lf %lf", op, &v[0], &v[1], &v[2]) != 4) {
                fprintf(stderr, "bad seqtwist: %s", line);
                return 1;
            }
            for (i = 0; i < 3; i++) { qv[i] = (pl_real)v[i]; }
            result[0] = pl_quat_twist_angle(state, qv);
            for (i = 0; i < 4; i++) { result[i + 1] = state[i]; }
            print_reals(result, 5);

        } else if (strcmp(op, "pivreset") == 0) {
            pl_real step;
            if (!parse_reals(line, &step, 1)) {
                fprintf(stderr, "bad pivreset: %s", line);
                return 1;
            }
            pl_pivot_init(&pivot, step);

        } else if (strcmp(op, "pivnoise") == 0) {
            pl_real values[10];
            if (!parse_reals(line, values, 10)) {
                fprintf(stderr, "bad pivnoise: %s", line);
                return 1;
            }
            pl_pivot_set_noise(&pivot, values, (int)values[9]);

        } else if (strcmp(op, "pivupdate") == 0) {
            pl_real values[15];
            if (!parse_reals(line, values, 15)) {
                fprintf(stderr, "bad pivupdate: %s", line);
                return 1;
            }
            pl_pivot_update(&pivot, values, values + 3, values + 6);

        } else if (strcmp(op, "pivnormal") == 0) {
            pl_pivot_normal eq;
            pl_real row[24];
            pl_pivot_normal_equations(&pivot, &eq);
            for (i = 0; i < 9; i++) { row[i] = eq.ata[i]; }
            for (i = 0; i < 3; i++) { row[9 + i] = eq.atb[i]; }
            row[12] = eq.btb;
            row[13] = (pl_real)eq.samples;
            for (i = 0; i < 9; i++) { row[14 + i] = eq.noise[i]; }
            row[23] = eq.noise_variance;
            print_reals(row, 24);

        } else if (strcmp(op, "pivsolve") == 0) {
            pl_pivot_solution solution;
            print_solution(pl_pivot_solve(&pivot, &solution), &solution);

        } else if (strcmp(op, "calreset") == 0) {
            pl_pivot_calibration_init(&calibration);

        } else if (strcmp(op, "calfold") == 0) {
            pl_pivot_calibration_fold(&calibration, &pivot);

        } else if (strcmp(op, "calsolve") == 0) {
            pl_pivot_solution solution;
            print_solution(pl_pivot_calibration_solve(&calibration, &solution),
                           &solution);

        } else if (strcmp(op, "pipinit") == 0) {
            /* The config, then whether to fold into the session calibration. */
            pl_real values[CONFIG_VALUES + 1];
            pl_pipeline_config cfg;
            if (!parse_reals(line, values, CONFIG_VALUES + 1)) {
                fprintf(stderr, "bad pipinit: %s", line);
                return 1;
            }
            config_from(values, &cfg);
            printf("%d\n", pl_pipeline_init(&pipeline, &cfg,
                                            values[CONFIG_VALUES] != 0
                                                ? &calibration : NULL,
                                            &track));

        } else if (strcmp(op, "pipstep") == 0) {
            int16_t gyro[3], accel[3];
            pl_stroke_result result;
            if (!parse_counts(line, gyro, accel)) {
                fprintf(stderr, "bad pipstep: %s", line);
                return 1;
            }
            if (pl_pipeline_step(&pipeline, gyro, accel, &result)) {
                print_stroke(&result);
            }

        } else if (strcmp(op, "sesinit") == 0) {
            pl_real values[CONFIG_VALUES];
            pl_pipeline_config cfg;
            if (!parse_reals(line, values, CONFIG_VALUES)) {
                fprintf(stderr, "bad sesinit: %s", line);
                return 1;
            }
            config_from(values, &cfg);
            printf("%d\n", pl_session_init(&session, &cfg, &track));

        } else if (strcmp(op, "sesstep") == 0) {
            /* A result row (11 values), and on the step a stroke was
             * abandoned, a three-value row: -1, reason, state it left. */
            int16_t gyro[3], accel[3];
            pl_stroke_result result;
            if (!parse_counts(line, gyro, accel)) {
                fprintf(stderr, "bad sesstep: %s", line);
                return 1;
            }
            if (pl_session_step(&session, gyro, accel, &result)) {
                print_stroke(&result);
            }
            if (session.abandoned_this_step) {
                printf("-1 %d %d\n", (int)session.last_abandon_reason,
                       (int)session.last_abandoned_from);
            }

        } else if (strcmp(op, "pipstate") == 0) {
            pl_real row[18];
            row[0] = (pl_real)pipeline.state;
            row[1] = (pl_real)pipeline.n;
            for (i = 0; i < 3; i++) {
                row[2 + i] = pipeline.bias[i];
                row[5 + i] = pipeline.g0[i];
            }
            row[8] = pipeline.i_backswing_start;
            row[9] = pipeline.i_transition;
            row[10] = (pl_real)pipeline.i_impact;
            row[11] = (pl_real)pipeline.i_motion_end;
            for (i = 0; i < 4; i++) { row[12 + i] = pipeline.q_impact[i]; }
            row[16] = (pl_real)pipeline.track_first_n;
            row[17] = (pl_real)pipeline.i_speed_n;
            print_reals(row, 18);

        } else if (strcmp(op, "precision") == 0) {
            printf("%s\n", PL_REAL_NAME);

        } else {
            fprintf(stderr, "unknown op: %s", line);
            return 1;
        }
    }

    return 0;
}
