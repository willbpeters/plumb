/* The stroke pipeline's configuration on the device, in one place.
 *
 * ================================ PLACEHOLDERS ================================
 * Every threshold below is the synthetic harness's placeholder, copied from
 * analysis/plumb/pipeline.py Thresholds() -- NOT derived from real strokes.
 * Invariant 5: detection thresholds are fixed from the Phase 2 corpus, in the
 * notebook, and only then ported here. analysis/tests/test_stroke_config.py
 * fails if these drift from the Python's, so the device always runs exactly
 * what the harness verified.
 *
 * The lever arm is the harness's 0.85 m too. It is per-putter geometry (parent
 * spec 8.4), and an error in it is a proportional error in impact speed and
 * path. Measure it for the putter in use.
 * ==============================================================================
 */

#include "stroke_config.h"

#include <math.h>

/* analysis/plumb/sensor.py: FULL_SCALE_COUNTS and GRAVITY, which the reference
 * converts with. The driver configures +/-256 dps and +/-16 g
 * (board/src/qmi8658.c, CTRL2/CTRL3). */
#define FULL_SCALE_COUNTS 32768.0
#define GRAVITY 9.81
#define GYRO_DPS 256.0
#define ACCEL_G 16.0
#define PI 3.14159265358979323846

static double radians(double degrees)
{
    return degrees * (PI / 180.0);
}

void stroke_config_fill(pl_pipeline_config *cfg, double sample_rate_hz)
{
    pl_thresholds *th = &cfg->th;
    int i;

    th->stillness_window_s = (pl_real)0.50;
    th->stillness_gyro_std_rad = (pl_real)radians(0.8);
    th->onset_gyro_rad = (pl_real)radians(1.0);
    th->backswing_gyro_rad = (pl_real)radians(8.0);
    th->backswing_hold_s = (pl_real)0.04;
    th->transition_gyro_rad = (pl_real)radians(2.0);
    th->impact_accel_mps2 = (pl_real)100.0;
    th->followthrough_gyro_rad = (pl_real)radians(5.0);
    th->followthrough_hold_s = (pl_real)0.30;
    th->accel_gain_static = (pl_real)0.02;
    th->accel_gain_stroke = (pl_real)0.0;
    th->path_straight_arc_m = (pl_real)0.003;
    th->pivot_max_residual = (pl_real)0.5;
    th->stroke_timeout_s = (pl_real)5.0;

    /* Measured at startup (acq/rate.h), never the nominal 896.8 (spec 6.4). */
    cfg->sample_rate_hz = (pl_real)sample_rate_hz;
    cfg->gyro_rad_per_count = (pl_real)(radians(GYRO_DPS) / FULL_SCALE_COUNTS);
    cfg->accel_mps2_per_count = (pl_real)(ACCEL_G * GRAVITY / FULL_SCALE_COUNTS);
    cfg->lever_arm_m = (pl_real)0.85;

    /* The device calibration (spec 8.1) has not been run on this unit
     * (HANDOFF.md, open defect 6): identity until it has. */
    for (i = 0; i < 3; i++) {
        cfg->accel_offset_mps2[i] = (pl_real)0.0;
        cfg->accel_gain[i] = (pl_real)1.0;
    }
}
