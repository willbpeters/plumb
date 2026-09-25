/* The numeric type the algorithm runs in, and the reason it is a choice.
 *
 * The ESP32-S3 has a single-precision FPU. `double` compiles, but it is
 * emulated in software, so every double-precision multiply in the attitude
 * integrator becomes a function call at 896.8 Hz. `float` runs in hardware and
 * is roughly an order of magnitude cheaper.
 *
 * The reference implementation in analysis/ is float64 throughout, because
 * NumPy is. So single precision on the device is a DIVERGENCE from the thing
 * that was proven correct, and the size of that divergence is a measurement,
 * not an assumption -- which is what the differential harness in firmware/test
 * exists to produce. Build it both ways and compare.
 *
 * Default is double, so the port starts out matching the reference exactly and
 * any difference found is a translation error rather than a precision effect.
 * Define PLUMB_SINGLE_PRECISION to measure the cost of the FPU-friendly build.
 *
 * MEASURED, 2026-09-22, 400 random cases per operation, worst difference from
 * the NumPy reference relative to the magnitude of the quantity:
 *
 *     operation        double        float
 *     multiply         0.000e+00     1.078e-07
 *     rotate           0.000e+00     1.644e-07
 *     integrate        0.000e+00     1.025e-07
 *     twist_angle      0.000e+00     3.872e-05
 *     conjugate        0.000e+00
 *     normalize        0.000e+00
 *     from_axis_angle  0.000e+00
 *     to_matrix        0.000e+00
 *
 * The double column is EXACTLY zero on every operation -- the C reproduces
 * NumPy bit for bit, which is what keeping the arithmetic in the Python's
 * order buys. It also requires -ffp-contract=off (set in analysis/tools/
 * cbuild.py and in this component's CMakeLists.txt): a compiler free to fuse
 * a*b+c into one FMA changes the rounding, and the ESP32-S3's GCC does so by
 * default, so without the flag the comparison says nothing about the device.
 * The float column is about one ulp of single precision (1.19e-07)
 * on everything except twist_angle, where the larger number is the relative
 * metric losing meaning as the angle passes through zero rather than the
 * answer being worse.
 *
 * WHOLE STROKE, MEASURED 2026-09-25. One ulp per step is not one ulp per
 * stroke, so the harness also replays every integrate call the Python pipeline
 * makes from ADDRESS entry to impact, carrying the attitude in pl_real from
 * step to step (test_single_precision_across_a_whole_stroke). ~1,850 steps,
 * three putter types, two face angles, noiseless and at 0.28 dps:
 *
 *     double    0 exactly -- the C reproduces the pipeline's face angle
 *               bit for bit across the whole stroke, not just per call
 *     float     at most 4.0e-5 deg of face angle, against a 1.0 deg target
 *
 * A 30 s address hold (53,719 steps) gave 9.9e-6 deg: it does not walk.
 * On this evidence single precision is safe for the attitude integrator,
 * with ~25,000x margin. Not yet covered: forming the rates themselves in
 * float, and everything pipeline.c will add. Re-measure then; the harness
 * is built for exactly that.
 */

#ifndef PLUMB_REAL_H
#define PLUMB_REAL_H

#include <math.h>

#ifdef PLUMB_SINGLE_PRECISION
typedef float pl_real;
#define PL_SQRT(x)        sqrtf(x)
#define PL_ATAN2(y, x)    atan2f((y), (x))
#define PL_SIN(x)         sinf(x)
#define PL_COS(x)         cosf(x)
#define PL_FABS(x)        fabsf(x)
#define PL_REAL_NAME      "float"
#else
typedef double pl_real;
#define PL_SQRT(x)        sqrt(x)
#define PL_ATAN2(y, x)    atan2((y), (x))
#define PL_SIN(x)         sin(x)
#define PL_COS(x)         cos(x)
#define PL_FABS(x)        fabs(x)
#define PL_REAL_NAME      "double"
#endif

#endif /* PLUMB_REAL_H */
