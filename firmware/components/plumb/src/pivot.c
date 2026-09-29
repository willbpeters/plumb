/* See plumb/pivot.h. Translation of analysis/plumb/pivot.py.
 *
 * The accumulation is kept literal against the Python, including the order of
 * the arithmetic, for the reason quat.c gives: a difference in the harness then
 * means a translation error, not a rounding artefact to be ruled out.
 *
 * The solve cannot be literal. NumPy's eigh is LAPACK; here it is cyclic
 * Jacobi rotations, which for a symmetric 3x3 are short, need no workspace,
 * and converge quadratically. The eigenvectors therefore differ from NumPy's
 * in their last bits and possibly their signs. Nothing downstream depends on
 * either: the offset is sum_i v_i (v_i . atb) / lambda_i, which is invariant
 * to the sign of v_i. The harness compares the offset, rank and residual.
 */

#include "plumb/pivot.h"

#include <float.h>

#ifdef PLUMB_SINGLE_PRECISION
#define PL_EPSILON FLT_EPSILON
#else
#define PL_EPSILON DBL_EPSILON
#endif

/* Enough for any symmetric 3x3: Jacobi converges quadratically once the
 * off-diagonal is small, and in practice finishes in five or six sweeps. */
#define JACOBI_MAX_SWEEPS 32

/* out = a @ v, row-major 3x3 times 3-vector. */
static void mat_vec(const pl_real a[9], const pl_real v[3], pl_real out[3])
{
    int i;
    for (i = 0; i < 3; i++) {
        out[i] = a[3 * i] * v[0] + a[3 * i + 1] * v[1] + a[3 * i + 2] * v[2];
    }
}

/* out = a^T @ v */
static void mat_t_vec(const pl_real a[9], const pl_real v[3], pl_real out[3])
{
    int j;
    for (j = 0; j < 3; j++) {
        out[j] = a[j] * v[0] + a[3 + j] * v[1] + a[6 + j] * v[2];
    }
}

/* out = a @ b */
static void mat_mul(const pl_real a[9], const pl_real b[9], pl_real out[9])
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j]
                           + a[3 * i + 2] * b[6 + j];
        }
    }
}

/* out = a^T @ b */
static void mat_t_mul(const pl_real a[9], const pl_real b[9], pl_real out[9])
{
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            out[3 * i + j] = a[i] * b[j] + a[3 + i] * b[3 + j]
                           + a[6 + i] * b[6 + j];
        }
    }
}

static pl_real dot3(const pl_real a[3], const pl_real b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* The matrix that performs `v x _`. */
static void skew(const pl_real v[3], pl_real out[9])
{
    const pl_real zero = (pl_real)0.0;
    out[0] = zero;   out[1] = -v[2];  out[2] = v[1];
    out[3] = v[2];   out[4] = zero;   out[5] = -v[0];
    out[6] = -v[1];  out[7] = v[0];   out[8] = zero;
}

/* Eigen-decomposition of a symmetric 3x3 by cyclic Jacobi rotations.
 * `values[i]` pairs with column i of `vectors` (row-major), as eigh's do. */
static void symmetric_eigen(const pl_real m[9], pl_real values[3],
                            pl_real vectors[9])
{
    pl_real a[9];
    int sweep, i, p, q, k;

    for (i = 0; i < 9; i++) {
        a[i] = m[i];
        vectors[i] = (i % 4 == 0) ? (pl_real)1.0 : (pl_real)0.0;
    }

    for (sweep = 0; sweep < JACOBI_MAX_SWEEPS; sweep++) {
        const pl_real off = a[1] * a[1] + a[2] * a[2] + a[5] * a[5];
        const pl_real diag = a[0] * a[0] + a[4] * a[4] + a[8] * a[8];
        /* Converged when the off-diagonal is below rounding of the diagonal.
         * The exact-zero test covers a diagonal that is itself all zero. */
        if (off == (pl_real)0.0
            || off <= PL_EPSILON * PL_EPSILON * diag) {
            break;
        }
        for (p = 0; p < 2; p++) {
            for (q = p + 1; q < 3; q++) {
                const pl_real apq = a[3 * p + q];
                pl_real theta, t, c, s;
                if (apq == (pl_real)0.0) {
                    continue;
                }
                /* Rotate by phi with cot(2 phi) = theta, taking the smaller
                 * root of t^2 + 2 theta t - 1 = 0 for t = tan(phi): the
                 * rotation stays under 45 degrees, which is what makes the
                 * cyclic method converge. */
                theta = (a[3 * q + q] - a[3 * p + p]) / ((pl_real)2.0 * apq);
                t = (pl_real)1.0
                    / (PL_FABS(theta) + PL_SQRT(theta * theta + (pl_real)1.0));
                if (theta < (pl_real)0.0) {
                    t = -t;
                }
                c = (pl_real)1.0 / PL_SQRT(t * t + (pl_real)1.0);
                s = t * c;

                /* a <- J^T a J, vectors <- vectors J, with J the identity
                 * except J[p][p] = J[q][q] = c, J[p][q] = s, J[q][p] = -s. */
                for (k = 0; k < 3; k++) {
                    const pl_real akp = a[3 * k + p], akq = a[3 * k + q];
                    a[3 * k + p] = c * akp - s * akq;
                    a[3 * k + q] = s * akp + c * akq;
                }
                for (k = 0; k < 3; k++) {
                    const pl_real apk = a[3 * p + k], aqk = a[3 * q + k];
                    a[3 * p + k] = c * apk - s * aqk;
                    a[3 * q + k] = s * apk + c * aqk;
                }
                for (k = 0; k < 3; k++) {
                    const pl_real vkp = vectors[3 * k + p];
                    const pl_real vkq = vectors[3 * k + q];
                    vectors[3 * k + p] = c * vkp - s * vkq;
                    vectors[3 * k + q] = s * vkp + c * vkq;
                }
                /* Zero by construction; set it so rounding cannot leave a
                 * residue for the next rotation to chase. */
                a[3 * p + q] = (pl_real)0.0;
                a[3 * q + p] = (pl_real)0.0;
            }
        }
    }

    values[0] = a[0];
    values[1] = a[4];
    values[2] = a[8];
}

void pl_pivot_normal_empty(pl_pivot_normal *eq)
{
    int i;
    for (i = 0; i < 9; i++) {
        eq->ata[i] = (pl_real)0.0;
        eq->noise[i] = (pl_real)0.0;
    }
    for (i = 0; i < 3; i++) {
        eq->atb[i] = (pl_real)0.0;
    }
    eq->btb = (pl_real)0.0;
    eq->samples = 0;
    eq->noise_variance = (pl_real)0.0;
}

void pl_pivot_normal_add(pl_pivot_normal *acc, const pl_pivot_normal *other)
{
    int i;
    for (i = 0; i < 9; i++) {
        acc->ata[i] = acc->ata[i] + other->ata[i];
        acc->noise[i] = acc->noise[i] + other->noise[i];
    }
    for (i = 0; i < 3; i++) {
        acc->atb[i] = acc->atb[i] + other->atb[i];
    }
    acc->btb = acc->btb + other->btb;
    acc->samples = acc->samples + other->samples;
    acc->noise_variance = acc->noise_variance + other->noise_variance;
}

int pl_pivot_solve_normal(const pl_pivot_normal *eq, pl_pivot_solution *out)
{
    const pl_real sigmas = (pl_real)PL_PIVOT_SIGNIFICANCE_SIGMAS;
    pl_real corrected[9], values[3], vectors[9], column[3];
    pl_real coefficients[3], offset[3], ata_offset[3];
    pl_real largest, noise_sd, residual, sigma_b;
    int usable[3], significant[3];
    int i, k, any = 0, rank = 0, dof;

    if (eq->samples < PL_PIVOT_MINIMUM_SAMPLES || eq->btb <= (pl_real)0.0) {
        return 0;
    }

    for (i = 0; i < 9; i++) {
        corrected[i] = eq->ata[i] - eq->noise[i];
    }
    /* Not necessarily positive: subtracting an estimated noise can leave a
     * small negative eigenvalue in a direction that held nothing but noise. */
    symmetric_eigen(corrected, values, vectors);

    largest = values[0];
    for (i = 1; i < 3; i++) {
        if (values[i] > largest) {
            largest = values[i];
        }
    }
    if (largest <= (pl_real)0.0) {
        return 0;
    }
    noise_sd = PL_SQRT(eq->noise_variance);

    for (i = 0; i < 3; i++) {
        usable[i] = values[i] > (pl_real)PL_PIVOT_SINGULAR_VALUE_CUTOFF * largest
                    && values[i] > sigmas * noise_sd;
        any = any || usable[i];
    }
    if (!any) {
        return 0;
    }

    for (i = 0; i < 3; i++) {
        column[0] = vectors[i];
        column[1] = vectors[3 + i];
        column[2] = vectors[6 + i];
        coefficients[i] = usable[i] ? dot3(column, eq->atb) / values[i]
                                    : (pl_real)0.0;
    }
    for (k = 0; k < 3; k++) {
        offset[k] = vectors[3 * k] * coefficients[0]
                  + vectors[3 * k + 1] * coefficients[1]
                  + vectors[3 * k + 2] * coefficients[2];
    }

    /* Residual against the RAW design: what the model leaves unexplained, as
     * a share of what there was to explain. */
    mat_t_vec(eq->ata, offset, ata_offset);   /* offset @ ata */
    residual = eq->btb - (pl_real)2.0 * dot3(offset, eq->atb)
             + dot3(ata_offset, offset);
    if (residual < (pl_real)0.0) {
        residual = (pl_real)0.0;
    }

    /* Standard error per direction, sigma_b / sqrt(lambda). Optimistic --
     * integrated rows are not independent -- hence the second test. */
    dof = 3 * eq->samples - 3;
    if (dof < 1) {
        dof = 1;
    }
    sigma_b = PL_SQRT(residual / (pl_real)dof);
    for (i = 0; i < 3; i++) {
        significant[i] = usable[i]
            && PL_FABS(coefficients[i])
               > sigmas * (sigma_b / PL_SQRT(values[i]));
        rank += significant[i];
    }

    for (k = 0; k < 3; k++) {
        pl_real sum = (pl_real)0.0;
        for (i = 0; i < 3; i++) {
            if (significant[i]) {
                sum = sum + vectors[3 * k + i] * coefficients[i];
            }
        }
        out->offset[k] = sum;
    }
    out->rank = rank;
    out->residual_fraction = residual / eq->btb;
    out->samples = eq->samples;
    return 1;
}

void pl_pivot_init(pl_pivot_estimator *est, pl_real dt)
{
    int i;
    est->dt = dt;
    for (i = 0; i < 9; i++) {
        est->mm[i] = (pl_real)0.0;
        est->sum_m[i] = (pl_real)0.0;
        est->noise_cov[i] = (pl_real)0.0;
    }
    for (i = 0; i < 3; i++) {
        est->velocity[i] = (pl_real)0.0;
        est->previous[i] = (pl_real)0.0;
        est->my[i] = (pl_real)0.0;
        est->sum_y[i] = (pl_real)0.0;
    }
    est->has_previous = 0;
    est->yy = (pl_real)0.0;
    est->samples = 0;
    est->noise_samples = 0;
}

void pl_pivot_set_noise(pl_pivot_estimator *est, const pl_real covariance[9],
                        int samples)
{
    int i;
    for (i = 0; i < 9; i++) {
        est->noise_cov[i] = covariance[i];
    }
    est->noise_samples = samples;
}

void pl_pivot_update(pl_pivot_estimator *est, const pl_real omega[3],
                     const pl_real a_body[3], const pl_real rotation[9])
{
    pl_real a_world[3], s[9], m[9], mtm[9], mty[3];
    const pl_real *y = est->velocity;
    int i;

    mat_vec(rotation, a_body, a_world);
    if (est->has_previous) {
        /* Trapezoidal, as the attitude is: the rectangle rule would put a
         * half-sample lag between the velocity and the design. */
        for (i = 0; i < 3; i++) {
            est->velocity[i] = est->velocity[i]
                + (pl_real)0.5 * (a_world[i] + est->previous[i]) * est->dt;
        }
    }
    for (i = 0; i < 3; i++) {
        est->previous[i] = a_world[i];
    }
    est->has_previous = 1;

    skew(omega, s);
    mat_mul(rotation, s, m);
    mat_t_mul(m, m, mtm);
    mat_t_vec(m, y, mty);
    for (i = 0; i < 9; i++) {
        est->mm[i] = est->mm[i] + mtm[i];
        est->sum_m[i] = est->sum_m[i] + m[i];
    }
    for (i = 0; i < 3; i++) {
        est->my[i] = est->my[i] + mty[i];
    }
    est->yy = est->yy + dot3(y, y);
    for (i = 0; i < 3; i++) {
        est->sum_y[i] = est->sum_y[i] + y[i];
    }
    est->samples = est->samples + 1;
}

void pl_pivot_normal_equations(const pl_pivot_estimator *est,
                               pl_pivot_normal *out)
{
    const int n = est->samples;
    const pl_real *s = est->noise_cov;
    pl_real smts[9], smty[3], trace_s, trace_noise, count;
    int i, j;

    if (n == 0) {
        pl_pivot_normal_empty(out);
        return;
    }
    count = (pl_real)n;

    mat_t_mul(est->sum_m, est->sum_m, smts);
    mat_t_vec(est->sum_m, est->sum_y, smty);
    for (i = 0; i < 9; i++) {
        out->ata[i] = est->mm[i] - smts[i] / count;
    }
    for (i = 0; i < 3; i++) {
        out->atb[i] = est->my[i] - smty[i] / count;
    }
    out->btb = est->yy - dot3(est->sum_y, est->sum_y) / count;
    out->samples = n;

    /* Expected contribution of the gyro noise to ata: (n - 1) because
     * centring removes one sample's worth. See pivot.py for the variance,
     * and for why it is optimistic on the board's coloured noise. */
    trace_s = s[0] + s[4] + s[8];
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            const pl_real eye = (i == j) ? trace_s : (pl_real)0.0;
            out->noise[3 * i + j] = (pl_real)(n - 1) * (eye - s[3 * i + j]);
        }
    }
    out->noise_variance = (pl_real)0.0;
    if (est->noise_samples > 1) {
        const pl_real relative = PL_SQRT(
            (pl_real)2.0 / (pl_real)(est->noise_samples - 1)
            + (pl_real)2.0 / count);
        pl_real sd;
        trace_noise = out->noise[0] + out->noise[4] + out->noise[8];
        sd = relative * trace_noise / (pl_real)3.0;
        out->noise_variance = sd * sd;
    }
}

int pl_pivot_solve(const pl_pivot_estimator *est, pl_pivot_solution *out)
{
    pl_pivot_normal eq;
    pl_pivot_normal_equations(est, &eq);
    return pl_pivot_solve_normal(&eq, out);
}

void pl_pivot_calibration_init(pl_pivot_calibration *cal)
{
    pl_pivot_normal_empty(&cal->eq);
    cal->strokes = 0;
}

void pl_pivot_calibration_fold(pl_pivot_calibration *cal,
                               const pl_pivot_estimator *est)
{
    pl_pivot_normal eq;
    if (est->samples < PL_PIVOT_MINIMUM_SAMPLES) {
        return;
    }
    pl_pivot_normal_equations(est, &eq);
    pl_pivot_normal_add(&cal->eq, &eq);
    cal->strokes = cal->strokes + 1;
}

int pl_pivot_calibration_solve(const pl_pivot_calibration *cal,
                               pl_pivot_solution *out)
{
    return pl_pivot_solve_normal(&cal->eq, out);
}
