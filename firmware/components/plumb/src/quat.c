/* See plumb/quat.h. Translation of analysis/plumb/quat.py.
 *
 * Kept deliberately literal against the Python, including the order of the
 * arithmetic. Floating-point addition is not associative, so reassociating an
 * expression for elegance changes the last bits of the answer -- and the whole
 * point of the differential harness is that a difference means a translation
 * error rather than a rounding artefact somebody has to go and rule out.
 */

#include "plumb/quat.h"

static pl_real norm3(const pl_real v[3])
{
    return PL_SQRT(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void pl_quat_identity(pl_real out[4])
{
    out[0] = (pl_real)1.0;
    out[1] = (pl_real)0.0;
    out[2] = (pl_real)0.0;
    out[3] = (pl_real)0.0;
}

void pl_quat_from_axis_angle(const pl_real axis[3], pl_real angle,
                             pl_real out[4])
{
    const pl_real n = norm3(axis);
    pl_real half, s;

    if (n == (pl_real)0.0) {
        pl_quat_identity(out);
        return;
    }

    half = (pl_real)0.5 * angle;
    s = PL_SIN(half);
    out[0] = PL_COS(half);
    out[1] = s * (axis[0] / n);
    out[2] = s * (axis[1] / n);
    out[3] = s * (axis[2] / n);
}

void pl_quat_multiply(const pl_real a[4], const pl_real b[4], pl_real out[4])
{
    const pl_real aw = a[0], ax = a[1], ay = a[2], az = a[3];
    const pl_real bw = b[0], bx = b[1], by = b[2], bz = b[3];

    /* Locals first, so out may alias a or b. */
    const pl_real w = aw * bw - ax * bx - ay * by - az * bz;
    const pl_real x = aw * bx + ax * bw + ay * bz - az * by;
    const pl_real y = aw * by - ax * bz + ay * bw + az * bx;
    const pl_real z = aw * bz + ax * by - ay * bx + az * bw;

    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

void pl_quat_conjugate(const pl_real q[4], pl_real out[4])
{
    const pl_real w = q[0];
    const pl_real x = -q[1];
    const pl_real y = -q[2];
    const pl_real z = -q[3];

    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

void pl_quat_normalize(const pl_real q[4], pl_real out[4])
{
    const pl_real n = PL_SQRT(q[0] * q[0] + q[1] * q[1]
                              + q[2] * q[2] + q[3] * q[3]);
    out[0] = q[0] / n;
    out[1] = q[1] / n;
    out[2] = q[2] / n;
    out[3] = q[3] / n;
}

void pl_quat_rotate(const pl_real q[4], const pl_real v[3], pl_real out[3])
{
    pl_real qv[4], conj[4], tmp[4];

    qv[0] = (pl_real)0.0;
    qv[1] = v[0];
    qv[2] = v[1];
    qv[3] = v[2];

    pl_quat_conjugate(q, conj);
    pl_quat_multiply(q, qv, tmp);
    pl_quat_multiply(tmp, conj, tmp);

    out[0] = tmp[1];
    out[1] = tmp[2];
    out[2] = tmp[3];
}

void pl_quat_integrate(const pl_real q[4], const pl_real omega_body[3],
                       pl_real dt, pl_real out[4])
{
    const pl_real angle = norm3(omega_body) * dt;
    pl_real step[4], product[4];

    if (angle == (pl_real)0.0) {
        out[0] = q[0];
        out[1] = q[1];
        out[2] = q[2];
        out[3] = q[3];
        return;
    }

    pl_quat_from_axis_angle(omega_body, angle, step);
    pl_quat_multiply(q, step, product);
    pl_quat_normalize(product, out);
}

pl_real pl_quat_twist_angle(const pl_real q[4], const pl_real axis[3])
{
    const pl_real n = norm3(axis);
    const pl_real projection = q[1] * (axis[0] / n)
                             + q[2] * (axis[1] / n)
                             + q[3] * (axis[2] / n);
    return (pl_real)2.0 * PL_ATAN2(projection, q[0]);
}

void pl_quat_to_matrix(const pl_real q[4], pl_real out[9])
{
    const pl_real w = q[0], x = q[1], y = q[2], z = q[3];
    const pl_real two = (pl_real)2.0;
    const pl_real one = (pl_real)1.0;

    out[0] = one - two * (y * y + z * z);
    out[1] = two * (x * y - w * z);
    out[2] = two * (x * z + w * y);
    out[3] = two * (x * y + w * z);
    out[4] = one - two * (x * x + z * z);
    out[5] = two * (y * z - w * x);
    out[6] = two * (x * z - w * y);
    out[7] = two * (y * z + w * x);
    out[8] = one - two * (x * x + y * y);
}
