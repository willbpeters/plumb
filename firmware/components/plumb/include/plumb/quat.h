/* Quaternion and rotation primitives. Translation of analysis/plumb/quat.py.
 *
 * Hamilton convention, scalar-first (w, x, y, z). A quaternion rotates a
 * vector from body coordinates to reference coordinates:
 * v_ref = q * v_body * q^-1.
 *
 * Pure C99. No ESP-IDF, no allocation, no globals -- so it builds for the host
 * and for the device from the same source, which is what lets the differential
 * harness compare it against the Python it came from.
 *
 * `from_matrix` and `rot_x/y/z` are deliberately NOT here. They exist in the
 * Python only to generate synthetic strokes; nothing in Pipeline calls them,
 * and porting code the device will never run would mean maintaining a second
 * copy of something with no test pulling on it.
 */

#ifndef PLUMB_QUAT_H
#define PLUMB_QUAT_H

#include "plumb/real.h"

/* Quaternions are pl_real[4] as {w, x, y, z}; vectors are pl_real[3].
 * Output arrays may alias inputs unless noted. */

void pl_quat_identity(pl_real out[4]);

/* Rotation of `angle` radians about `axis`, which need not be normalised.
 * A zero-length axis yields the identity, matching the Python. */
void pl_quat_from_axis_angle(const pl_real axis[3], pl_real angle,
                             pl_real out[4]);

void pl_quat_multiply(const pl_real a[4], const pl_real b[4], pl_real out[4]);

void pl_quat_conjugate(const pl_real q[4], pl_real out[4]);

void pl_quat_normalize(const pl_real q[4], pl_real out[4]);

/* v_ref = q * v_body * q^-1 */
void pl_quat_rotate(const pl_real q[4], const pl_real v[3], pl_real out[3]);

/* Advance attitude by a body-frame angular rate held constant over dt.
 *
 * The exact exponential map, not a first-order update: integration error that
 * looks like algorithm error is the kind of thing that costs a week. A zero
 * rate returns q unchanged, as the Python does. */
void pl_quat_integrate(const pl_real q[4], const pl_real omega_body[3],
                       pl_real dt, pl_real out[4]);

/* Signed rotation of q about `axis` (swing-twist decomposition).
 *
 * This is how face angle is extracted: the component of the attitude change
 * that is a rotation about measured gravity. Rotation perpendicular to the
 * axis contributes nothing. */
pl_real pl_quat_twist_angle(const pl_real q[4], const pl_real axis[3]);

/* Row-major 3x3. */
void pl_quat_to_matrix(const pl_real q[4], pl_real out[9]);

#endif /* PLUMB_QUAT_H */
