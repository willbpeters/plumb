"""Quaternion and rotation primitives.

Hamilton convention, scalar-first (w, x, y, z). A quaternion rotates a vector
from body coordinates to reference coordinates: v_ref = q * v_body * q^-1.
"""

import numpy as np


def identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0])


def from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n == 0.0:
        return identity()
    axis = axis / n
    half = 0.5 * angle
    return np.concatenate(([np.cos(half)], np.sin(half) * axis))


def multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = np.concatenate(([0.0], np.asarray(v, dtype=float)))
    return multiply(multiply(q, qv), conjugate(q))[1:]


def integrate(q: np.ndarray, omega_body: np.ndarray, dt: float) -> np.ndarray:
    """Advance attitude by a body-frame angular rate held constant over dt.

    Uses the exact exponential map rather than a first-order update, so
    integration error does not masquerade as an algorithm error in the
    noiseless test.
    """
    omega_body = np.asarray(omega_body, dtype=float)
    angle = np.linalg.norm(omega_body) * dt
    if angle == 0.0:
        return q
    return normalize(multiply(q, from_axis_angle(omega_body, angle)))


def twist_angle(q: np.ndarray, axis: np.ndarray) -> float:
    """Signed rotation of q about `axis` (swing-twist decomposition).

    This is how face angle is extracted: the component of the attitude change
    that is a rotation about measured gravity. Rotation perpendicular to the
    axis contributes nothing.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    projection = float(np.dot(q[1:], axis))
    return 2.0 * np.arctan2(projection, q[0])


def to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def from_matrix(m: np.ndarray) -> np.ndarray:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return normalize(np.array([0.25 / s,
                                   (m[2, 1] - m[1, 2]) * s,
                                   (m[0, 2] - m[2, 0]) * s,
                                   (m[1, 0] - m[0, 1]) * s]))
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return normalize(q)


def rot_x(angle: float) -> np.ndarray:
    return from_axis_angle(np.array([1.0, 0.0, 0.0]), angle)


def rot_y(angle: float) -> np.ndarray:
    return from_axis_angle(np.array([0.0, 1.0, 0.0]), angle)


def rot_z(angle: float) -> np.ndarray:
    return from_axis_angle(np.array([0.0, 0.0, 1.0]), angle)
