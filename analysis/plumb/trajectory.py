"""Ground-truth stroke generation.

Produces attitude and body-frame angular velocity at the stroke ODR
(SAMPLE_RATE_HZ, below) for a stroke whose face angle at impact we specify.
Offline only; never ported to C.

Frames and conventions are documented in the implementation plan preamble.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np

from plumb import quat

# The QMI8658's ODR steps derive from the gyro's natural frequency, not from
# round numbers, so the spec's original 500 Hz does not exist on this part. The
# neighbours are 448.4 and 896.8 Hz; 896.8 was chosen because tempo ratio is the
# binding accuracy constraint and tempo error scales with sample resolution
# (parent spec 6.4, amended 2026-09-21).
#
# Nominal, and deliberately still nominal now that the real rate is known.
#
# The board measures 906.86 Hz -- 1.12% high, repeatable to +/-0.01 Hz across
# five captures, taken from the sensor's own sample counter against the MCU
# clock (docs/bringup-results.md). MEMS oscillators do run a few percent off,
# and that error goes straight into every integrated angle.
#
# It is not moved to 906.86 because that is THIS board's oscillator, not the
# part's. Putting one unit's calibration into a shared constant would trade a
# known 1.12% error for a hidden one of unknown size on every other unit. The
# decision that actually resolves it -- per-unit calibration, or firmware that
# measures its own rate at startup, which it now can -- is open and is Will's.
SAMPLE_RATE_HZ = 896.8


class ArcType(Enum):
    """How much the face rotates with the swing.

    This controls the motion the device measures, not how it measures it
    (parent spec 2.1). Face angle at impact is identical across all three.
    """

    STRAIGHT = 0.0
    NEAR_ZERO_ROTATION = 0.02
    ARCED = 0.35


@dataclass(frozen=True)
class StrokeParams:
    backswing_amplitude_deg: float = 12.0
    followthrough_amplitude_deg: float = 12.0
    backswing_duration_s: float = 0.70
    tempo_ratio: float = 2.0
    face_angle_at_impact_deg: float = 0.0
    arc_type: ArcType = ArcType.ARCED
    lie_angle_deg: float = 20.0
    lever_arm_m: float = 0.85          # sensor to sweet spot
    pivot_offset_m: float = 0.55       # sensor to rotation centre
    address_duration_s: float = 1.00
    followthrough_hold_s: float = 0.60


@dataclass
class Trajectory:
    time: np.ndarray
    theta: np.ndarray
    phi: np.ndarray
    q_true: np.ndarray          # (N, 4)
    omega_true: np.ndarray      # (N, 3) body frame, rad/s
    address_end_index: int
    transition_index: int
    impact_index: int
    true_face_angle_deg: float
    true_tempo_ratio: float
    params: StrokeParams


def _smoothstep(u: np.ndarray) -> np.ndarray:
    """Raised-cosine ramp from 0 to 1 with zero slope at both ends."""
    return 0.5 * (1.0 - np.cos(np.pi * np.clip(u, 0.0, 1.0)))


def _smoothstep_derivative(u: np.ndarray, duration: float) -> np.ndarray:
    inside = (u >= 0.0) & (u <= 1.0)
    return np.where(inside, 0.5 * np.pi * np.sin(np.pi * np.clip(u, 0.0, 1.0)) / duration, 0.0)


def shaft_rotation_for_face_angle(face_angle_rad: float, lie_angle_rad: float) -> float:
    """Invert the ground-plane projection.

    Rotating a shaft tilted by `lie` through `phi` moves the face normal by
    atan(tan(phi) * cos(lie)) in the ground plane, not by phi. The generator is
    parameterized by the reportable quantity, so it solves for phi.
    """
    return float(np.arctan(np.tan(face_angle_rad) / np.cos(lie_angle_rad)))


def generate(p: StrokeParams) -> Trajectory:
    dt = 1.0 / SAMPLE_RATE_HZ
    lie = np.radians(p.lie_angle_deg)
    amp_back = np.radians(p.backswing_amplitude_deg)
    amp_fwd = np.radians(p.followthrough_amplitude_deg)

    # Forward phase sweeps from -amp_back to +amp_fwd on a raised cosine, so
    # theta = 0 falls at a known fraction of that phase. Impact is that crossing.
    u_impact = np.arccos(1.0 - 2.0 * amp_back / (amp_back + amp_fwd)) / np.pi

    # Phase boundaries snap to whole samples and every duration is derived from
    # the snapped counts. That is what makes theta exactly zero at impact: at
    # that sample u equals u_impact exactly, by construction.
    #
    # Without snapping, a tempo ratio like 1.5 puts impact at sample 233.33 and
    # theta is merely near zero there. The `arc_gain * theta` term in the face
    # rotation then fails to vanish, ground truth drifts by roughly 0.02 deg,
    # and the drift scales with arc_gain -- which would look exactly like the
    # putter-type dependence invariant 1 prohibits, while actually being a
    # sampling artifact of the generator.
    #
    # A grid at SAMPLE_RATE_HZ cannot land an arbitrary tempo ratio on a whole
    # sample, so the achieved ratio is recorded rather than the requested one.
    n_addr = int(round(p.address_duration_s / dt))
    n_back = int(round(p.backswing_duration_s / dt))
    n_down = int(round(p.backswing_duration_s / p.tempo_ratio / dt))

    backswing_duration = n_back * dt
    forward_duration = (n_down * dt) / u_impact

    address_end_index = n_addr
    transition_index = n_addr + n_back
    impact_index = transition_index + n_down

    t_addr_end = address_end_index * dt
    t_transition = transition_index * dt
    t_impact = impact_index * dt
    t_end = t_transition + forward_duration + p.followthrough_hold_s

    n = int(round(t_end / dt)) + 1
    time = np.arange(n) * dt

    # --- swing angle theta(t) and its derivative -------------------------
    theta = np.zeros(n)
    theta_dot = np.zeros(n)

    back = (time >= t_addr_end) & (time < t_transition)
    u = (time[back] - t_addr_end) / backswing_duration
    theta[back] = -amp_back * _smoothstep(u)
    theta_dot[back] = -amp_back * _smoothstep_derivative(u, backswing_duration)

    fwd = time >= t_transition
    u = np.clip((time[fwd] - t_transition) / forward_duration, 0.0, 1.0)
    theta[fwd] = -amp_back + (amp_back + amp_fwd) * _smoothstep(u)
    theta_dot[fwd] = (amp_back + amp_fwd) * _smoothstep_derivative(u, forward_duration)

    # --- face rotation phi(t) and its derivative -------------------------
    # Two independent contributions:
    #   arc_gain * theta  - face rotating with the swing. Zero at impact by
    #                       construction, since theta is zero there.
    #   ramp * phi_impact - the delivered face angle.
    arc_gain = p.arc_type.value
    phi_impact = shaft_rotation_for_face_angle(np.radians(p.face_angle_at_impact_deg), lie)

    ramp_duration = t_impact - t_addr_end
    u_ramp = (time - t_addr_end) / ramp_duration
    ramp = _smoothstep(u_ramp)
    ramp_dot = _smoothstep_derivative(u_ramp, ramp_duration)

    phi = arc_gain * theta + phi_impact * ramp
    phi_dot = arc_gain * theta_dot + phi_impact * ramp_dot

    # --- attitude and angular velocity -----------------------------------
    q_true = np.empty((n, 4))
    omega_true = np.empty((n, 3))
    q_lie = quat.rot_x(lie)
    for i in range(n):
        q_true[i] = quat.multiply(quat.multiply(q_lie, quat.rot_y(theta[i])), quat.rot_z(phi[i]))
        omega_true[i] = (theta_dot[i] * np.array([np.sin(phi[i]), np.cos(phi[i]), 0.0])
                         + phi_dot[i] * np.array([0.0, 0.0, 1.0]))

    return Trajectory(
        time=time,
        theta=theta,
        phi=phi,
        q_true=q_true,
        omega_true=omega_true,
        address_end_index=address_end_index,
        transition_index=transition_index,
        impact_index=impact_index,
        true_face_angle_deg=p.face_angle_at_impact_deg,
        true_tempo_ratio=n_back / n_down,
        params=p,
    )
