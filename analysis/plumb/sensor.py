"""QMI8658 forward model.

Emits raw int16 counts, deliberately. A float-valued boundary would let clean
values reach the pipeline with quantization silently skipped, and quantization
sets a hard floor on achievable resolution.
"""

from dataclasses import dataclass

import numpy as np

from plumb import quat
from plumb.trajectory import SAMPLE_RATE_HZ, Trajectory

GRAVITY = 9.81
INT16_MAX = 32767
INT16_MIN = -32768


@dataclass(frozen=True)
class FullScale:
    """Configured ranges (parent spec 6.4). One LSB is the resolution floor."""

    gyro_dps: float = 250.0
    accel_g: float = 16.0

    @property
    def gyro_rad_per_count(self) -> float:
        return np.radians(self.gyro_dps) / INT16_MAX

    @property
    def accel_mps2_per_count(self) -> float:
        return self.accel_g * GRAVITY / INT16_MAX


@dataclass(frozen=True)
class SensorParams:
    gyro_bias_dps: float = 0.0
    gyro_bias_walk_dps_per_s: float = 0.0
    gyro_noise_dps: float = 0.0
    gyro_scale_error: float = 0.0
    accel_bias_mps2: float = 0.0
    accel_noise_mps2: float = 0.0
    accel_scale_error: float = 0.0
    impact_peak_g: float = 60.0
    impact_duration_s: float = 0.004


@dataclass
class SensorOutput:
    gyro_counts: np.ndarray     # (N, 3) int16
    accel_counts: np.ndarray    # (N, 3) int16
    full_scale: FullScale


def _quantize(values: np.ndarray, per_count: float) -> np.ndarray:
    counts = np.rint(values / per_count)
    return np.clip(counts, INT16_MIN, INT16_MAX).astype(np.int16)


def simulate(traj: Trajectory, p: SensorParams, seed: int,
             full_scale: FullScale = FullScale()) -> SensorOutput:
    rng = np.random.default_rng(seed)
    n = len(traj.time)
    dt = 1.0 / SAMPLE_RATE_HZ

    # --- gyroscope --------------------------------------------------------
    walk = np.cumsum(rng.normal(0.0, p.gyro_bias_walk_dps_per_s * np.sqrt(dt), size=(n, 3)), axis=0)
    bias = np.radians(p.gyro_bias_dps + walk)
    noise = np.radians(rng.normal(0.0, p.gyro_noise_dps, size=(n, 3)))
    gyro = traj.omega_true * (1.0 + p.gyro_scale_error) + bias + noise

    # --- accelerometer ----------------------------------------------------
    # Specific force at the sensor: rigid-body acceleration about the pivot,
    # minus gravity, expressed in the body frame.
    #
    # `d` runs FROM the pivot TO the sensor, which is the direction the
    # parent spec's formula assumes. Body Z points head-to-butt, and the pivot
    # (hands and sternum) sits above the grip butt, so d is negative Z.
    # Sanity check: for a pendulum this makes omega x (omega x d) point from
    # the sensor back toward the pivot, which is centripetal, as it must be.
    d = np.array([0.0, 0.0, -traj.params.pivot_offset_m])
    omega_dot = np.gradient(traj.omega_true, dt, axis=0)
    a_body = np.cross(omega_dot, d) + np.cross(traj.omega_true, np.cross(traj.omega_true, d))

    g_world = np.array([0.0, 0.0, -GRAVITY])
    accel = np.empty((n, 3))
    for i in range(n):
        g_body = quat.rotate(quat.conjugate(traj.q_true[i]), g_world)
        accel[i] = a_body[i] - g_body

    # Impact impulse. Allowed to saturate: impact is a trigger, not a
    # measurement (parent spec 6.4).
    width = max(1, int(round(p.impact_duration_s * SAMPLE_RATE_HZ)))
    window = np.hanning(2 * width + 1)
    lo = max(0, traj.impact_index - width)
    hi = min(n, traj.impact_index + width + 1)
    impulse = p.impact_peak_g * GRAVITY * window[: hi - lo]
    accel[lo:hi, 0] -= impulse

    accel = accel * (1.0 + p.accel_scale_error) + p.accel_bias_mps2
    accel += rng.normal(0.0, p.accel_noise_mps2, size=(n, 3)) if p.accel_noise_mps2 else 0.0

    return SensorOutput(
        gyro_counts=_quantize(gyro, full_scale.gyro_rad_per_count),
        accel_counts=_quantize(accel, full_scale.accel_mps2_per_count),
        full_scale=full_scale,
    )
