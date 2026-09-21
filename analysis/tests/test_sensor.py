import numpy as np
import pytest

from plumb.sensor import FullScale, SensorParams, simulate
from plumb.trajectory import StrokeParams, generate


def test_counts_are_int16():
    s = simulate(generate(StrokeParams()), SensorParams(), seed=1)
    assert s.gyro_counts.dtype == np.int16
    assert s.accel_counts.dtype == np.int16


def test_noiseless_gyro_roundtrips_within_one_quantum():
    traj = generate(StrokeParams())
    s = simulate(traj, SensorParams(), seed=1)
    fs = FullScale()
    recovered = s.gyro_counts.astype(float) * fs.gyro_rad_per_count
    quantum = fs.gyro_rad_per_count
    np.testing.assert_allclose(recovered, traj.omega_true, atol=quantum)


def test_accelerometer_at_address_reads_gravity_tilted_by_lie_angle():
    """At rest the accelerometer measures specific force, which points 'up' in
    body coordinates: (0, sin(lie), cos(lie)) * 9.81."""
    traj = generate(StrokeParams(lie_angle_deg=20.0))
    s = simulate(traj, SensorParams(), seed=1)
    fs = FullScale()
    a = s.accel_counts[traj.address_end_index // 2].astype(float) * fs.accel_mps2_per_count
    lie = np.radians(20.0)
    expected = 9.81 * np.array([0.0, np.sin(lie), np.cos(lie)])
    np.testing.assert_allclose(a, expected, atol=0.05)


def test_impact_impulse_saturates_and_that_is_acceptable():
    """Clipping at impact is expected, not a defect: impact is a trigger, not a
    measurement (parent spec 6.4). A 60 g impulse against a 16 g full scale
    cannot do anything else.

    Cast to int32 before taking magnitudes. abs() of int16 -32768 overflows
    back to -32768, so a saturation check done in int16 silently reads the
    wrong value at exactly the rail it is trying to detect."""
    traj = generate(StrokeParams())
    s = simulate(traj, SensorParams(), seed=1)
    window = s.accel_counts[traj.impact_index - 2:traj.impact_index + 6].astype(np.int32)
    assert np.abs(window).max() >= 32767


def test_same_seed_gives_identical_output():
    traj = generate(StrokeParams())
    p = SensorParams(gyro_noise_dps=0.5, gyro_bias_dps=1.0)
    a = simulate(traj, p, seed=42)
    b = simulate(traj, p, seed=42)
    np.testing.assert_array_equal(a.gyro_counts, b.gyro_counts)


def test_different_seed_gives_different_noise():
    traj = generate(StrokeParams())
    p = SensorParams(gyro_noise_dps=0.5)
    a = simulate(traj, p, seed=1)
    b = simulate(traj, p, seed=2)
    assert not np.array_equal(a.gyro_counts, b.gyro_counts)
