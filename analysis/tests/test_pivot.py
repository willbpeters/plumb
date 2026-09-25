"""Tests for pivot-offset estimation.

The pivot offset is the missing third of the path measurement. The pipeline
computes face velocity as `omega x r`, which assumes the sensor does not
translate; it does, because the putter swings about the hands rather than about
the sensor, so the face travels on `r + d` where `d` runs from the pivot to the
sensor. With the harness's defaults that is 1.4 m against the 0.85 m assumed,
and 0.85/1.4 is the 61% shortfall once pinned in test_pipeline.py.

Ground truth is known here because these tests generate it.
"""

import numpy as np
import pytest

from plumb import quat
from plumb.pivot import PivotCalibration, PivotEstimator, skew
from plumb.trajectory import SAMPLE_RATE_HZ, ArcType, StrokeParams, generate

DT = 1.0 / SAMPLE_RATE_HZ

# The stillness window the pipeline measures noise over (0.5 s at 896.8 Hz).
STILL_SAMPLES = 448


def rigid_body_accel(omega, omega_dot, d):
    """The acceleration a sensor at offset `d` from the pivot actually feels.

    Written out per sample from the parent spec's section 2 relation rather
    than reusing the estimator's own arithmetic -- a test that shares an
    implementation with the code under test proves only self-consistency. The
    estimator works in velocity; this is acceleration, so the two meet only
    through the physics.
    """
    return (np.cross(omega_dot, d)
            + np.cross(omega, np.cross(omega, d)))


def stroke_inputs(params=None, noise_dps=0.0, seed=0):
    """Measured angular velocity through a generated stroke, and the truth."""
    traj = generate(params or StrokeParams())
    measured = traj.omega_true.copy()
    if noise_dps:
        rng = np.random.default_rng(seed)
        measured = measured + np.radians(
            rng.normal(0.0, noise_dps, measured.shape))
    return traj, measured


def measured_noise(noise_dps, seed):
    """The noise covariance as the pipeline would measure it: from a window
    of stillness, not from the parameter that generated it."""
    rng = np.random.default_rng(seed + 1000)
    still = np.radians(rng.normal(0.0, noise_dps, (STILL_SAMPLES, 3)))
    return np.cov(still.T)


def estimate(traj, measured_omega, d_true, first=None, last=None,
             accel_noise=0.0, seed=1, noise_dps=0.0, estimator=None):
    """Feed one stroke through the estimator and return it.

    The acceleration handed in is the TRUE rigid-body acceleration for `d_true`
    -- what a perfect accelerometer on that body would read -- and so is the
    attitude, while the angular rate is the noisy measured one. That split is
    the point: the noise that matters here is the noise in the design matrix.
    """
    first = traj.address_end_index if first is None else first
    last = traj.impact_index if last is None else last
    omega_dot_true = np.gradient(traj.omega_true, DT, axis=0)
    rng = np.random.default_rng(seed)
    est = estimator or PivotEstimator(DT)
    if noise_dps:
        est.set_noise(measured_noise(noise_dps, seed), STILL_SAMPLES)
    reference = quat.conjugate(traj.q_true[first])
    for i in range(first, last):
        a = rigid_body_accel(traj.omega_true[i], omega_dot_true[i], d_true)
        if accel_noise:
            a = a + rng.normal(0.0, accel_noise, 3)
        rotation = quat.to_matrix(quat.multiply(reference, traj.q_true[i]))
        est.update(measured_omega[i], a, rotation)
    return est


D_TRUE = np.array([0.0, 0.0, -0.55])


def test_recovers_the_pivot_offset_from_clean_data():
    """Exact, to the discretisation. The acceleration form this replaced left
    ~1% here, from a low-pass filter it could not apply consistently; the
    velocity form needs no filter (see plumb/pivot.py)."""
    traj, omega = stroke_inputs()
    solution = estimate(traj, omega, D_TRUE).solve()
    assert solution is not None
    assert solution.offset == pytest.approx(D_TRUE, abs=0.002)


def test_recovers_a_different_pivot_offset():
    """A taller golfer, or a longer putter. Nothing may be fitted to 0.55."""
    d = np.array([0.0, 0.0, -0.80])
    params = StrokeParams(pivot_offset_m=0.80)
    traj, omega = stroke_inputs(params)
    solution = estimate(traj, omega, d).solve()
    assert solution.offset == pytest.approx(d, abs=0.002)


def test_the_component_along_the_rotation_axis_is_reported_unobservable():
    """A rotation about an axis tells you nothing about the offset ALONG that
    axis: an offset parallel to omega does not move. The estimator must return
    the minimum-norm answer rather than inventing that component, and must say
    the system was rank-deficient rather than let a caller believe all three
    numbers were measured.

    A real stroke turns about two axes -- the swing and the face rotation --
    so this uses a pure single-axis rotation, where the deficiency is exact.
    """
    d_with_axial = D_TRUE + np.array([0.0, 0.3, 0.0])   # +Y is the only axis
    est = PivotEstimator(DT)
    n = 2240
    t = np.arange(n) * DT
    rate = 3.0 * np.sin(2.0 * np.pi * t / (n * DT))
    rate_dot = np.gradient(rate, DT)
    angle = np.concatenate(([0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * DT)))
    for w, w_dot, theta in zip(rate, rate_dot, angle):
        omega = np.array([0.0, w, 0.0])
        omega_dot = np.array([0.0, w_dot, 0.0])
        est.update(omega, rigid_body_accel(omega, omega_dot, d_with_axial),
                   quat.to_matrix(quat.rot_y(theta)))
    solution = est.solve()
    assert solution.rank == 2
    assert solution.offset[2] == pytest.approx(-0.55, rel=0.01)
    assert abs(solution.offset[1]) < 0.001


def test_a_noise_only_direction_is_not_reported():
    """The same exact deficiency, under noise, with the noise correction on.

    Subtracting the expected noise from a direction that held nothing but
    noise leaves the difference of two similar numbers, and dividing by that
    produced a swing-axis offset of 0.15 to 0.27 m on the straight putter
    before the significance test learned to compare it against the noise
    estimate's own uncertainty. Measured energy there: 0.039 against a noise
    share of 0.040-0.046.
    """
    for seed in range(1, 6):
        traj, omega = stroke_inputs(StrokeParams(arc_type=ArcType.STRAIGHT),
                                    noise_dps=0.28, seed=seed)
        solution = estimate(traj, omega, D_TRUE, noise_dps=0.28, seed=seed).solve()
        assert solution.rank == 2, f"seed {seed}: {solution}"
        # Not exactly zero: under noise the kept eigenvectors are not exactly
        # perpendicular to Y. Measured 9.4e-7 m, against 0.15-0.27 m before.
        assert abs(solution.offset[1]) < 0.001


def test_survives_gyro_noise_at_the_measured_floor():
    """0.28 dps is this board's measured resting floor. One stroke, noise
    correction on, every component against truth -- including the cross-shaft
    ones, which are where an error turns face rotation into fake path."""
    errors = []
    for seed in range(1, 6):
        traj, omega = stroke_inputs(noise_dps=0.28, seed=seed)
        solution = estimate(traj, omega, D_TRUE, noise_dps=0.28, seed=seed).solve()
        errors.append(np.linalg.norm(solution.offset - D_TRUE))
    print(f"\n  one stroke at 0.28 dps, |d - truth|: "
          f"{', '.join(f'{1e3 * e:.1f}' for e in errors)} mm")
    assert max(errors) < 0.03


def test_degrades_gracefully_rather_than_wildly_at_ten_times_the_noise():
    traj, omega = stroke_inputs(noise_dps=2.8, seed=4)
    solution = estimate(traj, omega, D_TRUE, noise_dps=2.8, seed=4).solve()
    assert solution.offset[2] == pytest.approx(-0.55, rel=0.35)


def test_noise_correction_is_what_removes_the_bias():
    """Errors-in-variables, shown directly: the same noisy arced strokes with
    and without the measured noise handed to the fit. Without it, gyro noise
    in the design matrix shrinks the weakest direction toward zero and the
    offset lands off the shaft axis."""
    params = StrokeParams(arc_type=ArcType.ARCED, lie_angle_deg=5.0)
    raw, corrected = PivotCalibration(), PivotCalibration()
    for seed in range(1, 11):
        traj, omega = stroke_inputs(params, noise_dps=0.28, seed=seed)
        raw.fold(estimate(traj, omega, D_TRUE))
        corrected.fold(estimate(traj, omega, D_TRUE, noise_dps=0.28, seed=seed))
    raw_error = np.linalg.norm(raw.solve().offset - D_TRUE)
    corrected_error = np.linalg.norm(corrected.solve().offset - D_TRUE)
    print(f"\n  ten arced strokes: |d - truth| {1e3 * raw_error:.1f} mm uncorrected, "
          f"{1e3 * corrected_error:.1f} mm corrected")
    assert corrected_error < 0.01
    assert raw_error > 3 * corrected_error


def test_the_estimate_barely_depends_on_putter_type():
    """Invariant 1. The pivot is swing geometry: where the golfer's hands are.
    A zero-torque putter and a blade swung the same way have the same pivot,
    and an estimate that moved with face rotation would be reading putter type
    into a quantity that has nothing to do with it."""
    offsets = {}
    for arc in ArcType:
        params = StrokeParams(arc_type=arc)
        traj, omega = stroke_inputs(params)
        offsets[arc.name] = estimate(traj, omega, D_TRUE).solve().offset[2]
    spread = max(offsets.values()) - min(offsets.values())
    assert spread < 0.003, offsets


def test_a_motionless_record_yields_no_estimate():
    """Nothing rotated, so nothing about the pivot is observable. Returning a
    number here would be returning noise with a physical-looking magnitude."""
    est = PivotEstimator(DT)
    for _ in range(500):
        est.update(np.zeros(3), np.zeros(3), np.eye(3))
    assert est.solve() is None


def test_too_few_samples_yields_no_estimate():
    traj, omega = stroke_inputs()
    est = estimate(traj, omega, D_TRUE,
                   first=traj.address_end_index,
                   last=traj.address_end_index + 3)
    assert est.solve() is None


def test_residual_reports_when_the_model_does_not_explain_the_data():
    """Fed acceleration that no rigid rotation about any pivot could produce,
    the fit must say so rather than return its best wrong answer quietly."""
    traj, omega = stroke_inputs()
    rng = np.random.default_rng(5)
    est = PivotEstimator(DT)
    for i in range(traj.address_end_index, traj.impact_index):
        est.update(omega[i], rng.normal(0.0, 5.0, 3), np.eye(3))
    solution = est.solve()
    assert solution is None or solution.residual_fraction > 0.5


def test_residual_is_small_when_the_model_does_explain_the_data():
    traj, omega = stroke_inputs()
    solution = estimate(traj, omega, D_TRUE).solve()
    assert solution.residual_fraction < 0.01


def test_skew_matches_the_cross_product_it_stands_for():
    rng = np.random.default_rng(6)
    a, b = rng.normal(size=3), rng.normal(size=3)
    assert skew(a) @ b == pytest.approx(np.cross(a, b))
