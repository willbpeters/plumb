"""Tests for pivot-offset estimation.

The pivot offset is the missing third of the path measurement. The pipeline
computes face velocity as `omega x r`, which assumes the sensor does not
translate; it does, because the putter swings about the hands rather than about
the sensor, so the face travels on `r + d` where `d` runs from the pivot to the
sensor. With the harness's defaults that is 1.4 m against the 0.85 m assumed,
and 0.85/1.4 is the 61% shortfall that has been pinned in test_pipeline.py.

Ground truth is known here because these tests generate it.
"""

import numpy as np
import pytest

from plumb.pivot import PivotEstimator, skew
from plumb.trajectory import SAMPLE_RATE_HZ, ArcType, StrokeParams, generate

DT = 1.0 / SAMPLE_RATE_HZ


def rigid_body_accel(omega, omega_dot, d):
    """The acceleration a sensor at offset `d` from the pivot actually feels.

    Written out per sample from the parent spec's section 2 relation rather
    than reusing the estimator's own matrix -- a test that shares an
    implementation with the code under test proves only self-consistency.
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


def estimate(traj, measured_omega, d_true, first=None, last=None,
             accel_noise=0.0, seed=1, **kwargs):
    """Feed one stroke through the estimator and return it.

    The acceleration handed in is the TRUE rigid-body acceleration for `d_true`
    -- what a perfect accelerometer on that body would read -- while the
    angular rate is the noisy measured one. That split is the point: the noise
    that matters here is the noise in the design matrix.
    """
    first = traj.address_end_index if first is None else first
    last = traj.impact_index if last is None else last
    omega_dot_true = np.gradient(traj.omega_true, DT, axis=0)
    rng = np.random.default_rng(seed)
    est = PivotEstimator(DT, **kwargs)
    for i in range(first, last):
        a = rigid_body_accel(traj.omega_true[i], omega_dot_true[i], d_true)
        if accel_noise:
            a = a + rng.normal(0.0, accel_noise, 3)
        est.update(measured_omega[i], a)
    return est


D_TRUE = np.array([0.0, 0.0, -0.55])


def test_recovers_the_pivot_offset_from_clean_data():
    """1% on noiseless data, and the 1% is understood rather than tolerated.

    The design matrix is quadratic in the angular rate, so low-passing the rate
    is not the same operation as low-passing the equation, and the difference
    is a small bias that depends on the signal's spectrum. Filtering both sides
    removes the first-order part of it -- without that it is 1.5% -- and what
    is left is second order. It buys the noise performance the next tests
    check, which is worth far more than the 1%.
    """
    traj, omega = stroke_inputs()
    solution = estimate(traj, omega, D_TRUE).solve()
    assert solution is not None
    assert solution.offset[2] == pytest.approx(-0.55, rel=0.02)


def test_recovers_a_different_pivot_offset():
    """A taller golfer, or a longer putter. Nothing may be fitted to 0.55."""
    d = np.array([0.0, 0.0, -0.80])
    params = StrokeParams(pivot_offset_m=0.80)
    traj, omega = stroke_inputs(params)
    solution = estimate(traj, omega, d).solve()
    assert solution.offset[2] == pytest.approx(-0.80, rel=0.02)


def test_the_component_along_the_rotation_axis_is_reported_unobservable():
    """A rotation about an axis tells you nothing about the offset ALONG that
    axis: an offset parallel to omega produces no acceleration at all. The
    estimator must return the minimum-norm answer rather than inventing that
    component, and must say the system was rank-deficient rather than let a
    caller believe all three numbers were measured.

    A real stroke turns about two axes -- the swing and the face rotation --
    so this uses a pure single-axis rotation, where the deficiency is exact.
    It runs over 2.5 s rather than 1 s so its 0.4 Hz fundamental sits inside
    the fit's filter passband, like a real stroke's does; at 1 Hz the filter
    corner biases the magnitude by 10% and obscures what is being tested.
    """
    d_with_axial = D_TRUE + np.array([0.0, 0.3, 0.0])   # +Y is the only axis
    est = PivotEstimator(DT)
    n = 2240
    t = np.arange(n) * DT
    rate = 3.0 * np.sin(2.0 * np.pi * t / (n * DT))
    rate_dot = np.gradient(rate, DT)
    for w, w_dot in zip(rate, rate_dot):
        omega = np.array([0.0, w, 0.0])
        omega_dot = np.array([0.0, w_dot, 0.0])
        est.update(omega, rigid_body_accel(omega, omega_dot, d_with_axial))
    solution = est.solve()
    assert solution.rank == 2
    assert solution.offset[2] == pytest.approx(-0.55, rel=0.05)
    assert abs(solution.offset[1]) < 0.01


def test_survives_gyro_noise_at_the_measured_floor():
    """0.28 dps is this board's measured resting floor. The estimate has to
    hold there, because averaging over the stroke is the whole reason this is
    a least-squares fit rather than the per-sample differentiation that was
    tried and abandoned."""
    traj, omega = stroke_inputs(noise_dps=0.28, seed=3)
    solution = estimate(traj, omega, D_TRUE).solve()
    # Measured 2.1%. Through to the arc that is 2.1% x 0.55/1.4 = 0.8%, against
    # the 10% the parent spec asks of the path metric.
    assert solution.offset[2] == pytest.approx(-0.55, rel=0.05)


def test_degrades_gracefully_rather_than_wildly_at_ten_times_the_noise():
    traj, omega = stroke_inputs(noise_dps=2.8, seed=4)
    solution = estimate(traj, omega, D_TRUE).solve()
    assert solution.offset[2] == pytest.approx(-0.55, rel=0.35)


def test_the_estimate_barely_depends_on_putter_type():
    """Invariant 1. The pivot is swing geometry: where the golfer's hands are.
    A zero-torque putter and a blade swung the same way have the same pivot,
    and an estimate that moved with face rotation would be reading putter type
    into a quantity that has nothing to do with it.

    Measured spread is 6.3 mm on a 550 mm offset, 1.1%, and it comes from the
    filter bias above rather than from anything keyed to rotation -- the rate's
    spectrum differs between the three, so the residual bias does too. The
    consequence for the reported arc is measured in test_pipeline.py, where it
    is 0.4 mm on a 24 mm arc.
    """
    offsets = {}
    for arc in ArcType:
        params = StrokeParams(arc_type=arc)
        traj, omega = stroke_inputs(params)
        offsets[arc.name] = estimate(traj, omega, D_TRUE).solve().offset[2]
    spread = max(offsets.values()) - min(offsets.values())
    assert spread < 0.01, offsets


def test_a_motionless_record_yields_no_estimate():
    """Nothing rotated, so nothing about the pivot is observable. Returning a
    number here would be returning noise with a physical-looking magnitude."""
    est = PivotEstimator(DT)
    for _ in range(500):
        est.update(np.zeros(3), np.zeros(3))
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
        est.update(omega[i], rng.normal(0.0, 5.0, 3))
    solution = est.solve()
    assert solution.residual_fraction > 0.5


def test_residual_is_small_when_the_model_does_explain_the_data():
    """Small, not zero: the filter bias leaves about 1% of the acceleration
    unexplained even on perfect data. What matters is the two orders of
    magnitude between this and the unexplainable case above."""
    traj, omega = stroke_inputs()
    solution = estimate(traj, omega, D_TRUE).solve()
    assert solution.residual_fraction < 0.05


def test_skew_matches_the_cross_product_it_stands_for():
    rng = np.random.default_rng(6)
    a, b = rng.normal(size=3), rng.normal(size=3)
    assert skew(a) @ b == pytest.approx(np.cross(a, b))
