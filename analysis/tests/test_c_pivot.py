"""Differential test: pivot.c against plumb/pivot.py, and both against truth.

The same re-verification test_c_port.py does for the quaternion layer, for the
pivot-offset fit. Two kinds of comparison, because they catch different
things:

  - AGAINST THE PYTHON, on the exact inputs the pipeline hands the estimator
    during simulated strokes. Catches a faithful-looking translation of the
    wrong formula.
  - AGAINST GROUND TRUTH, on the rigid-body inputs test_pivot.py generates.
    Catches an error the C and the Python share -- a test of the C against the
    Python alone would pass that.

The accumulation is a literal translation and is compared tightly. The solve
is not: NumPy's eigh is LAPACK, and the C uses Jacobi rotations, so the
eigenvectors differ in their last bits (and possibly their signs, which the
offset does not depend on). What must agree is everything the eigenvectors
are used for -- the offset, its rank and its residual.
"""

import numpy as np
import pytest

from plumb import quat
from plumb.pivot import PivotCalibration, PivotEstimator
from plumb.trajectory import ArcType, StrokeParams

from tests.test_c_port import _build, fmt, run_c
from tests.test_pivot import (D_TRUE, DT, estimate, rigid_body_accel,
                              stroke_inputs)


@pytest.fixture(scope="session")
def portcheck():
    return _build("portcheck.exe")


@pytest.fixture(scope="session")
def portcheck_single():
    return _build("portcheck-single.exe", "PLUMB_SINGLE_PRECISION")


class RecordingEstimator(PivotEstimator):
    """The Python estimator, writing down every call as a harness line, so
    the C sees exactly what the Python saw -- nothing reconstructed."""

    def __init__(self, dt: float) -> None:
        super().__init__(dt)
        self.lines = [f"pivreset {fmt(dt)}"]

    def set_noise(self, covariance, samples: int) -> None:
        super().set_noise(covariance, samples)
        self.lines.append(
            f"pivnoise {fmt(np.asarray(covariance, dtype=float).ravel())} "
            f"{int(samples)}")

    def update(self, omega, a_body, rotation) -> None:
        super().update(omega, a_body, rotation)
        self.lines.append(
            f"pivupdate {fmt(omega, a_body, np.asarray(rotation).ravel())}")


def pipeline_strokes():
    """The estimator exactly as the pipeline drives it, on simulated strokes:
    real bias subtraction, real gravity removal, the pipeline's own attitude.
    Every putter type at two lies, noiseless and at the board's measured
    floor."""
    from plumb import pipeline as pipeline_module
    from plumb.pipeline import Pipeline, Thresholds
    from plumb.sensor import SensorParams, simulate
    from plumb.trajectory import generate

    noisy = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                         gyro_bias_dps=1.5)
    strokes = []
    original = pipeline_module.PivotEstimator
    pipeline_module.PivotEstimator = RecordingEstimator
    try:
        for arc in ArcType:
            for lie in (5.0, 20.0):
                for sensor in (SensorParams(), noisy):
                    traj = generate(StrokeParams(arc_type=arc,
                                                 lie_angle_deg=lie))
                    out = simulate(traj, sensor, seed=3)
                    pipe = Pipeline(Thresholds(), out.full_scale)
                    for i in range(len(traj.time)):
                        pipe.step(out.gyro_counts[i], out.accel_counts[i])
                    noise = "noisy" if sensor.gyro_noise_dps else "clean"
                    strokes.append((f"{arc.name.lower()} lie {lie:.0f} {noise}",
                                    pipe._pivot))
    finally:
        pipeline_module.PivotEstimator = original
    return strokes


@pytest.fixture(scope="module")
def strokes():
    return pipeline_strokes()


def c_solve(exe, lines):
    """The C's answer: None, or (offset, rank, residual_fraction, samples)."""
    (row,) = run_c(exe, [*lines, "pivsolve"])
    if row[0] == 0:
        return None
    return np.array(row[1:4]), int(row[4]), row[5], int(row[6])


def normal_equation_arrays(eq):
    return [eq.ata.ravel(), eq.atb, [eq.btb], [eq.samples], eq.noise.ravel(),
            [eq.noise_variance]]


def test_normal_equations_match_the_python(portcheck, strokes):
    """The accumulation, compared term by term, relative to each term's own
    magnitude. A literal translation, so this is tight."""
    worst = 0.0
    for label, est in strokes:
        (row,) = run_c(portcheck, [*est.lines, "pivnormal"])
        c = np.array(row)
        at = 0
        for expected in normal_equation_arrays(est.normal_equations()):
            expected = np.asarray(expected, dtype=float)
            produced = c[at:at + expected.size]
            at += expected.size
            scale = max(np.abs(expected).max(), 1e-300)
            worst = max(worst, float(np.abs(produced - expected).max() / scale))
        assert at == c.size, f"{label}: harness printed {c.size} values"
    print(f"\n  normal equations, worst relative difference {worst:.3e}")
    assert worst < 1e-12


def test_solution_matches_the_python(portcheck, strokes):
    """The offset the path uses, its rank and its residual -- everything the
    eigenvectors feed. The rank is compared exactly: a direction kept by one
    and dropped by the other is a disagreement about what was measured."""
    worst = 0.0
    for label, est in strokes:
        expected = est.solve()
        produced = c_solve(portcheck, est.lines)
        assert (produced is None) == (expected is None), label
        if expected is None:
            continue
        offset, rank, residual, samples = produced
        assert rank == expected.rank, label
        assert samples == expected.samples, label
        assert residual == pytest.approx(expected.residual_fraction,
                                         rel=1e-9, abs=1e-12), label
        worst = max(worst, float(np.abs(offset - expected.offset).max()))
    print(f"\n  offset, worst difference from NumPy {worst:.3e} m")
    assert worst < 1e-9


def test_recovers_the_pivot_offset_from_clean_data(portcheck):
    """Against ground truth, the criterion test_pivot.py holds the Python to."""
    for d, params in ((D_TRUE, StrokeParams()),
                      (np.array([0.0, 0.0, -0.80]),
                       StrokeParams(pivot_offset_m=0.80))):
        traj, omega = stroke_inputs(params)
        est = estimate(traj, omega, d, estimator=RecordingEstimator(DT))
        offset, _, residual, _ = c_solve(portcheck, est.lines)
        assert offset == pytest.approx(d, abs=0.002)
        assert residual < 0.01


def test_the_component_along_the_rotation_axis_is_reported_unobservable(
        portcheck):
    """A pure single-axis swing: the offset along the axis produces no motion,
    so the C must return rank 2 and leave that component at zero, not invent
    it. This is the case where a hand-rolled eigensolver's handling of a
    zero eigenvalue shows."""
    d_with_axial = D_TRUE + np.array([0.0, 0.3, 0.0])
    est = RecordingEstimator(DT)
    n = 2240
    t = np.arange(n) * DT
    rate = 3.0 * np.sin(2.0 * np.pi * t / (n * DT))
    rate_dot = np.gradient(rate, DT)
    angle = np.concatenate(([0.0],
                            np.cumsum(0.5 * (rate[1:] + rate[:-1]) * DT)))
    for w, w_dot, theta in zip(rate, rate_dot, angle):
        omega = np.array([0.0, w, 0.0])
        omega_dot = np.array([0.0, w_dot, 0.0])
        est.update(omega, rigid_body_accel(omega, omega_dot, d_with_axial),
                   quat.to_matrix(quat.rot_y(theta)))
    offset, rank, _, _ = c_solve(portcheck, est.lines)
    assert rank == 2
    assert offset[2] == pytest.approx(-0.55, rel=0.01)
    assert abs(offset[1]) < 0.001


def test_a_noise_only_direction_is_not_reported(portcheck):
    """The straight putter's swing axis under noise, noise correction on: the
    significance test has to reject it in the C as it does in the Python."""
    for seed in range(1, 6):
        traj, omega = stroke_inputs(StrokeParams(arc_type=ArcType.STRAIGHT),
                                    noise_dps=0.28, seed=seed)
        est = estimate(traj, omega, D_TRUE, noise_dps=0.28, seed=seed,
                       estimator=RecordingEstimator(DT))
        offset, rank, _, _ = c_solve(portcheck, est.lines)
        assert rank == 2, f"seed {seed}"
        assert abs(offset[1]) < 0.001


def test_a_motionless_record_yields_no_estimate(portcheck):
    est = RecordingEstimator(DT)
    for _ in range(500):
        est.update(np.zeros(3), np.zeros(3), np.eye(3))
    assert c_solve(portcheck, est.lines) is None


def test_too_few_samples_yields_no_estimate(portcheck):
    traj, omega = stroke_inputs()
    est = estimate(traj, omega, D_TRUE, first=traj.address_end_index,
                   last=traj.address_end_index + 3,
                   estimator=RecordingEstimator(DT))
    assert c_solve(portcheck, est.lines) is None


def calibration_session(exe, session):
    lines = ["calreset"]
    for est in session:
        lines += [*est.lines, "calfold"]
    (row,) = run_c(exe, [*lines, "calsolve"])
    return row


def arced_session():
    """Ten noisy arced strokes at lie 5, the hardest case for the pivot."""
    params = StrokeParams(arc_type=ArcType.ARCED, lie_angle_deg=5.0)
    session = []
    for seed in range(1, 11):
        traj, omega = stroke_inputs(params, noise_dps=0.28, seed=seed)
        session.append(estimate(traj, omega, D_TRUE, noise_dps=0.28,
                                seed=seed, estimator=RecordingEstimator(DT)))
    return session


def python_session(session):
    calibration = PivotCalibration()
    for est in session:
        calibration.fold(est)
    return calibration.solve()


def test_a_calibrated_session_matches_the_python_and_the_truth(portcheck):
    """Parent spec 8.4's per-golfer calibration: normal equations summed
    across strokes. Against the Python, and against the pivot that generated
    the strokes."""
    session = arced_session()
    expected = python_session(session)
    row = calibration_session(portcheck, session)
    assert row[0] == 1
    offset = np.array(row[1:4])
    assert int(row[4]) == expected.rank
    assert np.abs(offset - expected.offset).max() < 1e-9
    print(f"\n  ten arced strokes, C: |d - truth| "
          f"{1e3 * np.linalg.norm(offset - D_TRUE):.2f} mm")
    assert np.linalg.norm(offset - D_TRUE) < 0.01


def test_single_precision_divergence_is_measured_not_assumed(
        portcheck_single, strokes):
    """What float costs the pivot, as a number.

    pivot.py's port note warned that centring from running sums cancels, and
    that in single precision the weakest eigenvalue (~0.1 against sums of
    ~230) keeps about four digits. Measured before this port, with NumPy in
    float32 on these same pipeline inputs: at most 0.17 mm on the arced
    putter's offset for running sums, 0.10 mm for Welford's running means,
    and nothing measurable on the other two putters -- against a 0.55 m
    offset. Not worth departing from the arithmetic the Python was proven
    with; this test keeps the number honest now that it is the real C.

    Loose bound on purpose, like test_c_port's: it catches a build that is
    broken, and the printed number is the output.
    """
    worst = 0.0
    for label, est in strokes:
        expected = est.solve()
        produced = c_solve(portcheck_single, est.lines)
        assert (produced is None) == (expected is None), label
        if expected is None:
            continue
        offset, rank, _, _ = produced
        assert rank == expected.rank, label
        worst = max(worst, float(np.abs(offset - expected.offset).max()))

    session = arced_session()
    row = calibration_session(portcheck_single, session)
    session_worst = float(np.abs(np.array(row[1:4])
                                 - python_session(session).offset).max())

    print(f"\n  single precision: worst offset difference {1e3 * worst:.3f} mm "
          f"per stroke, {1e3 * session_worst:.3f} mm over a ten-stroke session")
    assert worst < 1e-3
    assert session_worst < 1e-3
    assert worst > 0.0, "float agreed exactly: the define did not take effect"
