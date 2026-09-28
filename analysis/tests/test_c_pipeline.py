"""Differential test: pipeline.c against plumb/pipeline.py, and against truth.

The last of the C port. The two layers under it are already verified
(test_c_port.py for the quaternions, test_c_pivot.py for the pivot fit), so a
difference here is in the state machine, the stillness window, the track or
the path geometry.

The C is fed the same int16 counts the Python is -- the sensor simulator's
output, the thing the IMU will actually deliver -- and compared at every stage
a disagreement could start from: the address references, each stroke
boundary, the impact attitude, and the four metrics. A boundary one sample
off is invisible in a face angle and obvious as an index, which is why the
indices are compared exactly.

Not bit for bit. NumPy's mean, std and cov use pairwise summation and BLAS,
and the C sums in order, so the address references differ in their last bits
and everything downstream inherits that. The tolerances below are set far
under anything that matters and far above that rounding; the printed worst
differences are the output.

And against GROUND TRUTH, because a test of the C against the Python alone
would pass an error the two share.
"""

from dataclasses import astuple, fields

import numpy as np
import pytest

from plumb.calibration import solve_accel_calibration
from plumb.pipeline import Pipeline, State, Thresholds
from plumb.pivot import PivotCalibration
from plumb.sensor import GRAVITY, SensorParams, simulate
from plumb.trajectory import SAMPLE_RATE_HZ, ArcType, StrokeParams, generate

from tests.test_c_port import _build, fmt, run_c
from tests.test_pipeline import true_face_path

NOISY = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                     gyro_bias_dps=1.5)

DIRECTIONS = {0: "straight", 1: "in-to-out", 2: "out-to-in"}
STATES = list(State)


@pytest.fixture(scope="session")
def portcheck():
    return _build("portcheck.exe")


@pytest.fixture(scope="session")
def portcheck_single():
    return _build("portcheck-single.exe", "PLUMB_SINGLE_PRECISION")


@pytest.fixture(scope="session")
def portcheck_short_track():
    """A ring too short for any real stroke, to see what the C does when a
    stroke outruns it."""
    return _build("portcheck-short.exe", "PL_PIPELINE_TRACK_MAX=256")


def init_line(full_scale, rate=SAMPLE_RATE_HZ, accel_calibration=None,
              calibrated_pivot=False, thresholds=Thresholds()):
    # Thresholds in declaration order: the C struct is field for field.
    assert [f.name for f in fields(Thresholds)][0] == "stillness_window_s"
    offset = np.zeros(3) if accel_calibration is None else accel_calibration.offset_mps2
    gain = np.ones(3) if accel_calibration is None else accel_calibration.gain
    return (f"pipinit {fmt(rate, full_scale.gyro_rad_per_count, full_scale.accel_mps2_per_count, 0.85)} "
            f"{fmt(*astuple(thresholds))} {fmt(offset, gain)} "
            f"{1 if calibrated_pivot else 0}")


def step_lines(out):
    return [f"pipstep {' '.join(str(int(v)) for v in g)} "
            f"{' '.join(str(int(v)) for v in a)}"
            for g, a in zip(out.gyro_counts, out.accel_counts)]


class CStroke:
    """What the C reported for one stroke, parsed from the harness."""

    def __init__(self, rows):
        assert rows[0] == [1.0], f"pipinit refused the config: {rows[0]}"
        results = [r for r in rows[1:] if len(r) == 9]
        states = [r for r in rows[1:] if len(r) == 17]
        assert len(states) == 1 and len(results) <= 1
        self.result = None
        if results:
            r = results[0]
            self.result = dict(face_valid=bool(r[0]), face_angle_deg=r[1],
                               backswing_s=r[2], downswing_s=r[3],
                               tempo_ratio=r[4], path_valid=bool(r[5]),
                               path_arc_m=r[6], path_travel_m=r[7],
                               path_direction=DIRECTIONS[int(r[8])])
        s = states[0]
        self.state = STATES[int(s[0])]
        self.n = int(s[1])
        self.bias, self.g0 = np.array(s[2:5]), np.array(s[5:8])
        self.i_backswing_start, self.i_transition = s[8], s[9]
        self.i_impact, self.i_motion_end = int(s[10]), int(s[11])
        self.q_impact = np.array(s[12:16])
        self.track_first_n = int(s[16])


def run_both(exe, stroke=StrokeParams(), sensor=SensorParams(), seed=1,
             rate=SAMPLE_RATE_HZ, accel_calibration=None):
    traj = generate(stroke)
    out = simulate(traj, sensor, seed=seed)
    pipe = Pipeline(Thresholds(), out.full_scale, sample_rate_hz=rate,
                    accel_calibration=accel_calibration)
    result = None
    for i in range(len(traj.time)):
        result = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or result
    lines = [init_line(out.full_scale, rate, accel_calibration),
             *step_lines(out), "pipstate"]
    return traj, pipe, result, CStroke(run_c(exe, lines))


def grid():
    for arc in ArcType:
        for lie in (5.0, 20.0):
            for face in (-2.0, 3.0):
                for sensor, label in ((SensorParams(), "clean"), (NOISY, "noisy")):
                    yield (f"{arc.name.lower()} lie {lie:.0f} face {face:+.0f} {label}",
                           StrokeParams(arc_type=arc, lie_angle_deg=lie,
                                        face_angle_at_impact_deg=face),
                           sensor)
    for tempo in (1.5, 3.0):
        yield f"tempo {tempo}", StrokeParams(tempo_ratio=tempo), NOISY


class Worst:
    def __init__(self):
        self.values = {}

    def update(self, key, difference):
        self.values[key] = max(self.values.get(key, 0.0), float(difference))

    def report(self, title):
        print(f"\n  {title}")
        for key, value in self.values.items():
            print(f"    {key:<22} {value:.3e}")


def compare(label, pipe, result, c, worst, tight=True):
    """Every stage a disagreement could start from, in order."""
    assert c.state is pipe.state, label
    assert c.n == pipe.n, label
    worst.update("bias (rad/s)", np.abs(c.bias - pipe.bias).max())
    worst.update("g0 (m/s^2)", np.abs(c.g0 - pipe.g0).max())
    worst.update("backswing start", abs(c.i_backswing_start - pipe.i_backswing_start))
    assert c.i_transition == pipe.i_transition, label
    assert c.i_impact == pipe.i_impact, label
    assert c.i_motion_end == pipe.i_motion_end, label
    assert c.track_first_n == pipe._track_first_n, label
    worst.update("q at impact", np.abs(c.q_impact - pipe.q_impact).max())

    r = c.result
    assert r is not None, f"{label}: the C never finished the stroke"
    assert r["face_valid"] and r["path_valid"], label
    worst.update("face angle (deg)", abs(r["face_angle_deg"] - result.face_angle_deg))
    worst.update("backswing (s)", abs(r["backswing_s"] - result.backswing_s))
    worst.update("downswing (s)", abs(r["downswing_s"] - result.downswing_s))
    worst.update("tempo ratio", abs(r["tempo_ratio"] - result.tempo_ratio))
    worst.update("arc (m)", abs(r["path_arc_m"] - result.path_arc_m))
    worst.update("travel (m)", abs(r["path_travel_m"] - result.path_travel_m))
    assert r["path_direction"] == result.path_direction, label


def test_every_stroke_matches_the_python(portcheck):
    worst = Worst()
    for label, stroke, sensor in grid():
        _, pipe, result, c = run_both(portcheck, stroke, sensor)
        compare(label, pipe, result, c, worst)
    worst.report("C against NumPy, double precision, worst over the grid:")
    v = worst.values
    assert v["bias (rad/s)"] < 1e-12 and v["g0 (m/s^2)"] < 1e-12
    assert v["backswing start"] < 1e-6
    assert v["q at impact"] < 1e-10
    assert v["face angle (deg)"] < 1e-8
    assert max(v["backswing (s)"], v["downswing (s)"], v["tempo ratio"]) < 1e-9
    assert max(v["arc (m)"], v["travel (m)"]) < 1e-9


def test_the_c_meets_the_same_ground_truth(portcheck):
    """The criteria test_pipeline.py holds the Python to, applied to the C:
    face angle noiselessly to 0.1 deg, tempo to 0.05, and arc within 10% of
    the true face path over the same window."""
    for face in (-5.0, 0.0, 2.0):
        for tempo in (1.5, 2.5):
            traj, pipe, _, c = run_both(portcheck, StrokeParams(
                face_angle_at_impact_deg=face, tempo_ratio=tempo))
            r = c.result
            assert r["face_angle_deg"] == pytest.approx(face, abs=0.1)
            assert r["tempo_ratio"] == pytest.approx(traj.true_tempo_ratio, abs=0.05)
            arc_true, travel_true = true_face_path(traj, pipe)
            assert 0.90 < r["path_arc_m"] / arc_true < 1.10
            assert r["path_travel_m"] == pytest.approx(travel_true, rel=0.05)


def test_a_calibrated_session_matches_the_python(portcheck):
    """The pivot calibration persists across strokes (parent spec 8.4), so
    the C's session has to carry it exactly as the Python's does -- a fold
    skipped or doubled shows here and nowhere per-stroke."""
    stroke = StrokeParams(lie_angle_deg=5.0)
    python = PivotCalibration()
    lines = ["calreset"]
    expected = []
    for seed in range(1, 9):
        traj = generate(stroke)
        out = simulate(traj, NOISY, seed=seed)
        pipe = Pipeline(Thresholds(), out.full_scale, pivot_calibration=python)
        result = None
        for i in range(len(traj.time)):
            result = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or result
        expected.append(result)
        lines += [init_line(out.full_scale, calibrated_pivot=True),
                  *step_lines(out)]
    rows = [r for r in run_c(portcheck, lines) if len(r) == 9]
    assert len(rows) == len(expected)
    worst = max(abs(row[6] - e.path_arc_m) for row, e in zip(rows, expected))
    print(f"\n  eight-stroke calibrated session, worst arc difference {worst:.3e} m")
    assert worst < 1e-9


def test_the_device_calibration_is_applied_as_the_python_applies_it(portcheck):
    """A biased, mis-scaled accelerometer and its tumble calibration: the
    offset-then-gain order matters, and swapping it would still look close."""
    bias, scale = np.array([0.05, 0.2, -0.1]), np.array([0.01, -0.02, 0.015])
    ups = np.vstack([np.eye(3), -np.eye(3)]) * GRAVITY
    calibration = solve_accel_calibration(ups * (1.0 + scale) + bias, gravity=GRAVITY)
    sensor = SensorParams(gyro_noise_dps=0.28, accel_bias_mps2=bias,
                          accel_scale_error=scale)
    worst = Worst()
    _, pipe, result, c = run_both(portcheck, StrokeParams(lie_angle_deg=5.0),
                                  sensor, accel_calibration=calibration)
    compare("calibrated accelerometer", pipe, result, c, worst)
    assert worst.values["arc (m)"] < 1e-9
    assert worst.values["g0 (m/s^2)"] < 1e-12


def test_the_measured_sample_rate_reaches_the_c(portcheck):
    """This board's 906.86 Hz, handed in as the firmware will hand it in."""
    worst = Worst()
    _, pipe, result, c = run_both(portcheck, StrokeParams(face_angle_at_impact_deg=3.0),
                                  NOISY, rate=906.86)
    compare("906.86 Hz", pipe, result, c, worst)
    assert worst.values["face angle (deg)"] < 1e-8


def test_a_stroke_that_outruns_the_track_reports_no_path(portcheck,
                                                         portcheck_short_track):
    """Path unavailable -- not a number computed from part of the stroke --
    and face angle and tempo exactly as they are with room to spare, because
    neither touches the track."""
    lines = None
    traj = generate(StrokeParams())
    out = simulate(traj, NOISY, seed=1)
    lines = [init_line(out.full_scale), *step_lines(out), "pipstate"]
    full = CStroke(run_c(portcheck, lines)).result
    short = CStroke(run_c(portcheck_short_track, lines)).result
    assert full["path_valid"] and not short["path_valid"]
    assert short["face_valid"]
    assert short["face_angle_deg"] == full["face_angle_deg"]
    assert short["tempo_ratio"] == full["tempo_ratio"]


def test_an_impact_too_long_to_be_a_putt_reports_no_face_angle(portcheck):
    """A 100 ms saturated spike is ~90 samples, past PL_PIPELINE_IMPACT_MAX.
    Its middle attitude was never stored, so face angle is unavailable; the
    impact instant is still known, so tempo is not."""
    traj, pipe, result, c = run_both(portcheck, StrokeParams(),
                                     SensorParams(impact_duration_s=0.1))
    assert pipe.i_impact - traj.impact_index < 60  # the Python still found it
    assert not c.result["face_valid"]
    assert c.result["tempo_ratio"] == pytest.approx(result.tempo_ratio, abs=1e-9)


def test_a_config_the_pipeline_cannot_run_is_refused(portcheck):
    """A stillness window that does not fit the ring is refused at init, not
    silently truncated: a shorter window would be a different threshold."""
    long_window = Thresholds(stillness_window_s=5.0)
    from plumb.sensor import FullScale
    (row,) = run_c(portcheck, [init_line(FullScale(), thresholds=long_window)])
    assert row == [0.0]


def test_single_precision_divergence_is_measured_not_assumed(portcheck_single):
    """The whole pipeline in float, as the device will run it, against the
    float64 reference -- the number real.h left open: rates formed in float,
    and everything the pipeline adds on top of the integrator.

    Loose bounds on purpose; the printed worst differences are the output.
    Indices may legitimately move by a sample in float when a comparison
    lands within rounding of a threshold, so they are reported, not asserted.
    """
    worst = Worst()
    boundaries = []
    directions = []
    for label, stroke, sensor in grid():
        _, pipe, result, c = run_both(portcheck_single, stroke, sensor)
        r = c.result
        assert r is not None and r["face_valid"] and r["path_valid"], label
        if (c.i_transition, c.i_impact, c.i_motion_end) != (
                pipe.i_transition, pipe.i_impact, pipe.i_motion_end):
            boundaries.append(label)
        if r["path_direction"] != result.path_direction:
            directions.append(label)
        worst.update("face angle (deg)", abs(r["face_angle_deg"] - result.face_angle_deg))
        worst.update("tempo ratio", abs(r["tempo_ratio"] - result.tempo_ratio))
        worst.update("arc (fraction)", abs(r["path_arc_m"] / result.path_arc_m - 1.0))
        worst.update("travel (fraction)", abs(r["path_travel_m"] / result.path_travel_m - 1.0))
    worst.report("single precision against float64, worst over the grid:")
    print(f"    boundaries moved: {boundaries or 'none'}")
    print(f"    directions changed: {directions or 'none'}")
    v = worst.values
    assert v["face angle (deg)"] < 0.01
    assert v["tempo ratio"] < 0.01
    assert v["arc (fraction)"] < 0.01
    assert v["face angle (deg)"] > 0.0, "float agreed exactly: the define did not take effect"
