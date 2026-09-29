"""Differential test: the C session loop and the pipeline's exits, against
plumb/session.py and plumb/pipeline.py, and against ground truth.

The same continuous streams test_session.py builds -- a practice stroke with
no impact followed by a real putt, a waggle at address, a stroke carried off
-- fed as int16 counts through the Python Session and the C session. What
must agree: which strokes were abandoned, why, and from which state; and
every result that was produced.
"""

from dataclasses import astuple

import numpy as np
import pytest

from plumb.pipeline import State, Thresholds
from plumb.session import Session
from plumb.sensor import SensorParams
from plumb.trajectory import SAMPLE_RATE_HZ

from tests.test_c_pipeline import (DIRECTIONS, RESULT_FIELDS, portcheck,  # noqa: F401
                                   portcheck_single)  # noqa: F401
from tests.test_c_port import fmt, run_c
from tests.test_pipeline import sample_index, true_face_speed
from tests.test_session import NOISY, practice, real

STATES = list(State)
REASONS = {1: "rest", 2: "timeout"}
ABANDON_FIELDS = 3


def session_init_line(full_scale, thresholds=Thresholds()):
    return (f"sesinit {fmt(SAMPLE_RATE_HZ, full_scale.gyro_rad_per_count, full_scale.accel_mps2_per_count, 0.85)} "
            f"{fmt(*astuple(thresholds))} {fmt(np.zeros(3), np.ones(3))}")


def both(exe, parts, thresholds=Thresholds()):
    """Python Session and C session over the same stream. Returns the Python
    session, its results, and the C's abandonments and results."""
    full_scale = parts[0][1].full_scale
    session = Session(thresholds, full_scale)
    py_results = []
    lines = [session_init_line(full_scale, thresholds)]
    for traj, out in parts:
        for g, a in zip(out.gyro_counts, out.accel_counts):
            r = session.step(g, a)
            if r is not None:
                py_results.append(r)
            lines.append(f"sesstep {' '.join(str(int(v)) for v in g)} "
                         f"{' '.join(str(int(v)) for v in a)}")
    rows = run_c(exe, lines)
    assert rows[0] == [1.0], "sesinit refused the config"
    c_abandoned = [(REASONS[int(r[1])], STATES[int(r[2])])
                   for r in rows[1:] if len(r) == ABANDON_FIELDS]
    c_results = [r for r in rows[1:] if len(r) == RESULT_FIELDS]
    return session, py_results, c_abandoned, c_results


def assert_results_agree(py_results, c_results, tolerance):
    assert len(c_results) == len(py_results)
    worst = 0.0
    for py, c in zip(py_results, c_results):
        assert bool(c[0])
        worst = max(worst, abs(c[1] - py.face_angle_deg))
        assert c[4] == pytest.approx(py.tempo_ratio, abs=tolerance)
        assert bool(c[5]) == (py.path_arc_m is not None)
        if py.path_arc_m is not None:
            assert c[6] == pytest.approx(py.path_arc_m, abs=tolerance)
            assert DIRECTIONS[int(c[8])] == py.path_direction
        assert bool(c[9]) == (py.impact_speed_mps is not None)
        if py.impact_speed_mps is not None:
            assert c[10] == pytest.approx(py.impact_speed_mps, abs=tolerance)
    assert worst <= tolerance
    return worst


def test_a_practice_stroke_then_a_putt(portcheck):
    for sensor, seeds in ((SensorParams(), (7, 1)), (NOISY, (3, 4))):
        parts = [practice(sensor, seed=seeds[0]),
                 real(sensor, seed=seeds[1], face_angle_at_impact_deg=2.0)]
        session, py, c_abandoned, c_results = both(portcheck, parts)
        assert c_abandoned == session.abandoned == [("rest", State.DOWNSWING)]
        assert_results_agree(py, c_results, 1e-9)
        # And against truth: the C's putt, not just the C's agreement.
        assert c_results[0][1] == pytest.approx(2.0, abs=0.1)


def test_a_waggle_at_address(portcheck):
    parts = [practice(backswing_amplitude_deg=0.5),
             real(face_angle_at_impact_deg=3.0)]
    session, py, c_abandoned, c_results = both(portcheck, parts)
    assert c_abandoned == session.abandoned == [("rest", State.ADDRESS)]
    assert_results_agree(py, c_results, 1e-9)
    assert c_results[0][1] == pytest.approx(3.0, abs=0.1)


def test_timeouts_before_and_after_impact(portcheck):
    traj, out = real()
    impact_s = (traj.impact_index - traj.address_end_index) * (traj.time[1] - traj.time[0])
    for timeout, expected_abandoned, expected_results in (
            (0.5, [("timeout", State.BACKSWING)], 0),
            (impact_s + 0.1, [], 1)):
        session, py, c_abandoned, c_results = both(
            portcheck, [(traj, out)], Thresholds(stroke_timeout_s=timeout))
        assert c_abandoned == session.abandoned == expected_abandoned
        assert len(c_results) == expected_results
        assert_results_agree(py, c_results, 1e-9)
        if expected_results:
            assert not c_results[0][5], "path must be unavailable after a timeout"
            speed_at = sample_index(session.last_pipeline.i_speed_n)
            assert c_results[0][10] == pytest.approx(true_face_speed(traj, speed_at),
                                                     rel=0.005)


def test_a_session_of_putts_carries_the_pivot(portcheck):
    """Four putts and a practice stroke between them: every result agrees,
    which it can only do if the C folds the same strokes into the same
    calibration in the same order."""
    parts = [real(NOISY, seed=1, lie_angle_deg=5.0),
             practice(NOISY, seed=9),
             *(real(NOISY, seed=s, lie_angle_deg=5.0) for s in (2, 3, 4))]
    session, py, c_abandoned, c_results = both(portcheck, parts)
    assert c_abandoned == session.abandoned
    worst = assert_results_agree(py, c_results, 1e-9)
    print(f"\n  five-part session: {len(c_results)} putts, worst face difference {worst:.1e} deg")


def test_single_precision_session(portcheck_single):
    """The same practice-then-putt stream in float: the same decisions, and
    the results within the single-precision bounds test_c_pipeline measured."""
    parts = [practice(NOISY, seed=3), real(NOISY, seed=4, face_angle_at_impact_deg=2.0)]
    session, py, c_abandoned, c_results = both(portcheck_single, parts)
    assert c_abandoned == session.abandoned
    worst = assert_results_agree(py, c_results, 0.01)
    print(f"\n  float session: worst face difference {worst:.1e} deg")
