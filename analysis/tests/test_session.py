"""The device loop: strokes one after another, and strokes that never finish.

Parent spec 7.1's state machine only runs forward. A real device sees
practice strokes with no ball, putters picked up after address, waggles and
re-grips, and golfers walking off mid-follow-through. Will's decision,
2026-09-28: a stroke that has not reached IMPACT is abandoned when the putter
comes back to rest (the same stillness test as address), or after a maximum
stroke duration as a backstop.

Every stream here is physically continuous. A practice stroke is generated
with no follow-through and a square face, so it comes to rest exactly at its
own address pose, and the next stroke starts from that pose. Concatenating
two ordinary strokes would teleport the putter between them, and the
accelerometer would see gravity jump -- an artefact of the test, not a stroke.
"""

from dataclasses import replace

import numpy as np
import pytest

from plumb.pipeline import Pipeline, State, Thresholds
from plumb.session import Session
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import StrokeParams, generate

from tests.test_pipeline import sample_index, true_face_speed

NOISY = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                     gyro_bias_dps=1.5)


def practice(sensor=SensorParams(), seed=7, **stroke):
    """A stroke with no ball: no impact spike, and back to rest at address.
    Held long enough after for the rest test to see it."""
    params = StrokeParams(followthrough_amplitude_deg=0.0,
                          face_angle_at_impact_deg=0.0,
                          followthrough_hold_s=1.5, **stroke)
    traj = generate(params)
    out = simulate(traj, replace(sensor, impact_peak_g=0.0), seed=seed)
    return traj, out


def real(sensor=SensorParams(), seed=1, **stroke):
    traj = generate(StrokeParams(**stroke))
    return traj, simulate(traj, sensor, seed=seed)


def run_session(parts, thresholds=Thresholds()):
    """Feed the parts' samples through one Session back to back. Returns the
    session, the results, and each part's first sample index in the stream."""
    session = Session(thresholds, parts[0][1].full_scale)
    results, offsets, at = [], [], 0
    for traj, out in parts:
        offsets.append(at)
        for i in range(len(traj.time)):
            r = session.step(out.gyro_counts[i], out.accel_counts[i])
            if r is not None:
                results.append(r)
        at += len(traj.time)
    return session, results, offsets


def speed_sample(session):
    """Stream index of the sample the last stroke's speed was taken at: the
    pipeline's own sample number, from where in the stream it started."""
    return session.last_pipeline_start + sample_index(session.last_pipeline.i_speed_n)


def assert_matches_truth(result, traj, speed_index):
    assert result.face_angle_deg == pytest.approx(traj.true_face_angle_deg, abs=0.1)
    assert result.tempo_ratio == pytest.approx(traj.true_tempo_ratio, abs=0.05)
    assert result.impact_speed_mps == pytest.approx(
        true_face_speed(traj, speed_index), rel=0.02)


def test_a_practice_stroke_is_abandoned_and_the_putt_after_it_measured():
    """The case that would otherwise freeze the device: a practice stroke
    reaches DOWNSWING, no impact ever comes, and the pipeline waits for one
    forever with the render gate armed."""
    first, second = practice(), real(face_angle_at_impact_deg=2.0)
    session, results, offsets = run_session([first, second])
    assert session.abandoned == [("rest", State.DOWNSWING)]
    assert len(results) == 1
    assert_matches_truth(results[0], second[0], speed_sample(session) - offsets[1])


def test_the_same_under_noise():
    first, second = practice(NOISY, seed=3), real(NOISY, seed=4,
                                                  face_angle_at_impact_deg=-1.5)
    session, results, offsets = run_session([first, second])
    assert [reason for reason, _ in session.abandoned] == ["rest"]
    assert len(results) == 1
    assert_matches_truth(results[0], second[0], speed_sample(session) - offsets[1])


def test_a_waggle_at_address_re_captures_address():
    """Motion too small to confirm a backswing (a waggle, a re-grip), then
    stillness again. The references captured before it are stale: face angle
    is relative to address (invariant 3), and the address that matters is
    the one the stroke started from. So the pipeline re-addresses."""
    waggle = practice(backswing_amplitude_deg=0.5)
    session, results, offsets = run_session([waggle, real(face_angle_at_impact_deg=3.0)])
    assert session.abandoned == [("rest", State.ADDRESS)]
    assert len(results) == 1
    assert results[0].face_angle_deg == pytest.approx(3.0, abs=0.1)


def test_stillness_at_address_alone_does_not_re_address():
    """Standing still over the ball is what ADDRESS is for; only motion and
    then stillness re-captures."""
    traj, out = real(address_duration_s=4.0)
    session, results, _ = run_session([(traj, out)])
    assert session.abandoned == []
    assert len(results) == 1


def test_a_stroke_carried_off_is_abandoned_by_the_timeout():
    """Picked up and never put down: the rest test cannot fire, so the
    timeout does. Forced here with a timeout shorter than the backswing."""
    th = Thresholds(stroke_timeout_s=0.5)
    session, results, _ = run_session([real()], th)
    assert session.abandoned == [("timeout", State.BACKSWING)]
    assert results == []


def test_a_timeout_after_impact_keeps_what_impact_fixed():
    """Past impact the stroke is committed: face angle, tempo and speed are
    all fixed at impact. If the follow-through never goes quiet -- walking
    off still swinging -- the timeout finishes the stroke with those, and
    path, which needs the motion's end, is unavailable rather than measured
    over the walk. Forced with a timeout that lands in the follow-through."""
    traj, out = real()
    backswing_to_impact = (traj.impact_index - traj.address_end_index) / (
        1.0 / (traj.time[1] - traj.time[0]))
    th = Thresholds(stroke_timeout_s=backswing_to_impact + 0.1)
    session, results, _ = run_session([(traj, out)], th)
    assert session.abandoned == []
    (result,) = results
    assert result.path_arc_m is None and result.path_direction is None
    assert result.path_travel_m is None
    assert_matches_truth(result, traj, speed_sample(session))


def test_the_pivot_calibration_carries_across_strokes():
    """Parent spec 8.4: the per-golfer pivot is learned over a session, so
    the session carries one calibration across every pipeline it starts --
    and an abandoned stroke adds nothing to it, having no impact to bound
    its fit."""
    parts = [real(NOISY, seed=s, lie_angle_deg=5.0) for s in range(1, 5)]
    session, results, _ = run_session(parts)
    assert len(results) == 4
    assert session.pivot_calibration.strokes == 4

    session, _, _ = run_session([practice(), *parts[:1]])
    assert session.pivot_calibration.strokes == 1


def test_a_normal_stroke_is_untouched_by_any_of_this():
    """The new exits must not fire on a real stroke: same result as a lone
    Pipeline, across the putter grid."""
    for stroke in (StrokeParams(), StrokeParams(tempo_ratio=1.5,
                                                backswing_duration_s=1.0)):
        traj = generate(stroke)
        out = simulate(traj, NOISY, seed=2)
        pipe = Pipeline(Thresholds(), out.full_scale)
        lone = None
        for i in range(len(traj.time)):
            lone = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or lone
        session, results, _ = run_session([(traj, out)])
        assert session.abandoned == []
        assert results == [lone]
