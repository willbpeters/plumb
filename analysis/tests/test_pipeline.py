import numpy as np
import pytest

from plumb.pipeline import Pipeline, State, Thresholds
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import ArcType, StrokeParams, generate


def run_stroke(stroke: StrokeParams, sensor: SensorParams = SensorParams(), seed: int = 1):
    traj = generate(stroke)
    out = simulate(traj, sensor, seed=seed)
    pipe = Pipeline(Thresholds(), out.full_scale)
    result = None
    for i in range(len(traj.time)):
        r = pipe.step(out.gyro_counts[i], out.accel_counts[i])
        if r is not None:
            result = r
    return traj, pipe, result


def test_reaches_address_and_captures_gravity():
    traj, pipe, _ = run_stroke(StrokeParams(lie_angle_deg=20.0))
    assert pipe.address_captured
    lie = np.radians(20.0)
    expected = np.array([0.0, np.sin(lie), np.cos(lie)])
    measured = pipe.g0 / np.linalg.norm(pipe.g0)
    np.testing.assert_allclose(measured, expected, atol=1e-2)


def test_gyro_bias_is_nulled_at_address():
    """Invariant 4. A 1.5 dps standing bias must be estimated and removed."""
    _, pipe, _ = run_stroke(StrokeParams(), SensorParams(gyro_bias_dps=1.5))
    np.testing.assert_allclose(np.degrees(pipe.bias), [1.5, 1.5, 1.5], atol=0.1)


def test_backswing_is_detected():
    """BACKSWING was the terminal state before Task 6 wired up the rest of the
    state machine; now it is a mid-sequence transition, confirmed in full by
    test_state_machine_visits_every_state_in_order below. This checks only the
    prefix that was this test's original intent."""
    _, pipe, _ = run_stroke(StrokeParams())
    assert pipe.visited[:3] == [State.IDLE, State.ADDRESS, State.BACKSWING]


def test_state_machine_visits_every_state_in_order():
    _, pipe, _ = run_stroke(StrokeParams())
    order = [State.IDLE, State.ADDRESS, State.BACKSWING, State.DOWNSWING,
             State.IMPACT, State.FOLLOWTHROUGH, State.DONE]
    assert pipe.visited == order


@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_tempo_ratio_recovered(tempo):
    """Compared against the ratio the generator actually produced, not the one
    requested -- the 500 Hz grid cannot hit an arbitrary ratio exactly, and
    holding the pipeline to a target the stroke never contained would be
    measuring the generator's rounding, not the pipeline."""
    traj, _, result = run_stroke(StrokeParams(tempo_ratio=tempo))
    assert result is not None
    assert result.tempo_ratio == pytest.approx(traj.true_tempo_ratio, abs=0.05)


@pytest.mark.parametrize("face_angle", [-5.0, -2.0, 0.0, 1.0, 2.0, 5.0])
def test_face_angle_recovered_noiselessly(face_angle):
    """At zero noise and zero bias the pipeline is a pure algebraic inverse of
    the forward model. Any error here is a sign error, a frame mix-up or a
    quaternion convention mismatch."""
    _, _, result = run_stroke(StrokeParams(face_angle_at_impact_deg=face_angle))
    assert result is not None
    assert result.face_angle_deg == pytest.approx(face_angle, abs=0.1)


@pytest.mark.parametrize("arc", list(ArcType))
def test_face_angle_recovered_for_every_arc_type(arc):
    """Invariant 1. A zero-torque putter produces very little face rotation and
    must recover exactly as well as an arced one."""
    _, _, result = run_stroke(
        StrokeParams(face_angle_at_impact_deg=2.0, arc_type=arc))
    assert result.face_angle_deg == pytest.approx(2.0, abs=0.1)


def test_face_angle_does_not_depend_on_stroke_size():
    """A longer backswing delivering the same face angle must report the same
    number. Face angle is an attitude difference between two instants, so
    nothing about the size of the motion between them should enter it.

    This is also a weak check on invariant 3: attitude is integrated from
    identity at address, so the reported angle is address-relative by
    construction and no absolute heading can leak in. There is no heading
    parameter in the generator to vary, because the device has no heading
    reference to be wrong about."""
    a = run_stroke(StrokeParams(face_angle_at_impact_deg=2.0))[2]
    b = run_stroke(StrokeParams(face_angle_at_impact_deg=2.0,
                                backswing_amplitude_deg=15.0,
                                followthrough_amplitude_deg=15.0))[2]
    assert a.face_angle_deg == pytest.approx(b.face_angle_deg, abs=0.1)
