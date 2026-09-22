import numpy as np
import pytest

from plumb import quat
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


def test_recovery_quality_does_not_depend_on_arc_type():
    """Invariant 1, stated quantitatively rather than as a tolerance.

    It is not enough that every arc type lands inside tolerance. The ERROR
    itself must not track arc gain -- if it does, the algorithm contains a
    putter-type prior that a loose tolerance is merely hiding, and it will grow
    on real strokes that rotate faster than these.

    This test is the reason the impact instant is taken at the middle of the
    acceleration spike rather than at its leading edge. With the leading edge
    the spread is 0.0356 deg; with the midpoint it is 0.0004 deg."""
    errors = {}
    for arc in ArcType:
        _, _, result = run_stroke(
            StrokeParams(face_angle_at_impact_deg=2.0, arc_type=arc))
        errors[arc.name] = result.face_angle_deg - 2.0
    spread = max(errors.values()) - min(errors.values())
    assert spread < 0.005, f"recovery error varies with arc type: {errors}"


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


def test_path_direction_classified():
    _, _, result = run_stroke(StrokeParams(arc_type=ArcType.ARCED))
    assert result.path_direction in {"straight", "in-to-out", "out-to-in"}
    assert result.path_arc_m > 0.0


def test_vertical_shaft_traces_a_straight_path():
    """The arc comes from the swing axis being tilted by the lie angle, so the
    head travels on a cone whose ground-plane projection curves. Remove the
    tilt and the cone degenerates to a plane: the path must go straight.

    This is the test that proves the arc is real geometry rather than
    accumulated integration error, because error would not vanish here."""
    _, _, result = run_stroke(StrokeParams(lie_angle_deg=0.0))
    assert result.path_arc_m < 1e-4
    assert result.path_direction == "straight"


@pytest.mark.parametrize("lie,expected_mm", [(5.0, 3.6), (10.0, 7.3), (20.0, 14.3)])
def test_path_arc_grows_with_lie_angle(lie, expected_mm):
    """A flatter lie swings the head on a more tilted cone and arcs more. The
    expected values are measured, and they are close to linear in the lie angle
    over this range, which is what the small-angle geometry predicts."""
    _, _, result = run_stroke(StrokeParams(lie_angle_deg=lie))
    assert result.path_arc_m * 1000 == pytest.approx(expected_mm, abs=0.2)


def true_face_path(traj):
    """The face point's real ground-plane track, straight from the generator.

    The face is rigidly attached (pivot_offset + lever_arm) below the pivot along
    the shaft, so its world position is just the true attitude applied to that
    offset. This is the ground truth the path tests were missing: every other
    path test in this file checks the pipeline against ITSELF -- straight shaft
    gives a straight line, arc grows with lie angle, arc ignores putter type --
    and all of them pass with the error below present.
    """
    p = traj.params
    face_body = np.array([0.0, 0.0, -(p.pivot_offset_m + p.lever_arm_m)])
    lo, hi = traj.address_end_index, traj.impact_index + 200
    world = np.array([quat.rotate(q, face_body) for q in traj.q_true[lo:hi]])
    up = np.array([0.0, 0.0, 1.0])
    flat = world - np.outer(world @ up, up)
    flat = flat - flat[0]
    travel = flat[-1] - flat[0]
    forward = travel / np.linalg.norm(travel)
    lateral = flat @ np.cross(up, forward)
    return float(np.ptp(lateral)), float(np.linalg.norm(travel))


def test_path_direction_matches_the_true_face_path():
    """Direction is what the screen shows and what parent spec section 3 holds
    to 95% agreement. It survives the scale error documented below, because a
    scale error does not change which side of the line the head drifts to."""
    traj, _, result = run_stroke(StrokeParams(arc_type=ArcType.ARCED))
    arc_true, _ = true_face_path(traj)
    assert arc_true > 0.003
    assert result.path_direction in {"in-to-out", "out-to-in"}


def test_path_arc_magnitude_is_known_to_fall_short_of_truth():
    """KNOWN GAP, pinned deliberately rather than left undiscovered.

    The pipeline computes face velocity as `omega x r` and so assumes the sensor
    itself does not translate. It does: the putter pivots near the hands, roughly
    0.55 m above the grip butt, so the face swings on a ~1.4 m radius rather than
    the 0.85 m lever arm. The missing `v_sensor` term is about a third of the
    face's real motion.

    Measured shortfall is ~39%, against the 10% arc-magnitude target in parent
    spec section 3. THIS METRIC DOES NOT MEET SPEC.

    Recovering the pivot offset per stroke was tried and abandoned: it requires
    differentiating the gyro, which amplifies noise by 1/dt, and the estimate
    collapses by ~0.1 dps -- around this sensor's own noise floor. The viable
    route is a stored per-golfer calibration (section 8.4 territory), which
    averages the estimate over many strokes instead of trusting one.

    This test pins the current behaviour so a regression is visible, and fails
    if the gap widens. Tighten the bound when the calibration lands.
    """
    traj, _, result = run_stroke(StrokeParams())
    arc_true, _ = true_face_path(traj)
    ratio = result.path_arc_m / arc_true
    assert 0.55 < ratio < 0.75, (
        f"arc is {100 * ratio:.0f}% of truth; expected the known ~61% shortfall"
    )


def test_path_arc_does_not_depend_on_putter_type():
    """Invariant 1, applied to path.

    Path is swing geometry, not putter geometry. A zero-torque putter and a
    blade swung on the same plane trace the same path and differ only in how
    the face rotates along it. If arc magnitude tracked arc gain, the pipeline
    would be reading face rotation into a metric that has nothing to do with
    it."""
    arcs = {}
    for arc in ArcType:
        _, _, result = run_stroke(StrokeParams(arc_type=arc))
        arcs[arc.name] = result.path_arc_m

    spread = max(arcs.values()) - min(arcs.values())
    mean_arc = sum(arcs.values()) / len(arcs)

    # The bound is relative, and it is floored by quantization rather than by
    # anything about putters. One gyro LSB is 1.36e-4 rad/s; integrated over
    # roughly a thousand samples with a 0.85 m lever arm, that dither random-
    # walks to about 7 um of displacement. Measured spread is ~11 um on a
    # ~14.3 mm arc, which is 0.08% -- the floor, not a signal.
    #
    # 0.5% leaves six times that margin while still catching a real dependence:
    # the arc-type defect this suite found in face angle was 1.8% of its value.
    assert spread < 0.005 * mean_arc, (
        f"path arc varies with putter type by {100 * spread / mean_arc:.3f}%: {arcs}"
    )
