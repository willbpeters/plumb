import numpy as np
import pytest

from plumb import quat
from plumb.pipeline import Pipeline, State, Thresholds
from plumb.sensor import GRAVITY, FullScale, SensorParams, simulate
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


def test_accel_correction_pulls_a_tilted_attitude_back_to_gravity():
    """The gravity correction must converge, not diverge.

    Ground truth is known exactly: the device is held still, so its true
    attitude relative to address is identity, and the estimate is seeded 2 deg
    off in pure tilt -- the component gravity can observe. A correct
    complementary filter shrinks that error at rate `gain`; a sign error in the
    cross product runs the same loop as positive feedback and grows it.

    Normal runs cannot see this. q resets to identity on ADDRESS entry and the
    stock gain is a ~50 s time constant, so the wrong sign moved a 2 deg tilt
    only to 2.082 deg over 2 s. A strong gain exposes it: 86.95 deg."""
    fs = FullScale()
    g_counts = np.round(np.array([0.0, 0.0, GRAVITY]) / fs.accel_mps2_per_count).astype(np.int16)
    zero = np.zeros(3, dtype=np.int16)

    pipe = Pipeline(Thresholds(accel_gain_static=2.0), fs)
    pipe.state = State.ADDRESS
    pipe.g0 = g_counts * fs.accel_mps2_per_count
    pipe.bias = np.zeros(3)
    pipe.q = quat.rot_x(np.radians(2.0))

    def tilt_deg():
        g_hat = pipe.g0 / np.linalg.norm(pipe.g0)
        predicted = quat.rotate(quat.conjugate(pipe.q), g_hat)
        return np.degrees(np.arccos(np.clip(predicted @ g_hat, -1.0, 1.0)))

    start = tilt_deg()
    for _ in range(int(2.0 / pipe.dt)):
        pipe.step(zero, g_counts)
    end = tilt_deg()

    assert pipe.state is State.ADDRESS, "zero input must not trigger a backswing"
    # 2 s at 2 rad/s is four time constants: e^-4 of 2 deg is 0.037 deg.
    assert end < 0.1, f"tilt went {start:.3f} -> {end:.3f} deg; the correction diverges"


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
    tilt and the cone degenerates to a plane: the path must go straight. Truth
    here is exactly zero, so this is the test that proves the arc is real
    geometry rather than accumulated error -- error would not vanish here.

    The floor is 0.19 mm rather than the 0.09 mm it was before the pivot offset
    was estimated, and the reason is worth stating: the arc is now built from
    `r + d`, and the small cross-shaft error in `d` puts a little lateral
    motion into a stroke that has none. It is a fifth of a millimetre against a
    3 mm straight-path threshold, and the classification is unaffected.
    """
    _, _, result = run_stroke(StrokeParams(lie_angle_deg=0.0))
    assert result.path_arc_m < 3e-4
    assert result.path_direction == "straight"


@pytest.mark.parametrize("lie", [5.0, 10.0, 20.0])
def test_path_arc_grows_with_lie_angle(lie):
    """A flatter lie swings the head on a more tilted cone and arcs more.

    This used to compare against three numbers measured from the pipeline
    itself -- 3.6, 7.3 and 14.3 mm -- which made it a regression pin rather
    than a check, and it passed happily while the pipeline read 61% of truth.
    It now compares against the generator, which knows the answer, at the 10%
    relative accuracy parent spec section 3 asks of this metric.
    """
    traj, pipe, result = run_stroke(StrokeParams(lie_angle_deg=lie))
    arc_true, _ = true_face_path(traj, pipe)
    assert result.path_arc_m == pytest.approx(arc_true, rel=0.10)


def true_face_path(traj, pipeline=None):
    """The face point's real ground-plane track, straight from the generator.

    The face is rigidly attached (pivot_offset + lever_arm) below the pivot along
    the shaft, so its world position is just the true attitude applied to that
    offset. This is the ground truth the path tests were missing: every other
    path test in this file checks the pipeline against ITSELF -- straight shaft
    gives a straight line, arc grows with lie angle, arc ignores putter type --
    and all of them pass with a scale error present.

    Pass the pipeline to take the truth over exactly the samples it tracked.
    Without that the comparison measures the two windows against each other as
    much as the algorithm: the arc keeps growing through the follow-through, so
    a fixed `impact + 200` window understates truth by 12% against a pipeline
    that tracked 568 samples past impact. That mismatch was invisible while the
    pipeline was reading 61% of truth and became the dominant term as soon as
    it was not.
    """
    p = traj.params
    face_body = np.array([0.0, 0.0, -(p.pivot_offset_m + p.lever_arm_m)])
    if pipeline is not None and pipeline._track_first_n is not None:
        lo, hi = pipeline._track_first_n, pipeline._track_last_n + 1
    else:
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


def test_path_arc_magnitude_meets_the_spec_target():
    """The gap that used to be pinned here is closed.

    The pipeline computed face velocity as `omega x r`, which assumes the
    sensor does not translate. It does: the putter pivots near the hands, so
    the face swings on `r + d` -- 1.4 m rather than the 0.85 m lever arm, and
    0.85/1.4 = 0.607 was the shortfall, measured at 61%.

    The pivot offset is now estimated per stroke (plumb/pivot.py). The earlier
    attempt was abandoned because differentiating the gyro amplifies noise by
    1/dt and the per-sample estimate collapses around this sensor's own floor.
    What changed is the framing, not the sensor: `a = omega_dot x d + omega x
    (omega x d)` is LINEAR in d, so a stroke is one over-determined system of
    three unknowns against a thousand samples rather than a thousand
    independent guesses, and the noise averages down instead of dominating.

    Measured against ground truth over the same window: 1.3% to 5.1% of truth
    across arc types and lie angles, against the 10% target in parent spec
    section 3. THIS METRIC NOW MEETS SPEC, noiselessly. It has not been
    measured against a real stroke, because there is no logged corpus yet.
    """
    traj, pipe, result = run_stroke(StrokeParams())
    arc_true, _ = true_face_path(traj, pipe)
    ratio = result.path_arc_m / arc_true
    assert 0.90 < ratio < 1.10, f"arc is {100 * ratio:.1f}% of truth"


def test_path_arc_barely_depends_on_putter_type():
    """Invariant 1, applied to path -- and a trade made with open eyes.

    Path is swing geometry, not putter geometry. A zero-torque putter and a
    blade swung on the same plane trace the same path and differ only in how
    the face rotates along it. If arc magnitude tracked arc gain, the pipeline
    would be reading face rotation into a metric that has nothing to do with
    it.

    THE MEASURED SPREAD GREW, from 0.08% to 1.69%, when the pivot offset began
    to be estimated. It is not a putter-type prior -- nothing normalises
    against expected rotation and no threshold keys off rotation amplitude, see
    plumb/pivot.py -- but it is a real dependence, and it is recorded here
    rather than tucked away.

    Where it comes from: the pivot fit low-passes the angular rate before
    building its design matrix, which is what stops noise in the derivative
    from collapsing the estimate. The matrix is quadratic in that rate, so
    filtering the rate is not identical to filtering the equation, and the
    small residual bias depends on the rate's spectrum -- which differs between
    a putter whose face rotates through the stroke and one whose face does not.
    Measured on the pivot offset itself: -0.5681 m for a straight-face stroke
    against -0.5556 m for an arced one, both against a true -0.55.

    What it costs, in the units that matter: 0.4 mm of spread on a 24 mm arc.
    Against the alternative -- no pivot estimate at all and 39% of the arc
    missing -- it is a good trade, and it stays well inside the 10% accuracy
    parent spec section 3 asks of this metric. The screen does not show arc
    magnitude at all; it shows the shape.
    """
    arcs = {}
    for arc in ArcType:
        _, _, result = run_stroke(StrokeParams(arc_type=arc))
        arcs[arc.name] = result.path_arc_m

    spread = max(arcs.values()) - min(arcs.values())
    mean_arc = sum(arcs.values()) / len(arcs)

    # Both bounds are measured rather than chosen. The relative one catches any
    # real growth in the dependence -- it is 1.5x the measured 1.69%, where the
    # old bound was 6x a 0.08% quantization floor. The absolute one is the
    # check that actually protects the golfer: half a millimetre is below
    # anything anyone could act on, and it holds even if a future change makes
    # the arc itself larger.
    assert spread < 0.025 * mean_arc, (
        f"path arc varies with putter type by {100 * spread / mean_arc:.3f}%: {arcs}"
    )
    assert spread < 0.0005, f"putter-type spread is {1000 * spread:.3f} mm"


def test_the_pivot_converges_when_calibrated_across_strokes():
    """How this metric has to be delivered, and what still stands in the way.

    A single stroke at this board's measured 0.28 dps noise floor recovers the
    pivot to about 17%, and the arc that follows lands anywhere from 8% under
    truth to 84% over -- worse per stroke than the systematic 39% shortfall it
    replaced, because that shortfall was at least consistent. Sharing one
    PivotCalibration across strokes sums their normal equations, which is
    inverse-variance weighting for free: 12% after two strokes, 2.4% after
    five. The pivot is a property of the golfer, learned over a session, not of
    the stroke -- which is where parent spec section 8.4 already put it.

    WHAT THIS DOES NOT YET FIX, measured at 0.28 dps over strokes 5 to 10:

        lie    no pivot (before)    calibrated
         5     0.665 of truth       1.244
        20     0.620                1.086

    The systematic scale error is gone. What is left is an OVERSHOOT that grows
    as the true arc shrinks, and it is a second defect this work uncovered
    rather than caused: arc is reported as `ptp(lateral)`, and peak-to-peak of
    an integrated signal is biased upward by noise, because a maximum minus a
    minimum collects the extremes of the random walk. A 24 mm arc absorbs it at
    +8.6%; a 6 mm arc does not, at +24%. It was present before and hidden under
    the shortfall, which was pulling the other way.

    So this test asserts what is now true -- the pivot converges -- and not
    that the arc meets the 10% target under noise, because it does not. See
    HANDOFF.md.
    """
    from plumb.pivot import PivotCalibration
    from plumb.sensor import SensorParams, simulate
    from plumb.trajectory import generate

    sensor = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                          gyro_bias_dps=1.5)
    params = StrokeParams(lie_angle_deg=5.0)
    calibration = PivotCalibration()
    after = {}
    for stroke, seed in enumerate(range(1, 11), start=1):
        traj = generate(params)
        out = simulate(traj, sensor, seed=seed)
        pipe = Pipeline(Thresholds(), out.full_scale,
                        pivot_calibration=calibration)
        for i in range(len(traj.time)):
            pipe.step(out.gyro_counts[i], out.accel_counts[i])
        after[stroke] = calibration.solve().offset[2]

    assert calibration.strokes == 10
    one = abs(after[1] + 0.55) / 0.55
    five = abs(after[5] + 0.55) / 0.55
    assert one > 0.10, f"one stroke should be poor, was {100 * one:.1f}%"
    assert five < 0.05, f"five strokes should converge, was {100 * five:.1f}%"
    assert abs(after[10] + 0.55) / 0.55 < 0.08
