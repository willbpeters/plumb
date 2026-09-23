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
    requested -- the sample grid cannot hit an arbitrary ratio exactly, and
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


@pytest.mark.parametrize("lie", [
    # A 6 mm arc does not absorb the error; a 24 mm one does. The lie angle is
    # the knob that changes the true arc without changing anything else, so
    # parametrising on it is what makes the dependence visible rather than
    # averaged away.
    pytest.param(5.0, marks=pytest.mark.xfail(
        strict=True,
        reason="open defect 4: arc reads 124% of truth at a 5 deg lie. "
               "Most of it is the pivot estimate, not the statistic -- "
               "see the measurements in this test's docstring.")),
    20.0,
])
def test_arc_magnitude_meets_the_spec_target_under_noise(lie):
    """Parent spec section 3 asks 10% of arc magnitude. Under noise it is not met.

    This is open defect 4, and measuring it moved the diagnosis. The handoff
    attributed the inflation to `ptp(lateral)` collecting the extremes of the
    random walk that integration lays on top of the signal. That effect is
    real and it is the smaller half. Substituting the TRUE pivot offset into
    the same noisy stroke isolates the two:

        lie 5 deg, 0.28 dps, pivot calibration converged over 5 strokes

        truth                                     1.000
        pipeline, zero noise                      1.051   pivot filter bias
        noisy samples, TRUE pivot substituted     1.096   <- this is defect 4
        noisy samples, estimated pivot            1.244   <- what we report

    So roughly 60% of the excess belongs to the pivot ESTIMATE rather than to
    the statistic, and replacing the statistic cannot reach it. Measured
    directly: a quadratic fit of lateral against forward, which is the smooth
    fit the handoff proposed, scores 1.237 against ptp's 1.244.

    What the pivot estimate is doing wrong is recorded in plumb/pivot.py --
    a spurious lateral component where the truth is zero, reaching 0.42 m in
    one configuration.

    The 20 deg case is NOT marked xfail, and that is the point of the
    parametrisation: at a 24 mm arc the same absolute error is 8.6% and lands
    inside spec. The device would pass its own acceptance test on an arced
    stroke and fail it on the straight one it most needs to get right.
    """
    from plumb.pivot import PivotCalibration

    sensor = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                          gyro_bias_dps=1.5)
    params = StrokeParams(lie_angle_deg=lie)
    calibration = PivotCalibration()

    ratios = []
    for seed in range(1, 11):
        traj = generate(params)
        out = simulate(traj, sensor, seed=seed)
        pipe = Pipeline(Thresholds(), out.full_scale,
                        pivot_calibration=calibration)
        result = None
        for i in range(len(traj.time)):
            r = pipe.step(out.gyro_counts[i], out.accel_counts[i])
            if r is not None:
                result = r
        arc_true, _ = true_face_path(traj, pipe)
        # Strokes 1-4 are the calibration still converging and are excluded on
        # purpose; that convergence is measured separately in
        # test_the_pivot_converges_when_calibrated_across_strokes.
        if seed >= 5:
            ratios.append(result.path_arc_m / arc_true)

    mean = sum(ratios) / len(ratios)
    worst = max(ratios, key=lambda r: abs(r - 1.0))
    # Printed, not just asserted -- the convention that has found five defects
    # on this project. A bias shows up here as a mean away from 1.0 even when
    # the individual strokes scatter either side of it.
    print(f"\n  lie {lie:>4.1f} deg  mean {mean:.3f}  worst {worst:.3f}  "
          f"n={len(ratios)}")
    assert 0.90 < mean < 1.10, f"arc averages {100 * mean:.1f}% of truth"


# -- Address reference and abandoned strokes -------------------------------
#
# Every stroke the generator produced before these tests began with exactly
# one second of perfect stillness and ended in an impact. The three defects
# below lived entirely outside that envelope: the address reference was taken
# the moment stillness was first seen and never again, so anything that
# happened afterwards was counted as part of the stroke; and a motion that
# never struck a ball had no way out of the state machine at all.

from dataclasses import replace

from plumb.sensor import GRAVITY, FullScale
from plumb.trajectory import SAMPLE_RATE_HZ


def _feed(pipe, gyro, accel):
    result = None
    for g, a in zip(gyro, accel):
        r = pipe.step(g, a)
        if r is not None:
            result = r
    return result


def _address_rotated_about_shaft(traj, phi_of_i, phi_dot_of_i, upto):
    """The generated stroke, with the ADDRESS samples before `upto` turned
    about the shaft by phi. Physically consistent: attitude, body rate and the
    accelerometer's view of gravity all move together, because simulate()
    derives the accelerometer from the attitude it is given."""
    q_true = traj.q_true.copy()
    omega_true = traj.omega_true.copy()
    for i in range(upto):
        q_true[i] = quat.multiply(traj.q_true[i], quat.rot_z(phi_of_i(i)))
        omega_true[i] = np.array([0.0, 0.0, phi_dot_of_i(i)])
    return replace(traj, q_true=q_true, omega_true=omega_true)


def test_face_reaimed_during_address_is_not_counted_as_face_angle():
    """Invariant 3: face angle is relative to ADDRESS, and address is the pose
    the golfer takes the putter back from -- not the first half-second of
    stillness the device happened to see.

    The golfer settles with the face 2 deg shut, the device captures that,
    then the golfer squares the face, holds, and strokes. Truth is the
    generator's face angle, 0, because the stroke itself is unchanged. The
    reference-at-first-stillness pipeline reported 1.88 deg here: the re-aim,
    seen through the lie angle.
    """
    dt = 1.0 / SAMPLE_RATE_HZ
    delta = np.radians(-2.0)
    i0, i1 = int(1.0 / dt), int(1.5 / dt)

    def phi(i):
        u = np.clip((i - i0) / (i1 - i0), 0.0, 1.0)
        return delta * (1.0 - 0.5 * (1.0 - np.cos(np.pi * u)))

    def phi_dot(i):
        if not i0 <= i <= i1:
            return 0.0
        u = (i - i0) / (i1 - i0)
        return -delta * 0.5 * np.pi * np.sin(np.pi * u) / ((i1 - i0) * dt)

    traj = generate(StrokeParams(address_duration_s=3.0))
    traj = _address_rotated_about_shaft(traj, phi, phi_dot, upto=i1 + 1)
    out = simulate(traj, SensorParams(), seed=1)
    pipe = Pipeline(Thresholds(), out.full_scale)
    result = _feed(pipe, out.gyro_counts, out.accel_counts)

    assert result is not None
    print(f"\n  re-aimed address: face {result.face_angle_deg:+.4f} deg, truth 0")
    assert result.face_angle_deg == pytest.approx(0.0, abs=0.1)


def test_a_long_address_does_not_lengthen_the_integration_window():
    """Invariant 4 rests on drift having ~1.5 s to accumulate. That holds only
    if integration starts at takeaway. Started at the first sight of
    stillness, a golfer who stands over the ball for ten seconds integrates
    bias error for eleven and a half.

    Measured before the fix, these five seeds at 0.28 dps and a slow bias walk
    gave 0.340 deg after a 10 s address, against under 0.04 deg after a 1 s
    one. After it, the address length should not matter, because the
    reference is re-taken from the stillness immediately before the stroke.
    """
    sensor = SensorParams(gyro_noise_dps=0.28, gyro_bias_dps=1.0,
                          gyro_bias_walk_dps_per_s=0.01)
    traj = generate(StrokeParams(address_duration_s=10.0))
    errors = []
    for seed in range(5):
        out = simulate(traj, sensor, seed=seed)
        pipe = Pipeline(Thresholds(), out.full_scale)
        result = _feed(pipe, out.gyro_counts, out.accel_counts)
        errors.append(abs(result.face_angle_deg))
    print(f"\n  10 s address: worst face error {max(errors):.4f} deg")
    assert max(errors) < 0.1


def test_track_storage_is_set_by_the_stroke_not_the_address():
    """The path track stores a vector and a 3x3 per sample. Stored from the
    start of ADDRESS it grows for as long as the golfer stands still, which on
    the device is unbounded memory in a state that can last indefinitely."""
    lengths = {}
    for seconds in (1.0, 10.0):
        traj = generate(StrokeParams(address_duration_s=seconds))
        out = simulate(traj, SensorParams(), seed=1)
        pipe = Pipeline(Thresholds(), out.full_scale)
        _feed(pipe, out.gyro_counts, out.accel_counts)
        lengths[seconds] = len(pipe._track_matrix)
    assert lengths[10.0] == pytest.approx(lengths[1.0], abs=2), lengths


def _rest(counts_accel_last, seconds):
    n = int(seconds * SAMPLE_RATE_HZ)
    return (np.zeros((n, 3), dtype=np.int16),
            np.tile(counts_accel_last, (n, 1)).astype(np.int16))


def test_a_stroke_that_never_strikes_a_ball_is_abandoned():
    """A practice stroke has no impact spike. Before this, the machine sat in
    DOWNSWING forever, growing its buffers by a sample every 1.1 ms, and never
    measured another stroke. It must give up, go back to IDLE, and then
    measure the next real stroke as if nothing had happened."""
    practice = simulate(generate(StrokeParams()),
                        SensorParams(impact_peak_g=0.0), seed=1)
    rest_g, rest_a = _rest(practice.accel_counts[-1], 8.0)
    real_traj = generate(StrokeParams(face_angle_at_impact_deg=2.0))
    real = simulate(real_traj, SensorParams(), seed=2)

    pipe = Pipeline(Thresholds(), practice.full_scale)
    assert _feed(pipe, np.vstack([practice.gyro_counts, rest_g]),
                 np.vstack([practice.accel_counts, rest_a])) is None
    assert pipe.aborts == 1
    assert pipe.state in (State.IDLE, State.ADDRESS)
    assert len(pipe._track_matrix) == 0

    result = _feed(pipe, real.gyro_counts, real.accel_counts)
    assert result is not None
    assert result.face_angle_deg == pytest.approx(2.0, abs=0.1)
    assert result.tempo_ratio == pytest.approx(real_traj.true_tempo_ratio,
                                               abs=0.05)


def test_leaving_address_without_a_stroke_returns_to_idle():
    """A golfer who settles and then fidgets, or walks off, has no valid
    address any more. Rotation about the shaft never trips backswing
    detection -- deliberately, see invariant 1 -- so without this the machine
    stays in ADDRESS holding a reference that no longer describes anything."""
    dt = 1.0 / SAMPLE_RATE_HZ
    traj = generate(StrokeParams(address_duration_s=8.0))
    start = int(1.0 / dt)
    amp, hz = np.radians(2.0), 1.0

    def phi(i):
        return 0.0 if i < start else amp * np.sin(2 * np.pi * hz * (i - start) * dt)

    def phi_dot(i):
        return 0.0 if i < start else amp * 2 * np.pi * hz * np.cos(
            2 * np.pi * hz * (i - start) * dt)

    end = int(7.0 / dt)
    traj = _address_rotated_about_shaft(traj, phi, phi_dot, upto=end)
    out = simulate(traj, SensorParams(), seed=1)
    pipe = Pipeline(Thresholds(), out.full_scale)
    _feed(pipe, out.gyro_counts[:end], out.accel_counts[:end])
    assert State.ADDRESS in pipe.visited
    assert pipe.state is State.IDLE
