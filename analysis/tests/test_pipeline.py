import numpy as np
import pytest

from plumb import quat
from plumb.pipeline import Pipeline, State, Thresholds
from plumb.sensor import GRAVITY, FullScale, SensorParams, simulate
from plumb.trajectory import SAMPLE_RATE_HZ, ArcType, StrokeParams, generate


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

    The floor is 0.003 mm. It was 0.19 mm while the pivot fit carried a small
    cross-shaft error in `d` -- the arc is built from `r + d`, so that error
    put lateral motion into a stroke that has none -- and the velocity-form fit
    removed it (plumb/pivot.py).
    """
    _, _, result = run_stroke(StrokeParams(lie_angle_deg=0.0))
    assert result.path_arc_m < 3e-4
    assert result.path_direction == "straight"


@pytest.mark.parametrize("sign", [+1, -1])
def test_sensor_error_after_the_face_stops_does_not_reach_the_arc(sign):
    """The arc is measured over the motion, not over the stillness after it.

    The face track is the integral of a noisy rate, so it wanders whenever it
    is being integrated -- including the 0.3 s follow-through hold that
    confirms the stroke is over, when the real face is not moving at all. The
    arc used to run to the END of that hold, so the wander moved the chord that
    defines "forward", and with it every lateral value (open defect 4 in
    HANDOFF.md).

    Ground truth is known exactly: a rate injected ONLY during that hold moves
    nothing real, and by then face angle, tempo and the pivot fit are all
    settled, so the true arc is the undisturbed stroke's and every other output
    must be identical too. 0.3 dps sits well under the 5 dps follow-through
    threshold, as real residual noise does. Before the fix it moved a 24 mm arc
    by -12.1% and +12.9% for the two signs; after it, by 0.000%.
    """
    traj = generate(StrokeParams(lie_angle_deg=20.0))
    out = simulate(traj, SensorParams(), seed=1)

    def run(gyro):
        pipe = Pipeline(Thresholds(), out.full_scale)
        result = None
        for i in range(len(traj.time)):
            result = pipe.step(gyro[i], out.accel_counts[i]) or result
        return pipe, result

    clean_pipe, clean = run(out.gyro_counts)
    done = clean_pipe._track_last_n
    hold = int(round(Thresholds().followthrough_hold_s * SAMPLE_RATE_HZ))
    drifting = out.gyro_counts.copy()
    drifting[done - hold:done, 0] += np.int16(
        sign * round(np.radians(0.3) / out.full_scale.gyro_rad_per_count))
    pipe, disturbed = run(drifting)

    print(f"\n  arc {1e3 * clean.path_arc_m:.3f} mm clean, "
          f"{1e3 * disturbed.path_arc_m:.3f} mm with drift in the hold "
          f"({100 * (disturbed.path_arc_m / clean.path_arc_m - 1):+.3f}%)")
    assert pipe._track_last_n == done, "the drift must not move the end of the stroke"
    assert disturbed.face_angle_deg == clean.face_angle_deg
    assert disturbed.tempo_ratio == clean.tempo_ratio
    assert disturbed.path_arc_m == pytest.approx(clean.path_arc_m, rel=1e-3)


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


def sample_index(n: int) -> int:
    """The stream index of the pipeline's sample number n.

    Pipeline.n counts from 1: it is incremented before a sample is processed,
    so the first sample is n = 1 and lives at index 0. Every i_* the pipeline
    records is an n. Ground truth indexed by n directly is one sample late --
    true_face_path and the impact speed tests both were, found 2026-09-28 by a
    session test whose offsets made the error large enough to see.
    """
    return n - 1


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
        lo = sample_index(pipeline._track_first_n)
        hi = sample_index(pipeline._track_last_n) + 1
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

    Measured against ground truth over the same window: 0.996 to 1.002 of
    truth across arc types and lie angles, against the 10% target in parent
    spec section 3. (It was 1.3% to 5.1% over, then 0.8% to 4.8% once the arc
    was measured over the motion only; the rest was the acceleration-form
    pivot fit's filter bias.) THIS METRIC MEETS SPEC, noiselessly and -- see
    test_arc_under_noise_does_not_depend_on_putter_type -- on the session mean
    under noise. It has not been measured against a real stroke, because there
    is no logged corpus yet.
    """
    traj, pipe, result = run_stroke(StrokeParams())
    arc_true, _ = true_face_path(traj, pipe)
    ratio = result.path_arc_m / arc_true
    assert 0.90 < ratio < 1.10, f"arc is {100 * ratio:.1f}% of truth"


def test_path_arc_barely_depends_on_putter_type():
    """Invariant 1, applied to path.

    Path is swing geometry, not putter geometry. A zero-torque putter and a
    blade swung on the same plane trace the same path and differ only in how
    the face rotates along it. If arc magnitude tracked arc gain, the pipeline
    would be reading face rotation into a metric that has nothing to do with
    it.

    Measured spread: 0.37%, 0.09 mm on a 24 mm arc. It grew from 0.08% to
    1.69% when the pivot offset began to be estimated, because the
    acceleration-form fit low-passed the rate before squaring it, and that
    bias depended on the rate's spectrum -- which differs between a face that
    rotates and one that does not. The velocity form has no filter; the spread
    went back down. Under noise the same fix took the arced putter from 1.142
    of truth to 1.010, level with the straight putter's 1.006.
    """
    arcs = {}
    for arc in ArcType:
        _, _, result = run_stroke(StrokeParams(arc_type=arc))
        arcs[arc.name] = result.path_arc_m

    spread = max(arcs.values()) - min(arcs.values())
    mean_arc = sum(arcs.values()) / len(arcs)

    # Both bounds are measured rather than chosen. The relative one catches any
    # real growth in the dependence -- 1.5x the measured 0.37% (it was 1.5x
    # 1.69% while the filter bias was there). The absolute one is the
    # check that actually protects the golfer: half a millimetre is below
    # anything anyone could act on, and it holds even if a future change makes
    # the arc itself larger.
    assert spread < 0.0056 * mean_arc, (
        f"path arc varies with putter type by {100 * spread / mean_arc:.3f}%: {arcs}"
    )
    assert spread < 0.0005, f"putter-type spread is {1000 * spread:.3f} mm"


def test_the_pivot_converges_when_calibrated_across_strokes():
    """How the pivot is delivered: one stroke is good, a session is better.

    History, because the numbers moved a long way. The first estimator fit the
    ACCELERATION relation, which needs the rate differentiated; at 0.28 dps one
    stroke recovered the pivot to about 17% and only a five-stroke calibration
    got it to 2.4% -- and even then the arced putter's arc read 1.142 of truth,
    because noise in the design matrix shrank the fit's weakest direction
    (errors-in-variables; see plumb/pivot.py for the oracles that pinned it).

    The velocity form needs no derivative, fits the start velocity rather than
    trusting one noisy sample for it, and subtracts the gyro noise's measured
    contribution. Measured at 0.28 dps plus 1.5 dps bias, arced putter, lie 5,
    |d - truth| over the FULL vector, not just the component along the shaft:

        single strokes      3-34 mm
        calibrated, k >= 2  1.4-5.7 mm

    Asserted on the whole vector because the cross-shaft components are the
    ones that turn face rotation into fake path; the old test checked only
    offset[2] and would have passed with all of the arced putter's error in it.
    """
    from plumb.pivot import PivotCalibration
    from plumb.sensor import SensorParams, simulate
    from plumb.trajectory import generate

    sensor = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                          gyro_bias_dps=1.5)
    params = StrokeParams(lie_angle_deg=5.0, arc_type=ArcType.ARCED)
    truth = np.array([0.0, 0.0, -params.pivot_offset_m])
    calibration = PivotCalibration()
    single, calibrated = [], []
    for seed in range(1, 11):
        traj = generate(params)
        out = simulate(traj, sensor, seed=seed)
        pipe = Pipeline(Thresholds(), out.full_scale,
                        pivot_calibration=calibration)
        for i in range(len(traj.time)):
            pipe.step(out.gyro_counts[i], out.accel_counts[i])
        single.append(np.linalg.norm(pipe._pivot.solve().offset - truth))
        calibrated.append(np.linalg.norm(calibration.solve().offset - truth))

    print(f"\n  |d - truth| mm, single strokes: "
          f"{' '.join(f'{1e3 * e:.1f}' for e in single)}"
          f"\n  calibrated after each stroke:    "
          f"{' '.join(f'{1e3 * e:.1f}' for e in calibrated)}")
    assert calibration.strokes == 10
    assert max(single) < 0.05
    assert max(calibrated[1:]) < 0.01


def test_arc_under_noise_does_not_depend_on_putter_type():
    """Invariant 1 for path, under the noise the board actually has.

    The same session -- same seeds, so the same noise -- is played with a
    straight-face putter and an arced one. Their true arcs are identical (path
    is swing geometry), so their measured arcs must be too, and both must agree
    with the generator.

    They did not. The arced putter read 1.142 of truth at lie 5 against the
    straight one's 1.010, because face rotation tilts the rotation axis ~19 deg
    off body Y and the pivot fit's weakest direction then carries a real share
    of `d` -- which gyro noise in the design matrix shrank toward zero
    (errors-in-variables), turning face rotation into sideways face travel.
    Oracles pinned it: the true angular rate in the fit alone took the arced
    putter to 1.049; the true acceleration alone changed nothing (1.137).

    Strokes 5 to 16, so the pivot calibration has had four strokes to settle.
    The per-stroke spread at lie 5 is ~+/-15% (a 6 mm arc against a ~0.5 mm
    random walk in the track), so the bound is on the session mean.
    """
    from plumb.pivot import PivotCalibration

    sensor = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02,
                          gyro_bias_dps=1.5)
    means = {}
    for arc in (ArcType.STRAIGHT, ArcType.ARCED):
        calibration = PivotCalibration()
        ratios = []
        for seed in range(1, 17):
            traj = generate(StrokeParams(lie_angle_deg=5.0, arc_type=arc))
            out = simulate(traj, sensor, seed=seed)
            pipe = Pipeline(Thresholds(), out.full_scale,
                            pivot_calibration=calibration)
            result = None
            for i in range(len(traj.time)):
                result = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or result
            if seed >= 5:
                ratios.append(result.path_arc_m / true_face_path(traj, pipe)[0])
        means[arc.name] = float(np.mean(ratios))

    print(f"\n  session-mean arc / truth at lie 5, 0.28 dps: {means}")
    assert abs(means["ARCED"] - 1.0) < 0.05, means
    assert abs(means["ARCED"] - means["STRAIGHT"]) < 0.03, means


def test_accelerometer_bias_needs_the_device_calibration():
    """Open defect 6, and parent spec 8.1 shown to be load-bearing for path.

    An accelerometer bias reaches the arc two ways, found by oracles: it tilts
    the ground plane that `g0` defines, which leaks the head's vertical motion
    into lateral (a bias on body Y, the swing axis, does this and the stroke
    cannot observe it); and it leaks (I - R^T) b into the pivot fit (body X).
    Neither is fixable per stroke. The device calibration removes both.

    Ground truth throughout: the true face path. The biased board's
    calibration is solved from a simulated tumble of the same sensor, by the
    same solver the host tool uses, not handed in.
    """
    from plumb.calibration import solve_accel_calibration
    from plumb.pivot import PivotCalibration
    from plumb.sensor import GRAVITY

    bias = np.array([0.05, 0.2, -0.1])
    scale = 0.01
    base = dict(gyro_noise_dps=0.28, accel_noise_mps2=0.02, gyro_bias_dps=1.5)

    # The tumble: this sensor's own resting readings on six faces, through the
    # same bias-and-scale model the simulator applies to the stroke.
    ups = np.vstack([np.eye(3), -np.eye(3)]) * GRAVITY
    tumble = ups * (1.0 + scale) + bias
    accel_cal = solve_accel_calibration(tumble, gravity=GRAVITY)

    def session(sensor, calibration=None):
        pivot = PivotCalibration()
        ratios = []
        for seed in range(1, 13):
            traj = generate(StrokeParams(lie_angle_deg=5.0))
            out = simulate(traj, sensor, seed=seed)
            pipe = Pipeline(Thresholds(), out.full_scale,
                            pivot_calibration=pivot, accel_calibration=calibration)
            result = None
            for i in range(len(traj.time)):
                result = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or result
            if seed >= 5:
                ratios.append(result.path_arc_m / true_face_path(traj, pipe)[0])
        return float(np.mean(ratios))

    clean = session(SensorParams(**base))
    biased = SensorParams(**base, accel_bias_mps2=bias, accel_scale_error=scale)
    raw = session(biased)
    calibrated = session(biased, accel_cal)

    print(f"\n  arc / truth at lie 5: unbiased {clean:.3f}, biased {raw:.3f}, "
          f"biased and calibrated {calibrated:.3f}")
    assert abs(raw - clean) > 0.1, "the bias should visibly move the arc"
    assert calibrated == pytest.approx(clean, abs=0.005)


def test_result_reports_both_phases_and_the_travel():
    """The screens need the phases, not only their ratio (the tempo bars are
    drawn to length), and the path drawing is scaled by the travel. All three
    were computed and thrown away; now they are reported, against the truth
    the generator produced."""
    for tempo in (1.5, 2.0, 3.0):
        traj, pipe, result = run_stroke(StrokeParams(tempo_ratio=tempo))
        dt = 1.0 / SAMPLE_RATE_HZ
        true_backswing = (traj.transition_index - traj.address_end_index) * dt
        true_downswing = (traj.impact_index - traj.transition_index) * dt
        # One sample either way: the phases' own boundaries are sample-grid
        # instants in the generator and back-extrapolated ones in the pipeline.
        assert result.backswing_s == pytest.approx(true_backswing, abs=2 * dt)
        assert result.downswing_s == pytest.approx(true_downswing, abs=2 * dt)
        assert result.tempo_ratio == result.backswing_s / result.downswing_s
        _, travel_true = true_face_path(traj, pipe)
        assert result.path_travel_m == pytest.approx(travel_true, rel=0.05)


def test_the_sample_rate_is_the_callers_and_it_matters():
    """Will's decision, 2026-09-27: the firmware measures its own rate at
    startup rather than assuming 896.8 Hz, because this board runs at
    906.86. So the pipeline takes the rate it is given.

    Why it matters, against ground truth: data taken at one rate and
    integrated at another scales every integrated angle by the ratio. Tempo
    is a ratio of durations and does not move; face angle does, by exactly
    the 1.12% this board's oscillator is off nominal."""
    traj = generate(StrokeParams(face_angle_at_impact_deg=5.0))
    out = simulate(traj, SensorParams(), seed=1)
    results = {}
    for rate in (SAMPLE_RATE_HZ, 906.86):
        pipe = Pipeline(Thresholds(), out.full_scale, sample_rate_hz=rate)
        assert pipe.dt == 1.0 / rate
        result = None
        for i in range(len(traj.time)):
            result = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or result
        results[rate] = result
    right, wrong = results[SAMPLE_RATE_HZ], results[906.86]
    assert right.face_angle_deg == pytest.approx(5.0, abs=0.01)
    assert wrong.face_angle_deg == pytest.approx(
        right.face_angle_deg * SAMPLE_RATE_HZ / 906.86, rel=0.002)
    assert wrong.tempo_ratio == pytest.approx(right.tempo_ratio, abs=0.01)


def true_face_speed(traj, index):
    """The face's real speed at a sample, from the generator's own rate and
    geometry: a rigid rotation about the pivot, face at pivot_offset +
    lever_arm below it along the shaft. Written from the generator's
    parameters, not from anything the pipeline computes."""
    p = traj.params
    face = np.array([0.0, 0.0, -(p.pivot_offset_m + p.lever_arm_m)])
    return float(np.linalg.norm(np.cross(traj.omega_true[index], face)))


@pytest.mark.parametrize("tempo", [1.5, 2.0, 3.0])
@pytest.mark.parametrize("amplitude", [6.0, 12.0, 20.0])
def test_impact_speed_matches_the_true_face_speed(tempo, amplitude):
    """Parent spec 1.2.1: impact speed is a first-build metric.

    Noiselessly, |omega x (d + r)| is exact given the rate, so what is left is
    the pivot fit (about 2 mm on a 1.4 m radius, 0.15%) and WHEN: the
    pipeline takes the last sample before the impact spike, because from the
    spike on the gyro reads the collision as well as the swing. Held to 0.5%
    against the truth at that sample, and the gap to the truth at the
    generator's impact instant is printed -- it is the definition's cost, not
    the algorithm's."""
    traj, pipe, result = run_stroke(StrokeParams(
        tempo_ratio=tempo, backswing_amplitude_deg=amplitude,
        followthrough_amplitude_deg=amplitude))
    at_sample = true_face_speed(traj, sample_index(pipe.i_speed_n))
    at_impact = true_face_speed(traj, traj.impact_index)
    print(f"\n  tempo {tempo} amp {amplitude}: {result.impact_speed_mps:.4f} m/s, "
          f"truth {at_sample:.4f} at its sample ({sample_index(pipe.i_speed_n)}), "
          f"{at_impact:.4f} at impact ({traj.impact_index})")
    assert sample_index(pipe.i_speed_n) < traj.impact_index
    assert result.impact_speed_mps == pytest.approx(at_sample, rel=0.005)


def test_impact_speed_under_noise():
    """At the board's measured floor (0.28 dps, 1.5 dps bias), one stroke.
    A single sample of rate carries 0.28 dps against ~50 dps at impact
    (0.5%), and the per-stroke pivot fit a few mm more; 2% is about three
    of those combined sigmas. The spread is printed."""
    errors = []
    for seed in range(1, 11):
        traj, pipe, result = run_stroke(
            StrokeParams(), SensorParams(gyro_noise_dps=0.28,
                                         accel_noise_mps2=0.02,
                                         gyro_bias_dps=1.5), seed=seed)
        truth = true_face_speed(traj, sample_index(pipe.i_speed_n))
        errors.append(result.impact_speed_mps / truth - 1.0)
    errors = np.array(errors)
    print(f"\n  impact speed at 0.28 dps, 10 strokes: error mean "
          f"{100 * errors.mean():+.2f}%, worst {100 * np.abs(errors).max():.2f}%")
    assert np.abs(errors).max() < 0.02


def test_impact_speed_does_not_depend_on_putter_type():
    """Invariant 1: speed is swing, and a zero-torque putter swung the same
    way arrives at the same speed as a blade."""
    speeds = {arc.name: run_stroke(StrokeParams(arc_type=arc))[2].impact_speed_mps
              for arc in ArcType}
    assert max(speeds.values()) - min(speeds.values()) < 0.002 * max(speeds.values())


def test_no_pivot_means_no_impact_speed_not_a_low_one():
    """Without the pivot the face radius is the lever arm alone, 0.85 m
    against 1.4 m: 39% low. That is not a measurement, so it is not
    reported. Forced here by refusing every pivot fit."""
    traj = generate(StrokeParams())
    out = simulate(traj, SensorParams(), seed=1)
    pipe = Pipeline(Thresholds(pivot_max_residual=-1.0), out.full_scale)
    result = None
    for i in range(len(traj.time)):
        result = pipe.step(out.gyro_counts[i], out.accel_counts[i]) or result
    assert result.impact_speed_mps is None
    assert result.face_angle_deg == pytest.approx(0.0, abs=0.1)


# -- The address reference -------------------------------------------------
#
# Carried over from bc2d3d7, which was written in parallel with the exits
# above (ecb1caa) against the same parent and fixed the same class of defect a
# different way: it re-took the address reference from the last stillness
# before the back-extrapolated onset and replayed the samples since. The merge
# kept the exits design, which is ported to C and running on the board, and
# kept these tests. The first passes under it; the other two measure what it
# does not reach. bc2d3d7's _rebase is the reference fix for both, not yet
# reconciled with the rest test or ported.


def _address_reaimed(address_s, reaim_deg, reaim_s):
    """A stroke whose face is turned about the shaft by reaim_deg over reaim_s,
    starting 1 s into address, then held. Physically consistent: attitude, body
    rate and the accelerometer's view of gravity move together, because
    simulate() derives the accelerometer from the attitude it is given. Truth
    is the generator's face angle, 0 -- the stroke itself is unchanged."""
    from dataclasses import replace

    dt = 1.0 / SAMPLE_RATE_HZ
    delta = np.radians(reaim_deg)
    i0, i1 = int(1.0 / dt), int((1.0 + reaim_s) / dt)
    traj = generate(StrokeParams(address_duration_s=address_s))
    q_true, omega_true = traj.q_true.copy(), traj.omega_true.copy()
    for i in range(i1 + 1):
        u = np.clip((i - i0) / (i1 - i0), 0.0, 1.0)
        phi = -delta * 0.5 * (1.0 - np.cos(np.pi * u)) + delta
        phi_dot = (-delta * 0.5 * np.pi * np.sin(np.pi * u) / ((i1 - i0) * dt)
                   if i0 <= i <= i1 else 0.0)
        q_true[i] = quat.multiply(traj.q_true[i], quat.rot_z(phi))
        omega_true[i] = np.array([0.0, 0.0, phi_dot])
    return replace(traj, q_true=q_true, omega_true=omega_true)


def _session_face(traj):
    from plumb.session import Session

    out = simulate(traj, SensorParams(), seed=1)
    session = Session(Thresholds(), out.full_scale)
    result = None
    for g, a in zip(out.gyro_counts, out.accel_counts):
        result = session.step(g, a) or result
    return result


def test_a_quick_reaim_during_address_is_not_counted_as_face_angle():
    """Invariant 3: face angle is relative to the address the stroke starts
    from, not the first stillness the device saw. The golfer settles 2 deg
    shut, squares the face over half a second, holds, and strokes. Read from
    first stillness this was 1.88 deg. The re-aim fails the rest test and the
    settle passes it, so the session re-captures address."""
    result = _session_face(_address_reaimed(3.0, -2.0, 0.5))
    assert result is not None
    assert result.face_angle_deg == pytest.approx(0.0, abs=0.1)


@pytest.mark.xfail(strict=True, reason=(
    "a re-aim slow enough never to fail the 0.5 s stillness test is never "
    "re-captured: 2 deg over 2 s reads 1.88 deg (measured at the merge)"))
def test_a_slow_reaim_during_address_is_not_counted_as_face_angle():
    """The same re-aim over two seconds. Its rate peaks near 1.6 dps and its
    0.5 s standard deviation stays under the 0.8 dps stillness threshold, so
    the rest test never sees the golfer leave the pose, and the reference
    stays where the face was first held."""
    result = _session_face(_address_reaimed(4.0, -2.0, 2.0))
    assert result is not None
    assert result.face_angle_deg == pytest.approx(0.0, abs=0.1)


@pytest.mark.xfail(strict=True, reason=(
    "attitude is integrated from first stillness, so a 10 s address "
    "integrates bias error for ~11.5 s: 0.340 deg worst (measured at the "
    "merge) against under 0.04 deg after a 1 s address"))
def test_a_long_address_does_not_lengthen_the_integration_window():
    """Invariant 4 rests on drift having ~1.5 s to accumulate. That holds only
    if integration starts at takeaway, or the bias is re-nulled close to it.
    Five seeds at 0.28 dps with a slow bias walk."""
    sensor = SensorParams(gyro_noise_dps=0.28, gyro_bias_dps=1.0,
                          gyro_bias_walk_dps_per_s=0.01)
    errors = []
    for seed in range(5):
        _, _, result = run_stroke(StrokeParams(address_duration_s=10.0),
                                  sensor, seed=seed)
        errors.append(abs(result.face_angle_deg))
    print(f"\n  10 s address: worst face error {max(errors):.4f} deg")
    assert max(errors) < 0.1
