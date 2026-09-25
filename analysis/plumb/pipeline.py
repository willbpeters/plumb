"""Streaming implementation of the parent spec's section 7 processing chain.

One sample at a time, no lookahead: the firmware sees samples as the IMU
interrupt delivers them and cannot inspect the future. Structuring the
reference implementation the same way makes lookahead impossible rather than
merely discouraged, and makes the eventual C port a translation rather than a
rewrite.

Only `Pipeline` and its state are ported to C.
"""

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from plumb import quat
from plumb.pivot import PivotCalibration, PivotEstimator, skew
from plumb.sensor import FullScale
from plumb.trajectory import SAMPLE_RATE_HZ


class State(Enum):
    IDLE = auto()
    ADDRESS = auto()
    BACKSWING = auto()
    DOWNSWING = auto()
    IMPACT = auto()
    FOLLOWTHROUGH = auto()
    DONE = auto()


@dataclass(frozen=True)
class Thresholds:
    """Every threshold is configurable and none is a guessed constant.

    Parent spec invariant 5: these are derived from the logged corpus in
    Phase 2 and fixed in the notebook before the port to C. The values here are
    placeholders for the synthetic harness only, and are deliberately not
    presented as tuned. Nothing keys off rotation amplitude (invariant 1).
    """

    stillness_window_s: float = 0.50
    stillness_gyro_std_rad: float = np.radians(0.8)
    onset_gyro_rad: float = np.radians(1.0)
    backswing_gyro_rad: float = np.radians(8.0)
    backswing_hold_s: float = 0.04
    transition_gyro_rad: float = np.radians(2.0)
    impact_accel_mps2: float = 100.0
    followthrough_gyro_rad: float = np.radians(5.0)
    followthrough_hold_s: float = 0.30
    accel_gain_static: float = 0.02
    accel_gain_stroke: float = 0.0
    path_straight_arc_m: float = 0.003
    # Low-pass corner for the rate that feeds the pivot fit, in Hz. See
    # plumb/pivot.py for the measured trade-off this sits on, and note that it
    # is a filter design rather than a detection threshold.
    pivot_filter_hz: float = 2.0
    # Above this unexplained share of the measured acceleration the pivot fit
    # is not describing a rigid rotation about a fixed point, and the path
    # falls back to the sensor's own lever arm.
    pivot_max_residual: float = 0.5


@dataclass
class StrokeResult:
    face_angle_deg: float
    tempo_ratio: float
    path_arc_m: float
    path_direction: str


class Pipeline:
    def __init__(self, thresholds: Thresholds, full_scale: FullScale,
                 lever_arm_m: float = 0.85,
                 pivot_calibration: PivotCalibration | None = None):
        self.th = thresholds
        self.fs = full_scale
        self.lever_arm = lever_arm_m
        self.dt = 1.0 / SAMPLE_RATE_HZ

        self.state = State.IDLE
        self.visited: list[State] = [State.IDLE]
        self.n = 0

        self._still: list[np.ndarray] = []
        self._still_accel: list[np.ndarray] = []
        self.address_captured = False
        self.bias = np.zeros(3)
        self.g0 = np.zeros(3)

        self.q = quat.identity()          # attitude relative to address
        self._prev_omega = None           # previous bias-corrected rate, for trapezoidal integration
        self.q_impact = None
        self.i_backswing_start = None
        self.i_transition = None
        self.i_impact = None
        self._hold = 0
        self._omega_history: list[np.ndarray] = []

        # Back-tracking state. Detecting a stroke boundary always lags the
        # boundary itself; these record where it actually was. Past data only.
        self._last_quiet_n = 0
        self._last_same_sign_n = None
        self._backswing_axis = None
        self._backswing_sign = None
        self._prev_magnitude = None
        self._impact_window: list[tuple[int, np.ndarray]] = []

        self._face_track: list[np.ndarray] = []
        # Per-sample R(q) [omega]x dt. The extra face displacement from
        # the sensor's own translation is exactly this matrix times the pivot
        # offset, which is not known until the stroke ends -- so the part that
        # depends on it is carried as a matrix and multiplied out once, rather
        # than the pipeline waiting or guessing. Nine floats a sample, against
        # three for the track itself; see the port note in _compute.
        self._track_matrix: list[np.ndarray] = []
        # Which samples the face track actually spans. Ground truth has to be
        # taken over the same window or the comparison measures the window
        # rather than the algorithm -- see test_pipeline.true_face_path.
        self._track_first_n: int | None = None
        self._track_last_n: int | None = None
        self._pivot = PivotEstimator(self.dt, thresholds.pivot_filter_hz)
        # Shared across strokes when the caller supplies one. A single stroke
        # recovers the pivot to only about 17% at this board's noise floor;
        # five converge to 2.4% (see PivotCalibration). Without one, the path
        # is computed from this stroke alone and is correspondingly noisy.
        self._pivot_calibration = pivot_calibration
        self._pivot_solution = None
        self._impact_track_index = 0

    # -- conversion ------------------------------------------------------
    def _to_rad_s(self, counts: np.ndarray) -> np.ndarray:
        return counts.astype(float) * self.fs.gyro_rad_per_count

    def _to_mps2(self, counts: np.ndarray) -> np.ndarray:
        return counts.astype(float) * self.fs.accel_mps2_per_count

    def _enter(self, state: State) -> None:
        self.state = state
        self.visited.append(state)
        self._hold = 0

    # -- main entry point ------------------------------------------------
    def step(self, gyro_counts: np.ndarray, accel_counts: np.ndarray):
        omega = self._to_rad_s(gyro_counts)
        accel = self._to_mps2(accel_counts)
        self.n += 1

        if self.state is State.IDLE:
            self._step_idle(omega, accel)
            return None

        if self.state is State.DONE:
            return None

        self._integrate(omega, accel)

        if self.state is State.ADDRESS:
            self._step_address(omega)
        elif self.state is State.BACKSWING:
            self._step_backswing(omega, accel)
        elif self.state is State.DOWNSWING:
            self._step_downswing(omega, accel)
        elif self.state is State.IMPACT:
            self._step_impact(omega, accel)
        elif self.state is State.FOLLOWTHROUGH:
            return self._step_followthrough(omega, accel)
        return None

    def _step_idle(self, omega, accel) -> None:
        """Wait for stillness, then capture both references at once.

        Parent spec 7.1: entering ADDRESS captures the gravity vector g0 and the
        gyro bias, both as means over the SAME stillness window. Averaging
        gravity over a longer span than the stillness test covers would mean
        averaging over motion the stillness test never vetted.
        """
        window = int(self.th.stillness_window_s / self.dt)
        self._still.append(omega)
        self._still_accel.append(accel)
        if len(self._still) < window:
            return
        self._still = self._still[-window:]
        self._still_accel = self._still_accel[-window:]

        recent = np.array(self._still)
        if np.all(recent.std(axis=0) < self.th.stillness_gyro_std_rad):
            self.bias = recent.mean(axis=0)
            self.g0 = np.array(self._still_accel).mean(axis=0)
            self.address_captured = True
            self.q = quat.identity()
            self._enter(State.ADDRESS)

    def _step_address(self, omega) -> None:
        """Detect the backswing, and record where it actually started.

        Confirming a backswing needs a threshold crossing plus a hold, and both
        are late -- the rate has to climb from zero to 8 deg/s and then persist
        for 40 ms. The stroke began earlier, when the rate first left zero.

        That latency does not cancel in the tempo ratio, because impact is
        detected within one sample while backswing start is ~54 samples late.
        Measured on the noiseless generator, taking the confirmation instant as
        the start gives a worst tempo error of 0.351 against the 0.05 target in
        parent spec section 3 -- seven times over, systematically biased low,
        and worsening as tempo ratio rises. Back-tracking to the last quiet
        sample brings it to 0.034.

        `_last_quiet_n` reads only past samples, so this is a ring buffer rather
        than lookahead and remains implementable on-device.
        """
        corrected = omega - self.bias

        # Measure the swing using only the component PERPENDICULAR to the shaft.
        #
        # Face rotation is rotation ABOUT the shaft axis (body Z), so the full
        # 3-axis magnitude mixes it into the swing measurement, and the onset
        # threshold then fires earlier for a putter whose face turns more. That
        # is segmentation keying off rotation amplitude, which invariant 1
        # prohibits -- measured as a one-sample shift in the recorded stroke
        # start between an arced and a zero-torque putter, identical in every
        # other respect.
        #
        # The perpendicular component is exactly |theta_dot|, because the swing
        # contributes theta_dot * (sin phi, cos phi, 0) and sin^2 + cos^2 = 1.
        # It is independent of face rotation by construction, not by tuning.
        magnitude = np.linalg.norm(corrected[:2])

        # Back-extrapolate the onset instead of taking the last quiet sample.
        #
        # The stroke begins where the rate is zero, but any threshold is crossed
        # later, by an amount that depends on the threshold itself -- so the
        # recorded start inherits the arbitrariness of a guessed constant, which
        # is what invariant 5 warns about. The rate rises essentially linearly
        # out of rest, so extrapolating the two samples that straddle the
        # threshold back to zero recovers the onset to a fraction of a sample
        # and barely depends on where the threshold sits.
        #
        # Uses the previous sample only: past data, one stored float.
        if magnitude < self.th.onset_gyro_rad:
            self._last_quiet_n = self.n
        elif self._prev_magnitude is not None and self._prev_magnitude < self.th.onset_gyro_rad:
            rise = magnitude - self._prev_magnitude
            if rise > 0.0:
                self._last_quiet_n = self.n - 1 - self._prev_magnitude / rise
        self._prev_magnitude = magnitude

        if magnitude > self.th.backswing_gyro_rad:
            self._hold += 1
            if self._hold * self.dt >= self.th.backswing_hold_s:
                self.i_backswing_start = self._last_quiet_n
                self._enter(State.BACKSWING)
        else:
            self._hold = 0

    def _integrate(self, omega, accel) -> None:
        """Attitude integration with the accelerometer correction gain driven
        by state.

        Parent spec 7.2 and invariant 2: between BACKSWING and FOLLOWTHROUGH the
        accelerometer reads gravity plus stroke acceleration, so a stock
        complementary correction is dragged off attitude by the very motion it
        is meant to measure. Drift is bounded instead by the short integration
        window and the bias null at address.
        """
        # FOLLOWTHROUGH counts as in-stroke. The putter is still moving fast
        # there, so the accelerometer is still reading gravity plus stroke
        # acceleration. Parent spec 7.2 restores the gain in IDLE and ADDRESS
        # only -- those are the two states where the device is actually still.
        # The state label alone is not enough. Confirming a backswing takes a
        # threshold crossing plus a 40 ms hold, so the machine still reads
        # ADDRESS for roughly 53 samples AFTER the stroke has physically begun.
        # Correcting against the accelerometer in that window is precisely the
        # failure invariant 2 describes -- the accelerometer is already reading
        # gravity plus stroke acceleration -- and the error it injects is then
        # frozen in, because the gain is zero for the rest of the stroke.
        #
        # Measured: a straight stroke with zero face angle must produce zero
        # lateral face-point displacement, for a geometric reason. It produced
        # 25.5 mm. With this stillness gate it produces 0.
        #
        # So the gain is gated on the device actually being still, using the
        # same onset threshold the back-tracking uses. During genuine stillness
        # this changes nothing: attitude was just initialized from g0, so the
        # correction is already a no-op. Its only effect was in the lag window,
        # where it was harmful.
        in_stroke = self.state in (State.BACKSWING, State.DOWNSWING,
                                   State.IMPACT, State.FOLLOWTHROUGH)
        moving = np.linalg.norm(omega - self.bias) > self.th.onset_gyro_rad
        gain = self.th.accel_gain_stroke if (in_stroke or moving) else self.th.accel_gain_static

        # Trapezoidal, not rectangle: average the rate across the interval being
        # integrated. That requires the sample at the END of the interval, which
        # is one sample of LATENCY, not lookahead -- the firmware integrates the
        # interval [i-1, i] when sample i arrives, and already has that sample in
        # hand. It costs one stored vector.
        #
        # Measured on the noiseless generator, worst case over 45 strokes:
        #   rectangle rule    0.0553 deg of face angle  (55% of the 0.1 deg budget)
        #   trapezoidal rule  0.0013 deg                 (1.3%)
        # a 41x improvement for one add and one multiply. Rectangle rule would
        # spend half the noiseless budget on nothing before noise even appears.
        corrected = omega - self.bias
        if self._prev_omega is None:
            self._prev_omega = corrected
        rate = 0.5 * (corrected + self._prev_omega)
        self._prev_omega = corrected
        self.q = quat.integrate(self.q, rate, self.dt)

        if gain > 0.0 and np.linalg.norm(accel) > 0.0:
            measured = accel / np.linalg.norm(accel)
            reference = self.g0 / np.linalg.norm(self.g0)
            predicted = quat.rotate(quat.conjugate(self.q), reference)
            # measured x predicted, not the other way round. q maps body to
            # reference and integrate() applies a BODY-frame rate, under which a
            # fixed reference vector seen from the body moves as dp/dt = p x w.
            # With w = m x p that gives p x (m x p) = m - p (p . m): p turns
            # toward m. The reversed product turns it away -- positive feedback,
            # measured taking a 2 deg tilt to 87 deg in 2 s at a gain of 2.0.
            # The stock gain hid it (2.000 -> 2.082 deg); see
            # test_accel_correction_pulls_a_tilted_attitude_back_to_gravity.
            error = np.cross(measured, predicted)
            # The gain is a RATE, in rad/s per unit of error -- not a per-sample
            # step. Writing `gain * error / self.dt` and then integrating over
            # dt cancels the dt and makes each sample apply a fixed rotation of
            # `gain * error`, so the correction actually applied per second
            # scales with the sample rate. The same constant then means
            # something different at every ODR, which is the class of silent
            # error invariant 5 exists to prevent.
            #
            # Measured when the stroke rate moved from 500 Hz to 896.8 Hz: worst
            # face-angle error over 0 to 0.5 dps of gyro noise went from 0.0137
            # to 0.2358 deg, and the degradation curve stopped being monotonic.
            # Nothing about the sensor got worse -- the correction simply ran
            # twice as hard.
            self.q = quat.integrate(self.q, gain * error, self.dt)

        # Face-point velocity from the rigid-body relation (parent spec 7.4).
        # Only path needs the lever arm; face angle and tempo do not, because
        # angular velocity is identical at every point of a rigid body.
        r_body = np.array([0.0, 0.0, -self.lever_arm])
        v_face = np.cross(corrected, r_body)
        rotation = quat.to_matrix(self.q)
        self._face_track.append(rotation @ v_face * self.dt)
        if self._track_first_n is None:
            self._track_first_n = self.n
        self._track_last_n = self.n

        # ...and the part of the face's motion that comes from the sensor
        # translating rather than rotating in place. The putter swings about
        # the hands, so the face travels on `r + d`; dropping `d` is what makes
        # the arc 61% of truth.
        # A per-sample increment, exactly like the track beside it: both are
        # summed once, at the end. Accumulating a running total here and then
        # summing it again integrates the translation twice, which shows up as
        # an arc of twenty-four metres rather than fourteen millimetres.
        self._track_matrix.append(rotation @ skew(corrected) * self.dt)

        # Feed the pivot fit, but only before impact. The impact impulse is a
        # 60 g spike that saturates the accelerometer by design (parent spec
        # 6.4) and is not rigid-body motion about anything; including it would
        # let a trigger corrupt a measurement.
        if self.state in (State.BACKSWING, State.DOWNSWING):
            gravity_body = quat.rotate(quat.conjugate(self.q), self.g0)
            self._pivot.update(corrected, accel - gravity_body)

    def _dominant_axis(self) -> int:
        recent = np.array(self._omega_history[-25:])
        return int(np.argmax(np.abs(recent).mean(axis=0)))

    def _step_backswing(self, omega, accel) -> None:
        """Detect the transition, and record where the direction actually flipped.

        The dominant axis and its backswing sign are frozen on entry rather than
        recomputed per sample, so a late-stroke wobble cannot silently reinterpret
        which axis the stroke is about.

        The magnitude gate only CONFIRMS a reversal, guarding against sign
        chatter while the rate passes through zero. The instant recorded is the
        last sample still carrying the backswing's sign, so the confirmation
        delay does not bias the tempo ratio -- same reasoning as the backswing
        onset above.

        Note what is NOT here: nothing keys off how far the face rotated.
        Segmentation is timing, direction and acceleration only (invariant 1).
        """
        corrected = omega - self.bias
        self._omega_history.append(corrected)

        if self._backswing_axis is None:
            self._backswing_axis = self._dominant_axis()
            self._backswing_sign = np.sign(corrected[self._backswing_axis])
            self._last_same_sign_n = self.n
            return

        axis = self._backswing_axis
        if np.sign(corrected[axis]) == self._backswing_sign:
            self._last_same_sign_n = self.n
        elif abs(corrected[axis]) > self.th.transition_gyro_rad:
            # The rate crossed zero somewhere BETWEEN the last same-sign sample
            # and the next one, so the last same-sign sample is the near edge of
            # the bracket, not the crossing. Taking it directly biases the
            # transition half a sample early and, because backswing and
            # downswing sit on opposite sides of it, that half sample is
            # subtracted from one and added to the other -- it enters the tempo
            # ratio twice, with the same sign.
            self.i_transition = self._last_same_sign_n + 0.5
            self._enter(State.DOWNSWING)

    def _step_downswing(self, omega, accel) -> None:
        if np.linalg.norm(accel) > self.th.impact_accel_mps2:
            self._impact_window = [(self.n, self.q.copy())]
            self._impact_track_index = len(self._face_track)
            self._enter(State.IMPACT)

    def _step_impact(self, omega, accel) -> None:
        """Take the impact instant as the MIDDLE of the acceleration spike.

        The threshold fires on the spike's rising edge, one or more samples
        before the strike. Sampling attitude there includes residual pre-impact
        rotation, and because that leak couples through the shaft axis it scales
        with how fast the face was rotating -- measured as 0.037 deg of error for
        a straight stroke against 0.072 deg for an arced one. Accuracy that
        tracks arc type is a putter-type prior (invariant 1), even at that size,
        and it would grow on faster real strokes.

        The peak cannot be used: the spike saturates (parent spec 6.4, and a
        60 g impulse against a 16 g full scale can do nothing else), so several
        samples read the same clipped value and the peak carries no information.

        The midpoint of the above-threshold run is threshold-insensitive,
        saturation-robust, and unbiased for a symmetric impulse. Measured worst
        error falls from 0.0733 deg to 0.0012 deg, and the arc-type spread from
        0.0356 deg to 0.0004 deg.

        Cost is a few samples of latency against a 500 ms budget, and a short
        buffer of attitudes -- past data only.

        Phase 2 note: a real strike may not be symmetric, since the putter
        decelerates and then the ball departs. Whether the midpoint stays
        unbiased on real impulses is a question for the logged corpus, like
        every threshold here.
        """
        if np.linalg.norm(accel) > self.th.impact_accel_mps2:
            self._impact_window.append((self.n, self.q.copy()))
            return
        self.i_impact, self.q_impact = self._impact_window[len(self._impact_window) // 2]
        self._enter(State.FOLLOWTHROUGH)

    def _step_followthrough(self, omega, accel):
        if np.linalg.norm(omega - self.bias) < self.th.followthrough_gyro_rad:
            self._hold += 1
            if self._hold * self.dt >= self.th.followthrough_hold_s:
                self._enter(State.DONE)
                return self._compute()
        else:
            self._hold = 0
        return None

    def _face_angle_at_impact(self) -> float:
        """Rotation about measured gravity between address and impact.

        Attitude is integrated from identity at address, so `q_impact` is
        already the address-relative rotation (invariant 3 -- there is no
        external heading reference and none is implied here).

        Taking the twist about g0 rather than reading a body axis is what makes
        this correct for a tilted shaft: g0 is (0, sin lie, cos lie) in body
        coordinates, not the body Z axis, so shaft lie angle is handled by
        measurement rather than by a stored constant (parent spec 7.3).
        """
        return quat.twist_angle(self.q_impact, self.g0)

    def _compute(self) -> StrokeResult:
        backswing = (self.i_transition - self.i_backswing_start) * self.dt
        downswing = (self.i_impact - self.i_transition) * self.dt

        # Path must be measured in the GROUND plane, for the same reason face
        # angle is (parent spec 7.3). Body Y is tilted out of horizontal by the
        # lie angle, so reading lateral displacement straight off it mixes
        # vertical swing motion into the number. Gravity at address defines the
        # plane; the stroke's own net travel defines forward within it.
        #
        # Where the arc actually comes from: the swing axis is tilted by the lie
        # angle, so the head travels on a cone and its ground-plane projection
        # curves. It is NOT produced by face rotation -- a zero-torque putter and
        # a blade swung on the same plane trace the same path and differ only in
        # face rotation. Measured arc is identical across all three arc types to
        # three decimal places, and falls to 0.002 mm for a vertical shaft.
        # Solve for the pivot offset now the stroke is over, and add the
        # translation term it unlocks. `d` is the vector from the pivot to the
        # sensor, so the face swings on `r + d`: with a 0.85 m lever arm and a
        # 0.55 m pivot offset that is 1.4 m, and 0.85/1.4 = 0.607 is precisely
        # the arc shortfall this replaces.
        #
        # PORT NOTE: the 3x3 per sample costs 48 KB over a 1.5 s stroke at
        # 896.8 Hz, against 16 KB for the track alone. That is affordable on
        # this part but it is not free, and invariant 7 already has claims on
        # internal SRAM. If it becomes tight, the reduction is to assume the
        # pivot lies on the shaft axis, which collapses the matrix back to a
        # vector -- at the cost of an assumption about where the hands are that
        # the measurement currently does not need.
        if self._pivot_calibration is not None:
            self._pivot_calibration.fold(self._pivot)
            self._pivot_solution = self._pivot_calibration.solve()
        else:
            self._pivot_solution = self._pivot.solve()
        track_list = self._face_track
        if (self._pivot_solution is not None
                and self._pivot_solution.residual_fraction
                <= self.th.pivot_max_residual):
            d = self._pivot_solution.offset
            track_list = [v + m @ d for v, m in zip(self._face_track,
                                                    self._track_matrix)]

        track = (np.cumsum(np.array(track_list), axis=0)
                 if track_list else np.zeros((1, 3)))
        g_hat = self.g0 / np.linalg.norm(self.g0)
        horizontal = track - np.outer(track @ g_hat, g_hat)

        travel = horizontal[-1] - horizontal[0]
        distance = float(np.linalg.norm(travel))
        if distance == 0.0:
            arc, direction = 0.0, "straight"
        else:
            lateral = horizontal @ np.cross(g_hat, travel / distance)
            arc = float(np.ptp(lateral))
            before = lateral[: self._impact_track_index]
            after = lateral[self._impact_track_index:]
            delta = ((after.mean() if len(after) else 0.0)
                     - (before.mean() if len(before) else 0.0))
            if arc < self.th.path_straight_arc_m:
                direction = "straight"
            else:
                direction = "in-to-out" if delta > 0 else "out-to-in"

        return StrokeResult(
            face_angle_deg=np.degrees(self._face_angle_at_impact()),
            tempo_ratio=backswing / downswing,
            path_arc_m=arc,
            path_direction=direction,
        )
