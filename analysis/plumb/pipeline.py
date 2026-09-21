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


@dataclass
class StrokeResult:
    face_angle_deg: float
    tempo_ratio: float
    path_arc_m: float
    path_direction: str


class Pipeline:
    def __init__(self, thresholds: Thresholds, full_scale: FullScale,
                 lever_arm_m: float = 0.85):
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
        self._impact_window: list[tuple[int, np.ndarray]] = []

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
        magnitude = np.linalg.norm(corrected)

        if magnitude < self.th.onset_gyro_rad:
            self._last_quiet_n = self.n

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
        in_stroke = self.state in (State.BACKSWING, State.DOWNSWING,
                                   State.IMPACT, State.FOLLOWTHROUGH)
        gain = self.th.accel_gain_stroke if in_stroke else self.th.accel_gain_static

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
            error = np.cross(predicted, measured)
            self.q = quat.integrate(self.q, gain * error / self.dt, self.dt)

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
        return StrokeResult(
            face_angle_deg=np.degrees(self._face_angle_at_impact()),
            tempo_ratio=backswing / downswing,
            path_arc_m=0.0,         # Task 8
            path_direction="unknown",
        )
