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
        elif self.state is State.ADDRESS:
            self._step_address(omega)
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
        corrected = omega - self.bias
        if np.linalg.norm(corrected) > self.th.backswing_gyro_rad:
            self._hold += 1
            if self._hold * self.dt >= self.th.backswing_hold_s:
                self.i_backswing_start = self.n
                self._enter(State.BACKSWING)
        else:
            self._hold = 0
