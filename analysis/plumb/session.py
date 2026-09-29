"""The device loop: one stroke after another.

A Pipeline is one stroke, and DONE and ABANDONED are terminal. The device runs
indefinitely, so something has to start the next one -- and carry the pivot
calibration across, since the pivot is the golfer's, learned over a session
(parent spec 8.4). That is all this is. The firmware does the same
(firmware/components/plumb/src/session.c).
"""

from plumb.calibration import AccelCalibration
from plumb.pipeline import Pipeline, State, Thresholds
from plumb.pivot import PivotCalibration
from plumb.sensor import FullScale
from plumb.trajectory import SAMPLE_RATE_HZ


class Session:
    def __init__(self, thresholds: Thresholds, full_scale: FullScale,
                 sample_rate_hz: float = SAMPLE_RATE_HZ, lever_arm_m: float = 0.85,
                 accel_calibration: AccelCalibration | None = None):
        self._args = (thresholds, full_scale)
        self._kwargs = dict(lever_arm_m=lever_arm_m,
                            accel_calibration=accel_calibration,
                            sample_rate_hz=sample_rate_hz)
        self.pivot_calibration = PivotCalibration()
        # (reason, state it was abandoned from), in order.
        self.abandoned: list[tuple[str, State]] = []
        self.last_pipeline: Pipeline | None = None
        # Stream positions, so a pipeline's own sample numbers can be placed.
        self.samples = 0
        self.pipeline_start = 0
        self.last_pipeline_start = 0
        self.pipeline = self._fresh()

    def _fresh(self) -> Pipeline:
        return Pipeline(*self._args, pivot_calibration=self.pivot_calibration,
                        **self._kwargs)

    def step(self, gyro_counts, accel_counts):
        """One sample. Returns a StrokeResult on the sample a stroke finishes."""
        result = self.pipeline.step(gyro_counts, accel_counts)
        self.samples += 1
        if self.pipeline.state is State.ABANDONED:
            self.abandoned.append((self.pipeline.abandon_reason,
                                   self.pipeline.abandoned_from))
        if self.pipeline.state in (State.DONE, State.ABANDONED):
            self.last_pipeline = self.pipeline
            self.last_pipeline_start = self.pipeline_start
            self.pipeline_start = self.samples
            self.pipeline = self._fresh()
        return result
