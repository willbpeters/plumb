"""Device accelerometer calibration: parent spec 8.1, the tier run once per unit.

Why path needs it, measured on the harness (HANDOFF.md, open defect 6). An
accelerometer bias reaches the arc two ways, and neither is fixable per
stroke:

  - It tilts the ground plane. `g0`, captured at address, is gravity plus the
    bias, so the plane the path is projected onto is tilted by bias / g and the
    head's vertical travel (~3 cm through a stroke) leaks into lateral. A bias
    on body Y -- the axis the stroke rotates about -- does this, and nothing in
    the motion can observe it. 0.2 m/s^2 there: arc 1.055 -> 1.296 at lie 5.
  - It leaks (I - R^T) b into the pivot fit, since `g0` cancels the bias only at
    the address orientation. Body X, 0.2 m/s^2: +8% arc.

Face angle is essentially immune (about 0.004 deg at 0.2 m/s^2). The board read
9.689 m/s^2 at rest against 9.81 in bring-up, so an error of that order exists
on this unit already; one orientation cannot say whether it is offset or gain.

THE MODEL. `true = (raw - offset) * gain`, per axis. At rest the true reading
is gravity, so in every resting pose |gain * (raw - offset)| = g. That holds at
ANY orientation, so the poses need only span both signs of every axis; the
fixture does not have to be square, and a few degrees off costs nothing.
Solved by Gauss-Newton from offset 0, gain 1.

Not modelled: cross-axis misalignment (the axes not being exactly orthogonal)
and temperature drift of the offset. Both are real on MEMS parts. The first
would need nine parameters and more poses; the second needs a measurement.
"""

from dataclasses import dataclass

import numpy as np

# Six unknowns: an offset and a gain per axis.
PARAMETERS = 6

# Largest acceptable misfit of any pose, in m/s^2. Not a detection threshold in
# the algorithm (invariant 5 is about those) but an acceptance bound on a
# calibration, derived from what the calibration is FOR: arc moves about 1.2%
# per 0.01 m/s^2 of bias on the worst axis at lie 5, so a pose that misfits by
# more than 0.02 m/s^2 says the result cannot be trusted to the accuracy path
# needs. A board resting still and averaged for a second misfits by ~0.001.
MAX_RESIDUAL_MPS2 = 0.02

# The Jacobian's smallest singular value, relative to its largest, below which
# a parameter is not determined by the poses given -- an axis never seen
# pointing both ways.
CONDITION_LIMIT = 1e-6


class CalibrationError(Exception):
    """The poses given cannot produce a calibration that can be trusted."""


@dataclass(frozen=True)
class AccelCalibration:
    offset_mps2: np.ndarray
    gain: np.ndarray
    # Poses beyond the six unknowns. Zero means the fit was exact and could not
    # check itself: a bad pose would have been absorbed without a trace.
    degrees_of_freedom: int = 0
    worst_residual_mps2: float = 0.0

    @staticmethod
    def identity() -> "AccelCalibration":
        return AccelCalibration(np.zeros(3), np.ones(3))

    def apply(self, accel_mps2) -> np.ndarray:
        return (np.asarray(accel_mps2, dtype=float) - self.offset_mps2) * self.gain


def solve_accel_calibration(resting_means, gravity: float,
                            max_residual_mps2: float = MAX_RESIDUAL_MPS2,
                            iterations: int = 50) -> AccelCalibration:
    """Offset and gain per axis from the mean reading in each resting pose.

    `resting_means` is (poses, 3), in m/s^2 as converted by FullScale but not
    otherwise corrected. `gravity` is local g, which sets the gain's absolute
    scale; it varies by about 0.5% over the Earth's surface, so pass the local
    value when it matters. Offsets do not depend on it to first order.
    """
    r = np.asarray(resting_means, dtype=float)
    if r.ndim != 2 or r.shape[1] != 3 or len(r) < PARAMETERS:
        raise CalibrationError(
            f"{len(r)} poses for {PARAMETERS} unknowns: rest the board on all "
            f"six faces at least")

    offset = np.zeros(3)
    gain = np.ones(3)
    for _ in range(iterations):
        u = gain * (r - offset)
        norm = np.linalg.norm(u, axis=1)
        residual = norm - gravity
        unit = u / norm[:, None]
        jacobian = np.hstack([-gain * unit, (r - offset) * unit])

        singular = np.linalg.svd(jacobian, compute_uv=False)
        if singular[-1] < CONDITION_LIMIT * singular[0]:
            raise CalibrationError(
                "the poses do not determine every axis: each axis has to be "
                "seen pointing both up and down")

        step, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
        offset = offset + step[:3]
        gain = gain + step[3:]
        if np.abs(step).max() < 1e-12:
            break

    residual = np.linalg.norm(gain * (r - offset), axis=1) - gravity
    worst = float(np.abs(residual).max())
    dof = len(r) - PARAMETERS
    if dof > 0 and worst > max_residual_mps2:
        pose = int(np.argmax(np.abs(residual)))
        raise CalibrationError(
            f"pose {pose} misfits by {worst:.3f} m/s^2 (limit "
            f"{max_residual_mps2}): the board was moving or being held. Repeat it.")
    return AccelCalibration(offset, gain, dof, worst)
