"""Pivot-offset estimation, for the path metric.

The problem this solves. Parent spec section 7.4 computes face velocity as
`omega x r`, which is the rigid-body relation with the sensor's own velocity
dropped. The sensor does translate: a putting stroke rotates about the hands,
not about the butt of the grip, so the face swings on `r + d` where `d` runs
from the pivot to the sensor. With the harness's defaults that is 1.4 m against
the 0.85 m assumed, and 0.85 / 1.4 = 0.607 -- which is exactly the 61% arc
shortfall pinned in test_pipeline.py. The diagnosis was already written down;
what was missing was a way to measure `d`.

Why this way. Estimating the pivot per sample was tried and abandoned because
it needs the gyro differentiated, which amplifies noise by 1/dt and collapses
around 0.1 dps -- this sensor's own floor. But the relation

    a_body = omega_dot x d + omega x (omega x d)                    (spec 2)

is LINEAR in d, so a stroke's worth of samples is an over-determined linear
system rather than a sequence of independent guesses. Three unknowns against
roughly a thousand samples: the noise that defeated the per-sample estimate
averages down by the square root of the sample count, and the centripetal term
carries no derivative at all. At the top of a downswing that term alone is
around 6 m/s^2, against an accelerometer resolution of 0.005.

What it cannot see, and why that needs more than a rank test. A rotation about
an axis says nothing about the offset ALONG that axis -- an offset parallel to
omega produces no acceleration. For a single-axis swing the system is exactly
rank two and the minimum-norm answer leaves that component at zero, which is
right.

The dangerous case is the one in between. A stroke that rotates the face a
little excites the third direction a little, so its eigenvalue is small but not
zero, and a rank test passes it. Measured on the harness: a near-zero-rotation
putter produced an offset of 1.04 m ACROSS the shaft, against a true zero. The
arc survived it, because the same near-degeneracy that let the component be
wrong also made the answer insensitive to it -- but a metre of nonsense sitting
in a reported quantity is not something to leave in place because it happened
not to matter here.

So each eigen-direction is kept only if its coefficient is larger than its own
uncertainty, estimated from the fit's residual. That is a measurement of what
the stroke actually observed rather than a threshold on what it ought to look
like, and it is self-calibrating: a quieter sensor keeps more directions.

Why the angular rate is filtered first. The noise here is in the DESIGN matrix,
not only in the measurement: omega and omega_dot both build K. Least squares
with a noisy design matrix is biased toward zero -- feed it the raw signal at
this board's 0.28 dps floor and the estimate collapses to a few millimetres,
which is exactly the collapse the per-sample attempt reported. The centripetal
term is not the problem: 0.005 rad/s of noise against a 3.5 rad/s peak is 0.14%.
The derivative is, because differencing white noise scales it by sqrt(2)/dt,
which at 896.8 Hz turns 0.005 rad/s into 6 rad/s^2 against a true peak near 20.

So omega is low-passed before K is built. A putting stroke is a ~1.5 s motion
whose content sits well under 20 Hz -- the same reasoning that set the gyro's
hardware LPF -- so this costs no signal. The filter is one pole and causal, and
it is used ONLY for the pivot fit: filtering the rate that feeds attitude
integration would delay the attitude and bias the face angle.

BOTH sides are filtered, and that is not symmetry for its own sake: filtering
only K leaves K lagging b by the filter's phase, which showed up as a 1.5% bias
on otherwise perfect data. It is not exact, though. `d` is a constant, so
`LPF[K d] = LPF[K] d` -- but what is built here is K from the filtered rate,
and the centripetal term is quadratic in it, so K(LPF[omega]) is not LPF[K].
That, together with the filters starting mid-stroke (the fit is fed from
BACKSWING entry, not from rest), is the ~1% left on noiseless data.

Filtering the built matrix instead was tried (2026-09-25), and measured on the
test_pivot harness, mean of five seeds, error in the recovered offset:

    fc 2.0 Hz                          noiseless   0.28 dps   2.8 dps
    rate filtered (this code)            0.98%       3.15%     22.7%
    matrix filtered, started mid-stroke  3.70%       2.43%     22.8%
    matrix filtered, started from rest   0.04%       6.59%     24.2%

Exact on noiseless data only when every filter starts from rest, which the
pipeline cannot do, and no better at this board's noise floor either way. The
rate filter stays.

Nothing here keys off face rotation, so nothing here carries a putter-type
prior (invariant 1). The pivot is swing geometry: where the golfer's hands are.
"""

from dataclasses import dataclass

import numpy as np

# Relative cutoff for treating an eigenvalue as exactly zero. A numerical
# guard against dividing by a rounding error, nothing more -- the real decision
# about which directions were observed is the significance test below.
SINGULAR_VALUE_CUTOFF = 1e-12

# How many standard errors a direction's coefficient must clear to be reported.
# Two is the usual bar for "distinguishable from nothing", and the result is
# not sensitive to it: the observed directions clear it by orders of magnitude
# and the unobserved ones miss by orders of magnitude.
SIGNIFICANCE_SIGMAS = 2.0

# Below this many samples there is no averaging to speak of and the estimate is
# the per-sample one that was already shown not to work.
MINIMUM_SAMPLES = 50

# Low-pass corner for the rate that builds K, in Hz. Not a detection threshold:
# it is a filter design, and it sits on a trade-off that was measured rather
# than argued. Error in the recovered offset, mean of five seeds:
#
#     fc Hz   noiseless   0.28 dps   2.8 dps      <- gyro noise
#      0.75      17.13%     10.03%     7.65%
#      1.50       1.24%      3.31%    17.01%
#      2.00       0.98%      3.15%    22.54%      <- chosen
#      3.00       1.82%      5.18%    33.70%
#      20.0       0.34%     20.79%    93.22%
#
# Below about 1.5 Hz the filter starts eating the stroke itself; above 3 Hz the
# derivative's noise starts biasing the fit toward zero. 0.28 dps is this
# board's measured resting floor (docs/bringup-results.md), so that is the
# column that decides it, and 1.5 to 3 Hz all clear the 10% arc target there.
# The choice is not knife-edge, which is the point of showing the table.
RATE_FILTER_HZ = 2.0


def skew(v) -> np.ndarray:
    """The matrix that performs `v x _`."""
    x, y, z = v
    return np.array([[0.0, -z, y],
                     [z, 0.0, -x],
                     [-y, x, 0.0]])


@dataclass
class PivotSolution:
    offset: np.ndarray          # d, from pivot to sensor, body frame, metres
    rank: int                   # 3 if the stroke excited every direction
    residual_fraction: float    # unexplained share of the measured acceleration
    samples: int


def solve_normal_equations(ata, atb, btb: float, samples: int
                           ) -> PivotSolution | None:
    """Least-squares offset from accumulated normal equations.

    Separate from the estimator because the same arithmetic serves one stroke
    and a hundred: summing the normal equations across strokes IS
    inverse-variance weighting, with no extra machinery and no stored history.
    """
    if samples < MINIMUM_SAMPLES or btb <= 0.0:
        return None

    # Symmetric and positive semi-definite by construction, so eigh gives the
    # decomposition without forming a pseudo-inverse of something
    # ill-conditioned.
    values, vectors = np.linalg.eigh(ata)
    largest = values.max()
    if largest <= 0.0:
        return None
    usable = values > SINGULAR_VALUE_CUTOFF * largest
    if not usable.any():
        return None

    coefficients = (vectors.T @ atb) / np.where(usable, values, 1.0)
    coefficients = np.where(usable, coefficients, 0.0)
    offset = vectors @ coefficients

    residual = max(0.0, btb - 2.0 * offset @ atb + offset @ ata @ offset)

    # Standard error per direction: sigma_b / sqrt(lambda). Three equations per
    # sample, three parameters. A direction the stroke barely excited has a
    # small lambda and therefore a large error bar, which is exactly the
    # statement that it was not measured.
    dof = max(1, 3 * samples - 3)
    sigma_b = np.sqrt(residual / dof)
    errors = np.where(usable, sigma_b / np.sqrt(np.where(usable, values, 1.0)),
                      np.inf)
    significant = usable & (np.abs(coefficients) > SIGNIFICANCE_SIGMAS * errors)

    return PivotSolution(
        offset=vectors[:, significant] @ coefficients[significant],
        rank=int(significant.sum()),
        residual_fraction=float(residual / btb),
        samples=samples,
    )


class PivotCalibration:
    """The pivot offset accumulated across strokes, for one golfer.

    Measured on the harness at this board's 0.28 dps noise floor, a SINGLE
    stroke recovers the offset to about 17%, and the arc that follows from it
    lands anywhere between 8% under truth and 84% over. That is not a usable
    per-stroke number, and it is the same conclusion the earlier attempt
    reached -- reached again from the other direction, because this estimator
    is not biased, it is noisy.

    Summed across strokes it converges: 12% after two, 2.4% after five, and a
    floor near 5% thereafter set by the filter bias rather than by noise. So
    the pivot is a property of the golfer to be learned over a session, not a
    property of the stroke to be measured in one -- which is what parent spec
    section 8.4 anticipated when it put auto-calibration there rather than in
    the per-stroke path.

    Costs nine floats and a counter, whatever the session length.
    """

    def __init__(self) -> None:
        self._ata = np.zeros((3, 3))
        self._atb = np.zeros(3)
        self._btb = 0.0
        self.samples = 0
        self.strokes = 0

    def fold(self, estimator: "PivotEstimator") -> None:
        """Add one completed stroke's evidence."""
        if estimator.samples < MINIMUM_SAMPLES:
            return
        self._ata += estimator._ata
        self._atb += estimator._atb
        self._btb += estimator._btb
        self.samples += estimator.samples
        self.strokes += 1

    def solve(self) -> PivotSolution | None:
        return solve_normal_equations(self._ata, self._atb, self._btb,
                                      self.samples)


class PivotEstimator:
    """Accumulates the normal equations for `d`, one sample at a time.

    Streaming, like everything else the firmware runs: nine accumulators and a
    three-vector, no history, no lookahead. The stroke is over before the
    system is solved, but nothing is stored while it runs.
    """

    def __init__(self, dt: float, filter_hz: float = RATE_FILTER_HZ) -> None:
        self.dt = dt
        # One-pole low pass: y += alpha (x - y). tau = 1 / (2 pi fc).
        tau = 1.0 / (2.0 * np.pi * filter_hz)
        self._alpha = dt / (tau + dt)
        self._filtered = None
        self._previous = None
        self._filtered_accel = None
        self._ata = np.zeros((3, 3))
        self._atb = np.zeros(3)
        self._btb = 0.0
        self.samples = 0

    def update(self, omega, a_body) -> None:
        """One sample.

        `a_body` is the sensor's own acceleration with gravity already removed
        -- the accelerometer reading minus the gravity direction rotated into
        the current body frame. Getting that subtraction wrong leaks up to
        9.81 m/s^2 into a signal of a few, so it is the caller's job and the
        caller has the attitude.

        The angular rate is filtered and differentiated here rather than by the
        caller, so that a caller cannot pass the unfiltered rate and quietly
        get an estimate biased to zero.
        """
        omega = np.asarray(omega, dtype=float)
        a_body = np.asarray(a_body, dtype=float)
        if self._filtered is None:
            self._filtered = omega.copy()
            self._previous = omega.copy()
            self._filtered_accel = a_body.copy()
            return
        self._previous = self._filtered
        self._filtered = self._filtered + self._alpha * (omega - self._filtered)
        self._filtered_accel = (self._filtered_accel
                                + self._alpha * (a_body - self._filtered_accel))
        omega_dot = (self._filtered - self._previous) / self.dt

        k = skew(omega_dot) + skew(self._filtered) @ skew(self._filtered)
        b = self._filtered_accel
        self._ata += k.T @ k
        self._atb += k.T @ b
        self._btb += float(np.dot(b, b))
        self.samples += 1

    def solve(self) -> PivotSolution | None:
        """This stroke's offset alone, or None when it did not show one.

        None rather than a small number: a motionless record contains no
        information about the pivot, and an answer produced from it would have
        a physically plausible magnitude and no physical meaning.

        For a number to actually use, prefer PivotCalibration -- one stroke is
        not enough at this sensor's noise floor, and the docstring there has
        the measurements.
        """
        return solve_normal_equations(self._ata, self._atb, self._btb,
                                      self.samples)
