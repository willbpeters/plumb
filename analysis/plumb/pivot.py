"""Pivot-offset estimation, for the path metric.

The problem this solves. Parent spec section 7.4 computes face velocity as
`omega x r`, which is the rigid-body relation with the sensor's own velocity
dropped. The sensor does translate: a putting stroke rotates about the hands,
not about the butt of the grip, so the face swings on `r + d` where `d` runs
from the pivot to the sensor. With the harness's defaults that is 1.4 m against
the 0.85 m assumed, and 0.85 / 1.4 = 0.607 -- which is exactly the 61% arc
shortfall once pinned in test_pipeline.py.

THE RELATION USED. A point fixed in a body rotating about a fixed pivot moves
with world velocity `R [omega]x d`, where R is the attitude. The accelerometer
measures the derivative of that, so integrating it in the world frame gives

    v(t) = integral of R a_body dt  =  R [omega]x d  -  v0                (1)

which is LINEAR in d, with v0 the (unknown) velocity when the fit started. A
stroke's worth of samples is an over-determined system of three unknowns --
four with v0, which is eliminated by centring both sides on their means, so no
single sample anchors anything.

WHY NOT THE ACCELERATION FORM. The first version fit the derivative of (1),

    a_body = omega_dot x d + omega x (omega x d),

which needs omega differentiated. Differentiating white noise amplifies it,
so the rate was low-passed first -- and that bought a chain of problems, each
measured on the arced putter at lie 5, where the arc is smallest and the
errors show:

  - The filter could not be applied consistently. K is quadratic in the rate,
    so K(LPF[omega]) is not LPF[K], and the filters started mid-stroke. ~1% on
    the offset noiselessly, projected onto the weakest direction: +5% arc.
  - Errors-in-variables. Noise in the DESIGN matrix biases least squares
    toward zero, hardest in the least-excited direction. On the arced putter
    face rotation tracks the swing, so the rotation axis tilts ~19 deg off
    body Y and the weakest direction carries a real 0.18 m share of d; the
    derivative's noise power per sample (~0.006) matched that direction's
    signal (~0.006) and shrank it by half. The fake remainder sits off the
    shaft axis, where face rotation turns it into sideways face travel: 1.142
    of true arc against 1.010 for a straight putter on the same noise.
    Oracles pinned it -- the true rate in the fit took it to 1.049, the true
    acceleration changed nothing (1.137).

The velocity form needs no derivative and no filter, and is exact on
noiseless data (0.997-1.002 of true arc for every putter type).

TWO THINGS IT STILL HAS TO HANDLE.

  1. The start. Subtracting R0 [omega_0]x -- one noisy sample -- puts that
     sample's noise into every row for the whole stroke, a constant error no
     filter removes (low-passing made it WORSE: 1.153 -> 1.224 as the corner
     fell). Treating v0 as unknown removes it: 1.153 -> 1.079.
  2. Errors-in-variables again, now from the raw rate: the design is
     R [omega + n]x, whose noise contributes E[Nt N] = tr(S) I - S per sample
     for a gyro noise covariance S -- whatever the noise spectrum, since it is
     a per-sample second moment, so the hardware LPF does not invalidate it.
     S is MEASURED, in the same address stillness window that nulls the bias,
     and its expected contribution is subtracted from the normal equations
     ("corrected least squares"). 1.079 -> 1.002, and 0.999 at 0.56 dps.

WHAT IT CANNOT SEE. An offset along the rotation axis produces no motion, so a
single-axis swing is exactly rank two and the minimum-norm answer leaves that
component at zero, which is right. The dangerous case is a direction that is
nearly unobserved: after the noise is subtracted, what is left there is the
difference of two similar numbers. So a direction is kept only if its
corrected energy clears SIGNIFICANCE_SIGMAS times the uncertainty of the noise
estimate itself -- a statistic of this stroke's sample counts, not a tuned
threshold. Measured on one stroke at 0.28 dps: the straight putter's swing
axis has raw energy 0.039 against a noise share of 0.040-0.046 (rejected);
the arced putter's weakest direction has 0.11 against 0.04 (kept).

Nothing here keys off face rotation, so nothing carries a putter-type prior
(invariant 1). The pivot is swing geometry: where the golfer's hands are.

PORT NOTE. Centring is done from running sums (sum M^T M - S^T S / n), which
cancels: fine in double, but in single precision the weakest eigenvalue
(~0.07 against sums of ~300) keeps only ~4 significant digits. Use running
means (Welford) in the C, and measure it with the differential harness.
"""

from dataclasses import dataclass

import numpy as np

# Relative cutoff for treating an eigenvalue as exactly zero. A numerical
# guard against dividing by a rounding error, nothing more -- the real decision
# about which directions were observed is the significance test below.
SINGULAR_VALUE_CUTOFF = 1e-12

# How many standard errors a direction must clear to be reported -- both its
# corrected energy against the noise estimate's uncertainty, and its
# coefficient against its own error bar. This is a false-alarm rate, chosen,
# not fitted. Measured over 40 straight-putter strokes at 0.28 dps, the
# swing axis (truly unobservable) scores mean +0.30, SD 0.78 -- the
# uncertainty estimate is if anything conservative -- and one stroke in 40
# cleared 2.0 (at 2.26) and reported 0.17 m along an axis nothing measured,
# which is the 2.3% a one-sided 2-sigma test predicts. At 3 the rate is 0.13%.
# What it could cost: the arced putter's weakest direction, the one that
# matters, scores ~20, so nothing measurable. Revisit on the Phase 2 corpus,
# where the noise is not white.
SIGNIFICANCE_SIGMAS = 3.0

# Below this many samples there is no averaging to speak of.
MINIMUM_SAMPLES = 50


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
    residual_fraction: float    # unexplained share of the measured velocity
    samples: int


@dataclass
class NormalEquations:
    """One stroke's evidence, or a session's: everything the solve needs.

    `noise` is the expected contribution of gyro noise to `ata`, already
    included in it and to be subtracted; `noise_variance` is the variance of
    that estimate, in the same units squared. Summing across strokes is
    inverse-variance weighting for the signal and adds the variances for the
    noise estimate, so a session is just the elementwise sum.
    """

    ata: np.ndarray
    atb: np.ndarray
    btb: float
    samples: int
    noise: np.ndarray
    noise_variance: float

    @staticmethod
    def empty() -> "NormalEquations":
        return NormalEquations(np.zeros((3, 3)), np.zeros(3), 0.0, 0,
                               np.zeros((3, 3)), 0.0)

    def __add__(self, other: "NormalEquations") -> "NormalEquations":
        return NormalEquations(self.ata + other.ata, self.atb + other.atb,
                               self.btb + other.btb,
                               self.samples + other.samples,
                               self.noise + other.noise,
                               self.noise_variance + other.noise_variance)


def solve_normal_equations(eq: NormalEquations) -> PivotSolution | None:
    """Least-squares offset, corrected for noise in the design matrix."""
    if eq.samples < MINIMUM_SAMPLES or eq.btb <= 0.0:
        return None

    corrected = eq.ata - eq.noise
    # Symmetric by construction, so eigh gives the decomposition without
    # forming a pseudo-inverse of something ill-conditioned. Not necessarily
    # positive: subtracting an estimated noise can leave a small negative
    # eigenvalue in a direction that held nothing but noise.
    values, vectors = np.linalg.eigh(corrected)
    largest = values.max()
    if largest <= 0.0:
        return None
    noise_sd = np.sqrt(eq.noise_variance)
    usable = ((values > SINGULAR_VALUE_CUTOFF * largest)
              & (values > SIGNIFICANCE_SIGMAS * noise_sd))
    if not usable.any():
        return None

    safe = np.where(usable, values, 1.0)
    coefficients = np.where(usable, (vectors.T @ eq.atb) / safe, 0.0)
    offset = vectors @ coefficients

    # Residual of the fit against the RAW design. What the model leaves
    # unexplained, as a share of what there was to explain.
    residual = max(0.0, eq.btb - 2.0 * offset @ eq.atb
                   + offset @ eq.ata @ offset)

    # Standard error per direction: sigma_b / sqrt(lambda). Optimistic --
    # the rows of an integrated signal are not independent -- which is why it
    # is the second test and not the first.
    dof = max(1, 3 * eq.samples - 3)
    sigma_b = np.sqrt(residual / dof)
    errors = np.where(usable, sigma_b / np.sqrt(safe), np.inf)
    significant = usable & (np.abs(coefficients) > SIGNIFICANCE_SIGMAS * errors)

    return PivotSolution(
        offset=vectors[:, significant] @ coefficients[significant],
        rank=int(significant.sum()),
        residual_fraction=float(residual / eq.btb),
        samples=eq.samples,
    )


class PivotCalibration:
    """The pivot offset accumulated across strokes, for one golfer.

    One stroke carries only so much information about d, however good the
    estimator; summing the normal equations across strokes is
    inverse-variance weighting for free, with no stored history. The pivot is
    a property of the golfer to be learned over a session, not of the stroke
    -- which is where parent spec section 8.4 put auto-calibration.

    Costs a handful of floats and a counter, whatever the session length.
    """

    def __init__(self) -> None:
        self._eq = NormalEquations.empty()
        self.strokes = 0

    @property
    def samples(self) -> int:
        return self._eq.samples

    def fold(self, estimator: "PivotEstimator") -> None:
        """Add one completed stroke's evidence."""
        if estimator.samples < MINIMUM_SAMPLES:
            return
        self._eq = self._eq + estimator.normal_equations()
        self.strokes += 1

    def solve(self) -> PivotSolution | None:
        return solve_normal_equations(self._eq)


class PivotEstimator:
    """Accumulates the normal equations for `d`, one sample at a time.

    Streaming, like everything else the firmware runs: running sums, the
    integrated velocity and the previous acceleration -- no history, no
    lookahead. The stroke is over before the system is solved, but nothing
    is stored while it runs.
    """

    def __init__(self, dt: float) -> None:
        self.dt = dt
        self._velocity = np.zeros(3)
        self._previous = None
        self._mm = np.zeros((3, 3))
        self._my = np.zeros(3)
        self._yy = 0.0
        self._sum_m = np.zeros((3, 3))
        self._sum_y = np.zeros(3)
        self.samples = 0
        self._noise_cov = np.zeros((3, 3))
        self._noise_samples = 0

    def set_noise(self, covariance, samples: int) -> None:
        """The gyro noise covariance, per sample, in (rad/s)^2, as measured
        over `samples` samples of stillness. The pipeline supplies it from the
        same window that nulls the bias. Without it the fit is uncorrected,
        which is exact on noiseless data and biased on noisy data."""
        self._noise_cov = np.asarray(covariance, dtype=float)
        self._noise_samples = int(samples)

    def update(self, omega, a_body, rotation) -> None:
        """One sample.

        `a_body` is the sensor's own acceleration with gravity already removed
        -- the accelerometer reading minus the gravity direction rotated into
        the current body frame. Getting that subtraction wrong leaks up to
        9.81 m/s^2 into a signal of a few, so it is the caller's job and the
        caller has the attitude. `rotation` is that attitude as a matrix,
        body to the address frame.
        """
        a_world = rotation @ np.asarray(a_body, dtype=float)
        if self._previous is not None:
            # Trapezoidal, for the same reason attitude is: the rectangle rule
            # puts a half-sample lag between the velocity and the design.
            self._velocity = (self._velocity
                              + 0.5 * (a_world + self._previous) * self.dt)
        self._previous = a_world

        m = rotation @ skew(np.asarray(omega, dtype=float))
        y = self._velocity
        self._mm += m.T @ m
        self._my += m.T @ y
        self._yy += float(y @ y)
        self._sum_m += m
        self._sum_y += y
        self.samples += 1

    def normal_equations(self) -> NormalEquations:
        """This stroke's equations, centred to eliminate v0 in (1)."""
        n = self.samples
        if n == 0:
            return NormalEquations.empty()
        ata = self._mm - self._sum_m.T @ self._sum_m / n
        atb = self._my - self._sum_m.T @ self._sum_y / n
        btb = self._yy - float(self._sum_y @ self._sum_y) / n

        # Expected contribution of the gyro noise to ata. Centring removes one
        # sample's worth, hence n - 1. The uncertainty of that estimate is the
        # relative error of a variance from `_noise_samples` samples, plus the
        # fluctuation of the noise actually realised in this stroke's n --
        # both for white noise, which the board's own LPF makes optimistic.
        # That is recorded in HANDOFF.md as something the Phase 2 corpus has
        # to check.
        s = self._noise_cov
        noise = (n - 1) * (np.trace(s) * np.eye(3) - s)
        variance = 0.0
        if self._noise_samples > 1:
            relative = np.sqrt(2.0 / (self._noise_samples - 1) + 2.0 / n)
            variance = float((relative * np.trace(noise) / 3.0) ** 2)
        return NormalEquations(ata, atb, btb, n, noise, variance)

    def solve(self) -> PivotSolution | None:
        """This stroke's offset alone, or None when it did not show one.

        None rather than a small number: a motionless record contains no
        information about the pivot, and an answer produced from it would have
        a physically plausible magnitude and no physical meaning.
        """
        return solve_normal_equations(self.normal_equations())
