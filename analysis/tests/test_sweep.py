import numpy as np
import pytest

from plumb.sweep import (FACE_TARGET_DEG, detection_rate, noise_breakdown_dps,
                         run_sweep, worst_face_error)
from plumb.trajectory import ArcType

# Reduced grid: the full grid lives in sweep.main() and is too slow for pytest.
SMALL = dict(face_angles=(-5.0, 0.0, 2.0), tempos=(2.0,), biases=(0.0, 2.0), seeds=(1,))


@pytest.fixture(scope="module")
def clean_rows():
    return run_sweep(noises=(0.0,), **SMALL)


@pytest.fixture(scope="module")
def noisy_rows():
    return run_sweep(noises=(0.0, 0.05, 0.2), **SMALL)


def test_sweep_covers_the_grid(clean_rows):
    assert len(clean_rows) == 3 * len(ArcType) * 1 * 1 * 2 * 1
    assert {r.arc_type for r in clean_rows} == {a.name for a in ArcType}


def test_noiseless_recovery_is_far_inside_the_target(clean_rows):
    """The whole point of the harness. At zero noise the pipeline is an
    algebraic inverse of the forward model, so this should not be close."""
    assert detection_rate(clean_rows) == 1.0
    assert worst_face_error(clean_rows) < 0.1


def test_a_standing_bias_does_not_degrade_recovery(clean_rows):
    """Invariant 4: bias is re-nulled every stroke, so a 2 dps standing bias
    should be indistinguishable from none."""
    by_bias = {}
    for r in clean_rows:
        by_bias.setdefault(r.gyro_bias_dps, []).append(abs(r.face_error_deg))
    assert set(by_bias) == {0.0, 2.0}
    assert max(by_bias[2.0]) == pytest.approx(max(by_bias[0.0]), abs=0.02)


def test_accuracy_degrades_monotonically_with_noise(noisy_rows):
    """Degradation must be graceful and predictable, not erratic."""
    worst = [worst_face_error([r for r in noisy_rows if r.gyro_noise_dps == n])
             for n in sorted({r.gyro_noise_dps for r in noisy_rows})]
    assert worst == sorted(worst), f"error is not monotonic in noise: {worst}"


def test_breakdown_point_is_reportable(noisy_rows):
    """Definition of done: we can STATE the noise level at which the target is
    exceeded. This asserts the number exists and is sane, not that it is large."""
    limit = noise_breakdown_dps(noisy_rows)
    assert limit >= 0.0
    assert limit in {r.gyro_noise_dps for r in noisy_rows}


def test_sweep_is_reproducible():
    small = run_sweep(noises=(0.2,), **SMALL)
    again = run_sweep(noises=(0.2,), **SMALL)
    assert small == again
