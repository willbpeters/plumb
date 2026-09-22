"""Tests for the resting-noise statistics.

The estimators are tested against hand-computed values, not against each
other. A robust estimator that agreed with the raw one on every input would be
useless, and testing one by the other would never show it.
"""

import numpy as np
import pytest

from tools.rest_noise import position_sigma, robust_sigma, window_sigma


def test_robust_sigma_recovers_the_scale_of_a_clean_normal_sample():
    rng = np.random.default_rng(0)
    x = rng.normal(0.0, 2.0, 20000)
    assert robust_sigma(x) == pytest.approx(2.0, rel=0.05)


def test_robust_sigma_ignores_an_outlier_that_moves_the_raw_one():
    """This is the whole point of carrying both. A handful of large samples --
    a footfall, a scrambled read -- inflates the standard deviation without
    changing the median absolute deviation, so the two disagreeing is the
    signal that the distribution has a tail."""
    x = np.concatenate([np.random.default_rng(1).normal(0.0, 1.0, 10000),
                        np.full(20, 60.0)])
    assert x.std() > 2.0
    assert robust_sigma(x) == pytest.approx(1.0, rel=0.05)


def test_window_sigma_separates_a_quiet_stretch_from_a_disturbed_one():
    """A single sigma over a whole capture cannot tell a noisy sensor from a
    quiet sensor on a desk somebody walked past. Per-window sigma can."""
    quiet = np.random.default_rng(2).normal(0.0, 0.1, 500)
    disturbed = np.random.default_rng(3).normal(0.0, 5.0, 500)
    windows = window_sigma(np.concatenate([quiet, disturbed])[:, None], 500)
    assert windows.shape == (2, 1)
    assert windows[0, 0] == pytest.approx(0.1, rel=0.2)
    assert windows[1, 0] == pytest.approx(5.0, rel=0.2)


def test_position_sigma_finds_noise_that_depends_on_position_in_a_batch():
    """A stationary sensor cannot produce noise that depends on where in a
    transfer the sample sat. When it appears, the transfer is the source --
    which is what the FIFO read path was measured doing."""
    rng = np.random.default_rng(4)
    batches = np.zeros((300, 8, 1))
    batches[:, :4, :] = rng.normal(0.0, 0.2, (300, 4, 1))   # early: quiet
    batches[:, 4:, :] = rng.normal(0.0, 2.0, (300, 4, 1))   # late: not
    sigma = position_sigma(batches)
    assert sigma.shape == (8, 1)
    assert sigma[:4, 0].max() < 0.5
    assert sigma[4:, 0].min() > 1.5
