"""tools/rate_check.py's two pure pieces, against known answers."""

import numpy as np
import pytest

from tools.rate_check import host_clock_rate, parse_rate_line

# Exactly as firmware/main/stream.c prints it.
LINE = ("# sample rate 906.9249 Hz +/- 3.5e-05 (fit, 1 sigma; crystal not included), "
        "measurement 2: 1023 points over 1.128 s, 1 rejected, 0 skipped, "
        "rms 0.40 us, max 3.84 us\n")


def test_parses_the_firmwares_rate_line():
    r = parse_rate_line("# measuring sample rate\n" + LINE)
    assert r == {"hz": 906.9249, "se": 3.5e-05, "used": 1023.0, "span": 1.128,
                 "rejected": 1.0, "skipped": 0.0, "rms": 0.40, "max": 3.84}


def test_no_rate_line_is_none_not_a_number():
    assert parse_rate_line("# sample rate: measuring\n") is None


def test_host_fit_recovers_the_rate_through_usb_delay():
    """Chunks arrive late by a millisecond or more, one-sided and irregular,
    like a USB serial bridge delivers them. The delay is stationary, so the
    slope survives it."""
    rng = np.random.default_rng(1)
    hz = 906.93
    indices = np.cumsum(rng.integers(20, 60, 1500))       # ~60 s of chunks
    arrivals = indices / hz + rng.exponential(0.002, indices.size) + 7.0
    fitted, se = host_clock_rate(arrivals, indices)
    assert fitted == pytest.approx(hz, abs=4 * se)
    # What the check is for: resolving a crystal difference of ~10 ppm. 2 ms
    # of scatter over 1500 chunks in 60 s predicts 2e-3 * sqrt(12/1500) / 60
    # = 3 ppm; the fit reports 2.7.
    assert 1e6 * se / hz < 5.0
