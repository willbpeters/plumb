"""Tests for the axis and sign check (spec 9.1).

Every test here compares against a rotation whose angle is known because the
test generated it. A test that only checked the integrator against itself would
pass just as happily with the handedness inverted, which is the single thing
this tool exists to catch.
"""

import numpy as np
import pytest

from tools.axis_check import (Axis, integrate_rotation, is_deliberate_turn,
                              rotation_verdict, vertical_axis)

GRAVITY = 9.81
RATE_HZ = 906.86


def resting_accel(up_axis: int, up_sign: int, tilt_deg: float = 0.0):
    """Mean accelerometer reading for a board resting with one axis up.

    A sensor at rest measures proper acceleration, so the axis pointing UP
    reads +1 g, not -1 g.
    """
    a = np.zeros(3)
    a[up_axis] = up_sign * GRAVITY * np.cos(np.radians(tilt_deg))
    a[(up_axis + 1) % 3] = GRAVITY * np.sin(np.radians(tilt_deg))
    return a


def rotation_record(axis: int, degrees: float, rate_hz=RATE_HZ, still_s=1.0,
                    turn_s=1.5, bias=(1.86, 2.88, 0.08), noise_dps=0.28,
                    seed=0):
    """A still stretch, then a smooth turn of exactly `degrees` about `axis`.

    The turn is a raised cosine, so it starts and ends at zero rate like a hand
    movement does, and its integral is exact by construction.
    """
    rng = np.random.default_rng(seed)
    still_n = int(still_s * rate_hz)
    turn_n = int(turn_s * rate_hz)

    t = np.arange(turn_n) / rate_hz
    shape = 1.0 - np.cos(2.0 * np.pi * t / turn_s)      # integrates to turn_s
    rate = shape * (degrees / turn_s)

    gyro = np.zeros((still_n + turn_n + still_n, 3))
    gyro[still_n:still_n + turn_n, axis] = rate
    gyro += np.array(bias)
    gyro += rng.normal(0.0, noise_dps, gyro.shape)
    return gyro, np.array(bias)


def test_vertical_axis_identifies_which_way_up_the_board_is():
    for axis in range(3):
        for sign in (+1, -1):
            found = vertical_axis(resting_accel(axis, sign))
            assert found == Axis(axis, sign)


def test_vertical_axis_rejects_a_board_that_is_not_resting_square():
    """40 degrees off is not a face, and reporting an axis for it would invite
    a sign check against a direction that is not the one being rotated about."""
    assert vertical_axis(resting_accel(2, 1, tilt_deg=40.0)) is None


def test_vertical_axis_rejects_a_board_that_is_not_at_rest():
    """Held in a hand and moving, the magnitude is no longer one g, and
    nothing about the reading means what it appears to mean."""
    assert vertical_axis(np.array([0.0, 0.0, 3.0 * GRAVITY])) is None


def test_integration_recovers_a_known_ninety_degree_turn():
    gyro, bias = rotation_record(axis=2, degrees=90.0)
    result = integrate_rotation(gyro, RATE_HZ, bias, noise_dps=0.28)
    assert result is not None
    assert result.angles_deg[2] == pytest.approx(90.0, abs=1.0)
    assert result.dominant == Axis(2, +1)


def test_integration_recovers_the_sign_of_a_reversed_turn():
    """The whole point. An integrator that lost the sign would pass every
    magnitude test in this file."""
    gyro, bias = rotation_record(axis=0, degrees=-90.0)
    result = integrate_rotation(gyro, RATE_HZ, bias, noise_dps=0.28)
    assert result.angles_deg[0] == pytest.approx(-90.0, abs=1.0)
    assert result.dominant == Axis(0, -1)


def test_uncorrected_gyro_bias_would_swamp_the_answer():
    """Why the bias is removed before integrating, in numbers: 2.88 dps over
    the 3.5 s record is 10 degrees of pure error on a channel that did not
    move. This is invariant 4 in miniature."""
    gyro, bias = rotation_record(axis=0, degrees=90.0)
    uncorrected = integrate_rotation(gyro, RATE_HZ, np.zeros(3), noise_dps=0.28)
    corrected = integrate_rotation(gyro, RATE_HZ, bias, noise_dps=0.28)
    assert abs(uncorrected.angles_deg[1]) > 3.0
    assert abs(corrected.angles_deg[1]) < 0.5


def test_cross_axis_response_is_reported_not_hidden():
    gyro, bias = rotation_record(axis=1, degrees=90.0)
    result = integrate_rotation(gyro, RATE_HZ, bias, noise_dps=0.28)
    assert result.cross_ratio < 0.02


def test_a_still_record_reports_no_rotation_rather_than_a_tiny_one():
    """Nothing moved, so there is no rotation to report. Returning a small
    angle instead would let a missed prompt read as a measurement."""
    gyro, bias = rotation_record(axis=2, degrees=0.0, turn_s=0.01)
    assert integrate_rotation(gyro, RATE_HZ, bias, noise_dps=0.28) is None


def test_the_motion_threshold_scales_with_the_measured_noise():
    """The trigger is derived from the noise measured in the same run, not
    chosen. On a board with a much worse floor, a rotation that would have
    triggered at 0.28 dps must not trigger on noise alone."""
    gyro, bias = rotation_record(axis=2, degrees=0.0, noise_dps=3.0, seed=7)
    assert integrate_rotation(gyro, RATE_HZ, bias, noise_dps=3.0) is None
    # ...and a real turn still does, on the same noisy board.
    gyro, bias = rotation_record(axis=2, degrees=90.0, noise_dps=3.0, seed=8)
    result = integrate_rotation(gyro, RATE_HZ, bias, noise_dps=3.0)
    assert result.angles_deg[2] == pytest.approx(90.0, abs=5.0)


def test_verdict_passes_when_the_turn_matches_the_right_hand_rule():
    """Turned anticlockwise seen from above, about an axis whose POSITIVE
    direction points up: the matching gyro channel must read positive."""
    assert rotation_verdict(Axis(2, +1), Axis(2, +1)).ok
    # Same board resting the other way up: +Z now points down, so the same
    # anticlockwise turn is negative about +Z.
    assert rotation_verdict(Axis(2, -1), Axis(2, -1)).ok


def test_verdict_fails_a_left_handed_triad():
    verdict = rotation_verdict(Axis(2, +1), Axis(2, -1))
    assert not verdict.ok
    assert "sign" in verdict.detail.lower()


def test_verdict_fails_when_the_wrong_channel_responded():
    verdict = rotation_verdict(Axis(2, +1), Axis(0, +1))
    assert not verdict.ok
    assert "channel" in verdict.detail.lower()


def test_a_brief_transient_is_not_a_quarter_turn():
    """Seen on hardware: something knocked the desk, the 20-sigma trigger
    fired, the integral came to +0.0 degrees, and it was recorded as a FAILED
    axis check. A verdict on a turn nobody made is worse than no verdict."""
    gyro, bias = rotation_record(axis=0, degrees=2.0, turn_s=0.05)
    result = integrate_rotation(gyro, RATE_HZ, bias, noise_dps=0.28)
    assert result is not None          # motion did happen
    assert not is_deliberate_turn(result)


def test_a_quarter_turn_is_a_deliberate_turn():
    gyro, bias = rotation_record(axis=0, degrees=90.0)
    assert is_deliberate_turn(integrate_rotation(gyro, RATE_HZ, bias, 0.28))


def test_a_quarter_turn_the_other_way_is_also_a_deliberate_turn():
    """Magnitude, not sign -- turning it the wrong way is a FAIL to report,
    not an input to discard."""
    gyro, bias = rotation_record(axis=0, degrees=-90.0)
    assert is_deliberate_turn(integrate_rotation(gyro, RATE_HZ, bias, 0.28))
