"""Tests for the device accelerometer calibration (parent spec 8.1).

Why it exists, measured: a 0.2 m/s^2 accelerometer bias on body Y moved the
arc at lie 5 from 1.055 to 1.296 of truth, and none of it is observable from
the stroke, because Y is the axis the stroke rotates about. See HANDOFF.md,
open defect 6. The board read 9.689 m/s^2 at rest against 9.81 during bring-up,
so an error of that order is already known to exist on this unit.

Ground truth is known: the readings are generated from a stated offset and
gain, and the solver has to find them.
"""

import numpy as np
import pytest

from plumb import quat
from plumb.calibration import (AccelCalibration, CalibrationError,
                               solve_accel_calibration)

G = 9.81
OFFSET = np.array([0.05, -0.12, 0.08])      # m/s^2, the order bring-up implies
GAIN = np.array([1.0 / 1.012, 1.0 / 0.991, 1.0 / 1.004])


def reading(g_body, noise=0.0, rng=None):
    """What an accelerometer with OFFSET and GAIN reports at rest, averaged.

    At rest it measures proper acceleration: +g along whichever direction is
    up. The calibration's contract is `true = (raw - offset) * gain`, so the
    raw reading is the inverse of that.
    """
    raw = np.asarray(g_body, dtype=float) / GAIN + OFFSET
    if noise:
        raw = raw + rng.normal(0.0, noise, 3)
    return raw


def six_faces(tilt_deg=0.0, seed=0):
    """A tumble: each face up in turn, each resting a little off square, as a
    printed fixture on a real bench will. Averaged over a second at 0.02 m/s^2
    noise per sample, 896.8 samples: 0.0007 m/s^2 left on each mean."""
    rng = np.random.default_rng(seed)
    means = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            up = np.zeros(3)
            up[axis] = sign
            tilt = quat.from_axis_angle(rng.normal(size=3),
                                        np.radians(tilt_deg) * rng.uniform(0.5, 1.0))
            means.append(reading(G * quat.rotate(tilt, up),
                                 noise=0.02 / np.sqrt(896.8), rng=rng))
    return np.array(means)


def test_recovers_offset_and_gain_from_a_square_tumble():
    cal = solve_accel_calibration(six_faces(), gravity=G)
    print(f"\n  offset error {1e3 * np.abs(cal.offset_mps2 - OFFSET).max():.2f} mm/s^2,"
          f" gain error {np.abs(cal.gain - GAIN).max():.1e}")
    assert cal.offset_mps2 == pytest.approx(OFFSET, abs=0.002)
    assert cal.gain == pytest.approx(GAIN, abs=2e-4)


def test_a_fixture_resting_a_few_degrees_off_square_does_not_matter():
    """The model is |gain * (raw - offset)| = g in every pose, which holds at
    any orientation -- the faces only have to span both signs of every axis.
    So a fixture that sits 5 degrees off costs nothing, and the calibration
    needs no precision jig."""
    cal = solve_accel_calibration(six_faces(tilt_deg=5.0, seed=3), gravity=G)
    assert cal.offset_mps2 == pytest.approx(OFFSET, abs=0.002)
    assert cal.gain == pytest.approx(GAIN, abs=2e-4)


def test_arbitrary_resting_orientations_also_work():
    rng = np.random.default_rng(7)
    ups = rng.normal(size=(12, 3))
    ups /= np.linalg.norm(ups, axis=1, keepdims=True)
    means = np.array([reading(G * u, 0.0007, rng) for u in ups])
    cal = solve_accel_calibration(means, gravity=G)
    assert cal.offset_mps2 == pytest.approx(OFFSET, abs=0.003)


def test_poses_that_do_not_span_every_axis_are_refused():
    """Offset and gain along an axis are only separable if that axis was seen
    pointing both up and down (or near it). A tumble that skipped a face must
    fail loudly -- a calibration that fits its own inputs and is wrong about
    the axis nobody measured is the silent failure this project keeps
    finding."""
    means = six_faces()
    # Six unknowns. With the -Y face gone, one pose sees Y, and one equation
    # cannot separate Y's offset from Y's gain.
    for missing in ([3], [2, 3]):
        with pytest.raises(CalibrationError):
            solve_accel_calibration(np.delete(means, missing, axis=0), gravity=G)


def test_a_board_that_was_moving_is_refused():
    """The fit residual says whether every pose was really at rest -- but only
    with more poses than unknowns. Six faces fit six unknowns exactly, so a
    bad pose there is absorbed into a wrong calibration with zero residual.
    Hence the extra poses, and the host tool asks for them."""
    rng = np.random.default_rng(11)
    extra = rng.normal(size=(2, 3))
    extra = G * extra / np.linalg.norm(extra, axis=1, keepdims=True)
    means = np.vstack([six_faces(), [reading(u) for u in extra]])
    solve_accel_calibration(means, gravity=G)            # clean: accepted
    means[4] = means[4] * 1.05                            # someone was holding it
    with pytest.raises(CalibrationError):
        solve_accel_calibration(means, gravity=G)


def test_six_poses_fit_exactly_and_say_they_could_not_check_themselves():
    cal = solve_accel_calibration(six_faces(), gravity=G)
    assert cal.degrees_of_freedom == 0


def test_calibration_round_trips_a_reading():
    cal = AccelCalibration(OFFSET, GAIN)
    truth = np.array([0.3, -9.7, 1.2])
    assert cal.apply(reading(truth)) == pytest.approx(truth)


def test_identity_calibration_changes_nothing():
    a = np.array([1.0, 2.0, 3.0])
    assert AccelCalibration.identity().apply(a) == pytest.approx(a)


def test_the_tool_tracks_which_faces_are_still_needed():
    from tools.accel_cal import faces_missing, summarise
    poses = [summarise(np.tile(m, (10, 1)), np.zeros((10, 3))) for m in six_faces()[:4]]
    assert faces_missing(poses) == ["+Z", "-Z"]
    poses.append(summarise(np.tile([0.0, 5.0, 8.0], (10, 1)), np.zeros((10, 3))))
    assert faces_missing(poses) == ["+Z", "-Z"], "a propped pose is not a face"


def test_the_tool_keeps_the_raw_pose_means_in_its_record():
    """Parent spec 11: store the measurement, not only the answer, so the
    calibration can be re-solved if the model changes."""
    import json

    from tools.accel_cal import record, summarise
    means = six_faces()
    poses = [summarise(np.tile(m, (10, 1)), np.zeros((10, 3))) for m in means]
    cal = solve_accel_calibration(means, gravity=G)
    saved = json.loads(json.dumps(record(poses, cal, G)))
    again = solve_accel_calibration([p["mean_mps2"] for p in saved["poses"]],
                                    gravity=saved["gravity_mps2"])
    assert again.offset_mps2 == pytest.approx(cal.offset_mps2)
