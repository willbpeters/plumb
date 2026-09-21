# analysis/tests/test_invariants.py
"""Tests that exist to keep the non-obvious decisions non-obvious.

Each of these fails if a specific invariant from CLAUDE.md is quietly removed.
They are documentation that executes.
"""

import numpy as np
import pytest

from plumb.pipeline import Pipeline, Thresholds
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import ArcType, StrokeParams, generate


def recover(stroke, thresholds=Thresholds(), sensor=SensorParams(), seed=1):
    traj = generate(stroke)
    out = simulate(traj, sensor, seed=seed)
    pipe = Pipeline(thresholds, out.full_scale, stroke.lever_arm_m)
    result = None
    for i in range(len(traj.time)):
        r = pipe.step(out.gyro_counts[i], out.accel_counts[i])
        if r is not None:
            result = r
    return result


def test_invariant_2_accel_correction_must_stay_off_during_the_stroke():
    """CLAUDE.md invariant 2.

    Between BACKSWING and FOLLOWTHROUGH the accelerometer reads gravity PLUS
    stroke acceleration. A stock Madgwick or Mahony filter trusts it as a
    gravity reference and gets dragged off attitude by the very motion it is
    supposed to be measuring.

    If this test ever passes with the gain restored, the gain schedule is not
    doing anything and the reasoning in parent spec 7.2 is wrong. Keeping it is
    how we know the schedule is load-bearing rather than superstition."""
    stroke = StrokeParams(face_angle_at_impact_deg=2.0)
    correct = recover(stroke)
    naive = recover(stroke, Thresholds(accel_gain_stroke=0.05))

    correct_error = abs(correct.face_angle_deg - 2.0)
    naive_error = abs(naive.face_angle_deg - 2.0)

    assert correct_error < 0.05
    assert naive_error > 10 * correct_error, (
        f"restoring the gain changed almost nothing "
        f"(correct {correct_error:.4f} deg, naive {naive_error:.4f} deg) -- "
        f"either the schedule is not load-bearing or the test is not exercising it"
    )


def test_invariant_4_bias_nulling_absorbs_a_standing_gyro_bias():
    """CLAUDE.md invariant 4.

    Drift is bounded by the short integration window plus re-nulling the gyro
    bias every stroke, NOT by accelerometer correction -- which is off during
    the stroke precisely because it cannot be trusted there.

    A 2 dps standing bias integrated over a ~1 s stroke is about 2 degrees of
    raw drift, twenty times the face-angle budget. Nulling it at address has to
    absorb essentially all of that."""
    stroke = StrokeParams(face_angle_at_impact_deg=2.0)
    clean = recover(stroke)
    biased = recover(stroke, sensor=SensorParams(gyro_bias_dps=2.0))

    assert abs(biased.face_angle_deg - 2.0) < 0.1
    assert abs(biased.face_angle_deg - clean.face_angle_deg) < 0.1


def test_invariant_1_no_threshold_keys_off_rotation_amplitude():
    """CLAUDE.md invariant 1.

    A zero-torque putter legitimately produces very little face rotation. Any
    threshold tuned on rotation AMPLITUDE would report that as a broken stroke.

    This checks the property structurally: the same stroke is run with face
    rotation scaled across two orders of magnitude, and segmentation must be
    bit-identical. Segmentation keys off timing, direction and acceleration
    only -- never off how far the face turned."""
    segmentations = {}
    for arc in ArcType:
        traj = generate(StrokeParams(arc_type=arc, face_angle_at_impact_deg=2.0))
        out = simulate(traj, SensorParams(), seed=1)
        pipe = Pipeline(Thresholds(), out.full_scale)
        for i in range(len(traj.time)):
            pipe.step(out.gyro_counts[i], out.accel_counts[i])
        segmentations[arc.name] = (pipe.i_backswing_start, pipe.i_transition, pipe.i_impact)

    distinct = set(segmentations.values())
    assert len(distinct) == 1, (
        f"segmentation changed with face rotation amplitude: {segmentations}"
    )
