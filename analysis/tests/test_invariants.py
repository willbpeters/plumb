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
    # 5.0 rad/s per unit of error is a typical complementary-filter Kp -- the
    # sort of value a stock Madgwick or Mahony implementation ships with. That
    # is the point: the invariant is about what happens when you use a normal
    # filter normally, not an absurd setting.
    #
    # Calibrated after the gain was corrected to be a rate rather than a
    # per-sample step. Measured, gain left on through the stroke:
    #   0.5 -> 0.0005 deg    1.0 -> 0.0025    2.0 -> 0.0043
    #   5.0 -> 2.70 deg      10.0 -> 349 deg
    # 5.0 is the first value that clearly exceeds the +/-1.0 deg target in
    # parent spec section 3.
    stroke = StrokeParams(face_angle_at_impact_deg=2.0)
    correct = recover(stroke)
    naive = recover(stroke, Thresholds(accel_gain_stroke=5.0))

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


def test_invariant_1_face_rotation_cannot_steer_the_transition():
    """CLAUDE.md invariant 1, for a face that turns faster than the swing.

    The test above scales face rotation only as far as the generator's arc
    types go, and at backswing confirmation none of them turns the face faster
    than the shaft swings: the arced stroke reads (0.05, -12.55, -4.39) deg/s.
    Backswing is confirmed when the swing has barely started, so the swing
    component is small there and a face that opens early can outrun it.

    Face rotation is rotation about the shaft, body Z. Here a face-rotation rate
    is added to that channel only, larger than the swing at confirmation and
    reversing 100 ms AFTER the swing does. Real faces do not have to reverse
    with the swing: releasing the face early or late is a stroke
    characteristic, which is exactly why the transition must not be timed from
    it. Nothing else changes, so the swing reverses at the same instant and the
    segmentation must be identical. Timing the transition off body Z moves it
    by the full 100 ms and puts the error straight into the tempo ratio.
    """
    traj = generate(StrokeParams(arc_type=ArcType.STRAIGHT))
    out = simulate(traj, SensorParams(), seed=1)
    dt = traj.time[1] - traj.time[0]

    start = traj.time[traj.address_end_index]
    reversal = traj.time[traj.transition_index] + 0.100
    end = traj.time[traj.impact_index]
    peak = np.radians(40.0)
    t = traj.time
    face_rate = np.zeros_like(t)
    opening = (t >= start) & (t < reversal)
    face_rate[opening] = peak * np.sin(0.5 * np.pi * np.clip((t[opening] - start) / 0.03, 0.0, 1.0))
    closing = (t >= reversal) & (t < end)
    face_rate[closing] = -peak * np.sin(np.pi * (t[closing] - reversal) / (end - reversal))

    turned = out.gyro_counts.copy()
    turned[:, 2] = np.clip(
        turned[:, 2] + np.round(face_rate / out.full_scale.gyro_rad_per_count),
        -32768, 32767).astype(np.int16)

    segmentations = {}
    for label, gyro in (("swing only", out.gyro_counts), ("fast face", turned)):
        pipe = Pipeline(Thresholds(), out.full_scale)
        for i in range(len(traj.time)):
            pipe.step(gyro[i], out.accel_counts[i])
        segmentations[label] = (pipe.i_backswing_start, pipe.i_transition,
                                pipe.i_impact)

    shift_ms = (segmentations["fast face"][1] - segmentations["swing only"][1]) * dt * 1e3
    print(f"\n  transition moved {shift_ms:+.1f} ms under face rotation: {segmentations}")
    assert segmentations["fast face"] == segmentations["swing only"], (
        f"face rotation moved the segmentation: {segmentations}"
    )


def _radio_check():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[2] / "firmware" / "tools" / "check_no_radio.py"
    spec = importlib.util.spec_from_file_location("check_no_radio", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Excerpts in GNU ld's map format: the archive-member section, then a placed
# symbol. The radio one is what linking esp_wifi_init() produces. The clean one
# carries two lines from the first real build's map that name Wi-Fi without
# being radio code: a ROM function's address from the linker script, and the
# peripheral-clock helper every build links. A check that flags them fails
# every build and gets switched off.
_CLEAN_MAP = """\
Archive member included to satisfy reference by file (symbol)

esp-idf/esp_timer/libesp_timer.a(esp_timer.c.obj)
                              esp-idf/main/libmain.a(app_main.c.obj) (esp_timer_get_time)
 .text.esp_timer_get_time
                0x42001234       0x10 esp-idf/esp_timer/libesp_timer.a(esp_timer.c.obj)
                0x42001234                esp_timer_get_time
                0x40005ab4                        wifi_get_macaddr = 0x40005ab4
  wifi_bt_common_module_enable                      esp-idf/esp_hw_support/libesp_hw_support.a(periph_ctrl.c.obj)
"""

_RADIO_MAP = _CLEAN_MAP + """\
esp-idf/esp_wifi/libesp_wifi.a(wifi_init.c.obj)
                              esp-idf/main/libmain.a(app_main.c.obj) (esp_wifi_init)
 .text.esp_wifi_init
                0x42005678       0x40 esp-idf/esp_wifi/libesp_wifi.a(wifi_init.c.obj)
                0x42005678                esp_wifi_init
"""


def test_invariant_6_map_check_passes_a_clean_map():
    """CLAUDE.md invariant 6. The check has to pass a real build's map, or it
    is noise that gets switched off."""
    assert _radio_check().offending(_CLEAN_MAP) == []


def test_invariant_6_map_check_rejects_linked_wifi():
    """CLAUDE.md invariant 6: Wi-Fi stays compiled out in every build. The
    check is what makes that a property of the binary rather than a hope."""
    found = _radio_check().offending(_RADIO_MAP)
    assert found
    assert any("esp_wifi" in line for line in found)
