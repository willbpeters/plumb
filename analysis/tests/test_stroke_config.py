"""The device's configuration is the configuration the harness verified.

firmware/main/stroke_config.c carries the pipeline's thresholds on the
device. They are placeholders until Phase 2 (invariant 5), and the one thing
worse than a placeholder is a placeholder that quietly differs from the one
every test in this suite ran with. So the C file is parsed and held to the
Python's Thresholds(), field for field, with nothing missing and nothing extra.
"""

import inspect
import re
from dataclasses import fields
from pathlib import Path

import numpy as np
import pytest

from plumb import pipeline as pipeline_module
from plumb.pipeline import Thresholds
from plumb.sensor import FULL_SCALE_COUNTS, GRAVITY, FullScale

CONFIG = Path(__file__).resolve().parents[2] / "firmware" / "main" / "stroke_config.c"
ASSIGNMENT = re.compile(
    r"th->(\w+)\s*=\s*\(pl_real\)(radians\()?([-\d.]+)\)?;")
DEFINE = re.compile(r"#define\s+(\w+)\s+([-\d.]+)")


@pytest.fixture(scope="module")
def source():
    return CONFIG.read_text()


def test_every_threshold_is_the_harness_value(source):
    found = {}
    for name, in_degrees, value in ASSIGNMENT.findall(source):
        assert name not in found, f"{name} assigned twice"
        v = float(value)
        found[name] = np.radians(v) if in_degrees else v
    expected = {f.name: getattr(Thresholds(), f.name) for f in fields(Thresholds)}
    assert set(found) == set(expected), (
        f"missing {set(expected) - set(found)}, extra {set(found) - set(expected)}")
    for name, value in expected.items():
        assert found[name] == pytest.approx(value, rel=1e-12), name


def test_the_conversions_are_the_references(source):
    defines = {k: float(v) for k, v in DEFINE.findall(source)}
    assert defines["FULL_SCALE_COUNTS"] == FULL_SCALE_COUNTS
    assert defines["GRAVITY"] == GRAVITY
    assert defines["GYRO_DPS"] == FullScale().gyro_dps
    assert defines["ACCEL_G"] == FullScale().accel_g


def test_the_lever_arm_is_the_harness_value(source):
    (lever,) = re.findall(r"cfg->lever_arm_m\s*=\s*\(pl_real\)([\d.]+);", source)
    default = inspect.signature(pipeline_module.Pipeline).parameters["lever_arm_m"].default
    assert float(lever) == default
