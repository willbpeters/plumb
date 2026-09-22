"""Differential test: the ported C against the Python it came from.

The conventions require algorithms to be proven in Python and then "re-verified
against the same corpus" after the port to C. This is that re-verification for
the quaternion layer.

It matters more on this project than on most. There is no JTAG on this board
(parent spec 14.3), so a translation error that survives to the device is
debugged with printf, on hardware, against no reference at all. Caught here it
is a diff with line numbers.

The test does not check the C against a hand-written expectation. It checks it
against the exact function the reference implementation runs, on the same
inputs, which is the only comparison that can catch a faithful-looking
translation of the wrong formula.
"""

import subprocess
from pathlib import Path

import numpy as np
import pytest

from plumb import quat
from tools import cbuild

CASES = 400
SEED = 20260922


def _build(name: str, *defines: str) -> Path:
    """Build the harness, skipping loudly if this machine cannot.

    The first version of this shelled out to a build script, which needed `sh`
    on PATH. Run from PowerShell rather than Git Bash it reported ten silent
    skips and verified nothing, while reading as a pass. Building through
    tools/cbuild.py needs no shell, and the skip now says what it means.
    """
    try:
        result = cbuild.build(name, *defines)
    except cbuild.CompilerNotFound as missing:
        message = (f"THE C PORT IS NOT VERIFIED HERE: {missing}. "
                   f"Install a C compiler to run the differential test.")
        print(f"\n*** {message}")
        pytest.skip(message)
    return result.executable


@pytest.fixture(scope="session")
def portcheck():
    return _build("portcheck.exe")


@pytest.fixture(scope="session")
def portcheck_single():
    return _build("portcheck-single.exe", "PLUMB_SINGLE_PRECISION")


def run_c(exe: Path, lines: list[str]) -> list[list[float]]:
    result = subprocess.run([str(exe)], input="\n".join(lines) + "\n",
                            capture_output=True, text=True)
    if result.returncode != 0:
        pytest.fail(f"harness exited {result.returncode}: {result.stderr}")
    return [[float(v) for v in line.split()]
            for line in result.stdout.strip().splitlines()]


def random_cases(rng):
    """Inputs for every op the Pipeline actually calls.

    Quaternions are random unit quaternions rather than small perturbations of
    the identity: a translation error in one component of a multiply is
    invisible near the identity, where three of the four terms are zero.
    """
    quats = rng.normal(size=(CASES, 4))
    quats /= np.linalg.norm(quats, axis=1, keepdims=True)
    others = rng.normal(size=(CASES, 4))
    others /= np.linalg.norm(others, axis=1, keepdims=True)
    vectors = rng.normal(size=(CASES, 3)) * 10.0
    # Rates spanning stillness to well past a putting stroke's peak, and a dt
    # at the real sample interval.
    rates = rng.normal(size=(CASES, 3)) * rng.choice([1e-4, 1e-2, 1.0, 10.0],
                                                     size=(CASES, 1))
    angles = rng.uniform(-np.pi, np.pi, CASES)
    return quats, others, vectors, rates, angles


def fmt(*arrays) -> str:
    return " ".join(f"{v:.17g}" for a in arrays for v in np.atleast_1d(a))


def worst_relative_error(c_values, py_values) -> float:
    """Largest difference on any row, relative to that row's magnitude.

    Not per-component. A quaternion multiply can produce a component that is
    nearly zero through cancellation, and dividing by that component measures
    the cancellation rather than the translation -- it reports 2.4e-5 for a
    single-precision build whose answers are in fact correct to a few ulps of
    the quantity they belong to. The vector's own magnitude is the scale that
    means something.
    """
    c = np.asarray(c_values, dtype=float)
    py = np.asarray(py_values, dtype=float)
    magnitude = np.maximum(np.linalg.norm(py, axis=1, keepdims=True), 1e-12)
    return float(np.max(np.abs(c - py) / magnitude))


def compare(exe, op, inputs, python_fn, label, tolerance):
    lines = [f"{op} {fmt(*case)}" for case in inputs]
    produced = run_c(exe, lines)
    expected = [np.atleast_1d(python_fn(*case)) for case in inputs]
    worst = worst_relative_error(produced, expected)
    # Printed, not just asserted: a tolerance that passes tells you nothing
    # about how much headroom it had.
    print(f"  {label:<24} worst relative difference {worst:.3e}")
    assert worst < tolerance, f"{label}: {worst:.3e}"
    return worst


@pytest.fixture(scope="session")
def cases():
    return random_cases(np.random.default_rng(SEED))


def test_the_harness_reports_the_precision_it_was_built_with(portcheck):
    """The harness names the type it was built with, so a run cannot silently
    be measuring the wrong build -- which is the failure mode of having two
    binaries that differ only in a compile-time define."""
    out = subprocess.run([str(portcheck)], input="precision\n",
                         capture_output=True, text=True).stdout.strip()
    assert out == "double"


def test_multiply_matches(portcheck, cases):
    quats, others, _, _, _ = cases
    compare(portcheck, "multiply", list(zip(quats, others)),
            quat.multiply, "multiply", 1e-12)


def test_conjugate_matches(portcheck, cases):
    quats, _, _, _, _ = cases
    compare(portcheck, "conjugate", [(q,) for q in quats],
            quat.conjugate, "conjugate", 1e-15)


def test_normalize_matches(portcheck, cases):
    _, _, vectors, rates, _ = cases
    # Deliberately un-normalised, and spanning a wide range of magnitudes.
    raw = np.hstack([vectors, rates[:, :1]])
    compare(portcheck, "normalize", [(q,) for q in raw],
            quat.normalize, "normalize", 1e-12)


def test_rotate_matches(portcheck, cases):
    quats, _, vectors, _, _ = cases
    compare(portcheck, "rotate", list(zip(quats, vectors)),
            quat.rotate, "rotate", 1e-12)


def test_from_axis_angle_matches(portcheck, cases):
    _, _, vectors, _, angles = cases
    compare(portcheck, "axisangle", list(zip(vectors, angles)),
            quat.from_axis_angle, "from_axis_angle", 1e-12)


def test_integrate_matches(portcheck, cases):
    """The one that runs 896.8 times a second and accumulates. A difference
    here does not stay small."""
    quats, _, _, rates, _ = cases
    dt = 1.0 / 906.86
    compare(portcheck, "integrate", [(q, w, dt) for q, w in zip(quats, rates)],
            quat.integrate, "integrate", 1e-12)


def test_twist_angle_matches(portcheck, cases):
    """How face angle is extracted, so this is the arithmetic the headline
    number rests on."""
    quats, _, vectors, _, _ = cases
    compare(portcheck, "twist", list(zip(quats, vectors)),
            quat.twist_angle, "twist_angle", 1e-12)


def test_to_matrix_matches(portcheck, cases):
    quats, _, _, _, _ = cases
    compare(portcheck, "tomatrix", [(q,) for q in quats],
            lambda q: quat.to_matrix(q).ravel(), "to_matrix", 1e-15)


def test_single_precision_divergence_is_measured_not_assumed(portcheck_single,
                                                             cases):
    """What the FPU-friendly build costs, as a number.

    The ESP32-S3's FPU is single-precision; doubles are emulated in software,
    which at 896.8 Hz is a cost worth knowing. The reference implementation is
    float64, so a single-precision device build diverges from the thing that
    was proven correct, and this measures by how much rather than assuming it
    is fine.

    The bound is loose on purpose. It is here to catch a build that is broken
    rather than merely less precise; the number printed is the useful output.
    """
    quats, others, vectors, rates, angles = cases
    dt = 1.0 / 906.86
    worst = {
        "multiply": compare(portcheck_single, "multiply",
                            list(zip(quats, others)), quat.multiply,
                            "multiply (single)", 1e-6),
        "rotate": compare(portcheck_single, "rotate",
                          list(zip(quats, vectors)), quat.rotate,
                          "rotate (single)", 1e-6),
        "integrate": compare(portcheck_single, "integrate",
                             [(q, w, dt) for q, w in zip(quats, rates)],
                             quat.integrate, "integrate (single)", 1e-6),
        "twist_angle": compare(portcheck_single, "twist",
                               list(zip(quats, vectors)), quat.twist_angle,
                               "twist_angle (single)", 1e-4),
    }
    assert max(worst.values()) > 1e-9, (
        "single-precision build agrees with float64 to better than 1e-9, "
        "which means PLUMB_SINGLE_PRECISION did not take effect"
    )
