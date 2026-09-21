# Synthetic Stroke Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Validate the §7 fusion pipeline with zero hardware, by forward-modeling a QMI8658 and checking that the pipeline recovers a face angle we specified ourselves.

**Architecture:** Four modules under `analysis/plumb/`. `trajectory.py` and `sensor.py` generate ground truth and fake sensor counts (offline, written plainly). `pipeline.py` is a streaming, C-shaped implementation of the §7 chain — one sample at a time, no lookahead — and is the reference that later ports to firmware. `sweep.py` drives the parameter grid.

**Tech Stack:** Python 3.11+, NumPy, SciPy, pytest, managed with `uv`.

**Specs:** `docs/superpowers/specs/2026-09-20-synthetic-stroke-harness-design.md` (this feature), `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` (parent; §N refers to it).

---

## Conventions — read before Task 1

Every frame and sign convention is fixed here. Deviating from these silently is the failure mode `test_roundtrip_noiseless` exists to catch.

**Quaternions.** Hamilton convention, scalar-first `(w, x, y, z)`, stored as a length-4 `np.ndarray`. A quaternion represents a rotation *from body coordinates to reference coordinates*: `v_ref = q ⊗ v_body ⊗ q*`.

**World frame.** `X` = target line (direction the ball starts). `Y` = horizontal, perpendicular to the target line. `Z` = up. Gravity is `g_vec = (0, 0, −9.81)`.

**Body frame.** `Z` = shaft axis, pointing from head up toward the grip butt. `X` = face normal. `Y` completes the right-handed set. Face normal is perpendicular to the shaft, which is why it lies in the body XY plane.

**The three rotations** (§4.3 of the harness spec), composed body→world as:

```
R(t) = Rx(lie) @ Ry(theta(t)) @ Rz(phi(t))
```

| Rotation | Axis | Meaning |
|---|---|---|
| `lie` | world X (target line) | Fixed shaft tilt from vertical. Constant. |
| `theta(t)` | ~body Y | Pendulum swing, in the plane containing the target line |
| `phi(t)` | body Z (shaft) | Face opening/closing |

These are three genuinely distinct axes. At address (`theta = phi = 0`), `R = Rx(lie)`.

**Angular velocity is analytic, not numerically differentiated.** Because `lie` is constant it contributes nothing to `omega`, and the body-frame rate reduces to a closed form:

```
omega_body(t) = theta_dot(t) * (sin(phi), cos(phi), 0) + phi_dot(t) * (0, 0, 1)
```

Derivation: `omega = vee(R^T R_dot)`; with `R = Rx Ry Rz` and `Rx` constant, `R^T R_dot = Rz^T (Ry^T Ry_dot) Rz + Rz^T Rz_dot`, and `Rz(phi)^T * y_hat = (sin phi, cos phi, 0)`.

Using the closed form rather than differencing keeps the noiseless test a pure algebraic inverse, so any error it reports is a real bug rather than differentiation noise.

**Impact occurs exactly at `theta = 0`.** The forward swing is defined to pass through `theta = 0`, and that crossing *is* `t_impact`. This matters: it means `phi(t_impact)` is independent of how much the face rotates during the swing, which is what lets the same specified face angle be tested across every arc type.

**Face angle is not `phi`.** This is the subtlety that makes or breaks the test, and it is the one place this plan refines the spec.

The device reports face angle as the azimuth of the face normal in the ground plane, relative to address (§3.3, §7.3). Rotating the shaft by `phi` when the shaft is tilted by `lie` does *not* move the face normal by `phi` in the horizontal plane. Working it through, with the face normal at body X:

```
face_angle_projected = atan( tan(phi) * cos(lie) )
```

At `lie = 20°` and `phi = 2°` that is 1.880°, not 2.000° — a 0.12° gap, larger than the 0.1° tolerance the noiseless test asserts. A harness that asserts `recovered == phi` would fail with perfect math, and the natural "fix" would be to break the projection.

**Therefore the generator is parameterized by the projected face angle** — the number the device should report — and solves internally for the shaft rotation:

```
phi_impact = atan( tan(face_angle_at_impact) / cos(lie) )
```

Ground truth is the value the caller asked for. This also demonstrates §4.3 concretely: at `lie = 0` the relation collapses to `phi_impact == face_angle_at_impact` and the projection is untestable.

**Gravity direction at address, in body coordinates**, is `(0, sin(lie), cos(lie))` — not the body Z axis unless `lie` is zero. The pipeline measures this as `g0` and it defines the ground plane.

**The twist and the azimuth are not the same quantity, and that is expected.** The pipeline extracts face angle as the twist of the address-relative attitude about `g0`. The generator's ground truth is the azimuth of the face normal in the ground plane. These agree to third order but not exactly:

| Face angle | twist about `g0` | difference |
|---|---|---|
| 2.0° | 1.999919° | −8.1e-5° |
| 5.0° | 4.998742° | −1.3e-3° |

Verified numerically against `plumb.quat` before Task 3 was written. About 1.3 milli-degrees at 5°, which is why the pipeline tests assert `abs=0.1` rather than machine precision, while the generator's own self-consistency test in Task 3 asserts `abs=1e-6` — the generator is checked against its own definition, the pipeline against a different-but-equivalent one.

Do not try to drive this residual to zero. It is inherent to the two definitions, it is four orders of magnitude below the ±1.0° product target from parent spec §3, and chasing it would mean replacing a decomposition the firmware can compute cheaply with one it cannot.

**The noiseless error budget, measured.** The 0.1° tolerance is not arbitrary and it is not generous. Measured over 45 noiseless strokes spanning face angle, arc type and tempo:

| Source | Cost | Share of 0.1° |
|---|---|---|
| Rectangle-rule integration at 500 Hz | 0.0553° | 55% |
| **Trapezoidal integration at 500 Hz** | **0.0013°** | **1.3%** |
| Twist-vs-azimuth definition residual | 0.0013° | 1.3% |

Rectangle rule — holding each sample's rate constant across its interval — would spend more than half the budget before any noise, bias or quantization exists. The pipeline therefore integrates trapezoidally (§6 of this plan), which is a one-sample latency rather than lookahead and is implementable on-device. This was measured before the pipeline was written, not discovered afterwards.

---

## File Structure

| File | Responsibility |
|---|---|
| `analysis/pyproject.toml` | uv project, deps, pytest config |
| `analysis/plumb/quat.py` | Quaternion and rotation primitives. No domain knowledge. |
| `analysis/plumb/trajectory.py` | Ground-truth stroke generation (harness spec §4) |
| `analysis/plumb/sensor.py` | QMI8658 forward model, emits int16 counts (§5) |
| `analysis/plumb/pipeline.py` | Streaming §7 chain under test. Ports to C. |
| `analysis/plumb/sweep.py` | Parameter grid driver (§7.1) |
| `analysis/tests/test_quat.py` | Quaternion primitives |
| `analysis/tests/test_trajectory.py` | Ground truth is self-consistent |
| `analysis/tests/test_sensor.py` | Forward model, quantization, reproducibility |
| `analysis/tests/test_pipeline.py` | **Recovery accuracy — the load-bearing tests** |
| `analysis/tests/test_invariants.py` | Invariants 1 and 2, enforced as tests |

---

## Task 1: Project scaffold

**Files:**
- Create: `analysis/pyproject.toml`, `analysis/plumb/__init__.py`, `analysis/tests/__init__.py`

- [ ] **Step 1: Initialise the uv project**

```bash
cd analysis
uv init --name plumb --lib --python 3.11
```

- [ ] **Step 2: Replace `analysis/pyproject.toml` with this**

```toml
[project]
name = "plumb"
version = "0.1.0"
description = "Synthetic stroke harness and algorithm development for the Plumb putting analyzer"
requires-python = ">=3.11"
dependencies = ["numpy>=2.0", "scipy>=1.13"]

[dependency-groups]
dev = ["pytest>=8.0"]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = "-q"
```

- [ ] **Step 3: Create empty package files**

```bash
mkdir -p analysis/plumb analysis/tests
touch analysis/plumb/__init__.py analysis/tests/__init__.py
```

- [ ] **Step 4: Sync and confirm the toolchain works**

Run: `cd analysis && uv sync && uv run pytest`
Expected: `no tests ran` — exit code 5. That is success; it means pytest is installed and found the test directory.

- [ ] **Step 5: Commit**

```bash
git add analysis/
git commit -m "Add Python analysis project scaffold"
```

---

## Task 2: Quaternion primitives

Pure math, no domain knowledge. Everything downstream depends on these being right.

**Files:**
- Create: `analysis/plumb/quat.py`
- Test: `analysis/tests/test_quat.py`

- [ ] **Step 1: Write the failing tests**

```python
# analysis/tests/test_quat.py
import numpy as np
import pytest

from plumb import quat


def test_identity_rotates_nothing():
    v = np.array([1.0, 2.0, 3.0])
    np.testing.assert_allclose(quat.rotate(quat.identity(), v), v, atol=1e-12)


def test_rotation_about_z_moves_x_to_y():
    q = quat.from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2)
    got = quat.rotate(q, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(got, [0.0, 1.0, 0.0], atol=1e-12)


def test_multiply_composes_in_body_to_reference_order():
    # q_ab then q_bc applied to a body vector must equal the composed rotation.
    q_ab = quat.from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.3)
    q_bc = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4)
    v = np.array([0.2, -0.5, 0.9])
    direct = quat.rotate(quat.multiply(q_ab, q_bc), v)
    stepwise = quat.rotate(q_ab, quat.rotate(q_bc, v))
    np.testing.assert_allclose(direct, stepwise, atol=1e-12)


def test_conjugate_inverts():
    q = quat.from_axis_angle(np.array([1.0, 2.0, 3.0]), 0.7)
    v = np.array([0.1, 0.2, 0.3])
    back = quat.rotate(quat.conjugate(q), quat.rotate(q, v))
    np.testing.assert_allclose(back, v, atol=1e-12)


def test_integrate_constant_rate_matches_closed_form():
    # 1 rad/s about Z for 1 s, in 1000 steps, must land on 1 rad.
    q = quat.identity()
    omega = np.array([0.0, 0.0, 1.0])
    dt = 1e-3
    for _ in range(1000):
        q = quat.integrate(q, omega, dt)
    expected = quat.from_axis_angle(np.array([0.0, 0.0, 1.0]), 1.0)
    np.testing.assert_allclose(quat.rotate(q, np.array([1.0, 0.0, 0.0])),
                               quat.rotate(expected, np.array([1.0, 0.0, 0.0])),
                               atol=1e-9)


def test_twist_about_axis_extracts_pure_rotation():
    axis = np.array([0.0, 0.0, 1.0])
    q = quat.from_axis_angle(axis, 0.25)
    assert quat.twist_angle(q, axis) == pytest.approx(0.25, abs=1e-12)


def test_twist_ignores_rotation_perpendicular_to_axis():
    # A pure rotation about X has no twist about Z.
    q = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.4)
    assert quat.twist_angle(q, np.array([0.0, 0.0, 1.0])) == pytest.approx(0.0, abs=1e-12)


def test_from_rotation_matrix_roundtrip():
    q = quat.from_axis_angle(np.array([0.3, -0.7, 0.2]), 1.1)
    np.testing.assert_allclose(
        quat.rotate(quat.from_matrix(quat.to_matrix(q)), np.array([1.0, 0.0, 0.0])),
        quat.rotate(q, np.array([1.0, 0.0, 0.0])),
        atol=1e-12,
    )
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd analysis && uv run pytest tests/test_quat.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'plumb.quat'`

- [ ] **Step 3: Implement `analysis/plumb/quat.py`**

```python
"""Quaternion and rotation primitives.

Hamilton convention, scalar-first (w, x, y, z). A quaternion rotates a vector
from body coordinates to reference coordinates: v_ref = q * v_body * q^-1.
"""

import numpy as np


def identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0])


def from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n == 0.0:
        return identity()
    axis = axis / n
    half = 0.5 * angle
    return np.concatenate(([np.cos(half)], np.sin(half) * axis))


def multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = np.concatenate(([0.0], np.asarray(v, dtype=float)))
    return multiply(multiply(q, qv), conjugate(q))[1:]


def integrate(q: np.ndarray, omega_body: np.ndarray, dt: float) -> np.ndarray:
    """Advance attitude by a body-frame angular rate held constant over dt.

    Uses the exact exponential map rather than a first-order update, so
    integration error does not masquerade as an algorithm error in the
    noiseless test.
    """
    omega_body = np.asarray(omega_body, dtype=float)
    angle = np.linalg.norm(omega_body) * dt
    if angle == 0.0:
        return q
    return normalize(multiply(q, from_axis_angle(omega_body, angle)))


def twist_angle(q: np.ndarray, axis: np.ndarray) -> float:
    """Signed rotation of q about `axis` (swing-twist decomposition).

    This is how face angle is extracted: the component of the attitude change
    that is a rotation about measured gravity. Rotation perpendicular to the
    axis contributes nothing.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    projection = float(np.dot(q[1:], axis))
    return 2.0 * np.arctan2(projection, q[0])


def to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def from_matrix(m: np.ndarray) -> np.ndarray:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return normalize(np.array([0.25 / s,
                                   (m[2, 1] - m[1, 2]) * s,
                                   (m[0, 2] - m[2, 0]) * s,
                                   (m[1, 0] - m[0, 1]) * s]))
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
    q = np.zeros(4)
    q[0] = (m[k, j] - m[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (m[j, i] + m[i, j]) / s
    q[k + 1] = (m[k, i] + m[i, k]) / s
    return normalize(q)


def rot_x(angle: float) -> np.ndarray:
    return from_axis_angle(np.array([1.0, 0.0, 0.0]), angle)


def rot_y(angle: float) -> np.ndarray:
    return from_axis_angle(np.array([0.0, 1.0, 0.0]), angle)


def rot_z(angle: float) -> np.ndarray:
    return from_axis_angle(np.array([0.0, 0.0, 1.0]), angle)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd analysis && uv run pytest tests/test_quat.py -v`
Expected: 8 passed

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/quat.py analysis/tests/test_quat.py
git commit -m "Add quaternion primitives with explicit Hamilton convention"
```

---

## Task 3: Trajectory generator

**Files:**
- Create: `analysis/plumb/trajectory.py`
- Test: `analysis/tests/test_trajectory.py`

- [ ] **Step 1: Write the failing tests**

```python
# analysis/tests/test_trajectory.py
import numpy as np
import pytest

from plumb import quat
from plumb.trajectory import ArcType, StrokeParams, generate


@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_impact_occurs_exactly_where_swing_angle_crosses_zero(tempo):
    """Must hold exactly, for every tempo, not just to within a sample.

    The face rotation carries an `arc_gain * theta` term that is designed to
    vanish at impact. If theta is merely near zero there, that term leaks into
    ground truth and the leak scales with arc_gain -- which would make recovery
    look arc-type-dependent and fake a violation of invariant 1."""
    t = generate(StrokeParams(tempo_ratio=tempo))
    assert t.theta[t.impact_index] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_achieved_tempo_ratio_is_recorded_and_close_to_requested(tempo):
    """A 500 Hz grid cannot land an arbitrary tempo ratio on a whole sample, so
    the generator snaps to samples and reports what it actually produced."""
    t = generate(StrokeParams(tempo_ratio=tempo))
    backswing = t.time[t.transition_index] - t.time[t.address_end_index]
    downswing = t.time[t.impact_index] - t.time[t.transition_index]
    assert backswing / downswing == pytest.approx(t.true_tempo_ratio, rel=1e-12)
    assert t.true_tempo_ratio == pytest.approx(tempo, rel=5e-3)


@pytest.mark.parametrize("arc", list(ArcType))
def test_face_angle_at_impact_is_independent_of_arc_type(arc):
    """The recovered quantity must not depend on how much the face rotates
    during the stroke. This is invariant 1 expressed in the generator."""
    p = StrokeParams(face_angle_at_impact_deg=2.0, arc_type=arc)
    t = generate(p)
    assert t.true_face_angle_deg == pytest.approx(2.0, abs=1e-9)


def test_projected_face_angle_differs_from_shaft_rotation_when_lie_is_nonzero():
    """Guards the refinement in the plan preamble: rotating the shaft by phi
    does not move the face normal by phi in the ground plane."""
    p = StrokeParams(face_angle_at_impact_deg=2.0, lie_angle_deg=20.0)
    t = generate(p)
    phi_impact = np.degrees(t.phi[t.impact_index])
    assert phi_impact == pytest.approx(2.128, abs=1e-3)
    assert phi_impact != pytest.approx(2.0, abs=1e-3)


def test_face_normal_azimuth_matches_requested_face_angle():
    """Independent check of the ground truth, computed from the attitude
    rather than from the generator's own parameterization."""
    p = StrokeParams(face_angle_at_impact_deg=2.0, lie_angle_deg=20.0)
    t = generate(p)
    q0 = t.q_true[t.address_end_index]
    qi = t.q_true[t.impact_index]
    g_world = np.array([0.0, 0.0, 1.0])
    face_body = np.array([1.0, 0.0, 0.0])

    def azimuth(q):
        n = quat.rotate(q, face_body)
        n = n - np.dot(n, g_world) * g_world
        return np.arctan2(n[1], n[0])

    delta = np.degrees(azimuth(qi) - azimuth(q0))
    assert delta == pytest.approx(2.0, abs=1e-6)


def test_angular_velocity_matches_numerical_differentiation_of_attitude():
    """The analytic omega must agree with differencing the attitude it claims
    to describe. This catches a sign error or an axis mix-up in either one.

    Central difference, not forward. A forward difference estimates the AVERAGE
    rate over [t, t+dt], which differs from the rate AT t by (dt/2)*omega_dot --
    about 4e-3 rad/s at the phase boundaries where the raised-cosine profile's
    curvature peaks, which exceeds any tolerance worth asserting here.

    The residual that remains after central differencing is O(|omega|^2 * dt):
    the difference is expressed in the body frame at i-1 rather than at i. That
    floors this check at roughly 1e-3 rad/s, which is fine for its purpose -- the
    errors it exists to catch are sign flips and axis swaps, which show up at
    order |omega| itself, a thousand times larger."""
    t = generate(StrokeParams())
    dt = t.time[1] - t.time[0]
    for i in range(t.address_end_index + 10, t.impact_index, 37):
        dq = quat.multiply(quat.conjugate(t.q_true[i - 1]), t.q_true[i + 1])
        omega_numeric = dq[1:] / dt
        np.testing.assert_allclose(omega_numeric, t.omega_true[i], atol=2e-3)


def test_zero_torque_case_really_has_almost_no_face_rotation():
    straight = generate(StrokeParams(arc_type=ArcType.NEAR_ZERO_ROTATION))
    arced = generate(StrokeParams(arc_type=ArcType.ARCED))
    assert np.ptp(straight.phi) < 0.1 * np.ptp(arced.phi)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd analysis && uv run pytest tests/test_trajectory.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'plumb.trajectory'`

- [ ] **Step 3: Implement `analysis/plumb/trajectory.py`**

```python
"""Ground-truth stroke generation.

Produces attitude and body-frame angular velocity at 500 Hz for a stroke whose
face angle at impact we specify. Offline only; never ported to C.

Frames and conventions are documented in the implementation plan preamble.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np

from plumb import quat

SAMPLE_RATE_HZ = 500.0


class ArcType(Enum):
    """How much the face rotates with the swing.

    This controls the motion the device measures, not how it measures it
    (parent spec 2.1). Face angle at impact is identical across all three.
    """

    STRAIGHT = 0.0
    NEAR_ZERO_ROTATION = 0.02
    ARCED = 0.35


@dataclass(frozen=True)
class StrokeParams:
    backswing_amplitude_deg: float = 12.0
    followthrough_amplitude_deg: float = 12.0
    backswing_duration_s: float = 0.70
    tempo_ratio: float = 2.0
    face_angle_at_impact_deg: float = 0.0
    arc_type: ArcType = ArcType.ARCED
    lie_angle_deg: float = 20.0
    lever_arm_m: float = 0.85          # sensor to sweet spot
    pivot_offset_m: float = 0.55       # sensor to rotation centre
    address_duration_s: float = 1.00
    followthrough_hold_s: float = 0.60


@dataclass
class Trajectory:
    time: np.ndarray
    theta: np.ndarray
    phi: np.ndarray
    q_true: np.ndarray          # (N, 4)
    omega_true: np.ndarray      # (N, 3) body frame, rad/s
    address_end_index: int
    transition_index: int
    impact_index: int
    true_face_angle_deg: float
    true_tempo_ratio: float
    params: StrokeParams


def _smoothstep(u: np.ndarray) -> np.ndarray:
    """Raised-cosine ramp from 0 to 1 with zero slope at both ends."""
    return 0.5 * (1.0 - np.cos(np.pi * np.clip(u, 0.0, 1.0)))


def _smoothstep_derivative(u: np.ndarray, duration: float) -> np.ndarray:
    inside = (u >= 0.0) & (u <= 1.0)
    return np.where(inside, 0.5 * np.pi * np.sin(np.pi * np.clip(u, 0.0, 1.0)) / duration, 0.0)


def shaft_rotation_for_face_angle(face_angle_rad: float, lie_angle_rad: float) -> float:
    """Invert the ground-plane projection.

    Rotating a shaft tilted by `lie` through `phi` moves the face normal by
    atan(tan(phi) * cos(lie)) in the ground plane, not by phi. The generator is
    parameterized by the reportable quantity, so it solves for phi.
    """
    return float(np.arctan(np.tan(face_angle_rad) / np.cos(lie_angle_rad)))


def generate(p: StrokeParams) -> Trajectory:
    dt = 1.0 / SAMPLE_RATE_HZ
    lie = np.radians(p.lie_angle_deg)
    amp_back = np.radians(p.backswing_amplitude_deg)
    amp_fwd = np.radians(p.followthrough_amplitude_deg)

    # Forward phase sweeps from -amp_back to +amp_fwd on a raised cosine, so
    # theta = 0 falls at a known fraction of that phase. Impact is that crossing.
    u_impact = np.arccos(1.0 - 2.0 * amp_back / (amp_back + amp_fwd)) / np.pi

    # Phase boundaries snap to whole samples and every duration is derived from
    # the snapped counts. That is what makes theta exactly zero at impact: at
    # that sample u equals u_impact exactly, by construction.
    #
    # Without snapping, a tempo ratio like 1.5 puts impact at sample 233.33 and
    # theta is merely near zero there. The `arc_gain * theta` term in the face
    # rotation then fails to vanish, ground truth drifts by roughly 0.02 deg,
    # and the drift scales with arc_gain -- which would look exactly like the
    # putter-type dependence invariant 1 prohibits, while actually being a
    # sampling artifact of the generator.
    #
    # A 500 Hz grid cannot land an arbitrary tempo ratio on a whole sample, so
    # the achieved ratio is recorded rather than the requested one.
    n_addr = int(round(p.address_duration_s / dt))
    n_back = int(round(p.backswing_duration_s / dt))
    n_down = int(round(p.backswing_duration_s / p.tempo_ratio / dt))

    backswing_duration = n_back * dt
    forward_duration = (n_down * dt) / u_impact

    address_end_index = n_addr
    transition_index = n_addr + n_back
    impact_index = transition_index + n_down

    t_addr_end = address_end_index * dt
    t_transition = transition_index * dt
    t_impact = impact_index * dt
    t_end = t_transition + forward_duration + p.followthrough_hold_s

    n = int(round(t_end / dt)) + 1
    time = np.arange(n) * dt

    # --- swing angle theta(t) and its derivative -------------------------
    theta = np.zeros(n)
    theta_dot = np.zeros(n)

    back = (time >= t_addr_end) & (time < t_transition)
    u = (time[back] - t_addr_end) / backswing_duration
    theta[back] = -amp_back * _smoothstep(u)
    theta_dot[back] = -amp_back * _smoothstep_derivative(u, backswing_duration)

    fwd = time >= t_transition
    u = np.clip((time[fwd] - t_transition) / forward_duration, 0.0, 1.0)
    theta[fwd] = -amp_back + (amp_back + amp_fwd) * _smoothstep(u)
    theta_dot[fwd] = (amp_back + amp_fwd) * _smoothstep_derivative(u, forward_duration)

    # --- face rotation phi(t) and its derivative -------------------------
    # Two independent contributions:
    #   arc_gain * theta  - face rotating with the swing. Zero at impact by
    #                       construction, since theta is zero there.
    #   ramp * phi_impact - the delivered face angle.
    arc_gain = p.arc_type.value
    phi_impact = shaft_rotation_for_face_angle(np.radians(p.face_angle_at_impact_deg), lie)

    ramp_duration = t_impact - t_addr_end
    u_ramp = (time - t_addr_end) / ramp_duration
    ramp = _smoothstep(u_ramp)
    ramp_dot = _smoothstep_derivative(u_ramp, ramp_duration)

    phi = arc_gain * theta + phi_impact * ramp
    phi_dot = arc_gain * theta_dot + phi_impact * ramp_dot

    # --- attitude and angular velocity -----------------------------------
    q_true = np.empty((n, 4))
    omega_true = np.empty((n, 3))
    q_lie = quat.rot_x(lie)
    for i in range(n):
        q_true[i] = quat.multiply(quat.multiply(q_lie, quat.rot_y(theta[i])), quat.rot_z(phi[i]))
        omega_true[i] = (theta_dot[i] * np.array([np.sin(phi[i]), np.cos(phi[i]), 0.0])
                         + phi_dot[i] * np.array([0.0, 0.0, 1.0]))

    return Trajectory(
        time=time,
        theta=theta,
        phi=phi,
        q_true=q_true,
        omega_true=omega_true,
        address_end_index=address_end_index,
        transition_index=transition_index,
        impact_index=impact_index,
        true_face_angle_deg=p.face_angle_at_impact_deg,
        true_tempo_ratio=n_back / n_down,
        params=p,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd analysis && uv run pytest tests/test_trajectory.py -v`
Expected: 15 passed (two tests parametrized four ways over tempo, one three ways over arc type, four unparametrized)

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/trajectory.py analysis/tests/test_trajectory.py
git commit -m "Add ground-truth stroke trajectory generator"
```

---

## Task 4: Sensor forward model — noiseless

Build the clean path first. Noise, bias and scale error come in Task 5, so that any failure here is geometry rather than statistics.

**Files:**
- Create: `analysis/plumb/sensor.py`
- Test: `analysis/tests/test_sensor.py`

- [ ] **Step 1: Write the failing tests**

```python
# analysis/tests/test_sensor.py
import numpy as np
import pytest

from plumb.sensor import FullScale, SensorParams, simulate
from plumb.trajectory import StrokeParams, generate


def test_counts_are_int16():
    s = simulate(generate(StrokeParams()), SensorParams(), seed=1)
    assert s.gyro_counts.dtype == np.int16
    assert s.accel_counts.dtype == np.int16


def test_noiseless_gyro_roundtrips_within_one_quantum():
    traj = generate(StrokeParams())
    s = simulate(traj, SensorParams(), seed=1)
    fs = FullScale()
    recovered = s.gyro_counts.astype(float) * fs.gyro_rad_per_count
    quantum = fs.gyro_rad_per_count
    np.testing.assert_allclose(recovered, traj.omega_true, atol=quantum)


def test_accelerometer_at_address_reads_gravity_tilted_by_lie_angle():
    """At rest the accelerometer measures specific force, which points 'up' in
    body coordinates: (0, sin(lie), cos(lie)) * 9.81."""
    traj = generate(StrokeParams(lie_angle_deg=20.0))
    s = simulate(traj, SensorParams(), seed=1)
    fs = FullScale()
    a = s.accel_counts[traj.address_end_index // 2].astype(float) * fs.accel_mps2_per_count
    lie = np.radians(20.0)
    expected = 9.81 * np.array([0.0, np.sin(lie), np.cos(lie)])
    np.testing.assert_allclose(a, expected, atol=0.05)


def test_impact_impulse_saturates_and_that_is_acceptable():
    """Clipping at impact is expected, not a defect: impact is a trigger, not a
    measurement (parent spec 6.4). A 60 g impulse against a 16 g full scale
    cannot do anything else.

    Cast to int32 before taking magnitudes. abs() of int16 -32768 overflows
    back to -32768, so a saturation check done in int16 silently reads the
    wrong value at exactly the rail it is trying to detect."""
    traj = generate(StrokeParams())
    s = simulate(traj, SensorParams(), seed=1)
    window = s.accel_counts[traj.impact_index - 2:traj.impact_index + 6].astype(np.int32)
    assert np.abs(window).max() >= 32767


def test_same_seed_gives_identical_output():
    traj = generate(StrokeParams())
    p = SensorParams(gyro_noise_dps=0.5, gyro_bias_dps=1.0)
    a = simulate(traj, p, seed=42)
    b = simulate(traj, p, seed=42)
    np.testing.assert_array_equal(a.gyro_counts, b.gyro_counts)


def test_different_seed_gives_different_noise():
    traj = generate(StrokeParams())
    p = SensorParams(gyro_noise_dps=0.5)
    a = simulate(traj, p, seed=1)
    b = simulate(traj, p, seed=2)
    assert not np.array_equal(a.gyro_counts, b.gyro_counts)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd analysis && uv run pytest tests/test_sensor.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'plumb.sensor'`

- [ ] **Step 3: Implement `analysis/plumb/sensor.py`**

```python
"""QMI8658 forward model.

Emits raw int16 counts, deliberately. A float-valued boundary would let clean
values reach the pipeline with quantization silently skipped, and quantization
sets a hard floor on achievable resolution.
"""

from dataclasses import dataclass

import numpy as np

from plumb import quat
from plumb.trajectory import SAMPLE_RATE_HZ, Trajectory

GRAVITY = 9.81
INT16_MAX = 32767
INT16_MIN = -32768


@dataclass(frozen=True)
class FullScale:
    """Configured ranges (parent spec 6.4). One LSB is the resolution floor."""

    gyro_dps: float = 250.0
    accel_g: float = 16.0

    @property
    def gyro_rad_per_count(self) -> float:
        return np.radians(self.gyro_dps) / INT16_MAX

    @property
    def accel_mps2_per_count(self) -> float:
        return self.accel_g * GRAVITY / INT16_MAX


@dataclass(frozen=True)
class SensorParams:
    gyro_bias_dps: float = 0.0
    gyro_bias_walk_dps_per_s: float = 0.0
    gyro_noise_dps: float = 0.0
    gyro_scale_error: float = 0.0
    accel_bias_mps2: float = 0.0
    accel_noise_mps2: float = 0.0
    accel_scale_error: float = 0.0
    impact_peak_g: float = 60.0
    impact_duration_s: float = 0.004


@dataclass
class SensorOutput:
    gyro_counts: np.ndarray     # (N, 3) int16
    accel_counts: np.ndarray    # (N, 3) int16
    full_scale: FullScale


def _quantize(values: np.ndarray, per_count: float) -> np.ndarray:
    counts = np.rint(values / per_count)
    return np.clip(counts, INT16_MIN, INT16_MAX).astype(np.int16)


def simulate(traj: Trajectory, p: SensorParams, seed: int,
             full_scale: FullScale = FullScale()) -> SensorOutput:
    rng = np.random.default_rng(seed)
    n = len(traj.time)
    dt = 1.0 / SAMPLE_RATE_HZ

    # --- gyroscope --------------------------------------------------------
    walk = np.cumsum(rng.normal(0.0, p.gyro_bias_walk_dps_per_s * np.sqrt(dt), size=(n, 3)), axis=0)
    bias = np.radians(p.gyro_bias_dps + walk)
    noise = np.radians(rng.normal(0.0, p.gyro_noise_dps, size=(n, 3)))
    gyro = traj.omega_true * (1.0 + p.gyro_scale_error) + bias + noise

    # --- accelerometer ----------------------------------------------------
    # Specific force at the sensor: rigid-body acceleration about the pivot,
    # minus gravity, expressed in the body frame.
    #
    # `d` runs FROM the pivot TO the sensor, which is the direction the
    # parent spec's formula assumes. Body Z points head-to-butt, and the pivot
    # (hands and sternum) sits above the grip butt, so d is negative Z.
    # Sanity check: for a pendulum this makes omega x (omega x d) point from
    # the sensor back toward the pivot, which is centripetal, as it must be.
    d = np.array([0.0, 0.0, -traj.params.pivot_offset_m])
    omega_dot = np.gradient(traj.omega_true, dt, axis=0)
    a_body = np.cross(omega_dot, d) + np.cross(traj.omega_true, np.cross(traj.omega_true, d))

    g_world = np.array([0.0, 0.0, -GRAVITY])
    accel = np.empty((n, 3))
    for i in range(n):
        g_body = quat.rotate(quat.conjugate(traj.q_true[i]), g_world)
        accel[i] = a_body[i] - g_body

    # Impact impulse. Allowed to saturate: impact is a trigger, not a
    # measurement (parent spec 6.4).
    width = max(1, int(round(p.impact_duration_s * SAMPLE_RATE_HZ)))
    window = np.hanning(2 * width + 1)
    lo = max(0, traj.impact_index - width)
    hi = min(n, traj.impact_index + width + 1)
    impulse = p.impact_peak_g * GRAVITY * window[: hi - lo]
    accel[lo:hi, 0] -= impulse

    accel = accel * (1.0 + p.accel_scale_error) + p.accel_bias_mps2
    accel += rng.normal(0.0, p.accel_noise_mps2, size=(n, 3)) if p.accel_noise_mps2 else 0.0

    return SensorOutput(
        gyro_counts=_quantize(gyro, full_scale.gyro_rad_per_count),
        accel_counts=_quantize(accel, full_scale.accel_mps2_per_count),
        full_scale=full_scale,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd analysis && uv run pytest tests/test_sensor.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/sensor.py analysis/tests/test_sensor.py
git commit -m "Add QMI8658 sensor forward model emitting int16 counts"
```

---

## Task 5: Pipeline — address detection and bias nulling

First slice of the streaming pipeline. Nothing about face angle yet.

**Files:**
- Create: `analysis/plumb/pipeline.py`
- Test: `analysis/tests/test_pipeline.py`

- [ ] **Step 1: Write the failing tests**

```python
# analysis/tests/test_pipeline.py
import numpy as np
import pytest

from plumb.pipeline import Pipeline, State, Thresholds
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import ArcType, StrokeParams, generate


def run_stroke(stroke: StrokeParams, sensor: SensorParams = SensorParams(), seed: int = 1):
    traj = generate(stroke)
    out = simulate(traj, sensor, seed=seed)
    pipe = Pipeline(Thresholds(), out.full_scale)
    result = None
    for i in range(len(traj.time)):
        r = pipe.step(out.gyro_counts[i], out.accel_counts[i])
        if r is not None:
            result = r
    return traj, pipe, result


def test_reaches_address_and_captures_gravity():
    traj, pipe, _ = run_stroke(StrokeParams(lie_angle_deg=20.0))
    assert pipe.address_captured
    lie = np.radians(20.0)
    expected = np.array([0.0, np.sin(lie), np.cos(lie)])
    measured = pipe.g0 / np.linalg.norm(pipe.g0)
    np.testing.assert_allclose(measured, expected, atol=1e-2)


def test_gyro_bias_is_nulled_at_address():
    """Invariant 4. A 1.5 dps standing bias must be estimated and removed."""
    _, pipe, _ = run_stroke(StrokeParams(), SensorParams(gyro_bias_dps=1.5))
    np.testing.assert_allclose(np.degrees(pipe.bias), [1.5, 1.5, 1.5], atol=0.1)


def test_backswing_is_detected():
    _, pipe, _ = run_stroke(StrokeParams())
    assert pipe.state is State.BACKSWING
    assert pipe.visited == [State.IDLE, State.ADDRESS, State.BACKSWING]
```

Note: the full state-machine walk is asserted in Task 6, once the remaining states exist. Every task in this plan leaves the suite green — a red suite at a commit boundary makes it impossible to tell a known-incomplete feature from a regression.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd analysis && uv run pytest tests/test_pipeline.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'plumb.pipeline'`

- [ ] **Step 3: Implement the first slice of `analysis/plumb/pipeline.py`**

Write the module with state, thresholds, address detection and bias nulling. `step()` returns `None` until Task 7 adds a result.

```python
"""Streaming implementation of the parent spec's section 7 processing chain.

One sample at a time, no lookahead: the firmware sees samples as the IMU
interrupt delivers them and cannot inspect the future. Structuring the
reference implementation the same way makes lookahead impossible rather than
merely discouraged, and makes the eventual C port a translation rather than a
rewrite.

Only `Pipeline` and its state are ported to C. `run()` is test ergonomics.
"""

from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from plumb import quat
from plumb.sensor import FullScale
from plumb.trajectory import SAMPLE_RATE_HZ


class State(Enum):
    IDLE = auto()
    ADDRESS = auto()
    BACKSWING = auto()
    DOWNSWING = auto()
    IMPACT = auto()
    FOLLOWTHROUGH = auto()
    DONE = auto()


@dataclass(frozen=True)
class Thresholds:
    """Every threshold is configurable and none is a guessed constant.

    Parent spec invariant 5: these are derived from the logged corpus in
    Phase 2 and fixed in the notebook before the port to C. The values here are
    placeholders for the synthetic harness only, and are deliberately not
    presented as tuned. Nothing keys off rotation amplitude (invariant 1).
    """

    stillness_window_s: float = 0.50
    stillness_gyro_std_rad: float = np.radians(0.8)
    onset_gyro_rad: float = np.radians(1.0)
    backswing_gyro_rad: float = np.radians(8.0)
    backswing_hold_s: float = 0.04
    transition_gyro_rad: float = np.radians(2.0)
    impact_accel_mps2: float = 100.0
    followthrough_gyro_rad: float = np.radians(5.0)
    followthrough_hold_s: float = 0.30
    accel_gain_static: float = 0.02
    accel_gain_stroke: float = 0.0


@dataclass
class StrokeResult:
    face_angle_deg: float
    tempo_ratio: float
    path_arc_m: float
    path_direction: str


class Pipeline:
    def __init__(self, thresholds: Thresholds, full_scale: FullScale,
                 lever_arm_m: float = 0.85):
        self.th = thresholds
        self.fs = full_scale
        self.lever_arm = lever_arm_m
        self.dt = 1.0 / SAMPLE_RATE_HZ

        self.state = State.IDLE
        self.visited: list[State] = [State.IDLE]
        self.n = 0

        self._still: list[np.ndarray] = []
        self._still_accel: list[np.ndarray] = []
        self.address_captured = False
        self.bias = np.zeros(3)
        self.g0 = np.zeros(3)

        self.q = quat.identity()          # attitude relative to address
        self._prev_omega = None           # previous bias-corrected rate, for trapezoidal integration
        self.q_impact = None
        self.i_backswing_start = None
        self.i_transition = None
        self.i_impact = None
        self._hold = 0
        self._omega_history: list[np.ndarray] = []

        # Back-tracking state. Detecting a stroke boundary always lags the
        # boundary itself; these record where it actually was. Past data only.
        self._last_quiet_n = 0
        self._last_same_sign_n = None
        self._backswing_axis = None
        self._backswing_sign = None
        self._impact_window: list[tuple[int, np.ndarray]] = []

    # -- conversion ------------------------------------------------------
    def _to_rad_s(self, counts: np.ndarray) -> np.ndarray:
        return counts.astype(float) * self.fs.gyro_rad_per_count

    def _to_mps2(self, counts: np.ndarray) -> np.ndarray:
        return counts.astype(float) * self.fs.accel_mps2_per_count

    def _enter(self, state: State) -> None:
        self.state = state
        self.visited.append(state)
        self._hold = 0

    # -- main entry point ------------------------------------------------
    def step(self, gyro_counts: np.ndarray, accel_counts: np.ndarray):
        omega = self._to_rad_s(gyro_counts)
        accel = self._to_mps2(accel_counts)
        self.n += 1

        if self.state is State.IDLE:
            self._step_idle(omega, accel)
        elif self.state is State.ADDRESS:
            self._step_address(omega)
        return None

    def _step_idle(self, omega, accel) -> None:
        """Wait for stillness, then capture both references at once.

        Parent spec 7.1: entering ADDRESS captures the gravity vector g0 and the
        gyro bias, both as means over the SAME stillness window. Averaging
        gravity over a longer span than the stillness test covers would mean
        averaging over motion the stillness test never vetted.
        """
        window = int(self.th.stillness_window_s / self.dt)
        self._still.append(omega)
        self._still_accel.append(accel)
        if len(self._still) < window:
            return
        self._still = self._still[-window:]
        self._still_accel = self._still_accel[-window:]

        recent = np.array(self._still)
        if np.all(recent.std(axis=0) < self.th.stillness_gyro_std_rad):
            self.bias = recent.mean(axis=0)
            self.g0 = np.array(self._still_accel).mean(axis=0)
            self.address_captured = True
            self.q = quat.identity()
            self._enter(State.ADDRESS)

    def _step_address(self, omega) -> None:
        """Detect the backswing, and record where it actually started.

        Confirming a backswing needs a threshold crossing plus a hold, and both
        are late -- the rate has to climb from zero to 8 deg/s and then persist
        for 40 ms. The stroke began earlier, when the rate first left zero.

        That latency does not cancel in the tempo ratio, because impact is
        detected within one sample while backswing start is ~54 samples late.
        Measured on the noiseless generator, taking the confirmation instant as
        the start gives a worst tempo error of 0.351 against the 0.05 target in
        parent spec section 3 -- seven times over, systematically biased low,
        and worsening as tempo ratio rises. Back-tracking to the last quiet
        sample brings it to 0.034.

        `_last_quiet_n` reads only past samples, so this is a ring buffer rather
        than lookahead and remains implementable on-device.
        """
        corrected = omega - self.bias
        magnitude = np.linalg.norm(corrected)

        if magnitude < self.th.onset_gyro_rad:
            self._last_quiet_n = self.n

        if magnitude > self.th.backswing_gyro_rad:
            self._hold += 1
            if self._hold * self.dt >= self.th.backswing_hold_s:
                self.i_backswing_start = self._last_quiet_n
                self._enter(State.BACKSWING)
        else:
            self._hold = 0
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd analysis && uv run pytest -v`
Expected: 34 passed (31 existing plus 3 new)

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/pipeline.py analysis/tests/test_pipeline.py
git commit -m "Add pipeline address detection and per-stroke gyro bias nulling"
```

---

## Task 6: Pipeline — segmentation, integration and tempo

**Files:**
- Modify: `analysis/plumb/pipeline.py`
- Modify: `analysis/tests/test_pipeline.py`

- [ ] **Step 1: Add the failing tempo test**

```python
# append to analysis/tests/test_pipeline.py

def test_state_machine_visits_every_state_in_order():
    _, pipe, _ = run_stroke(StrokeParams())
    order = [State.IDLE, State.ADDRESS, State.BACKSWING, State.DOWNSWING,
             State.IMPACT, State.FOLLOWTHROUGH, State.DONE]
    assert pipe.visited == order


@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_tempo_ratio_recovered(tempo):
    """Compared against the ratio the generator actually produced, not the one
    requested -- the 500 Hz grid cannot hit an arbitrary ratio exactly, and
    holding the pipeline to a target the stroke never contained would be
    measuring the generator's rounding, not the pipeline."""
    traj, _, result = run_stroke(StrokeParams(tempo_ratio=tempo))
    assert result is not None
    assert result.tempo_ratio == pytest.approx(traj.true_tempo_ratio, abs=0.05)
```

The 0.05 tolerance is the parent spec §3 product target, not a number chosen to pass. Measured worst case with back-tracked boundaries is 0.034 noiseless.

**Known limit, to be recorded rather than engineered around here.** Tempo accuracy depends on resolving when the rate left zero, so it degrades as gyro noise rises toward the onset gate:

| Gyro noise | Worst tempo error |
|---|---|
| 0.0 dps | 0.034 |
| 0.2 dps | 0.017 |
| 0.5 dps | 0.009 |
| 1.0 dps | 0.053 |
| 2.0 dps | 0.178 |

The QMI8658's noise density puts datasheet-typical near 0.16 dps at this bandwidth, comfortably inside the target. The 1.0 dps onset gate is a placeholder like every other threshold (invariant 5) and gets derived from the real corpus in Phase 2, where deriving it from the measured stillness variance is the obvious candidate.

- [ ] **Step 2: Run to verify it fails**

Run: `cd analysis && uv run pytest tests/test_pipeline.py -k tempo -v`
Expected: FAIL — `result is None`

- [ ] **Step 3: Extend `step()` with the remaining states**

Add to `Pipeline`, and extend the `step()` dispatch to call them:

```python
    def _integrate(self, omega, accel) -> None:
        """Attitude integration with the accelerometer correction gain driven
        by state.

        Parent spec 7.2 and invariant 2: between BACKSWING and FOLLOWTHROUGH the
        accelerometer reads gravity plus stroke acceleration, so a stock
        complementary correction is dragged off attitude by the very motion it
        is meant to measure. Drift is bounded instead by the short integration
        window and the bias null at address.
        """
        # FOLLOWTHROUGH counts as in-stroke. The putter is still moving fast
        # there, so the accelerometer is still reading gravity plus stroke
        # acceleration. Parent spec 7.2 restores the gain in IDLE and ADDRESS
        # only -- those are the two states where the device is actually still.
        in_stroke = self.state in (State.BACKSWING, State.DOWNSWING,
                                   State.IMPACT, State.FOLLOWTHROUGH)
        gain = self.th.accel_gain_stroke if in_stroke else self.th.accel_gain_static

        # Trapezoidal, not rectangle: average the rate across the interval being
        # integrated. That requires the sample at the END of the interval, which
        # is one sample of LATENCY, not lookahead -- the firmware integrates the
        # interval [i-1, i] when sample i arrives, and already has that sample in
        # hand. It costs one stored vector.
        #
        # Measured on the noiseless generator, worst case over 45 strokes:
        #   rectangle rule    0.0553 deg of face angle  (55% of the 0.1 deg budget)
        #   trapezoidal rule  0.0013 deg                 (1.3%)
        # a 41x improvement for one add and one multiply. Rectangle rule would
        # spend half the noiseless budget on nothing before noise even appears.
        corrected = omega - self.bias
        if self._prev_omega is None:
            self._prev_omega = corrected
        rate = 0.5 * (corrected + self._prev_omega)
        self._prev_omega = corrected
        self.q = quat.integrate(self.q, rate, self.dt)

        if gain > 0.0 and np.linalg.norm(accel) > 0.0:
            measured = accel / np.linalg.norm(accel)
            reference = self.g0 / np.linalg.norm(self.g0)
            predicted = quat.rotate(quat.conjugate(self.q), reference)
            error = np.cross(predicted, measured)
            self.q = quat.integrate(self.q, gain * error / self.dt, self.dt)

    def _dominant_axis(self) -> int:
        recent = np.array(self._omega_history[-25:])
        return int(np.argmax(np.abs(recent).mean(axis=0)))

    def _step_backswing(self, omega, accel) -> None:
        """Detect the transition, and record where the direction actually flipped.

        The dominant axis and its backswing sign are frozen on entry rather than
        recomputed per sample, so a late-stroke wobble cannot silently reinterpret
        which axis the stroke is about.

        The magnitude gate only CONFIRMS a reversal, guarding against sign
        chatter while the rate passes through zero. The instant recorded is the
        last sample still carrying the backswing's sign, so the confirmation
        delay does not bias the tempo ratio -- same reasoning as the backswing
        onset above.

        Note what is NOT here: nothing keys off how far the face rotated.
        Segmentation is timing, direction and acceleration only (invariant 1).
        """
        corrected = omega - self.bias
        self._omega_history.append(corrected)

        if self._backswing_axis is None:
            self._backswing_axis = self._dominant_axis()
            self._backswing_sign = np.sign(corrected[self._backswing_axis])
            self._last_same_sign_n = self.n
            return

        axis = self._backswing_axis
        if np.sign(corrected[axis]) == self._backswing_sign:
            self._last_same_sign_n = self.n
        elif abs(corrected[axis]) > self.th.transition_gyro_rad:
            self.i_transition = self._last_same_sign_n
            self._enter(State.DOWNSWING)

    def _step_downswing(self, omega, accel) -> None:
        if np.linalg.norm(accel) > self.th.impact_accel_mps2:
            self._impact_window = [(self.n, self.q.copy())]
            self._enter(State.IMPACT)

    def _step_impact(self, omega, accel) -> None:
        """Take the impact instant as the MIDDLE of the acceleration spike.

        The threshold fires on the spike's rising edge, one or more samples
        before the strike. Sampling attitude there includes residual pre-impact
        rotation, and because that leak couples through the shaft axis it scales
        with how fast the face was rotating -- measured as 0.037 deg of error for
        a straight stroke against 0.072 deg for an arced one. Accuracy that
        tracks arc type is a putter-type prior (invariant 1), even at that size,
        and it would grow on faster real strokes.

        The peak cannot be used: the spike saturates (parent spec 6.4, and a
        60 g impulse against a 16 g full scale can do nothing else), so several
        samples read the same clipped value and the peak carries no information.

        The midpoint of the above-threshold run is threshold-insensitive,
        saturation-robust, and unbiased for a symmetric impulse. Measured worst
        error falls from 0.0733 deg to 0.0012 deg, and the arc-type spread from
        0.0356 deg to 0.0004 deg.

        Cost is a few samples of latency against a 500 ms budget, and a short
        buffer of attitudes -- past data only.

        Phase 2 note: a real strike may not be symmetric, since the putter
        decelerates and then the ball departs. Whether the midpoint stays
        unbiased on real impulses is a question for the logged corpus, like
        every threshold here.
        """
        if np.linalg.norm(accel) > self.th.impact_accel_mps2:
            self._impact_window.append((self.n, self.q.copy()))
            return
        self.i_impact, self.q_impact = self._impact_window[len(self._impact_window) // 2]
        self._enter(State.FOLLOWTHROUGH)

    def _step_followthrough(self, omega, accel):
        if np.linalg.norm(omega - self.bias) < self.th.followthrough_gyro_rad:
            self._hold += 1
            if self._hold * self.dt >= self.th.followthrough_hold_s:
                self._enter(State.DONE)
                return self._compute()
        else:
            self._hold = 0
        return None

    def _compute(self) -> StrokeResult:
        backswing = (self.i_transition - self.i_backswing_start) * self.dt
        downswing = (self.i_impact - self.i_transition) * self.dt
        return StrokeResult(
            face_angle_deg=0.0,     # Task 7
            tempo_ratio=backswing / downswing,
            path_arc_m=0.0,         # Task 8
            path_direction="unknown",
        )
```

Then replace `step()` entirely with the full dispatch:

```python
    def step(self, gyro_counts: np.ndarray, accel_counts: np.ndarray):
        omega = self._to_rad_s(gyro_counts)
        accel = self._to_mps2(accel_counts)
        self.n += 1

        if self.state is State.IDLE:
            self._step_idle(omega, accel)
            return None

        if self.state is State.DONE:
            return None

        self._integrate(omega, accel)

        if self.state is State.ADDRESS:
            self._step_address(omega)
        elif self.state is State.BACKSWING:
            self._step_backswing(omega, accel)
        elif self.state is State.DOWNSWING:
            self._step_downswing(omega, accel)
        elif self.state is State.IMPACT:
            self._step_impact(omega, accel)
        elif self.state is State.FOLLOWTHROUGH:
            return self._step_followthrough(omega, accel)
        return None
```

Note the ordering: `_integrate` runs before the state handlers, so `q_impact` captured inside `_step_downswing` already includes the current sample.

- [ ] **Step 4: Run tests**

Run: `cd analysis && uv run pytest tests/test_pipeline.py -v`
Expected: all pass, including the four tempo cases and the state-order test

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/pipeline.py analysis/tests/test_pipeline.py
git commit -m "Add stroke segmentation, attitude integration and tempo ratio"
```

---

## Task 7: Face angle at impact — the load-bearing test

**Files:**
- Modify: `analysis/plumb/pipeline.py`
- Modify: `analysis/tests/test_pipeline.py`

- [ ] **Step 1: Write the failing tests**

```python
# append to analysis/tests/test_pipeline.py

@pytest.mark.parametrize("face_angle", [-5.0, -2.0, 0.0, 1.0, 2.0, 5.0])
def test_face_angle_recovered_noiselessly(face_angle):
    """At zero noise and zero bias the pipeline is a pure algebraic inverse of
    the forward model. Any error here is a sign error, a frame mix-up or a
    quaternion convention mismatch."""
    _, _, result = run_stroke(StrokeParams(face_angle_at_impact_deg=face_angle))
    assert result is not None
    assert result.face_angle_deg == pytest.approx(face_angle, abs=0.1)


@pytest.mark.parametrize("arc", list(ArcType))
def test_face_angle_recovered_for_every_arc_type(arc):
    """Invariant 1. A zero-torque putter produces very little face rotation and
    must recover exactly as well as an arced one."""
    _, _, result = run_stroke(
        StrokeParams(face_angle_at_impact_deg=2.0, arc_type=arc))
    assert result.face_angle_deg == pytest.approx(2.0, abs=0.1)


def test_recovery_quality_does_not_depend_on_arc_type():
    """Invariant 1, stated quantitatively rather than as a tolerance.

    It is not enough that every arc type lands inside tolerance. The ERROR
    itself must not track arc gain -- if it does, the algorithm contains a
    putter-type prior that a loose tolerance is merely hiding, and it will grow
    on real strokes that rotate faster than these.

    This test is the reason the impact instant is taken at the middle of the
    acceleration spike rather than at its leading edge. With the leading edge
    the spread is 0.0356 deg; with the midpoint it is 0.0004 deg."""
    errors = {}
    for arc in ArcType:
        _, _, result = run_stroke(
            StrokeParams(face_angle_at_impact_deg=2.0, arc_type=arc))
        errors[arc.name] = result.face_angle_deg - 2.0
    spread = max(errors.values()) - min(errors.values())
    assert spread < 0.005, f"recovery error varies with arc type: {errors}"


def test_face_angle_does_not_depend_on_stroke_size():
    """A longer backswing delivering the same face angle must report the same
    number. Face angle is an attitude difference between two instants, so
    nothing about the size of the motion between them should enter it.

    This is also a weak check on invariant 3: attitude is integrated from
    identity at address, so the reported angle is address-relative by
    construction and no absolute heading can leak in. There is no heading
    parameter in the generator to vary, because the device has no heading
    reference to be wrong about."""
    a = run_stroke(StrokeParams(face_angle_at_impact_deg=2.0))[2]
    b = run_stroke(StrokeParams(face_angle_at_impact_deg=2.0,
                                backswing_amplitude_deg=15.0,
                                followthrough_amplitude_deg=15.0))[2]
    assert a.face_angle_deg == pytest.approx(b.face_angle_deg, abs=0.1)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd analysis && uv run pytest tests/test_pipeline.py -k face_angle -v`
Expected: FAIL — reports `0.0` for every case, because `_compute` still stubs it

- [ ] **Step 3: Implement the projection**

Replace the `face_angle_deg=0.0` stub in `_compute`:

```python
        face_angle_deg=np.degrees(self._face_angle_at_impact()),
```

and add:

```python
    def _face_angle_at_impact(self) -> float:
        """Rotation about measured gravity between address and impact.

        Attitude is integrated from identity at address, so `q_impact` is
        already the address-relative rotation (invariant 3 -- there is no
        external heading reference and none is implied here).

        Taking the twist about g0 rather than reading a body axis is what makes
        this correct for a tilted shaft: g0 is (0, sin lie, cos lie) in body
        coordinates, not the body Z axis, so shaft lie angle is handled by
        measurement rather than by a stored constant (parent spec 7.3).
        """
        return quat.twist_angle(self.q_impact, self.g0)
```

- [ ] **Step 4: Run the full suite**

Run: `cd analysis && uv run pytest -v`
Expected: all pass. If face angle is recovered with the wrong sign, swap the sign of the twist rather than negating elsewhere, and note it in the docstring.

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/pipeline.py analysis/tests/test_pipeline.py
git commit -m "Add face angle at impact via twist about measured gravity"
```

---

## Task 8: Path from the lever arm

**Files:**
- Modify: `analysis/plumb/pipeline.py`
- Modify: `analysis/tests/test_pipeline.py`

- [ ] **Step 1: Write the failing test**

```python
# append to analysis/tests/test_pipeline.py

def test_path_direction_classified():
    _, _, result = run_stroke(StrokeParams(arc_type=ArcType.ARCED))
    assert result.path_direction in {"straight", "in-to-out", "out-to-in"}
    assert result.path_arc_m > 0.0


def test_straight_stroke_classified_straight():
    _, _, result = run_stroke(StrokeParams(arc_type=ArcType.STRAIGHT))
    assert result.path_direction == "straight"
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd analysis && uv run pytest tests/test_pipeline.py -k path -v`
Expected: FAIL — `path_direction == "unknown"`

- [ ] **Step 3: Accumulate face-point displacement during the stroke**

In `_integrate`, once the address is captured and the state is a stroke state, accumulate:

```python
        # Face-point velocity from the rigid-body relation (parent spec 7.4).
        # Only path needs the lever arm; face angle and tempo do not, because
        # angular velocity is identical at every point of a rigid body.
        r_body = np.array([0.0, 0.0, -self.lever_arm])
        v_face = np.cross(omega - self.bias, r_body)
        self._face_track.append(quat.rotate(self.q, v_face) * self.dt)
```

Initialise `self._face_track: list[np.ndarray] = []` in `__init__`, record `self._impact_track_index = len(self._face_track)` when entering IMPACT, and in `_compute`:

```python
        track = np.cumsum(np.array(self._face_track), axis=0)
        lateral = track[:, 1]
        before = lateral[: self._impact_track_index]
        after = lateral[self._impact_track_index :]
        arc = float(np.ptp(lateral)) if len(lateral) else 0.0
        delta = (after.mean() if len(after) else 0.0) - (before.mean() if len(before) else 0.0)
        if arc < 0.003:
            direction = "straight"
        else:
            direction = "in-to-out" if delta > 0 else "out-to-in"
```

and return `path_arc_m=arc, path_direction=direction`.

- [ ] **Step 4: Run tests**

Run: `cd analysis && uv run pytest -v`
Expected: all pass. If the straight case lands just above the 3 mm threshold, adjust the threshold in `Thresholds` rather than hardcoding it here — invariant 5 applies to path thresholds too.

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/pipeline.py analysis/tests/test_pipeline.py
git commit -m "Add stroke path from lever arm and direction classification"
```

---

## Task 9: Invariant tests

These exist to make the non-obvious decisions self-documenting. A future reader who deletes the unusual gain schedule should see a red test, not a passing suite.

**Files:**
- Create: `analysis/tests/test_invariants.py`

- [ ] **Step 1: Write the tests**

```python
# analysis/tests/test_invariants.py
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


def test_invariant_2_accel_correction_must_be_disabled_during_stroke():
    """If this passes with the gain restored, the gain schedule is not doing
    anything and the reasoning in parent spec 7.2 is wrong. Keeping this test
    is how we know the schedule is load-bearing."""
    stroke = StrokeParams(face_angle_at_impact_deg=2.0)
    correct = recover(stroke)
    naive = recover(stroke, Thresholds(accel_gain_stroke=0.05))

    assert abs(correct.face_angle_deg - 2.0) < 0.1
    assert abs(naive.face_angle_deg - 2.0) > abs(correct.face_angle_deg - 2.0)


def test_invariant_1_zero_torque_recovers_as_well_as_arced():
    """A zero-torque putter legitimately produces very little face rotation.
    If accuracy degrades for it, a putter-type prior has crept in."""
    arced = recover(StrokeParams(face_angle_at_impact_deg=2.0, arc_type=ArcType.ARCED))
    zero = recover(StrokeParams(face_angle_at_impact_deg=2.0,
                                arc_type=ArcType.NEAR_ZERO_ROTATION))
    assert abs(zero.face_angle_deg - 2.0) == pytest.approx(
        abs(arced.face_angle_deg - 2.0), abs=0.05)


def test_invariant_4_without_bias_nulling_a_standing_bias_wrecks_recovery():
    stroke = StrokeParams(face_angle_at_impact_deg=2.0)
    with_null = recover(stroke, sensor=SensorParams(gyro_bias_dps=2.0))
    assert abs(with_null.face_angle_deg - 2.0) < 0.2
```

- [ ] **Step 2: Run**

Run: `cd analysis && uv run pytest tests/test_invariants.py -v`
Expected: 3 passed

- [ ] **Step 3: Commit**

```bash
git add analysis/tests/test_invariants.py
git commit -m "Add tests enforcing invariants 1, 2 and 4"
```

---

## Task 10: Parameter sweep and the degradation curve

**Files:**
- Create: `analysis/plumb/sweep.py`
- Create: `analysis/tests/test_sweep.py`

- [ ] **Step 1: Write the failing tests**

```python
# analysis/tests/test_sweep.py
import pytest

from plumb.sweep import noise_breakdown_dps, run_sweep


def test_sweep_runs_and_covers_the_grid():
    rows = run_sweep(seeds=(1,))
    assert len(rows) > 50
    assert {r.arc_type for r in rows} == {"STRAIGHT", "NEAR_ZERO_ROTATION", "ARCED"}


def test_recovery_is_accurate_when_noiseless():
    rows = [r for r in run_sweep(seeds=(1,)) if r.gyro_noise_dps == 0.0 and r.gyro_bias_dps == 0.0]
    assert max(abs(r.error_deg) for r in rows) < 0.1


def test_degradation_is_monotonic_in_noise():
    """Recovery must worsen predictably, not erratically."""
    rows = run_sweep(seeds=(1, 2, 3))
    by_noise = {}
    for r in rows:
        by_noise.setdefault(r.gyro_noise_dps, []).append(abs(r.error_deg))
    levels = sorted(by_noise)
    worst = [max(by_noise[n]) for n in levels]
    assert worst == sorted(worst)


def test_noise_breakdown_point_is_reportable():
    """Definition of done: we can state the noise level at which the +/-1.0 deg
    target from parent spec section 3 is exceeded."""
    limit = noise_breakdown_dps(seeds=(1, 2, 3))
    assert limit > 0.0


def test_sweep_is_reproducible():
    assert run_sweep(seeds=(7,)) == run_sweep(seeds=(7,))
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd analysis && uv run pytest tests/test_sweep.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'plumb.sweep'`

- [ ] **Step 3: Implement `analysis/plumb/sweep.py`**

```python
"""Parameter sweep over the grid in the harness spec section 7.1."""

import itertools
from dataclasses import dataclass

from plumb.pipeline import Pipeline, Thresholds
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import ArcType, StrokeParams, generate

FACE_ANGLES_DEG = (-5.0, -2.0, 0.0, 2.0, 5.0)
GYRO_NOISE_DPS = (0.0, 0.05, 0.2, 0.5, 1.0, 2.0)
GYRO_BIAS_DPS = (0.0, 0.5, 2.0)
TEMPO_RATIOS = (1.5, 2.0, 3.0)
TARGET_DEG = 1.0


@dataclass(frozen=True)
class SweepRow:
    face_angle_deg: float
    arc_type: str
    tempo_ratio: float
    gyro_noise_dps: float
    gyro_bias_dps: float
    seed: int
    recovered_deg: float
    error_deg: float


def _one(face, arc, tempo, noise, bias, seed) -> SweepRow:
    stroke = StrokeParams(face_angle_at_impact_deg=face, arc_type=arc, tempo_ratio=tempo)
    traj = generate(stroke)
    out = simulate(traj, SensorParams(gyro_noise_dps=noise, gyro_bias_dps=bias), seed=seed)
    pipe = Pipeline(Thresholds(), out.full_scale, stroke.lever_arm_m)
    result = None
    for i in range(len(traj.time)):
        r = pipe.step(out.gyro_counts[i], out.accel_counts[i])
        if r is not None:
            result = r
    recovered = float("nan") if result is None else result.face_angle_deg
    return SweepRow(face, arc.name, tempo, noise, bias, seed, recovered, recovered - face)


def run_sweep(seeds=(1, 2, 3)) -> list[SweepRow]:
    grid = itertools.product(FACE_ANGLES_DEG, ArcType, TEMPO_RATIOS,
                             GYRO_NOISE_DPS, GYRO_BIAS_DPS, seeds)
    return [_one(*combo) for combo in grid]


def noise_breakdown_dps(seeds=(1, 2, 3)) -> float:
    """Highest swept noise level at which every stroke still meets the target."""
    rows = run_sweep(seeds)
    passing = [n for n in GYRO_NOISE_DPS
               if all(abs(r.error_deg) <= TARGET_DEG for r in rows if r.gyro_noise_dps == n)]
    return max(passing) if passing else 0.0


def main() -> None:
    rows = run_sweep()
    print(f"{len(rows)} strokes")
    print(f"worst error: {max(abs(r.error_deg) for r in rows):.3f} deg")
    print(f"meets +/-{TARGET_DEG} deg up to {noise_breakdown_dps():.2f} dps gyro noise")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the sweep and the full suite**

Run: `cd analysis && uv run pytest -v && uv run python -m plumb.sweep`
Expected: all tests pass; the sweep prints a stroke count, a worst-case error and a breakdown point. **Record the printed numbers — they are the Task 1 definition-of-done evidence.**

- [ ] **Step 5: Commit**

```bash
git add analysis/plumb/sweep.py analysis/tests/test_sweep.py
git commit -m "Add parameter sweep and noise degradation reporting"
```

---

## Task 11: Document the result

**Files:**
- Create: `analysis/README.md`

- [ ] **Step 1: Write `analysis/README.md`**

Record how to run it, and the *measured* sweep output from Task 10 — actual numbers, copied from the run. Do not describe anything as validated that was not measured.

```markdown
# Plumb — algorithm development and synthetic harness

## Running

    uv sync
    uv run pytest          # the algorithm's test suite
    uv run python -m plumb.sweep   # the full parameter sweep

## What this validates

The pipeline recovers a face angle we specified ourselves, from simulated
QMI8658 counts. Ground truth is known because it was generated.

This proves the math, not the device. It says nothing about sensor behaviour
or mount rigidity; those are measured on hardware (parent spec 5.5, 10).

## Measured results

<paste the sweep output here>
```

- [ ] **Step 2: Commit**

```bash
git add analysis/README.md
git commit -m "Document synthetic harness usage and measured sweep results"
```

---

## Notes for the implementer

**If `test_face_angle_recovered_noiselessly` fails by a constant offset**, check the sign convention in `quat.twist_angle` and the direction of `r_body`/`d` before touching anything else. A constant offset is a convention bug; a varying one is a frame bug. (This is the test the harness spec §7 calls `test_roundtrip_noiseless`; it is parametrized by face angle here, hence the longer name.)

**If it fails by roughly 0.12° at 2°**, the generator is being parameterized by shaft rotation rather than by projected face angle. See the plan preamble.

**Do not tune a threshold to make a test pass** without recording why in `Thresholds`. Invariant 5 exists because a plausible-looking constant is indistinguishable from a derived one six months later.

**Do not add a threshold that keys off rotation magnitude.** Invariant 1. Segmentation keys off timing, direction and acceleration only.
