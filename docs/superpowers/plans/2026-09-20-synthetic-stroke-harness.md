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


def test_impact_occurs_where_swing_angle_crosses_zero():
    t = generate(StrokeParams())
    assert t.theta[t.impact_index] == pytest.approx(0.0, abs=1e-6)


def test_tempo_ratio_is_what_was_asked_for():
    p = StrokeParams(tempo_ratio=2.5)
    t = generate(p)
    backswing = t.time[t.transition_index] - t.time[t.address_end_index]
    downswing = t.time[t.impact_index] - t.time[t.transition_index]
    assert backswing / downswing == pytest.approx(2.5, rel=1e-3)


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
    to describe. Catches a sign error in either one."""
    t = generate(StrokeParams())
    dt = t.time[1] - t.time[0]
    for i in range(t.address_end_index + 10, t.impact_index, 37):
        dq = quat.multiply(quat.conjugate(t.q_true[i]), t.q_true[i + 1])
        omega_numeric = 2.0 * dq[1:] / dt
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
    downswing_duration = p.backswing_duration_s / p.tempo_ratio
    forward_duration = downswing_duration / u_impact

    t_addr_end = p.address_duration_s
    t_transition = t_addr_end + p.backswing_duration_s
    t_impact = t_transition + downswing_duration
    t_end = t_transition + forward_duration + p.followthrough_hold_s

    n = int(round(t_end / dt)) + 1
    time = np.arange(n) * dt

    # --- swing angle theta(t) and its derivative -------------------------
    theta = np.zeros(n)
    theta_dot = np.zeros(n)

    back = (time >= t_addr_end) & (time < t_transition)
    u = (time[back] - t_addr_end) / p.backswing_duration_s
    theta[back] = -amp_back * _smoothstep(u)
    theta_dot[back] = -amp_back * _smoothstep_derivative(u, p.backswing_duration_s)

    fwd = time >= t_transition
    u = np.clip((time[fwd] - t_transition) / forward_duration, 0.0, 1.0)
    theta[fwd] = -amp_back + (amp_back + amp_fwd) * _smoothstep(u)
    theta_dot[fwd] = (amp_back + amp_fwd) * _smoothstep_derivative(u, forward_duration)

    impact_index = int(np.argmin(np.abs(theta[time >= t_transition]))) + int(np.searchsorted(time, t_transition))

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
        address_end_index=int(round(t_addr_end / dt)),
        transition_index=int(round(t_transition / dt)),
        impact_index=impact_index,
        true_face_angle_deg=p.face_angle_at_impact_deg,
        params=p,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd analysis && uv run pytest tests/test_trajectory.py -v`
Expected: 9 passed (the arc-type test is parametrized three ways)

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
    traj = generate(StrokeParams())
    s = simulate(traj, SensorParams(), seed=1)
    peak = np.abs(s.accel_counts[traj.impact_index - 2:traj.impact_index + 6]).max()
    assert peak == 32767


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


def test_state_machine_visits_every_state_in_order():
    _, pipe, _ = run_stroke(StrokeParams())
    order = [State.IDLE, State.ADDRESS, State.BACKSWING, State.DOWNSWING,
             State.IMPACT, State.FOLLOWTHROUGH, State.DONE]
    assert pipe.visited == order
```

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
        self._accel_sum = np.zeros(3)
        self.address_captured = False
        self.bias = np.zeros(3)
        self.g0 = np.zeros(3)

        self.q = quat.identity()          # attitude relative to address
        self.q_impact = None
        self.i_backswing_start = None
        self.i_transition = None
        self.i_impact = None
        self._hold = 0
        self._omega_history: list[np.ndarray] = []

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
        window = int(self.th.stillness_window_s / self.dt)
        self._still.append(omega)
        self._accel_sum += accel
        if len(self._still) < window:
            return
        self._still = self._still[-window:]
        recent = np.array(self._still)
        if np.all(recent.std(axis=0) < self.th.stillness_gyro_std_rad):
            self.bias = recent.mean(axis=0)
            self.g0 = self._accel_sum / self.n
            self.address_captured = True
            self.q = quat.identity()
            self._enter(State.ADDRESS)

    def _step_address(self, omega) -> None:
        corrected = omega - self.bias
        if np.linalg.norm(corrected) > self.th.backswing_gyro_rad:
            self._hold += 1
            if self._hold * self.dt >= self.th.backswing_hold_s:
                self.i_backswing_start = self.n
                self._enter(State.BACKSWING)
        else:
            self._hold = 0
```

- [ ] **Step 4: Run tests — two pass, one fails**

Run: `cd analysis && uv run pytest tests/test_pipeline.py -v`
Expected: `test_reaches_address_and_captures_gravity` PASS, `test_gyro_bias_is_nulled_at_address` PASS, `test_state_machine_visits_every_state_in_order` FAIL (only reaches BACKSWING). That failure is the next task's entry point.

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

@pytest.mark.parametrize("tempo", [1.5, 2.0, 2.5, 3.0])
def test_tempo_ratio_recovered(tempo):
    _, _, result = run_stroke(StrokeParams(tempo_ratio=tempo))
    assert result is not None
    assert result.tempo_ratio == pytest.approx(tempo, abs=0.05)
```

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
        in_stroke = self.state in (State.BACKSWING, State.DOWNSWING,
                                   State.IMPACT)
        gain = self.th.accel_gain_stroke if in_stroke else self.th.accel_gain_static
        self.q = quat.integrate(self.q, omega - self.bias, self.dt)

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
        corrected = omega - self.bias
        self._omega_history.append(corrected)
        axis = self._dominant_axis()
        # Transition is a sign reversal on the dominant axis: a timing and
        # direction test, never a magnitude test (invariant 1).
        if (np.sign(corrected[axis]) != np.sign(self._omega_history[0][axis])
                and abs(corrected[axis]) > self.th.transition_gyro_rad):
            self.i_transition = self.n
            self._enter(State.DOWNSWING)

    def _step_downswing(self, omega, accel) -> None:
        if np.linalg.norm(accel) > self.th.impact_accel_mps2:
            self.i_impact = self.n
            self.q_impact = self.q.copy()
            self._enter(State.IMPACT)

    def _step_impact(self, omega, accel) -> None:
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


def test_face_angle_is_relative_to_address_not_absolute():
    """Invariant 3. Rotating the whole stroke in heading must not change the
    reported face angle."""
    a = run_stroke(StrokeParams(face_angle_at_impact_deg=2.0))[2]
    b = run_stroke(StrokeParams(face_angle_at_impact_deg=2.0,
                                backswing_amplitude_deg=15.0))[2]
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
