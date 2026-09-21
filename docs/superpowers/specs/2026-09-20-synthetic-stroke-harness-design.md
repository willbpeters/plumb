# Synthetic Stroke Harness — Design Specification

**Working name:** Plumb
**Date:** 2026-09-20
**Author:** Will Peters
**Status:** Approved for planning
**Parent spec:** `2026-09-15-putting-analyzer-design.md` — section references below refer to it.

---

## 1. Purpose

Validate the entire §7 fusion pipeline with zero hardware, by forward-modeling the sensor and
checking whether the pipeline recovers a face angle we specified ourselves.

Ground truth is known because we generated it. When the pipeline is later run against real
strokes, the math will already be proven, so any error observed on hardware is attributable to
the sensor or the mount — a single-variable problem instead of a two-variable mystery.

This harness is the algorithm's test suite. It is not a report generated at the end of
development; it is the thing the algorithm is developed against.

## 2. Scope

**In scope:** ground-truth stroke generation, a QMI8658 forward model, a streaming
implementation of the §7 processing chain, and a parameter sweep asserting recovery accuracy.

**Out of scope:** plotting, report generation, and the accuracy study itself (§10 of the parent
spec). Those require real paired measurements and a format that cannot usefully be guessed at
before that data exists.

## 3. Architecture

Four modules under `analysis/plumb/`, each with a single responsibility.

| Module | Role | Ported to C |
|---|---|---|
| `trajectory.py` | Ground-truth stroke generator (§4 below) | No — offline only |
| `sensor.py` | QMI8658 forward model (§5 below) | No — offline only |
| `pipeline.py` | The §7 processing chain under test (§6 below) | **Yes — reference implementation** |
| `sweep.py` | Parameter grid driver (§7 below) | No |

Only `pipeline.py` is written in a C-shaped style. The generator and forward model never run
on-device, so they are written plainly; lookahead is meaningless in code that produces an
entire trajectory by construction.

### 3.1 Why the pipeline is streaming, not vectorized

The firmware sees one sample at a time at 500 Hz with no knowledge of the future. Vectorized
NumPy over a complete time series invites algorithms that silently depend on lookahead —
scanning an array for its maximum to locate impact, filtering the whole signal before
segmenting it, integrating over a window whose end is not yet known.

Such a pipeline can validate perfectly here and be unimplementable on-device. The failure
surfaces only at port time, when the algorithm, the sensor and the mount are all unproven
simultaneously — exactly the multi-variable situation this harness exists to prevent.

`pipeline.py` therefore exposes an explicit state object and a `step(sample)` function
consuming one sample at a time. Lookahead becomes structurally impossible rather than
prohibited by discipline. The cost is Python execution speed, which the sweep size does not
make significant.

## 4. Ground-truth trajectory generator

Produces `q_true(t)` and `omega_true(t)` at 500 Hz for a parameterized putting stroke.

### 4.1 Parameters

| Parameter | Meaning |
|---|---|
| Backswing amplitude | Peak swing rotation from address |
| Total duration | Stroke length in seconds |
| Tempo ratio | Backswing time : downswing time, default 2:1 |
| Face angle at impact | **The quantity to be recovered** |
| Arc type | Straight-back-straight-through, arced, or near-zero-rotation |
| Lever arm `r` | Sensor to sweet spot |
| Pivot offset `d` | Sensor to rotation center |
| Lie angle | Shaft tilt from vertical — see §4.3 |

### 4.2 Motion model

Rotation about a pivot near the golfer's hands and sternum, with a smooth pendulum-like
velocity profile. This does not need to be biomechanically faithful. It needs to be *known*.

### 4.3 Three rotational degrees of freedom are required

The generator models three distinct rotations, and conflating them invalidates the test:

1. **Swing rotation** — about a horizontal axis through the pivot. Drives tempo and path.
2. **Face rotation** — about the shaft axis. This is the recovered quantity.
3. **Lie angle** — a fixed tilt so the shaft is not vertical.

The lie angle is the one most easily omitted and must not be. §7.3 of the parent spec projects
the relative rotation onto the ground plane defined by the measured gravity vector `g0`. If the
shaft is vertical in the generator, that projection degenerates to a no-op and passes whether
the projection math is correct or not. A realistic lie angle is what makes the projection step
a real test rather than a trivially satisfied one.

Separating swing rotation from face rotation matters for the same reason: a single-axis
generator cannot distinguish a pipeline that recovers face angle from one that merely reports
swing amplitude.

## 5. Sensor forward model

Given a trajectory, produce what a QMI8658 would actually report.

### 5.1 Gyroscope

```
omega_meas = omega_true + bias(t) + white_noise
```

`bias(t)` is a slow random walk. Noise is scaled to a configurable angular random walk.

### 5.2 Accelerometer

The accelerometer measures specific force. With `g_vec = (0, 0, -9.81)` in the world frame and
`R` the body-to-world rotation:

```
a_world = omega_dot x d + omega x (omega x d)
a_meas  = R^T (a_world - g_vec)
```

### 5.3 Error chain, applied in order

Scale-factor error, then bias, then white noise, then **quantization to int16 at the configured
full-scale range** (gyro ±250 dps, accel ±16 g).

The quantization step is easy to omit and it matters: it sets a hard floor on achievable
resolution that no amount of filtering recovers.

### 5.4 Impact

An acceleration impulse is injected at `t_impact` and **allowed to saturate**. Clipping is
expected and acceptable — impact is a trigger, not a measurement (§6.4).

### 5.5 Output boundary

The forward model emits **raw int16 counts**, not floating-point physical units. The pipeline
receives those counts together with the configured full-scale range and performs its own
conversion.

This makes it structurally impossible to bypass quantization. A float-valued boundary would
permit clean values to pass through untouched, and the omission would be invisible.

## 6. Pipeline under test

A streaming implementation of the parent spec's §7 chain:

stillness/ADDRESS detection → gyro bias null → backswing/transition/impact segmentation →
orientation integration with accelerometer correction disabled during the stroke → face angle
at impact projected onto the ground plane defined by `g0` → tempo ratio → path from
`v_face = omega x r`.

### 6.1 Interface

```
run(counts, fsr) -> StrokeResult
```

where `fsr` is the configured full-scale range for both sensors.

The pipeline never receives ground truth. Truth remains in the test, which compares after the
fact. This boundary is deliberate: a harness whose pipeline can see the answer validates
itself, which is the most common defect in synthetic test rigs and the hardest to notice.

### 6.2 State

An explicit state object mirroring the §7.1 state machine, advanced by `step(sample)`, which
consumes exactly one sample and returns a completed `StrokeResult` or nothing.

`run()` is a thin convenience wrapper that feeds samples to `step()` in order and returns the
first completed result. It exists for test ergonomics and holds no logic of its own. Only
`step()` and the state object are ported to C; `run()` has no on-device counterpart, because
on-device the sample source is the IMU interrupt rather than an array.

### 6.3 Invariants enforced structurally

| Invariant | Enforcement |
|---|---|
| 1 — no putter-type priors | Accuracy assertions run parametrized across all arc types; the near-zero-rotation case must match the arced case |
| 2 — accel gain to ~0 during stroke | Gain is an explicit parameter driven by the state machine; a dedicated test asserts that leaving it enabled degrades the result |
| 4 — per-stroke gyro bias nulling | Bias is nulled in the ADDRESS stillness window; the bias sweep in §7 fails without it |
| 5 — thresholds derived, never guessed | All thresholds live in a single `Thresholds` dataclass, fully configurable, with no hardcoded plausible-looking defaults |

The test for invariant 2 exists specifically to document why the invariant exists. Without a
test that fails when the gain is restored, a future reader has no evidence that the
unusual-looking gain schedule is load-bearing.

## 7. Test strategy

Tests are written before the implementation they exercise.

| Test | Assertion |
|---|---|
| `test_roundtrip_noiseless` | Zero noise, zero bias: recovered face angle matches specified value to well under 0.1° |
| Arc-type parametrization | The above, across straight / arced / near-zero-rotation |
| `test_degradation` | Recovery degrades gracefully as noise and bias increase, and reports the noise level at which the ±1.0° target from §3 is exceeded |
| `test_accel_gain_matters` | Leaving accelerometer correction enabled through the stroke measurably degrades recovery |
| `test_reproducible` | Identical seed produces identical results |

`test_roundtrip_noiseless` is the load-bearing test. At zero noise the pipeline is a pure
mathematical inverse of the forward model, so any error is a sign error, a frame mix-up or a
quaternion convention mismatch. It is written first and it fails first.

### 7.1 Sweep grid

| Swept | Range |
|---|---|
| Gyro bias | 0 to a realistically large offset |
| Noise level | 0 to well past datasheet-typical |
| Face angle at impact | −5° to +5° |
| Arc type | Straight, arced, near-zero-rotation |
| Tempo | 1.5:1 to 3:1 |

### 7.2 Definition of done

- Noiseless recovery well under 0.1°.
- Degradation is graceful and predictable, and the noise level at which ±1.0° is exceeded can
  be stated.
- The near-zero-rotation case recovers as accurately as the arced case. If it does not, a
  putter-type prior has crept in and must be found and removed.
- Results are reproducible from a seed, and the whole sweep runs from one command.

## 8. Tooling and reproducibility

- `uv` for dependency management and locking. Python 3, NumPy, SciPy, pytest.
- Randomness via explicit `numpy.random.Generator` instances. Seeds are passed as arguments;
  global random state is never used.
- The sweep runs from a single command.

Reproducibility is a requirement rather than a convenience: the accuracy study is a deliverable
of equal standing with the firmware (§15), and a result nobody can re-run is worth
substantially less.

## 9. Open items

None blocking. Threshold values remain parameterized until the Phase 2 corpus exists, per
invariant 5.
