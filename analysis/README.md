# Plumb — algorithm development and synthetic harness

Python side of the [Plumb](../README.md) putting stroke analyzer: a synthetic stroke harness
that validates the §7 fusion pipeline with **no hardware at all**.

## Running

```bash
uv sync
uv run pytest                   # the algorithm's test suite (78 tests, ~20 s)
uv run python -m plumb.sweep    # the full parameter sweep (~8 min)
```

## What this proves, and what it does not

The pipeline recovers a face angle **we specified ourselves**, from simulated QMI8658 counts.
Ground truth is known because it was generated.

This proves the *math*: quaternion conventions, frame handling, the ground-plane projection,
bias nulling, segmentation. It says **nothing** about sensor behaviour or mount rigidity.
Those are measured on hardware (parent spec §5.5 tap test, §10 validation study).

The value of doing it first is that when the board is mounted and the numbers look wrong, the
algorithm is already proven — so the error is the sensor or the mount, which is a
single-variable problem instead of a two-variable mystery.

## Layout

| Module | Role | Ports to C |
|---|---|---|
| `plumb/quat.py` | Quaternion primitives. Hamilton, scalar-first, body→reference. | No |
| `plumb/trajectory.py` | Ground-truth stroke generation | No — offline only |
| `plumb/sensor.py` | QMI8658 forward model, emits raw int16 counts | No — offline only |
| `plumb/pipeline.py` | The §7 chain under test. Streaming, one sample at a time. | **Yes — reference implementation** |
| `plumb/sweep.py` | Parameter grid driver | No |

`pipeline.py` is written C-shaped — an explicit state object advanced by `step()`, never
looking at a future sample — so the eventual firmware port is a translation rather than a
rewrite, and lookahead the firmware cannot have is structurally impossible rather than merely
discouraged.

## Measured results

From `uv run python -m plumb.sweep`, 2835 strokes over face angle (−5° … +5°), arc type
(straight / near-zero-rotation / arced), tempo (1.5:1 … 3:1), gyro noise and gyro bias.

```
strokes simulated      : 2835
detection rate         : 71.4%  (parent spec section 3 target: 98%)
worst face-angle error : 0.0756 deg  (target 1.0 deg)
meets face target up to: 0.50 dps gyro noise

 noise dps   detected   worst face err   worst tempo err
      0.00       100%           0.0012            0.0090
      0.05       100%           0.0087            0.0244
      0.10       100%           0.0160            0.0301
      0.20       100%           0.0307            0.0354
      0.50       100%           0.0756            0.0423
      1.00         0%              nan               nan
      2.00         0%              nan               nan
```

At 896.8 Hz, **tempo meets the §3 target of 0.05 at every noise level where the
stroke is detected at all** — worst 0.0423. At the previously assumed 500 Hz it
failed at 0.5 dps, at 0.0969.

Read the headline detection rate carefully: it is **100% at every noise level up to 0.5 dps
and 0% above it**, not a uniform 71%. See the detection cliff below.

### Against the definition of done

| Criterion | Result |
|---|---|
| Noiseless face angle well under 0.1° | **0.0012°** |
| Degradation graceful and predictable | Monotonic across four decades of noise |
| Noise level where ±1.0° is exceeded can be stated | Face angle holds to **0.5 dps**; the binding limit is detection, not accuracy |
| Zero-torque recovers as well as arced | Arc-type spread in error **0.000018°** |
| Reproducible from a seed, one command | Yes — `test_sweep_is_reproducible` |

For scale: the QMI8658's noise density puts datasheet-typical near **0.16 dps** at this
bandwidth, where worst face error is about 0.025°.

## Two limits worth knowing before hardware

**Detection has a cliff, and accuracy does not.** Above roughly 0.5 dps of gyro noise the
stillness threshold (0.8 dps) sits under the noise floor, ADDRESS never triggers, and no
stroke is reported at all. Face-angle *accuracy* is still degrading gracefully at that point —
it is detection that fails first. The stillness threshold therefore has a hard floor set by
the sensor's real noise, and deriving it from measured stillness variance is the obvious
Phase 2 candidate (§7.1, invariant 5).

**Tempo is still the tighter metric, but it now clears its target everywhere detection
works.** Worst case 0.0423 against the §3 target of 0.05, where face angle has roughly
thirteen times more margin. It was 0.0969 — a failure — at the previously assumed 500 Hz;
raising the rate to 896.8 Hz is what closed it, because tempo depends on resolving *when
the rate left zero* and that is limited by sample resolution and the noise floor.

The target is also partly definitional: the reference system will decide where a backswing
begins by its own criterion, so it deserves revisiting once paired measurements exist.

## Design decisions that are load-bearing

Each of these was measured, not assumed, and each has a test that fails if it is removed.

**Trapezoidal integration, not rectangle.** Holding each sample's rate constant across its
interval costs 0.0553° of face angle at 500 Hz — 55% of the noiseless budget — against 0.0013°
for trapezoidal. Averaging across the interval needs the sample at its end, which is one
sample of *latency*, not lookahead, and is implementable on-device.

**Impact is the middle of the acceleration spike, not its leading edge.** The threshold fires
on the rising edge, before the strike, and the residual pre-impact rotation leaks into the
twist about gravity in proportion to how fast the face is turning — arc-type-dependent
accuracy, which invariant 1 forbids. The peak cannot be used because the spike saturates by
design. The midpoint is threshold-insensitive and saturation-robust.

**Segmentation measures only the shaft-perpendicular rate.** Face rotation is rotation *about*
the shaft, so a full three-axis magnitude mixes it into the swing measurement and makes the
onset instant depend on how much the face turns. The perpendicular component is exactly
|θ̇| and is independent of face rotation by construction.

**Stroke boundaries are back-extrapolated.** Any threshold is crossed after the event it is
detecting. Taking the crossing as the boundary cost 0.351 of tempo error against a 0.05 target.
Extrapolating the rate linearly back to zero brings it to 0.016 and makes the result barely
depend on where the threshold sits.

**Accelerometer correction is gated on measured stillness, not on the state label.** BACKSWING
is not confirmed until ~53 samples after motion begins, so the machine still reads ADDRESS
while the stroke is underway. Correcting against the accelerometer there is exactly the failure
invariant 2 describes, and the injected error is then frozen in.

**The correction gain is a rate, not a per-sample step.** Writing `gain * error / dt` and then
integrating over `dt` cancels the `dt`, so each *sample* applies a fixed rotation and the
correction applied per *second* scales with the sample rate — the same constant silently means
something different at every ODR. Moving from 500 Hz to 896.8 Hz exposed it: worst face-angle
error under noise went from 0.014° to 0.236° and the degradation curve stopped being monotonic,
while nothing about the modelled sensor had got worse. Changing the sample rate is, in effect,
a test for rate-independence, and it is worth running deliberately after any change to the
filter.

**Face angle is not shaft rotation.** Rotating a shaft tilted by lie angle `λ` through `φ`
moves the face normal by `atan(tan(φ)·cos(λ))` in the ground plane. At λ=20° and φ=2° that is
1.880°, not 2.000°. The generator is parameterized by the reportable quantity and solves for
the shaft rotation, so ground truth is the number the device should report.

## Conventions

Fixed, and documented in
[the implementation plan](../docs/superpowers/plans/2026-09-20-synthetic-stroke-harness.md).

- **Quaternions:** Hamilton, scalar-first `(w, x, y, z)`, body→reference.
- **World frame:** X = target line, Y = horizontal perpendicular, Z = up, gravity `(0,0,−9.81)`.
- **Body frame:** Z = shaft axis (head→butt), X = face normal.
- **Full scale:** gyro ±256 dps, accel ±16 g. The gyro range is *not* ±250 — that
  value does not exist on the QMI8658, whose table is powers of two, and the parent
  spec was amended on 2026-09-21. The full-scale divisor is 2¹⁵, not `INT16_MAX`:
  the datasheet's 128 LSB/dps at ±256 dps settles it, since 256 × 128 = 32768.
- **Sample rate:** 896.8 Hz — still *nominal*, not measured. The QMI8658's ODR steps
  derive from the gyro's natural frequency rather than round numbers, so the spec's
  original 500 Hz does not exist; the neighbours are 448.4 and 896.8 Hz, and 896.8
  was chosen because tempo error scales with sample resolution and tempo is the
  binding constraint (parent spec §6.4, amended 2026-09-21). MEMS oscillators run a
  few percent off nominal, and that error goes straight into every integrated angle,
  so this constant and every number above get re-baselined once Task 8 of the
  bring-up plan measures the achieved rate on the real board.
- **Randomness:** explicit `numpy.random.Generator`, seeds passed as arguments. Global random
  state is never used — the accuracy study is a deliverable, and a result nobody can re-run is
  worth substantially less.
