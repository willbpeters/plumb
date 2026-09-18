# Handoff — Plumb

**Date:** 2026-09-18
**For:** Claude Code, picking this project up cold
**From:** design session with Will (project owner)

---

## Read first

1. `CLAUDE.md` — hard invariants. Eight of them. They are the decisions that fail silently.
2. `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` — the full spec, and the
   source of truth. Section numbers throughout this file refer to it.

Do not start writing code until you have read both.

---

## What this is

A putting stroke analyzer that mounts in the butt of a putter grip. A 6-axis IMU (QMI8658) on
a Waveshare ESP32-S3-Touch-LCD-1.28 derives **face angle at impact**, **tempo ratio** and
**stroke path**, and shows them on a 240×240 round LCD immediately after each putt. No phone,
no app, no network.

The hardware is the easy part. The project's substance is deriving putter face orientation
from a sensor mounted a meter away from the face, and then **validating that derivation
against an independent reference system** with real statistics. The accuracy study is a
deliverable of equal standing with the firmware.

Relevant physics, because it shapes everything: angular velocity is identical at every point
of a rigid body, so **face angle and tempo need no putter geometry at all**. Only path does,
because path is translation and scales with the sensor-to-face lever arm. (§2)

---

## Current state

**Nothing is built.** The repository contains the spec, this handoff, `CLAUDE.md`, and empty
directories. Two commits, both documentation.

**Hardware is in transit.** The Waveshare ESP32-S3-Touch-LCD-1.28 has been identified but not
yet confirmed ordered — check with Will. Nothing that requires the physical board can proceed.

**Timeline:** 15 weeks, targeting completion before the end of Fall 2026 instruction
(11 December). Will is abroad in Spring 2027, which is a hard stop on physical work. There is
no slack on the back end. (§13)

---

## What is unblocked right now

Only one thing is on the critical path and buildable today. Start there.

### ▸ Task 1 — Synthetic stroke harness (`analysis/`)

**Goal: validate the entire fusion pipeline with zero hardware**, by forward-modeling the
sensor and checking whether the pipeline recovers a face angle we specified ourselves.

This is the algorithm's test suite. Ground truth is known because we generated it. When the
board arrives, the math will already be proven, so any error observed on real hardware is the
*sensor or the mount* — a single-variable problem instead of a two-variable mystery.

**1a. Stroke trajectory generator.**

Produce a ground-truth motion for a putting stroke, parameterized by:

- Backswing amplitude and total duration
- Tempo ratio (backswing time : downswing time), default 2:1
- **Face angle at impact** — the quantity we will try to recover
- Arc type: straight-back-straight-through, or an arc with a configurable amount of face
  rotation. Include a near-zero-rotation case to represent a zero-torque putter.
- Lever arm `r` (sensor to sweet spot) and pivot offset `d` (sensor to rotation center)

Model the stroke as rotation about a pivot near the golfer's hands/sternum. A smooth
pendulum-like profile is fine — this does not need to be biomechanically perfect, it needs to
be *known*.

Output: time series of true orientation `q_true(t)` and true angular velocity `ω_true(t)` at
500 Hz.

**1b. Sensor forward model.** Given the trajectory, produce what a QMI8658 would actually
report.

Gyroscope (body frame):
```
ω_meas = ω_true + bias(t) + white_noise
```
with `bias(t)` as a slow random walk and noise scaled to a configurable angular random walk.

Accelerometer measures **specific force**, so with `g_vec = (0, 0, −9.81)` in world frame and
`R` the body-to-world rotation:
```
a_world = ω̇ × d + ω × (ω × d)
a_meas  = Rᵀ (a_world − g_vec)
```

Then apply, in this order: scale-factor error, bias, white noise, and **quantization to int16
at the configured full-scale range** (gyro ±250 dps, accel ±16 g). The quantization step is
easy to forget and it matters — it sets a hard floor on achievable resolution.

Inject an acceleration impulse at `t_impact`. **Let it saturate.** Clipping is expected and
acceptable; impact is a trigger, not a measurement. (§6.4)

**1c. Pipeline under test.** Implement the §7 processing chain in Python:

stillness/ADDRESS detection → gyro bias null → backswing/transition/impact segmentation →
orientation integration with accelerometer correction disabled during the stroke → face angle
at impact, projected onto the ground plane defined by `g₀` → tempo ratio → path from
`v_face = ω × r`.

**1d. The test sweep.** Assert recovery accuracy across a grid of conditions:

| Swept | Range |
|---|---|
| Gyro bias | 0 to a realistically large offset |
| Noise level | 0 to well past datasheet-typical |
| Face angle at impact | −5° to +5° |
| Arc type | straight, arced, near-zero-rotation |
| Tempo | 1.5:1 to 3:1 |

**Definition of done for Task 1:**

- At zero noise and zero bias, recovered face angle matches the specified value to well under
  0.1°. If it does not, the math is wrong — this is the test that catches sign errors, frame
  mix-ups and bad quaternion conventions.
- Recovery degrades *gracefully and predictably* as noise and bias increase, and you can state
  the noise level at which it exceeds the ±1.0° target from §3.
- The near-zero-rotation (zero-torque) case recovers as accurately as the arced case. **If it
  does not, a putter-type prior has crept in** — find it and remove it. See invariant 1.
- Results are reproducible from a seed, and the whole sweep runs from one command.

Write the tests first. The sweep is not a report you generate at the end; it is the thing you
develop against.

---

## What is blocked on hardware

Do not start these. Listed so you know the shape of what follows.

| Phase | Blocked on |
|---|---|
| 0. Bring-up — battery, display, IMU stream, charging | Board |
| 1. Mount — base/puck print, grip survey, **tap test** | Board + putters |
| 2. Logging — LittleFS, USB dump, first real corpus | Board |
| 3–5. Tempo, face angle in C, path, UI | Board |
| 6. Validation study | All of the above |

**Phase 1 contains a hard gate.** The tap test (§5.5) measures whether the rubber grip mount is
rigid enough — strike the head, FFT the ringdown, find the mount's resonance. Above ~500 Hz is
fine; below ~200 Hz means the mount is flexing and no amount of good math will reach ±1°. If it
fails, the escalation path in §5.5 runs *before* any further firmware work. Every downstream
accuracy number depends on this.

---

## Parallel work that needs no code

Will owns these. Mentioned so you do not duplicate or block on them:

- Grip compatibility survey (§5.6) — many putter grips have sealed butt caps, and zero-torque
  grips are often non-round. This must happen before base geometry is frozen.
- Printing base candidates on the Bambu A1.
- Contacting Sac State Kinesiology about optical motion capture access (§10.2). Long lead time,
  free, better ground truth than any commercial putting analyzer.

---

## Open questions for Will

Ask rather than assume:

1. Has the board actually been ordered, and what is the delivery date?
2. Which three putters are in the study, and is he willing to modify a grip? (Blade, mallet and
   zero-torque are needed for §10.3.)
3. Has Kinesiology responded? If not, the SAM PuttLab fallback needs budgeting.
4. `analysis/` dependency management — plain `requirements.txt`, or `uv`/`poetry`?

---

## Working agreement

- The spec is the contract. If something in it is wrong or unclear, say so and propose an
  amendment — do not quietly work around it.
- Commit the spec amendment in the same change as the code that depends on it.
- Never claim something is validated, passing or complete without having run it and seen the
  output.
- Will is a CS student who wants to understand the reasoning, not just receive working code.
  Explain the *why* behind non-obvious choices, especially in the sensor fusion work.
