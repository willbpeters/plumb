# Handoff — Plumb

**Date:** 2026-09-21
**For:** Claude Code, picking this project up cold
**Supersedes:** the 2026-09-18 handoff, which was written before anything was built

---

## Read first

1. `CLAUDE.md` — eight hard invariants. They are the decisions that fail silently.
2. `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` — the spec, and the source
   of truth. **It has been amended three times; see "Spec amendments" below.**
3. `docs/bringup-results.md` — everything the real hardware has told us, including two open
   defects. Read this before touching firmware.

Repo: `C:\Users\willi\Dev\plumb`, pushed to `github.com/willbpeters/plumb` (private).

---

## What this is

A putting stroke analyzer that mounts in the butt of a putter grip. A 6-axis IMU (QMI8658) on a
Waveshare ESP32-S3-Touch-LCD-1.28 derives **face angle at impact**, **tempo ratio**, **stroke
path** and **impact speed**, and shows them on a 240×240 round LCD immediately after each putt.
No phone, no app, no network.

The substance is deriving putter face orientation from a sensor a metre from the face, and
proving the derivation. Angular velocity is identical at every point of a rigid body, so face
angle and tempo need no putter geometry at all. Only path does, because path is translation.
(§2)

---

## Current state

**The board is in hand and working.** The algorithm is proven in simulation. The instrument
firmware runs but has an open defect that blocks every hardware measurement.

### Done

| | |
|---|---|
| **Synthetic harness** (`analysis/`) | 80 tests. Face angle recovers to **0.0012°** noiseless, 0.0756° at 0.5 dps gyro noise, against a ±1.0° target. Tempo 0.009 against 0.05. |
| **Bring-up smoke test** | Toolchain, flashing, I²C, IMU identity all confirmed on hardware. |
| **IMU streaming instrument** | `firmware/bringup-arduino/imu_stream` — QMI8658 driver, CSV and binary framing, serial control, host capture in `analysis/tools/capture.py`. |
| **Screen design** | Five screens designed and reviewed. Decisions recorded below. |

### Open defects — read `docs/bringup-results.md` for the detail

1. **~20% of samples lost in the FIFO.** `FIFO_OVFLOW` sets on 224 of 225 consecutive drains.
   The datasheet confirms the FIFO does not accept samples while `FIFO_RD_MODE` is set, so time
   spent draining is data the sensor never stores. Predicted loss 19.5%, measured 20.8%.
2. **The FIFO read returns corrupted data.** Noise depends on position within the batch —
   samples 2–7 read 0.37 dps, samples 28–63 read 1.0–1.3. Impossible for a stationary sensor.

**Consequence: no hardware measurement is trustworthy yet.** §9.1 through §9.5 of the
instrument spec are all blocked or invalid.

3. **Path arc magnitude is ~61% of truth** — a known gap, pinned by a test, documented in
   `test_path_arc_magnitude_is_known_to_fall_short_of_truth`. Direction classification, which
   is what the screen shows, is unaffected.

4. **Initialisation is intermittent.** First boot after flashing can fail the config read-back;
   a reset clears it. The datasheet specifies a 150 ms turn-on time that `begin()` does not
   allow. Not yet fixed.

---

## ▸ Next task — direct-register read path

The one thing standing between here and four acceptance measurements.

Bypass the FIFO entirely: read the six output registers directly, no CTRL9 handshake, no read
mode. Then re-measure the resting gyro noise floor. It is decisive either way:

- **σ falls to ~0.1 dps** → the FIFO read path was the fault. §6.4's mandate for FIFO batching
  should then be revisited — see below.
- **σ stays near 1 dps** → the noise is real and `stillness_gyro_std_rad` needs raising.

While in that file, two small fixes worth folding in: the **150 ms turn-on delay**, and making
the sequence counter count samples *produced* rather than *delivered*, so `dropped: 0` stops
being misleading.

**§6.4's justification for FIFO batching does not survive the datasheet.** It claims batching
cuts bus cost "by an order of magnitude". Batching cuts *transaction overhead*, not *data
volume*, and volume dominates at 12 bytes per sample: 896.8 Hz × 12 B is ~25% of a 400 kHz bus
however it is read. Direct polling costs ~32% and loses nothing. I²C is capped at 400 kHz
(Table 38), so the loss cannot be bought back with bus speed. If the experiment confirms it,
propose the amendment.

---

## Spec amendments already made

All three came from reading the datasheet or measuring the hardware. Each is recorded in the
spec with its reasoning.

| § | Was | Now | Why |
|---|---|---|---|
| 6.4 | gyro ±250 dps | **±256 dps** | ±250 does not exist on this part. Its table is powers of two. Converting at 250 while configured at 256 puts a 2.4% scale error into every integrated angle. |
| 6.4 | 500 Hz stroke, 100 Hz monitor | **896.8 Hz, 112.1 Hz** | Neither exists. ODR steps derive from the gyro's natural frequency. 896.8 chosen over 448.4 because tempo is the binding constraint and its error halved. |
| 1.2.1 | — | **new** | Distance approximation recorded as deferred, not rejected. Impact speed promoted to a first-build metric — it falls out of `v = ω × r` for free. |

Also corrected in `analysis/plumb/sensor.py`: the full-scale divisor is 2¹⁵, not `INT16_MAX`.
Those differ by one count and mean different things. The accelerometer sanity check on real
hardware (9.689 m/s² against 9.81) confirms it.

---

## Product decisions made with Will

- **Report, never judge.** No screen says "good" or "perfect". The number is reported and the
  graphic carries the intuition. A device that grades a stroke asserts a standard, and 1.8°
  open is a miss for a tour player and a fine putt for most people.
- **Swipeable metrics**, one per screen.
- **Everything drawn in the golfer's frame** — looking down at the grip, left is the target,
  bottom is you, the stroke animates right to left. Toe away, heel nearest.
- **Face angle is a putter, not a dial.** Drawings are amplified (rotation 5×, arc 4×) because
  1.8° across a 110 px head moves the toe three pixels. The amplification is constant and
  monotonic; the numbers are always exact.
- **Arc magnitude is off the screen.** The shape is the message.
- Screen studies: https://claude.ai/artifact/Lqtq3YX7w1LEzUQ3kwAeNa

---

## Validation — scope reduced by Will

Will has deprioritised the §10.2 optical motion capture tier and the Sac State Kinesiology
contact. His call, recorded here so nobody re-litigates it.

**Tiers 0–2 of §10.1 remain free and are still worth doing** — a printed protractor plate, a
pendulum, and a phone at 240 fps. Tier 0 especially: it is the same printer as the base, and if
the device cannot hit a known static angle nothing downstream matters.

Note this sits in tension with §1.3 and `CLAUDE.md`, which both say a finished device with
unvalidated numbers fails the goal. Raise it once if it becomes relevant; do not keep raising it.

---

## Blocked on Will

- **Print a base.** The tap test (§5.5) is still the highest-risk unknown in the project and it
  has not started. If the mount resonates below ~200 Hz, §5.5's escalation runs *before* any
  further firmware work.
- **A LiPo with an MX1.25 connector** — §4.4. Most hobby cells ship JST-PH, which will not mate.
  Meter the polarity before first connection.

**Resolved:** the blade putter's grip has an **open butt cap**, so the §5.1 barbed-taper base
works as specified. No step-drilling needed.

---

## Conventions that have earned their keep

- **Write the test first, and make it compare against ground truth, not against the code.**
  Three separate defects this project found were hiding under tests that passed because they
  only checked the code was consistent with itself.
- **Print measured values, do not just assert them.** Four defects were invisible as pass/fail
  and obvious as a column of numbers.
- **When a test fails, find out why before changing it.** Every escalation in this project so
  far — refusing to loosen a tolerance, refusing to commit red — found a real defect.
- **Do not tune a threshold to make a test pass.** Invariant 5.
- Algorithms are proven in Python, then ported to C and re-verified against the same corpus.
  There is no JTAG on this board (§14.3).

## Working agreement

- The spec is the contract. If something in it is wrong, say so and propose an amendment —
  do not quietly work around it. Commit the amendment with the code that depends on it.
- Never claim something is validated, passing or complete without having run it and seen the
  output.
- Will is a CS student who wants to understand the reasoning, not just receive working code.
  Explain the *why*, especially in the sensor fusion work.
