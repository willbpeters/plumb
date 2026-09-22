# Handoff — Plumb

**Date:** 2026-09-21 (third session of the day)
**For:** Claude Code, picking this project up cold
**Supersedes:** the earlier 2026-09-21 handoff, which was written before the
direct-register experiment ran

---

## Read first

1. `CLAUDE.md` — eight hard invariants. They are the decisions that fail silently.
2. `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` — the spec, and the source
   of truth. **It has been amended four times; see "Spec amendments" below.**
3. `docs/bringup-results.md` — everything the real hardware has told us. **Read the last
   section first**; it corrects two numbers in the earlier ones and says so.

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

**The board is in hand and working, and the instrument now measures honestly.** Three of the
five acceptance criteria in the instrument spec are met. The algorithm is proven in simulation.
The blocking defect is gone.

### Done

| | |
|---|---|
| **Synthetic harness** (`analysis/`) | 80 tests. Face angle recovers to **0.0012°** noiseless, 0.0756° at 0.5 dps gyro noise, against a ±1.0° target. Tempo 0.009 against 0.05. |
| **IMU streaming instrument** | `firmware/bringup-arduino/imu_stream`. Two read paths, selectable at runtime; direct registers is the default. |
| **§9.2 zero dropped samples** | **54720 samples over 60 s, zero lost, zero duplicated, overflow flag clear.** On the direct path. |
| **§9.3 measured ODR** | **906.86 Hz** at stroke rate, from the sensor's own counter. 1.12% above the 896.8 nominal. Maximum rate still unmeasured. |
| **§9.4 resting gyro noise** | **Sensor floor 0.22–0.24 dps**, stable across sessions, against a 0.8 dps stillness threshold. Whole-capture σ runs 0.28–0.56 depending on what the room is doing — see open defect 4. |
| **Host tooling** | `board.py` (one copy of the connect sequence and its hazard), `capture.py`, `rest_noise.py`, `axis_check.py`. 110 tests. |
| **Screen design** | Five screens designed and reviewed. Decisions recorded below. |

### What the direct-register experiment settled

The FIFO read path was the fault, decisively. Same board, same surface, 60 s each, back to back:

| | Direct registers | FIFO |
|---|---|---|
| Sensor ODR | **906.86 Hz** | frozen counter — unmeasurable |
| Delivered | 906.86 Hz | 707.80 Hz |
| Lost | **0%** | **21.9%** |
| Overflow flag | never | 665 of 665 batches |
| Resting σ, worst axis | **0.2765 dps** | 0.3405 dps, varying with position in the batch |

Parent spec §6.4's "FIFO batching is mandatory" is **amended** to direct polling, with those
numbers in the spec. The handoff before this one authorised proposing that amendment if the
experiment confirmed it. It did.

### Open defects

1. **The FIFO path still loses 21.9%** and still delivers position-dependent sample quality
   (1.4× σ spread within a batch — real, 30σ, but smaller than the 3× recorded earlier; the
   earlier figure was taken with a capture pipeline that could splice in stale frames). No
   longer on the path to the first build, but it is **the only way to sample above ~1.1 kHz**,
   so §9.5's tap test needs it fixed or needs a different plan.
2. **Path arc magnitude is ~61% of truth** — a known gap, pinned by a test, documented in
   `test_path_arc_magnitude_is_known_to_fall_short_of_truth`. Direction classification, which
   is what the screen shows, is unaffected.
3. **Resting noise is 3–4× datasheet-typical.** 0.22–0.28 dps measured against 0.074 predicted
   from 15 mdps/√Hz over the LPF's ~24 Hz bandwidth. Unexplained. Not worth chasing while
   there is 3× margin against the threshold that matters.
4. **Environmental vibration, not sensor noise, is what will defeat stillness detection.** Two
   captures an hour apart on an untouched board: the quietest half-second windows agreed to
   within 10% (0.22 against 0.24 dps), while the worst window went from 0.41 to **1.217 dps —
   over the 0.8 threshold**. Mostly on one axis, so it is mechanical coupling; a cable would do
   it. This is not a reason to raise the threshold (invariant 5), it is a reason the Phase 2
   corpus has to be recorded somewhere representative or it answers the wrong question.

**Resolved this session:** the FIFO sample loss (routed around), the corrupted FIFO reads
(quantified, and no longer in the signal path), the intermittent init (soft reset + the
datasheet's 15 ms, not the 150 ms it was first read as), and a capture defect that could
silently splice stale frames into a measurement.

---

## ▸ Next task — pick one of three

Nothing is blocked on code any more. In order of what unblocks the most:

**1. §9.1 axes and signs — the tool is built and waiting for a hand.**

```
cd analysis && uv run python tools/axis_check.py --port COM4
```

It prompts, you act, it decides. Rest the board on a face; the accelerometer says which axis
points up, absolutely, because a sensor at rest reads +1 g on whatever points up. Then turn the
board a quarter turn anticlockwise as seen from above; by the right-hand rule the matching gyro
channel must read positive about that up direction. Three faces covers all three axes. It
removes the gyro bias before integrating, derives its motion threshold from the noise it just
measured, and refuses to deliver a verdict on a turn that was really a knock.

What it settles is the sensor triad: which gyro channel is which board axis, and whether the
triad is right-handed. A left-handed triad is the kind of defect that reports a stroke that
opened as one that closed. **The other half of §9.1 — how that triad sits relative to the
PUTTER (Z along the shaft, X the face normal) — is a property of the mount and cannot be
checked until a base is printed.**

Everything downstream depends on this and it gates the port of the fusion code.

**2. The 906.86 Hz decision — Will's call, and it needs making before the port.** A 1.12% scale
error goes into every integrated angle and no filtering removes it. `SAMPLE_RATE_HZ` is
deliberately left at nominal 896.8, because 906.86 is *this board's* oscillator and baking one
unit's calibration into a shared constant trades a known error for a hidden one. The real
options: per-unit calibration, or firmware that measures its own rate at startup — which it can
now do in about ten lines, since the counter exists and works.

**3. Phase 2, the logged corpus.** The instrument is trustworthy enough to log strokes with.
Capture motion windows, not strokes — see §2.1 of the instrument spec for why that ordering
matters, and invariant 5 for why thresholds cannot come first.

The tap test (§5.5) is still the highest-risk unknown in the project, and it is still blocked on
a printed base *and* on sampling above 1 kHz. The untried option for the second: 1793.6 Hz with
direct polling and the timestamp dropped from the read, which buys the headroom. Nyquist 896 Hz
is enough to see the resonance §5.5 looks for.

---

## Spec amendments already made

All four came from reading the datasheet or measuring the hardware. Each is recorded in the
spec with its reasoning.

| § | Was | Now | Why |
|---|---|---|---|
| 6.4 | gyro ±250 dps | **±256 dps** | ±250 does not exist on this part. Its table is powers of two. Converting at 250 while configured at 256 puts a 2.4% scale error into every integrated angle. |
| 6.4 | 500 Hz stroke, 100 Hz monitor | **896.8 Hz, 112.1 Hz** | Neither exists. ODR steps derive from the gyro's natural frequency. 896.8 chosen over 448.4 because tempo is the binding constraint and its error halved. |
| 6.4 | "FIFO batching is mandatory" | **direct register polling** | Measured: the FIFO loses 21.9% of samples and cannot count what it loses; direct polling loses none. The original bus-cost argument confused transaction overhead with data volume. |
| 1.2.1 | — | **new** | Distance approximation recorded as deferred, not rejected. Impact speed promoted to a first-build metric — it falls out of `v = ω × r` for free. |

Also corrected in `analysis/plumb/sensor.py`: the full-scale divisor is 2¹⁵, not `INT16_MAX`.
Those differ by one count and mean different things.

---

## Things the datasheet does not say, that the board does

Three findings from this session. All measured, all in `docs/bringup-results.md` with the
evidence.

- **The TIMESTAMP counter is frozen while the FIFO is enabled.** It counts writes to the output
  registers, so in FIFO mode it does not move: 84 across eight polls, against 19 counts per
  20 ms in bypass. The consequence is architectural — the FIFO path cannot count its own losses,
  and any future FIFO fix has to be validated from outside the part.
- **CTRL1.ADDR_AI defaults to 0**, so burst reads do not advance the register address. Correct
  for FIFO_DATA by accident; silently wrong for the output registers, where it would have
  returned twelve copies of AX_L and a standard deviation that meant nothing.
- **Turn-on time is two numbers.** System Turn On Time is 15 ms (initialisation, during which
  the datasheet says not to write at all); Gyro Turn On Time is 150 ms + 3/ODR (before the
  output means anything). The earlier handoff conflated them.

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
- **The 906.86 Hz decision.** See next task 2.

**Resolved:** the blade putter's grip has an **open butt cap**, so the §5.1 barbed-taper base
works as specified. No step-drilling needed.

---

## Working with the board

It answers on **COM4** (CH343 USB-serial bridge, VID 0x1A86, PID 0x55D3). From the repo root:

```
arduino-cli compile --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,CDCOnBoot=default" firmware/bringup-arduino/imu_stream
arduino-cli upload -p COM4 --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,CDCOnBoot=default" firmware/bringup-arduino/imu_stream
```

**Opening the serial port resets the board only about half the time.** Measured, not assumed.
`capture.py` handles it by asking the board what it is doing and settling it before configuring
anything; any new host tool has to do the same, or it will silently capture stale frames from
the previous run. That failure does not look like a failure.

Serial commands are single characters: `c`/`b` format, `1`/`9` rate, `f`/`d` read path,
`s` start-stop (a toggle), `?` status.

---

## Conventions that have earned their keep

- **Write the test first, and make it compare against ground truth, not against the code.**
  Three separate defects this project found were hiding under tests that passed because they
  only checked the code was consistent with itself.
- **Print measured values, do not just assert them.** Five defects now were invisible as
  pass/fail and obvious as a column of numbers. `rest_noise.py` exists for exactly this.
- **Carry a robust estimator alongside the raw one.** Raw σ against 1.4826×MAD is what
  separated a real noise floor (ratio 1.05) from a contaminated one (ratio 1.4) this session.
- **When a test fails, find out why before changing it.** Every escalation in this project so
  far — refusing to loosen a tolerance, refusing to commit red — found a real defect.
- **When an instrument reports a zero, ask whether it could report anything else.** "dropped: 0"
  sat next to 20% sample loss for a whole session because the counter could not see it.
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
