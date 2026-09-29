# Handoff — Plumb

**Date:** 2026-09-27 (firmware skeleton added; the rest as of 2026-09-25)
**For:** Claude Code, picking this project up cold
**Supersedes:** the 2026-09-21 handoff. What changed since is under "This session
(2026-09-25)" below; everything else still stands.

---

## Read first

1. `CLAUDE.md` — eight hard invariants. They are the decisions that fail silently.
2. `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` — the spec, and the source
   of truth. **It carries nine marked amendments plus one added section (§1.2.1); see "Spec
   amendments" below.**
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

**The board is in hand and working, and the instrument now measures honestly.** Four of the
five acceptance criteria in the instrument spec are met. The algorithm is proven in simulation.
The blocking defect is gone.

### Done

| | |
|---|---|
| **Synthetic harness** (`analysis/`) | 151 tests collected on 2026-09-25, all passing (`uv run pytest --co` for today's count). Face angle recovers to **0.0012°** noiseless, 0.0756° at 0.5 dps gyro noise, against a ±1.0° target. Tempo 0.009 against 0.05. **These agree with the generator, not with real strokes — see the note under this table.** |
| **IMU streaming instrument** | `firmware/bringup-arduino/imu_stream`. Two read paths, selectable at runtime; direct registers is the default. |
| **§9.1 axes and signs** | **PASS.** Gyro channels map 1:1 to board axes and the triad is right-handed — no remapping needed at the driver boundary. The putter-relative half needs a printed base. |
| **§9.2 zero dropped samples** | **54720 samples over 60 s, zero lost, zero duplicated, overflow flag clear.** On the direct path. |
| **§9.3 measured ODR** | **906.86 Hz** at stroke rate, from the sensor's own counter. 1.12% above the 896.8 nominal. Maximum rate still unmeasured. |
| **§9.4 resting gyro noise** | **Sensor floor 0.22–0.24 dps**, stable across sessions, against a 0.8 dps stillness threshold. Whole-capture σ runs 0.28–0.56 depending on what the room is doing — see open defect 4. |
| **Host tooling** | `board.py` (one copy of the connect sequence and its hazard), `capture.py`, `rest_noise.py`, `axis_check.py`, `accel_cal.py` (written, not yet run on the board). |
| **Pivot offset estimation** | `plumb/pivot.py` — velocity form, noise-corrected. Closes the 61% path shortfall; arc now 0.996–1.002 of truth noiselessly for every putter type, and the session mean under 0.28 dps no longer depends on putter type (straight 1.006, arced 1.010 at lie 5). |
| **C port, started** | `firmware/components/plumb/` — `quat.c` ported and **bit-identical to NumPy** on every operation, and across a whole replayed stroke, proven by a differential harness that runs the same cases through both. Pure C99, builds for host and device from one source, FMA contraction off on both. |
| **Single precision, whole stroke** | **4.0×10⁻⁵° of face angle at worst** over ~1,850 accumulated steps, against a 1.0° target. A 30 s address hold does not grow it. Safe for the attitude integrator; see `real.h`. |
| **UI screens, on the host** | `firmware/components/plumb_ui/` — face angle, tempo, path, impact speed and idle, pure LVGL 9.6 (submodule), rendered headlessly and tested by measuring pixels against the inputs: 5× face rotation, tempo bar lengths, path direction, ring sweep, the round aperture, zero missing glyphs. `cd analysis; uv run python -m tools.uisnap` writes PNGs. |
| **Display and touch, on the board** | `firmware/bringup-arduino/display` — the same screens on the GC9A01 panel, swipe with the touch controller, verified by Will at the board 2026-09-26. `MADCTL` 0x48, inversion on, SPI 80 MHz, full-screen refresh 15.8–20.7 ms per screen. Build and flash with `uv run python -m tools.board_ui flash --port COM4`. Pins and measurements in `docs/bringup-results.md`. |
| **Screen design** | Five screens designed and reviewed. Decisions recorded below. |
| **ESP-IDF firmware skeleton** (2026-09-27) | `firmware/` on ESP-IDF v5.5.5, now what the board runs. IMU read on core 0, woken by its own DRDY line (INT2 → GPIO3) in SyncSample mode; UI on core 1; the invariant-8 gate as an atomic flag (`a`/`o` on the console for now). **0 of 54,653 samples lost in 60 s at rest**, through `capture.py` unchanged. **Rendering pushes read latency past the 1103 µs period, and with streaming lost 0.72%; with the gate armed, 0 lost and latency max 820 µs.** No radio code linked, checked on every build. The IMU comes up on 60 of 60 resets. Screens and swipes checked by Will. Build: `firmware/idf.ps1 build`, flash: `firmware/idf.ps1 -p COM4 flash`. Everything in `docs/bringup-results.md`, last section. |

**What the synthetic numbers do and do not show.** The 0.0012° and 0.0756° face-angle figures
demonstrate that the pipeline agrees with the generator's model of a stroke, not that it is
accurate on real strokes. The generator and the pipeline share assumptions that a real stroke
need not honour: impact always falls at the address swing angle (`theta = 0` at impact by
construction in `trajectory.generate`); the swing is always about body Y; the impact impulse is
a symmetric Hanning pulse (`sensor.simulate`); and the pivot and sweet spot both lie on the
shaft axis (`r = (0, 0, -L)`), where spec §8.4 calls for a full lever-arm vector. An error that
lives in any of those assumptions cannot show up in these numbers. Accuracy on real strokes is
what the spec's §10 validation study exists to measure.

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
2. **Path arc magnitude: the 61% shortfall is fixed, and it uncovered a second defect.**
   The pipeline computed face velocity as `omega x r`, assuming the sensor does not translate.
   It does — the putter swings about the hands, so the face travels on `r + d`, 1.4 m against
   the 0.85 m assumed, and 0.85/1.4 = 0.607 was exactly the measured shortfall. `plumb/pivot.py`
   now estimates `d` per stroke by least squares: the relation is LINEAR in `d`, so a stroke is
   one over-determined system rather than the thousand independent guesses the abandoned first
   attempt was making. **Noiseless arc error: 39% → 1.3–5.1%, inside the §3 target.**

   One stroke is not enough at 0.28 dps (17% on the pivot, and 8% under to 84% over on the arc),
   so `PivotCalibration` sums the normal equations across strokes — inverse-variance weighting
   for free — and converges to 2.4% by the fifth. That is §8.4's per-golfer calibration,
   arrived at from measurement.

   **Still not meeting §3 under noise**, and the reason is now a different defect — see 4.
3. **Resting noise is 3–4× datasheet-typical.** 0.22–0.28 dps measured against 0.074 predicted
   from 15 mdps/√Hz over the LPF's ~24 Hz bandwidth. Unexplained. Not worth chasing while
   there is 3× margin against the threshold that matters.
4. **Arc under noise — fixed on the harness, both halves.** Two causes, found by oracles
   (replace one input of the fit with ground truth at a time and see what moves):

   - **Track wander while the face is still**, mainly through the chord's end point. Fixed by
     measuring the arc over the motion only.
   - **The pivot fit's weakest direction.** Not the tilt leak the previous version of this
     handoff guessed — feeding the fit the *true* acceleration changed nothing (1.137), so the
     six-unknown fit suggested here would not have helped. It was **errors-in-variables**:
     the true angular rate alone took the arced putter from 1.142 to 1.049. On the arced putter
     the rotation axis tilts ~19° off body Y, the weakest direction carries a real 0.18 m of
     `d`, and noise in the design matrix shrank it by half; the remainder sat off the shaft
     axis, where face rotation turned it into sideways path. Fixed by fitting in velocity
     (no derivative, no filter), treating the start velocity as unknown, and subtracting the
     gyro noise's measured contribution. The reasoning and every intermediate number are in
     the `pivot.py` docstring.

   Whole pipeline, calibrated pivot, 0.28 dps + 1.5 dps bias, strokes 5–20, arc / truth:

   | putter, lie | start of 2026-09-25 | motion only | + velocity fit |
   |---|---|---|---|
   | straight, 5 | 1.041 [0.788, 1.353] | 1.018 [0.856, 1.203] | 1.006 [0.845, 1.190] |
   | straight, 20 | 1.017 [0.954, 1.088] | 1.010 [0.974, 1.051] | 0.999 [0.962, 1.040] |
   | arced, 5 | 1.175 [0.927, 1.495] | 1.155 [1.009, 1.348] | 1.010 [0.850, 1.188] |
   | arced, 20 | 1.070 [1.010, 1.144] | 1.064 [1.028, 1.110] | 0.999 [0.963, 1.040] |

   What is left at lie 5 is **spread, not bias**: a 6 mm arc against a ~0.5 mm random walk in
   the integrated track, per stroke. That is gyro noise through the attitude, and no pivot fit
   removes it. Session means meet the 10% target; single small-arc strokes do not always.

   **Two assumptions to check on the Phase 2 corpus.** The noise correction's *uncertainty*
   (which decides whether a direction was observed) is computed for white noise; the board's
   LPF makes real noise coloured, so the significance test will be optimistic there. And the
   correction uses the stillness window's noise as the stroke's noise; hand tremor at address
   or vibration during the stroke would break that.

5. **Environmental vibration, not sensor noise, is what will defeat stillness detection.** Two
   captures an hour apart on an untouched board: the quietest half-second windows agreed to
   within 10% (0.22 against 0.24 dps), while the worst window went from 0.41 to **1.217 dps —
   over the 0.8 threshold**. Mostly on one axis, so it is mechanical coupling; a cable would do
   it. This is not a reason to raise the threshold (invariant 5), it is a reason the Phase 2
   corpus has to be recorded somewhere representative or it answers the wrong question.

6. **Accelerometer bias inflates the arc — mechanism found, fix built, not yet run on hardware.**
   Oracles on each use of `g0` separately (0.2 m/s² bias, arced, lie 5; no-bias 1.031, biased
   1.406):

   - **Ground plane** (true gravity there alone → 1.203). `g0` is gravity *plus* bias, so the
     plane the path is projected onto tilts by bias/g, and the head's ~3 cm of vertical travel
     leaks into lateral. Per axis this is **body Y — the swing axis, which the stroke cannot
     observe** (1.296 → 1.066 with the true plane).
   - **Pivot fit.** `g0` cancels the bias only at the address orientation, leaving
     `(I − Rᵀ) b`. Body X, +8%.
   - **Face angle** moves ~0.004°: immune.

   Linear and symmetric at small bias: **1.21% of arc per 0.01 m/s² on Y at lie 5**, 0.42% on X,
   about a quarter of that at lie 20. This board read 9.689 m/s² against 9.81 at rest in
   bring-up, so its error is in the range that matters. **Requirement: residual offset under
   ~0.02 m/s² (2 mg)** to keep this under 2.5% of arc at lie 5.

   Not fixable per stroke; the fix is spec §8.1's device calibration, which was specified and
   never implemented. `plumb/calibration.py` solves offset and gain per axis from resting poses
   and the pipeline applies it; a simulated tumble of a biased board restores the arc exactly
   (1.347 → 1.055, the unbiased value). `tools/accel_cal.py` runs it on the board. **Open:** it
   has not been run, cross-axis misalignment and the offset's temperature drift are not
   modelled, and the firmware will need the result in NVS and in each record's header (§11).

**Resolved on 2026-09-21:** the FIFO sample loss (routed around), the corrupted FIFO reads
(quantified, and no longer in the signal path), the intermittent init (soft reset + the
datasheet's 15 ms, not the 150 ms it was first read as), and a capture defect that could
silently splice stale frames into a measurement.

### This session (2026-09-25)

Commits from `cbe265b` on. Every code change came with a test that
compares against ground truth.

- **The accelerometer correction had the wrong sign** — positive feedback. `predicted × measured`
  turned the attitude *away* from gravity; a 2° tilt went to 87° in 2 s at a gain of 2.0. The
  stock gain's ~50 s time constant hid it (2.000° → 2.082°). Now `measured × predicted`.
- **Face rotation could time the transition** (invariant 1). The transition was the sign flip
  of the largest body axis at backswing entry, including Z, the face-rotation axis, and entry
  is at ~12.5 dps of swing, so an early-opening face could win: a 100 ms shift, all of it into
  tempo. Now the swing perpendicular to the shaft, projected on the backswing direction. The
  "25-sample" dominant-axis average also only ever saw one sample.
- **FMA contraction is off** on host and device, without which the bit-for-bit claim says
  nothing about the ESP32-S3.
- **Arc is measured over the motion only** (open defect 4, first half).
- **The pivot is fitted in velocity, corrected for gyro noise** (open defect 4, second half).
  The arced putter's arc went from 1.142 to 1.010 of truth at lie 5, level with the straight
  putter. Spec §7.4 amended; it still described `v = ω × r`.
- **Single precision measured across a whole stroke** (next task 1, its open question).
- **Corrected, not changed:** the pivot docstring claimed its filtering left the equation
  exact; it does not, and filtering the matrix instead was measured and is not better at the
  board's noise floor (table in `pivot.py`).

---

## ▸ Next task — pick one

Nothing is blocked on code any more. In order of what unblocks the most:

**0. What the firmware skeleton leaves (2026-09-27).** `firmware/` is the product build now.
Still to do on it, in order:

- **Rendering's delay: found and fixed (2026-09-27).** Rendering evicted the I²C driver from the
  shared cache; the driver now runs from IRAM (`board/linker.lf`). Streaming while animating
  went from 0.72% lost to 0 of 55,064. The remaining tail is touch polling on the shared bus,
  which the gate already suspends during strokes. The UART interrupt was never the cause.
  `docs/bringup-results.md`, last section.
- **Samples lost in the first second after boot** (10 of 12 boots, usually 1, once 24). The
  suspect is the touch controller's init holding the bus; untested. It matters once a wake
  (§9) sits right before a stroke.
- **Before §11 logging:** flash writes disable the cache for both cores. The I²C driver is now in
  IRAM, but the acquisition task, `qmi8658.c` and the GPIO ISR dispatch are not. Anything that
  writes flash while the IMU runs needs those in IRAM too, measured with `j`.
- Fonts to PSRAM (invariant 7 permits it; they are in flash-mapped rodata now), the final
  rotation once the base fixes how the board sits, and the 120 MHz flash question (spec §6.3
  asks for it; not attempted, reasons in the skeleton spec).

**1. The C port is complete; wire it into the firmware.** `quat.c`, `pivot.c` and
`pipeline.c` all done, all verified against the Python and against ground truth (branch
`c-port-pivot`). What is left is the firmware around it — see "Next, in order" below.

> **`pipeline.c` done (2026-09-28).** `analysis/tests/test_c_pipeline.py`, 10 tests. The C is
> fed the simulator's int16 counts and compared at every stage over 14 strokes (three putters,
> lie 5/20, two face angles, clean and noisy, two tempos), plus a calibrated session, an
> accelerometer calibration, and the 906.86 Hz rate:
>
> | | double vs NumPy | float vs NumPy |
> |---|---|---|
> | bias, g0, every stroke boundary, attitude at impact | **0 exactly** | no boundary moved |
> | face angle | **0 exactly** | 3.3×10⁻⁵° |
> | tempo ratio | **0 exactly** | 8.7×10⁻⁸ |
> | arc, travel | 3×10⁻¹⁵ m | 0.035%, 0.006% |
> | path direction | identical | identical |
>
> This answers what `real.h` left open — rates formed in float and everything the pipeline
> adds on top of the integrator. **Single precision costs nothing measurable**; the
> precision call is still yours, but the evidence is now whole-pipeline.
>
> The arc's 3×10⁻¹⁵ is the only place the C departs from the Python on purpose: it sums
> the track from the window start, because it keeps a ring rather than the whole list.
>
> Also added to the Python first, tests against ground truth: `Pipeline(sample_rate_hz=)`
> (your 2026-09-27 decision; a wrong rate scales face angle by exactly the rate ratio and
> leaves tempo alone), and `StrokeResult.backswing_s / downswing_s / path_travel_m`, which the
> UI already draws from.
>
> **Track ring size — DECIDED by Will 2026-09-28: keep 3 s.** `PL_PIPELINE_TRACK_MAX` = 2720 samples (3.0 s at
> 906.86 Hz). In float **one `pl_pipeline` is 156,640 bytes, 130,560 of them the ring** —
> internal SRAM, since invariant 7 keeps PSRAM for fonts and images. Sized from the harness:
> the slowest strokes generated (1.0 s backswing, tempo 1.5) need 2318 samples from onset to
> DONE. A stroke that outruns the ring reports **path unavailable**; face angle and tempo
> never touch it (tested). Reductions if SRAM is tight: store the ground-plane projection only
> (8 floats a sample, not 12 — a Python change first), or assume the pivot is on the shaft
> axis (collapses the matrix to a vector, at the cost of an assumption). Real stroke lengths
> come from the Phase 2 corpus; the synthetic generator's are a guess.
>
> **Other behaviour the Python did not need:** an impact spike longer than
> `PL_PIPELINE_IMPACT_MAX` (64 samples, ~70 ms — a putt's is ~4) reports face angle
> unavailable but keeps tempo; a stillness window that does not fit its ring is refused at
> init rather than shortened. No threshold has a value anywhere in the C (invariant 5); they
> arrive in `pl_pipeline_config`.
>
> **Impact speed — done, Python then C (2026-09-28).** |ω × (d + r)|: the face as a point of
> the rigid body rotating about the fitted pivot. Taken at the last downswing sample before
> the impact spike, because from the spike on the gyro reads the collision too. **Unavailable
> without a pivot solution** — the lever arm alone reads 39% low (0.85 against ~1.4 m), and
> that is not a measurement. Against the generator's true face speed:
>
> | | |
> |---|---|
> | Noiseless, 9 strokes, 0.49–3.29 m/s | ≤ 0.16% |
> | Sample-before-impact against the true impact instant | ≤ 0.02% (the definition's cost) |
> | 0.28 dps + 1.5 dps bias, 10 strokes | mean +0.04%, worst 0.70% |
> | Across putter types | < 0.2% (invariant 1) |
> | C against the Python, double / float | 2.7×10⁻¹⁵ m/s / 5.9×10⁻⁶ relative |
>
> **Your call: the spec has no accuracy target for impact speed.** §1.2.1 makes it a
> first-build metric, but §3's table has no row for it, so nothing says what the §10 study
> should hold it to. The harness numbers above are the evidence to set one from; the real
> uncertainty will be larger (real strokes, a real pivot, a real lever-arm measurement —
> the lever arm is a per-putter input, and an error in it is a proportional error in speed).
>
> **Startup rate measurement — done, on the board (2026-09-28).** `acq/rate.c`: least squares
> of DRDY edge time (`esp_timer`) against the sensor's own sample index over the first 1,024
> samples after boot; records where the task fell behind are skipped, mispaired edges
> rejected at 5 robust sigmas. 8 host tests against generated ground truth, including that the
> reported error is honest (z SD 1.00 over 60 runs). On the board: **906.9314 Hz, 30
> back-to-back measurements within 1.0 ppm** (SD 0.28 ppm — 8× the fit's own error, because
> the oscillator wanders smoothly, not because the fit is wrong). **Against the PC's clock the
> same unit reads 13.4 ppm higher** — the two crystals' difference; it bounds the systematic
> term, it cannot say which clock is right. 906.86 (09-21) → 906.93 (09-27) → 906.925 cold /
> 906.931 warm (09-28): the case for measuring rather than storing. Printed at boot and by
> `?`; `m` re-measures; `stream_sample_rate()` returns it. Spec §6.4 amended, §7.2 points at
> it. `uv run python -m tools.rate_check --port COM4` repeats the whole check.
>
> **Next, in order:**
> 1. **Wire `pl_pipeline` into `main`**, with `sample_rate_hz` from `stream_sample_rate()`: acquisition ring → `pl_pipeline_step` on core 0 →
>    result to the UI on core 1, gate armed from ADDRESS to DONE (invariant 8). Add `plumb` to
>    `main`'s REQUIRES — until then **`idf.ps1 build` compiles none of the port**; it has been
>    cross-compiled by hand with the ESP32-S3 GCC 14.2 at `-Werror`, both precisions.
> 2. Thresholds on the device are the harness's placeholders until Phase 2 (invariant 5) —
>    they must be passed in from one clearly-labelled place, not scattered.

> **`pivot.c` done (2026-09-28).** `analysis/tests/test_c_pivot.py`,
> 9 tests, both precisions, checked to have run rather than skipped:
>
> | | double | float |
> |---|---|---|
> | normal equations vs NumPy | 3.4×10⁻¹⁶ relative | — |
> | offset vs NumPy, per stroke | 2.4×10⁻¹³ m | ≤ 0.53 mm |
> | offset vs NumPy, 10-stroke session | < 10⁻⁹ m | 0.11 mm |
> | 10 arced strokes at 0.28 dps, vs truth | 4.50 mm | — |
>
> Plus ground truth on the rank-deficient single-axis swing, the straight putter's
> noise-only direction, and the no-estimate cases. The solve uses Jacobi rotations in place
> of LAPACK's `eigh`, so it is compared on offset, rank and residual rather than bit for bit.
>> **Welford was not adopted.** The old port note asked for it; measured in NumPy float32
> first, running sums cost ≤ 0.17 mm of offset and Welford ≤ 0.10 mm. The port keeps the
> Python's arithmetic. `pivot.py`'s port note now says so.
>
> **Found by the port, recorded, not fixed — Will's call if it ever matters.** The design
> energy is Σ(|ω|²I − ωωᵀ), so for a single-axis swing the two directions perpendicular to
> the axis carry *exactly* equal energy (204.9053 against 204.9053). The eigenvectors in that
> plane are an arbitrary basis and the per-direction significance test depends on the basis,
> so the reported **rank** there is not a stable quantity: float dropped a 0.55 mm component
> that double kept, once in twelve strokes. The offset moved 0.53 mm. Nothing consumes rank
> except reporting. A basis-free test would judge the degenerate plane as a whole.
>

```
cd analysis; uv run pytest tests/test_c_port.py -q -s
```

builds the harness and prints the worst difference per operation. It came back **0.000e+00 on
every quaternion operation in double precision** — the C reproduces NumPy bit for bit, because
the translation keeps the arithmetic in the Python's order. Floating-point addition is not
associative, so tidying an expression while translating changes the last bits and turns a clean
diff into an investigation. Keep doing it that way.

Two things to know before continuing:

- **The precision decision now has its whole-stroke number, and it favours single.** See
  `real.h`. About one ulp per operation, and **at most 4.0×10⁻⁵° of face angle across a whole
  replayed stroke** — ~25,000× inside the 1.0° target — with no growth over a 30 s address.
  Doubles are software-emulated on the ESP32-S3 at 896.8 Hz, so single is the likely choice;
  it is still Will's call. **Now measured on the whole pipeline too** (2026-09-28, above):
  3.3×10⁻⁵° of face angle, no stroke boundary moved.
- **The build lives in `analysis/tools/cbuild.py`, not in a shell script.** It discovers the
  toolchain itself — cc/gcc/clang, else MSVC, which is driven directly because `vcvars64.bat`
  hangs in Git Bash here. It was a shell script for about an hour, and in that hour running
  the suite from PowerShell skipped all ten cases silently and read as a pass, because `sh`
  was not on PATH. Verify the port test actually RAN, not merely that it was green.

**2. The 906.86 Hz decision — DECIDED 2026-09-27: measure at startup** (next task 1, step 1).
The reasoning that led there, kept for the record: a 1.12% scale
error goes into every integrated angle and no filtering removes it. `SAMPLE_RATE_HZ` is
deliberately left at nominal 896.8, because 906.86 is *this board's* oscillator and baking one
unit's calibration into a shared constant trades a known error for a hidden one. The real
options: per-unit calibration, or firmware that measures its own rate at startup — which it can
now do in about ten lines, since the counter exists and works.

**Also ready to run:** the accelerometer tumble (open defect 6), which needs a hand and about
five minutes — see "Blocked on Will".

**Port note for `pivot.c`:** the centring is done from running sums, which cancels. Fine in
double; in single precision the weakest eigenvalue (~0.07 against sums of ~300) keeps about four
significant digits. Use running means (Welford), and measure it with the harness.

**3. Phase 2, the logged corpus.** The instrument is trustworthy enough to log strokes with.
Capture motion windows, not strokes — see §2.1 of the instrument spec for why that ordering
matters, and invariant 5 for why thresholds cannot come first.

The tap test (§5.5) is still the highest-risk unknown in the project, and it is still blocked on
a printed base *and* on sampling above 1 kHz. The untried option for the second: 1793.6 Hz with
direct polling and the timestamp dropped from the read, which buys the headroom. Nyquist 896 Hz
is enough to see the resonance §5.5 looks for.

---

## Spec amendments already made

Nine amendments to existing text, each marked *Amended* in the spec with its reasoning, plus
one new section. The four to §6.4 came from reading the datasheet or measuring the hardware;
the §11 one is a consequence of the §6.4 rate change that was missed at the time; the §7.4 one
records what the path code has actually done since the pivot estimate went in; the §4.4 one
records the charger the schematic actually shows; the §6.2 one records data-ready pacing.

| § | Was | Now | Why |
|---|---|---|---|
| 6.4 | gyro ±250 dps | **±256 dps** | ±250 does not exist on this part. Its table is powers of two. Converting at 250 while configured at 256 puts a 2.4% scale error into every integrated angle. |
| 6.4 | 500 Hz stroke, 100 Hz monitor | **896.8 Hz, 112.1 Hz** | Neither exists. ODR steps derive from the gyro's natural frequency. 896.8 chosen over 448.4 because tempo is the binding constraint and its error halved. |
| 6.4 | "FIFO batching is mandatory" | **direct register polling** | Measured: the FIFO loses 21.9% of samples and cannot count what it loses; direct polling loses none. The original bus-cost argument confused transaction overhead with data volume. |
| 11 | ≈9 KB per stroke, ~1,400 strokes | **≈16 KB, ~800 strokes** | Storage estimate was still computed at 500 Hz. Recomputed at 896.8 Hz: 1,345 samples × 12 B against the ~13 MB partition, before headers. Amended 2026-09-22. |
| 7.4 | `v_face = ω × r` | **`r + d`, `d` fitted per stroke and per golfer** | The sensor translates; the stroke rotates about the hands. The code has estimated `d` since `9516b6f` without a spec amendment; recorded 2026-09-25 with the velocity-form fit that replaced the acceleration form. |
| 4.4 | ETA6096, ≤ 800 mA | **ETA6098, 1 A** (R15 = 160 kΩ) | Read from the Rev3 schematic at display bring-up, as §4.4 asks. 2.5C on the planned 400 mAh cell. Not yet bench-measured. Amended 2026-09-26. |
| 6.2 | "hardware-timer driven" | **paced by the IMU's DRDY line** | A timer on the ESP32's clock drifts against the IMU's 906.86 Hz and would duplicate or miss ~1 sample in 100. Amended 2026-09-27 with the skeleton's loss and jitter measurements. |
| 6.4 | 896.8 Hz, used as the rate | **the rate measured at startup** | This unit runs 1.12% fast and moves ~75 ppm between days; a constant goes stale. Measured to ~15 ppm (crystal-bound). Will's decision 2026-09-27, amended 2026-09-28 with the board measurements. |
| 7.1 | forward only | **ABANDONED on rest or timeout before impact; DONE without path on timeout after** | A practice stroke waited in DOWNSWING forever with rendering suspended. Rest = the address test failing then passing again. Will's decision, amended 2026-09-28. |
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
- **INT2 needs `CTRL1` bit 4, which rev A marks reserved** (2026-09-27). Without it no DRDY edge
  ever arrives.
- **The CTRL9 handshake takes 3253 µs**, measured on every boot; imu_stream's 50-read budget
  was ~3.2 ms and failed on 7 to 11 boots in 40 on the new firmware. Now bounded by time.
- **A reset mid-read leaves the IMU holding SDA.** ESP-IDF's bus clear does not free it; nine
  clocks and a STOP, before the driver takes the pins, does (`board/src/i2c_bus.c`).
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

- **Run the accelerometer tumble.** `cd analysis; uv run python -m tools.accel_cal --port COM4
  --gravity 9.800` — six faces, then two poses of any kind, board still each time. It prints
  offset and gain and saves the raw pose means under `data/calibration/`. This says whether
  open defect 6 is a 2% problem or a 25% one on this unit. (9.800 is for Sacramento; pass local
  g if elsewhere.)

- **Print a base.** The tap test (§5.5) is still the highest-risk unknown in the project and it
  has not started. If the mount resonates below ~200 Hz, §5.5's escalation runs *before* any
  further firmware work.
- **A LiPo with an MX1.25 connector** — §4.4. Most hobby cells ship JST-PH, which will not mate.
  Meter the polarity before first connection. **And not 400 mAh:** the schematic shows the
  charger set to **1 A** (ETA6098, R15 = 160 kΩ), which is 2.5C on the spec's 400 mAh cell.
  Fit at least 1000 mAh, or change R15; confirm the board is Rev3 from its silkscreen first.
  Spec §4.4 amended 2026-09-26; details in `docs/bringup-results.md`.
- ~~**The 906.86 Hz decision.**~~ **Decided 2026-09-27:** measure the rate at startup. See
  next task 1.

**Resolved:** the blade putter's grip has an **open butt cap**, so the §5.1 barbed-taper base
works as specified. No step-drilling needed.

---

## Working with the board

It answers on **COM4** (CH343 USB-serial bridge, VID 0x1A86, PID 0x55D3).

**It runs the product firmware now** (since 2026-09-27). From `firmware/` in PowerShell:
`./idf.ps1 build`, `./idf.ps1 -p COM4 flash`. Console at 921600: `s` stream, `b`/`c` format,
`a`/`o` arm/open the gate, `x` cycle screens (the rendering load), `j` jitter report, `r`
example result, `n`/`p` screens, `?` status. `capture.py` works on it unchanged. Read it
without resetting it: `uv run python -m tools.board_ui send "?" --port COM4 --baud 921600`.

The bring-up sketches still build, and flashing one replaces the firmware:

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
