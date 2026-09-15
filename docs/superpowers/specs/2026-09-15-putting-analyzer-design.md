# Grip-Mounted Putting Stroke Analyzer — Design Specification

**Working name:** Plumb
**Date:** 2026-09-15
**Author:** Will Peters
**Status:** Approved for planning

---

## 1. Overview

A self-contained, battery-powered sensor that mounts in the butt of a putter grip and reports
stroke metrics on an onboard round display immediately after each putt. No phone, no app, no
network connection.

The device consists of two physically separate parts:

- A **puck** — the electronics, display and battery, shared across all putters.
- A **base** — a cheap printed part that is installed semi-permanently in each putter and
  carries the angular index the puck seats against.

The engineering substance of the project is not the hardware. It is the derivation of putter
face orientation from a single 6-axis IMU mounted a meter away from the face, and the
empirical validation of that derivation against an independent reference system.

### 1.1 Goals

1. Report **face angle at impact**, **tempo ratio**, and **stroke path** within 500 ms of
   follow-through, on-device.
2. Achieve validated face-angle accuracy sufficient for coaching use (see §3).
3. Produce a published accuracy study covering blade, mallet and zero-torque putters.
4. Move between putters in under ten seconds with no recalibration.

### 1.2 Non-goals

These are explicitly out of scope and will not be built:

- Companion mobile application.
- Cloud sync, accounts, or session history beyond on-device storage.
- Wi-Fi connectivity of any kind. The radio is disabled in firmware.
- Green reading, break estimation, or make/miss prediction.
- Full-swing (non-putting) analysis. Load cases and sample rates differ substantially.
- Absolute face angle relative to the target line. See §3.3.

### 1.3 Project positioning

This is a portfolio project whose purpose is to demonstrate depth in embedded systems,
sensor fusion and empirical validation. Where a choice exists between a faster path and a
more defensible one, the more defensible one wins. A finished device with unvalidated
numbers does not satisfy the goal.

---

## 2. Physical principle

For a rigid body, angular velocity is identical at every point. The grip end of a putter
rotates at exactly the same rate as the face, with no dependence on the distance between them.

Consequently:

| Metric | Requires putter geometry | Rationale |
|---|---|---|
| Tempo ratio | No | Pure timing |
| Face angle at impact | No | Angular velocity is position-invariant |
| Stroke path | **Yes** | Translation scales with the sensor-to-face lever arm |

Face angle and tempo are therefore putter-agnostic and require only the fixed rotational
offset between the sensor axes and the face normal. Path requires the lever arm vector `r`
from sensor to sweet spot, which is a per-putter quantity.

For a point P offset by `r` from the sensor S on a rigid body:

```
a_P = a_S + ω̇ × r + ω × (ω × r)
```

This relation is used in two places: to propagate motion from the sensor to the face when
computing path (§7.4), and inverted, to estimate `r` itself during optional auto-calibration
(§8.4).

### 2.1 Putter type is a data difference, not a calibration difference

A zero-torque putter places the head's center of gravity on the shaft axis, so the head has
no inherent tendency to rotate open and closed through the stroke. A blade or face-balanced
mallet places the CG off-axis and does rotate.

This is a difference in the motion the device measures, not in how the device should measure
it. The algorithm must therefore contain **no putter-type priors**: no normalization of face
rotation against expected magnitude, and no segmentation thresholds tuned on rotation
amplitude. A zero-torque putter that produces low face rotation is reporting a correct
result, not a degraded one.

The only per-putter quantities in the system are the clocking offset and the lever arm.

---

## 3. Success criteria

All figures are measured against the reference system described in §10, over the study
design in §10.3.

| Metric | Target |
|---|---|
| Face angle, bias vs reference | ≤ 0.5° |
| Face angle, 95% limits of agreement | ≤ ±1.0° |
| Face angle, test–retest repeatability (remount, same putter) | ≤ 0.5° SD |
| Tempo ratio, absolute error | ≤ 0.05 |
| Path arc magnitude, relative error | ≤ 10% |
| Path direction classification (straight / in-out / out-in) | ≥ 95% agreement |
| Stroke detection, true positive rate | ≥ 98% |
| False stroke triggers | ≤ 1 per 30 min of idle handling |
| Result latency after follow-through | ≤ 500 ms |
| Strokes per charge | ≥ 500 |
| Putter changeover time | ≤ 10 s |

### 3.1 Why path has a looser target

Path is derived from translation, which requires integrating acceleration. Integration of
noisy acceleration accumulates error quadratically in time. A 10% arc-magnitude target is
honest for this sensor placement; a tighter claim would not survive validation.

### 3.2 Why repeatability is listed separately

Test–retest repeatability isolates mounting error from algorithm error. If bias against the
reference is acceptable but repeatability is poor, the mechanical index is at fault, not the
math. These are separately actionable, so they are separately measured.

### 3.3 Declared limitation: face angle is relative to address

The device has no external heading reference. Gravity constrains pitch and roll absolutely
but provides no information about heading, and a magnetometer is unusable next to a steel
putter head.

The device therefore measures **face angle relative to the address position**, treating the
face at address as square. A golfer who addresses the ball with the face 2° open will be
told they delivered a square face on a putt that starts 2° right.

This limitation is inherent to grip-mounted sensing without an external reference. It will be
stated in the user-facing documentation and in the accuracy study, not hidden. An optional
per-putter "square to line" routine (§8.3) lets a user establish an absolute reference against
an alignment stick if they want one.

---

## 4. Hardware

### 4.1 Bill of materials

| Item | Part | Qty | ~Cost |
|---|---|---|---|
| MCU / display / IMU board | Waveshare ESP32-S3-Touch-LCD-1.28 | 1 | $25 |
| Battery | 3.7 V 400 mAh LiPo, protected (PCM), MX1.25 1.25 mm connector | 1 | $8 |
| Retention magnets | N52 neodymium discs, 4 × 6 mm | 4 | $2 |
| Puck shell | PETG, printed | 1 | ~$1 |
| Base | PETG, printed | 1 per putter | ~$0.30 |
| **Total (device + 3 bases)** | | | **~$37** |

### 4.2 Board resources

| Resource | Value | Allocation |
|---|---|---|
| MCU | ESP32-S3R2, dual-core Xtensa LX7 @ 240 MHz | Core 0: sensor. Core 1: UI. |
| Internal SRAM | 512 KB | LVGL draw buffers, IMU ring buffer, stack |
| PSRAM | **2 MB quad-SPI** | Fonts and image assets only — not framebuffers |
| Flash | 16 MB | ~2 MB application, ~13 MB LittleFS stroke log |
| Display | 240×240 round, GC9A01 over SPI | 80 MHz, DMA |
| IMU | QMI8658 6-axis, I²C @ 400 kHz on GPIO6/7 | FIFO batched, INT on GPIO3/4 |
| Touch | CST816S, shares the I²C bus | Polled only outside the stroke window |
| Battery sense | GPIO1, 200 K/100 K divider | `V = 3.3/4096 × 3 × adc` |
| Charger | ETA6096 over USB-C | Charge current must be ≤1C for the fitted cell; verified against the board schematic at bring-up |

### 4.3 Power budget

Active draw is approximately 70–100 mA (CPU at 240 MHz, backlight on, LVGL rendering, IMU at
500 Hz). A 400 mAh cell yields roughly 4–5 hours of continuous operation.

Because the device sleeps between strokes (§9), realistic practice use is measured in weeks
rather than hours. The ≥500 strokes-per-charge target in §3 is set against the sleep-enabled
duty cycle, not continuous operation.

The GPIO1 sense divider draws ~14 µA continuously across the battery, approximately 10 mAh
per month. This is accepted, not mitigated.

### 4.4 Battery integration hazards

These are build-time hazards with permanent consequences and are called out explicitly:

1. **Connector pitch.** The board accepts MX1.25 (1.25 mm). Most hobby LiPo cells ship with
   JST-PH (2.0 mm), which will not mate. Source a cell with the correct connector.
2. **Polarity.** Two-pin LiPo polarity is not standardized between vendors. Meter the cell
   against the board silkscreen before first connection. If reversed, re-pin the housing by
   lifting the retention tab; do not cut and splice.
3. **Never cut both battery leads simultaneously.** Cut one at a time.
4. **Charge rate.** The ETA6096 is rated to 800 mA but the programmed current is set by a
   board resistor. Read the Waveshare schematic and confirm charge current ≤ 1C for the
   chosen cell. If the board is set to 500 mA, a 400 mAh cell is at 1.25C and a larger cell
   should be used.
5. **Protection circuit.** Only cells with an integrated PCM are acceptable.

---

## 5. Mechanical design

### 5.1 Two-part architecture

**Base** (one per putter, installed once):

- Anchors into the grip butt hole via a **barbed taper**, chosen over a thread because grip
  butt holes vary in diameter between manufacturers and a barb tolerates that variation.
- Presents a **D-shaped key** on its upper face.
- Carries four magnet pockets.

**Puck** (one total, moves between putters):

- Printed shell housing the board and cell, with a D-shaped socket in its base and matching
  magnets.
- USB-C port exposed through a shell cutout for charging.

Separation of concerns: the barb anchors, the D-key indexes, the magnets retain. No single
feature does two jobs.

### 5.2 Why magnets are safe here

The QMI8658 is a 6-axis part with no magnetometer. Magnetic retention would destroy the
heading estimate of a 9-axis sensor, but this design has already abandoned magnetic heading
because a steel putter head makes it unusable. The two constraints cancel.

### 5.3 Clocking: mechanical repeatability, software truth

A barbed or threaded base seats at an arbitrary rotation. This is not corrected mechanically.

- The **D-key provides repeatability** — the puck returns to the same angular position on
  every remount.
- A **stored software offset provides accuracy** — measured once per putter via the routine
  in §8.3, and valid indefinitely because the base does not move.

This eliminates the need for a lockable two-piece hub with a set screw. If field experience
shows bases loosening and rotating, the mechanical version becomes a v2 item.

### 5.4 Mass

Base ~3 g, board 13 g, cell ~7 g, shell ~7 g — approximately **30 g**, standing roughly 20 mm
proud of the butt. This is mild counterbalancing and is generally perceived favorably.

Because the same puck is used on every putter, added mass is a controlled constant across the
validation study rather than a per-putter confound.

### 5.5 Primary mechanical risk: mount compliance

The base anchors into rubber. The device must resolve 1° through an impact shock event, and
any compliance between the base and the shaft means the IMU partly measures the plug flexing
rather than the putter rotating. A compliant mount also introduces a mechanical resonance
inside the signal band, which corrupts impact detection specifically.

**This is the highest-risk unknown in the build and is resolved empirically, not by
assumption.**

**Tap test procedure:** mount the puck, log raw IMU at maximum rate, strike the putter head
with a soft mallet, and take an FFT of the ringdown.

- Resonance above ~500 Hz → mount is effectively rigid. Proceed.
- Resonance below ~200 Hz → mount is unacceptably compliant. Escalate.

**Escalation path:** open the grip butt hole with a step drill and anchor an expanding collet
into the steel shaft bore (~.580" ID). This is rigid but destructive to the grip and cannot be
used on borrowed or demo putters.

### 5.6 Grip compatibility survey

Before the base geometry is finalized, the butt-end construction of every putter in the study
must be inspected. Many putter grips ship with a sealed or pinhole butt cap, and zero-torque
grips are frequently non-round. A base design premised on an open, round, concentric hole
will not fit all three study putters.

---

## 6. Firmware architecture

### 6.1 Platform

ESP-IDF with FreeRTOS. Chosen over Arduino for explicit task scheduling, core pinning and
timer control, all of which are load-bearing for sampling determinism.

LVGL 9.2 or later for the UI. Version 9.0 carried a documented ~37% performance regression on
ESP32-S3 relative to 8.3.9; 9.2+ recovers most of it and adds ESP32-S3 assembly-optimized
blending.

### 6.2 Core assignment

Graphics and sensor acquisition never compete for the same instant, and this is enforced
structurally:

- **Core 0** — IMU acquisition task. Hardware-timer driven, high priority, writes to a
  lock-free ring buffer. Nothing else runs on this core during a stroke.
- **Core 1** — LVGL rendering, UI state, storage writes, power management.

During an armed stroke, the UI renders nothing. The screen displays a result only after
follow-through completes, when nobody is looking at it anyway. This removes rendering as a
source of sampling jitter entirely.

### 6.3 Display configuration

```
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_ESPTOOLPY_FLASHFREQ_120M=y          # QIO
CONFIG_COMPILER_OPTIMIZATION_PERF=y
CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y
CONFIG_LV_MEMCPY_MEMSET_STD=y
# SPI LCD at 80 MHz with DMA
# LVGL draw buffers: 2 × (240 × 60) RGB565 = 57.6 KB, internal DMA-capable SRAM
# Display refresh period: 10 ms
# Wi-Fi: disabled
```

Draw buffers are placed in internal SRAM rather than PSRAM. Published benchmarks that favor
PSRAM framebuffers on this display use 8 MB octal PSRAM; this board carries 2 MB quad PSRAM at
roughly half the bandwidth, where internal SRAM wins. Espressif's guidance of 10–25% of screen
resolution per buffer, double-buffered, is followed.

### 6.4 IMU acquisition

- Sample rate: **500 Hz** during an armed stroke, 100 Hz while monitoring for address.
- Full-scale ranges: gyro ±250 dps, accelerometer ±16 g.
- **FIFO batching is mandatory.** A 12-byte burst read costs ~325 µs on a 400 kHz I²C bus;
  polling single samples at 500 Hz consumes ~16% of the bus and a great deal of CPU. Batched
  FIFO reads triggered by the QMI8658 interrupt reduce this by an order of magnitude.
- **Touch polling is suspended between arm and follow-through completion.** The CST816S shares
  the I²C bus and would otherwise steal bus time during the measurement window.

Accelerometer saturation at impact is expected and acceptable. Impact is used as a trigger,
not as a measurement; a clipped spike is a cleaner edge than an unclipped one.

The board commits the IMU to I²C. If a future revision requires sustained rates above 1 kHz,
that is the argument for a custom PCB with an SPI-connected IMU, and is out of scope here.

---

## 7. Signal processing

### 7.1 State machine

```
SLEEP → IDLE → ADDRESS → BACKSWING → DOWNSWING → IMPACT → FOLLOWTHROUGH → COMPUTE → DISPLAY → IDLE
```

| State | Entry condition |
|---|---|
| SLEEP | 60 s in IDLE with no motion |
| IDLE | Wake-on-motion interrupt from QMI8658 |
| ADDRESS | Gyro variance below threshold for 500 ms |
| BACKSWING | Sustained \|ω\| above threshold, consistent sign |
| DOWNSWING | Dominant-axis angular velocity sign reversal |
| IMPACT | Acceleration magnitude spike above threshold |
| FOLLOWTHROUGH | \|ω\| below threshold for 300 ms |

Entering ADDRESS captures two things: the gravity vector `g₀`, and the gyro bias `b` as the
mean angular rate over the stillness window. Sampling steps to 500 Hz on ADDRESS entry.

All detection thresholds referenced above are deliberately left unnumbered here. They are
derived empirically from the logged stroke corpus during Phase 2 and fixed in the Python
notebook before the algorithm is ported to C (§7.6). Choosing them analytically ahead of real
data would be guessing, and thresholds tuned on rotation amplitude in particular would
reintroduce the putter-type priors prohibited in §2.1.

### 7.2 Orientation estimation

Orientation is obtained by integrating bias-corrected angular velocity `(ω − b)` from the
address attitude, which is itself initialized from `g₀`.

**Accelerometer correction is heavily down-weighted during the stroke.** This is the single
most important implementation detail in the pipeline. A stock Madgwick or Mahony filter
assumes the accelerometer reads gravity; during a putting stroke it reads gravity plus stroke
acceleration, and a naive filter will be dragged off attitude by the motion it is supposed to
be measuring. The correction gain is set near zero between BACKSWING and FOLLOWTHROUGH, and
restored in IDLE and ADDRESS.

Gyro drift is bounded not by accelerometer correction but by the **short integration window**.
Bias is nulled at address and integration runs for roughly 1.5 s, so drift has no time to
accumulate. This is why per-stroke bias nulling (§8.5) is non-optional.

### 7.3 Face angle at impact

```
q_rel      = q_impact ⊗ q_address⁻¹
face_angle = project(q_rel, ground plane defined by g₀) + clocking_offset
```

The ground plane comes from the measured gravity vector at address, so shaft lie angle is
handled implicitly. Lie angle is never entered by the user and never stored; it is measured
on every stroke.

### 7.4 Path

Face-point velocity is obtained from `v_face = ω × r` using the stored lever arm, integrated
over the stroke window to produce a horizontal displacement track. Path is classified as
straight, in-to-out or out-to-in by comparing lateral displacement before and after impact,
and arc magnitude is reported alongside the classification.

Direction classification is the primary output and is held to a 95% agreement target.
Arc magnitude is secondary and held to 10%.

### 7.5 Tempo

```
tempo_ratio = (t_transition − t_backswing_start) / (t_impact − t_transition)
```

Two timestamps from the state machine. No fusion, no geometry. This is expected to be the
first metric working end-to-end and should be used to prove out the segmentation pipeline
before face angle is attempted.

### 7.6 Development workflow

The algorithm is **developed in Python against logged strokes, then ported to C.** Numerical
methods are not debugged on a device with a 1.28" screen and no debugger.

1. Firmware logs raw IMU to LittleFS.
2. Strokes are dumped over USB to CSV.
3. Segmentation and fusion are developed in a Jupyter notebook against the recorded corpus.
4. The settled algorithm is ported to C and re-verified against the same corpus.

The notebook and the stroke corpus are committed to the repository. They are part of the
deliverable, not scratch work.

---

## 8. Calibration model

Three tiers, at three different frequencies. The two-part mount collapses what would
otherwise be a separate per-installation tier into the per-putter profile.

| Tier | Corrects | Frequency | Storage |
|---|---|---|---|
| Device | Gyro and accelerometer bias and scale factor | Once per device | NVS |
| Putter profile | Clocking offset, lever arm `r` | Once per putter | NVS |
| Stroke | Gyro bias null, gravity reference | Every stroke | RAM |

### 8.1 Device calibration

Standard 6-position tumble test. The device is rested on each of six faces of a printed
fixture; accelerometer bias and scale are solved per axis, and gyro bias is captured while
stationary. Run once at build time and stored permanently.

### 8.2 Putter profiles

Profiles are stored in NVS and selected on-device. Each holds a name, a clocking offset, and a
lever arm vector. Profile switching is the only user action required when changing putters.

### 8.3 Clocking offset

Square the putter face against an alignment stick, hold still, confirm. The measured heading
becomes the profile's clocking offset.

This runs once per putter. Because the base remains in the grip, the offset stays valid across
remounts, which is the central benefit of the two-part mechanical design.

### 8.4 Lever arm

**Version 1 — manual.** The user selects putter length and head type from a list; a default
lever arm is derived. Approximately 90% of achievable path accuracy for a very small fraction
of the effort. This ships first.

**Version 2 — automatic.** With the putter head resting on the ground as a fixed pivot,
`a_face = 0`, so:

```
a_sensor = −( ω̇ × r + ω × (ω × r) )
```

The user waggles the grip for five seconds and the three components of `r` are recovered by
least squares over the collected samples. Convergence behavior is plotted and included in the
write-up.

Version 2 is a v2 feature and does not gate any milestone.

### 8.5 Per-stroke calibration

Gyro bias is re-nulled and the gravity reference re-captured during every ADDRESS stillness
window. This is what keeps orientation integration bounded (§7.2) and is not optional.

---

## 9. Power management

There is no power switch. The battery connects directly, and idle drain is managed in
firmware.

The QMI8658's wake-on-motion feature drives an interrupt on GPIO3/GPIO4. The ESP32-S3 enters
deep sleep after 60 s of stillness and wakes when the putter is picked up.

The resulting interaction has no buttons: pick up the putter and the display comes on; set it
down and it sleeps. Charging is via the exposed USB-C port.

---

## 10. Validation

### 10.1 Sequencing

Validation escalates in cost, and each tier gates the next. No money is spent on an external
reference until the free tiers pass.

| Tier | Cost | Tests | Gate to proceed |
|---|---|---|---|
| 0. Rotary jig | ~$0 | Static angular accuracy | Within ±0.5° at 0, ±1, ±2, ±5° |
| 1. Pendulum | $0 | Dynamic gyro integration | Within ±1° of energy-conservation prediction |
| 2. Phone at 240 fps | $0 | Face angle at impact | Within ±1–2° of video measurement |
| 3. Optical motion capture | $0–250 | Full independent reference | Study targets in §3 |

The rotary jig is a printed indexed protractor plate that clamps the device at known angles.
If Tier 0 fails, nothing downstream matters.

### 10.2 Reference system

**Preferred: optical motion capture via the Sac State Kinesiology department.** Vicon- or
Qualisys-class systems provide sub-millimeter ground truth, exceed the accuracy of any
commercial putting analyzer, cost nothing, and carry the possibility of faculty involvement or
research credit. This is pursued first.

**Fallback: SAM PuttLab.** Ultrasound triangulation, measuring the same kinematic parameters
via an independent physical mechanism. Approximately $100–250 for a fitting session.

**Excluded: Blast Motion and Capto.** Both are grip- or shaft-mounted IMUs. Shared technology
means correlated errors, making them a peer comparison rather than a reference.

**Excluded as primary: Quintic Ball Roll.** High-speed camera focused on ball behavior — roll,
skid, spin. Valuable as a secondary outcome check on whether reported face angle predicts
start line, but not a like-for-like kinematic reference.

### 10.3 Study design

- **3 putters × 30 strokes = 90 paired measurements.** Blade, mallet and zero-torque.
- **Synchronization by sequence, not clock.** One stroke at a time, logged in order on both
  systems, matched by index.
- **Test–retest block:** the puck is removed and remounted between blocks on at least one
  putter, to separate mounting error from algorithm error.

### 10.4 Statistics

Agreement is reported as **Bland–Altman limits of agreement** — mean difference ± 1.96 SD of
the differences — not as correlation. Two instruments can correlate at 0.98 and still disagree
by 3°; correlation measures whether they move together, agreement measures whether they report
the same value. Results are broken out per putter type to test whether the geometry
assumptions generalize.

### 10.5 Logistics

Two constraints that invalidate a session if unplanned:

1. **Bases must be pre-installed in putters the study controls.** A fitter's demo putters
   cannot be modified. The three study putters are owned or borrowed, with bases fitted and
   clocked in advance.
2. **Per-stroke data export must be confirmed before booking.** A summary report without
   per-stroke values is unusable for method comparison.

---

## 11. Data logging

A LittleFS partition occupies the ~13 MB of flash not used by the application.

One stroke at 500 Hz × 1.5 s × 6 axes × 2 bytes ≈ **9 KB**, giving capacity for roughly
**1,400 raw strokes** on-device.

Each record carries a header — timestamp, putter profile ID, firmware version, active
calibration values — followed by raw int16 samples. Raw samples are stored rather than derived
metrics, so the corpus remains usable when the algorithm changes.

The corpus is dumped over USB and is the input to the Python development workflow in §7.6.

---

## 12. Risks

| Risk | Impact | Mitigation |
|---|---|---|
| Mount compliance corrupts angular measurement | Project fails its accuracy target | Tap test in Phase 1 (§5.5); escalate to shaft-bore collet |
| Grip butt caps sealed or non-round | Base does not fit study putters | Compatibility survey before base geometry is frozen (§5.6) |
| Accelerometer correction drags attitude during stroke | Systematic face-angle error | Correction gain near zero during stroke window (§7.2) |
| I²C bus contention at 500 Hz | Sampling jitter | FIFO batching, touch polling suspended (§6.4) |
| Path accuracy unattainable via double integration | One of three metrics unreliable | Direction classification is the primary output; arc magnitude secondary (§7.4) |
| Motion capture access not granted | No gold-standard reference | SAM PuttLab fallback (§10.2) |
| Reverse battery polarity | Board destroyed | Meter before first connection (§4.4) |

---

## 13. Milestones

Timeline is set against the Fall 2026 semester, which ends 11 December 2026. A Spring 2027
semester abroad is a hard deadline for physical work on this project, so the validation study
must complete before the end of Fall.

| Phase | Deliverable | Target |
|---|---|---|
| 0. Bring-up | Board powers from battery, display renders, IMU streams, charging verified | Week 1–2 |
| 1. Mount | Base and puck printed, grip survey complete, **tap test passed** | Week 2–4 |
| 2. Logging | Raw IMU to LittleFS, USB dump, first stroke corpus collected | Week 4–6 |
| 3. Tempo | Full state machine, tempo reported on-device | Week 6–7 |
| 4. Face angle | Fusion in Python, ported to C, Tier 0–2 validation passed | Week 7–11 |
| 5. Path + UI | Lever arm, path classification, finished LVGL interface | Week 11–13 |
| 6. Study | Motion capture session, Bland–Altman analysis, written report | Week 13–15 |

Phase 1 carries a hard gate. If the tap test fails, the escalation path in §5.5 is taken
before any further firmware work, because every downstream accuracy number depends on it.

---

## 14. Repository structure

```
/firmware        ESP-IDF project
/analysis        Jupyter notebooks, algorithm development
/data            Logged stroke corpus
/hardware        Printed base and puck models, calibration jig, tap-test fixture
/docs            This spec, validation report, accuracy study
```

The accuracy study and the stroke corpus are deliverables of equal standing with the firmware.
