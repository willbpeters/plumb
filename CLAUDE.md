# Plumb — project context

Grip-mounted putting stroke analyzer. A 6-axis IMU in the butt of a putter grip derives face
angle, tempo and path, and displays them on a 240×240 round LCD. Standalone: no app, no
network.

**Source of truth: `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md`.** Read it
before writing code. Section numbers below refer to it. If this file and the spec ever
disagree, the spec wins — and say so rather than silently picking one.

This is a portfolio project. Where a faster path and a more defensible path conflict, take the
defensible one. A finished device with unvalidated numbers fails the goal.

---

## Hard invariants

These are the decisions that fail *silently* — code that looks correct, runs fine, and
produces wrong numbers. Do not change any of them without explicit approval from Will.

1. **No putter-type priors anywhere in the algorithm.** Do not normalize face rotation against
   expected magnitude. Do not tune segmentation thresholds on rotation amplitude. A
   zero-torque putter legitimately produces very little face rotation; an algorithm that
   assumes "normal" rotation will report it as broken. Thresholds key off timing and
   acceleration, never rotation size. (§2.1)

2. **Accelerometer correction gain goes to ~0 during the stroke.** Between BACKSWING and
   FOLLOWTHROUGH the accelerometer reads gravity *plus stroke acceleration*. A stock
   Madgwick/Mahony filter will be dragged off attitude by the motion it is supposed to be
   measuring. Restore the gain in IDLE and ADDRESS. (§7.2)

3. **Face angle is relative to address, not to the target line.** There is no external heading
   reference and a magnetometer is unusable next to a steel putter head. Never label the
   output as absolute face angle. (§3.3)

4. **Gyro bias is re-nulled every stroke** during the ADDRESS stillness window. Drift is
   bounded by the short (~1.5 s) integration window, not by accelerometer correction. This is
   not optional. (§7.2, §8.5)

5. **Detection thresholds are derived from logged data, never guessed.** They get fixed in the
   Python notebook during Phase 2 and only then ported. If you need a threshold before real
   data exists, parameterize it and leave it configurable — do not hardcode a plausible-looking
   number. (§7.1)

6. **Wi-Fi stays compiled out in every build.** One firmware configuration, and it is the one
   that gets validated. The Wi-Fi stack changes interrupt behavior and therefore sampling
   jitter, which is what the whole accuracy argument rests on. No OTA. (§14.4)

7. **LVGL draw buffers live in internal SRAM, not PSRAM.** This board is the ESP32-S3**R2** —
   2 MB *quad* PSRAM, roughly half the bandwidth of the octal parts used in published
   benchmarks. PSRAM is for fonts and image assets only. (§6.3)

8. **Sensor acquisition is pinned to core 0, LVGL to core 1**, and nothing renders between arm
   and follow-through. (§6.2)

---

## Stack

- **Firmware:** ESP-IDF + FreeRTOS, C. Target `esp32s3`.
- **UI:** LVGL **9.2 or later** (9.0 carried a ~37% ESP32-S3 regression). UI layer must be
  pure LVGL with zero ESP-IDF calls so it builds against the PC simulator.
- **Analysis:** Python 3 + NumPy + SciPy, Jupyter notebooks under `analysis/`.
- **Board config traps:** PSRAM mode **QSPI** (not OPI); flash size **16 MB**; in Arduino IDE
  only, "USB CDC On Boot" must be **Disabled**. (§14.2)

## Layout

```
/firmware        ESP-IDF project
/analysis        Python: synthetic harness, algorithm development, notebooks
/data            Logged stroke corpus (raw int16 + header)
/hardware        Printed base/puck models, calibration jig, tap-test fixture
/docs            Spec, validation report, accuracy study
```

## Conventions

- **Test-driven where testable.** The synthetic stroke harness (`analysis/`) is the algorithm's
  test suite — ground truth is known because it was generated. Write the failing test first.
- Algorithms are developed and proven in Python, then ported to C and **re-verified against
  the same corpus**. Do not develop numerical methods directly in firmware; there is no JTAG
  on this board (§14.3).
- Store raw samples, never derived metrics. The corpus must stay usable when the algorithm
  changes. (§11)
- Prefer small, single-purpose modules. The UI module takes a results struct and knows nothing
  about hardware.

## Do not

- Do not invent accuracy numbers, or describe anything as validated that has not been measured.
- Do not add features outside §1.2 (no app, no cloud, no Wi-Fi, no green reading, no full-swing
  support). Deferred-but-wanted items live in §1.2.1 and are distinct from non-goals.
- Do not trust a test that only checks the code against itself. Several defects in this project
  passed every such test. Compare against ground truth where ground truth exists.

## State

The board is in hand and working. **`HANDOFF.md` is current** — read it for what is built, what
is proven, the open hardware defects, and what is blocked on Will. The spec has been amended
three times against the datasheet and the hardware; those amendments are recorded in it.
