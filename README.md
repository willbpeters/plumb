# Plumb

A putting stroke analyzer that mounts in the butt of a putter grip. A 6-axis IMU derives face
angle, tempo, path and impact speed and shows them on a 240×240 round LCD after each putt. No
app, no network.

> **[Photo / GIF placeholder: the puck on a putter, and the result screens after a stroke]**

## What it measures

| Metric | Definition |
|---|---|
| **Face angle at impact** | Rotation of the face between address and impact. **Relative to address, not to the target line**: there is no heading reference, and a magnetometer is unusable next to a steel head. |
| **Tempo** | Backswing time ÷ downswing time. |
| **Path** | In-to-out / out-to-in / straight, and arc, reconstructed from a pivot point fitted per stroke and refined over a session. |
| **Impact speed** | \|ω × (d + r)\| at the last sample before impact, using the fitted pivot. |

It reports and does not grade: no screen says "good".

**Hardware:** Waveshare ESP32-S3-Touch-LCD-1.28 (ESP32-S3R2: dual core, 2 MB quad PSRAM,
16 MB flash), QMI8658 IMU, GC9A01 240×240 round LCD, CST816 touch. The IMU runs at a nominal
896.8 Hz; this unit measures **~906.9 Hz**, so the firmware measures the true rate at every
boot (least squares of data-ready edge time against the sensor's own sample counter) instead
of trusting the nominal value. A 1.1% rate error would be a 1.1% error in every angle.

## Architecture

```
analysis/ (Python, NumPy)                       firmware/ (ESP-IDF, C99)
  synthetic stroke generator ──► pipeline.py ══► pipeline.c, pivot.c, quat.c
  sensor model (measured noise)      ▲                    │
                                     └── differential harness: same int16 inputs,
                                         compared stage by stage
```

- **Python first, then C.** Every algorithm is developed and tested in Python against a
  synthetic generator whose ground truth is known, then ported to C. The board has no JTAG, so
  a harness compiles the firmware's C on the host and runs it against NumPy on identical
  inputs. In double precision the C matches NumPy **exactly** (0 difference) on bias, every
  stroke boundary, face angle and tempo over 14 strokes. The device builds in single
  precision, which costs 3.3×10⁻⁵° of face angle.
- **Core pinning.** Sensor acquisition runs on core 0, woken by the IMU's data-ready
  interrupt. The stroke pipeline and LVGL run on core 1, and rendering is suspended from
  backswing through follow-through. Measured: rendering during streaming lost 0.72% of
  samples; with the gate armed, 0 of 54,720. Moving the I²C driver to IRAM (rendering was
  evicting it from the shared cache) brought continuous rendering to 0 of 55,064 as well.
- **Wi-Fi and Bluetooth are compiled out of every build.** The radio stack changes interrupt
  behaviour and so sampling jitter, which the accuracy argument depends on, and there is one
  firmware configuration so the validated build is the shipped build. A post-link check
  (`firmware/tools/check_no_radio.py`) fails the build if any radio archive is linked; a
  canary build that calls `esp_wifi_init()` was used to show the check fires.
- **The UI is pure LVGL 9** with no ESP-IDF calls. It renders headlessly on the host, where
  the tests measure pixels back into the numbers each screen was asked to draw.

## Engineering decisions

- **Accelerometer correction gain → 0 during the stroke.** Mid-stroke the accelerometer reads
  gravity plus the stroke's own acceleration, so a stock Madgwick/Mahony filter is pulled off
  attitude by the motion it is measuring. Attitude is gyro-only from backswing to
  follow-through.
- **Gyro bias is re-nulled at every address.** Bias drifts between strokes. Measuring it in
  each address stillness window, then integrating for only ~1.5 s, keeps drift small without
  relying on the accelerometer.
- **No putter-type priors.** A zero-torque putter barely rotates. Segmentation keys off timing
  and acceleration, never rotation size, so a small rotation is reported rather than treated
  as an error.
- **Thresholds come from logged data, not guesses.** Detection thresholds are placeholders,
  held in one config file that a test keeps in sync between Python and C. They get their
  values from the real-stroke corpus.
- **Store raw samples, never derived metrics.** The stroke log records int16 sensor counts
  losslessly, including exactly which samples were lost, so the corpus stays usable when the
  algorithm changes.

## Results so far

**Every number here comes from synthetic strokes or a bench. None comes from a real putt.**

| | Result | Source |
|---|---|---|
| Face angle error | 0.0012° noiseless, 0.0756° at 0.5 dps gyro noise (target ±1.0°) | synthetic |
| Tempo ratio error | 0.009 (target 0.05) | synthetic |
| Path arc, session mean / truth | 1.006 straight, 1.010 arced (lie 5, measured noise + bias) | synthetic |
| Impact speed error | ≤ 0.16% noiseless; worst 0.70% with noise | synthetic |
| C vs NumPy, float build | 3.3×10⁻⁵° face angle; no stroke boundary moved | host harness |
| Sample loss, 60 s at rest | 0 of 54,653 | board |
| Pipeline step cost at rest | 272 µs mean, 743 µs max (1,103 µs sample period) | board |
| Startup rate measurement | 30 runs within 1.0 ppm of each other | board |
| Python suite | 305 tests: 303 pass, 2 strict xfails for a known defect (2026-11-04) | host |

The synthetic figures show the pipeline agrees with the generator's model of a stroke. The
generator and the pipeline share assumptions (impact at the address swing angle, swing about
one body axis, a symmetric impact pulse), so errors in those assumptions cannot show up here.
Measuring that is what the validation study is for.

### Not done yet

| Item | Status |
|---|---|
| Printed mount (grip base + puck) | Not printed |
| Tap test of mount resonance (§5.5) | Not started. Highest-risk unknown; also needs sampling above ~1.1 kHz |
| On-device stroke logging | Encoder and log written in C, 28 host tests: samples round-trip bit-exact through the C encoder and Python decoder. **Not yet compiled for or run on the board** |
| First real stroke on the board | Pipeline runs on the board at rest; no stroke swung yet |
| Accelerometer calibration | Solver written and tested; board tumble not run |
| Detection thresholds from real data | Blocked on the logged corpus |
| Real-putt validation (protractor plate, pendulum, 240 fps video) | Not started |
| Battery and charging | Blocked on a cell (charger is set to 1 A; see spec §4.4) |

## Repo layout

```
analysis/   Python: synthetic harness, pipeline reference, host tools, pytest suite
firmware/   ESP-IDF project: components/{acq,board,plumb,plumb_ui,rec}, main/
data/       Logged stroke corpus (raw int16 + header); empty until logging runs
hardware/   Mount, calibration jig, tap-test fixture models; empty until the base is printed
docs/       Design spec (source of truth), bring-up measurements, roadmap
HANDOFF.md  Current state, open defects, what is blocked
```

The design spec is `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md`; measured
hardware behaviour is in `docs/bringup-results.md`.

## Building and testing

Python tests (needs [uv](https://docs.astral.sh/uv/); the C-port and UI tests also need a host
C compiler: gcc, clang, `zig cc` or MSVC):

```bash
git submodule update --init
cd analysis
uv sync
uv run pytest
```

Check that the C tests ran, not skipped: without a compiler they skip.

Firmware (ESP-IDF v5.5.5, target `esp32s3`). From `firmware/` in PowerShell, using the wrapper
that sets up the IDF environment:

```powershell
./idf.ps1 build
./idf.ps1 -p COM4 flash
```

With an IDF environment already active, `idf.py build` and `idf.py -p <port> flash` work
directly. The console is at 921600 baud; `?` prints status.
