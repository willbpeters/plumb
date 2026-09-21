# Bring-up measurements

Measured on hardware. Nothing here is estimated, and nothing is copied from a
datasheet — every number is something the board actually produced.

Produced by `firmware/bringup-arduino/imu_stream`, captured with
`analysis/tools/capture.py`. Section references are to
`docs/superpowers/specs/2026-09-21-imu-streaming-design.md`.

| Date | Firmware | Board |
|---|---|---|
| _not yet run_ | `imu_stream` | Waveshare ESP32-S3-Touch-LCD-1.28 |

---

## Axes and signs (§9.1)

Rotate the board about each axis and record which gyro channel responds, with
what sign.

| Rotation applied | Channel that responded | Sign |
|---|---|---|
| | | |

**Expected body frame:** Z along the shaft pointing from head to butt, X the
face normal, Y completing the right-handed set.

If the mapping differs, that is not a bug. It is the sensor's physical
orientation on the PCB, and it gets recorded here as the fixed rotation between
sensor axes and the body frame — never corrected by editing the algorithm.

**Result:**

---

## Dropped samples (§9.2)

60-second capture at stroke rate, and again at maximum rate.

| Mode | Samples | Batches | Dropped | Overflows |
|---|---|---|---|---|
| Stroke | | | | |
| Max | | | | |

Both dropped and overflows must be **zero**. If not, the batch size or the FIFO
watermark is wrong — that is the finding, and lowering the rate to hide it is
not the fix.

**Result:**

---

## Measured ODR (§9.3)

| Mode | Nominal | Measured | Difference |
|---|---|---|---|
| Stroke | | | |
| Max | | | |

`SAMPLE_RATE_HZ` in `analysis/plumb/trajectory.py` is an assumption until this
table is filled in. If the measured rate differs from nominal by more than about
1%, that error goes straight into every integrated angle and the constant must be
updated, the harness re-run, and the numbers in `analysis/README.md` refreshed.

**Note before running:** the spec's original 500 Hz is not available on this part.
The QMI8658's gyro ODR table is 7174.4 / 3587.2 / 1793.6 / 896.8 / 448.4 / 224.2 /
112.1 / 56.05 / 28.025 Hz. The driver currently selects 448.4 Hz as stroke rate.
Whether to move up to 896.8 Hz is an open decision — better tempo resolution and
lower integration error, against the §4.3 power budget.

**Result:**

---

## Resting gyro noise (§9.4)

The most consequential measurement here.

60 seconds at stroke rate, board resting on a solid surface.

| Axis | σ (dps) |
|---|---|
| X | |
| Y | |
| Z | |

```bash
cd analysis && uv run python -c "
import numpy as np
from plumb.sensor import FullScale
d = np.load('rest.npy').astype(float)
fs = FullScale()
gyro = np.degrees(d[:, 3:] * fs.gyro_rad_per_count)
print('per-axis sigma, dps:', gyro.std(axis=0))
print('worst axis sigma   :', gyro.std(axis=0).max())
"
```

**Compare against `Thresholds.stillness_gyro_std_rad`, currently 0.8 dps.**

The synthetic harness found that stroke *detection* fails — not degrades, fails —
once gyro noise approaches this threshold, while face-angle *accuracy* is still
degrading gracefully. Detection is the binding constraint, and this measurement is
what decides where the threshold can sit.

If measured σ is within roughly a factor of 3 of 0.8 dps, the threshold needs
raising, and the tempo accuracy that depends on a low onset gate needs rechecking.

**Result:**

---

## Tap test ringdown (§9.5, parent spec §5.5)

Requires a putter with a printed base fitted, so this is gated on the mechanical
work rather than on firmware.

Maximum rate, binary format. Strike the head with a **soft** mallet and capture
the ringdown, then FFT the accelerometer channels.

| Quantity | Value |
|---|---|
| Sample rate used | |
| Dominant resonance | |
| Verdict | |

**The gate (parent spec §5.5):**

- Resonance **above ~500 Hz** → mount is effectively rigid. Proceed.
- Resonance **below ~200 Hz** → unacceptably compliant. Escalate to the step-drill
  and expanding-collet path **before any further firmware work**.

This is the highest-risk unknown in the project. Every downstream accuracy number
depends on the answer.

**Result:**
