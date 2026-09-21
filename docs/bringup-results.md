# Bring-up measurements

Measured on hardware. Nothing here is estimated, and nothing is copied from a
datasheet — every number is something the board actually produced.

Produced by `firmware/bringup-arduino/imu_stream`, captured with
`analysis/tools/capture.py`. Section references are to
`docs/superpowers/specs/2026-09-21-imu-streaming-design.md`.

| Date | Firmware | Board |
|---|---|---|
| 2026-09-21 | `imu_stream` | Waveshare ESP32-S3-Touch-LCD-1.28 |

---

## Summary

| Criterion | Status |
|---|---|
| §9.1 Axes and signs | **not done** |
| §9.2 Dropped samples | **FAIL** — ~20% of samples lost in the FIFO |
| §9.3 Measured ODR | **blocked** by §9.2 |
| §9.4 Resting gyro noise | **upper bound only** — 2.17 dps, see caveats |
| §9.5 Tap test | **blocked** — needs a printed base, and §9.2 first |

Two defects found, one fixed. The instrument is not yet trustworthy for any
time-dependent measurement.

---

## Defect 1 — FIFO reads were per-sample (FIXED)

`drain()` issued one complete I²C transaction per 12-byte sample, so a 64-sample
drain cost 64 transactions. Parent spec §6.4 mandates FIFO batching precisely to
avoid this.

Replaced with chunked burst reads of 120 bytes (10 samples), the largest
multiple of 12 that fits the ESP32 Wire buffer without raising it. 64
transactions became 7.

| | Before | After |
|---|---|---|
| Delivered rate | 652.68 Hz | **709.91 Hz** |
| Loss vs 896.8 Hz nominal | 27.2% | 20.8% |

A real improvement, and it exposed the floor beneath it.

## Defect 2 — ~20% of samples are lost in the FIFO (OPEN)

`FIFO_OVFLOW` is set on essentially every batch: **224 of 225** consecutive
drains over 20 seconds.

The driver's reading of the datasheet (§8.7/§8.8) is that the FIFO does not
accept new samples while the part is in read mode — exiting read mode is what
lets samples "resume filling". If that is correct, then every microsecond spent
draining is samples the sensor never stores, and the loss is bounded by I²C bus
time:

```
768 bytes x 9 bits / 400 kHz            = 17.3 ms  per 64-sample drain
64 samples / 896.8 Hz                   = 71.4 ms  refill
predicted loss = 17.3 / 88.7            = 19.5%
measured loss                           = 20.8%
```

The prediction and the measurement agree, which makes the model likely correct
and makes this **architectural rather than a tuning problem**. Note the loss
fraction is independent of batch size — it depends only on ODR and I²C clock —
so a bigger or smaller watermark will not help.

A FIFO that loses a fifth of its data would make the part largely pointless, so
the more probable conclusion is that **this driver is not using the FIFO the way
the part intends**. Resolving it needs §8.7/§8.8 read properly, and the
candidates worth checking are: whether read mode must be entered per drain at
all, whether Stream mode behaves differently from FIFO mode here, and whether
`RST_FIFO` on every drain (currently triggered by the latched overflow flag) is
discarding data it need not.

**Consequence:** any time-dependent measurement is invalid until this is fixed.
That includes the measured ODR (§9.3) and the tap test (§9.5), which needs
uniformly sampled data for its FFT.

---

## Axes and signs (§9.1)

Not yet done.

---

## Dropped samples (§9.2)

| Mode | Samples | Batches | Dropped | Overflows |
|---|---|---|---|---|
| Stroke, 20 s | 14400 | 225 | 0 | **224** |

`dropped: 0` here is **not** reassuring, and that is a flaw in the instrument's
own verification. The firmware assigns sequence numbers as `seq += r.count` — it
counts samples *delivered*, not samples the sensor *produced*. Anything lost
inside the FIFO is invisible to the sequence check, and only the overflow flag
catches it. Worth fixing so the check means what it appears to mean.

---

## Measured ODR (§9.3)

| Mode | Nominal | Delivered | Difference |
|---|---|---|---|
| Stroke | 896.8 Hz | 709.91 Hz | −20.8% |

**This is a delivery rate, not the sensor's ODR.** It measures how fast samples
reach the host, which is throttled by defect 2. The sensor's true ODR is
untested. `SAMPLE_RATE_HZ` in `analysis/plumb/trajectory.py` stays at its
nominal 896.8 until defect 2 is fixed and this can be measured honestly.

---

## Resting gyro noise (§9.4)

Board flat on a desk, 14400 samples at stroke rate.

| Axis | σ (dps) | Robust σ (1.4826 × MAD) |
|---|---|---|
| X | **2.1670** | 1.5173 |
| Y | 0.9794 | 0.6950 |
| Z | 0.8124 | 0.6023 |

**Worst axis σ = 2.17 dps**, against a `stillness_gyro_std_rad` threshold of
0.8 dps. Taken at face value the device would not reliably reach ADDRESS, and
so would not detect strokes at all.

**Treat this as an upper bound, not the sensor's noise floor.** Three reasons,
each independently sufficient to inflate it:

1. **The LPF is never configured.** `begin()` does not touch CTRL5, which holds
   the gyro and accelerometer low-pass filter enables and modes. The part is
   very likely running unfiltered at full bandwidth.
2. **The environment is in the measurement.** Peaks of 11–13 dps appear, which
   is 6σ — for 14400 Gaussian samples you would expect none beyond about 5σ.
   Raw σ is 1.4× the robust σ, a heavy tail consistent with real mechanical
   motion rather than thermal noise. A desk carries footfall and HVAC, and a
   gyro at roughly 450 Hz bandwidth hears all of it.
3. **896.8 Hz doubles the bandwidth** relative to the 448.4 Hz alternative, and
   noise scales with the square root of bandwidth.

For scale, the QMI8658's noise density puts datasheet-typical near **0.21 dps**
at this bandwidth — about a tenth of what was measured.

**Sample loss does not explain it.** Dropped samples reduce the count but do not
bias a standard deviation, and the robust estimator rules out corruption: if
σ were inflated by scrambled samples the raw/robust ratio would be ~10×, not
1.4×, and there would be visible outliers. There are essentially none.

**Before re-measuring:** configure the LPF, and isolate the board — on foam, on
a floor rather than a desk, with nobody moving nearby.

### Accelerometer sanity check (passed)

At rest the accelerometer should read one g total. It read **9.689 m/s²**
against 9.81 expected, within 1.3%.

That independently confirms the ±16 g full-scale constant and, with it, the
`FullScale` correction made on 2026-09-21 — including the 2¹⁵ divisor. A wrong
full-scale constant would have shown up here as a magnitude off by a factor of
two.

---

## Tap test ringdown (§9.5, parent spec §5.5)

Not started. Blocked on two things: a putter with a printed base, and defect 2 —
an FFT of non-uniformly sampled data with a fifth of its samples missing would
not be meaningful.

---

## Other observations

**Initialisation is intermittent.** The first boot after flashing printed
`# FATAL: QMI8658 init or config read-back failed`; a reset with the same
firmware succeeded. Cause unknown. Candidates: the part needs settling time
after power-up that `begin()` does not allow, or it is left in an odd state when
the MCU resets without the sensor resetting.

**Overflow reporting is inconsistent between runs.** Two earlier captures
reported `overflows: 0` where later runs of the same firmware reported 208 and
224. Not explained.
