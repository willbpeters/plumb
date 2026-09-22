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
| §9.1 Axes and signs | **not done** — needs the board rotated by hand |
| §9.2 Dropped samples | **PASS on the direct read path** — 54720 samples over 60 s, zero lost, zero duplicated, overflow flag clear throughout. Still fails on the FIFO path (21.9% lost) |
| §9.3 Measured ODR | **stroke rate done: 906.86 Hz**, against 896.8 nominal. Maximum rate not measured |
| §9.4 Resting gyro noise | **measured: 0.2765 dps** worst axis, against a 0.8 dps stillness threshold |
| §9.5 Tap test | **blocked** — needs a printed base, and neither read path can deliver uniformly sampled data above 1 kHz |

Read the sections in order: the 2026-09-21 morning session found the FIFO
defects, the later session read the datasheet, and the final section is the
direct-register experiment that settled it. Where the later sections correct
the earlier ones, they say so.

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

---

# Datasheet findings (2026-09-21, later session)

Datasheet obtained and read: QMI8658C rev 0.9, QST Corporation. Four things
settled, and one new defect found.

## 1. FIFO read mode really does suspend acquisition — CONFIRMED

FIFO_CTRL bit 7, FIFO_RD_MODE:

> "This bit is automatically set by using a CTRL9 command to request the FIFO to
> read data out of FIFO via FIFO_DATA register. It must be cleared again after
> the data read is complete **so that writing data to the FIFO can resume**."

The model behind the 19.5% prediction was right. Time in read mode is time the
sensor is not storing samples.

CTRL_CMD_REQ_FIFO also specifies the intended pattern, which this driver does
not follow:

> "The device will direct the FIFO data to the FIFO_DATA register 0x17 **until
> the FIFO is empty**. Then the host must set FIFO_rd_mode to 0."

The driver reads `min(available, capacity)` with capacity 64 against a 128-deep
FIFO, so it can exit read mode with data still in the buffer.

## 2. I²C is capped at 400 kHz — CONFIRMED

Table 38: `fSCL  SCL Clock Frequency  0 .. 400 kHz`. The read-mode loss cannot be
bought back with a faster bus.

This undermines the reasoning in parent spec §6.4, which justifies FIFO batching
by claiming it reduces bus cost "by an order of magnitude". Batching reduces
*transaction overhead*, not data volume, and at 12 bytes per sample the volume
dominates: 896.8 Hz × 12 B is ~25% of a 400 kHz bus no matter how it is read.
Direct register polling costs roughly 32% of the bus and loses nothing; the FIFO
costs ~25% and loses ~20% of samples. **§6.4's premise deserves re-examination.**

## 3. The low-pass filters default to OFF — FIXED

CTRL5 (0x06): `gLPF_EN` bit 4 and `aLPF_EN` bit 0 both default to 0, and this
driver never wrote CTRL5 at all. Bandwidth options are 2.66 / 3.63 / 5.39 /
13.37 percent of ODR.

Now configured per rate:

| Rate | Gyro LPF | Accel LPF | Why |
|---|---|---|---|
| Stroke | ON, mode 00 (23.9 Hz) | OFF | A stroke's content is under 20 Hz. Impact is a ~4 ms impulse whose leading edge must stay sharp. |
| Max (tap test) | OFF | OFF | §5.5 hunts resonance above 500 Hz. |

Effect on worst-axis σ over the full capture: 2.167 → 1.074 dps. The spiky
environmental content went away (0.5 s window σ max fell from 4.85 to 1.22).

## 4. Turn-on time is 150 ms — explains the intermittent init

Table 8: `System Turn On Time  150 ms  From Software Reset, No Power, or Power
Down`. `begin()` runs immediately after `Wire.begin()` with no settling delay,
which is the likely cause of the first-boot `FATAL` that a reset clears.

**Not yet fixed.**

## 5. NEW DEFECT — the FIFO read returns corrupted data

Noise depends on **position within the batch**, which is impossible for a
stationary sensor:

| Position in 64-sample batch | σ, gyro X (dps) |
|---|---|
| 2–7 | **0.37** |
| 28–35 | 1.07 – 1.26 |
| 56–63 | 1.05 – 1.34 |

Early samples are roughly 3× quieter than late ones. The datasheet's gyro noise
density is 15 mdps/√Hz, which at the LPF's 23.9 Hz bandwidth predicts σ ≈ 0.073
dps — so even the "clean" early samples are high, but the back half of each
batch is clearly not real data.

**Every noise figure in this document is contaminated by this.** The 1.09 dps
0.5-second-window result is not the sensor's noise floor and must not be used to
set `stillness_gyro_std_rad`.

## Where that leaves Task 9

**Not measured.** The best available lower bound is the ~0.37 dps seen in the
uncontaminated early samples, which would sit comfortably under the 0.8 dps
stillness threshold — but that is an observation from a broken read path, not a
measurement, and it is not evidence to build on.

## Suggested next step

Add a direct-register read path that bypasses the FIFO entirely (output
registers, no CTRL9, no read mode) and re-measure the noise floor. It is a small
change and it is decisive:

- If σ falls to roughly 0.1 dps, the FIFO read path is confirmed as the fault,
  and §6.4's FIFO-mandatory decision should be revisited — direct polling costs
  ~7 percentage points more bus and loses nothing.
- If σ stays near 1 dps, the noise is real and the stillness threshold needs
  raising instead.

Either outcome is actionable, which is what makes it the right next experiment.

---

# Direct register read path (2026-09-21, third session)

The experiment the previous section called for. Both read paths measured back
to back on the same board, at rest on the same surface, 60 seconds each, with
nothing between them but a serial command. Whatever the desk and the room were
doing, they were doing it to both.

Reproduce with, from `analysis/`:

```
uv run python tools/capture.py --port COM4 --seconds 60 --read-path direct --out rest_direct.npy
uv run python tools/capture.py --port COM4 --seconds 60 --read-path fifo   --out rest_fifo.npy
uv run python tools/rest_noise.py rest_direct.npy --odr 906.86
uv run python tools/rest_noise.py rest_fifo.npy   --odr 707.80 --batch 64
```

The `.npy` captures themselves are gitignored as capture artefacts, so the
numbers below are the record. They reproduced to within one sample across five
repeats of the direct capture.

## The answer

| | Direct registers | FIFO |
|---|---|---|
| Samples delivered in 60 s | 54720 | 42560 |
| Batches | 54720 (one sample each) | 665 (64 each) |
| Sequence gaps | **0** | 0 frames lost host-side; in-part loss not measurable |
| Duplicated samples | **0** | not measurable |
| Overflow flag | never set | set on **665 of 665** batches |
| Delivered rate | 906.86 Hz | 707.80 Hz |
| Sensor ODR | **906.86 Hz** | frozen counter, cannot be measured |
| Loss against what the sensor produced | **0%** | **21.9%** |

The FIFO's 21.9% is computed against the 906.86 Hz the direct path measured,
not against nominal. The earlier figure of 20.8% used the 896.8 Hz nominal rate
and was therefore slightly optimistic; the mechanism and the magnitude stand.

**The FIFO read path was the fault.** Both outcomes the previous section
predicted were on the table, and the first one won.

## Resting gyro noise (section 9.4) - measured

Board at rest, 60 s, stroke rate, gyro LPF enabled at mode 00.

| Axis | Direct sigma (dps) | Direct robust sigma | raw/robust | FIFO sigma (dps) |
|---|---|---|---|---|
| X | **0.2765** | 0.2548 | 1.09 | 0.3405 |
| Y | 0.2609 | 0.2432 | 1.07 | 0.3152 |
| Z | 0.2315 | 0.2201 | 1.05 | 0.2489 |

**Worst-axis sigma = 0.2765 dps against a `stillness_gyro_std_rad` threshold of
0.8 dps.** That is the number section 9.4 exists to produce, and it clears the
threshold with room to spare, which means the stroke detector can work on this
board.

Two things make this a floor rather than another upper bound:

1. **raw/robust is 1.05 to 1.09.** The earlier 2.17 dps measurement had a ratio
   of 1.4, a heavy tail consistent with real mechanical disturbance. This
   distribution has essentially no tail: the estimator that ignores outliers and
   the one that does not now agree.
2. **Half-second windows barely vary.** Over 120 windows the worst-axis sigma
   runs min 0.2207, median 0.2842, max 0.4109. If the room were the dominant
   contributor the minimum would sit far below the median. It does not.

It is still **3 to 4 times the datasheet-typical figure**: 15 mdps/sqrt(Hz) over
the LPF's ~24 Hz bandwidth predicts 0.074 dps, or 0.092 if the filter's
equivalent noise bandwidth is reckoned as first-order. Unexplained, and not
worth chasing while there is a 2.9x margin against the threshold that matters.
Quantisation is not the explanation: one count is 0.0078 dps, so sigma is 35
counts.

## Measured ODR (section 9.3) - 906.86 Hz, and it is not nominal

906.86 Hz against 896.8 Hz nominal: **1.12% high**, repeatable to +/-0.01 Hz
across five separate 20 to 60 s captures. Measured from the sensor's own
TIMESTAMP counter against the MCU's `micros()`, so it is the part's real output
rate rather than a delivery rate.

This is exactly the error section 5 of the instrument spec was written to catch,
and it is larger than the example that section used. **A 1.12% scale error goes
straight into every integrated angle** - 1.8 degrees of face rotation would be
reported as 1.78 - and no downstream filtering removes it.

It is also a property of this board's MEMS oscillator rather than of the part
number, so it does not transfer to another unit. **This is a decision for
Will.** `SAMPLE_RATE_HZ` in `analysis/plumb/trajectory.py` is left at its
nominal 896.8, because baking one board's oscillator into a shared constant
would be worse than leaving it nominal. The real options are a per-unit ODR
calibration, or having the firmware measure its own rate at startup - which it
can now do in about ten lines, since the counter exists and works.

## Position within the batch (defect 5) - smaller than recorded, and still real

Sigma by position within the FIFO path's 64-sample batches, 665 batches:

| Samples | gx | gy | gz |
|---|---|---|---|
| 0-7 | 0.3576 | 0.3159 | 0.2498 |
| 8-15 | 0.3439 | 0.3011 | 0.2356 |
| 16-23 | **0.2677** | 0.2415 | 0.2052 |
| 24-31 | 0.2712 | 0.2451 | 0.2152 |
| 32-39 | 0.3579 | 0.3283 | 0.2572 |
| 40-47 | 0.3619 | 0.3465 | 0.2675 |
| 48-55 | 0.3547 | 0.3442 | 0.2700 |
| 56-63 | 0.3681 | 0.3600 | 0.2771 |

The spread is 1.4x, not the 3x recorded earlier (0.37 early against 1.0 to 1.34
late). It is not noise - with 5320 samples per group the standard error on sigma
is 0.003, so 0.268 against 0.368 is a 30-sigma difference - and a stationary
sensor cannot have noise that depends on where in a transfer a sample sat. So
the effect is real and it belongs to the transfer.

**Why the earlier figure was larger is not established.** The likeliest
explanation is the capture defect found in this session (below): the earlier
number was measured with a pipeline that could silently splice in stale frames
from a previous run, and the counter that would have caught it did not exist
yet. Treat the 3x as unreliable rather than as something that was fixed. The
1.4x is what today's measurement supports.

## New finding - the TIMESTAMP counter is frozen while the FIFO is enabled

Not in the datasheet, which says only that the counter is "incremented by one
for each sample (x, y, z data set) from sensor with highest ODR" (rev A, Table
24). Polled at 20 ms intervals with the part otherwise idle:

| FIFO_CTRL mode | TIMESTAMP over 8 polls |
|---|---|
| FIFO mode | 84, 84, 84, 84, 84, 84, 84, 84 |
| Bypass | 630, 649, 668, 687, 706, 725, 744, 763 |

In bypass it advances 19 counts per 20 ms, consistent with 906.86 Hz once the
poll's own bus time is counted. In FIFO mode it does not move at all. The
counter tracks writes to the **output registers**, and in FIFO mode the samples
go to the FIFO instead.

The consequence is architectural rather than cosmetic: **the FIFO path cannot
count what it loses.** The part offers a latched overflow flag that says loss
happened and nothing that says how much. Any future FIFO fix will have to be
validated from outside the part, by comparing its delivered rate against the ODR
the direct path measures.

This is why `DrainResult` carries `sensorCounted` and why the binary frame now
has a flag bit for it. Sequence numbers mean one thing on one path and a
different thing on the other, and an instrument that let those be confused is
how "dropped: 0" came to sit next to 20% sample loss.

## Correction - the turn-on time is two numbers, not one

The previous section recorded "Table 8: System Turn On Time 150 ms". That
conflates two rows of the datasheet, and both of them matter:

| Datasheet row | Value | What it governs |
|---|---|---|
| System Turn On Time (Tables 7 and 8) | **15 ms** | Initialisation after power-up or soft reset. Section 3.3.1: "during which, there should be no write/configuration to QMI8658C, to prevent possible interference and failure." |
| Gyro Turn On Time (Table 8) | **150 ms + 3/ODR** | How long after enabling the gyro its output is worth reading |

So the intermittent init needed 15 ms of patience, not 150, and the 150 ms is a
*settling* delay that the noise measurement needed and that nobody had applied
deliberately. `begin()` now does a soft reset (write 0xB0 to 0x60), waits 15 ms,
confirms the reset the way the datasheet specifies - register 0x4D reads 0x80 -
and waits the gyro's 150 ms after enabling the sensors.

**No init failure has been seen since, across roughly twenty flash-and-boot
cycles.** That is not proof, given the fault was intermittent to begin with, but
the mechanism is now understood and addressed rather than guessed at.

## New finding - ADDR_AI defaults to 0, and the two paths need opposite settings

CTRL1 bit 6 (rev A section 16.1): "If ADDR_AI = 0, the register address will not
increase... Note that the default value of ADDR_AI is 0, so it is recommended to
set it to 1 from beginning, in case of burst read/write is required." Table 19
gives CTRL1's reset value as 0b00100000, which confirms it.

The driver had never written CTRL1. That was correct for the FIFO path by
accident - every read of FIFO_DATA pops the next byte, so a burst that does
*not* advance the address is exactly right - and it would have been silently
wrong for the direct path: a 12-byte burst from AX_L would have returned twelve
copies of AX_L, decoded into six identical axes and a standard deviation that
meant nothing. The bit is now set per read path, by read-modify-write.

Bit 5 (BE, byte order) is deliberately left untouched. Table 22 gives its reset
value as 1, which would mean big-endian reads, but the measured resting
accelerometer magnitude - 2019 counts against 2048 expected for 1 g at +/-16 g -
proves the interface hands over the low byte first, as the driver assumes.
Something in that table is inconsistent with the part. Read-modify-write means
nothing here depends on which reading is right.

## Instrument defect found and fixed - the capture could splice in stale frames

Worth recording because it invalidates measurements taken before it was fixed,
including some in the section above.

`capture.py` opened the serial port, assumed the DTR/RTS toggle had reset the
board, and sent `s` to start streaming. Measured across four consecutive
attempts, **the reset happens only about half the time.** When it does not, the
board is still streaming from the previous capture - and `s` is a toggle, so it
*stopped* the stream. The board replied `# streaming 0`, which nothing was
reading.

The failure does not look like a failure. It produces a short capture of stale
in-flight frames, decoded out of order, and before this session there was no
counter that would notice: sequence numbers came from a delivered count, and
`accumulate()` ignored backwards jumps entirely. Symptoms, in three failing
runs: 587 samples instead of 18347, and 20979 backwards sequence steps that were
silently discarded.

Fixed on the host rather than by trusting the reset. `capture.py` now asks the
board what it is doing (`?`), parses `streaming=N`, and settles it to stopped
before configuring anything, while the link is still plain text. It also stops
the stream when it finishes, so the next run starts from a known state. Three
consecutive 20 s captures then returned 18347 samples and 906.85 to 906.86 Hz,
identical to within one sample.

Two smaller fixes alongside. The decoder's leftover buffer was keeping a
trailing byte that could not begin a frame, so it crept upward by one byte per
read for a whole capture. And duplicated samples are now counted (`repeats`)
rather than passed over, because a duplicate is the one kind of bad sample that
makes a noise floor look *better* than it is.

## What is still open

1. **The FIFO read path still loses 21.9% of its samples** and still delivers
   position-dependent sample quality. It is no longer on the path to the first
   build - parent spec 6.4 is amended to direct polling - but it is the only way
   to sample above about 1.1 kHz, so the tap test needs it fixed or needs a
   different plan. Untried: 1793.6 Hz with direct polling and a 12-byte read.
2. **Section 9.1, axes and signs.** Needs a hand on the board; no code will do
   it.
3. **Section 9.5, the tap test.** Needs a printed base, and item 1.
4. **The 3 to 4x gap to datasheet-typical noise density.** Unexplained.
5. **What to do about 906.86 Hz.** Per-unit calibration, a startup measurement,
   or accepting a 1.12% scale error. Will's call.
