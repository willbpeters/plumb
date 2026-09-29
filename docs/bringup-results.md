# Bring-up measurements

Measured on hardware. Nothing here is estimated, and nothing is copied from a
datasheet — every number is something the board actually produced.

Produced by `firmware/bringup-arduino/imu_stream`, captured with
`analysis/tools/capture.py`. Section references are to
`docs/superpowers/specs/2026-09-21-imu-streaming-design.md`.

| Date | Firmware | Board |
|---|---|---|
| 2026-09-21 | `imu_stream` | Waveshare ESP32-S3-Touch-LCD-1.28 |
| 2026-09-22 | `imu_stream` | same board — §9.1 axes and signs |
| 2026-09-26 | `display` | same board — display and touch bring-up |

---

## Summary

| Criterion | Status |
|---|---|
| §9.1 Axes and signs | **PASS for the sensor triad** — gyro channels map 1:1 to board axes, right-handed, measured 2026-09-22. How the triad sits relative to the PUTTER still needs a printed base |
| §9.2 Dropped samples | **PASS on the direct read path** — 54720 samples over 60 s, zero lost, zero duplicated, overflow flag clear throughout. Still fails on the FIFO path (21.9% lost) |
| §9.3 Measured ODR | **stroke rate done: 906.86 Hz**, against 896.8 nominal. Maximum rate not measured |
| §9.4 Resting gyro noise | **measured.** Sensor floor **0.22-0.24 dps** (quietest windows, stable across sessions); whole-capture worst axis 0.28-0.56 dps depending on what the room is doing. Against a 0.8 dps stillness threshold |
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

Was "not yet done" through the 2026-09-21 sessions. Measured 2026-09-22 — see
"Axes and signs (section 9.1)" at the end of this document.

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

## Follow-up, same day — the noise floor is the sensor's, the total is the room's

A second direct capture taken about an hour after the one above, on the same
board, same surface, same firmware, and nothing touched in between:

| | 60 s capture | 20 s capture, later |
|---|---|---|
| Worst-axis sigma over the whole capture | 0.2765 dps | **0.5631 dps** |
| Quietest 0.5 s window | 0.2207 | **0.2440** |
| Median 0.5 s window | 0.2842 | 0.5182 |
| Worst 0.5 s window | 0.4109 | **1.2170** |
| raw/robust | 1.05-1.09 | 1.04-1.16 |

**The quietest windows agree to within 10%. Everything else moved.** That
splits the measurement cleanly in two, and the earlier section's claim that
"the room is not the dominant contributor" was true of that capture rather than
of this board in general:

- **The sensor's floor is about 0.22-0.24 dps**, stable across sessions and
  captures. That is the number that transfers.
- **The total at any moment depends on what the room is doing**, and it moved
  by 2x between two captures an hour apart. Most of it landed on gyro X (0.56
  against 0.23 on Z), which is a direction, so it is mechanical coupling rather
  than electrical noise. A tugging USB cable would do it.

### Why this matters more than the size of the number

Against the 0.8 dps `stillness_gyro_std_rad` threshold, the sensor has a 3.5x
margin and keeps it. But **one half-second window in the later capture reached
1.217 dps, which is over the threshold** — on a board that nobody touched.

So the thing that will defeat the stillness detector is not sensor noise. It is
the environment, and ADDRESS is exactly a stillness test over a window of about
this length. A golfer standing over a putt on grass is a quieter mechanical
environment than a desk with a cable on it, so this is not a reason to raise
the threshold — invariant 5 says thresholds come from logged data, and the data
that settles this is a corpus recorded on a green, not on a desk.

It is a reason to record, before Phase 2 begins, that **the false-negative mode
for stroke detection is environmental vibration**, and that the corpus has to
be captured somewhere representative or it will answer the wrong question.

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

---

# Axes and signs (section 9.1) — 2026-09-22

Measured with `tools/axis_check.py`, board rested on three faces and turned a
quarter turn anticlockwise about each vertical in turn. Gravity supplies the
reference: a sensor at rest reads +1 g on whatever axis points up, so the
"which axis is vertical" half needs no external instrument, and by the
right-hand rule an anticlockwise turn about that direction must read positive
on the matching gyro channel.

| Board axis up | Gyro that responded | Integrated angle | Verdict |
|---|---|---|---|
| +X | +X | +73.8 deg | **PASS** |
| +Y | +Y | +99.8 deg | **PASS** |
| -Z | -Z | -94.9 deg | **PASS** |

**The channel mapping is the identity, and the triad is right-handed relative
to the accelerometer.** Gyro X is board X, Y is Y, Z is Z, with no swap and no
sign flip anywhere. The port needs no remapping at the driver boundary, which
is the cheapest possible outcome and the one worth confirming rather than
assuming.

What this rules out is the defect class that matters: a left-handed triad would
have reported a stroke that opened as one that closed, and no amount of
downstream accuracy would have caught it.

## The angles are a sanity check, not a calibration

73.8, 99.8 and -94.9 degrees against a nominal 90. These were free-hand turns
of a book, so the spread is the hand, not the sensor. What they do establish is
that the whole chain -- counts to dps to integrated degrees, through the
full-scale constant, the ODR and the bias removal -- has no gross error in it.
A factor of two, a radians/degrees slip or a wrong full-scale constant would
have shown up here as 45, 180 or 5157 degrees rather than as "roughly a quarter
turn". Tier 0 of parent spec section 10.1, the printed protractor plate, is the
measurement that turns this into a calibration.

## Two things in the record worth keeping

**Cross-axis response was 28% and 40%**, against the ~16% the resting tilt alone
predicts (the board sat 6.0 and 9.1 degrees off square, and tan of those is
0.11 and 0.16). The excess is the turn not being purely about the vertical --
a board propped on edge and turned by hand wobbles. It does not threaten the
verdict, because the dominant channel led by more than a factor of two in every
round, but it is the reason this procedure cannot do better than identify
channels and signs.

**Gyro noise during the stillness windows read 1.12 and 2.31 dps**, against the
0.22-0.24 dps floor measured on a board lying flat and undisturbed. A board
propped on edge against a mug, with a cable attached, is a much worse
mechanical environment than a board lying flat -- which is the same lesson as
the half-second window that hit 1.217 dps, and more evidence that the threat to
stillness detection is the mounting and the environment rather than the sensor.
The tool derives its motion threshold from the noise it measures in that same
window, so the test worked anyway: it armed at 22.3 and 46.1 dps instead of the
~5 dps it would have used on a quiet board.

## What is still open in 9.1

**How the sensor triad sits relative to the PUTTER.** The body frame the
algorithm assumes is Z along the shaft pointing head to butt, X the face
normal. That is a property of the mount, not of the board, and it cannot be
measured until a base is printed and the puck seats in a grip at a known
clocking. This section establishes only that the three channels are what they
say they are, in the order and handedness the algorithm expects.

---

## Display and touch bring-up — 2026-09-26

Produced by `firmware/bringup-arduino/display` (design:
`docs/superpowers/specs/2026-09-26-display-bringup-design.md`), built and flashed with
`analysis/tools/board_ui.py`. Visual checks were made by Will at the board; everything else
was read back over serial.

**Status: the display renders, touch works, and the host-proven screens run unchanged on the
panel.** Phase 0's "display renders" item is closed.

### Pins — from the schematic, confirmed by the hardware working

Read from the Waveshare ESP32-S3-Touch-LCD-1.28 **Rev3** schematic, not measured; the panel
initialising, drawing and taking touches on exactly these pins is the confirmation.

| Net | GPIO | Net | GPIO |
|---|---|---|---|
| LCD_DC | 8 | I2C1 SDA (touch + IMU) | 6 |
| LCD_CS | 9 | I2C1 SCL (touch + IMU) | 7 |
| LCD_CLK | 10 | TP_INT | 5 |
| LCD_MOSI | 11 | TP_RST | 13 |
| LCD_MISO | 12 (unused) | IMU_INT1 / INT2 | 4 / 3 |
| LCD_RST | 14 | BAT_ADC | 1 |
| LCD_BL | 2 (low-side MOSFET, active high) | | |

### Orientation and colour — measured, by eye

| Setting | Value | How it was found |
|---|---|---|
| `MADCTL` | **0x48** (MX + BGR) | BGR alone (0x08) drew every glyph mirrored left to right; setting MX fixed it. Quadrant colours, positions and edge labels then all correct, USB-C toward the viewer. |
| Inversion | **on** (INVON, 21h) | Colours correct with it on; not changed. |
| SPI clock | **80 MHz** | No artifacts seen, so the 40 MHz fallback was not needed. |

Touch coordinates need no remapping: a swipe from right to left runs from x ≈ 220 to
x ≈ 100 in the panel's own coordinates.

### Full-screen refresh — measured

Render plus flush of the whole 240 × 240 screen, at rest after each screen's motion, via
`lv_refr_now()` timed with `micros()`. Two passes, identical to the microsecond.

| Screen | Full refresh |
|---|---|
| idle | 15.8 ms |
| face angle | 20.7 ms |
| tempo | 18.6 ms |
| path | 17.3 ms |
| impact speed | 20.0 ms |
| test pattern | 23.9 ms |

The SPI transfer alone is 115,200 bytes at 80 MHz = 11.5 ms, so rendering costs 4–9 ms per
full screen. Against parent spec goal 1's 500 ms from follow-through to result, drawing is
negligible.

Buffers: 2 × 28,800 B from `MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA` (invariant 7). Internal heap
free after init: 182,520 B. Missing glyphs: 0.

### Touch — measured

The controller answers at 0x15 with **chip ID 0xB5**. Common drivers read 0xB5 as the
**CST816T**, not the CST816S the board documentation names. The coordinate registers behave
identically, which is all this uses. Swipe log (Will, three lefts and a right):

```
touch 225 127  gesture LEFT -> next       release 163 130
touch 228 185                             release 225 186   (3 px: a tap, no gesture)
touch 213 104  gesture LEFT -> next       release 132 121
touch 218 114  gesture LEFT -> next       release 103 134
touch 208 135  gesture LEFT -> next       release 127 145
touch  36 146  gesture RIGHT -> previous  release  93 146
```

Every swipe produced the gesture in the right direction, and the screen changed each time.

### Charger — read from the schematic, NOT measured, and it matters before a cell is bought

The Rev3 schematic shows the charger as **ETA6098**. Parent spec §4.2 and §4.4 say ETA6096,
rated to 800 mA. Its current-set resistor R15 is **160 kΩ**, which the schematic's own table
maps to **1 A** (82 kΩ → 2 A, 66 kΩ → 2.5 A). On the parent spec's 400 mAh cell that is
**2.5C**, against §4.4's limit of 1C.

Not yet confirmed: the charge current on the bench, the value against the ETA6098 datasheet
rather than the schematic's table, and that this board is Rev3 (check the silkscreen). Until
then, treat 1 A as the charge current, and fit a cell of at least 1000 mAh or change R15.

---

## ESP-IDF firmware skeleton — 2026-09-27

Produced by `firmware/` (ESP-IDF v5.5.5; design:
`docs/superpowers/specs/2026-09-27-firmware-skeleton-design.md`, plan:
`docs/superpowers/plans/2026-09-27-firmware-skeleton.md`). Build and flash with
`firmware/idf.ps1 build` and `firmware/idf.ps1 -p COM4 flash`. The console runs at 921600;
`uv run python -m tools.board_ui send "?" --port COM4 --baud 921600` reads the status.

**Status: acquisition on core 0 is paced by the IMU's DRDY line and loses nothing at rest; the
UI runs on core 1; the invariant-8 gate is measured to remove the loss and the timing tail that
rendering causes.** Screens and swipes checked by Will at the board: right, and working.

### Loss — measured, from the sensor's own sample counter

| Condition | Samples | Lost | How |
|---|---|---|---|
| At rest, 60 s | 54,653 | **0** | `tools.capture --read-path direct`, unchanged; 906.93 Hz |
| Streaming + UI rendering continuously, 60 s | 54,177 | **391 (0.72%)** | also 222 duplicates, 89 wake-ups with no sample, 82 missed edges on the board's counters |
| Streaming + UI rendering requested, **gate armed**, 60 s | 54,720 | **0** | board counters did not move |
| UI rendering, not streaming, 30 s windows | ~28,600 each | 0, then 1 | the since-boot counter |

The measured rate, 906.93 Hz, agrees with imu_stream's 906.86 Hz from 2026-09-21 on a different
firmware and a different read path.

### Jitter — measured, 30 s windows, printed by `j`

Interval is between successive DRDY edges (ISR timestamps). Latency is from the edge to the end
of the locked read. Both are in µs, from a 1 µs histogram; percentiles are nearest-rank.

| Window | interval p50 / p99 / p99.9 / max | latency min / p50 / p99 / p99.9 / max |
|---|---|---|
| UI rendering continuously, gate open | 1103 / 1107 / 1110 / 1113 | 702 / 926 / 1025 / 1088 / **1148** |
| Same load requested, **gate armed** | 1103 / 1103 / 1106 / 1107 | 699 / 699 / 707 / 709 / **820** |

A first pair of windows, before the init fixes below, agreed to within a few µs (animating:
latency p50 918, max 1145; armed: p50 699, max 821).

What the numbers say:

- **The DRDY edges are steady under any load.** The intervals barely move, so the sensor's clock
  and the GPIO interrupt are not what rendering disturbs.
- **The read is what rendering slows.** Rendering moves the median read latency from 699 to
  926 µs, and it sets a tail that passes the 1103 µs sample period. The lock holds one sample
  from STATUSINT to GZ_H, and the datasheet says samples arriving while it is held are dropped
  (§13.2.3). That fits the losses under rendering and streaming together.
- **Where the delay comes from is not yet measured.** Acquisition is on core 0 and the ring
  never overflowed, so core 0 itself is being slowed. The two suspects are the cache the two
  cores share (LVGL on core 1 runs from flash and evicts core 0's I²C and task code) and the
  UART driver's interrupt, which was installed from core 0. The fixes, if they are wanted, are
  known: the acquisition path and the I²C ISR in IRAM, and the UART interrupt moved to core 1.
  **This matters for §11 logging:** flash writes stall the cache for both cores, so writing
  strokes to flash during a stroke would be the same problem, only worse.

### Wi-Fi absent — checked by the build, and the check shown to fire

`firmware/tools/check_no_radio.py` runs after every link and fails the build if any radio
archive (`libesp_wifi.a`, `libnet80211.a`, `libbt.a`, …) contributed to the image. Two things
were learned making it trustworthy:

- **Matching symbol names does not work.** The first real map named Wi-Fi in lines that are
  not radio code: ROM function addresses from the chip's linker script
  (`wifi_get_macaddr = 0x40005ab4`), and esp_hw_support's `wifi_bt_common_module_enable`, a
  peripheral-clock helper every build links. The check matches archives.
- **A canary build that links `esp_wifi_init()` fails it**, naming `libesp_wifi.a` and
  `libnet80211.a`. That build also overflows static DRAM by 21.5 KB, so the link itself fails
  before the post-link step runs; the check was run by hand on the map the linker wrote.

### The datasheet, corrected by the board

- **INT2 needs `CTRL1` bit 4.** Rev A's register table marks `CTRL1` bits 4:3 reserved. With them
  clear, no DRDY edge reached GPIO3: zero reads, and DRDY timeouts climbing at 10 per second.
  With bit 4 set, reads run at the sample rate. QMI8658A material and SensorLib name bit 4
  INT2_EN and bit 3 INT1_EN.
- **The CTRL9 handshake takes 3253 µs, every time.** The AHB-clock-gating command (0x12) was
  timed on every boot that reached it after the diagnostic went in: 96 boots, 3253–3254 µs.
  The first version of this driver copied imu_stream's budget of 50 STATUSINT reads, about
  3.2 ms at 400 kHz, right at that edge, and the handshake failed on 7 to 11 boots in 40. The
  poll is now bounded by time (100 ms). `CTRL8.bit7` is also set, which the register table says makes STATUSINT bit 7
  the handshake; section 5.10.1 says bit 7 is set either way. Which of the two changes mattered
  was not separated. The timing number suggests the budget did.
- **Every read finds Avail set and Locked still clear.** In SyncSample mode, the STATUSINT read
  that starts the lock sees Locked = 0, so the driver waits the 6 µs Data_Lock_Delay (Table 40)
  on every sample. The `unlocked` counter equals `reads`. Expected, not a fault.

### A reset in the middle of a read — measured, and fixed

The MCU can reset while the IMU is sending (the bus is busy about 45% of the time at
906.86 Hz), and the IMU is not reset with it. Across repeated resets through EN, **before:** 5
of 20, then 12 of 40, failed the IMU init. The status line now reports each boot's init step,
SDA level and bus-clear clocks, and they showed two separate causes:

| Cause | Evidence | Fix | After |
|---|---|---|---|
| SDA held low by the IMU mid-byte | every boot that found SDA low failed; ESP-IDF's `i2c_master_bus_reset()` left it low; a hand clear that stopped clocking at the first high SDA still failed 4 of 40 (a 1 data bit is not a release) | nine clocks with SDA released, then a STOP (NXP UM10204 §3.1.16), before the driver takes the pins | released in 1–7 clocks, every time |
| CTRL9 handshake timing out | STATUSINT never showed CmdDone within 50 reads | bounded by time; `CTRL8.bit7` | 3253 µs measured |

**After: 60 of 60 resets brought the IMU up**, six of them from a stuck bus.

### Memory — measured

Binary 651 KB (58% of the 1.5 MB app partition free). Internal heap free at runtime:
**52,947 B**, against 182,520 B under the Arduino display sketch. Not yet broken down: this
firmware adds the ring (14 KB) and two histograms (16 KB) in static DRAM, which does not
account for it all. The canary build's 21.5 KB DRAM overflow says static DRAM is the tighter
budget. PSRAM free: 2.08 MB,
untouched: the fonts are still in flash-mapped rodata (invariant 7 permits PSRAM for them; not
yet done).

### Not yet checked

- The 120 MHz flash of parent §6.3 (not attempted: it needs `SPI_FLASH_HPM_ON` and a flash part
  that supports it) and `LV_MEMCPY_MEMSET_STD` (an LVGL 8 name; its LVGL 9 equivalent lives in
  the shared `lv_conf.h`).

---

## Where rendering's delay comes from — 2026-09-27

Follows from the skeleton section above, which found that rendering on core 1 slowed the IMU
reads on core 0 and, together with streaming, lost 0.72% of samples. Found one variable at a
time, with the `j` report, which now splits each read's latency into three consecutive segments.

### The split — measured, 30 s windows, µs

| Segment | Armed p50 / max | Animating p50 / max | Rendering adds (p50) |
|---|---|---|---|
| Edge → task running | 7 / 14 | 10 / 70 | +3 |
| STATUSINT read (1 byte) | 162 / 249 | 320 / 474 | **+158** |
| Lock wait + burst (17 bytes) | 532 / 543 | 586 / 855 | +54 |

The interrupt and the scheduler are not the problem. The delay is inside the I²C transfers,
and mostly in the first one, although the second moves seventeen times the data. A bus or
memory slowed by rendering would hurt the long transfer most. The pattern fits **code gone
cold**: the I²C master driver runs from flash through the cache the two cores share, LVGL on
core 1 evicts it in the ~1 ms between samples, and the first transfer of each sample pays the
misses while the second finds the code warm.

### Cause 1: the I²C driver evicted from the shared cache — confirmed and fixed

The one change: a linker fragment (`firmware/components/board/linker.lf`) places ESP-IDF's
`i2c_master` and `i2c_hal` objects in IRAM. ESP-IDF v5.5.5 keeps only the driver's ISR there
by default; `I2C_ISR_IRAM_SAFE` would move four functions of the task-side path, not all of it.

| Animating, gate open | latency p50 | p99 | max | lost |
|---|---|---|---|---|
| Driver in flash | 930 | 1033 | 1156 | 4 since boot (includes boot-time loss, below) |
| **Driver in IRAM** | **700** | 958 | **1026** | 0 |
| Armed, for reference | 700 | 707 | 732 | 0 |

**Streaming while animating: 391 of 54,568 lost before, 0 of 55,064 after.** The worst read
now finishes inside the 1103 µs sample period. The cost is internal RAM: heap free fell from
52,947 to 38,711 B, about 12 KB of it this move (the record also grew 4 bytes, 2 KB across the
ring; the new histograms are in PSRAM).

### Cause 2: touch polling on the shared bus — the remaining tail, confirmed

With the driver in IRAM, the median while animating equals the armed median, and what is left
is a tail in about 1–3% of samples: STATUSINT p99 319 against 169, burst p99 786 against 539.

| Gate open | latency p50 | p99 | max |
|---|---|---|---|
| Nothing rendering, touch polled | 714 | 937 | 973 |
| Animating, touch polled | 700 | 958 | 1026 |
| **Animating, touch polling off** (a temporary build) | **714** | **729** | **745** |

LVGL reads the touch controller every 30 ms, about one IMU sample in 27. A 5-byte touch read
holds the shared bus for about 150–250 µs, the size of the extra time, and it lands in
whichever of the two IMU transfers it collides with. It is independent of rendering, and the
gate already stops it during a stroke (invariant 8; parent §6.4). Outside a stroke it costs
latency, not samples: 0 lost in every window with it.

If the tail ever matters outside a stroke, the fix is to schedule, not to suspend. A touch read
fits in the ~400 µs the bus is free after each IMU read, so the app could poll touch right after
an IMU read completes instead of on LVGL's own timer. Not done; it is a design decision.

### Open: samples lost in the first second after boot

Across 12 resets, 10 boots lost samples in the first ~0.9 s: usually 1 lost and 1 duplicated,
once 2 and 2, and once **24 lost** (the acquisition task starved for about 26 ms). It never
grows after boot; a 60 s stream afterwards added nothing. The leading suspect, not yet tested:
the touch controller's init on core 1, which starts while the IMU is already running, and whose
reads have a 10 ms timeout; a controller still waking from reset could hold the bus that long.
It matters once sleep and wake (§9) put a boot, or a wake, right before a stroke.

---

## Sample rate measured at startup (2026-09-28)

Will's decision of 2026-09-27, implemented: the firmware measures the IMU's rate itself, and the
pipeline integrates with it (spec §6.4, amended 2026-09-28). `acq/rate.c` fits DRDY edge time
(`esp_timer`, in the ISR) against the sample index from the sensor's own counter, over the first
1,024 samples after boot, rejecting mispaired edges at 5 robust sigmas.

**Cold boot, just after flashing:** 906.9249 Hz, 1,023 of 1,024 points kept, residual RMS
0.40 µs, max 3.84 µs, no samples lost.

**30 back-to-back measurements (`m`), board warm** — `tools/rate_check.py`:

| | |
|---|---|
| Mean | **906.9314 Hz** |
| SD | 2.5×10⁻⁴ Hz (**0.28 ppm**) |
| Range | 906.9310 – 906.9319 (1.0 ppm) |
| Fit's own standard error | 3.2×10⁻⁵ Hz, residual RMS 0.33–0.39 µs, 0–2 points rejected per run |
| Spread / reported error | **8.0** |

The spread is eight times the fit's error, and it is not scatter: the sequence falls smoothly
from 906.9319 to 906.9310 and climbs back. That is the oscillator wandering, which the fit does
not model and should not; it is what bounds one measurement, at about a part per million.

**Against an independent clock.** Streaming 60 s (54,540 samples) and fitting the sample index
against this PC's arrival times: **906.9436 ± 0.0016 Hz, 13.4 ppm above the board's figure.**
`esp_timer` counts the ESP32-S3's crystal; the PC has its own. 13 ppm is an ordinary
disagreement between two crystals, and the check bounds their difference without saying which
is right. So the measurement's systematic uncertainty is **of order 15 ppm**, and nothing on
the board can shrink it — the crystal is the reference.

**Across days, this unit:** 906.86 (09-21) → 906.93 (09-27) → 906.925 cold, 906.931 warm (09-28).
About 75 ppm between days and 7 ppm of warm-up. None of it matters to a metric (100 ppm is
0.001° on a 10° rotation, against 11,200 ppm uncorrected), but every one of those numbers
would have been a stale constant.

---

## The stroke pipeline on the board (2026-09-28)

`firmware/main/stroke.c`: a `pl_session` on the app task (core 1), fed from the ring drain,
driving the render gate from BACKSWING through FOLLOWTHROUGH.

**Memory.** The first link overflowed `dram0_0_seg` by 143,912 bytes: a 157 KB session in
single precision, 130 KB of it the 3 s path ring. At runtime 38.7 KB of internal SRAM was free.

| | Internal SRAM |
|---|---|
| LVGL code in IRAM (`LV_ATTRIBUTE_FAST_MEM_USE_IRAM`) | 111,946 B |
| LVGL static heap (`LV_MEM_SIZE` 96 KB) | 98,844 B |
| LVGL heap actually used, every screen cycled 20 s, measured twice | **9,968 B peak** |
| Free after moving LVGL code to flash and the heap to 32 KB | 204,415 B |
| Largest single free block | 126,976–139,264 B — the session did not fit in one piece |
| Free after the path ring (98 KB + 33 KB) and session (26 KB), allocated separately | **58,799 B** |

**Rendering cost of LVGL in flash** — `lv_timer_handler` duration while cycling screens, 20 s:
14.05 ms mean / 16.32 ms max in IRAM, 14.83 / 18.30 ms in flash.

**Step cost at rest** (ADDRESS, float): mean 272 µs, max 743 µs over 8,185 steps, against a
1,103 µs sample period. Most of it is the rest test's per-sample window statistics.

**Sample rate this boot:** 906.8538–906.8686 Hz across three boots, against 906.93 earlier the
same day — another ~75 ppm of the drift spec §6.4 now absorbs.

**Not yet exercised:** a real stroke. That needs a hand on the board; HANDOFF.md, next task 1.
