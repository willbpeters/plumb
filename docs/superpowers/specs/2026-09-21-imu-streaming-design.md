# IMU Streaming Instrument — Design Specification

**Working name:** Plumb
**Date:** 2026-09-21
**Author:** Will Peters
**Status:** Approved for planning
**Parent spec:** `2026-09-15-putting-analyzer-design.md` — section references below refer to it.

---

## 1. Purpose

Turn the board into a measuring instrument: configure the QMI8658, drain its FIFO on interrupt,
and stream raw int16 samples to a host.

This is not product firmware. It exists to answer four questions that cannot be answered any
other way, and that everything downstream depends on:

- Do the sensor axes and signs match the body-frame convention the algorithm assumes?
- Is the real sample rate what we think it is?
- What is the gyro noise floor on *this* board?
- Is the grip mount rigid enough (§5.5)?

The synthetic harness (`analysis/`) has already proven the fusion math to 0.0017° against known
ground truth. What it cannot tell us is anything about the physical sensor or the mount. This
instrument measures exactly those.

## 2. Scope

**In scope:** QMI8658 configuration, FIFO drain on interrupt, two wire formats, runtime control
over serial, and a host-side capture script.

**Out of scope:** on-device storage, LittleFS, stroke detection, the display, battery
management, and the tap-test FFT analysis itself. The analysis is a Python notebook written
once real ringdowns exist.

### 2.1 Why on-device logging is deliberately excluded

Logging *strokes* on-device requires detecting strokes, which requires segmentation thresholds,
which invariant 5 says must be derived from logged data. That is a loop with no entry point.

When logging is built (Phase 2), it should capture **motion windows, not strokes** — trigger on
something crude and unambiguous, capture a generous window either side, and segment offline.
Two reasons beyond breaking the loop:

1. §11 requires storing raw samples rather than derived metrics *so the corpus stays usable when
   the algorithm changes*. A device that records only what its current segmenter recognises has
   already applied the algorithm to the data, and discards precisely the strokes that would
   reveal the segmenter to be wrong.
2. §3 sets a false-trigger target of ≤1 per 30 minutes of idle handling. That cannot be measured
   by a device that stores only true positives.

This is recorded here so the reasoning is not lost before Phase 2 begins.

## 3. Architecture

Three translation units, split so the piece that eventually ports to ESP-IDF is isolated from
everything that does not.

| File | Responsibility | Ports to ESP-IDF |
|---|---|---|
| `qmi8658.h` / `qmi8658.cpp` | Register-level driver: init, ODR and full-scale configuration, FIFO drain, status and overflow reporting | **Yes** |
| `stream.h` / `stream.cpp` | Output framing — CSV and binary | No |
| `imu_stream.ino` | Setup, main loop, serial command handling | No |

The driver knows nothing about serial ports or output formats. That boundary is what makes the
eventual IDF port a translation rather than a rewrite, and it mirrors the reasoning that kept
`pipeline.py` streaming and free of lookahead.

Location: `firmware/bringup-arduino/imu_stream/`, alongside the existing `smoke_test`.

## 4. Sensor configuration

**Full-scale ranges: gyro ±256 dps, accelerometer ±16 g** (§6.4, amended 2026-09-21 — ±250 dps does not exist on this part; the QMI8658 gyro table is powers of two).

These are deliberately identical to `FullScale` in `analysis/plumb/sensor.py`, so that a count
means the same physical quantity in firmware and in Python. Any divergence here would silently
invalidate every comparison between simulated and real data.

**Two rates, selectable at runtime:**

| Mode | Rate | Purpose |
|---|---|---|
| Stroke | nearest supported ODR to 500 Hz | Axis checks, noise floor, stroke capture |
| Tap | the part's maximum supported ODR | §5.5 tap test |

The exact ODR values the QMI8658 supports are read from the datasheet at implementation time and
recorded in the driver as named constants. They are **not** guessed here, and the nominal value
is not trusted in any case — §9 requires the achieved rate to be measured.

**FIFO batching is mandatory** (§6.4). A 12-byte burst read costs roughly 325 µs on a 400 kHz
I²C bus; polling single samples at 500 Hz would consume about 16% of the bus. The FIFO watermark
interrupt drives batch drains, on the INT line identified in §4.2.

## 5. What "jitter" means here, and what it does not

Sample spacing is set by the QMI8658's internal oscillator, not by when the MCU drains the FIFO.
That is the entire point of FIFO batching.

**Per-sample MCU timestamps would therefore be misleading.** They would record when a sample was
*read*, not when it was *taken*, and labelling that jitter would attribute bus and scheduling
latency to the sensor's time base.

What firmware can genuinely get wrong is **losing samples**. A FIFO overflow silently breaks the
time base: the stream continues, the samples are valid, and the interval between two of them is
quietly wrong. Nothing downstream can detect this after the fact.

The instrument therefore reports three things instead of per-sample timestamps:

1. A **monotonic sample counter**, so the host can prove no sample was dropped.
2. The **FIFO overflow flag**, read every drain.
3. The **measured ODR** over a long window — total samples divided by elapsed time.

Point 3 matters more than it appears. `SAMPLE_RATE_HZ = 500.0` is assumed exact throughout the
harness, and MEMS oscillators are routinely a few percent off nominal. A true rate of 502 Hz puts
a 0.4% scale error into every integrated angle, which is a systematic bias no amount of
downstream filtering removes.

This also means the ODR and noise figures this instrument produces **transfer to the ESP-IDF
build unchanged**, because they are properties of the sensor rather than of the firmware. Only
overflow behaviour is implementation-dependent and needs rechecking after the port.

## 6. Wire format

### 6.1 CSV

One line per sample:

```
seq,ax,ay,az,gx,gy,gz
```

Raw int16 counts, not physical units — conversion belongs on the host, in one place. Readable in
the Arduino serial monitor and pastes directly into a spreadsheet. Used for axis checks and
anything at stroke rate.

### 6.2 Binary

Framed per FIFO batch:

| Field | Purpose |
|---|---|
| Sync word | Resynchronisation after a dropped byte |
| First sequence number | Gap detection across batches |
| Sample count | Batch length |
| Overflow flag | FIFO status at drain |
| Drain timestamp (`micros()`) | Honest about what it is: when the MCU read, not when the sensor sampled |
| N × 12 bytes | The samples |
| Checksum | Corruption detection |

Required for tap mode: at 921600 baud the link carries roughly 92 KB/s, and CSV at about 40 bytes
per sample caps usable capture near 1–2 kHz — too slow to resolve the resonance the tap test
looks for.

### 6.3 Why both

CSV is what you want when eyeballing whether an axis moved the way your hand did. Binary is the
only format that carries maximum-rate capture. The cost of supporting both is one branch in the
output path and a small host decoder.

## 7. Serial control

Single characters, no parser, no menu:

| Key | Action |
|---|---|
| `c` | CSV output |
| `b` | Binary output |
| `1` | Stroke rate |
| `9` | Maximum rate |
| `s` | Start / stop streaming |
| `?` | Status — current mode, rate, sample count, overflow count, measured ODR |

## 8. Host tooling

`analysis/tools/capture.py` reads the serial port, decodes either format, verifies sequence
continuity, and writes raw int16 samples to a file.

It lives inside the existing `analysis/` uv project rather than beside the firmware, so that
`from plumb.sensor import FullScale` resolves without path manipulation. Run as
`uv run python tools/capture.py` from `analysis/`. This adds `pyserial` as the project's first
non-numerical dependency.

Importing `FullScale` rather than redefining the scale factors means counts-to-physical
conversion is defined in exactly one place across the entire project, firmware constants
included. A capture script with its own copy of "±256 dps over 16 bits" is a second source of
truth that will eventually disagree with the first.

## 9. Acceptance criteria

The instrument is finished when it has produced these five results, with the numbers recorded:

1. **Axes and signs verified.** Rotating the board by hand about each axis produces the expected
   sign on the expected channel, consistent with the body frame: Z along the shaft pointing from
   head to butt, X the face normal.
2. **Zero dropped samples** across a 60-second capture, proven by sequence continuity, with the
   overflow flag clear throughout.
3. **Measured ODR stated** for both modes, against nominal. A number, not an assumption.
4. **Resting gyro noise σ measured, in dps, per axis.** This is the most consequential output of
   the whole instrument. The synthetic harness showed that stroke *detection* — not accuracy — is
   the binding constraint, and that it fails once gyro noise approaches the stillness threshold.
   This measurement is what tells us where that threshold can actually sit on real hardware.
5. **A tap produces a resolvable ringdown** at maximum rate, with usable spectrum beyond 1 kHz.
   This is the precondition for §5.5 meaning anything: the test looks for resonance above
   ~500 Hz, and Nyquist requires sampling above 1 kHz to see it. Sampling at 500 Hz would alias
   the exact band being cleared.

## 10. Open items

The QMI8658's supported ODR values, FIFO depth, watermark configuration and INT pin behaviour are
read from the datasheet during implementation. They are not recorded here because guessing them
would be exactly the kind of plausible-looking constant invariant 5 prohibits, and because §9
requires measuring the achieved rate regardless of what the datasheet claims.
