# IMU Streaming Instrument Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the board into a measuring instrument — configure the QMI8658, drain its FIFO on interrupt, and stream raw int16 samples to a host in either CSV or framed binary.

**Architecture:** An Arduino sketch in three translation units, with the register-level driver isolated from all output concerns so it ports to ESP-IDF later. A host-side Python capture script decodes either format and shares its scale factors with the synthetic harness.

**Tech Stack:** Arduino-ESP32 core 3.3.11, C++; Python 3.11 with pyserial on the host, inside the existing `analysis/` uv project.

**Spec:** `docs/superpowers/specs/2026-09-21-imu-streaming-design.md` (this feature), `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` (parent; §N refers to it).

---

## How this plan is verified, and why it differs from the harness plan

The synthetic harness was verified entirely by pytest, because every part of it was pure computation. This is firmware, and the split is:

| Component | Verified by |
|---|---|
| Host decoder (`capture.py`) | **pytest, TDD**, against golden byte vectors written by hand |
| Firmware compiles | `arduino-cli compile` — catches the majority of mistakes in seconds |
| Firmware behaviour | **On-device acceptance runs** (Tasks 7–10), with numbers recorded |

**Deliberate trade-off:** the firmware's binary *encoder* is not unit-tested. Testing it properly would mean a native build with a stubbed `Wire`, which is real infrastructure for throwaway bring-up code. Instead the wire format is pinned by golden vectors that the host decoder is tested against, so an encoder mismatch surfaces on the very first capture as a failed sync or checksum — loudly, immediately, and with the decoder already known-good. That is adequate for an instrument. It would not be adequate for product firmware.

**Nothing in Tasks 7–10 can be faked.** They require the board, and in one case a putter. If you are an agent executing this plan, stop at Task 6 and hand back.

---

## Constants that must come from the datasheet

The QMI8658's ODR values, FIFO depth, watermark configuration, control-register addresses and INT behaviour are **not written in this plan**. Guessing them would be exactly the plausible-looking constant invariant 5 prohibits, and a wrong register address produces a device that configures silently and streams garbage.

Task 3 reads them from the datasheet, defines them as named constants, and — critically — **reads the configuration back and verifies it matches what was written**. That read-back is what makes a wrong constant fail loudly instead of silently.

Two values are already known, confirmed on hardware by the existing smoke test:

- `WHO_AM_I` is register `0x00` and returns `0x05`
- `REVISION` is register `0x01`
- The device answers at I²C address `0x6B`, on SDA GPIO6 / SCL GPIO7 at 400 kHz

---

## File Structure

| File | Responsibility |
|---|---|
| `firmware/bringup-arduino/imu_stream/qmi8658.h` | Driver interface: config struct, sample struct, function declarations |
| `firmware/bringup-arduino/imu_stream/qmi8658.cpp` | Register map, init, ODR/FSR config, FIFO drain, status. **No serial, no formatting.** |
| `firmware/bringup-arduino/imu_stream/framing.h` | Output interface: format enum, emit functions |
| `firmware/bringup-arduino/imu_stream/framing.cpp` | CSV and binary framing |
| `firmware/bringup-arduino/imu_stream/imu_stream.ino` | Setup, loop, single-character serial commands |
| `analysis/tools/capture.py` | Host decoder and capture CLI |
| `analysis/tests/test_capture.py` | Decoder tests |
| `docs/bringup-results.md` | Where the Task 7–10 measurements get written down |

---

## Wire format — pinned here, implemented in two places

Both the firmware encoder and the host decoder implement this. It is written once, here, so the two cannot drift.

**Binary batch frame**, little-endian throughout:

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | Sync | `0xA5 0x5A` |
| 2 | 4 | `first_seq` | uint32, sequence number of the first sample in this batch |
| 6 | 1 | `count` | uint8, number of samples, 1–64 |
| 7 | 1 | `flags` | bit 0 = FIFO overflow since last batch; other bits reserved, zero |
| 8 | 4 | `drain_micros` | uint32, `micros()` when the MCU drained the FIFO |
| 12 | 12 × `count` | samples | Six int16: `ax, ay, az, gx, gy, gz` |
| 12 + 12×count | 1 | checksum | XOR of every byte from offset 2 up to but not including the checksum |

**CSV**, one line per sample, raw counts:

```
seq,ax,ay,az,gx,gy,gz
```

A `#` prefix marks any non-sample line (status output, banners), so the host can skip it.

---

## Task 1: Host decoder — frame parsing

TDD applies fully here. Write the tests first.

**Files:**
- Create: `analysis/tools/capture.py`
- Test: `analysis/tests/test_capture.py`

- [ ] **Step 1: Add pyserial to the project**

```bash
cd analysis && uv add pyserial
```

Expected: `pyproject.toml` gains `pyserial` under `dependencies`, and `uv.lock` updates.

- [ ] **Step 2: Write the failing tests**

```python
# analysis/tests/test_capture.py
"""Golden-vector tests for the wire format.

The firmware encoder is not unit-tested -- that would need a native build with a
stubbed Wire. Instead the format is pinned by these hand-written vectors, so an
encoder mismatch shows up on the first real capture as a sync or checksum
failure, against a decoder already known to be correct.
"""

import struct

import pytest

from tools.capture import SYNC, Batch, decode_batches, parse_csv_line


def build_frame(first_seq, samples, overflow=False, drain_micros=12345):
    """Hand-build a frame to the spec in the plan. Deliberately does NOT reuse
    the decoder's constants beyond the sync word -- a test that shares an
    encoder with the code under test proves only that it is self-consistent."""
    body = struct.pack("<IBBI", first_seq, len(samples), 1 if overflow else 0, drain_micros)
    for s in samples:
        body += struct.pack("<6h", *s)
    checksum = 0
    for b in body:
        checksum ^= b
    return SYNC + body + bytes([checksum])


def test_decodes_a_single_batch():
    frame = build_frame(7, [(1, 2, 3, 4, 5, 6), (-1, -2, -3, -4, -5, -6)])
    batches, leftover = decode_batches(frame)
    assert leftover == b""
    assert len(batches) == 1
    b = batches[0]
    assert b.first_seq == 7
    assert b.overflow is False
    assert b.drain_micros == 12345
    assert b.samples == [(1, 2, 3, 4, 5, 6), (-1, -2, -3, -4, -5, -6)]


def test_overflow_flag_is_surfaced():
    batches, _ = decode_batches(build_frame(0, [(0,) * 6], overflow=True))
    assert batches[0].overflow is True


def test_partial_frame_is_returned_as_leftover():
    """Serial reads land mid-frame constantly. The decoder must hold the
    remainder rather than discarding or misparsing it."""
    frame = build_frame(1, [(1, 2, 3, 4, 5, 6)])
    batches, leftover = decode_batches(frame[:-3])
    assert batches == []
    assert leftover == frame[:-3]


def test_resynchronises_after_leading_garbage():
    """A capture started mid-stream begins with a partial frame. The decoder
    must find the next sync word rather than giving up."""
    frame = build_frame(2, [(9, 8, 7, 6, 5, 4)])
    batches, _ = decode_batches(b"\x11\x22\x33" + frame)
    assert len(batches) == 1
    assert batches[0].first_seq == 2


def test_bad_checksum_is_rejected_and_stream_resynchronises():
    frame = bytearray(build_frame(3, [(1, 1, 1, 1, 1, 1)]))
    frame[-1] ^= 0xFF
    good = build_frame(4, [(2, 2, 2, 2, 2, 2)])
    batches, _ = decode_batches(bytes(frame) + good)
    assert [b.first_seq for b in batches] == [4]


def test_two_batches_in_one_read():
    payload = build_frame(0, [(1,) * 6]) + build_frame(1, [(2,) * 6])
    batches, leftover = decode_batches(payload)
    assert [b.first_seq for b in batches] == [0, 1]
    assert leftover == b""


def test_csv_line_parses_to_seq_and_counts():
    assert parse_csv_line("42,1,-2,3,-4,5,-6") == (42, (1, -2, 3, -4, 5, -6))


def test_csv_comment_and_blank_lines_are_ignored():
    assert parse_csv_line("# measured ODR 501.3 Hz") is None
    assert parse_csv_line("") is None
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd analysis && uv run pytest tests/test_capture.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'tools'`

- [ ] **Step 4: Create the package marker so `tools` is importable**

```bash
mkdir -p analysis/tools
touch analysis/tools/__init__.py
```

- [ ] **Step 5: Implement the decoder in `analysis/tools/capture.py`**

```python
"""Host-side capture for the IMU streaming instrument.

Decodes either wire format, verifies sequence continuity, and writes raw int16
samples to disk. Scale factors are imported from `plumb.sensor` rather than
redefined -- counts-to-physical is defined in exactly one place across the whole
project, firmware included.
"""

import struct
from dataclasses import dataclass

SYNC = b"\xa5\x5a"
HEADER = struct.Struct("<IBBI")     # first_seq, count, flags, drain_micros
SAMPLE = struct.Struct("<6h")
FLAG_OVERFLOW = 0x01


@dataclass
class Batch:
    first_seq: int
    overflow: bool
    drain_micros: int
    samples: list[tuple[int, int, int, int, int, int]]


def decode_batches(buffer: bytes) -> tuple[list[Batch], bytes]:
    """Decode every complete frame in `buffer`.

    Returns the batches and any trailing bytes that did not form a complete
    frame. Serial reads land mid-frame constantly, so the caller feeds the
    leftover back in with the next read.
    """
    batches: list[Batch] = []
    i = 0
    while True:
        start = buffer.find(SYNC, i)
        if start < 0:
            # No sync ahead. Keep only a possible split sync word.
            return batches, buffer[max(i, len(buffer) - 1):]

        body = start + len(SYNC)
        if len(buffer) < body + HEADER.size:
            return batches, buffer[start:]

        first_seq, count, flags, drain_micros = HEADER.unpack_from(buffer, body)
        end = body + HEADER.size + count * SAMPLE.size + 1
        if len(buffer) < end:
            return batches, buffer[start:]

        payload = buffer[body:end - 1]
        checksum = 0
        for b in payload:
            checksum ^= b

        if checksum != buffer[end - 1]:
            # Corrupt frame. Step past this sync word and look for the next one.
            i = body
            continue

        offset = body + HEADER.size
        samples = [SAMPLE.unpack_from(buffer, offset + n * SAMPLE.size)
                   for n in range(count)]
        batches.append(Batch(first_seq, bool(flags & FLAG_OVERFLOW),
                             drain_micros, samples))
        i = end


def parse_csv_line(line: str):
    """Returns (seq, counts) or None for comments, banners and blank lines."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) != 7:
        return None
    try:
        values = [int(p) for p in parts]
    except ValueError:
        return None
    return values[0], tuple(values[1:])
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cd analysis && uv run pytest tests/test_capture.py -v`
Expected: 8 passed

- [ ] **Step 7: Confirm the whole suite still passes**

Run: `cd analysis && uv run pytest -q`
Expected: 73 passed (65 existing plus 8 new)

- [ ] **Step 8: Commit**

```bash
git add analysis/tools/ analysis/tests/test_capture.py analysis/pyproject.toml analysis/uv.lock
git commit -m "Add host decoder for the IMU wire format"
```

---

## Task 2: Host capture CLI and gap detection

**Files:**
- Modify: `analysis/tools/capture.py`
- Modify: `analysis/tests/test_capture.py`

- [ ] **Step 1: Write the failing tests**

```python
# append to analysis/tests/test_capture.py

from tools.capture import CaptureStats, accumulate


def test_contiguous_batches_report_no_gaps():
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 100, [(0,) * 6] * 4))
    accumulate(stats, Batch(4, False, 200, [(0,) * 6] * 4))
    assert stats.samples == 8
    assert stats.dropped == 0
    assert stats.overflows == 0


def test_a_sequence_gap_is_counted_not_hidden():
    """A dropped sample breaks the time base silently -- nothing downstream can
    detect it afterwards. The capture must count it at the moment it happens."""
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 100, [(0,) * 6] * 4))
    accumulate(stats, Batch(10, False, 200, [(0,) * 6] * 4))
    assert stats.dropped == 6


def test_overflow_flags_are_counted():
    stats = CaptureStats()
    accumulate(stats, Batch(0, True, 100, [(0,) * 6]))
    assert stats.overflows == 1


def test_measured_odr_uses_first_and_last_drain_timestamps():
    """Nominal rate is not trusted. A real rate of 502 Hz against an assumed 500
    puts a 0.4 percent scale error into every integrated angle."""
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 0, [(0,) * 6] * 500))
    accumulate(stats, Batch(500, False, 1_000_000, [(0,) * 6] * 500))
    assert stats.measured_odr_hz() == pytest.approx(500.0, rel=1e-3)


def test_measured_odr_is_none_before_two_batches():
    assert CaptureStats().measured_odr_hz() is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd analysis && uv run pytest tests/test_capture.py -k "gap or overflow or odr or contiguous" -v`
Expected: FAIL — `ImportError: cannot import name 'CaptureStats'`

- [ ] **Step 3: Implement accumulation, then the CLI**

Append to `analysis/tools/capture.py`:

```python
@dataclass
class CaptureStats:
    samples: int = 0
    dropped: int = 0
    overflows: int = 0
    batches: int = 0
    first_micros: int | None = None
    last_micros: int | None = None
    first_batch_count: int = 0
    _next_seq: int | None = None

    def measured_odr_hz(self) -> float | None:
        """Samples per second, from the sensor's own delivery rate.

        Drain timestamps are the only clock available, and over a long window
        the drain rate equals the sample rate exactly -- the FIFO cannot produce
        samples faster or slower than the sensor makes them. Over a short window
        it is meaningless, which is why two batches is the minimum and Task 8
        asks for sixty seconds.

        The first batch's samples are excluded, and this is not an off-by-one to
        tidy away: the elapsed window runs from the FIRST drain to the LAST, so
        the samples that arrived AT the first drain were taken before the window
        opened. Counting them inflates the rate by a whole batch. At a 64-sample
        batch over 60 seconds the error is small; over a two-batch test it is
        100 percent, which is why the test uses exactly two batches.
        """
        if self.batches < 2 or self.first_micros is None:
            return None
        elapsed = (self.last_micros - self.first_micros) / 1e6
        if elapsed <= 0:
            return None
        return (self.samples - self.first_batch_count) / elapsed


def accumulate(stats: CaptureStats, batch: Batch) -> None:
    if stats._next_seq is not None and batch.first_seq > stats._next_seq:
        stats.dropped += batch.first_seq - stats._next_seq
    stats._next_seq = batch.first_seq + len(batch.samples)

    if stats.batches == 0:
        stats.first_batch_count = len(batch.samples)
        stats.first_micros = batch.drain_micros

    stats.samples += len(batch.samples)
    stats.batches += 1
    if batch.overflow:
        stats.overflows += 1
    stats.last_micros = batch.drain_micros
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd analysis && uv run pytest tests/test_capture.py -v`
Expected: 13 passed

- [ ] **Step 5: Add the CLI**

Append to `analysis/tools/capture.py`:

```python
def main() -> None:
    import argparse
    import sys

    import serial

    from plumb.sensor import FullScale

    ap = argparse.ArgumentParser(description="Capture IMU samples from the board.")
    ap.add_argument("--port", required=True, help="e.g. COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", default="capture.npy")
    args = ap.parse_args()

    fs = FullScale()
    print(f"# gyro {fs.gyro_dps} dps FS, accel {fs.accel_g} g FS", file=sys.stderr)

    stats = CaptureStats()
    rows: list[tuple[int, ...]] = []
    leftover = b""

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        import time
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            leftover += port.read(4096)
            batches, leftover = decode_batches(leftover)
            for batch in batches:
                accumulate(stats, batch)
                rows.extend(batch.samples)

    import numpy as np
    np.save(args.out, np.array(rows, dtype=np.int16))

    odr = stats.measured_odr_hz()
    print(f"samples   : {stats.samples}")
    print(f"batches   : {stats.batches}")
    print(f"dropped   : {stats.dropped}")
    print(f"overflows : {stats.overflows}")
    print(f"measured ODR: {odr:.2f} Hz" if odr else "measured ODR: not enough data")
    print(f"written   : {args.out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Confirm the suite passes and the CLI at least starts**

Run: `cd analysis && uv run pytest -q`
Expected: 78 passed

Run: `cd analysis && uv run python -m tools.capture --help`
Expected: argparse usage text, exit 0

- [ ] **Step 7: Commit**

```bash
git add analysis/tools/capture.py analysis/tests/test_capture.py
git commit -m "Add capture CLI with sequence-gap and ODR measurement"
```

---

## Task 3: QMI8658 driver — configuration with read-back verification

Firmware begins here. Verified by compiling, and by the read-back check that Step 3 builds in.

**Files:**
- Create: `firmware/bringup-arduino/imu_stream/qmi8658.h`
- Create: `firmware/bringup-arduino/imu_stream/qmi8658.cpp`

- [ ] **Step 1: Read the datasheet and record the constants**

Find the QMI8658 datasheet. Record, as named constants in `qmi8658.cpp`:

- Control register addresses for accelerometer config, gyroscope config, and the FIFO
- The encoding for gyro full scale **±256 dps** and accel full scale **±16 g** (§6.4, amended: the QMI8658 table is powers of two and has no ±250 setting)
- The ODR encoding table, and specifically: the value nearest **500 Hz**, and the **maximum** supported
- FIFO depth, watermark configuration, and the FIFO status/overflow register
- Which register enables the FIFO watermark interrupt, and on which INT pin

Do NOT proceed on guessed values. If the datasheet is ambiguous about a field, say so in your report rather than picking an interpretation.

- [ ] **Step 2: Write `firmware/bringup-arduino/imu_stream/qmi8658.h`**

```cpp
// Register-level driver for the QMI8658 6-axis IMU.
//
// Knows nothing about serial ports or output formats. That boundary is what
// makes the eventual ESP-IDF port a translation rather than a rewrite.

#pragma once

#include <Arduino.h>

enum class Rate {
  Stroke,   // nearest supported ODR to 500 Hz -- stroke capture, noise floor
  Max,      // highest supported ODR -- tap test (spec 5.5 needs > 1 kHz)
};

struct Sample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct DrainResult {
  uint8_t count;      // samples written to the caller's buffer
  bool overflow;      // FIFO overflowed since the previous drain
};

namespace qmi8658 {

// Returns false if WHO_AM_I does not read back 0x05, or if any configuration
// register fails to read back the value that was written.
bool begin(Rate rate);

// Reconfigure the output data rate. Same read-back guarantee as begin().
bool setRate(Rate rate);

// True when the FIFO has reached its watermark.
bool dataReady();

// Drains up to `capacity` samples. Never blocks.
DrainResult drain(Sample* out, uint8_t capacity);

// Nominal rate in Hz for the configured mode, from the datasheet table.
// The instrument measures the real rate anyway -- see the plan preamble.
float nominalRateHz();

}  // namespace qmi8658
```

- [ ] **Step 3: Implement `qmi8658.cpp` with read-back verification**

The structure is fixed; the register values come from Step 1.

```cpp
#include "qmi8658.h"

#include <Wire.h>

namespace {

constexpr uint8_t kAddress = 0x6B;      // confirmed on hardware by smoke_test
constexpr uint8_t kRegWhoAmI = 0x00;    // confirmed: returns 0x05
constexpr uint8_t kWhoAmIValue = 0x05;

// --- FROM DATASHEET, Step 1 -------------------------------------------
// Replace each of these with the real value and delete this comment block.
// Every one of them is read back and verified in writeVerified() below, so a
// wrong address or encoding fails loudly at begin() rather than producing a
// device that streams plausible garbage.
constexpr uint8_t kRegCtrlAccel = 0x00;   // TODO Step 1
constexpr uint8_t kRegCtrlGyro  = 0x00;   // TODO Step 1
constexpr uint8_t kRegCtrlFifo  = 0x00;   // TODO Step 1
constexpr uint8_t kRegFifoStatus = 0x00;  // TODO Step 1
constexpr uint8_t kRegFifoData  = 0x00;   // TODO Step 1
// ----------------------------------------------------------------------

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(kAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)kAddress, 1) != 1) return false;
  out = Wire.read();
  return true;
}

// Write, then read back and compare. A mistyped register address or a
// misread encoding table is the most likely failure in this whole module, and
// it is silent without this check -- the device simply runs at a rate nobody
// intended.
bool writeVerified(uint8_t reg, uint8_t value) {
  if (!writeReg(reg, value)) return false;
  uint8_t back = 0;
  if (!readReg(reg, back)) return false;
  return back == value;
}

}  // namespace
```

Then implement `begin`, `setRate`, `dataReady`, `drain` and `nominalRateHz` against the constants from Step 1, using `writeVerified` for every configuration write.

`drain` reads the FIFO status register, computes how many samples are available, burst-reads them, and converts each 12-byte group into a `Sample`. It must clear or acknowledge the overflow flag so the next drain reports fresh state.

- [ ] **Step 4: Verify it compiles**

Run:

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=enabled,CDCOnBoot=default" firmware/bringup-arduino/imu_stream
```

Expected: compiles. It will fail until Task 5 adds `imu_stream.ino`; if so, create a minimal `.ino` with empty `setup()`/`loop()` now and let Task 5 replace it.

- [ ] **Step 5: Commit**

```bash
git add firmware/bringup-arduino/imu_stream/qmi8658.h firmware/bringup-arduino/imu_stream/qmi8658.cpp
git commit -m "Add QMI8658 driver with configuration read-back"
```

---

## Task 4: Output formatting

**Files:**
- Create: `firmware/bringup-arduino/imu_stream/framing.h`
- Create: `firmware/bringup-arduino/imu_stream/framing.cpp`

- [ ] **Step 1: Write `framing.h`**

```cpp
// Wire formats for the IMU streaming instrument.
//
// The binary layout is pinned in the implementation plan and implemented twice:
// here, and in analysis/tools/capture.py. Keep them in step.

#pragma once

#include <Arduino.h>

#include "qmi8658.h"

enum class Format { Csv, Binary };

namespace stream {

// One CSV line per sample: seq,ax,ay,az,gx,gy,gz
void emitCsv(Stream& out, uint32_t firstSeq, const Sample* samples, uint8_t count);

// One framed batch. Layout is in the plan: sync, header, samples, XOR checksum.
void emitBinary(Stream& out, uint32_t firstSeq, const Sample* samples,
                uint8_t count, bool overflow, uint32_t drainMicros);

}  // namespace stream
```

- [ ] **Step 2: Implement `framing.cpp`**

```cpp
#include "framing.h"

namespace {
constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kFlagOverflow = 0x01;

// Accumulate the checksum over everything after the sync word, exactly as
// capture.py does. Writing the bytes and the checksum in one pass keeps the two
// implementations from drifting.
struct ChecksumWriter {
  Stream& out;
  uint8_t sum = 0;
  void write(uint8_t b) { out.write(b); sum ^= b; }
  void write32(uint32_t v) {
    write(v & 0xFF); write((v >> 8) & 0xFF);
    write((v >> 16) & 0xFF); write((v >> 24) & 0xFF);
  }
  void write16(int16_t v) {
    write((uint16_t)v & 0xFF); write(((uint16_t)v >> 8) & 0xFF);
  }
};
}  // namespace

void stream::emitCsv(Stream& out, uint32_t firstSeq, const Sample* s, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    out.printf("%lu,%d,%d,%d,%d,%d,%d\n", (unsigned long)(firstSeq + i),
               s[i].ax, s[i].ay, s[i].az, s[i].gx, s[i].gy, s[i].gz);
  }
}

void stream::emitBinary(Stream& out, uint32_t firstSeq, const Sample* s,
                        uint8_t count, bool overflow, uint32_t drainMicros) {
  out.write(kSync0);
  out.write(kSync1);

  ChecksumWriter w{out};
  w.write32(firstSeq);
  w.write(count);
  w.write(overflow ? kFlagOverflow : 0);
  w.write32(drainMicros);
  for (uint8_t i = 0; i < count; i++) {
    w.write16(s[i].ax); w.write16(s[i].ay); w.write16(s[i].az);
    w.write16(s[i].gx); w.write16(s[i].gy); w.write16(s[i].gz);
  }
  out.write(w.sum);
}
```

- [ ] **Step 3: Verify it compiles**

Run the `arduino-cli compile` command from Task 3 Step 4.
Expected: compiles clean.

- [ ] **Step 4: Commit**

```bash
git add firmware/bringup-arduino/imu_stream/framing.h firmware/bringup-arduino/imu_stream/framing.cpp
git commit -m "Add CSV and binary output framing"
```

---

## Task 5: Sketch — main loop and serial control

**Files:**
- Create: `firmware/bringup-arduino/imu_stream/imu_stream.ino`

- [ ] **Step 1: Write the sketch**

```cpp
// Plumb IMU streaming instrument.
//
// Measures what the synthetic harness cannot: sensor axes and signs, the real
// sample rate, the gyro noise floor, and whether the grip mount is rigid
// (spec 5.5). Not product firmware.
//
// Commands, single characters over serial:
//   c  CSV output          b  binary output
//   1  stroke rate         9  maximum rate
//   s  start / stop        ?  status

#include <Wire.h>

#include "qmi8658.h"
#include "framing.h"

static const int PIN_SDA = 6;   // spec 4.2
static const int PIN_SCL = 7;
static const uint32_t I2C_HZ = 400000;

static const uint8_t BATCH_MAX = 64;

static Sample batch[BATCH_MAX];
static Format format = Format::Csv;
static Rate rate = Rate::Stroke;
static bool streaming = false;

static uint32_t seq = 0;
static uint32_t overflowCount = 0;
static uint32_t startMicros = 0;

static void printStatus() {
  const uint32_t elapsed = micros() - startMicros;
  Serial.printf("# format=%s rate=%s(%.1f Hz nominal) streaming=%d\n",
                format == Format::Csv ? "csv" : "binary",
                rate == Rate::Stroke ? "stroke" : "max",
                qmi8658::nominalRateHz(), streaming);
  Serial.printf("# samples=%lu overflows=%lu elapsed_us=%lu\n",
                (unsigned long)seq, (unsigned long)overflowCount,
                (unsigned long)elapsed);
  if (streaming && elapsed > 0) {
    Serial.printf("# measured_odr=%.2f Hz\n", seq * 1e6f / elapsed);
  }
}

static void handleCommand(char c) {
  switch (c) {
    case 'c': format = Format::Csv;    Serial.println("# format csv"); break;
    case 'b': format = Format::Binary; Serial.println("# format binary"); break;
    case '1': rate = Rate::Stroke; qmi8658::setRate(rate); Serial.println("# rate stroke"); break;
    case '9': rate = Rate::Max;    qmi8658::setRate(rate); Serial.println("# rate max"); break;
    case 's':
      streaming = !streaming;
      seq = 0;
      overflowCount = 0;
      startMicros = micros();
      Serial.printf("# streaming %d\n", streaming);
      break;
    case '?': printStatus(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(921600);
  delay(2000);
  Serial.println("\n# Plumb IMU streaming instrument");

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  if (!qmi8658::begin(rate)) {
    Serial.println("# FATAL: QMI8658 init or config read-back failed");
    while (true) delay(1000);
  }
  Serial.println("# ready -- press s to start, ? for status");
}

void loop() {
  while (Serial.available()) handleCommand((char)Serial.read());

  if (!streaming || !qmi8658::dataReady()) return;

  const uint32_t drainMicros = micros();
  const DrainResult r = qmi8658::drain(batch, BATCH_MAX);
  if (r.count == 0) return;
  if (r.overflow) overflowCount++;

  if (format == Format::Csv) {
    stream::emitCsv(Serial, seq, batch, r.count);
  } else {
    stream::emitBinary(Serial, seq, batch, r.count, r.overflow, drainMicros);
  }
  seq += r.count;
}
```

- [ ] **Step 2: Verify it compiles**

Run the `arduino-cli compile` command from Task 3 Step 4.
Expected: compiles clean; note the reported flash and RAM usage.

- [ ] **Step 3: Commit**

```bash
git add firmware/bringup-arduino/imu_stream/imu_stream.ino
git commit -m "Add IMU streaming sketch with serial control"
```

---

## Task 6: Create the results document

**Files:**
- Create: `docs/bringup-results.md`

- [ ] **Step 1: Write the skeleton**

```markdown
# Bring-up measurements

Measured on hardware. Nothing here is estimated, and nothing is copied from a
datasheet — every number is something the board actually produced.

| Date | Firmware | Board |
|---|---|---|
| | `imu_stream` | Waveshare ESP32-S3-Touch-LCD-1.28 |

## Axes and signs (§9.1)

## Dropped samples (§9.2)

## Measured ODR (§9.3)

## Resting gyro noise (§9.4)

## Tap test ringdown (§9.5)
```

- [ ] **Step 2: Commit**

```bash
git add docs/bringup-results.md
git commit -m "Add skeleton for bring-up measurements"
```

---

## STOP — Tasks 7 to 10 require hardware

An agent executing this plan should stop here and hand back. The remaining tasks need the board connected, and Task 10 needs a putter with a printed base fitted.

---

## Task 7: Verify axes and signs (§9.1)

- [ ] Flash `imu_stream` and open the serial monitor at **921600** — the sketch raises the baud rate from the smoke test's 115200, so the dropdown needs changing. Press `c` then `s`.
- [ ] Hold the board with its USB port facing you. Rotate it about each axis in turn, slowly.
- [ ] For each axis, record which gyro channel responds and with what sign.
- [ ] Confirm against the body frame the algorithm assumes: **Z along the shaft, pointing from head to butt; X the face normal.** If the mapping differs, that is not a bug — it is the sensor's physical orientation on the board, and it gets recorded as the fixed rotation between sensor axes and the frame, not corrected by editing the algorithm.
- [ ] Write the mapping into `docs/bringup-results.md` and commit.

## Task 8: Dropped samples and measured ODR (§9.2, §9.3)

- [ ] Press `b` then `s`. Run: `cd analysis && uv run python -m tools.capture --port COM4 --seconds 60 --out stroke_rate.npy`
- [ ] Record `dropped`, `overflows` and `measured ODR`. **Dropped and overflows must both be zero.** If not, the batch size or the watermark is wrong — report it rather than lowering the rate.
- [ ] Repeat at maximum rate: press `9`, then capture again to `max_rate.npy`.
- [ ] Record both measured ODRs against nominal in `docs/bringup-results.md`, with the percentage difference. If either differs from nominal by more than about 1%, say so explicitly — that error goes straight into every integrated angle, and `SAMPLE_RATE_HZ` in the harness needs updating to the measured value.

## Task 9: Resting gyro noise floor (§9.4)

This is the most consequential measurement the instrument produces.

- [ ] Rest the board on a solid surface, away from foot traffic. Press `1` (stroke rate), `b`, `s`.
- [ ] Capture 60 seconds to `rest.npy`.
- [ ] Compute per-axis standard deviation in dps:

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

- [ ] Record the worst-axis σ. **Compare it against `Thresholds.stillness_gyro_std_rad`, currently 0.8 dps.** The harness showed detection fails — not degrades, fails — once noise approaches that threshold. If measured σ is within a factor of about 3 of it, the threshold needs raising and the tempo accuracy that depends on a low onset gate needs rechecking.
- [ ] Record in `docs/bringup-results.md` and commit.

## Task 10: Tap test ringdown (§9.5, §5.5)

Requires a putter with a printed base fitted, so it is gated on the mechanical work.

- [ ] Mount the puck. Press `9` (max rate), `b`, `s`.
- [ ] Strike the putter head with a **soft** mallet. Capture the ringdown.
- [ ] FFT the accelerometer channels and find the dominant resonance.
- [ ] Apply the §5.5 gate: **above ~500 Hz → mount is effectively rigid, proceed. Below ~200 Hz → unacceptably compliant, escalate** to the step-drill and expanding-collet path before any further firmware work.
- [ ] Record the spectrum, the resonant frequency, and the verdict in `docs/bringup-results.md`.

---

## Notes for the implementer

**Do not guess a register address.** Task 3's read-back verification exists because a wrong constant otherwise produces a device that initialises cleanly and streams data at a rate nobody chose. If the datasheet is ambiguous, report it.

**Do not lower the sample rate to make dropped samples go away.** A dropped sample breaks the time base silently. If the FIFO overflows, the watermark or batch size is wrong — that is the finding.

**`SAMPLE_RATE_HZ = 500.0` in `analysis/plumb/trajectory.py` is an assumption until Task 8 measures it.** If the real ODR differs, update that constant and re-run the harness; the recovery numbers in `analysis/README.md` were computed under the assumption and will shift.
