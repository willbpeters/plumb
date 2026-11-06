# On-device stroke logging — design

**Date:** 2026-09-29
**Parent spec:** `2026-09-15-putting-analyzer-design.md` (§11, amended by this design; §6.2,
§6.3; invariants 5, 7, 8)
**Roadmap:** `docs/roadmap.md`, week 6 code track ("§11 logging and a USB dump tool")
**Status:** approved with Will, 2026-09-29 (record sessions; raw partition, not LittleFS)

## Goal

The corpus that Phase 2 derives every threshold from (invariant 5) has to be recorded on the
putter, untethered. A USB cable hanging off the grip is exactly the mechanical coupling open
defect 5 warns about. This builds the recorder, the format, and the host side that reads
it back, without the board present. Only the measurements that need the board are left for it.

**Done means:**

1. A C recorder (`components/rec`, pure C) and a Python decoder (`plumb/corpus.py`),
   written independently from this document, agree bit for bit. The simulator's int16 samples
   go through the C and come back out of the Python unchanged, including sample gaps,
   full-scale jumps, a partition that fills, and a session torn by power loss.
2. The compression ratio, measured on simulated strokes and printed.
3. Firmware builds with the recorder wired in: console control, a record-from-boot flag, and
   the partition table.
4. Parent spec §11 amended.

**Left for the board:** whether a page program loses samples (the sensor's counter will
say), the real compression ratio, and the erase time.

## Two decisions, both Will's (2026-09-29)

**Record sessions, not strokes.** Recording is started and stopped by the golfer, and
everything in between is kept. Nothing on the device decides what a stroke is, so the corpus
carries no trigger bias. Invariant 5 needs that: thresholds are derived *from* the corpus, so
the corpus cannot be selected *by* a threshold. Strokes are cut out offline, and re-cut
whenever the algorithm changes. Per-stroke records (§11 as written) are the right thing for
the product later. They are not the right thing for Phase 2.

**A raw partition, not LittleFS.** On the ESP32-S3 a flash write or erase disables the cache
on both cores, so nothing in flash runs, including the acquisition task. The IMU is read
directly, one sample per ~1.1 ms, with no FIFO behind it. A sector erase (tens of ms) loses
dozens of samples. LittleFS erases on its own schedule during writes. An append-only log on a
raw partition erases only when told to (after a dump, at rest), and while recording it only
ever programs pages. Wear levelling buys nothing here: the project will cycle the partition
tens of times against NOR's ~100,000.

## Storage: condensed, lossless

Every metric reads both sensors. Face angle needs the accelerometer for `g₀`, tempo needs it
for the impact spike, and speed needs it for the pivot. So no channel can be dropped, and a
~4-sample spike rules out decimating one. Condensing is therefore lossless compression, which
also keeps CLAUDE.md's "raw samples, never derived metrics".

Per block of up to 64 samples, per axis: the first value raw, then each difference from the
previous sample, zigzag-mapped to unsigned, Rice-coded with the parameter `k` that makes that
axis's block smallest. Measured on the simulator at the board's noise floor (0.28 dps,
0.02 m/s²): 2.5× from the codes alone, estimated before this design. The built encoder
measured **2.32×** on a noisy stroke, headers and CRCs included (6.1× on a noiseless one,
which shows what the noise costs). Gyro ~7.9 bits per axis-sample, almost all sensor noise;
accelerometer ~4.9. Real room vibration will likely make it worse. Dropping gyro bits
under the noise floor would reach ~3.9×, but it is lossy and was not chosen.

**The escape.** A difference can be as large as ±65,535 (full scale to full scale). Its zigzag
value is then up to 131,070, too long to write in unary. A quotient `q = z >> k` of 24 or more
is written as 24 one-bits and then `z` in 17 raw bits.

## Format (version 1)

Little-endian. CRC-32 is zlib's (`binascii.crc32`), over every byte of the record before the
CRC.

**Session record**, always starting on a 256-byte page boundary:

| Offset | Type | Field |
|---|---|---|
| 0 | u32 | magic `0x534D4C50` ("PLMS") |
| 4 | u16 | format version, 1 |
| 6 | u16 | record length in bytes, 100 |
| 8 | u32 | session number, 1 + the sessions already in the log |
| 12 | u32 | esp_timer at start, ms since boot (low 32 bits). There is no clock on the device; the dump adds wall time. |
| 16 | f64 | measured sample rate, Hz (`acq/rate.h`) |
| 24 | f64 | its standard error, Hz |
| 32 | u16, u16 | gyro full scale (dps), accelerometer full scale (g) |
| 36 | u8 | putter profile ID (0 until profiles exist) |
| 37 | u8 | 1 if the accelerometer calibration below is real, 0 if it is identity |
| 38 | u16 | reserved, 0 |
| 40 | f32 × 6 | accelerometer offset (m/s²) ×3, then gain ×3 (`plumb/calibration.py`) |
| 64 | char × 32 | firmware version (`esp_app_desc_t.version`), NUL-padded |
| 96 | u32 | CRC-32 |

**Block record**, directly after the session record or the previous block:

| Offset | Type | Field |
|---|---|---|
| 0 | u16 | magic `0x4B42` ("BK") |
| 2 | u16 | payload length in bytes |
| 4 | u32 | unwrapped sensor sample index of the first sample (`acq_seqcount`) |
| 8 | u8 | sample count, 1..64 |
| 9 | u8 | reserved, 0 |
| 10 | u8 × 6 | Rice `k` per axis, 0..16 |
| 16 | i16 × 6 | first sample, raw |
| 28 | bits | payload |
| 28 + payload | u32 | CRC-32 |

Axis order everywhere is the wire frame's (`acq/frame.h`): accelerometer X, Y, Z, then gyro
X, Y, Z. In the payload the axes follow one another. For each axis in turn come `count − 1`
codes: `q` one-bits, a zero bit, then `k` low bits, MSB first. The payload is zero-padded to
a byte.

**Samples inside a block are consecutive.** An index gap closes the block, and the next one
starts at the new index. The gap is therefore recorded exactly, whatever caused it: the
acquisition ring overflowing, a flash stall, or the recorder falling behind and refusing
samples. It is the single signal for loss, and it is the sensor's own count, not ours
(bring-up §9.2 is why that matters).

**Reading a log.** Walk records from offset 0. When a record's magic or CRC fails, or the
next two bytes are `0xFFFF`, skip to the next page boundary and look for a session magic.
When everything left is `0xFF`, the log has ended. Power lost mid-write leaves a torn record
at the end of a session. It fails its CRC, and so does nothing after it, until the next
session's page-aligned header.

## Components

```
acq ring ─► stream_service (app task, core 1) ─┬─► stroke_feed (unchanged)
                                               └─► rec_port_feed ─► rec_log ─► flash
```

- **`components/rec`**: pure C, no ESP-IDF, host-built like `acq` and `plumb`.
  - `rec/block.h`: encode one block into bytes. Stateless.
  - `rec/log.h`: the log. It opens on a flash described by three callbacks (read, write,
    erase), finds the end, and starts and stops sessions. It feeds samples, stages the encoded
    bytes in a 4 KB buffer, and writes at most one aligned 256-byte page per service call, so
    no single cache-disabled stall exceeds one page program. Opening finds the end by a
    binary search over 4 KB sectors for the first fully erased one, then scans back for the
    last non-`0xFF` byte and rounds up to a page. A full log stops the session and says so. It
    never wraps.
  - `rec/crc32.h`.
- **`main/rec_port.c`**: the ESP-IDF glue. It finds the `corpus` partition and supplies the
  callbacks, the NVS record-from-boot flag, the console commands and the status line.
  Session start takes the measured sample rate, so recording cannot start before the rate
  exists, exactly as the pipeline waits for it.
  - Console: `l` starts or stops a session. `e` asks to erase and `Y` confirms. It runs in
    64 KB steps from the app loop, so the UI stays alive, and it is refused while recording.
    `B` toggles record-from-boot, which starts a session as soon as the rate is measured. `?`
    adds the used bytes and the current session's samples, gaps, and compression ratio.
  - No on-screen control yet. Without a battery the board is on USB anyway. A long-press and a
    REC mark are for when it is not.
- **`partitions.csv`**: nvs, phy_init, a 2 MB factory app, and `corpus` (data, subtype 0x40)
  from 0x210000 to the end of the 16 MB flash, 14.5 MB. That is about 22 minutes raw and
  ~50 at the measured simulated ratio.
- **`analysis/plumb/corpus.py`**: the decoder. It produces sessions, each with its header
  fields and its runs of consecutive samples (`first_index`, `accel_counts`, `gyro_counts` as
  (n, 3) int16), plus the lost-sample count between runs.
- **`analysis/tools/rec_dump.py`**: `esptool read_flash` on the partition in 1 MB pieces,
  stopping after a piece that is all `0xFF`. It writes the image with the wall time of the
  dump beside it under `data/`, and prints a summary. It is thin by design, and the decoder is
  where the tests are.

The recorder and the pipeline run side by side on core 1. Recording changes nothing about
what the pipeline sees or shows.

## Tests (host, no board)

In `analysis/tests/test_rec.py`. The C is built by `tools/cbuild.py` into
`firmware/test/reccheck`, which runs `rec_log` on a RAM flash:

- Round trip: simulated strokes → C → Python gives identical int16 samples and indices. Also
  noisy and noiseless streams, one sample, exactly 64, 65.
- Gaps of 1, 2 and 1,000 samples: runs split where they should, and the lost counts are exact.
- Full-scale steps of ±65,535 exercise the escape.
- Multiple sessions: numbering and page alignment.
- A full partition: the session stops and everything written before it decodes.
- A torn write: the RAM flash stops accepting bytes partway through a page. It is then
  reopened, and a second session is recorded after it. The first session decodes up to the
  last whole block, and the second decodes completely.
- The erased-sector search on open finds the same end as a linear scan.
- The ratio on simulated strokes is printed and bounded loosely (> 1.5×), to catch a broken
  `k` choice rather than to hold a number.

Firmware: `idf.py build` must succeed. The invariant-6 map check still passes.

## Out of scope

Cutting strokes out of sessions (the Phase 2 notebook), on-screen controls, per-stroke
records for the product, putter profiles (the ID field is reserved at 0), storing the
accelerometer calibration in NVS (the header has the field and the flag), and moving the
acquisition path into IRAM. That last one happens only if the board shows page programs losing
samples.
