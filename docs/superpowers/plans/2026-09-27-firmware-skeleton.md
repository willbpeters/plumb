# ESP-IDF Firmware Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The real product firmware on ESP-IDF v5.5.5: IMU sampled on core 0 from the sensor's own
data-ready line with zero loss, the plumb_ui screens on core 1, and invariant 8 enforced by a gate,
with loss, jitter and the absence of radio code measured.

**Architecture:** A pure-C acquisition core (ring buffer, sequence accounting, jitter histogram,
wire framing) is proven on the host against independent references first. ESP-IDF drivers for
the IMU, touch and panel sit in a `board` component. An `acq` task pinned to core 0 wakes on
the DRDY edge, reads a SyncSample-locked sample and pushes it to the ring. The app task on core 1
drains the ring, streams it in `imu_stream`'s framing (so `capture.py` works unchanged), keeps the
statistics, and runs LVGL only while the gate is open.

**Tech Stack:** ESP-IDF v5.5.5 (`esp_driver_i2c` master API, `esp_lcd`, `esp_driver_gpio`,
`esp_driver_uart`, `esp_timer`), FreeRTOS, LVGL 9.6.0 (submodule), C99; host tests with pytest and
MSVC through `analysis/tools/cbuild.py`.

**Spec:** `docs/superpowers/specs/2026-09-27-firmware-skeleton-design.md`.

---

## Decisions this plan makes that the spec left open

These are recorded here and repeated in the spec in Task 11.

1. **Console letters.** The spec gives `d` to "disarm gate", but `tools/board.py` sends `d` to
   select the direct read path, and the spec also requires `capture.py` to work unchanged. So
   the gate is `a` (arm) and `o` (open). The firmware also accepts `1` and `d`: they acknowledge
   the only rate and read path it has. `9` and `f` are refused out loud.
2. **The animating condition** for the jitter measurement is `x`: it cycles the result screens
   every 100 ms, a full-screen redraw each time. That is the heaviest load the UI can generate,
   and it is reproducible.
3. **LVGL's Kconfig is ignored** (`LV_KCONFIG_IGNORE`), so the device compiles LVGL from exactly
   the `lv_conf.h` that the host bench uses. Otherwise every option that `lv_conf.h` does not
   list would come from Kconfig defaults on the device and from `lv_conf_internal.h` defaults
   on the host, which would be two configurations.
4. **The build contains only what `main` pulls in** (`set(COMPONENTS main)`), so `esp_wifi` and
   `bt` are not even compiled. The map check (Task 8) guards against that changing. It is shown
   to fire once, on a deliberately radio-linked build, before it is trusted.
5. **Flash runs at 80 MHz QIO, not the 120 MHz in parent §6.3.** On the S3, 120 MHz needs
   `SPI_FLASH_HPM_ON` and a flash part that supports it, and the Kconfig help warns of random
   crashes after a ~20 °C temperature change in some modes. Nothing in this skeleton depends on
   flash speed. The disagreement with §6.3 is reported, not silently resolved.
6. **`LV_MEMCPY_MEMSET_STD` (parent §6.3) is an LVGL 8 name.** Its LVGL 9 equivalent lives in
   `lv_conf.h`, which the host shares. It is left for a measured change and reported.

## Register facts this plan depends on

From the QMI8658C datasheet rev A (QST document 13-52-27), re-read for this plan:

- §6.1: in SyncSample mode "FIFO function is not supported in this mode, and DRDY signal will be
  routed to INT2."
- §6.3: DRDY is edge-triggered on INT2 when a sensor is enabled, the FIFO is in bypass and
  `CTRL7.DRDY_DIS` (bit 5) is 0.
- Table 22: `CTRL7` bit 7 is SyncSample, bit 5 DRDY_DIS, bits 1:0 gEN/aEN. **`CTRL1` has no INT2
  enable bit** (bits 4:3 are reserved), so nothing needs setting there for DRDY. `CTRL1.BE`
  (bit 5) is left as the part resets it, with a read-modify-write, exactly as `imu_stream` does:
  that is the setting under which the little-endian unpack was proven on hardware.
- §13.2.1: over I²C, write `CAL1_L` (0x0B) = 0x01, then CTRL9 command 0x12, before using the
  lock.
- §13.2.2: `CTRL7` = 0x83 enables 6DOF in SyncSample mode.
- §13.2.3: reading STATUSINT (0x2D) with Avail (bit 0) set starts the lock. If Locked (bit 1) is
  still 0, wait `Data_Lock_Delay`, which is 6 µs at 896.8 Hz (Table 40). A burst read through
  GZ_H (0x40) releases the lock. **"Once the data sample is locked, new data will be dropped,
  until the release of the locking."** The lock is held for one burst, about 500 µs, against a
  1103 µs period. A read delayed past the next sample would drop it, and the TIMESTAMP counter
  would show the gap. That is what the loss count watches for.

## File structure

```
firmware/
  CMakeLists.txt                     project; LVGL via EXTRA_COMPONENT_DIRS; map check
  sdkconfig.defaults                 the one configuration (invariant 6)
  sdkconfig                          generated, committed: the exact configuration validated
  idf.ps1                            runs idf.py with v5.5.5's environment
  tools/check_no_radio.py            fails the build if radio code is linked
  main/
    CMakeLists.txt
    app_main.c                       boot, app task on core 1, console
    gate.c / gate.h                  the invariant-8 gate
    stream.c / stream.h              drains the ring: accounting, jitter, framing out
    ui_port.c / ui_port.h            LVGL + panel + touch, serviced only while the gate is open
    uart_io.c / uart_io.h            the one UART path, text and binary
  components/
    acq/
      CMakeLists.txt
      include/acq/record.h           one sample as read
      include/acq/ring.h, src/ring.c           SPSC ring (pure C)
      include/acq/seqcount.h, src/seqcount.c   24-bit counter accounting (pure C)
      include/acq/jitter.h, src/jitter.c       1 us histogram, nearest-rank percentiles (pure C)
      include/acq/frame.h, src/frame.c         imu_stream binary framing (pure C)
      include/acq/acq.h, src/acq.c             the core-0 task and DRDY interrupt (IDF)
    board/
      CMakeLists.txt
      include/board/pins.h
      include/board/i2c_bus.h, src/i2c_bus.c
      include/board/qmi8658.h, src/qmi8658.c
      include/board/cst816.h, src/cst816.c
      include/board/gc9a01.h, src/gc9a01.c     moved from bringup-arduino/display
    plumb_ui/CMakeLists.txt          new: IDF registration
  bringup-arduino/display/           gc9a01.* become wrappers onto components/board
  test/acqcheck.c                    host harness for the pure-C acq core
analysis/
  tools/cbuild.py                    build() takes sources and includes; build_acqcheck()
  tools/board_ui.py                  board include path; send --baud
  tests/test_acq.py                  the acq core against NumPy, capture.py, generated truth
  tests/test_invariants.py           invariant 6's map check against a radio map
```

---

### Task 1: Host harness for the acquisition core

The four pure-C modules are what the loss and jitter numbers are computed with. They are proven
on the host first, each against something that is not itself. That means NumPy's percentile for
the histogram, `capture.py`'s decoder for the framing, a generated sequence with known gaps for
the accounting, and a Python deque for the ring.

**Files:**
- Create: `firmware/components/acq/include/acq/record.h`, `ring.h`, `seqcount.h`, `jitter.h`, `frame.h`
- Create: `firmware/components/acq/src/ring.c`, `seqcount.c`, `jitter.c`, `frame.c`
- Create: `firmware/test/acqcheck.c`
- Modify: `analysis/tools/cbuild.py` (`build()` gains `sources` / `includes`; add `build_acqcheck()`)
- Test: `analysis/tests/test_acq.py`

- [ ] **Step 1: Write the failing tests**

`analysis/tests/test_acq.py`:
```python
"""The acquisition core against references that are not itself.

These four pieces of C compute every number the firmware skeleton reports: the
loss count, the jitter percentiles, the ring that carries samples between the
cores, and the frames capture.py reads. A test that checks them against their
own arithmetic proves nothing (CLAUDE.md), so each is checked against an
independent reference: NumPy's percentile, capture.py's decoder, a generated
sample sequence whose gaps are known because they were put there, and a Python
deque.
"""

import subprocess
from collections import deque

import numpy as np
import pytest

from tools import cbuild
from tools.capture import decode_batches

SEED = 20260927
MASK24 = 0xFFFFFF
UNKNOWN = 0xFFFFFFFF
RING_CAPACITY = 512
HIST_BINS = 2048


@pytest.fixture(scope="module")
def acqcheck():
    try:
        return cbuild.build_acqcheck().executable
    except cbuild.CompilerNotFound as missing:
        message = f"THE ACQUISITION CORE IS NOT VERIFIED HERE: {missing}"
        print(f"\n*** {message}")
        pytest.skip(message)


def run(exe, mode, numbers):
    text = " ".join(str(int(v)) for v in numbers)
    out = subprocess.run([str(exe), mode], input=text, capture_output=True,
                         text=True, check=True)
    return out.stdout.splitlines()


def hist(exe, values, per_mille):
    lines = run(exe, "hist", [len(values), *values, len(per_mille), *per_mille])
    head = lines[0].split()
    summary = dict(zip(head[0::2], (int(v) for v in head[1::2])))
    percentiles = {int(p): int(v) for _, p, v in (line.split() for line in lines[1:])}
    return summary, percentiles


def test_percentiles_match_numpy_nearest_rank(acqcheck):
    rng = np.random.default_rng(SEED)
    # Interval-shaped data: a spike near the 1103 us period, a tail of late
    # wake-ups. An odd count, so p50 and p99 do not land on exact ranks.
    values = np.concatenate([rng.normal(1103, 4, 27000),
                             rng.uniform(1103, 2000, 211)]).round().astype(int)
    per_mille = [1, 500, 900, 990, 999, 1000]
    summary, got = hist(acqcheck, values, per_mille)

    assert summary["count"] == len(values)
    assert summary["over"] == 0
    assert summary["min"] == values.min()
    assert summary["max"] == values.max()
    for pm in per_mille:
        expected = np.percentile(values, pm / 10, method="inverted_cdf")
        assert got[pm] == expected, f"p{pm / 10}"


def test_percentile_among_out_of_range_values_is_unknown_not_invented(acqcheck):
    """Values past the last bin are counted, not binned. A percentile that
    falls among them has no honest value except 'at least the range', and the
    exact maximum is kept separately."""
    values = [1000] * 90 + [5000 + i for i in range(10)]
    summary, got = hist(acqcheck, values, [500, 900, 950, 1000])
    assert summary["over"] == 10
    assert summary["max"] == 5009
    assert got[500] == 1000
    assert got[900] == 1000
    assert got[950] == UNKNOWN
    assert got[1000] == UNKNOWN


def test_empty_histogram_reports_unknown(acqcheck):
    summary, got = hist(acqcheck, [], [500])
    assert summary["count"] == 0
    assert got[500] == UNKNOWN


def test_sequence_accounting_recovers_generated_gaps(acqcheck):
    """A sample stream with known losses, duplicates and one backward step,
    starting just below the 24-bit wrap so the unwrap is exercised."""
    rng = np.random.default_rng(SEED + 1)
    start = MASK24 - 300
    produced = 20000
    kept = np.ones(produced, dtype=bool)
    lost = rng.choice(np.arange(1, produced - 1), size=137, replace=False)
    kept[lost] = False

    stream = []          # (timestamp, kind, true_index)
    duplicates = 0
    backward = 0
    last_kept = None
    for i in range(produced):
        if not kept[i]:
            continue
        stream.append(((start + i) & MASK24, "new", i))
        if rng.random() < 0.003:
            stream.append(((start + i) & MASK24, "dup", i))
            duplicates += 1
        if i == 9000 and last_kept is not None:
            stream.append(((start + last_kept) & MASK24, "back", None))
            backward += 1
        last_kept = i

    lines = run(acqcheck, "seq", [len(stream), *(t for t, _, _ in stream)])
    results = [line.split() for line in lines[:-1]]
    totals = lines[-1].split()
    totals = dict(zip(totals[1::2], (int(v) for v in totals[2::2])))

    kinds = {"0": "new", "1": "dup", "2": "back"}
    for (ts, kind, truth), (_, code, index) in zip(stream, results):
        assert kinds[code] == kind
        if kind == "new":
            assert int(index) == truth

    assert totals["lost"] == len(lost)
    assert totals["duplicated"] == duplicates
    assert totals["backward"] == backward
    assert totals["received"] == len(stream)
    assert totals["produced"] == produced


def test_ring_matches_a_bounded_queue(acqcheck):
    rng = np.random.default_rng(SEED + 2)
    ops = []
    # Fill past capacity, drain past empty, then random traffic.
    ops += [1] * (RING_CAPACITY + 40) + [0] * (RING_CAPACITY + 10)
    ops += list(rng.choice([0, 1], size=5000, p=[0.45, 0.55]))
    lines = run(acqcheck, "ring", [len(ops), *ops])

    model = deque()
    value = 0
    dropped = 0
    for op, line in zip(ops, lines):
        if op == 1:
            value += 1
            if len(model) < RING_CAPACITY:
                model.append(value)
                assert line == "push ok"
            else:
                dropped += 1
                assert line == "push full"
        else:
            if model:
                assert line == f"pop {model.popleft()}"
            else:
                assert line == "pop empty"
    assert lines[-1] == f"dropped {dropped}"


def test_frames_decode_with_capture_py(acqcheck):
    rng = np.random.default_rng(SEED + 3)
    expected = []
    wire = b""
    for _ in range(200):
        n = int(rng.integers(1, 17))
        first_seq = int(rng.integers(0, 2**32))
        flags = int(rng.integers(0, 4))
        micros = int(rng.integers(0, 2**32))
        samples = rng.integers(-32768, 32768, size=(n, 6))
        line = run(acqcheck, "frame",
                   [first_seq, flags, micros, n, *samples.ravel()])[0]
        wire += bytes.fromhex(line)
        expected.append((first_seq, flags, micros,
                         [tuple(int(v) for v in row) for row in samples]))

    batches, leftover = decode_batches(wire)
    assert leftover == b""
    assert len(batches) == len(expected)
    for batch, (first_seq, flags, micros, samples) in zip(batches, expected):
        assert batch.first_seq == first_seq
        assert batch.overflow == bool(flags & 1)
        assert batch.sensor_seq == bool(flags & 2)
        assert batch.drain_micros == micros
        assert batch.samples == samples
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `uv run pytest tests/test_acq.py -v` (from `analysis/`)
Expected: FAIL / ERROR with `AttributeError: module 'tools.cbuild' has no attribute 'build_acqcheck'`

- [ ] **Step 3: Generalise `cbuild.build()` and add `build_acqcheck()`**

In `analysis/tools/cbuild.py`, below `SOURCES = [...]` add:
```python
ACQ = REPO / "firmware" / "components" / "acq"
ACQ_SOURCES = [HARNESS / "acqcheck.c",
               *(ACQ / "src" / f"{name}.c"
                 for name in ("ring", "seqcount", "jitter", "frame"))]
```
Change the signature and the first lines of `build()`:
```python
def build(name: str, *defines: str, sources: list[Path] | None = None,
          includes: list[Path] | None = None) -> BuildResult:
    """Compile a host harness to `name` under firmware/test. Raises if it cannot.

    `defines` are passed through in the compiler's own spelling, e.g.
    "PLUMB_SINGLE_PRECISION". `sources` and `includes` default to the
    quaternion port's differential harness.
    """
    sources = SOURCES if sources is None else sources
    includes = [COMPONENT / "include"] if includes is None else includes
```
and in both compiler commands replace `f"-I{COMPONENT / 'include'}",` with
`*(f"-I{p}" for p in includes),` and `*(str(s) for s in SOURCES),` with
`*(str(s) for s in sources),`. Then add after `build()`:
```python
def build_acqcheck(name: str = "acqcheck.exe") -> BuildResult:
    """The acquisition core's pure-C modules, for tests/test_acq.py."""
    return build(name, sources=ACQ_SOURCES, includes=[ACQ / "include"])
```

- [ ] **Step 4: Write the record, ring, sequence, histogram and frame modules**

`firmware/components/acq/include/acq/record.h`:
```c
/* One IMU sample as the acquisition task read it.
 *
 * Raw counts and raw times only: the corpus stores raw samples, never derived
 * metrics (CLAUDE.md), and this is the record everything downstream is built
 * from. Pure C, no ESP-IDF, so the host harness shares it.
 */
#ifndef ACQ_RECORD_H
#define ACQ_RECORD_H

#include <stdint.h>

typedef struct {
    uint32_t timestamp;  /* the sensor's 24-bit sample counter, TIMESTAMP_L..H */
    int16_t accel[3];    /* AX, AY, AZ, raw counts */
    int16_t gyro[3];     /* GX, GY, GZ, raw counts */
    uint32_t edge_us;    /* esp_timer at the DRDY rising edge, low 32 bits */
    uint32_t done_us;    /* esp_timer when the burst read completed */
    uint8_t edges;       /* DRDY edges since the previous read: 1 unless the task fell behind */
    uint8_t statusint;   /* STATUSINT as first seen with Avail set: bit 0 Avail, bit 1 Locked */
    uint8_t polls;       /* STATUSINT reads it took to see Avail */
    uint8_t reserved;
} acq_record;

#endif /* ACQ_RECORD_H */
```

`firmware/components/acq/include/acq/ring.h`:
```c
/* Single-producer, single-consumer ring of acq_records.
 *
 * The producer is the acquisition task on core 0 and the consumer the app task
 * on core 1 (parent spec 6.2: "a lock-free ring buffer"). No lock and no
 * critical section: a FreeRTOS queue would take a spinlock that both cores
 * contend for, and that is jitter the acquisition core does not need.
 *
 * head and tail are free-running counters; the slot is the counter modulo the
 * capacity. The producer alone writes head and dropped, the consumer alone
 * writes tail.
 */
#ifndef ACQ_RING_H
#define ACQ_RING_H

#include <stdbool.h>
#include <stdint.h>

#include "acq/record.h"

/* A power of two. 512 records is 0.56 s at 906.86 Hz, which covers the
 * longest the app task blocks: one full-screen LVGL render. */
#define ACQ_RING_CAPACITY 512u

typedef struct {
    acq_record slots[ACQ_RING_CAPACITY];
    uint32_t head;
    uint32_t tail;
    uint32_t dropped; /* records refused because the ring was full */
} acq_ring;

void acq_ring_init(acq_ring *ring);

/* Producer. False, and counted in dropped, when the ring is full. */
bool acq_ring_push(acq_ring *ring, const acq_record *record);

/* Consumer. False when the ring is empty. */
bool acq_ring_pop(acq_ring *ring, acq_record *out);

/* Either side. Monotonic. */
uint32_t acq_ring_dropped(const acq_ring *ring);

#endif /* ACQ_RING_H */
```

`firmware/components/acq/src/ring.c`:
```c
#include "acq/ring.h"

#include <string.h>

/* Acquire/release ordering between the cores. On the device this is GCC's
 * __atomic builtins, which emit the memory barriers the two LX7 cores need.
 * The host harness is built with MSVC and used from one thread, so there it
 * degrades to plain accesses: the host test proves ordering of the queue and
 * the overflow accounting, not the memory model, and says so here. */
#if defined(__GNUC__)
#define LOAD_ACQUIRE(p) __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define STORE_RELEASE(p, v) __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#else
#define LOAD_ACQUIRE(p) (*(p))
#define STORE_RELEASE(p, v) (*(p) = (v))
#endif

#define MASK (ACQ_RING_CAPACITY - 1u)

void acq_ring_init(acq_ring *ring)
{
    memset(ring, 0, sizeof(*ring));
}

bool acq_ring_push(acq_ring *ring, const acq_record *record)
{
    const uint32_t head = ring->head;
    const uint32_t tail = LOAD_ACQUIRE(&ring->tail);
    if (head - tail >= ACQ_RING_CAPACITY) {
        STORE_RELEASE(&ring->dropped, ring->dropped + 1u);
        return false;
    }
    ring->slots[head & MASK] = *record;
    STORE_RELEASE(&ring->head, head + 1u);
    return true;
}

bool acq_ring_pop(acq_ring *ring, acq_record *out)
{
    const uint32_t tail = ring->tail;
    const uint32_t head = LOAD_ACQUIRE(&ring->head);
    if (head == tail) {
        return false;
    }
    *out = ring->slots[tail & MASK];
    STORE_RELEASE(&ring->tail, tail + 1u);
    return true;
}

uint32_t acq_ring_dropped(const acq_ring *ring)
{
    return LOAD_ACQUIRE(&ring->dropped);
}
```

`firmware/components/acq/include/acq/seqcount.h`:
```c
/* Sample accounting from the sensor's own 24-bit sample counter.
 *
 * This is the method that found the FIFO path's 21.9% loss: the part numbers
 * every sample it produces, so a step of more than one between consecutive
 * reads is a sample produced and never read, and a step of zero is the same
 * sample read twice. Counting what arrived cannot see either (imu_stream's
 * "dropped: 0" alongside 20% loss).
 *
 * A step of more than half the counter's range is taken as the counter going
 * backwards rather than as eight million lost samples.
 */
#ifndef ACQ_SEQCOUNT_H
#define ACQ_SEQCOUNT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ACQ_SEQ_NEW = 0,       /* a sample not seen before; index is its position */
    ACQ_SEQ_DUPLICATE = 1, /* the same counter value as the previous record */
    ACQ_SEQ_BACKWARD = 2,  /* the counter went back; not placed */
} acq_seq_result;

typedef struct {
    bool started;
    uint32_t last;       /* last counter value accepted, 24 bits */
    uint32_t index;      /* unwrapped position of that sample since the first */
    uint32_t received;   /* every record fed in */
    uint32_t lost;       /* samples the counter says were produced and never read */
    uint32_t duplicated;
    uint32_t backward;
} acq_seqcount;

void acq_seqcount_reset(acq_seqcount *s);

/* Feed one record's counter. *index receives the sample's position since the
 * first record for ACQ_SEQ_NEW and ACQ_SEQ_DUPLICATE. */
acq_seq_result acq_seqcount_update(acq_seqcount *s, uint32_t timestamp,
                                   uint32_t *index);

/* Samples the sensor produced from the first record to the last, inclusive. */
uint32_t acq_seqcount_produced(const acq_seqcount *s);

#endif /* ACQ_SEQCOUNT_H */
```

`firmware/components/acq/src/seqcount.c`:
```c
#include "acq/seqcount.h"

#include <string.h>

#define MASK24 0x00FFFFFFu
#define HALF24 0x00800000u

void acq_seqcount_reset(acq_seqcount *s)
{
    memset(s, 0, sizeof(*s));
}

acq_seq_result acq_seqcount_update(acq_seqcount *s, uint32_t timestamp,
                                   uint32_t *index)
{
    timestamp &= MASK24;
    s->received++;
    if (!s->started) {
        s->started = true;
        s->last = timestamp;
        s->index = 0;
        *index = 0;
        return ACQ_SEQ_NEW;
    }

    const uint32_t step = (timestamp - s->last) & MASK24;
    if (step == 0) {
        s->duplicated++;
        *index = s->index;
        return ACQ_SEQ_DUPLICATE;
    }
    if (step >= HALF24) {
        s->backward++;
        return ACQ_SEQ_BACKWARD;
    }
    s->lost += step - 1u;
    s->index += step;
    s->last = timestamp;
    *index = s->index;
    return ACQ_SEQ_NEW;
}

uint32_t acq_seqcount_produced(const acq_seqcount *s)
{
    return s->started ? s->index + 1u : 0u;
}
```

`firmware/components/acq/include/acq/jitter.h`:
```c
/* A 1 us histogram with exact extremes and nearest-rank percentiles.
 *
 * Used for two timings per sample: the interval between successive DRDY
 * edges, and the latency from an edge to the end of its read. No pass or fail
 * threshold is attached to either (the skeleton spec, invariant 5's spirit):
 * the histogram reports, a person decides.
 *
 * Values at or beyond ACQ_HIST_BINS microseconds are counted, and their
 * maximum kept exactly, but they are not binned. A percentile that falls
 * among them is ACQ_HIST_UNKNOWN rather than a made-up number.
 */
#ifndef ACQ_JITTER_H
#define ACQ_JITTER_H

#include <stdint.h>

#define ACQ_HIST_BINS 2048u
#define ACQ_HIST_UNKNOWN 0xFFFFFFFFu

typedef struct {
    uint32_t bins[ACQ_HIST_BINS];
    uint32_t count;
    uint32_t over; /* values >= ACQ_HIST_BINS */
    uint32_t min;
    uint32_t max;
} acq_hist;

void acq_hist_reset(acq_hist *h);
void acq_hist_add(acq_hist *h, uint32_t us);

/* The smallest recorded value v such that at least ceil(per_mille/1000 *
 * count) values are <= v. per_mille in 1..1000: 500 is the median, 990 p99. */
uint32_t acq_hist_percentile(const acq_hist *h, uint32_t per_mille);

#endif /* ACQ_JITTER_H */
```

`firmware/components/acq/src/jitter.c`:
```c
#include "acq/jitter.h"

#include <string.h>

void acq_hist_reset(acq_hist *h)
{
    memset(h, 0, sizeof(*h));
    h->min = ACQ_HIST_UNKNOWN;
}

void acq_hist_add(acq_hist *h, uint32_t us)
{
    h->count++;
    if (us < h->min) {
        h->min = us;
    }
    if (us > h->max) {
        h->max = us;
    }
    if (us < ACQ_HIST_BINS) {
        h->bins[us]++;
    } else {
        h->over++;
    }
}

uint32_t acq_hist_percentile(const acq_hist *h, uint32_t per_mille)
{
    if (h->count == 0) {
        return ACQ_HIST_UNKNOWN;
    }
    /* Integer ceiling: the rank is exact, with no floating point to round. */
    uint64_t rank = ((uint64_t)per_mille * h->count + 999u) / 1000u;
    if (rank == 0) {
        rank = 1;
    }
    uint64_t seen = 0;
    for (uint32_t us = 0; us < ACQ_HIST_BINS; us++) {
        seen += h->bins[us];
        if (seen >= rank) {
            return us;
        }
    }
    return ACQ_HIST_UNKNOWN;
}
```

`firmware/components/acq/include/acq/frame.h`:
```c
/* imu_stream's binary frame, so analysis/tools/capture.py reads this firmware
 * unchanged.
 *
 *   A5 5A | first_seq u32 | count u8 | flags u8 | micros u32 | count x 6 x i16 | xor
 *
 * Little-endian. The XOR covers everything between the sync word and itself.
 * The layout's other implementations are bringup-arduino/imu_stream/framing.cpp
 * and capture.py; tests/test_acq.py decodes this one with capture.py.
 */
#ifndef ACQ_FRAME_H
#define ACQ_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "acq/record.h"

#define ACQ_FRAME_MAX_SAMPLES 16u
#define ACQ_FRAME_FLAG_OVERFLOW 0x01u   /* samples were dropped before this frame */
#define ACQ_FRAME_FLAG_SENSOR_SEQ 0x02u /* seq comes from the sensor's own counter */
#define ACQ_FRAME_BYTES(n) (2u + 10u + 12u * (n) + 1u)

/* Encode `count` records (1..ACQ_FRAME_MAX_SAMPLES) with consecutive sequence
 * numbers from first_seq. Returns the bytes written to out, which must hold
 * ACQ_FRAME_BYTES(count). */
size_t acq_frame_encode(uint8_t *out, uint32_t first_seq, uint8_t flags,
                        uint32_t micros, const acq_record *records,
                        uint8_t count);

#endif /* ACQ_FRAME_H */
```

`firmware/components/acq/src/frame.c`:
```c
#include "acq/frame.h"

typedef struct {
    uint8_t *out;
    size_t n;
    uint8_t sum;
} writer;

static void put(writer *w, uint8_t b)
{
    w->out[w->n++] = b;
    w->sum ^= b;
}

static void put32(writer *w, uint32_t v)
{
    put(w, (uint8_t)(v & 0xFFu));
    put(w, (uint8_t)((v >> 8) & 0xFFu));
    put(w, (uint8_t)((v >> 16) & 0xFFu));
    put(w, (uint8_t)((v >> 24) & 0xFFu));
}

static void put16(writer *w, int16_t v)
{
    const uint16_t u = (uint16_t)v;
    put(w, (uint8_t)(u & 0xFFu));
    put(w, (uint8_t)(u >> 8));
}

size_t acq_frame_encode(uint8_t *out, uint32_t first_seq, uint8_t flags,
                        uint32_t micros, const acq_record *records,
                        uint8_t count)
{
    out[0] = 0xA5;
    out[1] = 0x5A;
    writer w = {out, 2, 0};
    put32(&w, first_seq);
    put(&w, count);
    put(&w, flags);
    put32(&w, micros);
    for (uint8_t i = 0; i < count; i++) {
        for (int axis = 0; axis < 3; axis++) {
            put16(&w, records[i].accel[axis]);
        }
        for (int axis = 0; axis < 3; axis++) {
            put16(&w, records[i].gyro[axis]);
        }
    }
    out[w.n] = w.sum;
    return w.n + 1;
}
```

- [ ] **Step 5: Write the host harness**

`firmware/test/acqcheck.c`:
```c
/* Host harness for the acquisition core's pure-C modules.
 *
 * Driven by analysis/tests/test_acq.py, which checks each module against an
 * independent reference. The first argument picks the module; the input is
 * whitespace-separated integers on stdin.
 *
 *   hist  : n, n values, k, k per-mille ranks
 *   seq   : n, n counter values
 *   ring  : n, n ops (1 push the next value, 0 pop)
 *   frame : first_seq, flags, micros, n, n x 6 samples
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "acq/frame.h"
#include "acq/jitter.h"
#include "acq/ring.h"
#include "acq/seqcount.h"

static long long next(void)
{
    long long v;
    if (scanf("%lld", &v) != 1) {
        fprintf(stderr, "acqcheck: input ended early\n");
        exit(2);
    }
    return v;
}

static int run_hist(void)
{
    static acq_hist h;
    acq_hist_reset(&h);
    const long long n = next();
    for (long long i = 0; i < n; i++) {
        acq_hist_add(&h, (uint32_t)next());
    }
    printf("count %lu over %lu min %lu max %lu\n", (unsigned long)h.count,
           (unsigned long)h.over, (unsigned long)h.min, (unsigned long)h.max);
    const long long k = next();
    for (long long i = 0; i < k; i++) {
        const uint32_t pm = (uint32_t)next();
        printf("p %lu %lu\n", (unsigned long)pm,
               (unsigned long)acq_hist_percentile(&h, pm));
    }
    return 0;
}

static int run_seq(void)
{
    acq_seqcount s;
    acq_seqcount_reset(&s);
    const long long n = next();
    for (long long i = 0; i < n; i++) {
        uint32_t index = 0;
        const acq_seq_result r = acq_seqcount_update(&s, (uint32_t)next(), &index);
        printf("r %d %lu\n", (int)r, (unsigned long)index);
    }
    printf("totals received %lu lost %lu duplicated %lu backward %lu produced %lu\n",
           (unsigned long)s.received, (unsigned long)s.lost,
           (unsigned long)s.duplicated, (unsigned long)s.backward,
           (unsigned long)acq_seqcount_produced(&s));
    return 0;
}

static int run_ring(void)
{
    static acq_ring ring;
    acq_ring_init(&ring);
    const long long n = next();
    uint32_t value = 0;
    for (long long i = 0; i < n; i++) {
        if (next() == 1) {
            acq_record r;
            memset(&r, 0, sizeof(r));
            r.timestamp = ++value;
            puts(acq_ring_push(&ring, &r) ? "push ok" : "push full");
        } else {
            acq_record r;
            if (acq_ring_pop(&ring, &r)) {
                printf("pop %lu\n", (unsigned long)r.timestamp);
            } else {
                puts("pop empty");
            }
        }
    }
    printf("dropped %lu\n", (unsigned long)acq_ring_dropped(&ring));
    return 0;
}

static int run_frame(void)
{
    const uint32_t first_seq = (uint32_t)next();
    const uint8_t flags = (uint8_t)next();
    const uint32_t micros = (uint32_t)next();
    const uint8_t n = (uint8_t)next();
    acq_record records[ACQ_FRAME_MAX_SAMPLES];
    memset(records, 0, sizeof(records));
    for (uint8_t i = 0; i < n; i++) {
        for (int axis = 0; axis < 3; axis++) {
            records[i].accel[axis] = (int16_t)next();
        }
        for (int axis = 0; axis < 3; axis++) {
            records[i].gyro[axis] = (int16_t)next();
        }
    }
    uint8_t out[ACQ_FRAME_BYTES(ACQ_FRAME_MAX_SAMPLES)];
    const size_t len = acq_frame_encode(out, first_seq, flags, micros, records, n);
    for (size_t i = 0; i < len; i++) {
        printf("%02x", out[i]);
    }
    printf("\n");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: acqcheck hist|seq|ring|frame < numbers\n");
        return 2;
    }
    if (strcmp(argv[1], "hist") == 0) return run_hist();
    if (strcmp(argv[1], "seq") == 0) return run_seq();
    if (strcmp(argv[1], "ring") == 0) return run_ring();
    if (strcmp(argv[1], "frame") == 0) return run_frame();
    fprintf(stderr, "acqcheck: unknown mode %s\n", argv[1]);
    return 2;
}
```

- [ ] **Step 6: Run the tests to make sure they pass**

Run: `uv run pytest tests/test_acq.py -v` (from `analysis/`)
Expected: 6 passed. Then the whole suite: `uv run pytest -q`. Expected: 211 passed (205 + 6),
with `test_c_port.py` still passing, which shows that `build()` still builds `portcheck`.

- [ ] **Step 7: Commit**

```bash
git add firmware/components/acq firmware/test/acqcheck.c analysis/tools/cbuild.py analysis/tests/test_acq.py
git commit -m "Add the acquisition core: ring, sample accounting, jitter histogram, framing"
```

---

### Task 2: The invariant-6 map check

**Files:**
- Create: `firmware/tools/check_no_radio.py`
- Test: `analysis/tests/test_invariants.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `analysis/tests/test_invariants.py`:
```python
def _radio_check():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[2] / "firmware" / "tools" / "check_no_radio.py"
    spec = importlib.util.spec_from_file_location("check_no_radio", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Excerpts in GNU ld's map format: the archive-member section, then a placed
# symbol. The radio one is what linking esp_wifi_init() produces.
_CLEAN_MAP = """\
Archive member included to satisfy reference by file (symbol)

esp-idf/esp_timer/libesp_timer.a(esp_timer.c.obj)
                              esp-idf/main/libmain.a(app_main.c.obj) (esp_timer_get_time)
 .text.esp_timer_get_time
                0x42001234       0x10 esp-idf/esp_timer/libesp_timer.a(esp_timer.c.obj)
                0x42001234                esp_timer_get_time
"""

_RADIO_MAP = _CLEAN_MAP + """\
esp-idf/esp_wifi/libesp_wifi.a(wifi_init.c.obj)
                              esp-idf/main/libmain.a(app_main.c.obj) (esp_wifi_init)
 .text.esp_wifi_init
                0x42005678       0x40 esp-idf/esp_wifi/libesp_wifi.a(wifi_init.c.obj)
                0x42005678                esp_wifi_init
"""


def test_invariant_6_map_check_passes_a_clean_map():
    """CLAUDE.md invariant 6. The check has to pass a real build's map, or it
    is noise that gets switched off."""
    assert _radio_check().offending(_CLEAN_MAP) == []


def test_invariant_6_map_check_rejects_linked_wifi():
    """CLAUDE.md invariant 6: Wi-Fi stays compiled out in every build. The
    check is what makes that a property of the binary rather than a hope."""
    found = _radio_check().offending(_RADIO_MAP)
    assert found
    assert any("esp_wifi" in line for line in found)
```

- [ ] **Step 2: Run it to make sure it fails**

Run: `uv run pytest tests/test_invariants.py -v -k invariant_6`
Expected: FAIL with `FileNotFoundError` for `check_no_radio.py`

- [ ] **Step 3: Write the check**

`firmware/tools/check_no_radio.py`:
```python
"""Fail the firmware build if any Wi-Fi or Bluetooth code was linked.

CLAUDE.md invariant 6: Wi-Fi stays compiled out in every build, because the
Wi-Fi stack changes interrupt behaviour and therefore sampling jitter, which
the accuracy argument rests on. That is a property of the linked binary, not of
the source tree, so this reads the linker map: it records every archive member
pulled in and every symbol placed.

Run by firmware/CMakeLists.txt after every link:

    python check_no_radio.py build/plumb.map
"""

import re
import sys
from pathlib import Path

# Archives that exist only for the radios: the Wi-Fi driver and its binary
# blobs (net80211, pp, core), the supplicant, coexistence, and the Bluetooth
# controller and host.
_ARCHIVES = re.compile(
    r"\blib(esp_wifi|wpa_supplicant|net80211|pp|core|coexist|espnow|mesh|"
    r"smartconfig|wapi|bt|btdm_app|btbb|ble_app|esp_coex)\.a\b")
# Placed symbols from the radio APIs.
_SYMBOLS = re.compile(r"(?<![\w.])(esp_wifi_\w+|wifi_\w+|esp_bt_\w+|esp_ble_\w+)\b")


def offending(map_text: str) -> list[str]:
    """Lines of the map that show radio code in the image, deduplicated."""
    found: list[str] = []
    for line in map_text.splitlines():
        if _ARCHIVES.search(line) or _SYMBOLS.search(line):
            stripped = line.strip()
            if stripped not in found:
                found.append(stripped)
    return found


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: check_no_radio.py <linker map>", file=sys.stderr)
        return 2
    path = Path(argv[1])
    if not path.is_file():
        print(f"check_no_radio: no linker map at {path}", file=sys.stderr)
        return 2
    found = offending(path.read_text(errors="replace"))
    if found:
        print("INVARIANT 6 VIOLATED: radio code is linked into the firmware "
              "(CLAUDE.md). First lines of the map that show it:", file=sys.stderr)
        for line in found[:20]:
            print(f"  {line}", file=sys.stderr)
        return 1
    print(f"check_no_radio: no Wi-Fi or Bluetooth code in {path.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
```

- [ ] **Step 4: Run the tests to make sure they pass**

Run: `uv run pytest tests/test_invariants.py -v`
Expected: all pass, including the two new ones.

- [ ] **Step 5: Commit**

```bash
git add firmware/tools/check_no_radio.py analysis/tests/test_invariants.py
git commit -m "Add the invariant-6 map check: fail the build if radio code is linked"
```

---

### Task 3: The board component and the display driver's move

**Files:**
- Move: `firmware/bringup-arduino/display/gc9a01.c` → `firmware/components/board/src/gc9a01.c`
- Move: `firmware/bringup-arduino/display/gc9a01.h` → `firmware/components/board/include/board/gc9a01.h`
- Create: `firmware/bringup-arduino/display/gc9a01.c` (wrapper)
- Modify: `firmware/bringup-arduino/display/display.ino` (include path)
- Modify: `analysis/tools/board_ui.py` (board include path; `send --baud`)
- Create: `firmware/components/board/CMakeLists.txt`, `include/board/pins.h`, `include/board/i2c_bus.h`, `src/i2c_bus.c`

- [ ] **Step 1: Move the driver, keeping one copy**

```bash
mkdir -p firmware/components/board/src firmware/components/board/include/board
git mv firmware/bringup-arduino/display/gc9a01.c firmware/components/board/src/gc9a01.c
git mv firmware/bringup-arduino/display/gc9a01.h firmware/components/board/include/board/gc9a01.h
```
In `firmware/components/board/src/gc9a01.c` change `#include "gc9a01.h"` to
`#include "board/gc9a01.h"`. In `display.ino` change `#include "gc9a01.h"` to
`#include "board/gc9a01.h"`.

`firmware/bringup-arduino/display/gc9a01.c`:
```c
/* Compiles the panel driver from its one location in the repo,
 * firmware/components/board; see board_ui.py. */
#include "src/gc9a01.c"
```

In `analysis/tools/board_ui.py`, add `BOARD = REPO / "firmware" / "components" / "board"`
under `UI = ...`, and make `_flags()`:
```python
def _flags() -> list[str]:
    ui, include, src = (p.as_posix() for p in (UI, UI / "include", UI / "src"))
    board, board_include = BOARD.as_posix(), (BOARD / "include").as_posix()
    extra = (f"-DLV_CONF_INCLUDE_SIMPLE -I{ui} -I{include} -I{src} "
             f"-I{board} -I{board_include}")
    return ["--library", LVGL.as_posix(),
            "--build-property", f"compiler.c.extra_flags={extra}",
            "--build-property", f"compiler.cpp.extra_flags={extra}"]
```
Give `send()` a `baud: int = 115200` parameter used for `port_obj.baudrate`, and add
`s.add_argument("--baud", type=int, default=115200)` with `send(args.port, args.commands,
args.seconds, args.baud)`. The product firmware's console runs at 921600.

- [ ] **Step 2: Rebuild the bring-up sketch to show the move broke nothing**

Run: `uv run python -m tools.board_ui build` (from `analysis/`)
Expected: compiles, and the sketch size matches the previous build to within a few bytes.

- [ ] **Step 3: Write pins and the shared bus**

`firmware/components/board/include/board/pins.h`:
```c
/* Waveshare ESP32-S3-Touch-LCD-1.28, Rev3 schematic
 * (docs/bringup-results.md, "Pins"). The panel's SPI pins live with its
 * driver, src/gc9a01.c, which is carried over verbatim from the bring-up. */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#define BOARD_PIN_I2C_SDA 6 /* shared: IMU and touch (parent spec 4.2) */
#define BOARD_PIN_I2C_SCL 7
#define BOARD_PIN_IMU_INT1 4
#define BOARD_PIN_IMU_INT2 3 /* DRDY in SyncSample mode (datasheet rev A 6.1) */
#define BOARD_PIN_TP_INT 5
#define BOARD_PIN_TP_RST 13

/* The QMI8658 is a fast-mode (400 kHz) I2C part. */
#define BOARD_I2C_HZ 400000

#endif /* BOARD_PINS_H */
```

`firmware/components/board/include/board/i2c_bus.h`:
```c
/* The one I2C bus, shared by the IMU and the touch controller.
 *
 * The master driver serialises transactions from different tasks with its own
 * bus lock. Only the acquisition task reads the IMU; touch is read by the app
 * task, and only while the invariant-8 gate is open, so under the gate the IMU
 * has the bus to itself. */
#ifndef BOARD_I2C_BUS_H
#define BOARD_I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

/* Create the bus. Its interrupt is allocated on the calling core: call it
 * from core 0, where acquisition runs. */
esp_err_t board_i2c_init(void);

i2c_master_bus_handle_t board_i2c_bus(void);

#endif /* BOARD_I2C_BUS_H */
```

`firmware/components/board/src/i2c_bus.c`:
```c
#include "board/i2c_bus.h"

#include "board/pins.h"

static i2c_master_bus_handle_t s_bus;

esp_err_t board_i2c_init(void)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = -1,
        .sda_io_num = BOARD_PIN_I2C_SDA,
        .scl_io_num = BOARD_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, &s_bus);
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return s_bus;
}
```

`firmware/components/board/CMakeLists.txt` (the IMU and touch sources are added in Task 4):
```cmake
# Board drivers on ESP-IDF: the shared I2C bus, the QMI8658 IMU, the CST816
# touch controller and the GC9A01 panel. Hardware only; no UI, no algorithm.
idf_component_register(
    SRCS "src/i2c_bus.c" "src/qmi8658.c" "src/cst816.c" "src/gc9a01.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_spi esp_lcd)
```

- [ ] **Step 4: Commit**

```bash
git add -A firmware/components/board firmware/bringup-arduino/display analysis/tools/board_ui.py
git commit -m "Move the panel driver into a board component, keep the sketch on it"
```

---

### Task 4: IMU and touch drivers

**Files:**
- Create: `firmware/components/board/include/board/qmi8658.h`, `src/qmi8658.c`
- Create: `firmware/components/board/include/board/cst816.h`, `src/cst816.c`

- [ ] **Step 1: Write the IMU driver**

`firmware/components/board/include/board/qmi8658.h`:
```c
/* QMI8658 IMU in SyncSample mode, paced by DRDY on INT2.
 *
 * Configuration is the stroke configuration proven by bringup-arduino/
 * imu_stream: 896.8 Hz nominal (906.86 Hz measured on this unit), +-16 g,
 * +-256 dps, gyro low-pass on, FIFO bypassed. What is new is the pacing:
 * the part's own data-ready line, and the lock that makes a slow I2C read
 * safe (datasheet rev A 6.1, 6.3, 13.2). */
#ifndef BOARD_QMI8658_H
#define BOARD_QMI8658_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define QMI8658_STATUSINT_AVAIL 0x01u
#define QMI8658_STATUSINT_LOCKED 0x02u

typedef struct {
    uint32_t timestamp; /* 24-bit sample counter */
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t statusint;  /* STATUSINT when Avail was first seen */
    uint8_t polls;      /* STATUSINT reads it took */
} qmi8658_sample;

/* Reset, verify and configure the part, leaving DRDY pulsing on INT2. */
esp_err_t qmi8658_init(i2c_master_bus_handle_t bus);

/* One locked read (13.2.3): STATUSINT starts the lock, the burst through GZ_H
 * releases it. ESP_ERR_NOT_FOUND if Avail never came up. */
esp_err_t qmi8658_read_locked(qmi8658_sample *out);

#endif /* BOARD_QMI8658_H */
```

`firmware/components/board/src/qmi8658.c`:
```c
/* See qmi8658.h. Register facts are from the QMI8658C datasheet rev A (QST
 * document 13-52-27) and were proven on this board by bringup-arduino/
 * imu_stream/qmi8658.cpp, whose comments carry the reasoning. */

#include "board/qmi8658.h"

#include "board/pins.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADDRESS 0x6B
#define TIMEOUT_MS 10

#define REG_WHO_AM_I 0x00
#define REG_CTRL1 0x02
#define REG_CTRL2 0x03
#define REG_CTRL3 0x04
#define REG_CTRL5 0x06
#define REG_CTRL7 0x08
#define REG_CTRL9 0x0A
#define REG_CAL1_L 0x0B
#define REG_FIFO_CTRL 0x14
#define REG_STATUSINT 0x2D
#define REG_TIMESTAMP_L 0x30
#define REG_GZ_H 0x40
#define REG_RESET_RESULT 0x4D
#define REG_RESET 0x60

#define WHO_AM_I_VALUE 0x05
#define RESET_COMMAND 0xB0
#define RESET_RESULT_OK 0x80
#define CTRL1_ADDR_AI 0x40
#define CTRL2_ACCEL_16G_896HZ ((0x03 << 4) | 0x03)
#define CTRL3_GYRO_256DPS_896HZ ((0x04 << 4) | 0x03)
#define CTRL5_STROKE_FILTERS 0x10 /* gyro LPF on, mode 00; accel LPF off */
#define CTRL7_SYNC_6DOF 0x83      /* SyncSample, DRDY_DIS=0, gEN, aEN (13.2.2) */
#define FIFO_CTRL_BYPASS 0x00     /* bypass: DRDY enabled (6.3) */
#define CTRL9_CMD_ACK 0x00
#define CTRL9_CMD_AHB_CLOCK_GATING 0x12
#define STATUSINT_CMD_DONE 0x80
#define CTRL9_POLLS 50

#define SYSTEM_TURN_ON_MS 15
#define GYRO_TURN_ON_MS 150
#define DATA_LOCK_DELAY_US 6 /* Table 40, ODR setting 3 */
#define AVAIL_POLLS 4
#define BLOCK_BYTES (REG_GZ_H - REG_TIMESTAMP_L + 1) /* 17 */
#define SAMPLE_OFFSET 5 /* past TIMESTAMP (3) and TEMP (2) */

static i2c_master_dev_handle_t s_dev;

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, TIMEOUT_MS);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), TIMEOUT_MS);
}

/* Write, then read back: a configuration write that did not land is found
 * here, not as a wrong number a week later. */
static esp_err_t write_verified(uint8_t reg, uint8_t value)
{
    esp_err_t err = write_reg(reg, value);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t back = 0;
    err = read_regs(reg, &back, 1);
    if (err != ESP_OK) {
        return err;
    }
    return back == value ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/* The CTRL9 protocol (5.9): write the command, wait for STATUSINT.CmdDone,
 * acknowledge. CTRL9 reads back whatever was written whether or not the
 * command ran, so CmdDone is the only confirmation there is. */
static esp_err_t ctrl9(uint8_t command)
{
    esp_err_t err = write_reg(REG_CTRL9, command);
    if (err != ESP_OK) {
        return err;
    }
    bool done = false;
    for (int i = 0; i < CTRL9_POLLS && !done; i++) {
        uint8_t status = 0;
        err = read_regs(REG_STATUSINT, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        done = (status & STATUSINT_CMD_DONE) != 0;
    }
    if (!done) {
        return ESP_ERR_TIMEOUT;
    }
    return write_reg(REG_CTRL9, CTRL9_CMD_ACK);
}

esp_err_t qmi8658_init(i2c_master_bus_handle_t bus)
{
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDRESS,
        .scl_speed_hz = BOARD_I2C_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &device, &s_dev);
    if (err != ESP_OK) {
        return err;
    }

    /* Soft reset, so every boot starts from the same state; the MCU resets
     * without power-cycling the sensor. 0x4D reads 0x80 after a good reset
     * and may be overwritten later, so it is checked now and never again. */
    err = write_reg(REG_RESET, RESET_COMMAND);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_TURN_ON_MS));
    uint8_t value = 0;
    if ((err = read_regs(REG_RESET_RESULT, &value, 1)) != ESP_OK) return err;
    if (value != RESET_RESULT_OK) return ESP_ERR_INVALID_STATE;
    if ((err = read_regs(REG_WHO_AM_I, &value, 1)) != ESP_OK) return err;
    if (value != WHO_AM_I_VALUE) return ESP_ERR_NOT_FOUND;

    /* ADDR_AI by read-modify-write: CTRL1.BE is left as reset sets it, the
     * setting the little-endian unpack was proven under. */
    uint8_t ctrl1 = 0;
    if ((err = read_regs(REG_CTRL1, &ctrl1, 1)) != ESP_OK) return err;
    if ((err = write_verified(REG_CTRL1, (uint8_t)(ctrl1 | CTRL1_ADDR_AI))) != ESP_OK) return err;

    if ((err = write_verified(REG_CTRL2, CTRL2_ACCEL_16G_896HZ)) != ESP_OK) return err;
    if ((err = write_verified(REG_CTRL3, CTRL3_GYRO_256DPS_896HZ)) != ESP_OK) return err;
    if ((err = write_verified(REG_CTRL5, CTRL5_STROKE_FILTERS)) != ESP_OK) return err;
    if ((err = write_verified(REG_FIFO_CTRL, FIFO_CTRL_BYPASS)) != ESP_OK) return err;

    /* 13.2.1: over I2C the internal AHB clock gating must be off for the lock
     * to be trustworthy. */
    if ((err = write_verified(REG_CAL1_L, 0x01)) != ESP_OK) return err;
    if ((err = ctrl9(CTRL9_CMD_AHB_CLOCK_GATING)) != ESP_OK) return err;

    if ((err = write_verified(REG_CTRL7, CTRL7_SYNC_6DOF)) != ESP_OK) return err;
    /* Gyro start-up: nothing sampled before this is believed. */
    vTaskDelay(pdMS_TO_TICKS(GYRO_TURN_ON_MS));
    return ESP_OK;
}

esp_err_t qmi8658_read_locked(qmi8658_sample *out)
{
    uint8_t status = 0;
    uint8_t polls = 0;
    esp_err_t err;
    while (polls < AVAIL_POLLS) {
        polls++;
        if ((err = read_regs(REG_STATUSINT, &status, 1)) != ESP_OK) {
            return err;
        }
        if (status & QMI8658_STATUSINT_AVAIL) {
            break;
        }
    }
    if (!(status & QMI8658_STATUSINT_AVAIL)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!(status & QMI8658_STATUSINT_LOCKED)) {
        /* Locking is in progress and completes within Data_Lock_Delay. */
        esp_rom_delay_us(DATA_LOCK_DELAY_US);
    }

    uint8_t block[BLOCK_BYTES];
    if ((err = read_regs(REG_TIMESTAMP_L, block, sizeof(block))) != ESP_OK) {
        return err;
    }
    out->timestamp = (uint32_t)block[0] | ((uint32_t)block[1] << 8) |
                     ((uint32_t)block[2] << 16);
    const uint8_t *s = block + SAMPLE_OFFSET;
    for (int axis = 0; axis < 3; axis++) {
        out->accel[axis] = (int16_t)((uint16_t)s[2 * axis + 1] << 8 | s[2 * axis]);
        out->gyro[axis] = (int16_t)((uint16_t)s[6 + 2 * axis + 1] << 8 | s[6 + 2 * axis]);
    }
    out->statusint = status;
    out->polls = polls;
    return ESP_OK;
}
```

- [ ] **Step 2: Write the touch driver**

`firmware/components/board/include/board/cst816.h`:
```c
/* CST816 capacitive touch controller (chip id 0xB5 on this board, measured). */
#ifndef BOARD_CST816_H
#define BOARD_CST816_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* Reset, read the chip id, keep it awake. */
esp_err_t cst816_init(i2c_master_bus_handle_t bus, uint8_t *chip_id);

/* True with a position while a finger is down. */
bool cst816_read(uint16_t *x, uint16_t *y);

#endif /* BOARD_CST816_H */
```

`firmware/components/board/src/cst816.c`:
```c
/* See cst816.h. Registers from the CST816S register description (Hynitron),
 * as used by bringup-arduino/display/cst816s.cpp. */

#include "board/cst816.h"

#include "board/pins.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADDRESS 0x15
#define TIMEOUT_MS 10
#define REG_FINGER_NUM 0x02 /* then XH, XL, YH, YL */
#define REG_CHIP_ID 0xA7
#define REG_DIS_AUTO_SLEEP 0xFE

static i2c_master_dev_handle_t s_dev;

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, TIMEOUT_MS);
}

esp_err_t cst816_init(i2c_master_bus_handle_t bus, uint8_t *chip_id)
{
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDRESS,
        .scl_speed_hz = BOARD_I2C_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &device, &s_dev);
    if (err != ESP_OK) {
        return err;
    }
    const gpio_config_t reset = {
        .pin_bit_mask = 1ULL << BOARD_PIN_TP_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&reset);
    gpio_set_level(BOARD_PIN_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BOARD_PIN_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(60)); /* answers for a while after reset */

    *chip_id = 0;
    err = read_regs(REG_CHIP_ID, chip_id, 1);
    /* Keep it awake so an idle read is "no finger", not a NACK. Best effort. */
    const uint8_t awake[2] = {REG_DIS_AUTO_SLEEP, 0x01};
    i2c_master_transmit(s_dev, awake, sizeof(awake), TIMEOUT_MS);
    return err;
}

bool cst816_read(uint16_t *x, uint16_t *y)
{
    uint8_t d[5];
    if (read_regs(REG_FINGER_NUM, d, sizeof(d)) != ESP_OK || d[0] == 0) {
        return false;
    }
    *x = (uint16_t)(((d[1] & 0x0F) << 8) | d[2]);
    *y = (uint16_t)(((d[3] & 0x0F) << 8) | d[4]);
    return true;
}
```

- [ ] **Step 3: Commit** (compiled in Task 7, the first IDF build)

```bash
git add firmware/components/board
git commit -m "Add the QMI8658 SyncSample driver and the CST816 touch driver on ESP-IDF"
```

---

### Task 5: The acquisition task

**Files:**
- Create: `firmware/components/acq/include/acq/acq.h`, `src/acq.c`, `CMakeLists.txt`

- [ ] **Step 1: Write it**

`firmware/components/acq/include/acq/acq.h`:
```c
/* The acquisition task: core 0, highest application priority, woken by the
 * IMU's data-ready edge (parent spec 6.2 as amended, invariant 8). */
#ifndef ACQ_ACQ_H
#define ACQ_ACQ_H

#include <stdint.h>

#include "acq/ring.h"
#include "esp_err.h"

typedef struct {
    uint32_t reads;          /* records pushed or refused */
    uint32_t read_errors;    /* I2C errors; no record */
    uint32_t not_available;  /* woken, but STATUSINT never showed Avail; no record */
    uint32_t unlocked;       /* Avail seen before Locked: waited Data_Lock_Delay */
    uint32_t missed_edges;   /* DRDY edges beyond one per read: the task fell behind */
    uint32_t drdy_timeouts;  /* 100 ms with no edge at all */
} acq_counters;

/* Start the task pinned to core 0 and wait for the IMU to configure. Must be
 * called after board_i2c_init(). */
esp_err_t acq_start(void);

/* The ring the task fills. The app task is its only reader. */
acq_ring *acq_ring_handle(void);

void acq_counters_get(acq_counters *out);

#endif /* ACQ_ACQ_H */
```

`firmware/components/acq/src/acq.c`:
```c
#include "acq/acq.h"

#include <string.h>

#include "board/i2c_bus.h"
#include "board/pins.h"
#include "board/qmi8658.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Above esp_timer's task (22), below the IPC tasks (24). */
#define ACQ_PRIORITY (configMAX_PRIORITIES - 2)
#define ACQ_CORE 0
#define ACQ_STACK 4096
#define DRDY_TIMEOUT_MS 100

static acq_ring s_ring;
static volatile acq_counters s_counters;
static TaskHandle_t s_task;
static TaskHandle_t s_starter;
static esp_err_t s_init_result;
static volatile uint32_t s_edge_us;

static void drdy_isr(void *arg)
{
    (void)arg;
    s_edge_us = (uint32_t)esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    portYIELD_FROM_ISR(woken);
}

static esp_err_t start_drdy(void)
{
    const gpio_config_t pin = {
        .pin_bit_mask = 1ULL << BOARD_PIN_IMU_INT2,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&pin);
    if (err != ESP_OK) {
        return err;
    }
    /* Installed from this task, so the GPIO interrupt lives on core 0. */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return gpio_isr_handler_add(BOARD_PIN_IMU_INT2, drdy_isr, NULL);
}

static void acq_task(void *arg)
{
    (void)arg;
    s_init_result = qmi8658_init(board_i2c_bus());
    if (s_init_result == ESP_OK) {
        s_init_result = start_drdy();
    }
    xTaskNotifyGive(s_starter);
    if (s_init_result != ESP_OK) {
        vTaskDelete(NULL);
    }

    for (;;) {
        const uint32_t edges = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DRDY_TIMEOUT_MS));
        if (edges == 0) {
            s_counters.drdy_timeouts++;
            continue;
        }
        const uint32_t edge_us = s_edge_us;
        qmi8658_sample sample;
        const esp_err_t err = qmi8658_read_locked(&sample);
        const uint32_t done_us = (uint32_t)esp_timer_get_time();
        if (edges > 1) {
            s_counters.missed_edges += edges - 1;
        }
        if (err == ESP_ERR_NOT_FOUND) {
            s_counters.not_available++;
            continue;
        }
        if (err != ESP_OK) {
            s_counters.read_errors++;
            continue;
        }
        if (!(sample.statusint & QMI8658_STATUSINT_LOCKED)) {
            s_counters.unlocked++;
        }

        acq_record record;
        memset(&record, 0, sizeof(record));
        record.timestamp = sample.timestamp;
        memcpy(record.accel, sample.accel, sizeof(record.accel));
        memcpy(record.gyro, sample.gyro, sizeof(record.gyro));
        record.edge_us = edge_us;
        record.done_us = done_us;
        record.edges = (uint8_t)(edges > 255 ? 255 : edges);
        record.statusint = sample.statusint;
        record.polls = sample.polls;
        s_counters.reads++;
        acq_ring_push(&s_ring, &record);
    }
}

esp_err_t acq_start(void)
{
    acq_ring_init(&s_ring);
    s_starter = xTaskGetCurrentTaskHandle();
    if (xTaskCreatePinnedToCore(acq_task, "acq", ACQ_STACK, NULL, ACQ_PRIORITY,
                                &s_task, ACQ_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return s_init_result;
}

acq_ring *acq_ring_handle(void)
{
    return &s_ring;
}

void acq_counters_get(acq_counters *out)
{
    /* Each field is one aligned 32-bit word written by one task; a copy may
     * mix fields a sample apart, which is fine for a diagnostic. */
    out->reads = s_counters.reads;
    out->read_errors = s_counters.read_errors;
    out->not_available = s_counters.not_available;
    out->unlocked = s_counters.unlocked;
    out->missed_edges = s_counters.missed_edges;
    out->drdy_timeouts = s_counters.drdy_timeouts;
}
```

`firmware/components/acq/CMakeLists.txt`:
```cmake
# Acquisition: the core-0 task (acq.c) and the pure-C pieces it and the app
# task share. ring, seqcount, jitter and frame have no ESP-IDF dependency and
# are proven on the host by analysis/tests/test_acq.py.
idf_component_register(
    SRCS "src/acq.c" "src/ring.c" "src/seqcount.c" "src/jitter.c" "src/frame.c"
    INCLUDE_DIRS "include"
    REQUIRES board esp_driver_gpio esp_timer)
```

- [ ] **Step 2: Commit** (compiled in Task 7)

```bash
git add firmware/components/acq
git commit -m "Add the acquisition task: DRDY edge on core 0, locked read, ring"
```

---

### Task 6: The app: UART, gate, stream, UI port, console

**Files:**
- Create: `firmware/main/CMakeLists.txt`, `app_main.c`, `gate.c/.h`, `stream.c/.h`, `ui_port.c/.h`, `uart_io.c/.h`

- [ ] **Step 1: Write the UART path**

`firmware/main/uart_io.h`:
```c
/* The one serial path, text and binary alike, owned by the app task. One
 * writer means a status line can never land in the middle of a frame. */
#ifndef UART_IO_H
#define UART_IO_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t uart_io_init(void);
void uart_io_write(const void *data, size_t len);
void uart_io_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* The next received byte, or -1. Never blocks. */
int uart_io_getc(void);

#endif /* UART_IO_H */
```

`firmware/main/uart_io.c`:
```c
#include "uart_io.h"

#include <stdarg.h>
#include <stdio.h>

#include "driver/uart.h"

#define PORT UART_NUM_0 /* the CH343 bridge */
#define BAUD 921600     /* imu_stream's rate; capture.py's default */
#define RX_BUFFER 1024
#define TX_BUFFER 8192

esp_err_t uart_io_init(void)
{
    const uart_config_t config = {
        .baud_rate = BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(PORT, RX_BUFFER, TX_BUFFER, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    return uart_param_config(PORT, &config);
}

void uart_io_write(const void *data, size_t len)
{
    uart_write_bytes(PORT, data, len);
}

void uart_io_printf(const char *fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n < 0) {
        return;
    }
    if ((size_t)n >= sizeof(line)) {
        n = sizeof(line) - 1;
    }
    uart_io_write(line, (size_t)n);
}

int uart_io_getc(void)
{
    uint8_t c;
    return uart_read_bytes(PORT, &c, 1, 0) == 1 ? c : -1;
}
```

- [ ] **Step 2: Write the gate**

`firmware/main/gate.h`:
```c
/* Invariant 8: nothing renders between arm and follow-through.
 *
 * While the gate is armed the app task does not call lv_timer_handler(), so
 * LVGL neither renders nor reads touch, and the touch controller's I2C
 * traffic stops (parent spec 6.4). A console command arms it for now; the
 * ported stroke state machine will arm it later, through the same call. A
 * panel transfer already on the SPI bus when the gate arms runs to completion
 * by DMA; it touches neither core 0 nor the I2C bus. */
#ifndef GATE_H
#define GATE_H

#include <stdbool.h>

void gate_arm(void);
void gate_open(void);
bool gate_armed(void);

#endif /* GATE_H */
```

`firmware/main/gate.c`:
```c
#include "gate.h"

/* Atomic, because the state machine that will arm it runs on the other core. */
static bool s_armed;

void gate_arm(void)
{
    __atomic_store_n(&s_armed, true, __ATOMIC_SEQ_CST);
}

void gate_open(void)
{
    __atomic_store_n(&s_armed, false, __ATOMIC_SEQ_CST);
}

bool gate_armed(void)
{
    return __atomic_load_n(&s_armed, __ATOMIC_SEQ_CST);
}
```

- [ ] **Step 3: Write the stream**

`firmware/main/stream.h`:
```c
/* Drains the acquisition ring on the app task: sample accounting since boot,
 * the jitter histograms, and the stream to the host in imu_stream's framing. */
#ifndef STREAM_H
#define STREAM_H

#include <stdbool.h>

void stream_init(void);
/* Drain everything in the ring. Call every pass of the app loop. */
void stream_service(void);

void stream_toggle(void);
bool stream_running(void);
void stream_set_binary(bool binary);
bool stream_binary(void);

/* Print the jitter window since the last report, then start a new one. */
void stream_jitter_report(void);
void stream_status(void);

#endif /* STREAM_H */
```

`firmware/main/stream.c`:
```c
#include "stream.h"

#include <string.h>

#include "acq/acq.h"
#include "acq/frame.h"
#include "acq/jitter.h"
#include "acq/seqcount.h"
#include "esp_timer.h"
#include "gate.h"
#include "uart_io.h"

static acq_ring *s_ring;

/* Since boot: every record, whether or not anything is streaming. */
static acq_seqcount s_boot;
static uint32_t s_prev_index;
static uint32_t s_prev_edge_us;
static bool s_have_prev;

/* The jitter window. Two histograms (8 KB each, internal RAM). */
static acq_hist s_interval;
static acq_hist s_latency;
static uint32_t s_window_start_us;
static bool s_window_armed;
static bool s_window_mixed;

/* The stream to the host. */
static bool s_streaming;
static bool s_binary;
static acq_seqcount s_stream;
static uint32_t s_stream_start_us;
static acq_record s_pending[ACQ_FRAME_MAX_SAMPLES];
static uint8_t s_pending_n;
static uint32_t s_pending_first;
static uint32_t s_dropped_seen;

static void window_reset(void)
{
    acq_hist_reset(&s_interval);
    acq_hist_reset(&s_latency);
    s_window_start_us = (uint32_t)esp_timer_get_time();
    s_window_armed = gate_armed();
    s_window_mixed = false;
}

void stream_init(void)
{
    s_ring = acq_ring_handle();
    acq_seqcount_reset(&s_boot);
    window_reset();
}

static void flush_frame(void)
{
    if (s_pending_n == 0) {
        return;
    }
    const uint32_t dropped = acq_ring_dropped(s_ring);
    uint8_t flags = ACQ_FRAME_FLAG_SENSOR_SEQ;
    if (dropped != s_dropped_seen) {
        flags |= ACQ_FRAME_FLAG_OVERFLOW;
        s_dropped_seen = dropped;
    }
    uint8_t out[ACQ_FRAME_BYTES(ACQ_FRAME_MAX_SAMPLES)];
    const size_t len = acq_frame_encode(out, s_pending_first, flags,
                                        s_pending[0].edge_us, s_pending,
                                        s_pending_n);
    uart_io_write(out, len);
    s_pending_n = 0;
}

static void emit(const acq_record *r)
{
    uint32_t index;
    if (acq_seqcount_update(&s_stream, r->timestamp, &index) != ACQ_SEQ_NEW) {
        return; /* counted, not sent: a repeat is not a measurement */
    }
    if (!s_binary) {
        uart_io_printf("%lu,%d,%d,%d,%d,%d,%d\n", (unsigned long)index,
                       r->accel[0], r->accel[1], r->accel[2],
                       r->gyro[0], r->gyro[1], r->gyro[2]);
        return;
    }
    /* A frame's samples are numbered consecutively from first_seq, so a gap
     * ends the frame: the host sees the gap as a gap. */
    if (s_pending_n > 0 && (index != s_pending_first + s_pending_n ||
                            s_pending_n == ACQ_FRAME_MAX_SAMPLES)) {
        flush_frame();
    }
    if (s_pending_n == 0) {
        s_pending_first = index;
    }
    s_pending[s_pending_n++] = *r;
}

void stream_service(void)
{
    if (gate_armed() != s_window_armed) {
        s_window_mixed = true;
    }
    acq_record r;
    while (acq_ring_pop(s_ring, &r)) {
        uint32_t index;
        if (acq_seqcount_update(&s_boot, r.timestamp, &index) == ACQ_SEQ_NEW) {
            /* Edge-to-edge only across consecutive samples: a lost sample
             * would read as a doubled interval, and loss is counted apart. */
            if (s_have_prev && index == s_prev_index + 1) {
                acq_hist_add(&s_interval, r.edge_us - s_prev_edge_us);
            }
            acq_hist_add(&s_latency, r.done_us - r.edge_us);
            s_prev_index = index;
            s_prev_edge_us = r.edge_us;
            s_have_prev = true;
        }
        if (s_streaming) {
            emit(&r);
        }
    }
    flush_frame();
}

void stream_toggle(void)
{
    flush_frame();
    s_streaming = !s_streaming;
    acq_seqcount_reset(&s_stream);
    s_pending_n = 0;
    s_dropped_seen = acq_ring_dropped(s_ring);
    s_stream_start_us = (uint32_t)esp_timer_get_time();
    uart_io_printf("# streaming %d\n", s_streaming);
}

bool stream_running(void)
{
    return s_streaming;
}

void stream_set_binary(bool binary)
{
    s_binary = binary;
}

bool stream_binary(void)
{
    return s_binary;
}

static void print_hist(const char *name, const acq_hist *h)
{
    char p[3][12];
    const uint32_t ranks[3] = {500, 990, 999};
    for (int i = 0; i < 3; i++) {
        const uint32_t v = acq_hist_percentile(h, ranks[i]);
        if (v == ACQ_HIST_UNKNOWN) {
            strcpy(p[i], h->count ? ">=2048" : "-");
        } else {
            snprintf(p[i], sizeof(p[i]), "%lu", (unsigned long)v);
        }
    }
    uart_io_printf("# %s n=%lu min=%lu p50=%s p99=%s p99.9=%s max=%lu over2048=%lu\n",
                   name, (unsigned long)h->count,
                   (unsigned long)(h->count ? h->min : 0), p[0], p[1], p[2],
                   (unsigned long)h->max, (unsigned long)h->over);
}

void stream_jitter_report(void)
{
    const uint32_t now = (uint32_t)esp_timer_get_time();
    const char *gate = s_window_mixed ? "CHANGED DURING WINDOW"
                       : s_window_armed ? "armed throughout" : "open throughout";
    uart_io_printf("# jitter window %.2f s, gate %s\n",
                   (now - s_window_start_us) / 1e6, gate);
    print_hist("interval_us", &s_interval);
    print_hist("latency_us", &s_latency);
    window_reset();
}

void stream_status(void)
{
    acq_counters c;
    acq_counters_get(&c);
    const uint32_t elapsed = (uint32_t)esp_timer_get_time() - s_stream_start_us;
    const uint32_t produced = acq_seqcount_produced(&s_stream);
    uart_io_printf("# plumb firmware: format=%s streaming=%d gate=%s\n",
                   s_binary ? "binary" : "csv", s_streaming,
                   gate_armed() ? "armed" : "open");
    uart_io_printf("# since boot: produced=%lu lost=%lu duplicated=%lu backward=%lu ring_dropped=%lu\n",
                   (unsigned long)acq_seqcount_produced(&s_boot),
                   (unsigned long)s_boot.lost, (unsigned long)s_boot.duplicated,
                   (unsigned long)s_boot.backward,
                   (unsigned long)acq_ring_dropped(s_ring));
    uart_io_printf("# acq: reads=%lu read_errors=%lu not_available=%lu unlocked=%lu missed_edges=%lu drdy_timeouts=%lu\n",
                   (unsigned long)c.reads, (unsigned long)c.read_errors,
                   (unsigned long)c.not_available, (unsigned long)c.unlocked,
                   (unsigned long)c.missed_edges, (unsigned long)c.drdy_timeouts);
    if (s_streaming && elapsed > 0) {
        uart_io_printf("# stream: produced=%lu lost=%lu produced_hz=%.2f\n",
                       (unsigned long)produced, (unsigned long)s_stream.lost,
                       produced * 1e6 / elapsed);
    }
}
```

- [ ] **Step 4: Write the UI port**

`firmware/main/ui_port.h`:
```c
/* LVGL, the GC9A01 panel and the CST816 touch controller, glued to plumb_ui.
 * Runs on the app task, core 1. Serviced only while the gate is open. */
#ifndef UI_PORT_H
#define UI_PORT_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t ui_port_init(void);
/* lv_timer_handler(), unless the gate is armed. */
void ui_port_service(void);

void ui_port_example_result(void);
void ui_port_next(void);
void ui_port_prev(void);
/* Cycle the result screens every 100 ms: the jitter test's UI load. */
void ui_port_set_exercise(bool on);
bool ui_port_exercise(void);
void ui_port_status(void);

#endif /* UI_PORT_H */
```

`firmware/main/ui_port.c`:
```c
#include "ui_port.h"

#include "board/cst816.h"
#include "board/gc9a01.h"
#include "board/i2c_bus.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "gate.h"
#include "lvgl.h"
#include "plumb/ui.h"
#include "uart_io.h"

#define W 240
#define H 240
/* Parent spec 6.3: 2 x (240 x 60) RGB565, internal DMA-capable SRAM. */
#define BUF_LINES 60
#define BUF_BYTES (W * BUF_LINES * 2)
#define SPI_HZ (80u * 1000u * 1000u)
#define EXERCISE_MS 100

static lv_display_t *s_display;
static lv_timer_t *s_exercise;
static bool s_exercising;
static bool s_touch_ok;

static void flush_done(void *ctx)
{
    (void)ctx;
    /* From the SPI DMA interrupt: the buffer is LVGL's again. */
    lv_display_flush_ready(s_display);
}

static void flush(lv_display_t *display, const lv_area_t *area, uint8_t *pixels)
{
    (void)display;
    /* LVGL renders little-endian RGB565; the panel reads big-endian. Swapped
     * here, not in lv_conf.h, which the host bench shares. */
    lv_draw_rgb565_swap(pixels, lv_area_get_size(area));
    gc9a01_draw(area->x1, area->y1, area->x2, area->y2, pixels);
}

static uint32_t tick(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    uint16_t x, y;
    /* lv_timer_handler() is not called under the gate, so this cannot run
     * then. If that ever changes, the bus still stays the IMU's. */
    if (!gate_armed() && s_touch_ok && cst816_read(&x, &y)) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void gesture(lv_event_t *event)
{
    (void)event;
    switch (lv_indev_get_gesture_dir(lv_indev_active())) {
    case LV_DIR_LEFT:
        pl_ui_next();
        uart_io_printf("# gesture left: screen %d\n", pl_ui_current_screen());
        break;
    case LV_DIR_RIGHT:
        pl_ui_prev();
        uart_io_printf("# gesture right: screen %d\n", pl_ui_current_screen());
        break;
    default:
        break;
    }
}

static void exercise_step(lv_timer_t *timer)
{
    (void)timer;
    pl_ui_next();
}

esp_err_t ui_port_init(void)
{
    uint8_t chip = 0;
    esp_err_t err = cst816_init(board_i2c_bus(), &chip);
    s_touch_ok = (err == ESP_OK);
    uart_io_printf("# touch: CST816 %s, chip id 0x%02X\n",
                   s_touch_ok ? "answered" : "DID NOT ANSWER", chip);

    /* Called on core 1, so the SPI interrupt is allocated here too. */
    err = gc9a01_init(SPI_HZ, GC9A01_MADCTL_MX | GC9A01_MADCTL_BGR, true,
                      BUF_BYTES, flush_done, NULL);
    if (err != ESP_OK) {
        return err;
    }

    lv_init();
    lv_tick_set_cb(tick);
    s_display = lv_display_create(W, H);
    /* Invariant 7: draw buffers in internal SRAM, never PSRAM. */
    void *buf1 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (buf1 == NULL || buf2 == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_buffers(s_display, buf1, buf2, BUF_BYTES,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(s_display, flush);

    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read);

    pl_ui_init(s_display);
    lv_obj_add_event_cb(lv_screen_active(), gesture, LV_EVENT_GESTURE, NULL);

    s_exercise = lv_timer_create(exercise_step, EXERCISE_MS, NULL);
    lv_timer_pause(s_exercise);
    return ESP_OK;
}

void ui_port_service(void)
{
    if (!gate_armed()) {
        lv_timer_handler();
    }
}

void ui_port_example_result(void)
{
    /* The bring-up's example (display.ino), so the two can be compared. */
    pl_ui_result r = {0};
    r.face_valid = true;
    r.face_angle_deg = 1.8f;
    r.tempo_valid = true;
    r.backswing_s = 0.70f;
    r.downswing_s = 0.34f;
    r.path_valid = true;
    r.path_dir = PL_PATH_OUT_TO_IN;
    r.path_arc_m = 0.006f;
    r.path_travel_m = 0.30f;
    r.speed_valid = true;
    r.impact_speed_mps = 1.62f;
    pl_ui_show_result(&r);
}

void ui_port_next(void)
{
    pl_ui_next();
}

void ui_port_prev(void)
{
    pl_ui_prev();
}

void ui_port_set_exercise(bool on)
{
    s_exercising = on;
    if (on) {
        ui_port_example_result(); /* cycling needs result screens to cycle */
        lv_timer_resume(s_exercise);
    } else {
        lv_timer_pause(s_exercise);
    }
}

bool ui_port_exercise(void)
{
    return s_exercising;
}

void ui_port_status(void)
{
    uart_io_printf("# ui: screen %d, missing glyphs %lu, exercise %s, touch %s\n",
                   pl_ui_current_screen(), (unsigned long)pl_ui_missing_glyphs(),
                   s_exercising ? "on" : "off", s_touch_ok ? "ok" : "absent");
    uart_io_printf("# heap: internal free %u B, PSRAM free %u B\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
```

- [ ] **Step 5: Write app_main and the console**

`firmware/main/app_main.c`:
```c
/* Plumb product firmware: the skeleton
 * (docs/superpowers/specs/2026-09-27-firmware-skeleton-design.md).
 *
 * Core 0: the acquisition task, woken by the IMU's data-ready edge.
 * Core 1: this app task. It drains the ring, streams to the host, and runs
 * LVGL only while the invariant-8 gate is open.
 *
 * Console, single characters at 921600 (imu_stream's letters where they
 * overlap, so tools/capture.py works unchanged):
 *   s  start / stop streaming      b / c  binary / CSV
 *   a  arm the gate                o      open the gate
 *   x  toggle the UI exercise      j      jitter report, then a new window
 *   r  example result   n / p  next / previous screen      ?  status
 *   1, d  acknowledged: the only rate and read path this firmware has
 *   9, f  refused: max rate and the FIFO path are not on this firmware
 */

#include "acq/acq.h"
#include "board/i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gate.h"
#include "stream.h"
#include "uart_io.h"
#include "ui_port.h"

#define APP_CORE 1
#define APP_PRIORITY 5
#define APP_STACK 16384

static esp_err_t s_imu_result = ESP_FAIL;
static esp_err_t s_ui_result = ESP_FAIL;

static void status(void)
{
    stream_status();
    uart_io_printf("# imu %s, ui %s\n", esp_err_to_name(s_imu_result),
                   esp_err_to_name(s_ui_result));
    if (s_ui_result == ESP_OK) {
        ui_port_status();
    }
}

static void console(char c)
{
    const bool ui = (s_ui_result == ESP_OK);
    switch (c) {
    case 's': stream_toggle(); break;
    case 'b': stream_set_binary(true); uart_io_printf("# format binary\n"); break;
    case 'c': stream_set_binary(false); uart_io_printf("# format csv\n"); break;
    case '1': uart_io_printf("# rate stroke\n"); break;
    case 'd': uart_io_printf("# path direct\n"); break;
    case '9': uart_io_printf("# rate max UNSUPPORTED on this firmware\n"); break;
    case 'f': uart_io_printf("# path fifo UNSUPPORTED on this firmware\n"); break;
    case 'a': gate_arm(); uart_io_printf("# gate armed\n"); break;
    case 'o': gate_open(); uart_io_printf("# gate open\n"); break;
    case 'j': stream_jitter_report(); break;
    case '?': status(); break;
    case 'x':
        if (ui) {
            ui_port_set_exercise(!ui_port_exercise());
            uart_io_printf("# exercise %s\n", ui_port_exercise() ? "on" : "off");
        }
        break;
    case 'r':
        if (ui) {
            ui_port_example_result();
            uart_io_printf("# result shown\n");
        }
        break;
    case 'n': if (ui) ui_port_next(); break;
    case 'p': if (ui) ui_port_prev(); break;
    default: break;
    }
}

static void app_task(void *arg)
{
    (void)arg;
    s_ui_result = ui_port_init();
    if (s_ui_result != ESP_OK) {
        uart_io_printf("# UI init failed: %s\n", esp_err_to_name(s_ui_result));
    }
    stream_init();
    status();
    uart_io_printf("# ready: s b c a o x j r n p ?\n");
    for (;;) {
        int c;
        while ((c = uart_io_getc()) >= 0) {
            console((char)c);
        }
        stream_service();
        if (s_ui_result == ESP_OK) {
            ui_port_service();
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    /* Nothing but this task's own output on the wire once it is up. */
    esp_log_level_set("*", ESP_LOG_WARN);
    ESP_ERROR_CHECK(uart_io_init());
    uart_io_printf("\n# plumb firmware skeleton\n");

    /* app_main runs on core 0: the bus interrupt and, inside acq_start, the
     * DRDY interrupt are allocated here, beside the acquisition task. */
    s_imu_result = board_i2c_init();
    if (s_imu_result == ESP_OK) {
        s_imu_result = acq_start();
    }
    if (s_imu_result != ESP_OK) {
        uart_io_printf("# IMU init failed: %s\n", esp_err_to_name(s_imu_result));
    }
    xTaskCreatePinnedToCore(app_task, "app", APP_STACK, NULL, APP_PRIORITY,
                            NULL, APP_CORE);
}
```

`firmware/main/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "app_main.c" "gate.c" "stream.c" "ui_port.c" "uart_io.c"
    INCLUDE_DIRS "."
    REQUIRES acq board plumb_ui lvgl esp_driver_uart esp_timer)
```

- [ ] **Step 6: Commit** (compiled in Task 7)

```bash
git add firmware/main
git commit -m "Add the app task: stream, jitter windows, the gate, UI port, console"
```

---

### Task 7: The IDF project and the first build

**Files:**
- Create: `firmware/CMakeLists.txt`, `firmware/sdkconfig.defaults`, `firmware/idf.ps1`,
  `firmware/components/plumb_ui/CMakeLists.txt`
- Generated, committed: `firmware/sdkconfig`

- [ ] **Step 1: Write the project files**

`firmware/CMakeLists.txt`:
```cmake
# Plumb product firmware, ESP-IDF v5.5.5.
#
#   ./idf.ps1 build        ./idf.ps1 -p COM4 flash
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/third_party/lvgl")
# Build only what main pulls in. esp_wifi and bt are not compiled at all
# (invariant 6); tools/check_no_radio.py below proves it on every link.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# One LVGL configuration, host and device: plumb_ui/lv_conf.h. LVGL's Kconfig
# is ignored so options lv_conf.h does not list take the same defaults here
# as on the host bench, not Kconfig's.
idf_build_set_property(COMPILE_DEFINITIONS "LV_KCONFIG_IGNORE" APPEND)
idf_build_set_property(COMPILE_OPTIONS
    "-I${CMAKE_CURRENT_LIST_DIR}/components/plumb_ui" APPEND)

project(plumb)

idf_build_get_property(python PYTHON)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD
    COMMAND ${python} ${CMAKE_CURRENT_LIST_DIR}/tools/check_no_radio.py
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.map
    VERBATIM)
```

`firmware/sdkconfig.defaults`:
```
# The one firmware configuration, and the one that gets validated
# (invariant 6). sdkconfig, generated from this, is committed alongside it.
CONFIG_IDF_TARGET="esp32s3"

# Parent spec 14.2: 16 MB flash, quad PSRAM (the R2 part; not octal).
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y

# Parent spec 6.3.
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_COMPILER_OPTIMIZATION_PERF=y
CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y

# 1 ms ticks: the app loop's vTaskDelay(1) is one millisecond.
CONFIG_FREERTOS_HZ=1000

# The console at imu_stream's rate, so capture.py and the boot log share a baud.
CONFIG_ESP_CONSOLE_UART_BAUDRATE=921600
CONFIG_LOG_DEFAULT_LEVEL_WARN=y
CONFIG_BOOTLOADER_LOG_LEVEL_WARN=y

# LVGL builds from plumb_ui/lv_conf.h (LV_KCONFIG_IGNORE in CMakeLists.txt);
# these two are read by LVGL's CMake, not its headers.
CONFIG_LV_BUILD_EXAMPLES=n
CONFIG_LV_BUILD_DEMOS=n
```

`firmware/idf.ps1`:
```powershell
# Run idf.py for this project with ESP-IDF v5.5.5's environment.
#   ./idf.ps1 build
#   ./idf.ps1 -p COM4 flash
# v5.5.5's export script needs a Python 3.11 first on PATH; uv's is used.
param([Parameter(ValueFromRemainingArguments = $true)] $IdfArgs)
$py = (Get-ChildItem "$env:APPDATA\uv\python" -Directory |
       Where-Object { $_.Name -like 'cpython-3.11*' } | Select-Object -First 1).FullName
if (-not $py) { throw "No uv Python 3.11 under $env:APPDATA\uv\python" }
$env:PATH = "$py;$env:PATH"
. "$env:USERPROFILE\esp\v5.5.5\export.ps1" | Out-Null
Push-Location $PSScriptRoot
try { idf.py @IdfArgs; exit $LASTEXITCODE } finally { Pop-Location }
```

`firmware/components/plumb_ui/CMakeLists.txt`:
```cmake
# The UI as an ESP-IDF component. The same sources build on the host
# (analysis/tools/cbuild.py build_ui) with no ESP-IDF at all: the UI knows
# nothing about hardware (CLAUDE.md), and that is kept true by building it
# both ways.
file(GLOB FONTS "${CMAKE_CURRENT_LIST_DIR}/fonts/*.c")
idf_component_register(
    SRCS "src/geometry.c" "src/style.c" "src/ui.c" "src/screen_face.c"
         "src/screen_tempo.c" "src/screen_path.c" "src/screen_speed.c" ${FONTS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src" "fonts" "."
    REQUIRES lvgl)
```

- [ ] **Step 2: Build**

Run (PowerShell, from `firmware/`): `./idf.ps1 set-target esp32s3` then `./idf.ps1 build`
Expected: `Project build complete`, and the post-build line
`check_no_radio: no Wi-Fi or Bluetooth code in plumb.map`. Fix compile errors as they come,
without changing behaviour. Any warning promoted to an error in our code is fixed, not
suppressed.

- [ ] **Step 3: Show the map check fires on a real radio build**

The unit test proves the check on a map excerpt. This proves it against a map the real linker
wrote. Temporarily add `esp_wifi` to `REQUIRES` in `main/CMakeLists.txt`, add
`#include "esp_wifi.h"` to `app_main.c`, and call `esp_wifi_init(NULL);` inside a branch that
never runs, such as `if (s_imu_result == 12345) {}`. Build.
Expected: the build FAILS with `INVARIANT 6 VIOLATED` followed by lines that name `libesp_wifi.a`
or `esp_wifi_init`. Revert both files with `git checkout firmware/main`, rebuild, and confirm the
clean line returns. Record the offending lines in the report (Task 10).

- [ ] **Step 4: Commit**

```bash
git add firmware/CMakeLists.txt firmware/sdkconfig.defaults firmware/sdkconfig firmware/idf.ps1 firmware/components/plumb_ui/CMakeLists.txt
git commit -m "Stand up the ESP-IDF project: one configuration, radio-free by construction"
```

---

### Task 8: Flash and check the basics on the board

- [ ] **Step 1: Flash**

Run: `./idf.ps1 -p COM4 flash` (this replaces the display bring-up sketch on the board).

- [ ] **Step 2: Read the boot and the status**

Run (from `analysis/`): `uv run python -m tools.board_ui send "?" --port COM4 --baud 921600 --seconds 2`
Expected: `imu ESP_OK, ui ESP_OK`, touch answered with chip id 0xB5, `reads` climbing between two
`?`, `read_errors=0 not_available=0 missed_edges=0 drdy_timeouts=0`, since-boot `lost=0`.
If `drdy_timeouts` climbs and `reads` stays 0, DRDY is not reaching GPIO3. Check CTRL7
and FIFO_CTRL before anything else.

- [ ] **Step 3: Screens and swipes, with Will at the board**

`r`, then `n` and `p`, then swipes left and right. The screens must match the display bring-up:
text the right way round, colours right, and swipes changing screens (the console logs
`# gesture left: screen N`).

---

### Task 9: Measure

- [ ] **Step 1: Loss over 60 s, through capture.py unchanged**

Run (from `analysis/`): `uv run python -m tools.capture --port COM4 --read-path direct --seconds 60 --out ../data/skeleton-60s.npy`
Expected: `dropped   : 0 (0.0% of what the sensor produced -- sequence numbers come from the
sensor's own counter)` and a produced ODR near 906.86 Hz. Follow with `?`: since-boot `lost=0
duplicated=0 backward=0 ring_dropped=0`.

- [ ] **Step 2: Jitter, UI animating**

Send `x`, then `j` to open a window, wait 30 s, then send `j` to report and `x` to stop:
`uv run python -m tools.board_ui send xj --port COM4 --baud 921600 --seconds 31` then
`uv run python -m tools.board_ui send jx --port COM4 --baud 921600 --seconds 2`
Expected: a report headed `gate open throughout`, with interval and latency lines.

- [ ] **Step 3: Jitter, gate armed**

`send aj` (31 s), then `send jo` (2 s).
Expected: a report headed `gate armed throughout`. Both reports go into
`docs/bringup-results.md` as measured, with no pass or fail threshold attached.

- [ ] **Step 4: Loss while animating**

Repeat Step 1 with the exercise on (`send x` first; capture's connect sequence does not touch it),
then `send x` to stop. Expected: dropped 0. If not, that is the finding.

---

### Task 10: Record and amend

**Files:**
- Modify: `docs/superpowers/specs/2026-09-15-putting-analyzer-design.md` (§6.2 amendment)
- Modify: `docs/superpowers/specs/2026-09-27-firmware-skeleton-design.md` (console letters; decisions)
- Modify: `docs/bringup-results.md` (skeleton section: loss, jitter, map check, screens)
- Modify: `HANDOFF.md`, `CLAUDE.md` (amendment count), `docs/roadmap.md` (week 5 code track)

- [ ] **Step 1: Amend parent §6.2**, under the core-0 bullet, in the file's amendment style:
"*Amended 2026-09-27.* This previously read 'Hardware-timer driven'. A timer runs on the
ESP32's clock and the IMU on its own; this unit's IMU runs at 906.86 Hz against a nominal 896.8,
so a timer at the nominal rate would duplicate or miss about one sample in a hundred.
Acquisition is paced by the IMU's data-ready line instead (INT2 → GPIO3), with the SyncSample lock
so that a 500 µs I²C read cannot be torn (datasheet rev A §6.1, §6.3, §13.2). Measured: …" and
then the Task 9 numbers.
- [ ] **Step 2:** Update the skeleton spec's console section (`a`/`o` for the gate, `x`, and the
`1 d 9 f` handling) and add the decisions list from the top of this plan, including the §6.3
flash and `LV_MEMCPY_MEMSET_STD` disagreements.
- [ ] **Step 3:** Write the results into `bringup-results.md`, and update `HANDOFF.md` with what is
built, what is measured and what is next. Update the amendment count in `CLAUDE.md` to seven.
- [ ] **Step 4: Run the whole suite and commit**

Run: `uv run pytest -q` (from `analysis/`). Expected: all pass.
```bash
git add docs HANDOFF.md CLAUDE.md
git commit -m "Record the firmware skeleton's measurements, amend 6.2 for data-ready pacing"
```

---

## Self-review

- Spec coverage. Done-means 1 (loss) is Task 9 Step 1. Done-means 2 (jitter) is Task 9 Steps
  2–3. Done-means 3 (screens) is Task 8 Step 3. Done-means 4 (map check) is Tasks 2 and 7. Done-means 5
  (§6.2 amendment) is Task 10. The architecture's components are Tasks 3–7. The console is Task
  6, with the letter change recorded. The shared-bus rule is in `gate.h`, `ui_port.c` and
  `i2c_bus.h`.
- The names are consistent across tasks: `acq_ring_handle`, `acq_counters_get`,
  `acq_seqcount_produced`, `ACQ_FRAME_BYTES`, `gate_armed`, `ui_port_*` and `stream_*`.
- Not host-testable, and said so: the ring's cross-core memory ordering, the driver register
  sequences and the task timing. Those are what the on-board measurements in Task 9 check.
