"""Host-side capture for the IMU streaming instrument.

Decodes either wire format, verifies sequence continuity, and writes raw int16
samples to disk. Scale factors are imported from `plumb.sensor` rather than
redefined -- counts-to-physical is defined in exactly one place across the whole
project, firmware included.
"""

import re
import struct
from dataclasses import dataclass

SYNC = b"\xa5\x5a"
HEADER = struct.Struct("<IBBI")     # first_seq, count, flags, drain_micros
SAMPLE = struct.Struct("<6h")
FLAG_OVERFLOW = 0x01
FLAG_SENSOR_SEQ = 0x02


@dataclass
class Batch:
    first_seq: int
    overflow: bool
    drain_micros: int
    samples: list[tuple[int, int, int, int, int, int]]
    # True when the sequence numbers come from the sensor's own sample counter
    # (the direct read path). Then a gap is a sample the part produced and
    # nobody read. False when they count what was delivered (the FIFO path,
    # where that counter is frozen): a gap is a frame lost between the board
    # and the host, and loss inside the part is not visible at all.
    sensor_seq: bool = False


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
            # No sync ahead. Keep the last byte only if it could be the first
            # half of a sync word split across two reads. Keeping it
            # unconditionally means a byte that can never start a frame is fed
            # back in and rescanned on every subsequent read, so the leftover
            # creeps up by one byte per read for the whole capture.
            tail = buffer[max(i, len(buffer) - 1):]
            return batches, tail if tail == SYNC[:1] else b""

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
                             drain_micros, samples,
                             bool(flags & FLAG_SENSOR_SEQ)))
        i = end


_STREAMING = re.compile(r"streaming=(\d)")


def parse_streaming_state(text: str) -> bool | None:
    """Whether the board says it is streaming, from its status line.

    None when no status line has arrived yet. The board's `s` command is a
    toggle and a script cannot see which way it will go: a previous capture
    that ended without stopping the stream leaves it running, and then `s`
    stops it. That failure is silent and looks like a capture -- 0.6 seconds of
    stale frames, decoded out of order, reported as thousands of repeats.
    """
    match = _STREAMING.search(text)
    if match is None:
        return None
    return match.group(1) == "1"


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


@dataclass
class CaptureStats:
    samples: int = 0
    dropped: int = 0
    repeats: int = 0
    overflows: int = 0
    batches: int = 0
    first_micros: int | None = None
    last_micros: int | None = None
    first_batch_count: int = 0
    first_produced: int | None = None
    last_produced: int | None = None
    sensor_seq: bool = False
    _next_seq: int | None = None

    def produced_odr_hz(self) -> float | None:
        """The sensor's own output data rate, in Hz.

        Sequence numbers come from the part's TIMESTAMP counter, which counts
        samples it PRODUCED. Dividing that span by elapsed drain time gives the
        real ODR even while samples are being lost between the sensor and the
        host -- which is what makes it different from measured_odr_hz(), and
        what spec 9.3 actually asks for. A true rate of 902 Hz against an
        assumed 896.8 is a 0.6 percent scale error in every integrated angle,
        and no amount of downstream filtering removes a scale error.

        Both ends of the span are paired with the drain timestamp taken at the
        same instant, so there is no batch-length offset to correct for.
        """
        if not self.sensor_seq:
            return None
        if self.batches < 2 or self.first_produced is None:
            return None
        elapsed = (self.last_micros - self.first_micros) / 1e6
        if elapsed <= 0:
            return None
        return (self.last_produced - self.first_produced) / elapsed

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
        opened. Counting them inflates the rate by a whole batch.
        """
        if self.batches < 2 or self.first_micros is None:
            return None
        elapsed = (self.last_micros - self.first_micros) / 1e6
        if elapsed <= 0:
            return None
        return (self.samples - self.first_batch_count) / elapsed


def accumulate(stats: CaptureStats, batch: Batch) -> None:
    if stats._next_seq is not None:
        if batch.first_seq > stats._next_seq:
            stats.dropped += batch.first_seq - stats._next_seq
        elif batch.first_seq < stats._next_seq:
            # Sequence numbers went backwards, so these samples were already
            # delivered. Counted rather than tolerated: a duplicate is the one
            # kind of bad sample that makes the noise floor look better than it
            # is.
            stats.repeats += stats._next_seq - batch.first_seq
    stats._next_seq = batch.first_seq + len(batch.samples)

    if stats.batches == 0:
        stats.first_batch_count = len(batch.samples)
        stats.first_micros = batch.drain_micros
        stats.first_produced = stats._next_seq
        stats.sensor_seq = batch.sensor_seq
    elif batch.sensor_seq != stats.sensor_seq:
        # The read path changed mid-capture, so the sequence field changed
        # meaning mid-capture. Nothing derived from it spans the join.
        stats.sensor_seq = False
    stats.last_produced = stats._next_seq

    stats.samples += len(batch.samples)
    stats.batches += 1
    if batch.overflow:
        stats.overflows += 1
    stats.last_micros = batch.drain_micros


def main() -> None:
    import argparse
    import sys
    import time

    import numpy as np
    import serial

    from plumb.sensor import FullScale
    from tools import board

    ap = argparse.ArgumentParser(description="Capture IMU samples from the board.")
    ap.add_argument("--port", required=True, help="e.g. COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", default="capture.npy")
    ap.add_argument("--rate", choices=("stroke", "max"), default="stroke",
                    help="stroke = 896.8 Hz, max = 7174.4 Hz (tap test)")
    ap.add_argument("--read-path", choices=("fifo", "direct"), default="fifo",
                    help="fifo = watermark batch drains (spec 6.4); direct = "
                         "poll the output registers, no FIFO. Direct is a "
                         "stroke-rate path only -- one sample costs ~500 us of "
                         "bus time against a 139 us period at max rate.")
    ap.add_argument("--no-start", action="store_true",
                    help="assume the board is already streaming binary")
    ap.add_argument("--verbose", action="store_true",
                    help="per-second diagnostics: raw bytes in, batches decoded, "
                         "leftover size. Distinguishes a board that stopped "
                         "sending from a decoder that stopped decoding.")
    args = ap.parse_args()

    fs = FullScale()
    print(f"# gyro {fs.gyro_dps} dps FS, accel {fs.accel_g} g FS", file=sys.stderr)

    stats = CaptureStats()
    rows: list[tuple[int, ...]] = []
    leftover = b""

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        if not args.no_start:
            # The connect sequence, and the reason it does not trust the port
            # reset, live in tools/board.py.
            board.connect(port, args.rate, args.read_path, binary=True)
            board.start(port)
            print(f"# streaming at {args.rate} rate via {args.read_path} path, "
                  f"capturing {args.seconds:.0f} s", file=sys.stderr)

        started = time.monotonic()
        deadline = started + args.seconds
        raw_bytes = 0
        tick = started
        tick_bytes = 0
        tick_batches = 0

        while time.monotonic() < deadline:
            chunk = port.read(4096)
            raw_bytes += len(chunk)
            tick_bytes += len(chunk)
            leftover += chunk
            batches, leftover = decode_batches(leftover)
            tick_batches += len(batches)
            for batch in batches:
                accumulate(stats, batch)
                rows.extend(batch.samples)

            now = time.monotonic()
            if args.verbose and now - tick >= 1.0:
                # Three numbers, and between them they localise the fault:
                #   bytes/s == 0        -> the BOARD stopped sending
                #   bytes/s > 0, no batches -> the DECODER is stuck
                #   leftover growing    -> decoder waiting on a frame that will
                #                          never complete, i.e. a false sync
                print(f"  t={now - started:5.1f}s  bytes/s={tick_bytes / (now - tick):8.0f}"
                      f"  batches={tick_batches:4d}  leftover={len(leftover):6d}"
                      f"  overflows={stats.overflows}", file=sys.stderr)
                tick, tick_bytes, tick_batches = now, 0, 0

        if not args.no_start:
            board.stop(port)

    np.save(args.out, np.array(rows, dtype=np.int16))

    delivered_odr = stats.measured_odr_hz()
    produced_odr = stats.produced_odr_hz()
    loss = (stats.dropped / (stats.samples + stats.dropped)
            if stats.samples + stats.dropped else 0.0)
    print(f"raw bytes : {raw_bytes}  ({12 * stats.samples} accounted for by samples)")
    print(f"undecoded : {len(leftover)} bytes left in the buffer at the end")
    print(f"samples   : {stats.samples}")
    print(f"batches   : {stats.batches}")
    if stats.sensor_seq:
        print(f"dropped   : {stats.dropped}  ({100 * loss:.1f}% of what the sensor "
              f"produced -- sequence numbers come from the sensor's own counter)")
    else:
        print(f"dropped   : {stats.dropped}  (frames lost between board and host; "
              f"loss INSIDE the part is not measurable on this read path)")
    print(f"repeats   : {stats.repeats}")
    print(f"overflows : {stats.overflows}")
    print(f"delivered rate: {delivered_odr:.2f} Hz" if delivered_odr
          else "delivered rate: not enough data")
    print(f"sensor ODR    : {produced_odr:.2f} Hz" if produced_odr
          else "sensor ODR    : not enough data")
    print(f"written   : {args.out}")


if __name__ == "__main__":
    main()
