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
        opened. Counting them inflates the rate by a whole batch.
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


def main() -> None:
    import argparse
    import sys
    import time

    import numpy as np
    import serial

    from plumb.sensor import FullScale

    ap = argparse.ArgumentParser(description="Capture IMU samples from the board.")
    ap.add_argument("--port", required=True, help="e.g. COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", default="capture.npy")
    ap.add_argument("--rate", choices=("stroke", "max"), default="stroke",
                    help="stroke = 896.8 Hz, max = 7174.4 Hz (tap test)")
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
            # Opening the port toggles DTR/RTS, which resets the ESP32. So the
            # board is rebooting right now and its setup() holds for 2 s before
            # it will accept anything. This is also why you cannot start the
            # stream from the Arduino serial monitor and then run this script:
            # the reset would stop it again.
            print(f"# resetting board, waiting for boot...", file=sys.stderr)
            time.sleep(2.5)
            port.reset_input_buffer()
            port.write(b"9" if args.rate == "max" else b"1")
            time.sleep(0.2)
            port.write(b"b")        # binary framing
            time.sleep(0.2)
            port.write(b"s")        # start streaming
            time.sleep(0.2)
            print(f"# streaming at {args.rate} rate, capturing {args.seconds:.0f} s",
                  file=sys.stderr)

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

    np.save(args.out, np.array(rows, dtype=np.int16))

    odr = stats.measured_odr_hz()
    print(f"raw bytes : {raw_bytes}  ({12 * stats.samples} accounted for by samples)")
    print(f"undecoded : {len(leftover)} bytes left in the buffer at the end")
    print(f"samples   : {stats.samples}")
    print(f"batches   : {stats.batches}")
    print(f"dropped   : {stats.dropped}")
    print(f"overflows : {stats.overflows}")
    print(f"measured ODR: {odr:.2f} Hz" if odr else "measured ODR: not enough data")
    print(f"written   : {args.out}")


if __name__ == "__main__":
    main()
