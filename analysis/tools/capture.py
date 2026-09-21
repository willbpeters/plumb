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
