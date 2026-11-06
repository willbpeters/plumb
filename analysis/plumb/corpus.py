"""Read the device's corpus log (parent spec §11 as amended; the format is in
docs/superpowers/specs/2026-09-29-stroke-logging-design.md).

Written from that document, not from firmware/components/rec, so that the
round trip in tests/test_rec.py checks two readings of the format against each
other rather than one against itself.

The result is raw int16 counts, exactly as the sensor produced them: the same
arrays `plumb.sensor.simulate` makes, so the pipeline runs on either.
"""

import binascii
import struct
from dataclasses import dataclass, field

import numpy as np

PAGE = 256
SESSION_MAGIC = 0x534D4C50
SESSION_BYTES = 100
BLOCK_MAGIC = 0x4B42
BLOCK_HEADER_BYTES = 28
AXES = 6
RICE_ESCAPE = 24
RICE_RAW_BITS = 17

_SESSION = struct.Struct("<IHHIIddHHBBH6f32sI")
_BLOCK = struct.Struct("<HHIBB6B6h")
assert _SESSION.size == SESSION_BYTES and _BLOCK.size == BLOCK_HEADER_BYTES


@dataclass
class Run:
    """Consecutive samples: no index gap anywhere inside."""
    first_index: int
    accel_counts: np.ndarray  # (n, 3) int16
    gyro_counts: np.ndarray   # (n, 3) int16

    def __len__(self) -> int:
        return len(self.accel_counts)


@dataclass
class Session:
    offset: int
    number: int
    start_ms: int
    rate_hz: float
    rate_se_hz: float
    gyro_dps: int
    accel_g: int
    putter_id: int
    calibrated: bool
    accel_offset_mps2: np.ndarray
    accel_gain: np.ndarray
    firmware: str
    runs: list[Run] = field(default_factory=list)
    # Ended by a record that failed its check rather than by erased flash:
    # power was lost while it was being written.
    torn: bool = False

    @property
    def samples(self) -> int:
        return sum(len(r) for r in self.runs)

    @property
    def lost(self) -> int:
        """Samples the sensor produced inside this session that are not here."""
        return sum(b.first_index - (a.first_index + len(a))
                   for a, b in zip(self.runs, self.runs[1:]))


def _u32(image: bytes, pos: int) -> int | None:
    return struct.unpack_from("<I", image, pos)[0] if pos + 4 <= len(image) else None


def _crc_ok(record: bytes) -> bool:
    return binascii.crc32(record[:-4]) == struct.unpack_from("<I", record, len(record) - 4)[0]


def _session(image: bytes, pos: int) -> Session | None:
    record = image[pos:pos + SESSION_BYTES]
    if len(record) < SESSION_BYTES or not _crc_ok(record):
        return None
    (magic, version, length, number, start_ms, rate, se, gyro_dps, accel_g,
     putter, calibrated, _, *rest) = _SESSION.unpack(record)
    offsets, gains, firmware = rest[0:3], rest[3:6], rest[6]
    if magic != SESSION_MAGIC or version != 1 or length != SESSION_BYTES:
        return None
    return Session(pos, number, start_ms, rate, se, gyro_dps, accel_g, putter,
                   bool(calibrated), np.array(offsets), np.array(gains),
                   firmware.rstrip(b"\0").decode("ascii", "replace"))


def _read_codes(payload: bytes, ks: tuple[int, ...], n: int) -> np.ndarray:
    """(AXES, n) zigzag values: for each axis in turn, n Rice codes, MSB first."""
    bits = np.unpackbits(np.frombuffer(payload, dtype=np.uint8)).tolist()
    pos = 0

    def take(width: int) -> int:
        nonlocal pos
        value = 0
        for b in bits[pos:pos + width]:
            value = value << 1 | b
        pos += width
        return value

    out = np.zeros((AXES, n), dtype=np.int64)
    for axis, k in enumerate(ks):
        for i in range(n):
            q = 0
            while q < RICE_ESCAPE and bits[pos]:
                q += 1
                pos += 1
            if q == RICE_ESCAPE:
                out[axis, i] = take(RICE_RAW_BITS)
            else:
                pos += 1  # the zero that ends the unary quotient
                out[axis, i] = q << k | take(k)
    return out


def _block(image: bytes, pos: int):
    """(first_index, samples (count, 6) int16, record length) or None."""
    if pos + BLOCK_HEADER_BYTES + 4 > len(image):
        return None
    magic, payload_len, first, count, _, *rest = _BLOCK.unpack_from(image, pos)
    ks, first_values = tuple(rest[:6]), rest[6:]
    length = BLOCK_HEADER_BYTES + payload_len + 4
    if (magic != BLOCK_MAGIC or not 1 <= count <= 64 or max(ks) > 16
            or pos + length > len(image) or not _crc_ok(image[pos:pos + length])):
        return None
    payload = image[pos + BLOCK_HEADER_BYTES:pos + BLOCK_HEADER_BYTES + payload_len]
    z = _read_codes(payload, ks, count - 1)
    deltas = np.where(z % 2 == 0, z // 2, -(z + 1) // 2)
    samples = np.cumsum(np.concatenate([np.array(first_values)[:, None], deltas],
                                       axis=1), axis=1).T
    return first, samples.astype(np.int16), length


def decode(image: bytes) -> list[Session]:
    """Every session in a dumped log, in the order written."""
    end = len(image.rstrip(b"\xff"))
    sessions: list[Session] = []
    current: Session | None = None
    pos = 0
    while pos < end:
        if _u32(image, pos) == SESSION_MAGIC and pos % PAGE == 0:
            session = _session(image, pos)
            if session is not None:
                sessions.append(session)
                current = session
                pos += SESSION_BYTES
                continue
        block = _block(image, pos) if current is not None else None
        if block is not None:
            first, samples, length = block
            runs = current.runs
            if runs and runs[-1].first_index + len(runs[-1]) == first:
                last = runs[-1]
                last.accel_counts = np.vstack([last.accel_counts, samples[:, :3]])
                last.gyro_counts = np.vstack([last.gyro_counts, samples[:, 3:]])
            else:
                runs.append(Run(first, samples[:, :3].copy(), samples[:, 3:].copy()))
            pos += length
            continue
        # Not a record here. Erased padding after a clean stop is 0xFF; anything
        # else is a record torn by power loss. Either way the next record, if
        # any, is a session on a later page boundary.
        if current is not None and image[pos:pos + 2] != b"\xff\xff":
            current.torn = True
        pos = (pos // PAGE + 1) * PAGE
    return sessions
