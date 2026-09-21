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
