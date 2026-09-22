"""Golden-vector tests for the wire format.

The firmware encoder is not unit-tested -- that would need a native build with a
stubbed Wire. Instead the format is pinned by these hand-written vectors, so an
encoder mismatch shows up on the first real capture as a sync or checksum
failure, against a decoder already known to be correct.
"""

import struct

import pytest

from tools.capture import (SYNC, Batch, decode_batches, parse_csv_line,
                           parse_streaming_state)


def build_frame(first_seq, samples, overflow=False, drain_micros=12345,
                sensor_seq=False):
    """Hand-build a frame to the spec in the plan. Deliberately does NOT reuse
    the decoder's constants beyond the sync word -- a test that shares an
    encoder with the code under test proves only that it is self-consistent."""
    flags = (1 if overflow else 0) | (2 if sensor_seq else 0)
    body = struct.pack("<IBBI", first_seq, len(samples), flags, drain_micros)
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


def test_a_repeated_sequence_number_is_counted_not_hidden():
    """The direct read path polls a data-ready flag and then reads the output
    registers, so a sample can be read twice if the flag is misread. Firmware
    rejects duplicates by timestamp, but if one ever reaches the host it must
    not pass as data: duplicates pull a standard deviation DOWN, which is the
    direction that looks like a good result."""
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 100, [(0,) * 6] * 4))
    accumulate(stats, Batch(2, False, 200, [(0,) * 6] * 4))
    assert stats.repeats == 2
    assert stats.dropped == 0


def test_produced_and_delivered_rates_separate_the_sensor_from_the_link():
    """Sequence numbers count what the sensor produced; the sample count counts
    what arrived. With a fifth of the samples lost in the part, those are
    different numbers, and only the first one is the ODR."""
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 0, [(0,) * 6] * 100, sensor_seq=True))
    accumulate(stats, Batch(900, False, 1_000_000, [(0,) * 6] * 100, sensor_seq=True))
    assert stats.dropped == 800
    assert stats.measured_odr_hz() == pytest.approx(100.0, rel=1e-3)
    assert stats.produced_odr_hz() == pytest.approx(900.0, rel=1e-3)


def test_produced_rate_equals_delivered_rate_when_nothing_is_lost():
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 0, [(0,) * 6] * 500, sensor_seq=True))
    accumulate(stats, Batch(500, False, 1_000_000, [(0,) * 6] * 500, sensor_seq=True))
    assert stats.produced_odr_hz() == pytest.approx(stats.measured_odr_hz(), rel=1e-9)


def test_produced_odr_is_none_before_two_batches():
    assert CaptureStats().produced_odr_hz() is None


def test_sensor_sequence_flag_is_surfaced():
    """Two different quantities share the sequence field, and which one it is
    changes what a gap means. The flag is the only thing that says which."""
    direct, _ = decode_batches(build_frame(0, [(0,) * 6], sensor_seq=True))
    fifo, _ = decode_batches(build_frame(0, [(0,) * 6], sensor_seq=False))
    assert direct[0].sensor_seq is True
    assert fifo[0].sensor_seq is False


def test_overflow_and_sensor_sequence_flags_are_independent():
    batches, _ = decode_batches(
        build_frame(0, [(0,) * 6], overflow=True, sensor_seq=True))
    assert batches[0].overflow is True
    assert batches[0].sensor_seq is True


def test_no_sensor_ODR_is_reported_when_the_sequence_is_only_a_delivered_count():
    """On the FIFO path the sensor's sample counter is frozen, so sequence
    numbers count what arrived. Dividing that by elapsed time would produce a
    number that looks like an ODR and is really just the delivery rate again --
    the same kind of claim as the "dropped: 0" that hid 20 percent loss."""
    stats = CaptureStats()
    accumulate(stats, Batch(0, False, 0, [(0,) * 6] * 500))
    accumulate(stats, Batch(500, False, 1_000_000, [(0,) * 6] * 500))
    assert stats.measured_odr_hz() == pytest.approx(500.0, rel=1e-3)
    assert stats.produced_odr_hz() is None


def test_streaming_state_is_read_from_the_status_line():
    """The board's start/stop command is a toggle, and a script cannot see
    which way it will go -- the previous capture may have left it streaming.
    Asking rather than assuming is the difference between a 20-second capture
    and 0.6 seconds of stale frames, which is what a blind toggle produced."""
    nl = chr(10)
    assert parse_streaming_state(
        "# format=binary rate=stroke(896.8 Hz) path=direct streaming=1" + nl) is True
    assert parse_streaming_state(
        "# format=csv rate=stroke(896.8 Hz) path=fifo streaming=0" + nl) is False


def test_streaming_state_is_unknown_when_the_status_line_has_not_arrived():
    assert parse_streaming_state("") is None
    assert parse_streaming_state("# ready -- press s to start") is None


def test_streaming_state_is_found_amid_binary_frame_bytes():
    """Status is asked for while frames are in flight, so the line arrives
    surrounded by binary."""
    noise = bytes([0xA5, 0x5A, 0x01, 0x02]).decode("utf-8", "replace")
    assert parse_streaming_state(noise + "streaming=1" + chr(10) + noise) is True


def test_a_trailing_byte_is_only_kept_when_it_could_start_a_sync_word():
    """Leftover is fed back in on the next read, so a byte that cannot begin a
    frame must not be kept: it would be rescanned forever and the buffer would
    creep upward for the whole capture."""
    batches, leftover = decode_batches(bytes([0x11, 0x22, 0x33]))
    assert batches == []
    assert leftover == b""
    batches, leftover = decode_batches(bytes([0x11, 0x22]) + SYNC[:1])
    assert leftover == SYNC[:1]
