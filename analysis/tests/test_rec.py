"""The corpus log: C recorder in, Python decoder out, against the samples fed.

Ground truth is the input itself -- the simulator's int16 counts and the
sample indices, gaps included, that were put there on purpose. The C
(firmware/components/rec) and the decoder (plumb/corpus.py) were written from
the design document separately, so agreeing is evidence about the format and
not one implementation checked against itself.
"""

import subprocess

import numpy as np
import pytest

from plumb import corpus
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import StrokeParams, generate
from tools import cbuild

PAGE = 256
NOISY = SensorParams(gyro_noise_dps=0.28, accel_noise_mps2=0.02, gyro_bias_dps=1.5)


@pytest.fixture(scope="module")
def reccheck():
    try:
        return cbuild.build_reccheck().executable
    except cbuild.CompilerNotFound as missing:
        message = f"THE CORPUS LOG IS NOT VERIFIED HERE: {missing}"
        print(f"\n*** {message}")
        pytest.skip(message)


def start(ms=1234, rate=906.9314, se=0.0003, putter=2, cal=1,
          offset=(0.01, -0.02, 0.03), gain=(1.001, 0.999, 1.002), fw="v0.1-test"):
    return (f"start {ms} {rate!r} {se!r} {putter} {cal} "
            f"{' '.join(map(repr, offset))} {' '.join(map(repr, gain))} {fw}")


def feed(indices, accel, gyro):
    rows = [f"{i} {a[0]} {a[1]} {a[2]} {g[0]} {g[1]} {g[2]}"
            for i, a, g in zip(indices, accel, gyro)]
    return f"feed {len(rows)}\n" + "\n".join(rows)


def run(exe, tmp_path, *commands):
    image = tmp_path / "image.bin"
    out = subprocess.run([str(exe), str(image)], input="\n".join(commands),
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    states = []
    for line in out.stdout.splitlines():
        head, *pairs = line.split()
        states.append((head, dict(zip(pairs[0::2], map(int, pairs[1::2])))))
    return states, image.read_bytes()


def stroke(sensor=NOISY, seed=1, **params):
    out = simulate(generate(StrokeParams(**params)), sensor, seed=seed)
    return out.accel_counts.astype(np.int16), out.gyro_counts.astype(np.int16)


def samples_of(session):
    """(indices, accel, gyro) concatenated over the session's runs."""
    idx = np.concatenate([np.arange(r.first_index, r.first_index + len(r))
                          for r in session.runs])
    accel = np.vstack([r.accel_counts for r in session.runs])
    gyro = np.vstack([r.gyro_counts for r in session.runs])
    return idx, accel, gyro


@pytest.mark.parametrize("sensor", [SensorParams(), NOISY], ids=["clean", "noisy"])
def test_a_stroke_comes_back_bit_for_bit(reccheck, tmp_path, sensor):
    accel, gyro = stroke(sensor)
    indices = np.arange(50_000, 50_000 + len(accel))
    states, image = run(reccheck, tmp_path, "flash 1048576", start(),
                        feed(indices, accel, gyro), "stop")
    [session] = corpus.decode(image)

    assert len(session.runs) == 1 and session.lost == 0 and not session.torn
    got_idx, got_accel, got_gyro = samples_of(session)
    np.testing.assert_array_equal(got_idx, indices)
    np.testing.assert_array_equal(got_accel, accel)
    np.testing.assert_array_equal(got_gyro, gyro)

    stopped = states[-1][1]
    ratio = len(accel) * 12 / stopped["bytes"]
    print(f"\n  {len(accel)} samples, {stopped['bytes']} bytes: {ratio:.2f}x "
          f"against 12 bytes a sample raw")
    # Loose: here to catch a broken choice of k, not to hold a number. The
    # clean stroke is smoother, so it compresses further.
    assert ratio > 1.5


def test_the_session_header_round_trips(reccheck, tmp_path):
    _, image = run(reccheck, tmp_path, "flash 65536",
                   start(ms=987654, rate=906.9314, se=0.00031, putter=7, cal=1,
                         offset=(0.05, 0.2, -0.1), gain=(1.01, 0.99, 1.0),
                         fw="v0.3-4-gabcdef"),
                   feed([5], [[1, 2, 3]], [[4, 5, 6]]), "stop")
    [s] = corpus.decode(image)
    assert (s.number, s.start_ms, s.putter_id, s.calibrated) == (1, 987654, 7, True)
    assert (s.rate_hz, s.rate_se_hz) == (906.9314, 0.00031)  # f64: exact
    assert (s.gyro_dps, s.accel_g) == (256, 16)
    np.testing.assert_allclose(s.accel_offset_mps2, [0.05, 0.2, -0.1], rtol=1e-7)
    np.testing.assert_allclose(s.accel_gain, [1.01, 0.99, 1.0], rtol=1e-7)
    assert s.firmware == "v0.3-4-gabcdef"


@pytest.mark.parametrize("n", [1, 2, 63, 64, 65, 128, 129])
def test_block_boundaries(reccheck, tmp_path, n):
    accel, gyro = stroke()
    accel, gyro = accel[:n], gyro[:n]
    _, image = run(reccheck, tmp_path, "flash 65536", start(),
                   feed(range(n), accel, gyro), "stop")
    [s] = corpus.decode(image)
    _, got_accel, got_gyro = samples_of(s)
    np.testing.assert_array_equal(got_accel, accel)
    np.testing.assert_array_equal(got_gyro, gyro)


@pytest.mark.parametrize("gap_at, gap", [(10, 1), (10, 2), (300, 1000),
                                         # a gap just after a full block closed
                                         (64, 5), (128, 1)])
def test_gaps_are_recorded_exactly(reccheck, tmp_path, gap_at, gap):
    accel, gyro = stroke()
    accel, gyro = accel[:600], gyro[:600]
    indices = np.arange(600) + 7
    indices[gap_at:] += gap
    states, image = run(reccheck, tmp_path, "flash 65536", start(),
                        feed(indices, accel, gyro), "stop")
    [s] = corpus.decode(image)

    assert s.lost == gap
    assert states[-1][1]["lost"] == gap  # the device's own count agrees
    assert [r.first_index for r in s.runs] == [7, 7 + gap_at + gap]
    got_idx, got_accel, _ = samples_of(s)
    np.testing.assert_array_equal(got_idx, indices)
    np.testing.assert_array_equal(got_accel, accel)


def test_full_scale_jumps_take_the_escape(reccheck, tmp_path):
    """A difference of +/-65,535, the largest possible, every sample."""
    n = 200
    swing = np.where(np.arange(n) % 2 == 0, 32767, -32768).astype(np.int16)
    accel = np.stack([swing, -swing - 1, np.zeros(n, np.int16)], axis=1)
    gyro = np.stack([swing, swing, (np.arange(n) * 37 % 65536 - 32768)], axis=1).astype(np.int16)
    _, image = run(reccheck, tmp_path, "flash 65536", start(),
                   feed(range(n), accel, gyro), "stop")
    [s] = corpus.decode(image)
    _, got_accel, got_gyro = samples_of(s)
    np.testing.assert_array_equal(got_accel, accel)
    np.testing.assert_array_equal(got_gyro, gyro)


def test_sessions_are_numbered_and_page_aligned(reccheck, tmp_path):
    accel, gyro = stroke()
    parts = []
    for k in range(3):
        n = 100 + 37 * k
        parts += [start(ms=k), feed(range(n), accel[:n], gyro[:n]), "stop"]
    _, image = run(reccheck, tmp_path, "flash 131072", *parts)
    sessions = corpus.decode(image)
    assert [s.number for s in sessions] == [1, 2, 3]
    assert [s.start_ms for s in sessions] == [0, 1, 2]
    assert all(s.offset % PAGE == 0 for s in sessions)
    assert [s.samples for s in sessions] == [100, 137, 174]


def test_numbering_continues_after_a_power_cycle(reccheck, tmp_path):
    accel, gyro = stroke()
    _, image = run(reccheck, tmp_path, "flash 131072",
                   start(), feed(range(100), accel[:100], gyro[:100]), "stop",
                   "reopen",
                   start(), feed(range(50), accel[:50], gyro[:50]), "stop")
    assert [s.number for s in corpus.decode(image)] == [1, 2]


def test_a_full_log_stops_and_keeps_what_it_wrote(reccheck, tmp_path):
    accel, gyro = stroke(address_duration_s=10.0)
    indices = np.arange(len(accel))
    states, image = run(reccheck, tmp_path, "flash 32768", start(),
                        feed(indices, accel, gyro), "stop")
    after_feed = states[2][1]
    assert after_feed["full"] == 1 and after_feed["recording"] == 0
    [s] = corpus.decode(image)
    got_idx, got_accel, got_gyro = samples_of(s)
    assert 0 < len(got_idx) < len(indices)
    assert len(got_idx) == after_feed["samples"]
    np.testing.assert_array_equal(got_idx, indices[:len(got_idx)])
    np.testing.assert_array_equal(got_accel, accel[:len(got_idx)])
    assert not s.torn


def test_power_lost_mid_write(reccheck, tmp_path):
    """The flash stops mid-page. The first session keeps every block written
    whole; after the power returns, the second session is complete."""
    accel, gyro = stroke()
    first = 1500
    _, image = run(reccheck, tmp_path, "flash 262144",
                   start(), feed(range(first), accel[:first], gyro[:first]),
                   "tear 100",
                   feed(range(first, len(accel)), accel[first:], gyro[first:]),
                   "reopen",
                   start(ms=2), feed(range(400), accel[:400], gyro[:400]), "stop")
    one, two = corpus.decode(image)

    assert one.torn
    got_idx, got_accel, _ = samples_of(one)
    # What had not reached flash when the power went is gone: the open block
    # (under 64 samples) and the staged bytes short of a page (a page holds
    # ~50 samples at this ratio). Here 1500 fed, 1408 kept: 28 + ~64 -- about
    # 0.1 s of a session, at its very end.
    print(f"\n  power lost after {first} samples: {len(got_idx)} kept")
    assert first - 64 - 128 < len(got_idx) <= first
    np.testing.assert_array_equal(got_idx, np.arange(len(got_idx)))
    np.testing.assert_array_equal(got_accel, accel[:len(got_idx)])

    assert (two.number, two.samples, two.torn) == (2, 400, False)
    np.testing.assert_array_equal(samples_of(two)[1], accel[:400])


@pytest.mark.parametrize("n", [0, 1, 100, 1000, 3000])
def test_opening_finds_the_end_a_linear_scan_finds(reccheck, tmp_path, n):
    accel, gyro = stroke(address_duration_s=3.0)
    commands = ["flash 262144"]
    if n:
        commands += [start(), feed(range(n), accel[:n], gyro[:n]), "stop"]
    states, image = run(reccheck, tmp_path, *commands, "reopen")
    last = len(image.rstrip(b"\xff"))
    assert states[-1][1]["end"] == -(-last // PAGE) * PAGE


def test_the_dump_reads_the_partition_the_firmware_declares():
    """The dump tool takes the layout from firmware/partitions.csv rather than
    a second copy, and the table must leave the corpus the rest of 16 MB."""
    from tools.rec_dump import partition

    offset, size = partition()
    assert (offset, size) == (0x210000, 0xDF0000)
    assert offset + size == 16 * 1024 * 1024
    assert offset % 4096 == 0 and size % 4096 == 0


def test_the_summary_reports_loss_and_tearing(reccheck, tmp_path):
    from tools.rec_dump import summary

    accel, gyro = stroke()
    indices = np.arange(600)
    indices[300:] += 12
    _, image = run(reccheck, tmp_path, "flash 131072",
                   start(), feed(indices, accel[:600], gyro[:600]), "stop")
    text = summary(image)
    print("\n" + text)
    assert "session 1: 600 samples" in text and "12 lost" in text
    assert "TORN" not in text


def test_erase_empties_the_log_and_restarts_numbering(reccheck, tmp_path):
    accel, gyro = stroke()
    _, image = run(reccheck, tmp_path, "flash 131072",
                   start(), feed(range(100), accel[:100], gyro[:100]), "stop",
                   "erase",
                   start(), feed(range(10), accel[:10], gyro[:10]), "stop")
    [s] = corpus.decode(image)
    assert (s.number, s.samples, s.offset) == (1, 10, 0)
