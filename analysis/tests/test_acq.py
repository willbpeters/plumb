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


# -- the startup rate measurement ------------------------------------------
#
# Will's decision, 2026-09-27: the firmware measures its own sample rate at
# startup, because this board's IMU runs 1.12% above nominal and that error
# scales every integrated angle. acq/rate.c fits the DRDY edge times against
# the sensor's own sample index. Ground truth here is a generated edge
# sequence whose period is known because it was put there.

RATE_CAPACITY = 1024
TRUE_HZ = 906.86


def edges(rng, n, hz=TRUE_HZ, jitter_us=0.0, start_us=123_456, lost=0.0,
          behind=0.0, slips=0.0):
    """(index, edge_us, edges) as the acquisition task would record them.

    `lost`: fraction of samples never read -- the counter skips them.
    `behind`: fraction of records where the task fell behind (edges > 1).
    `slips`: fraction where the edge time belongs to a later sample than
    the one read -- the pairing error a late read can cause -- modelled as
    one whole period of extra delay.
    """
    period = 1e6 / hz
    rows = []
    index = 0
    while len(rows) < n:
        t = start_us + index * period + rng.normal(0.0, jitter_us)
        count = 1
        if rng.random() < behind:
            count = 2
        if rng.random() < slips:
            t += period
        if rng.random() >= lost:
            rows.append((index, int(round(t)) & 0xFFFFFFFF, count))
        index += 1
    return rows


def rate(exe, rows, capacity=RATE_CAPACITY):
    numbers = [capacity, len(rows)]
    for index, edge, count in rows:
        numbers += [index, edge, count]
    out = subprocess.run([str(exe), "rate"],
                         input=" ".join(str(int(v)) for v in numbers),
                         capture_output=True, text=True, check=True)
    fields = out.stdout.split()
    if fields[0] != "ok":
        return None
    names = ["hz", "hz_se", "rms_us", "max_us", "used", "rejected", "skipped",
             "span_s", "accepted"]
    return dict(zip(names, (float(v) for v in fields[1:])))


def test_rate_is_recovered_from_clean_edges(acqcheck):
    """Integer-microsecond edges and nothing else: the esp_timer's own
    resolution is the only error."""
    est = rate(acqcheck, edges(np.random.default_rng(1), 1000))
    print(f"\n  clean: {est['hz']:.6f} Hz against {TRUE_HZ}, "
          f"se {est['hz_se']:.2e}, rms {est['rms_us']:.3f} us")
    assert est["hz"] == pytest.approx(TRUE_HZ, rel=1e-6)
    assert est["rejected"] == 0 and est["skipped"] == 0
    assert est["rms_us"] < 0.5


def test_rate_standard_error_is_honest(acqcheck):
    """The reported uncertainty has to describe the real scatter, or the spec
    amendment would be stating a number nobody measured. 60 independent
    measurements with 3 us of edge jitter: the errors divided by their own
    reported standard error should scatter with SD about 1."""
    z = []
    for seed in range(60):
        est = rate(acqcheck, edges(np.random.default_rng(seed), 1000,
                                   jitter_us=3.0))
        z.append((est["hz"] - TRUE_HZ) / est["hz_se"])
    z = np.array(z)
    print(f"\n  60 runs at 3 us jitter: z mean {z.mean():+.2f}, sd {z.std(ddof=1):.2f}")
    assert abs(z.mean()) < 0.5
    assert 0.75 < z.std(ddof=1) < 1.3


def test_lost_samples_do_not_bias_the_rate(acqcheck):
    """Counting records instead of the counter would read 10% loss as a rate
    10% low. The fit is on the counter's index, so loss only thins it."""
    est = rate(acqcheck, edges(np.random.default_rng(2), 1000, jitter_us=3.0,
                               lost=0.10))
    assert est["hz"] == pytest.approx(TRUE_HZ, abs=4 * est["hz_se"])


def test_a_task_that_fell_behind_is_skipped_not_fitted(acqcheck):
    """edges > 1 means more than one DRDY edge since the previous read, so
    the stored edge time may not be this sample's. Skipped and counted."""
    rows = edges(np.random.default_rng(3), 1000, jitter_us=3.0, behind=0.05)
    est = rate(acqcheck, rows)
    assert est["skipped"] == sum(1 for _, _, c in rows if c > 1)
    assert est["hz"] == pytest.approx(TRUE_HZ, abs=4 * est["hz_se"])


def test_mispaired_edges_are_rejected(acqcheck):
    """An edge time a whole period late is ~1100 us off the line against a
    few us of jitter. Left in, 2% of them would inflate the uncertainty
    tenfold; the robust rejection takes them out and says how many."""
    rng = np.random.default_rng(4)
    rows = edges(rng, 1000, jitter_us=3.0, slips=0.02)
    est = rate(acqcheck, rows)
    clean = rate(acqcheck, edges(np.random.default_rng(4), 1000, jitter_us=3.0))
    print(f"\n  2% mispaired: rejected {est['rejected']:.0f}, "
          f"max residual kept {est['max_us']:.1f} us, se {est['hz_se']:.2e} "
          f"against {clean['hz_se']:.2e} clean")
    assert 10 <= est["rejected"] <= 35
    assert est["max_us"] < 30
    assert est["hz"] == pytest.approx(TRUE_HZ, abs=4 * est["hz_se"])
    assert est["hz_se"] < 2 * clean["hz_se"]


def test_the_esp_timer_wrapping_mid_window_changes_nothing(acqcheck):
    """edge_us is the low 32 bits of esp_timer: it wraps every 71.6 minutes,
    so a measurement can straddle it."""
    start = 0xFFFFFFFF - 400_000
    est = rate(acqcheck, edges(np.random.default_rng(5), 1000, start_us=start))
    assert est["hz"] == pytest.approx(TRUE_HZ, rel=1e-6)


def test_too_few_points_give_no_rate(acqcheck):
    assert rate(acqcheck, edges(np.random.default_rng(6), 2)) is None


def test_the_buffer_stops_at_capacity(acqcheck):
    """Points past capacity are not taken, and the harness says how many
    were: the fit covers what it says it covers."""
    est = rate(acqcheck, edges(np.random.default_rng(7), 300), capacity=100)
    assert est["accepted"] == 100 and est["used"] == 100
    assert est["span_s"] == pytest.approx(99 / TRUE_HZ, rel=1e-3)
