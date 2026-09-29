"""Check the firmware's startup sample-rate measurement on the board.

Two questions, answered separately because they have different answers:

1. REPEATABILITY. The firmware measures the IMU's rate at boot and on `m`
   (firmware/components/acq/include/acq/rate.h). Asking for it N times in a
   row says how much it moves within a session -- which is what the fit's own
   standard error claims to predict. If the spread is much larger than the
   reported error, something the fit does not model is moving (the IMU's
   oscillator with temperature, most likely), and that is what bounds the
   measurement, not the fit.

2. THE CLOCK IT IS MEASURED AGAINST. The firmware times DRDY edges with
   esp_timer, which counts the ESP32-S3's own crystal. A crystal error is a
   rate error the firmware cannot see from inside. So the same samples are also
   timed against this PC's clock -- a different crystal -- by streaming them
   and fitting the sensor's sample index against host arrival time. Agreement
   bounds the two crystals' DIFFERENCE; it cannot say which is right.

    uv run python -m tools.rate_check --port COM4
"""

import argparse
import re
import sys
import time

import numpy as np

RATE_LINE = re.compile(
    r"# sample rate (?P<hz>[\d.]+) Hz \+/- (?P<se>[\d.eE+-]+) .*?"
    r"(?P<used>\d+) points over (?P<span>[\d.]+) s, (?P<rejected>\d+) rejected, "
    r"(?P<skipped>\d+) skipped, rms (?P<rms>[\d.]+) us, max (?P<max>[\d.]+) us")


def parse_rate_line(text: str) -> dict | None:
    """The firmware's `# sample rate ...` line, as numbers. None if absent."""
    match = RATE_LINE.search(text)
    if not match:
        return None
    return {k: float(v) for k, v in match.groupdict().items()}


def host_clock_rate(arrivals: np.ndarray, indices: np.ndarray):
    """Least-squares rate of sample index against host arrival time.

    Arrival times carry USB and buffering delay of a millisecond or more, but
    that delay is stationary, so it costs scatter and not slope. Returns
    (hz, standard error in hz).
    """
    t = np.asarray(arrivals, dtype=float)
    k = np.asarray(indices, dtype=float)
    t = t - t[0]
    k = k - k[0]
    # Fit time against index: the independent variable is the one without
    # delay in it. Slope is seconds per sample.
    design = np.column_stack([k, np.ones_like(k)])
    (slope, intercept), residual, _, _ = np.linalg.lstsq(design, t, rcond=None)
    dof = max(1, len(k) - 2)
    sigma = np.sqrt(float(residual[0]) / dof) if len(residual) else 0.0
    slope_se = sigma / np.sqrt(np.sum((k - k.mean()) ** 2))
    hz = 1.0 / slope
    return hz, hz * slope_se / slope


def _read_until(port, pattern: str, seconds: float) -> str:
    text = ""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        text += port.read(4096).decode("utf-8", "replace")
        if pattern in text:
            # Let the rest of the line arrive.
            time.sleep(0.05)
            text += port.read(4096).decode("utf-8", "replace")
            return text
    return text


def measure_repeats(port, repeats: int) -> list[dict]:
    results = []
    for i in range(repeats):
        port.reset_input_buffer()
        port.write(b"m")
        text = _read_until(port, "# sample rate ", 4.0)
        # The first line after `m` may be the acknowledgement; wait for a
        # completed measurement specifically.
        parsed = parse_rate_line(text)
        if parsed is None:
            text += _read_until(port, "us\n", 3.0)
            parsed = parse_rate_line(text)
        if parsed is None:
            print(f"# measurement {i + 1}: no result in the output", file=sys.stderr)
            continue
        results.append(parsed)
        print(f"  {i + 1:3d}  {parsed['hz']:.4f} Hz  +/- {parsed['se']:.1e}  "
              f"rejected {parsed['rejected']:.0f}  rms {parsed['rms']:.2f} us")
    return results


def stream_against_host(port, seconds: float):
    """Stream CSV and time each chunk's newest sample on this PC's clock."""
    from tools import board

    board.settle_stopped(port)
    port.write(b"c")
    time.sleep(board.COMMAND_SECONDS)
    port.reset_input_buffer()
    board.start(port)
    arrivals, indices = [], []
    pending = ""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        data = port.read(8192)
        now = time.perf_counter()
        if not data:
            continue
        pending += data.decode("ascii", "replace")
        lines = pending.split("\n")
        pending = lines.pop()
        newest = None
        for line in lines:
            if line.startswith("#"):
                continue
            head = line.split(",", 1)[0]
            if head.isdigit():
                newest = int(head)
        if newest is not None:
            arrivals.append(now)
            indices.append(newest)
    board.stop(port)
    board.settle_stopped(port)
    return np.array(arrivals), np.array(indices)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True)
    ap.add_argument("--repeats", type=int, default=30)
    ap.add_argument("--stream-seconds", type=float, default=60.0)
    ap.add_argument("--baud", type=int, default=921600)
    args = ap.parse_args()

    import serial

    from tools import board

    port = serial.Serial()
    port.port = args.port
    port.baudrate = args.baud
    port.timeout = 0.1
    # Held low before opening, so opening does not pulse the auto-download
    # circuit (tools/board.py). If the board resets anyway, its boot
    # measurement just runs first.
    port.dtr = False
    port.rts = False
    port.open()
    try:
        time.sleep(board.BOOT_SECONDS)
        board.settle_stopped(port)

        print(f"# {args.repeats} on-board measurements, back to back")
        results = measure_repeats(port, args.repeats)
        hz = np.array([r["hz"] for r in results])
        se = np.array([r["se"] for r in results])
        if len(hz) >= 2:
            print(f"# on board: mean {hz.mean():.4f} Hz, SD {hz.std(ddof=1):.1e} Hz "
                  f"({1e6 * hz.std(ddof=1) / hz.mean():.2f} ppm), range "
                  f"{hz.min():.4f} to {hz.max():.4f}; reported SE mean {se.mean():.1e} Hz")
            print(f"# spread / reported SE = {hz.std(ddof=1) / se.mean():.1f} "
                  f"(1 if the fit's error is the whole story)")

        print(f"# streaming {args.stream_seconds:.0f} s against this PC's clock")
        arrivals, indices = stream_against_host(port, args.stream_seconds)
        if len(indices) < 10:
            print("# too few samples arrived to fit", file=sys.stderr)
            return
        host_hz, host_se = host_clock_rate(arrivals, indices)
        print(f"# host clock: {host_hz:.4f} Hz +/- {host_se:.1e} over "
              f"{indices[-1] - indices[0]} samples, {len(indices)} chunks")
        if len(hz):
            ppm = 1e6 * (host_hz - hz.mean()) / hz.mean()
            print(f"# host minus board: {host_hz - hz.mean():+.4f} Hz = {ppm:+.1f} ppm "
                  f"-- the two crystals' difference plus any drift between the "
                  f"measurements")
    finally:
        port.close()


if __name__ == "__main__":
    main()
