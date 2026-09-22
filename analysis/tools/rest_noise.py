"""Resting-noise analysis for a capture written by tools/capture.py.

Answers spec 9.4 -- the gyro noise floor on this board, per axis, in dps --
and prints the numbers rather than asserting anything about them. Four defects
in this project were invisible as pass/fail and obvious as a column of figures.

Scale factors come from plumb.sensor, so a count means the same physical
quantity here, in the firmware, and in the synthetic harness.
"""

import numpy as np

from plumb.sensor import FullScale

AXES = ("gx", "gy", "gz")


def robust_sigma(x: np.ndarray) -> np.ndarray:
    """Sigma estimated from the median absolute deviation.

    1.4826 * MAD equals the standard deviation for a normal distribution and
    ignores a tail. Carrying both estimators is what makes a heavy tail
    visible: thermal noise gives raw ~= robust, while a room walking past, or
    a read path returning scrambled samples, gives raw >> robust.
    """
    median = np.median(x, axis=0)
    return 1.4826 * np.median(np.abs(x - median), axis=0)


def window_sigma(x: np.ndarray, samples_per_window: int) -> np.ndarray:
    """Sigma over consecutive windows: (n_windows, n_channels).

    A single sigma over a whole capture cannot distinguish a noisy sensor from
    a quiet sensor that was disturbed twice. The minimum across windows is the
    closest this measurement gets to the sensor alone.
    """
    n = len(x) // samples_per_window
    if n == 0:
        return np.empty((0, x.shape[1]))
    trimmed = x[:n * samples_per_window].reshape(n, samples_per_window, -1)
    return trimmed.std(axis=1)


def position_sigma(batches: np.ndarray) -> np.ndarray:
    """Sigma per position within a batch: (batch_len, n_channels).

    `batches` is (n_batches, batch_len, n_channels). A stationary sensor cannot
    have noise that depends on where in a transfer a sample sat, so any
    structure here belongs to the transfer, not the sensor.
    """
    return batches.std(axis=0)


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("capture", help="an .npy written by tools/capture.py")
    ap.add_argument("--odr", type=float, required=True,
                    help="measured sample rate in Hz -- the sensor's, not the "
                         "nominal one, and not the delivered one")
    ap.add_argument("--batch", type=int, default=0,
                    help="batch length, to report sigma by position within a "
                         "batch. 0 to skip.")
    args = ap.parse_args()

    fs = FullScale()
    rows = np.load(args.capture)
    accel = rows[:, 0:3].astype(np.float64) * fs.accel_mps2_per_count
    gyro = rows[:, 3:6].astype(np.float64) * (fs.gyro_dps / 32768.0)

    print(f"{args.capture}: {len(rows)} samples at {args.odr:.2f} Hz "
          f"({len(rows) / args.odr:.1f} s)")
    print(f"accel magnitude at rest: {np.linalg.norm(accel.mean(axis=0)):.3f} "
          f"m/s^2 (9.81 expected)")

    raw = gyro.std(axis=0)
    rob = robust_sigma(gyro)
    peak = np.abs(gyro - gyro.mean(axis=0)).max(axis=0)
    print("gyro, dps:")
    print(f"  {'axis':4} {'mean':>9} {'sigma':>8} {'robust':>8} "
          f"{'raw/robust':>11} {'peak':>8}")
    for i, axis in enumerate(AXES):
        print(f"  {axis:4} {gyro[:, i].mean():+9.4f} {raw[i]:8.4f} "
              f"{rob[i]:8.4f} {raw[i] / rob[i]:11.2f} {peak[i]:8.3f}")
    print(f"  worst axis sigma: {raw.max():.4f} dps "
          f"(robust {rob.max():.4f})")

    windows = window_sigma(gyro, int(0.5 * args.odr))
    if len(windows):
        worst = windows.max(axis=1)
        print(f"0.5 s windows ({len(windows)}): worst-axis sigma "
              f"min {worst.min():.4f}  median {np.median(worst):.4f}  "
              f"max {worst.max():.4f}")

    if args.batch:
        n = len(gyro) // args.batch
        if n < 2:
            print(f"not enough samples for {args.batch}-sample batches")
            return
        block = gyro[:n * args.batch].reshape(n, args.batch, 3)
        sigma = position_sigma(block)
        print(f"sigma by position within a {args.batch}-sample batch "
              f"({n} batches):")
        step = max(1, args.batch // 8)
        for lo in range(0, args.batch, step):
            hi = min(lo + step, args.batch)
            seg = sigma[lo:hi]
            print(f"  samples {lo:3d}-{hi - 1:3d}  "
                  + "  ".join(f"{a} {seg[:, i].mean():6.4f}"
                              for i, a in enumerate(AXES)))


if __name__ == "__main__":
    main()
