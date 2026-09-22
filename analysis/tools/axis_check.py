"""Axis and sign check for the IMU (spec 9.1).

Answers the question every sign convention downstream depends on: which gyro
channel is which board axis, and is the triad right-handed? Nothing in the
fusion code can be ported until that is known, and no amount of simulation
answers it -- it needs a hand on the board.

The procedure needs no printer, no putter and no protractor, because gravity
supplies the reference:

  1. Rest the board on a face. The accelerometer says which axis points up, and
     it says so absolutely -- a sensor at rest measures proper acceleration, so
     the axis pointing up reads +1 g.
  2. Turn the board anticlockwise, seen from above, about that same vertical.
     By the right-hand rule the matching gyro channel must read POSITIVE about
     the up direction, whichever way up the board happens to be.
  3. Repeat on two more faces, so all three axes are covered.

Step 2 is what catches a left-handed triad, and a left-handed triad is the kind
of defect that produces a plausible-looking number with the wrong sign -- a
stroke that opened reported as closed.

The integrated angle is printed alongside, as a free end-to-end check of
counts to dps to degrees on real hardware. Treat it as a sanity check and not
as a calibration: a quarter turn by hand is good to a few degrees at best. The
real version of that measurement is tier 0 of spec 10.1, the printed protractor
plate.
"""

from dataclasses import dataclass

import numpy as np

GRAVITY = 9.81
AXIS_NAMES = ("X", "Y", "Z")

# A face, not a corner. cos(18 deg) = 0.95, so this accepts a board resting on
# something slightly uneven and rejects one propped against a mug.
SQUARE_FRACTION = 0.95
# At rest the total must be one g. Anything else means the board is moving, or
# being held, and nothing about the reading means what it appears to mean.
GRAVITY_TOLERANCE = 0.15
# A quarter turn was asked for. Anything smaller than this was a knock, a
# nudge, or a missed prompt, and must not reach a verdict. See
# is_deliberate_turn().
QUARTER_TURN_MINIMUM_DEG = 20.0


@dataclass(frozen=True)
class Axis:
    index: int          # 0, 1, 2
    sign: int           # +1 or -1

    @property
    def name(self) -> str:
        return f"{'+' if self.sign > 0 else '-'}{AXIS_NAMES[self.index]}"


@dataclass
class Rotation:
    angles_deg: np.ndarray   # integrated angle per channel, bias removed
    dominant: Axis
    cross_ratio: float       # largest off-axis angle, as a fraction of the dominant
    peak_dps: float
    start: int
    stop: int
    duration_s: float


@dataclass
class Verdict:
    ok: bool
    detail: str


def vertical_axis(accel_mps2) -> Axis | None:
    """Which axis points up, from a resting accelerometer reading.

    None when the board is not resting squarely on a face, or is not at rest.
    Both cases have to be rejected rather than approximated: the sign check
    that follows is against the direction this returns, so a direction that is
    30 degrees off is worse than no answer.
    """
    a = np.asarray(accel_mps2, dtype=float)
    magnitude = float(np.linalg.norm(a))
    if abs(magnitude - GRAVITY) > GRAVITY_TOLERANCE * GRAVITY:
        return None
    index = int(np.argmax(np.abs(a)))
    if abs(a[index]) / magnitude < SQUARE_FRACTION:
        return None
    return Axis(index, 1 if a[index] > 0 else -1)


def tilt_degrees(accel_mps2) -> float:
    """How far off square the board is resting, for reporting."""
    a = np.asarray(accel_mps2, dtype=float)
    index = int(np.argmax(np.abs(a)))
    return float(np.degrees(np.arccos(
        min(1.0, abs(a[index]) / max(1e-9, np.linalg.norm(a))))))


def integrate_rotation(gyro_dps, rate_hz: float, bias_dps,
                       noise_dps: float, trigger_sigmas: float = 20.0,
                       release_sigmas: float = 3.0) -> Rotation | None:
    """Find the turn in a record and integrate it, per channel.

    `bias_dps` is subtracted first. That is not housekeeping: the measured
    resting bias on this board is up to 2.9 dps, which over a two-second window
    is six degrees of pure error on a channel that never moved. Re-nulling the
    bias immediately before integrating is invariant 4 in miniature.

    The motion threshold is derived from `noise_dps`, measured in the same run
    moments earlier, rather than chosen. Two levels: motion has to exceed
    `trigger_sigmas` to count as a turn at all, and the window then extends
    outward until the rate falls below `release_sigmas`. One level would clip
    the start and end of the turn, where the rate passes through zero, and
    report a quarter turn as about 80 degrees.

    Returns None when nothing moved, rather than a small angle -- a missed
    prompt must not read as a measurement.
    """
    gyro = np.asarray(gyro_dps, dtype=float) - np.asarray(bias_dps, dtype=float)
    if len(gyro) < 2:
        return None

    rate = np.linalg.norm(gyro, axis=1)
    moving = rate > trigger_sigmas * noise_dps
    if not moving.any():
        return None

    start = int(np.argmax(moving))
    stop = int(len(moving) - np.argmax(moving[::-1]) - 1)

    release = release_sigmas * noise_dps
    while start > 0 and rate[start - 1] > release:
        start -= 1
    while stop < len(rate) - 1 and rate[stop + 1] > release:
        stop += 1

    window = gyro[start:stop + 1]
    angles = np.trapezoid(window, dx=1.0 / rate_hz, axis=0)

    index = int(np.argmax(np.abs(angles)))
    dominant_angle = abs(angles[index])
    others = [abs(angles[i]) for i in range(3) if i != index]
    return Rotation(
        angles_deg=angles,
        dominant=Axis(index, 1 if angles[index] > 0 else -1),
        cross_ratio=float(max(others) / dominant_angle) if dominant_angle else 0.0,
        peak_dps=float(rate[start:stop + 1].max()),
        start=start,
        stop=stop,
        duration_s=(stop - start + 1) / rate_hz,
    )


def is_deliberate_turn(rotation: Rotation,
                       minimum_deg: float = QUARTER_TURN_MINIMUM_DEG) -> bool:
    """Whether this was the turn the procedure asked for.

    Observed on hardware: something knocked the desk while the board sat still,
    the 20-sigma trigger fired on the transient, the integral came to +0.0
    degrees on the wrong channel, and it was recorded as a FAILED axis check.
    A verdict delivered on a turn nobody made is worse than no verdict.

    This is a bound on a human action, not a detection threshold inside the
    algorithm -- a quarter turn was requested, and five degrees was not it. It
    has nothing to say about how the product segments a stroke.
    """
    return abs(rotation.angles_deg[rotation.dominant.index]) >= minimum_deg


def rotation_verdict(expected_up: Axis, observed: Axis) -> Verdict:
    """Check a turn against the right-hand rule.

    Anticlockwise seen from above is a positive rotation about the direction
    pointing up. So the gyro channel for that axis must respond, and with the
    sign of the axis direction that points up.
    """
    if observed.index != expected_up.index:
        return Verdict(False, (
            f"wrong channel: turning about board {expected_up.name} moved "
            f"gyro {AXIS_NAMES[observed.index]}, not "
            f"{AXIS_NAMES[expected_up.index]}. The channels are not in the "
            f"order the body frame assumes."))
    if observed.sign != expected_up.sign:
        return Verdict(False, (
            f"wrong sign: an anticlockwise turn about {expected_up.name} read "
            f"{'positive' if observed.sign > 0 else 'negative'} on gyro "
            f"{AXIS_NAMES[observed.index]}. The triad is left-handed with "
            f"respect to the accelerometer, and every integrated angle on this "
            f"axis will come out inverted."))
    return Verdict(True, f"anticlockwise about {expected_up.name} reads "
                         f"positive on gyro {AXIS_NAMES[observed.index]}")


# --------------------------------------------------------------------------
# Everything below is the interactive procedure. It needs the board and a hand.

def _collect(port, seconds: float, fs):
    """Read for `seconds` and return (accel m/s^2, gyro dps) arrays.

    The buffer is flushed first: the board streams throughout the prompts, and
    what accumulated while somebody was reading the screen is not part of the
    measurement.
    """
    import time

    from tools.capture import decode_batches

    port.reset_input_buffer()
    leftover = b""
    rows = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        leftover += port.read(8192)
        batches, leftover = decode_batches(leftover)
        for batch in batches:
            rows.extend(batch.samples)
    if not rows:
        return None, None
    data = np.array(rows, dtype=np.int16).astype(float)
    return (data[:, 0:3] * fs.accel_mps2_per_count,
            data[:, 3:6] * (fs.gyro_dps / 32768.0))


def main() -> None:
    import argparse
    import sys

    import serial

    from plumb.sensor import FullScale
    from tools import board

    ap = argparse.ArgumentParser(description="Axis and sign check (spec 9.1).")
    ap.add_argument("--port", required=True, help="e.g. COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--odr", type=float, default=906.86,
                    help="measured sample rate, from tools/capture.py")
    ap.add_argument("--still-seconds", type=float, default=1.5)
    ap.add_argument("--turn-seconds", type=float, default=6.0)
    args = ap.parse_args()

    fs = FullScale()
    results: dict[int, tuple[Axis, Rotation, Verdict]] = {}

    print(__doc__.strip())
    print("\n" + "=" * 72)
    print("Three faces, so all three axes get covered. Ctrl-C to stop.")
    print("=" * 72)

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        board.connect(port, rate="stroke", read_path="direct", binary=True)
        board.start(port)

        try:
            while len(results) < 3:
                remaining = [AXIS_NAMES[i] for i in range(3) if i not in results]
                print(f"\n--- still needed: {', '.join(remaining)} ---")
                input("Rest the board flat on a face you have not used yet, "
                      "let go, and press Enter: ")

                accel, gyro = _collect(port, args.still_seconds, fs)
                if accel is None:
                    print("  no data -- is the board still streaming?")
                    continue

                up = vertical_axis(accel.mean(axis=0))
                if up is None:
                    print(f"  not resting square: {tilt_degrees(accel.mean(axis=0)):.0f}"
                          f" degrees off, total "
                          f"{np.linalg.norm(accel.mean(axis=0)):.2f} m/s^2 "
                          f"(9.81 expected). Put it down flat and try again.")
                    continue

                bias = gyro.mean(axis=0)
                noise = float(gyro.std(axis=0).max())
                print(f"  up axis: board {up.name}   "
                      f"(resting {tilt_degrees(accel.mean(axis=0)):.1f} degrees "
                      f"off square)")
                print(f"  gyro bias {bias[0]:+.2f} {bias[1]:+.2f} {bias[2]:+.2f} dps,"
                      f" noise {noise:.3f} dps -> motion threshold "
                      f"{20 * noise:.1f} dps")

                if up.index in results:
                    print(f"  {AXIS_NAMES[up.index]} is already done. "
                          f"Try another face.")
                    continue

                input(f"Now turn the board a quarter turn ANTICLOCKWISE seen "
                      f"from above,\nkeeping it flat. Press Enter, then make "
                      f"the turn within {args.turn_seconds:.0f} s: ")
                _, gyro = _collect(port, args.turn_seconds, fs)
                if gyro is None:
                    print("  no data -- is the board still streaming?")
                    continue

                rotation = integrate_rotation(gyro, args.odr, bias, noise)
                if rotation is None:
                    print("  nothing moved. Not recorded.")
                    continue
                if not is_deliberate_turn(rotation):
                    print(f"  something moved, but it integrated to only "
                          f"{rotation.angles_deg[rotation.dominant.index]:+.1f} "
                          f"degrees on gyro {rotation.dominant.name} over "
                          f"{rotation.duration_s:.2f} s. That was a knock, not "
                          f"a quarter turn. Not recorded.")
                    continue

                a = rotation.angles_deg
                print(f"  integrated: X {a[0]:+7.1f}  Y {a[1]:+7.1f}  "
                      f"Z {a[2]:+7.1f} degrees")
                print(f"  dominant: gyro {rotation.dominant.name}, peak "
                      f"{rotation.peak_dps:.0f} dps over "
                      f"{rotation.duration_s:.2f} s, cross-axis "
                      f"{100 * rotation.cross_ratio:.1f}%")

                verdict = rotation_verdict(up, rotation.dominant)
                print(f"  {'PASS' if verdict.ok else 'FAIL'}: {verdict.detail}")
                results[up.index] = (up, rotation, verdict)
        except (KeyboardInterrupt, EOFError):
            print("\ninterrupted")
        finally:
            board.stop(port)

    print("\n" + "=" * 72)
    print("Summary")
    print("=" * 72)
    if not results:
        print("nothing measured")
        return
    print(f"  {'board axis up':<14} {'gyro responded':<16} {'angle':>8}  verdict")
    for index in sorted(results):
        up, rotation, verdict = results[index]
        print(f"  {up.name:<14} {rotation.dominant.name:<16} "
              f"{rotation.angles_deg[index]:+8.1f}  "
              f"{'PASS' if verdict.ok else 'FAIL'}")
    for index in sorted(results):
        _, _, verdict = results[index]
        if not verdict.ok:
            print(f"\n  {AXIS_NAMES[index]}: {verdict.detail}")

    every_axis = len(results) == 3
    all_passed = all(v.ok for _, _, v in results.values())
    if every_axis and all_passed:
        print("\nAll three axes match the body frame's handedness. Spec 9.1 is "
              "satisfied for the sensor triad.")
        print("The remaining half of 9.1 -- how that triad sits relative to the "
              "PUTTER (Z along the shaft, X the face normal) -- is a property "
              "of the mount and cannot be checked until a base is printed.")
    elif not every_axis:
        print(f"\nOnly {len(results)} of 3 axes measured. Not a result yet.")
    else:
        print("\nAt least one axis does not match. Do not port the fusion code "
              "until this is understood; the numbers will look reasonable and "
              "be wrong.")
    print("\nThe integrated angles are an end-to-end sanity check on counts -> "
          "dps -> degrees,\nnot a calibration: a quarter turn by hand is good "
          "to a few degrees. Tier 0 of\nspec 10.1, the printed protractor "
          "plate, is the real version of that measurement.")


if __name__ == "__main__":
    main()
