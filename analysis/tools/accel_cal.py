"""Accelerometer tumble calibration (parent spec 8.1), on the real board.

Why: an uncorrected accelerometer bias moves the path arc by 1.21% per
0.01 m/s^2 on the swing axis at lie 5 (HANDOFF.md, open defect 6), and the
stroke cannot observe it. This unit read 9.689 m/s^2 at rest against 9.81 in
bring-up, so its error is of the order that matters.

Procedure. Rest the board still on each of its six faces, then on at least two
more poses of any kind -- propped on an edge, leaning on something. The six
faces determine the six unknowns exactly; the extra poses are what let the fit
check itself, because six equations in six unknowns fit a bad pose perfectly.
The fixture does not need to be square: the model holds at any orientation.

Output: the offset and gain per axis, the fit residual per pose, and a JSON
record under data/calibration/ that keeps the raw pose means -- the
measurement, not just the answer (parent spec 11: store raw, not derived).
"""

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from plumb.calibration import (CalibrationError, PARAMETERS,
                               solve_accel_calibration)
from tools.axis_check import AXIS_NAMES, Axis, vertical_axis

REPO = Path(__file__).resolve().parents[2]
OUTPUT = REPO / "data" / "calibration"

# Poses beyond the six unknowns that the tool insists on. Two, so that one bad
# pose shows up as a misfit rather than being shared out between the rest.
EXTRA_POSES = 2


@dataclass
class Pose:
    mean_mps2: np.ndarray
    sd_mps2: np.ndarray
    gyro_sd_dps: float
    samples: int
    face: Axis | None       # which face was up, if it was resting on one


def summarise(accel_mps2, gyro_dps) -> Pose:
    accel = np.asarray(accel_mps2, dtype=float)
    return Pose(mean_mps2=accel.mean(axis=0), sd_mps2=accel.std(axis=0),
                gyro_sd_dps=float(np.asarray(gyro_dps).std(axis=0).max()),
                samples=len(accel), face=vertical_axis(accel.mean(axis=0)))


def faces_missing(poses: list[Pose]) -> list[str]:
    """Signed axes not yet seen pointing up. All six are needed: an axis seen
    only one way cannot separate its offset from its gain."""
    seen = {(p.face.index, p.face.sign) for p in poses if p.face is not None}
    return [f"{'+' if s > 0 else '-'}{AXIS_NAMES[i]}"
            for i in range(3) for s in (1, -1) if (i, s) not in seen]


def record(poses: list[Pose], cal, gravity: float) -> dict:
    return {
        "measured": datetime.now().isoformat(timespec="seconds"),
        "gravity_mps2": gravity,
        "model": "true = (raw - offset) * gain, per axis",
        "offset_mps2": cal.offset_mps2.tolist(),
        "gain": cal.gain.tolist(),
        "degrees_of_freedom": cal.degrees_of_freedom,
        "worst_residual_mps2": cal.worst_residual_mps2,
        "poses": [{"mean_mps2": p.mean_mps2.tolist(), "sd_mps2": p.sd_mps2.tolist(),
                   "gyro_sd_dps": p.gyro_sd_dps, "samples": p.samples,
                   "face": p.face.name if p.face else None} for p in poses],
    }


def main() -> None:
    import argparse

    import serial

    from plumb.sensor import FullScale
    from tools import board
    from tools.axis_check import _collect

    ap = argparse.ArgumentParser(description="Accelerometer tumble calibration (spec 8.1).")
    ap.add_argument("--port", required=True, help="e.g. COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=2.0,
                    help="averaging time per pose")
    ap.add_argument("--gravity", type=float, required=True,
                    help="local g in m/s^2 (Sacramento is about 9.800); sets the "
                         "gain's absolute scale")
    args = ap.parse_args()

    fs = FullScale()
    poses: list[Pose] = []
    print(__doc__.strip())

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        board.connect(port, rate="stroke", read_path="direct", binary=True)
        board.start(port)
        try:
            while True:
                missing = faces_missing(poses)
                extra = len(poses) - (6 - len(missing))
                if not missing and extra >= EXTRA_POSES:
                    break
                need = (f"faces still needed: {', '.join(missing)}" if missing
                        else f"now {EXTRA_POSES - extra} more pose(s) of any kind, "
                             f"not flat on a face")
                input(f"\n--- {need} ---\nPlace the board, let go, press Enter: ")
                accel, gyro = _collect(port, args.seconds, fs)
                if accel is None:
                    print("  no data -- is the board still streaming?")
                    continue
                pose = summarise(accel, gyro)
                print(f"  mean {np.round(pose.mean_mps2, 4)} m/s^2, |a| "
                      f"{np.linalg.norm(pose.mean_mps2):.4f}, sd "
                      f"{pose.sd_mps2.max():.4f}, gyro sd {pose.gyro_sd_dps:.3f} dps, "
                      f"face {pose.face.name if pose.face else 'none'}")
                if pose.face is not None and pose.face.name not in missing and missing:
                    print("  that face is already done; recorded anyway as an extra pose")
                poses.append(pose)
        except (KeyboardInterrupt, EOFError):
            print("\ninterrupted")
        finally:
            board.stop(port)

    if len(poses) < PARAMETERS:
        print("not enough poses; nothing solved")
        return
    try:
        cal = solve_accel_calibration([p.mean_mps2 for p in poses], args.gravity)
    except CalibrationError as err:
        print(f"\nREFUSED: {err}")
        return

    print(f"\noffset  {np.round(cal.offset_mps2, 4)} m/s^2")
    print(f"gain    {np.round(cal.gain, 5)}")
    print(f"worst pose misfit {cal.worst_residual_mps2:.4f} m/s^2 over "
          f"{cal.degrees_of_freedom} spare pose(s)")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / f"accel-{datetime.now():%Y%m%d-%H%M%S}.json"
    path.write_text(json.dumps(record(poses, cal, args.gravity), indent=2))
    print(f"saved {path}")


if __name__ == "__main__":
    main()
