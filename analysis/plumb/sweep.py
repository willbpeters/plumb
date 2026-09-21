"""Parameter sweep over the grid in the harness spec section 7.1.

Answers two questions the definition of done asks for: how accurately the
pipeline recovers a known face angle, and at what noise level it stops meeting
the parent spec section 3 target of +/-1.0 degrees.

Detection failure is recorded, not discarded. Above roughly 0.5 dps of gyro
noise the stillness threshold is under the noise floor, ADDRESS never triggers
and no stroke is reported at all. That is a real failure mode with its own
target in section 3, and averaging it away as a missing value would hide it.
"""

import itertools
from dataclasses import dataclass

from plumb.pipeline import Pipeline, Thresholds
from plumb.sensor import SensorParams, simulate
from plumb.trajectory import ArcType, StrokeParams, generate

FACE_ANGLES_DEG = (-5.0, -2.0, 0.0, 2.0, 5.0)
GYRO_NOISE_DPS = (0.0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0)
GYRO_BIAS_DPS = (0.0, 0.5, 2.0)
TEMPO_RATIOS = (1.5, 2.0, 3.0)
ARC_TYPES = tuple(ArcType)

FACE_TARGET_DEG = 1.0      # parent spec section 3
TEMPO_TARGET = 0.05


@dataclass(frozen=True)
class SweepRow:
    face_angle_deg: float
    arc_type: str
    tempo_ratio: float
    gyro_noise_dps: float
    gyro_bias_dps: float
    seed: int
    detected: bool
    recovered_deg: float
    face_error_deg: float
    tempo_error: float


def _one(face, arc, tempo, noise, bias, seed) -> SweepRow:
    stroke = StrokeParams(face_angle_at_impact_deg=face, arc_type=arc, tempo_ratio=tempo)
    traj = generate(stroke)
    out = simulate(traj, SensorParams(gyro_noise_dps=noise, gyro_bias_dps=bias), seed=seed)
    pipe = Pipeline(Thresholds(), out.full_scale, stroke.lever_arm_m)

    result = None
    for i in range(len(traj.time)):
        r = pipe.step(out.gyro_counts[i], out.accel_counts[i])
        if r is not None:
            result = r

    if result is None:
        return SweepRow(face, arc.name, tempo, noise, bias, seed,
                        False, 0.0, 0.0, 0.0)
    return SweepRow(face, arc.name, tempo, noise, bias, seed, True,
                    result.face_angle_deg,
                    result.face_angle_deg - face,
                    result.tempo_ratio - traj.true_tempo_ratio)


def run_sweep(face_angles=FACE_ANGLES_DEG, arcs=ARC_TYPES, tempos=TEMPO_RATIOS,
              noises=GYRO_NOISE_DPS, biases=GYRO_BIAS_DPS, seeds=(1, 2, 3)) -> list[SweepRow]:
    grid = itertools.product(face_angles, arcs, tempos, noises, biases, seeds)
    return [_one(*combo) for combo in grid]


def detection_rate(rows) -> float:
    return sum(r.detected for r in rows) / len(rows) if rows else 0.0


def worst_face_error(rows) -> float:
    """Worst error over DETECTED strokes. An undetected stroke has no error to
    report -- it is counted by detection_rate instead."""
    detected = [abs(r.face_error_deg) for r in rows if r.detected]
    return max(detected) if detected else 0.0


def noise_breakdown_dps(rows) -> float:
    """Highest swept noise level at which every stroke was detected AND met the
    face-angle target. Reported rather than asserted: the definition of done
    asks us to STATE this number, not to hit a particular value."""
    levels = sorted({r.gyro_noise_dps for r in rows})
    passing = [
        n for n in levels
        if all(r.detected and abs(r.face_error_deg) <= FACE_TARGET_DEG
               for r in rows if r.gyro_noise_dps == n)
    ]
    return max(passing) if passing else 0.0


def main() -> None:
    rows = run_sweep()
    print(f"strokes simulated      : {len(rows)}")
    print(f"detection rate         : {100 * detection_rate(rows):.1f}%  "
          f"(parent spec section 3 target: 98%)")
    print(f"worst face-angle error : {worst_face_error(rows):.4f} deg  "
          f"(target {FACE_TARGET_DEG} deg)")
    print(f"meets face target up to: {noise_breakdown_dps(rows):.2f} dps gyro noise")
    print()
    print(f"{'noise dps':>10} {'detected':>10} {'worst face err':>16} {'worst tempo err':>17}")
    for n in sorted({r.gyro_noise_dps for r in rows}):
        at = [r for r in rows if r.gyro_noise_dps == n]
        det = [r for r in at if r.detected]
        face = max((abs(r.face_error_deg) for r in det), default=float("nan"))
        tempo = max((abs(r.tempo_error) for r in det), default=float("nan"))
        print(f"{n:>10.2f} {100 * detection_rate(at):>9.0f}% {face:>16.4f} {tempo:>17.4f}")


if __name__ == "__main__":
    main()
