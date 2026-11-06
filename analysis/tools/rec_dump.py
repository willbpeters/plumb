"""Dump the device's corpus log over USB, and summarise a dumped image.

    uv run python -m tools.rec_dump --port COM4          dump to data/corpus/
    uv run python -m tools.rec_dump --summary IMAGE      what is in an image

The dump is `esptool read_flash` on the `corpus` partition, 1 MB at a time,
stopping after the first piece that is entirely erased: the log is written
front to back, so nothing lies beyond it. The firmware needs no dump code, and
the board resets into its bootloader to be read, so a session in progress is
lost from its last whole page (docs/superpowers/specs/
2026-09-29-stroke-logging-design.md). Stop it with `l` first.

esptool comes with ESP-IDF, or from PyPI (`uv add esptool`). Offsets come
from firmware/partitions.csv, so the layout is written down once.

There is no clock on the device. The wall time of the dump is recorded beside
the image, and each session's start_ms says how long after boot it began.
"""

import argparse
import datetime
import json
import subprocess
import sys
from pathlib import Path

from plumb import corpus

REPO = Path(__file__).resolve().parents[2]
PARTITIONS = REPO / "firmware" / "partitions.csv"
CHUNK = 1 << 20


def partition(name: str = "corpus", table: Path = PARTITIONS) -> tuple[int, int]:
    """(offset, size) of a partition in the firmware's table."""
    for line in table.read_text().splitlines():
        fields = [f.strip() for f in line.split("#")[0].split(",")]
        if len(fields) >= 5 and fields[0] == name:
            return int(fields[3], 0), int(fields[4], 0)
    raise KeyError(f"no partition {name!r} in {table}")


def _read_flash(port: str, baud: int, offset: int, size: int, path: Path) -> None:
    subprocess.run([sys.executable, "-m", "esptool", "--port", port,
                    "--baud", str(baud), "read_flash", hex(offset), hex(size),
                    str(path)], check=True)


def dump(port: str, out_dir: Path, baud: int = 921600) -> Path:
    offset, size = partition()
    stamp = datetime.datetime.now().astimezone()
    out_dir.mkdir(parents=True, exist_ok=True)
    image_path = out_dir / f"corpus-{stamp:%Y%m%d-%H%M%S}.bin"
    image = bytearray()
    piece = image_path.with_suffix(".part")
    for start in range(0, size, CHUNK):
        n = min(CHUNK, size - start)
        _read_flash(port, baud, offset + start, n, piece)
        data = piece.read_bytes()
        image += data
        if data.count(0xFF) == len(data):
            break
    piece.unlink(missing_ok=True)
    image_path.write_bytes(bytes(image))
    image_path.with_suffix(".json").write_text(json.dumps({
        "dumped_at": stamp.isoformat(), "port": port,
        "partition_offset": offset, "partition_size": size,
        "bytes_read": len(image)}, indent=2))
    return image_path


def summary(image: bytes) -> str:
    used = len(image.rstrip(b"\xff"))
    lines = [f"{used:,} bytes used"]
    for s in corpus.decode(image):
        seconds = s.samples / s.rate_hz if s.rate_hz else 0.0
        lines.append(
            f"session {s.number}: {s.samples:,} samples ({seconds:.1f} s at "
            f"{s.rate_hz:.4f} Hz), {s.lost} lost, {len(s.runs)} runs, "
            f"firmware {s.firmware}, putter {s.putter_id}, "
            f"{'calibrated' if s.calibrated else 'UNCALIBRATED accel'}"
            f"{', TORN (power lost)' if s.torn else ''}")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--out", type=Path, default=REPO / "data" / "corpus")
    parser.add_argument("--summary", type=Path, metavar="IMAGE")
    args = parser.parse_args()
    if args.summary:
        print(summary(args.summary.read_bytes()))
        return
    if not args.port:
        parser.error("--port is required to dump")
    path = dump(args.port, args.out, args.baud)
    print(f"wrote {path}")
    print(summary(path.read_bytes()))


if __name__ == "__main__":
    main()
