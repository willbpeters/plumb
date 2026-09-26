"""Drive the headless UI renderer (firmware/test/uisnap.c).

Renders plumb_ui screens on the PC, with no board: as NumPy RGB arrays for
tests, and as PNGs for looking at. UI spec section 6.

    uv run python -m tools.uisnap            # writes firmware/test/ui-gallery/
"""

import struct
import subprocess
import tempfile
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tools import cbuild

STRAIGHT, IN_TO_OUT, OUT_TO_IN = 0, 1, 2
SCREENS = ("face", "tempo", "path", "speed")
GALLERY = cbuild.HARNESS / "ui-gallery"

_executable: Path | None = None


@dataclass
class Result:
    """A stroke result as the UI receives it. None marks a metric invalid.
    Defaults are the screen studies' example stroke."""

    face: float | None = 1.8
    backswing_s: float | None = 0.70
    downswing_s: float = 0.34
    path_dir: int | None = OUT_TO_IN
    path_arc_m: float = 0.006
    path_travel_m: float = 0.30
    speed: float | None = 1.62

    def command(self) -> str:
        f, t = self.face is not None, self.backswing_s is not None
        p, s = self.path_dir is not None, self.speed is not None
        return ("result "
                f"{int(f)} {self.face if f else 0.0:.9g} "
                f"{int(t)} {self.backswing_s if t else 0.0:.9g} {self.downswing_s:.9g} "
                f"{int(p)} {self.path_dir if p else 0} "
                f"{self.path_arc_m:.9g} {self.path_travel_m:.9g} "
                f"{int(s)} {self.speed if s else 0.0:.9g}")


def executable() -> Path:
    """Build once per process. Raises cbuild.CompilerNotFound without one."""
    global _executable
    if _executable is None:
        _executable = cbuild.build_ui("uisnap.exe").executable
    return _executable


def run(lines: list[str]) -> list[str]:
    """Send commands, return the lines printed.

    UTF-8 explicitly: on Windows the default is cp1252, which would hand the
    renderer a degree sign as the single byte 0xB0 -- invalid UTF-8, and a
    glyph check that tests the encoding instead of the font."""
    result = subprocess.run([str(executable())], input="\n".join(lines) + "\n",
                            capture_output=True, text=True, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"uisnap exited {result.returncode}: {result.stderr}")
    return result.stdout.splitlines()


def to_rgb(raw: bytes) -> np.ndarray:
    """RGB565 little-endian, 240 x 240, to 8-bit RGB."""
    px = np.frombuffer(raw, dtype="<u2").reshape(240, 240).astype(np.uint32)
    r = ((px >> 11) & 31) * 255 // 31
    g = ((px >> 5) & 63) * 255 // 63
    b = (px & 31) * 255 // 31
    return np.dstack([r, g, b]).astype(np.uint8)


def render(result: Result | None = None, screen: int = 0, t_ms: int = 6000,
           idle: bool = False) -> np.ndarray:
    """One frame: show `result`, swipe to `screen`, run the clock `t_ms`."""
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "frame.rgb565"
        lines = []
        if result is not None and not idle:
            lines.append(result.command())
            lines += ["next"] * screen
        lines += [f"advance {t_ms}", f"snap {path}"]
        run(lines)
        return to_rgb(path.read_bytes())


def write_png(path: Path, rgb: np.ndarray) -> None:
    """Minimal PNG writer, so the bench needs nothing beyond NumPy."""
    height, width, _ = rgb.shape
    rows = b"".join(b"\x00" + rgb[y].tobytes() for y in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(rows))
                     + chunk(b"IEND", b""))


def gallery(out: Path = GALLERY) -> list[Path]:
    """Every screen at its final frame, plus the variants worth looking at."""
    out.mkdir(parents=True, exist_ok=True)
    cases = [("idle", None, 0)]
    cases += [(name, Result(), i) for i, name in enumerate(SCREENS)]
    cases += [("face-closed", Result(face=-2.4), 0),
              ("face-square", Result(face=0.02), 0),
              ("path-in-to-out", Result(path_dir=IN_TO_OUT), 2),
              ("path-straight", Result(path_dir=STRAIGHT), 2),
              ("no-reading", Result(face=None, backswing_s=None,
                                    path_dir=None, speed=None), 0)]
    written = []
    for name, result, screen in cases:
        path = out / f"{name}.png"
        write_png(path, render(result, screen, idle=result is None))
        written.append(path)
    return written


if __name__ == "__main__":
    for p in gallery():
        print(p)
