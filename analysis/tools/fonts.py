"""Regenerate the UI's LVGL bitmap fonts (UI spec section 5).

Saira Condensed, SIL Open Font License 1.1, converted with lv_font_conv at the
four sizes the screens use and subset to exactly the glyphs they print. The
generated .c files are committed, so building needs neither Node nor the
network; this script is only for changing sizes or glyphs.

Named for their role, not "Saira": the font's licence reserves that name, and
a converted subset is arguably a Modified Version (OFL section 3). The source
is credited in fonts/README.md.

Run from analysis/:  uv run python -m tools.fonts
"""

import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
FONTS = REPO / "firmware" / "components" / "plumb_ui" / "fonts"
SOURCE = FONTS / "source"

# Pinned: a different converter version can emit a different font struct.
CONVERTER = "lv_font_conv@1.5.3"

# Every glyph any screen prints: space, ". / 0-9 :", A-Z, degree sign,
# middle dot, em dash. test_ui checks the screens never ask for another.
RANGES = "0x20,0x2E-0x3A,0x41-0x5A,0xB0,0xB7,0x2014"

# (symbol, source file, pixel size). Sizes from the reviewed screen studies.
BUILD = [
    ("pl_font_hero", "SairaCondensed-Bold.ttf", 64),
    ("pl_font_word", "SairaCondensed-Bold.ttf", 30),
    ("pl_font_label", "SairaCondensed-Medium.ttf", 13),
    ("pl_font_marker", "SairaCondensed-Medium.ttf", 11),
]


def main() -> None:
    npx = shutil.which("npx")
    if npx is None:
        sys.exit("npx not found: install Node.js to regenerate the fonts "
                 "(the committed .c files build without it)")
    for name, ttf, size in BUILD:
        # Relative paths, run from source/, so the options recorded in each
        # generated file's header are the same on every machine.
        subprocess.run(
            [npx, "--yes", CONVERTER, "--font", ttf, "-r", RANGES,
             "--size", str(size), "--bpp", "4", "--format", "lvgl",
             "--no-compress", "--lv-include", "lvgl.h",
             "--lv-font-name", name, "-o", f"../{name}.c"],
            cwd=SOURCE, check=True)
        out = FONTS / f"{name}.c"
        print(f"{out.name}: {out.stat().st_size} bytes of C source")


if __name__ == "__main__":
    main()
