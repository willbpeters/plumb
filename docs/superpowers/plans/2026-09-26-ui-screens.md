# UI Screens on the Host Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Four result screens (face angle, tempo, path, impact speed) plus an idle screen, as a pure-LVGL component rendered headlessly on the PC and tested by measuring what is drawn against the inputs.

**Architecture:** `firmware/components/plumb_ui/` is C99 + LVGL with no hardware calls. One full-screen LVGL object draws the current screen in a `LV_EVENT_DRAW_MAIN` callback. Each screen is a pure function of `(result, elapsed ms)`; the pure maths lives in `geometry.c` with no LVGL. A host program, `firmware/test/uisnap.c`, drives the component from stdin commands, advances LVGL's clock itself, and dumps RGB565 frames that `analysis/tools/uisnap.py` turns into NumPy arrays and PNGs. `analysis/tools/cbuild.py` builds it with the compiler it already discovers.

**Tech Stack:** LVGL v9.6.0 (git submodule), MSVC or cc/gcc/clang, Python 3 + NumPy + pytest (existing `analysis/` uv project), `lv_font_conv` 1.5.3 via npx, Saira Condensed (SIL OFL 1.1).

**Spec:** `docs/superpowers/specs/2026-09-25-ui-screens-design.md`. Section numbers `§n` below refer to it.

---

## Verified before writing (spike, 2026-09-26)

These were run against LVGL v9.6.0 on this machine; the plan depends on them.

- All 483 LVGL sources compile with MSVC in ~4 s (`-MP`), and a headless display with `LV_DISPLAY_RENDER_MODE_FULL` plus a flush callback renders correctly.
- **`LV_COLOR_DEPTH` is deprecated in 9.6.** Setting it makes `lv_conf_internal.h` hit a `#warning`, and **MSVC's C compiler rejects `#warning` as fatal error C1021**. Use `LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565`.
- Public headers moved to `include/lvgl/`. Include only `"lvgl.h"`; the old `src/**/*.h` shims contain `#warning` too.
- Our code compiles at `-W4 -WX` when LVGL's headers are passed as `-external:I<lvgl> -external:W0`.
- `lv_font_conv@1.5.3` output compiles against 9.6 and renders `° · —`.
- Draw API field names in 9.6: `lv_draw_triangle_dsc_t{p[3], color, opa}`, `lv_draw_line_dsc_t{p1, p2, color, width, dash_width, dash_gap, opa, round_start, round_end}`, `lv_draw_arc_dsc_t{color, width, start_angle, end_angle, center, radius, opa, rounded}`, `lv_draw_label_dsc_t{text, font, color, letter_space, align, opa, text_local}`. Arc angles: 0° at 3 o'clock, increasing clockwise; the SW renderer reduces angles ≥ 360.

## Rules for every C file in this plan

- **ASCII only**, comments included. MSVC reads source in the system code page and warns (C4819) on anything else, and `-WX` makes that an error. Non-ASCII glyphs are written as escapes via the macros in `style.h` (`PL_UI_DEG`, `PL_UI_MIDDOT`, `PL_UI_DASH`).
- **Explicit casts for every float→int conversion.** `-W4 -WX` rejects implicit narrowing (C4244).
- **No non-constant aggregate initializers** (`lv_point_t p = {x, y}` with variables): C4204 at `-W4`. Assign fields.
- **Any text drawn from a stack buffer must set `dsc.text_local = 1`.** LVGL 9 draw tasks run after the event returns; `pl_ui_text` does this for every call.

## File map

| File | Responsibility |
|---|---|
| `.gitmodules`, `firmware/third_party/lvgl` | LVGL v9.6.0, pinned |
| `firmware/components/plumb_ui/lv_conf.h` | The one LVGL config: only our overrides |
| `firmware/components/plumb_ui/include/plumb/ui.h` | The whole public surface: result struct, API |
| `firmware/components/plumb_ui/src/geometry.{h,c}` | Pure maths: amplification, clamps, bar lengths, path ends, easing. No LVGL |
| `firmware/components/plumb_ui/src/style.{h,c}` | Palette, glyph-checked text, line/dot/arc/triangle helpers, target mark, no-reading block |
| `firmware/components/plumb_ui/src/screens.h` | The screen interface and the four screen symbols |
| `firmware/components/plumb_ui/src/screen_{face,tempo,path,speed}.c` | One screen each |
| `firmware/components/plumb_ui/src/ui.c` | Canvas, navigation, idle, animation clock, page dots |
| `firmware/components/plumb_ui/fonts/` | Generated `pl_font_*.c`, `pl_fonts.h`, `README.md`, `source/` (TTFs + `OFL.txt`) |
| `firmware/test/uisnap.c` | Headless host renderer, stdin commands |
| `analysis/tools/cbuild.py` | Gains `build_ui()` |
| `analysis/tools/fonts.py` | Regenerates the fonts |
| `analysis/tools/uisnap.py` | Python driver: run commands, render, PNG, gallery |
| `analysis/tests/test_ui.py` | All UI tests |

---

### Task 1: LVGL submodule and config

**Files:**
- Create: `.gitmodules` (via `git submodule add`), `firmware/third_party/lvgl` (submodule)
- Create: `firmware/components/plumb_ui/lv_conf.h`
- Modify: `.gitignore`

- [ ] **Step 1: Add the submodule pinned to v9.6.0**

Run from the repo root:
```bash
git submodule add https://github.com/lvgl/lvgl.git firmware/third_party/lvgl
git -C firmware/third_party/lvgl checkout v9.6.0
```

- [ ] **Step 2: Verify the pin**

Run: `git -C firmware/third_party/lvgl describe --tags`
Expected: `v9.6.0`

- [ ] **Step 3: Write the config**

Create `firmware/components/plumb_ui/lv_conf.h`:
```c
/* The one LVGL configuration for Plumb: host bench and device alike.
 *
 * Only overrides are listed; LVGL's lv_conf_internal.h supplies a default for
 * everything else. Kept short on purpose, so every line here is a decision.
 *
 * LV_COLOR_FORMAT_DEFAULT, not LV_COLOR_DEPTH: 9.6 deprecates the latter, and
 * its fallback path hits a #warning that MSVC's C compiler treats as a fatal
 * error (C1021). Measured, 2026-09-26.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

/* The GC9A01 takes RGB565 (parent spec 6.3). */
#define LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565

/* No OS inside the UI: the app task owns scheduling, and invariant 8 is
 * enforced there by not calling lv_timer_handler() during a stroke. */
#define LV_USE_OS LV_OS_NONE

/* LVGL's own heap, for objects and draw tasks. Not the draw buffers, which the
 * caller supplies (internal SRAM on the device, invariant 7). */
#define LV_MEM_SIZE (96 * 1024)

/* Parent spec 6.3: display refresh period 10 ms. Also the animation tick, so
 * the host bench resolves motion to 10 ms. */
#define LV_DEF_REFR_PERIOD 10

#define LV_USE_LOG 0
#define LV_BUILD_EXAMPLES 0
#define LV_BUILD_DEMOS 0

#endif /* LV_CONF_H */
```

- [ ] **Step 4: Ignore the host build products**

Append to `.gitignore`:
```
# built by analysis/tools/cbuild.py build_ui()
firmware/test/obj-lvgl/
firmware/test/obj-ui/
firmware/test/ui-gallery/
firmware/test/*.rgb565
```

- [ ] **Step 5: Commit**

```bash
git add .gitmodules firmware/third_party/lvgl firmware/components/plumb_ui/lv_conf.h .gitignore
git commit -m "Add LVGL v9.6.0 as a submodule, and the one lv_conf.h"
```

---

### Task 2: Fonts

**Files:**
- Create: `firmware/components/plumb_ui/fonts/source/SairaCondensed-Bold.ttf`, `SairaCondensed-Medium.ttf`, `OFL.txt`
- Create: `analysis/tools/fonts.py`
- Create (generated): `firmware/components/plumb_ui/fonts/pl_font_hero.c`, `pl_font_word.c`, `pl_font_label.c`, `pl_font_marker.c`
- Create: `firmware/components/plumb_ui/fonts/pl_fonts.h`, `firmware/components/plumb_ui/fonts/README.md`

- [ ] **Step 1: Fetch the font sources and licence**

Run from the repo root:
```bash
mkdir -p firmware/components/plumb_ui/fonts/source
cd firmware/components/plumb_ui/fonts/source
curl -sSL -o SairaCondensed-Bold.ttf https://github.com/google/fonts/raw/main/ofl/sairacondensed/SairaCondensed-Bold.ttf
curl -sSL -o SairaCondensed-Medium.ttf https://github.com/google/fonts/raw/main/ofl/sairacondensed/SairaCondensed-Medium.ttf
curl -sSL -o OFL.txt https://github.com/google/fonts/raw/main/ofl/sairacondensed/OFL.txt
```
Expected: two ~96 KB `.ttf` files; `OFL.txt` begins `Copyright 2016 The Saira Project Authors`.

- [ ] **Step 2: Write the generator**

Create `analysis/tools/fonts.py`:
```python
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
```

- [ ] **Step 3: Generate**

Run: `cd analysis; uv run python -m tools.fonts`
Expected: four lines, `pl_font_hero.c: ~150000 bytes of C source` and three smaller.

- [ ] **Step 4: Verify each file declares its symbol and the glyph ranges**

Run from the repo root:
```bash
grep -l "const lv_font_t pl_font_" firmware/components/plumb_ui/fonts/pl_font_*.c | wc -l
grep -h "^ \* Opts:" firmware/components/plumb_ui/fonts/pl_font_*.c
```
Expected: `4`, then four `Opts:` lines each containing `-r 0x20,0x2E-0x3A,0x41-0x5A,0xB0,0xB7,0x2014` and a relative `-o ../pl_font_*.c`.

- [ ] **Step 5: Write the declarations header**

Create `firmware/components/plumb_ui/fonts/pl_fonts.h`:
```c
/* The UI's fonts, generated by analysis/tools/fonts.py. See README.md. */
#ifndef PL_FONTS_H
#define PL_FONTS_H

#include "lvgl.h"

LV_FONT_DECLARE(pl_font_hero)   /* 64 px bold: the number   */
LV_FONT_DECLARE(pl_font_word)   /* 30 px bold: a word, a ratio suffix */
LV_FONT_DECLARE(pl_font_label)  /* 13 px medium: labels     */
LV_FONT_DECLARE(pl_font_marker) /* 11 px medium: faint markers */

#endif /* PL_FONTS_H */
```

- [ ] **Step 6: Write the fonts README**

Create `firmware/components/plumb_ui/fonts/README.md`:
```markdown
# UI fonts

Generated LVGL bitmap fonts. **Do not edit the `.c` files**; regenerate with
`cd analysis; uv run python -m tools.fonts`.

Derived from **Saira Condensed** by The Saira Project Authors, licensed under
the SIL Open Font License 1.1 (`source/OFL.txt`). The sources in `source/` are
unmodified. The generated fonts are converted and subset, so they are named
for their role (`pl_font_hero` and so on) rather than carrying the Reserved
Font Name.

| Symbol | Source | Size | Used for |
|---|---|---|---|
| `pl_font_hero` | Bold | 64 px | the metric's number |
| `pl_font_word` | Bold | 30 px | path direction, tempo `: 1`, the wordmark |
| `pl_font_label` | Medium | 13 px | labels |
| `pl_font_marker` | Medium | 11 px | faint markers (TOE, HEEL, TARGET, BALL, OUTSIDE) |

Glyphs: space, `. / 0-9 :`, `A-Z`, `°`, `·`, `—`. Nothing else, on purpose:
`analysis/tests/test_ui.py` renders every screen and fails if any string asks
for a glyph that is not here.

On the device the font data belongs in PSRAM (invariant 7).
```

- [ ] **Step 7: Commit**

```bash
git add firmware/components/plumb_ui/fonts analysis/tools/fonts.py
git commit -m "Generate the UI fonts from Saira Condensed, subset to the glyphs used"
```

---

### Task 3: The host bench end to end (geometry, style, UI shell, renderer, build)

This is the largest task because nothing renders until every layer exists. The four screens start as stubs that print their title; Tasks 4–7 replace them one by one.

**Files:**
- Create: `firmware/components/plumb_ui/include/plumb/ui.h`
- Create: `firmware/components/plumb_ui/src/geometry.h`, `geometry.c`
- Create: `firmware/components/plumb_ui/src/style.h`, `style.c`
- Create: `firmware/components/plumb_ui/src/screens.h`
- Create: `firmware/components/plumb_ui/src/screen_face.c`, `screen_tempo.c`, `screen_path.c`, `screen_speed.c` (stubs)
- Create: `firmware/components/plumb_ui/src/ui.c`
- Create: `firmware/test/uisnap.c`
- Modify: `analysis/tools/cbuild.py` (add `build_ui` after `build`)
- Create: `analysis/tools/uisnap.py`
- Test: `analysis/tests/test_ui.py`

- [ ] **Step 1: Write the failing tests**

Create `analysis/tests/test_ui.py`:
```python
"""Tests for the plumb_ui component, rendered headlessly.

UI spec: docs/superpowers/specs/2026-09-25-ui-screens-design.md, section 7.

The rule is the project's: check what is DRAWN against what was MEASURED, not
against a stored picture of itself. Geometry is checked against values derived
by hand here; pixels are measured from the rendered frame and compared with
the inputs that produced them.
"""

import numpy as np
import pytest

from tools import cbuild, uisnap
from tools.uisnap import IN_TO_OUT, OUT_TO_IN, STRAIGHT, Result

# The palette after RGB565 quantisation, expanded the way uisnap.to_rgb does.
BG = np.array([8, 8, 8])            # 0x0A0B0D
ACCENT = np.array([238, 182, 74])   # 0xE8B44A
FG = np.array([246, 238, 238])      # 0xF2EFE9

ROWS = np.arange(240)[:, None]      # broadcasts against a (240, 240) mask


@pytest.fixture(scope="session", autouse=True)
def bench():
    """Build the renderer once, or skip loudly: a skipped UI suite must never
    read as a pass (the lesson of test_c_port's first version)."""
    try:
        return uisnap.executable()
    except cbuild.CompilerNotFound as missing:
        message = f"THE UI IS NOT VERIFIED HERE: {missing}"
        print(f"\n*** {message}")
        pytest.skip(message)


def near(rgb, colour, tolerance=24):
    return np.all(np.abs(rgb.astype(int) - colour) <= tolerance, axis=-1)


def geom(command):
    return [float(v) for v in uisnap.run([command])[0].split()]


# -- geometry, against values derived by hand ---------------------------------

@pytest.mark.parametrize("face, drawn", [
    (1.8, -9.0),      # open: 5x, counter-clockwise on screen
    (-1.8, 9.0),      # closed: clockwise
    (0.0, 0.0),
    (12.0, -45.0),    # 60 deg drawn would spin the head sideways: clamped
    (-20.0, 45.0),
])
def test_face_rotation_is_five_times_the_angle_and_clamped(face, drawn):
    assert geom(f"facerot {face}")[0] == pytest.approx(drawn, abs=1e-4)


@pytest.mark.parametrize("face, side", [(0.04, 0), (-0.04, 0), (0.06, 1), (-0.06, 2)])
def test_square_only_below_the_one_decimal_rounding_limit(face, side):
    """0 square, 1 open, 2 closed. 0.05 is where %.1f stops printing 0.0: a
    display resolution, not a verdict."""
    assert geom(f"faceside {face}")[0] == side


@pytest.mark.parametrize("back, thru, expect", [
    (0.70, 0.35, (82.6, 41.3)),    # 118 px/s, fits
    (1.00, 0.50, (90.0, 45.0)),    # 118 px overruns 90: both scaled by 90/118
    (0.30, 0.90, (30.0, 90.0)),    # the through bar is the long one here
])
def test_tempo_bars_share_one_scale(back, thru, expect):
    assert geom(f"tempobars {back} {thru}") == pytest.approx(expect, abs=1e-3)


@pytest.mark.parametrize("direction, arc, travel, expect", [
    # h = 4 x 0.006 x 152 / 0.30 = 12.16 px
    (OUT_TO_IN, 0.006, 0.30, (196.0, 109.84, 44.0, 134.16)),
    (IN_TO_OUT, 0.006, 0.30, (196.0, 134.16, 44.0, 109.84)),
    (STRAIGHT, 0.006, 0.30, (196.0, 122.0, 44.0, 122.0)),
    (OUT_TO_IN, 0.030, 0.30, (196.0, 72.0, 44.0, 172.0)),     # 60.8 clamped to 50
    (OUT_TO_IN, 0.006, 0.00, (196.0, 122.0, 44.0, 122.0)),    # no travel: flat
])
def test_path_ends_are_the_arc_amplified_and_scaled_by_travel(direction, arc, travel, expect):
    assert geom(f"pathends {direction} {arc} {travel}") == pytest.approx(expect, abs=1e-3)


@pytest.mark.parametrize("mps, fraction", [(1.5, 0.5), (0.75, 0.25), (4.0, 1.0), (-1.0, 0.0)])
def test_speed_ring_fraction(mps, fraction):
    assert geom(f"speedfrac {mps}")[0] == pytest.approx(fraction, abs=1e-6)


# -- the glyph check has to be able to fail ------------------------------------

def test_the_glyph_counter_counts_what_the_font_lacks():
    """Lowercase is deliberately not in the fonts, so it must count as
    missing; the strings the screens use must not."""
    assert uisnap.run(["probe abc"]) == ["3"]
    assert uisnap.run(["probe 1.8° · —"]) == ["0"]


# -- the shell ------------------------------------------------------------------

def test_idle_shows_the_wordmark_and_nothing_else():
    rgb = uisnap.render(idle=True)
    ys, _ = np.nonzero(near(rgb, FG))
    assert len(ys) > 50, "the wordmark should be drawn"
    assert np.all(np.abs(ys - 120) < 30), "only the wordmark, centred"
    assert not near(rgb, ACCENT).any(), "no metric, no page dots"


def test_navigation_wraps_and_a_new_result_returns_to_face_angle():
    r = Result().command()
    assert uisnap.run(["screen", "next", "screen"]) == ["-1", "-1"], \
        "with no result there is nothing to swipe to"
    out = uisnap.run([r, "screen", "next", "screen", "next", "next", "next",
                      "screen", "prev", "screen", r, "screen"])
    assert out == ["0", "1", "0", "3", "0"]


@pytest.mark.parametrize("screen, dot_x", [(0, 102), (1, 114), (2, 126), (3, 138)])
def test_the_current_page_dot_is_lit(screen, dot_x):
    rgb = uisnap.render(Result(), screen=screen)
    ys, xs = np.nonzero(near(rgb, ACCENT) & (ROWS > 224))
    assert len(xs) > 5
    assert xs.mean() == pytest.approx(dot_x, abs=2)


@pytest.mark.parametrize("screen", [None, 0, 1, 2, 3])
def test_nothing_is_drawn_outside_the_round_aperture(screen):
    rgb = uisnap.render(idle=True) if screen is None else uisnap.render(Result(), screen=screen)
    yy, xx = np.mgrid[0:240, 0:240]
    outside = np.hypot(xx - 119.5, yy - 119.5) > 121
    assert np.all(rgb[outside] == BG), "something lit outside the 240 px circle"


@pytest.mark.parametrize("screen", [0, 1, 2, 3])
def test_a_metric_without_a_reading_draws_no_graphic(screen):
    empty = Result(face=None, backswing_s=None, path_dir=None, speed=None)
    rgb = uisnap.render(empty, screen=screen)
    assert not (near(rgb, ACCENT) & (ROWS < 220)).any()
```

- [ ] **Step 2: Run to see it fail**

Run: `cd analysis; uv run pytest tests/test_ui.py -q`
Expected: collection error, `ImportError: cannot import name 'uisnap' from 'tools'`.

- [ ] **Step 3: Write the public header**

Create `firmware/components/plumb_ui/include/plumb/ui.h`:
```c
/* plumb_ui: the result screens. Pure LVGL, no hardware (CLAUDE.md, stack).
 *
 * The UI takes a results struct and draws it; it never computes a metric.
 * Invariant 8 -- nothing renders between arm and follow-through -- is the app
 * task's to enforce, by not calling lv_timer_handler() in that window. The UI
 * starts motion only in pl_ui_show_result(), which the app calls after DONE.
 */
#ifndef PLUMB_UI_H
#define PLUMB_UI_H

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"

typedef enum {
    PL_PATH_STRAIGHT = 0,
    PL_PATH_IN_TO_OUT = 1,
    PL_PATH_OUT_TO_IN = 2
} pl_path_dir;

typedef struct {
    bool face_valid;
    float face_angle_deg;       /* + open, relative to address (invariant 3) */

    bool tempo_valid;
    float backswing_s;
    float downswing_s;          /* tempo ratio = backswing_s / downswing_s */

    bool path_valid;
    pl_path_dir path_dir;
    float path_arc_m;           /* shapes the drawing; never shown */
    float path_travel_m;        /* scales it */

    bool speed_valid;
    float impact_speed_mps;     /* clubhead, not ball */
} pl_ui_result;

enum {
    PL_UI_SCREEN_FACE = 0,
    PL_UI_SCREEN_TEMPO,
    PL_UI_SCREEN_PATH,
    PL_UI_SCREEN_SPEED,
    PL_UI_SCREEN_COUNT
};
#define PL_UI_IDLE (-1)

void pl_ui_init(lv_display_t *display);
void pl_ui_show_result(const pl_ui_result *result);
void pl_ui_next(void);
void pl_ui_prev(void);
void pl_ui_idle(void);
int pl_ui_current_screen(void);

/* Diagnostic: glyphs asked for and not in the font, since init. A missing
 * glyph renders as a silent blank, so this is how it gets noticed. */
uint32_t pl_ui_missing_glyphs(void);

#endif /* PLUMB_UI_H */
```

- [ ] **Step 4: Write the geometry**

Create `firmware/components/plumb_ui/src/geometry.h`:
```c
/* Pure display maths for plumb_ui. No LVGL, so it is testable on its own.
 *
 * Every constant here is a DISPLAY range or amplification, not a measurement
 * threshold; invariant 5 does not apply to them. The numbers shown on screen
 * are never amplified or clamped, only the drawings.
 */
#ifndef PL_UI_GEOMETRY_H
#define PL_UI_GEOMETRY_H

#include "plumb/ui.h"

/* 1.8 deg across a 110 px head moves the toe three pixels: invisible. */
#define PL_UI_FACE_GAIN 5.0f
/* Past this a 12 deg reading would spin the head sideways. */
#define PL_UI_FACE_CLAMP_DEG 45.0f
/* Where %.1f stops printing 0.0: a display resolution, not a verdict. */
#define PL_UI_SQUARE_DEG 0.05f

/* Same speed both ways, so bar length IS duration. */
#define PL_UI_TEMPO_PX_PER_S 118.0f
/* Longest bar that stays inside the aperture with its round cap. */
#define PL_UI_TEMPO_MAX_PX 90.0f

#define PL_UI_ARC_GAIN 4.0f
#define PL_UI_PATH_SPAN_PX 152.0f
#define PL_UI_PATH_CLAMP_PX 50.0f
#define PL_UI_PATH_BASELINE_Y 122.0f
#define PL_UI_CENTRE_X 120.0f

/* Typical putts land a third to two-thirds round. */
#define PL_UI_SPEED_FULL_SCALE_MPS 3.0f
#define PL_UI_SPEED_MAX_SWEEP_DEG 270.0f

typedef enum {
    PL_UI_FACE_SQUARE = 0,
    PL_UI_FACE_OPEN = 1,
    PL_UI_FACE_CLOSED = 2
} pl_ui_face_side;

float pl_ui_clamp01(float u);
float pl_ui_ease_out(float u);      /* cubic, u clamped to 0..1 */
float pl_ui_ease_in_out(float u);   /* smoothstep, u clamped to 0..1 */

/* On-screen rotation of the head, degrees, positive CLOCKWISE on screen.
 * Open turns the toe toward the target (left), so open is negative. */
float pl_ui_face_rotation_cw_deg(float face_deg);
pl_ui_face_side pl_ui_face_side_of(float face_deg);

/* Rotate (x, y) about (cx, cy) by deg_cw, clockwise on a y-down screen. */
void pl_ui_rotate_cw(float cx, float cy, float deg_cw, float x, float y,
                     float *out_x, float *out_y);

void pl_ui_tempo_bars(float backswing_s, float downswing_s,
                      float *back_px, float *thru_px);

/* The stroke runs right to left: (x0, y0) is where it starts, on the right. */
void pl_ui_path_ends(pl_path_dir dir, float arc_m, float travel_m,
                     float *x0, float *y0, float *x1, float *y1);

float pl_ui_speed_fraction(float mps);

#endif /* PL_UI_GEOMETRY_H */
```

Create `firmware/components/plumb_ui/src/geometry.c`:
```c
/* See geometry.h. */

#include "geometry.h"

#include <math.h>

#define PL_UI_PI 3.14159265358979f

float pl_ui_clamp01(float u)
{
    if (u < 0.0f) {
        return 0.0f;
    }
    if (u > 1.0f) {
        return 1.0f;
    }
    return u;
}

float pl_ui_ease_out(float u)
{
    const float v = 1.0f - pl_ui_clamp01(u);
    return 1.0f - v * v * v;
}

float pl_ui_ease_in_out(float u)
{
    const float v = pl_ui_clamp01(u);
    return v * v * (3.0f - 2.0f * v);
}

float pl_ui_face_rotation_cw_deg(float face_deg)
{
    float drawn = PL_UI_FACE_GAIN * face_deg;
    if (drawn > PL_UI_FACE_CLAMP_DEG) {
        drawn = PL_UI_FACE_CLAMP_DEG;
    }
    if (drawn < -PL_UI_FACE_CLAMP_DEG) {
        drawn = -PL_UI_FACE_CLAMP_DEG;
    }
    return -drawn;
}

pl_ui_face_side pl_ui_face_side_of(float face_deg)
{
    if (fabsf(face_deg) < PL_UI_SQUARE_DEG) {
        return PL_UI_FACE_SQUARE;
    }
    return face_deg > 0.0f ? PL_UI_FACE_OPEN : PL_UI_FACE_CLOSED;
}

void pl_ui_rotate_cw(float cx, float cy, float deg_cw, float x, float y,
                     float *out_x, float *out_y)
{
    const float a = deg_cw * (PL_UI_PI / 180.0f);
    const float c = cosf(a);
    const float s = sinf(a);
    const float dx = x - cx;
    const float dy = y - cy;
    *out_x = cx + dx * c - dy * s;
    *out_y = cy + dx * s + dy * c;
}

void pl_ui_tempo_bars(float backswing_s, float downswing_s,
                      float *back_px, float *thru_px)
{
    float back = PL_UI_TEMPO_PX_PER_S * (backswing_s > 0.0f ? backswing_s : 0.0f);
    float thru = PL_UI_TEMPO_PX_PER_S * (downswing_s > 0.0f ? downswing_s : 0.0f);
    const float longest = back > thru ? back : thru;
    if (longest > PL_UI_TEMPO_MAX_PX) {
        /* One factor for both, so the ratio on screen stays exact. */
        const float scale = PL_UI_TEMPO_MAX_PX / longest;
        back *= scale;
        thru *= scale;
    }
    *back_px = back;
    *thru_px = thru;
}

void pl_ui_path_ends(pl_path_dir dir, float arc_m, float travel_m,
                     float *x0, float *y0, float *x1, float *y1)
{
    float h = 0.0f;
    if (dir != PL_PATH_STRAIGHT && travel_m > 0.0f && arc_m > 0.0f) {
        h = PL_UI_ARC_GAIN * arc_m * (PL_UI_PATH_SPAN_PX / travel_m);
        if (h > PL_UI_PATH_CLAMP_PX) {
            h = PL_UI_PATH_CLAMP_PX;
        }
    }
    *x0 = PL_UI_CENTRE_X + 0.5f * PL_UI_PATH_SPAN_PX;
    *x1 = PL_UI_CENTRE_X - 0.5f * PL_UI_PATH_SPAN_PX;
    if (dir == PL_PATH_IN_TO_OUT) {
        /* Starts inside (below, the golfer's side), finishes outside. */
        *y0 = PL_UI_PATH_BASELINE_Y + h;
        *y1 = PL_UI_PATH_BASELINE_Y - h;
    } else {
        /* Out-to-in starts outside (above); straight has h = 0. */
        *y0 = PL_UI_PATH_BASELINE_Y - h;
        *y1 = PL_UI_PATH_BASELINE_Y + h;
    }
}

float pl_ui_speed_fraction(float mps)
{
    return pl_ui_clamp01(mps / PL_UI_SPEED_FULL_SCALE_MPS);
}
```

- [ ] **Step 5: Write the style layer**

Create `firmware/components/plumb_ui/src/style.h`:
```c
/* Palette, text and drawing helpers shared by every screen.
 * Colours are the reviewed screen studies'. */
#ifndef PL_UI_STYLE_H
#define PL_UI_STYLE_H

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"
#include "../fonts/pl_fonts.h"

#define PL_UI_BG     lv_color_hex(0x0A0B0D)
#define PL_UI_FG     lv_color_hex(0xF2EFE9)
#define PL_UI_DIM    lv_color_hex(0x6E747B)
#define PL_UI_FAINT  lv_color_hex(0x2A2E34)
#define PL_UI_STEEL  lv_color_hex(0x8E959D)
#define PL_UI_ACCENT lv_color_hex(0xE8B44A)

/* UTF-8, as escapes: C sources stay ASCII (MSVC, C4819). Kept as separate
 * string literals so a following hex digit cannot join the escape. */
#define PL_UI_DEG    "\xC2\xB0"
#define PL_UI_MIDDOT "\xC2\xB7"
#define PL_UI_DASH   "\xE2\x80\x94"

typedef enum {
    PL_UI_ALIGN_LEFT,
    PL_UI_ALIGN_CENTER,
    PL_UI_ALIGN_RIGHT
} pl_ui_align;

int32_t pl_ui_px(float v);

/* Draw `text` with its baseline at `baseline`, anchored at x by `align`.
 * Counts any glyph the font lacks (pl_ui_missing_glyphs). */
void pl_ui_text(lv_layer_t *layer, const lv_font_t *font, lv_color_t color,
                lv_opa_t opa, const char *text, int32_t x, int32_t baseline,
                pl_ui_align align, int32_t letter_space);
int32_t pl_ui_text_width(const lv_font_t *font, const char *text,
                         int32_t letter_space);
uint32_t pl_ui_count_missing(const lv_font_t *font, const char *text);

void pl_ui_line(lv_layer_t *layer, lv_color_t color, int32_t width,
                float x1, float y1, float x2, float y2, bool round,
                int32_t dash, int32_t gap);
/* A straight bar from (x1,y1) to (x2,y2), rotated about (cx,cy). */
void pl_ui_rotated_bar(lv_layer_t *layer, lv_color_t color, int32_t width,
                       float x1, float y1, float x2, float y2,
                       float cx, float cy, float deg_cw);
void pl_ui_triangle(lv_layer_t *layer, lv_color_t color, float ax, float ay,
                    float bx, float by, float cx, float cy);
void pl_ui_dot(lv_layer_t *layer, lv_color_t color, float cx, float cy,
               int32_t radius);
void pl_ui_arc(lv_layer_t *layer, lv_color_t color, int32_t radius,
               int32_t width, int32_t start_deg, int32_t end_deg, bool rounded);

/* The faint arrow and TARGET at the left edge: left is the target. */
void pl_ui_target_mark(lv_layer_t *layer);
/* What a metric without a reading shows: a faint dash pair and its title. */
void pl_ui_no_reading(lv_layer_t *layer, const char *title);

#endif /* PL_UI_STYLE_H */
```

Create `firmware/components/plumb_ui/src/style.c`:
```c
/* See style.h. */

#include "style.h"

#include <math.h>

#include "geometry.h"
#include "plumb/ui.h"

static uint32_t s_missing_glyphs;

/* All our strings are well-formed UTF-8, from the macros in style.h. */
static uint32_t next_codepoint(const char **text)
{
    const unsigned char *s = (const unsigned char *)*text;
    uint32_t c;
    if (s[0] < 0x80u) {
        c = s[0];
        *text += 1;
    } else if ((s[0] & 0xE0u) == 0xC0u) {
        c = ((uint32_t)(s[0] & 0x1Fu) << 6) | (uint32_t)(s[1] & 0x3Fu);
        *text += 2;
    } else if ((s[0] & 0xF0u) == 0xE0u) {
        c = ((uint32_t)(s[0] & 0x0Fu) << 12) | ((uint32_t)(s[1] & 0x3Fu) << 6)
            | (uint32_t)(s[2] & 0x3Fu);
        *text += 3;
    } else {
        c = 0xFFFDu;
        *text += 1;
    }
    return c;
}

int32_t pl_ui_px(float v)
{
    return (int32_t)lroundf(v);
}

uint32_t pl_ui_count_missing(const lv_font_t *font, const char *text)
{
    uint32_t missing = 0;
    const char *p = text;
    while (*p != '\0') {
        lv_font_glyph_dsc_t glyph;
        const uint32_t c = next_codepoint(&p);
        if (!lv_font_get_glyph_dsc(font, &glyph, c, 0)) {
            missing++;
        }
    }
    return missing;
}

uint32_t pl_ui_missing_glyphs(void)
{
    return s_missing_glyphs;
}

int32_t pl_ui_text_width(const lv_font_t *font, const char *text,
                         int32_t letter_space)
{
    lv_point_t size;
    lv_text_get_size(&size, text, font, letter_space, 0, LV_COORD_MAX,
                     LV_TEXT_FLAG_NONE);
    return size.x;
}

void pl_ui_text(lv_layer_t *layer, const lv_font_t *font, lv_color_t color,
                lv_opa_t opa, const char *text, int32_t x, int32_t baseline,
                pl_ui_align align, int32_t letter_space)
{
    lv_draw_label_dsc_t dsc;
    lv_area_t area;
    const int32_t line_height = lv_font_get_line_height(font);
    const int32_t width = pl_ui_text_width(font, text, letter_space);
    const int32_t top = baseline - (line_height - font->base_line);

    s_missing_glyphs += pl_ui_count_missing(font, text);

    if (align == PL_UI_ALIGN_CENTER) {
        x -= width / 2;
    } else if (align == PL_UI_ALIGN_RIGHT) {
        x -= width;
    }
    area.x1 = x;
    area.y1 = top;
    area.x2 = x + width;
    area.y2 = top + line_height;

    lv_draw_label_dsc_init(&dsc);
    dsc.text = text;
    /* Draw tasks run after this returns, and callers pass stack buffers. */
    dsc.text_local = 1;
    dsc.font = font;
    dsc.color = color;
    dsc.opa = opa;
    dsc.letter_space = letter_space;
    dsc.align = LV_TEXT_ALIGN_LEFT;
    lv_draw_label(layer, &dsc, &area);
}

void pl_ui_line(lv_layer_t *layer, lv_color_t color, int32_t width,
                float x1, float y1, float x2, float y2, bool round,
                int32_t dash, int32_t gap)
{
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = LV_OPA_COVER;
    dsc.width = width;
    dsc.p1.x = pl_ui_px(x1);
    dsc.p1.y = pl_ui_px(y1);
    dsc.p2.x = pl_ui_px(x2);
    dsc.p2.y = pl_ui_px(y2);
    dsc.round_start = (uint8_t)(round ? 1 : 0);
    dsc.round_end = (uint8_t)(round ? 1 : 0);
    dsc.dash_width = dash;
    dsc.dash_gap = gap;
    lv_draw_line(layer, &dsc);
}

void pl_ui_rotated_bar(lv_layer_t *layer, lv_color_t color, int32_t width,
                       float x1, float y1, float x2, float y2,
                       float cx, float cy, float deg_cw)
{
    float ax, ay, bx, by;
    pl_ui_rotate_cw(cx, cy, deg_cw, x1, y1, &ax, &ay);
    pl_ui_rotate_cw(cx, cy, deg_cw, x2, y2, &bx, &by);
    pl_ui_line(layer, color, width, ax, ay, bx, by, false, 0, 0);
}

void pl_ui_triangle(lv_layer_t *layer, lv_color_t color, float ax, float ay,
                    float bx, float by, float cx, float cy)
{
    lv_draw_triangle_dsc_t dsc;
    lv_draw_triangle_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = LV_OPA_COVER;
    dsc.p[0].x = pl_ui_px(ax);
    dsc.p[0].y = pl_ui_px(ay);
    dsc.p[1].x = pl_ui_px(bx);
    dsc.p[1].y = pl_ui_px(by);
    dsc.p[2].x = pl_ui_px(cx);
    dsc.p[2].y = pl_ui_px(cy);
    lv_draw_triangle(layer, &dsc);
}

void pl_ui_dot(lv_layer_t *layer, lv_color_t color, float cx, float cy,
               int32_t radius)
{
    lv_draw_rect_dsc_t dsc;
    lv_area_t area;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_color = color;
    dsc.bg_opa = LV_OPA_COVER;
    dsc.radius = LV_RADIUS_CIRCLE;
    area.x1 = pl_ui_px(cx) - radius;
    area.y1 = pl_ui_px(cy) - radius;
    area.x2 = pl_ui_px(cx) + radius;
    area.y2 = pl_ui_px(cy) + radius;
    lv_draw_rect(layer, &dsc, &area);
}

void pl_ui_arc(lv_layer_t *layer, lv_color_t color, int32_t radius,
               int32_t width, int32_t start_deg, int32_t end_deg, bool rounded)
{
    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = LV_OPA_COVER;
    dsc.width = width;
    dsc.center.x = 120;
    dsc.center.y = 120;
    dsc.radius = (uint16_t)radius;
    dsc.start_angle = start_deg;
    dsc.end_angle = end_deg;
    dsc.rounded = (uint8_t)(rounded ? 1 : 0);
    lv_draw_arc(layer, &dsc);
}

void pl_ui_target_mark(lv_layer_t *layer)
{
    pl_ui_triangle(layer, PL_UI_FAINT, 14.0f, 120.0f, 24.0f, 114.0f, 24.0f, 126.0f);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "TARGET",
               30, 124, PL_UI_ALIGN_LEFT, 2);
}

void pl_ui_no_reading(lv_layer_t *layer, const char *title)
{
    pl_ui_text(layer, &pl_font_hero, PL_UI_FAINT, LV_OPA_COVER,
               PL_UI_DASH " " PL_UI_DASH, 120, 150, PL_UI_ALIGN_CENTER, 0);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, title,
               120, 178, PL_UI_ALIGN_CENTER, 2);
}
```

- [ ] **Step 6: Write the screen interface and the four stubs**

Create `firmware/components/plumb_ui/src/screens.h`:
```c
/* A screen is a pure function of the result and the ms since it appeared. */
#ifndef PL_UI_SCREENS_H
#define PL_UI_SCREENS_H

#include <stdint.h>

#include "lvgl.h"
#include "plumb/ui.h"

typedef struct {
    void (*draw)(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms);
    /* How long its motion runs; 0 for none. */
    uint32_t (*duration_ms)(const pl_ui_result *result);
} pl_ui_screen;

extern const pl_ui_screen pl_ui_screen_face;
extern const pl_ui_screen pl_ui_screen_tempo;
extern const pl_ui_screen pl_ui_screen_path;
extern const pl_ui_screen pl_ui_screen_speed;

#endif /* PL_UI_SCREENS_H */
```

Create each stub. `firmware/components/plumb_ui/src/screen_face.c`:
```c
/* Face angle. Stub until Task 4. */
#include "screens.h"
#include "style.h"

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    (void)result;
    (void)t_ms;
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "FACE ANGLE",
               120, 124, PL_UI_ALIGN_CENTER, 2);
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    (void)result;
    return 0;
}

const pl_ui_screen pl_ui_screen_face = { draw, duration_ms };
```
`screen_tempo.c`, `screen_path.c`, `screen_speed.c`: identical except the first comment line, the title string (`"TEMPO"`, `"PATH"`, `"IMPACT SPEED"`) and the exported symbol (`pl_ui_screen_tempo`, `pl_ui_screen_path`, `pl_ui_screen_speed`).

- [ ] **Step 7: Write the UI shell**

Create `firmware/components/plumb_ui/src/ui.c`:
```c
/* See plumb/ui.h. One full-screen object draws whichever screen is current,
 * so every pixel on the display comes from a draw callback we wrote. */

#include "plumb/ui.h"

#include "screens.h"
#include "style.h"

static const pl_ui_screen *const SCREENS[PL_UI_SCREEN_COUNT] = {
    &pl_ui_screen_face,
    &pl_ui_screen_tempo,
    &pl_ui_screen_path,
    &pl_ui_screen_speed,
};

static lv_obj_t *s_canvas;
static pl_ui_result s_result;
static bool s_has_result;
static int s_screen = PL_UI_IDLE;
static int32_t s_t_ms;

static void set_t(void *var, int32_t value)
{
    (void)var;
    s_t_ms = value;
    lv_obj_invalidate(s_canvas);
}

/* The motion clock. An lv_anim, which runs inside lv_timer_handler() and
 * therefore never during a stroke (invariant 8 is the app's to enforce). */
static void start_motion(void)
{
    lv_anim_delete(&s_t_ms, set_t);
    s_t_ms = 0;
    if (s_screen != PL_UI_IDLE) {
        const uint32_t duration = SCREENS[s_screen]->duration_ms(&s_result);
        if (duration > 0) {
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, &s_t_ms);
            lv_anim_set_exec_cb(&a, set_t);
            lv_anim_set_values(&a, 0, (int32_t)duration);
            lv_anim_set_duration(&a, duration);
            lv_anim_start(&a);
        }
    }
    lv_obj_invalidate(s_canvas);
}

static void draw_dots(lv_layer_t *layer)
{
    int i;
    for (i = 0; i < PL_UI_SCREEN_COUNT; i++) {
        pl_ui_dot(layer, i == s_screen ? PL_UI_ACCENT : PL_UI_FAINT,
                  (float)(102 + 12 * i), 232.0f, 3);
    }
}

static void draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    if (s_screen == PL_UI_IDLE) {
        pl_ui_text(layer, &pl_font_word, PL_UI_FG, LV_OPA_COVER, "PLUMB",
                   120, 131, PL_UI_ALIGN_CENTER, 6);
        return;
    }
    SCREENS[s_screen]->draw(layer, &s_result, (uint32_t)s_t_ms);
    draw_dots(layer);
}

void pl_ui_init(lv_display_t *display)
{
    lv_obj_t *screen = lv_display_get_screen_active(display);
    lv_obj_set_style_bg_color(screen, PL_UI_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    s_canvas = lv_obj_create(screen);
    lv_obj_remove_style_all(s_canvas);
    lv_obj_set_pos(s_canvas, 0, 0);
    lv_obj_set_size(s_canvas, 240, 240);
    lv_obj_add_event_cb(s_canvas, draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_has_result = false;
    s_screen = PL_UI_IDLE;
    start_motion();
}

void pl_ui_show_result(const pl_ui_result *result)
{
    s_result = *result;
    s_has_result = true;
    s_screen = PL_UI_SCREEN_FACE;   /* the primary metric, after every stroke */
    start_motion();
}

void pl_ui_next(void)
{
    if (!s_has_result) {
        return;
    }
    s_screen = s_screen == PL_UI_IDLE ? PL_UI_SCREEN_FACE
                                      : (s_screen + 1) % PL_UI_SCREEN_COUNT;
    start_motion();
}

void pl_ui_prev(void)
{
    if (!s_has_result) {
        return;
    }
    s_screen = s_screen == PL_UI_IDLE
                   ? PL_UI_SCREEN_FACE
                   : (s_screen + PL_UI_SCREEN_COUNT - 1) % PL_UI_SCREEN_COUNT;
    start_motion();
}

void pl_ui_idle(void)
{
    s_screen = PL_UI_IDLE;
    start_motion();
}

int pl_ui_current_screen(void)
{
    return s_screen;
}
```

- [ ] **Step 8: Write the headless renderer**

Create `firmware/test/uisnap.c`:
```c
/* Headless renderer for plumb_ui: the UI's test bench.
 *
 * Reads commands on stdin, one per line, and answers on stdout -- the same
 * shape as portcheck.c. analysis/tools/uisnap.py drives it.
 *
 * LVGL's clock is advanced here, 1 ms at a time, and nothing reads a wall
 * clock, so every frame is a pure function of the commands before it.
 *
 *   result F face T back thru P dir arc travel S speed   show a result
 *   next | prev | idle                                   navigate
 *   advance MS                                           run the clock
 *   snap PATH                                            write RGB565 frame
 *   screen                                               print current index
 *   missing                                              print glyph misses
 *   probe TEXT                                           misses in hero font
 *   facerot DEG | faceside DEG | tempobars B T |
 *   pathends DIR ARC TRAVEL | speedfrac MPS              geometry
 */

#include <stdio.h>
#include <string.h>

#include "lvgl.h"
#include "plumb/ui.h"
#include "geometry.h"
#include "style.h"

#define W 240
#define H 240
#define LINE_MAX_CHARS 640

static uint16_t draw_buffer[W * H];
static uint16_t frame[W * H];

static void flush(lv_display_t *display, const lv_area_t *area, uint8_t *pixels)
{
    const uint16_t *src = (const uint16_t *)pixels;
    const int32_t width = lv_area_get_width(area);
    int32_t y;
    for (y = area->y1; y <= area->y2; y++) {
        memcpy(&frame[y * W + area->x1], &src[(y - area->y1) * width],
               (size_t)width * sizeof(uint16_t));
    }
    lv_display_flush_ready(display);
}

static void advance(unsigned ms)
{
    unsigned i;
    for (i = 0; i < ms; i++) {
        lv_tick_inc(1);
        lv_timer_handler();
    }
}

static int snap(const char *path)
{
    FILE *f;
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(NULL);
    f = fopen(path, "wb");
    if (f == NULL) {
        return 0;
    }
    fwrite(frame, sizeof(uint16_t), W * H, f);
    fclose(f);
    return 1;
}

int main(void)
{
    char line[LINE_MAX_CHARS];
    lv_display_t *display;

    lv_init();
    display = lv_display_create(W, H);
    lv_display_set_buffers(display, draw_buffer, NULL, sizeof(draw_buffer),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, flush);
    pl_ui_init(display);

    while (fgets(line, sizeof(line), stdin) != NULL) {
        char op[32];
        char text[LINE_MAX_CHARS];
        float a, b, c;
        int flag;

        if (sscanf(line, "%31s", op) != 1) {
            continue;
        }

        if (strcmp(op, "result") == 0) {
            pl_ui_result r;
            int fv, tv, pv, sv, dir;
            memset(&r, 0, sizeof(r));
            if (sscanf(line, "%31s %d %f %d %f %f %d %d %f %f %d %f", op,
                       &fv, &r.face_angle_deg, &tv, &r.backswing_s,
                       &r.downswing_s, &pv, &dir, &r.path_arc_m,
                       &r.path_travel_m, &sv, &r.impact_speed_mps) != 12) {
                fprintf(stderr, "bad result: %s", line);
                return 1;
            }
            r.face_valid = fv != 0;
            r.tempo_valid = tv != 0;
            r.path_valid = pv != 0;
            r.path_dir = (pl_path_dir)dir;
            r.speed_valid = sv != 0;
            pl_ui_show_result(&r);
        } else if (strcmp(op, "next") == 0) {
            pl_ui_next();
        } else if (strcmp(op, "prev") == 0) {
            pl_ui_prev();
        } else if (strcmp(op, "idle") == 0) {
            pl_ui_idle();
        } else if (strcmp(op, "advance") == 0) {
            unsigned ms;
            if (sscanf(line, "%31s %u", op, &ms) != 2) {
                fprintf(stderr, "bad advance: %s", line);
                return 1;
            }
            advance(ms);
        } else if (strcmp(op, "snap") == 0) {
            if (sscanf(line, "%31s %639[^\r\n]", op, text) != 2 || !snap(text)) {
                fprintf(stderr, "bad snap: %s", line);
                return 1;
            }
        } else if (strcmp(op, "screen") == 0) {
            printf("%d\n", pl_ui_current_screen());
        } else if (strcmp(op, "missing") == 0) {
            printf("%u\n", (unsigned)pl_ui_missing_glyphs());
        } else if (strcmp(op, "probe") == 0) {
            if (sscanf(line, "%31s %639[^\r\n]", op, text) != 2) {
                fprintf(stderr, "bad probe: %s", line);
                return 1;
            }
            printf("%u\n", (unsigned)pl_ui_count_missing(&pl_font_hero, text));
        } else if (strcmp(op, "facerot") == 0 && sscanf(line, "%31s %f", op, &a) == 2) {
            printf("%.6f\n", (double)pl_ui_face_rotation_cw_deg(a));
        } else if (strcmp(op, "faceside") == 0 && sscanf(line, "%31s %f", op, &a) == 2) {
            printf("%d\n", (int)pl_ui_face_side_of(a));
        } else if (strcmp(op, "tempobars") == 0
                   && sscanf(line, "%31s %f %f", op, &a, &b) == 3) {
            float back, thru;
            pl_ui_tempo_bars(a, b, &back, &thru);
            printf("%.6f %.6f\n", (double)back, (double)thru);
        } else if (strcmp(op, "pathends") == 0
                   && sscanf(line, "%31s %d %f %f", op, &flag, &a, &b) == 4) {
            float x0, y0, x1, y1;
            pl_ui_path_ends((pl_path_dir)flag, a, b, &x0, &y0, &x1, &y1);
            printf("%.6f %.6f %.6f %.6f\n", (double)x0, (double)y0,
                   (double)x1, (double)y1);
        } else if (strcmp(op, "speedfrac") == 0 && sscanf(line, "%31s %f", op, &c) == 2) {
            printf("%.6f\n", (double)pl_ui_speed_fraction(c));
        } else {
            fprintf(stderr, "unknown or malformed: %s", line);
            return 1;
        }
        fflush(stdout);
    }
    return 0;
}
```

- [ ] **Step 9: Add `build_ui` to cbuild.py**

In `analysis/tools/cbuild.py`, add after the `SOURCES = [...]` line:
```python
LVGL = REPO / "firmware" / "third_party" / "lvgl"
UI = REPO / "firmware" / "components" / "plumb_ui"
```

Add this block after the `build()` function and before `if __name__ == "__main__":`:
```python
def _unix_objects(cc: str, flags: list[str], sources: list[Path],
                  directory: Path) -> list[Path]:
    """Compile each source to an object, in parallel. cc has no -MP."""
    from concurrent.futures import ThreadPoolExecutor

    def one(source: Path):
        obj = directory / f"{source.stem}.o"
        result = subprocess.run([cc, *flags, "-c", str(source), "-o", str(obj)],
                                capture_output=True, text=True)
        return source, result

    with ThreadPoolExecutor() as pool:
        for source, result in pool.map(one, sources):
            if result.returncode != 0:
                raise RuntimeError(f"compiling {source} failed:\n"
                                   f"{result.stdout}\n{result.stderr}")
    return [directory / f"{s.stem}.o" for s in sources]


def build_ui(name: str = "uisnap.exe") -> BuildResult:
    """Build the headless UI renderer: LVGL, the plumb_ui component, uisnap.c.

    LVGL is compiled once into firmware/test/obj-lvgl and reused until its
    sources, its version or lv_conf.h change. It is third-party code built
    with relaxed warnings; our code is held to -W4 -WX (or -Wall -Wextra
    -Werror), with LVGL's headers marked external so their warnings cannot
    fail our build.
    """
    if not (LVGL / "src").is_dir():
        raise RuntimeError("LVGL is missing: run `git submodule update --init` "
                           "in the repository root")
    lvgl_sources = sorted((LVGL / "src").rglob("*.c"))
    stems = [s.stem for s in lvgl_sources]
    if len(set(stems)) != len(stems):
        raise RuntimeError("two LVGL sources share a file name; the one-"
                           "directory object cache cannot hold both")
    fonts = sorted((UI / "fonts").glob("pl_font_*.c"))
    ours = sorted((UI / "src").glob("*.c")) + [HARNESS / "uisnap.c"]

    output = HARNESS / name
    lib_dir = HARNESS / "obj-lvgl"
    our_dir = HARNESS / "obj-ui"
    lib_dir.mkdir(parents=True, exist_ok=True)
    our_dir.mkdir(parents=True, exist_ok=True)

    stamp = "\n".join([(UI / "lv_conf.h").read_text(),
                       (LVGL / "lv_version.h").read_text(),
                       describe_toolchain(),
                       *(str(s.relative_to(LVGL)) for s in lvgl_sources)])
    stamp_file = lib_dir / "stamp.txt"
    fresh = stamp_file.is_file() and stamp_file.read_text() == stamp
    common = ["-DLV_CONF_INCLUDE_SIMPLE"]

    unix = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if unix:
        lib_flags = ["-std=c99", "-O2", "-w", *common, f"-I{UI}", f"-I{LVGL}"]
        if not fresh:
            _unix_objects(unix, lib_flags, lvgl_sources, lib_dir)
            stamp_file.write_text(stamp)
        lib = [lib_dir / f"{s.stem}.o" for s in lvgl_sources]
        font_objs = _unix_objects(unix, lib_flags, fonts, our_dir)
        command = [unix, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                   *common, f"-I{UI / 'include'}", f"-I{UI / 'src'}",
                   f"-I{UI}", "-isystem", str(LVGL),
                   *(str(s) for s in ours), *(str(o) for o in font_objs + lib),
                   "-lm", "-o", str(output)]
        environment = None
        label = f"unix:{Path(unix).name}"
    else:
        msvc = _msvc()
        if msvc is None:
            raise CompilerNotFound(
                "no host C compiler found (looked for cc, gcc, clang, then "
                "MSVC under Program Files)")
        cl, include, lib_paths = msvc
        environment = dict(os.environ)
        environment["INCLUDE"] = os.pathsep.join(str(p) for p in include)
        environment["LIB"] = os.pathsep.join(str(p) for p in lib_paths)

        def cl_objects(flags: list[str], sources: list[Path],
                       directory: Path) -> list[Path]:
            if not sources:
                return []
            rsp = directory / "sources.rsp"
            rsp.write_text("\n".join(f'"{s}"' for s in sources))
            result = subprocess.run(
                [str(cl), "-nologo", "-TC", "-c", "-MP", *flags, f"@{rsp}",
                 f"-Fo:{directory}{os.sep}"],
                capture_output=True, text=True, env=environment,
                cwd=str(HARNESS))
            if result.returncode != 0:
                raise RuntimeError(f"compiling into {directory.name} failed:\n"
                                   f"{result.stdout}\n{result.stderr}")
            return [directory / f"{s.stem}.obj" for s in sources]

        lib_flags = ["-O2", "-W1", *common, f"-I{UI}", f"-I{LVGL}"]
        if not fresh:
            cl_objects(lib_flags, lvgl_sources, lib_dir)
            stamp_file.write_text(stamp)
        lib = [lib_dir / f"{s.stem}.obj" for s in lvgl_sources]
        font_objs = cl_objects(lib_flags, fonts, our_dir)
        link = our_dir / "link.rsp"
        link.write_text("\n".join(f'"{p}"' for p in [*ours, *font_objs, *lib]))
        command = [str(cl), "-nologo", "-TC", "-O2", "-W4", "-WX",
                   "-D_CRT_SECURE_NO_WARNINGS", *common,
                   f"-I{UI / 'include'}", f"-I{UI / 'src'}", f"-I{UI}",
                   f"-external:I{LVGL}", "-external:W0",
                   f"@{link}", f"-Fe:{output}", f"-Fo:{our_dir}{os.sep}"]
        label = "msvc"

    result = subprocess.run(command, capture_output=True, text=True,
                            env=environment, cwd=str(HARNESS))
    if result.returncode != 0 or not output.is_file():
        raise RuntimeError(f"building the UI bench failed ({label}):\n"
                           f"{result.stdout}\n{result.stderr}")
    return BuildResult(executable=output, compiler=label)
```

- [ ] **Step 10: Write the Python driver**

Create `analysis/tools/uisnap.py`:
```python
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
```

- [ ] **Step 11: Run the tests**

Run: `cd analysis; uv run pytest tests/test_ui.py -q -rs`
Expected: all pass, **no skips**. The first run compiles LVGL (a few seconds); later runs reuse `obj-lvgl`. If the build fails at `-W4 -WX`, fix the warning in our code (see "Rules for every C file"), never by lowering the warning level.

- [ ] **Step 12: Run the whole suite**

Run: `cd analysis; uv run pytest -q`
Expected: everything passes, including `test_c_port.py` (the existing `build()` is untouched).

- [ ] **Step 13: Commit**

```bash
git add firmware/components/plumb_ui firmware/test/uisnap.c analysis/tools/cbuild.py analysis/tools/uisnap.py analysis/tests/test_ui.py
git commit -m "Build the headless UI bench: geometry, style, shell, renderer, tests"
```

---

### Task 4: Face angle screen

**Files:**
- Modify: `firmware/components/plumb_ui/src/screen_face.c` (replace the stub)
- Test: `analysis/tests/test_ui.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `analysis/tests/test_ui.py`:
```python
# -- face angle -----------------------------------------------------------------

def face_edge_angle(rgb):
    """The drawn face edge's rotation from vertical, degrees, clockwise on
    screen, measured from the amber pixels by their principal axis."""
    ys, xs = np.nonzero(near(rgb, ACCENT) & (ROWS < 170))
    points = np.column_stack([xs, ys]).astype(float)
    points -= points.mean(axis=0)
    _, vectors = np.linalg.eigh(points.T @ points)
    dx, dy = vectors[:, -1]
    if dy < 0:
        dx, dy = -dx, -dy
    # A vertical edge (0, 1) rotated clockwise by t becomes (-sin t, cos t).
    return float(np.degrees(np.arctan2(-dx, dy))), len(xs)


@pytest.mark.parametrize("face, drawn", [(1.8, -9.0), (-1.8, 9.0), (12.0, -45.0)])
def test_the_face_edge_is_drawn_at_five_times_the_angle(face, drawn):
    """Measured from pixels: open turns counter-clockwise, closed clockwise,
    and a large reading stops at the clamp."""
    angle, count = face_edge_angle(uisnap.render(Result(face=face), screen=0))
    assert count > 200
    assert angle == pytest.approx(drawn, abs=1.0)


def test_the_head_swings_from_square():
    start, _ = face_edge_angle(uisnap.render(Result(face=1.8), screen=0, t_ms=0))
    early, _ = face_edge_angle(uisnap.render(Result(face=1.8), screen=0, t_ms=100))
    assert start == pytest.approx(0.0, abs=1.0)
    assert -4.5 < early < -1.0, "part way toward open, ease-out"
```

- [ ] **Step 2: Run to see them fail**

Run: `cd analysis; uv run pytest tests/test_ui.py -q -k "face_edge or swings_from"`
Expected: FAIL. The stub draws no amber, so `count > 200` fails.

- [ ] **Step 3: Implement the screen**

Replace `firmware/components/plumb_ui/src/screen_face.c` with:
```c
/* Face angle: a blade from above, heel nearest the golfer, the face lit.
 * UI spec 4.1. Drawn rotation is 5x and clamped; the number is exact. */

#include <math.h>
#include <stdio.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define PIVOT_X 120.0f
#define PIVOT_Y 100.0f
#define SWING_MS 850u

static void draw_head(lv_layer_t *layer, float rotation_cw)
{
    /* Body, 17 px wide, then the 5 px face on its target side, then the
     * hosel stub at the heel. Each is a straight bar rotated about the pivot. */
    pl_ui_rotated_bar(layer, PL_UI_STEEL, 17, 120.5f, 52.0f, 120.5f, 148.0f,
                      PIVOT_X, PIVOT_Y, rotation_cw);
    pl_ui_rotated_bar(layer, PL_UI_ACCENT, 5, 109.5f, 52.0f, 109.5f, 148.0f,
                      PIVOT_X, PIVOT_Y, rotation_cw);
    pl_ui_rotated_bar(layer, PL_UI_STEEL, 7, 129.0f, 141.5f, 143.0f, 141.5f,
                      PIVOT_X, PIVOT_Y, rotation_cw);
}

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    char hero[24];
    const char *label;

    if (!result->face_valid) {
        pl_ui_no_reading(layer, "FACE ANGLE");
        return;
    }

    /* Where square sits, so the gap to the face is the error, unread. */
    pl_ui_line(layer, PL_UI_FAINT, 2, 109.0f, 44.0f, 109.0f, 156.0f, false, 4, 6);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "TOE",
               148, 42, PL_UI_ALIGN_LEFT, 2);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "HEEL",
               148, 163, PL_UI_ALIGN_LEFT, 2);
    pl_ui_target_mark(layer);

    draw_head(layer, pl_ui_ease_out((float)t_ms / (float)SWING_MS)
                         * pl_ui_face_rotation_cw_deg(result->face_angle_deg));

    snprintf(hero, sizeof(hero), "%.1f" PL_UI_DEG,
             (double)fabsf(result->face_angle_deg));
    switch (pl_ui_face_side_of(result->face_angle_deg)) {
    case PL_UI_FACE_OPEN:
        label = "OPEN " PL_UI_MIDDOT " REL. ADDRESS";
        break;
    case PL_UI_FACE_CLOSED:
        label = "CLOSED " PL_UI_MIDDOT " REL. ADDRESS";
        break;
    default:
        label = "SQUARE " PL_UI_MIDDOT " REL. ADDRESS";
        break;
    }
    pl_ui_text(layer, &pl_font_hero, PL_UI_FG, LV_OPA_COVER, hero,
               120, 196, PL_UI_ALIGN_CENTER, -1);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, label,
               120, 216, PL_UI_ALIGN_CENTER, 2);
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    return result->face_valid ? SWING_MS : 0u;
}

const pl_ui_screen pl_ui_screen_face = { draw, duration_ms };
```

- [ ] **Step 4: Run the tests**

Run: `cd analysis; uv run pytest tests/test_ui.py -q`
Expected: all pass, including the aperture and no-reading tests for screen 0.

- [ ] **Step 5: Look at it**

Run: `cd analysis; uv run python -m tools.uisnap`
Then open `firmware/test/ui-gallery/face.png`, `face-closed.png` and `face-square.png` and compare them with the screen studies. Note anything that looks wrong for Will's review in Task 8. Do not tune numbers to make a test pass.

- [ ] **Step 6: Commit**

```bash
git add firmware/components/plumb_ui/src/screen_face.c analysis/tests/test_ui.py
git commit -m "Draw the face angle screen: 5x rotation measured back from pixels"
```

---

### Task 5: Tempo screen

**Files:**
- Modify: `firmware/components/plumb_ui/src/screen_tempo.c` (replace the stub)
- Test: `analysis/tests/test_ui.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `analysis/tests/test_ui.py`:
```python
# -- tempo ----------------------------------------------------------------------

BACK_ROW, THRU_ROW = 80, 118
CAP = 15   # a round cap adds width/2 at each end of a 15 px bar


def bar(rgb, row):
    """(drawn length, centre x) of the amber bar on a row, or None."""
    cols = np.nonzero(near(rgb[row], ACCENT))[0]
    if len(cols) == 0:
        return None
    return cols.max() - cols.min() + 1 - CAP, (cols.max() + cols.min()) / 2


def test_bar_lengths_are_the_durations_at_one_scale():
    rgb = uisnap.render(Result(backswing_s=0.70, downswing_s=0.35), screen=1)
    (back, _), (thru, _) = bar(rgb, BACK_ROW), bar(rgb, THRU_ROW)
    assert back == pytest.approx(118 * 0.70, abs=2)
    assert thru == pytest.approx(118 * 0.35, abs=2)
    assert back / thru == pytest.approx(2.0, rel=0.06)


def test_the_backswing_goes_away_from_the_target():
    rgb = uisnap.render(Result(), screen=1)
    assert bar(rgb, BACK_ROW)[1] > 120, "backswing grows right, away from the hole"
    assert bar(rgb, THRU_ROW)[1] < 120, "through-stroke grows left, toward it"


def test_a_long_backswing_shrinks_both_bars_together():
    rgb = uisnap.render(Result(backswing_s=1.0, downswing_s=0.5), screen=1)
    assert bar(rgb, BACK_ROW)[0] == pytest.approx(90, abs=2)
    assert bar(rgb, THRU_ROW)[0] == pytest.approx(45, abs=2)


def test_the_backswing_draws_before_the_through_stroke():
    """Real time: 0.70 s out, then 0.34 s back. The clock resolves 10 ms."""
    mid_back = uisnap.render(Result(), screen=1, t_ms=350)
    assert bar(mid_back, BACK_ROW)[0] == pytest.approx(118 * 0.35, abs=6)
    assert bar(mid_back, THRU_ROW) is None
    mid_thru = uisnap.render(Result(), screen=1, t_ms=700 + 170)
    assert bar(mid_thru, BACK_ROW)[0] == pytest.approx(118 * 0.70, abs=2)
    assert bar(mid_thru, THRU_ROW)[0] == pytest.approx(118 * 0.17, abs=6)
```

- [ ] **Step 2: Run to see them fail**

Run: `cd analysis; uv run pytest tests/test_ui.py -q -k "bar_lengths or away_from or shrinks or draws_before"`
Expected: FAIL (`TypeError: cannot unpack non-iterable NoneType`: the stub draws no bars).

- [ ] **Step 3: Implement the screen**

Replace `firmware/components/plumb_ui/src/screen_tempo.c` with:
```c
/* Tempo: the backswing bar grows right, then the through bar left, both at
 * one speed, so length is duration. UI spec 4.2. No "ideal" marker. */

#include <math.h>
#include <stdio.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define FADE_MS 300u
#define BACK_Y 80.0f
#define THRU_Y 118.0f

static uint32_t ms(float seconds)
{
    return seconds > 0.0f ? (uint32_t)lroundf(seconds * 1000.0f) : 0u;
}

/* How far through [start, start + span] the clock is, 0..1. A span of 0 is a
 * step, not a division by zero. */
static float progress(uint32_t t_ms, uint32_t start, uint32_t span)
{
    if (t_ms <= start) {
        return 0.0f;
    }
    if (span == 0u) {
        return 1.0f;
    }
    return pl_ui_clamp01((float)(t_ms - start) / (float)span);
}

static void draw_ratio(lv_layer_t *layer, const pl_ui_result *result, lv_opa_t opa)
{
    char number[16];
    const char *suffix = " : 1";
    int32_t w_number, w_suffix, x0;

    snprintf(number, sizeof(number), "%.2f",
             (double)(result->backswing_s / result->downswing_s));
    w_number = pl_ui_text_width(&pl_font_hero, number, -1);
    w_suffix = pl_ui_text_width(&pl_font_word, suffix, 0);
    x0 = 120 - (w_number + w_suffix) / 2;
    pl_ui_text(layer, &pl_font_hero, PL_UI_FG, opa, number, x0, 184,
               PL_UI_ALIGN_LEFT, -1);
    pl_ui_text(layer, &pl_font_word, PL_UI_DIM, opa, suffix, x0 + w_number, 184,
               PL_UI_ALIGN_LEFT, 0);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, opa, "BACK : THROUGH",
               120, 208, PL_UI_ALIGN_CENTER, 2);
}

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    float back_px, thru_px, back, thru, fade;
    uint32_t back_ms, thru_ms;

    if (!result->tempo_valid || result->downswing_s <= 0.0f) {
        pl_ui_no_reading(layer, "TEMPO");
        return;
    }
    pl_ui_tempo_bars(result->backswing_s, result->downswing_s, &back_px, &thru_px);
    back_ms = ms(result->backswing_s);
    thru_ms = ms(result->downswing_s);

    pl_ui_line(layer, PL_UI_FAINT, 1, 120.0f, 44.0f, 120.0f, 128.0f, false, 2, 4);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "BALL",
               128, 42, PL_UI_ALIGN_LEFT, 2);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "BACK",
               128, 66, PL_UI_ALIGN_LEFT, 2);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "THROUGH",
               112, 104, PL_UI_ALIGN_RIGHT, 2);

    back = back_px * (back_ms == 0u ? 1.0f : pl_ui_clamp01((float)t_ms / (float)back_ms));
    if (back >= 1.0f) {
        pl_ui_line(layer, PL_UI_ACCENT, 15, 120.0f, BACK_Y, 120.0f + back, BACK_Y,
                   true, 0, 0);
    }
    thru = thru_px * progress(t_ms, back_ms, thru_ms);
    if (thru >= 1.0f) {
        pl_ui_line(layer, PL_UI_ACCENT, 15, 120.0f, THRU_Y, 120.0f - thru, THRU_Y,
                   true, 0, 0);
    }

    fade = progress(t_ms, back_ms + thru_ms, FADE_MS);
    if (fade > 0.0f) {
        draw_ratio(layer, result, (lv_opa_t)lroundf(fade * 255.0f));
    }
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    if (!result->tempo_valid || result->downswing_s <= 0.0f) {
        return 0u;
    }
    return ms(result->backswing_s) + ms(result->downswing_s) + FADE_MS;
}

const pl_ui_screen pl_ui_screen_tempo = { draw, duration_ms };
```

- [ ] **Step 4: Run the tests**

Run: `cd analysis; uv run pytest tests/test_ui.py -q`
Expected: all pass.

- [ ] **Step 5: Look at it**

Run: `cd analysis; uv run python -m tools.uisnap`, open `firmware/test/ui-gallery/tempo.png`, and note anything for Will's review. Check the ratio text fits inside the circle; the aperture test will fail if it does not.

- [ ] **Step 6: Commit**

```bash
git add firmware/components/plumb_ui/src/screen_tempo.c analysis/tests/test_ui.py
git commit -m "Draw the tempo screen: bar lengths measured back as durations"
```

---

### Task 6: Path screen

**Files:**
- Modify: `firmware/components/plumb_ui/src/screen_path.c` (replace the stub)
- Test: `analysis/tests/test_ui.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `analysis/tests/test_ui.py`:
```python
# -- path -----------------------------------------------------------------------

def trail(rgb):
    ys, xs = np.nonzero(near(rgb, ACCENT) & (ROWS < 200))
    return xs, ys


def test_out_to_in_starts_outside_and_finishes_inside():
    """Outside is up (away from the golfer). The stroke runs right to left.
    Sign and non-flatness only: the magnitude is pinned by
    test_path_ends_are_the_arc_amplified_and_scaled_by_travel."""
    xs, ys = trail(uisnap.render(Result(path_dir=OUT_TO_IN), screen=2))
    start, finish = ys[xs > 180].mean(), ys[xs < 60].mean()
    assert start < 122 < finish
    assert finish - start > 12, "the ends are about 2 x 12.16 px apart vertically"


def test_in_to_out_is_the_mirror():
    xs, ys = trail(uisnap.render(Result(path_dir=IN_TO_OUT), screen=2))
    assert ys[xs > 180].mean() > 122 > ys[xs < 60].mean()


def test_a_straight_path_is_flat():
    _, ys = trail(uisnap.render(Result(path_dir=STRAIGHT), screen=2))
    assert np.abs(ys - 122).max() <= 4


def test_the_head_travels_right_to_left():
    def dot_x(t):
        rgb = uisnap.render(Result(), screen=2, t_ms=t)
        _, xs = np.nonzero(near(rgb, FG) & (ROWS > 40) & (ROWS < 190))
        return xs.mean()
    assert dot_x(300) > dot_x(800)
```

- [ ] **Step 2: Run to see them fail**

Run: `cd analysis; uv run pytest tests/test_ui.py -q -k "outside_and or mirror or flat or right_to_left"`
Expected: FAIL (no amber trail yet: `mean of empty slice` / NaN comparisons).

- [ ] **Step 3: Implement the screen**

Replace `firmware/components/plumb_ui/src/screen_path.c` with:
```c
/* Path: the head retraces the stroke right to left across the target line.
 * UI spec 4.3. The shape is the message: arc is amplified 4x and scaled by
 * the stroke's travel, and no millimetre figure is ever shown.
 *
 * Drawn as a straight segment. The studies wrote it as a quadratic, but its
 * control point was the chord midpoint, which is a straight line; the result
 * carries a direction and one arc magnitude, not a curve. */

#include <math.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define TRACE_MS 1100u
#define TEXT_MS 350u

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    float x0, y0, x1, y1, u, x, y, text;
    const char *words;

    if (!result->path_valid) {
        pl_ui_no_reading(layer, "PATH");
        return;
    }

    pl_ui_line(layer, PL_UI_FAINT, 2, 26.0f, PL_UI_PATH_BASELINE_Y, 214.0f,
               PL_UI_PATH_BASELINE_Y, false, 4, 6);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER, "OUTSIDE",
               200, 64, PL_UI_ALIGN_RIGHT, 2);
    pl_ui_text(layer, &pl_font_marker, PL_UI_FAINT, LV_OPA_COVER,
               "INSIDE " PL_UI_MIDDOT " YOU", 200, 188, PL_UI_ALIGN_RIGHT, 2);
    pl_ui_target_mark(layer);

    pl_ui_path_ends(result->path_dir, result->path_arc_m, result->path_travel_m,
                    &x0, &y0, &x1, &y1);
    pl_ui_line(layer, PL_UI_FAINT, 2, x0, y0, x1, y1, false, 0, 0);

    u = pl_ui_ease_in_out((float)t_ms / (float)TRACE_MS);
    x = x0 + (x1 - x0) * u;
    y = y0 + (y1 - y0) * u;
    if (fabsf(x - x0) >= 1.0f) {
        pl_ui_line(layer, PL_UI_ACCENT, 5, x0, y0, x, y, true, 0, 0);
    }
    pl_ui_dot(layer, PL_UI_FG, x, y, 7);

    text = t_ms <= TRACE_MS ? 0.0f
         : pl_ui_clamp01((float)(t_ms - TRACE_MS) / (float)TEXT_MS);
    if (text > 0.0f) {
        words = result->path_dir == PL_PATH_OUT_TO_IN   ? "OUT TO IN"
              : result->path_dir == PL_PATH_IN_TO_OUT   ? "IN TO OUT"
                                                        : "STRAIGHT";
        pl_ui_text(layer, &pl_font_word, PL_UI_FG, (lv_opa_t)lroundf(text * 255.0f),
                   words, 120, 216, PL_UI_ALIGN_CENTER, 1);
    }
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    return result->path_valid ? TRACE_MS + TEXT_MS : 0u;
}

const pl_ui_screen pl_ui_screen_path = { draw, duration_ms };
```

- [ ] **Step 4: Run the tests**

Run: `cd analysis; uv run pytest tests/test_ui.py -q`
Expected: all pass.

- [ ] **Step 5: Correct the UI spec to match**

In `docs/superpowers/specs/2026-09-25-ui-screens-design.md` §4.3, replace "a head that retraces the stroke right to left along a quadratic curve" with "a head that retraces the stroke right to left along a straight segment (the studies' quadratic had its control point at the chord midpoint, which is a straight line)".

- [ ] **Step 6: Commit**

```bash
git add firmware/components/plumb_ui/src/screen_path.c analysis/tests/test_ui.py docs/superpowers/specs/2026-09-25-ui-screens-design.md
git commit -m "Draw the path screen: direction and amplified arc checked from pixels"
```

---

### Task 7: Impact speed screen

**Files:**
- Modify: `firmware/components/plumb_ui/src/screen_speed.c` (replace the stub)
- Test: `analysis/tests/test_ui.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `analysis/tests/test_ui.py`:
```python
# -- impact speed -----------------------------------------------------------------

def ring_angles(rgb):
    """Angles of the amber ring pixels, degrees, counter-clockwise from three
    o'clock (maths convention, y up)."""
    ys, xs = np.nonzero(near(rgb, ACCENT))
    r = np.hypot(xs - 119.5, ys - 119.5)
    keep = (r > 78) & (r < 96)
    return np.degrees(np.arctan2(-(ys[keep] - 119.5), xs[keep] - 119.5))


def test_the_ring_fills_anticlockwise_from_three_oclock():
    """0.75 m/s is a quarter of the 3.0 m/s full scale: 67.5 of 270 deg."""
    a = ring_angles(uisnap.render(Result(speed=0.75), screen=3))
    assert len(a) > 50
    assert a.min() > -6 and a.max() < 74


def test_above_full_scale_the_ring_is_full_and_stops():
    """270 deg from three o'clock ends at six o'clock: the lower-right quarter
    stays empty, whatever the speed."""
    a = ring_angles(uisnap.render(Result(speed=4.0), screen=3))
    assert not np.any((a > -84) & (a < -6))
    assert np.any(np.abs(a) > 170), "it reaches nine o'clock"


def test_the_ring_fills_over_time():
    early = len(ring_angles(uisnap.render(Result(speed=2.0), screen=3, t_ms=150)))
    final = len(ring_angles(uisnap.render(Result(speed=2.0), screen=3)))
    assert 0 < early < final
```

- [ ] **Step 2: Run to see them fail**

Run: `cd analysis; uv run pytest tests/test_ui.py -q -k "ring"`
Expected: FAIL (`len(a) > 50` fails: no ring yet).

- [ ] **Step 3: Implement the screen**

Replace `firmware/components/plumb_ui/src/screen_speed.c` with:
```c
/* Impact speed: clubhead speed at impact, as a ring filling anticlockwise
 * from three o'clock, toward the target. UI spec 4.4. Clubhead, not ball. */

#include <math.h>
#include <stdio.h>

#include "geometry.h"
#include "screens.h"
#include "style.h"

#define FILL_MS 800u
#define RING_RADIUS 88
#define RING_WIDTH 5

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    char hero[16];
    int32_t sweep;

    if (!result->speed_valid) {
        pl_ui_no_reading(layer, "IMPACT SPEED");
        return;
    }

    pl_ui_arc(layer, PL_UI_FAINT, RING_RADIUS, RING_WIDTH, 0, 360, false);
    sweep = (int32_t)lroundf(PL_UI_SPEED_MAX_SWEEP_DEG
                             * pl_ui_speed_fraction(result->impact_speed_mps)
                             * pl_ui_ease_out((float)t_ms / (float)FILL_MS));
    if (sweep >= 1) {
        /* LVGL angles run clockwise from three o'clock, so anticlockwise by
         * `sweep` is the span from 360 - sweep round to 360. */
        pl_ui_arc(layer, PL_UI_ACCENT, RING_RADIUS, RING_WIDTH, 360 - sweep, 360, true);
    }

    snprintf(hero, sizeof(hero), "%.2f", (double)result->impact_speed_mps);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "IMPACT SPEED",
               120, 92, PL_UI_ALIGN_CENTER, 2);
    pl_ui_text(layer, &pl_font_hero, PL_UI_FG, LV_OPA_COVER, hero,
               120, 158, PL_UI_ALIGN_CENTER, -1);
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "M / S",
               120, 184, PL_UI_ALIGN_CENTER, 2);
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    return result->speed_valid ? FILL_MS : 0u;
}

const pl_ui_screen pl_ui_screen_speed = { draw, duration_ms };
```

- [ ] **Step 4: Run the tests**

Run: `cd analysis; uv run pytest tests/test_ui.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add firmware/components/plumb_ui/src/screen_speed.c analysis/tests/test_ui.py
git commit -m "Draw the impact speed screen: ring direction and full scale from pixels"
```

---

### Task 8: Glyph sweep, gallery, docs, review

**Files:**
- Test: `analysis/tests/test_ui.py` (append)
- Modify: `analysis/README.md`, `HANDOFF.md`

- [ ] **Step 1: Write the glyph sweep and gallery tests**

Append to `analysis/tests/test_ui.py`:
```python
# -- every string, every screen ---------------------------------------------------

SWEEP = [
    Result(),
    Result(face=-0.03, backswing_s=1.4, downswing_s=0.6, path_dir=IN_TO_OUT, speed=3.4),
    Result(face=12.35, path_dir=STRAIGHT, speed=0.0),
    Result(face=None, backswing_s=None, path_dir=None, speed=None),
]


def test_every_string_on_every_screen_has_its_glyphs(tmp_path):
    """A glyph the font lacks renders as nothing at all. Render every screen
    for results that exercise every label and number format, at mid-motion and
    at rest, then ask the UI how many glyphs it could not find."""
    frame = tmp_path / "frame.rgb565"
    lines = ["idle", "advance 10", f"snap {frame}"]
    for result in SWEEP:
        lines.append(result.command())
        for _ in range(4):
            lines += ["advance 400", f"snap {frame}", "advance 6000",
                      f"snap {frame}", "next"]
    lines.append("missing")
    assert uisnap.run(lines) == ["0"]


def test_the_gallery_writes_a_png_per_case(tmp_path):
    written = uisnap.gallery(tmp_path)
    assert len(written) == 10
    for path in written:
        assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
```

- [ ] **Step 2: Run them**

Run: `cd analysis; uv run pytest tests/test_ui.py -q -k "glyphs or gallery"`
Expected: PASS. If the glyph test fails, a screen prints a character outside `fonts/README.md`'s set: fix the string, or add the glyph to `RANGES` in `tools/fonts.py` and regenerate. Never delete the test.

- [ ] **Step 3: Run the whole suite, verify no skips**

Run: `cd analysis; uv run pytest -q -rs`
Expected: everything passes; the skip summary does not list `test_ui.py` or `test_c_port.py`.

- [ ] **Step 4: Measure the font cost**

Run from the repo root:
```bash
ls -la firmware/test/obj-ui/pl_font_*.obj
```
Record the four sizes in `firmware/components/plumb_ui/fonts/README.md` under a new heading `## Measured size` (object-file sizes on MSVC x64, which include relocation data, so they are an upper bound on the flash cost). UI spec §8 asked for this to be measured, not assumed.

- [ ] **Step 5: Document the bench**

In `analysis/README.md`, add a section after the first code block:
```markdown
## The UI bench

The result screens (`firmware/components/plumb_ui/`) render on the PC with no
board, through `firmware/test/uisnap.c` and `tools/uisnap.py`:

```bash
uv run pytest tests/test_ui.py      # pixel-measured checks against the inputs
uv run python -m tools.uisnap       # PNGs of every screen -> firmware/test/ui-gallery/
```

Needs the LVGL submodule (`git submodule update --init`). The first run
compiles LVGL once; later runs reuse `firmware/test/obj-lvgl/`.
```

In `HANDOFF.md`, add a row to the "Done" table:
```markdown
| **UI screens, on the host** | `firmware/components/plumb_ui/` — face angle, tempo, path, impact speed and idle, pure LVGL 9.6, rendered headlessly and tested by measuring pixels against the inputs (5× face rotation, tempo bar ratio, path direction, ring sweep, aperture, glyph coverage). Not yet on the board: the display has never been brought up. |
```
and under "Next task", add as the first item:
```markdown
**0. Bring the display up and put the screens on it.** The screens are proven on the host
(`uv run python -m tools.uisnap` for PNGs). What remains is the GC9A01 driver, LVGL's tick and
flush on the device, touch-to-swipe (`pl_ui_next`/`pl_ui_prev`), fonts in PSRAM and draw buffers
in internal SRAM (invariant 7), and the app task that stops calling `lv_timer_handler()`
between arm and follow-through (invariant 8).
```

- [ ] **Step 6: Commit**

```bash
git add analysis/tests/test_ui.py analysis/README.md HANDOFF.md firmware/components/plumb_ui/fonts/README.md
git commit -m "Sweep every UI string for glyphs, add the gallery, document the bench"
```

- [ ] **Step 7: Review with Will**

Run `cd analysis; uv run python -m tools.uisnap` and show Will the ten PNGs in `firmware/test/ui-gallery/` beside the screen studies. Fidelity to the studies is judged on those images, with him (UI spec §8); record any changes he asks for as follow-up tasks rather than tuning numbers inside this plan.
