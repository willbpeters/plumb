# UI screens on the host — design

**Date:** 2026-09-25
**Parent spec:** `2026-09-15-putting-analyzer-design.md` (§6.1–6.3, §1.2.1, invariants 7 and 8)
**Visual design:** the reviewed screen studies, https://claude.ai/artifact/Lqtq3YX7w1LEzUQ3kwAeNa
**Status:** approved with Will, 2026-09-25

---

## 1. Goal

Build the on-device result screens as a pure-LVGL component, render them on the PC with no
board, and verify them against their inputs. The board comes next, onto screens that are
already proven. Nothing here touches hardware.

**Done means:** four screens render on the host from a results struct, their PNGs match the
reviewed studies, and automated tests check what is drawn against what was measured.

## 2. Scope

**In:** the `plumb_ui` component; four metric screens and an idle screen; swipe navigation as
API calls; Saira Condensed fonts converted for LVGL; LVGL as a pinned submodule; a headless
host renderer; tests.

**Out:** the board and its display and touch drivers; ESP-IDF; an SDL interactive simulator
(a possible follow-up, on the same component); units settings; battery and menus; the
distance screen.

The distance screen is out because distance is deferred in parent §1.2.1. A screen reading
"needs calibration" for a feature that does not exist is a stub, not a screen.

## 3. Architecture

`firmware/components/plumb_ui/`, beside `plumb/`. C99 plus LVGL, and **zero ESP-IDF or
hardware calls**, so the same source builds for the host and the device (`CLAUDE.md`, stack).

```
plumb_ui/
  include/plumb/ui.h        the whole public surface
  src/ui.c                  screen list, navigation, idle
  src/style.c  style.h      palette, fonts, shared label helpers
  src/geometry.c geometry.h pure maths, no LVGL
  src/screen_face.c
  src/screen_tempo.c
  src/screen_path.c
  src/screen_speed.c
  fonts/                    generated LVGL fonts + OFL.txt
```

### 3.1 The one input

```c
typedef enum { PL_PATH_STRAIGHT, PL_PATH_IN_TO_OUT, PL_PATH_OUT_TO_IN } pl_path_dir;

typedef struct {
    bool  face_valid;    float face_angle_deg;   /* + open, relative to address */
    bool  tempo_valid;   float backswing_s;      float downswing_s;
    bool  path_valid;    pl_path_dir path_dir;
                         float path_arc_m;       /* shapes the drawing, never shown */
                         float path_travel_m;    /* scales it */
    bool  speed_valid;   float impact_speed_mps;
} pl_ui_result;
```

The UI never computes a metric; it only draws what it is given. Tempo ratio is
`backswing_s / downswing_s`, formed for display only, because the bars are drawn from the
durations. A metric whose flag is false shows `— —` in place of its number and draws no
graphic. Impact speed is the case in point: the pipeline does not compute it yet.

`path_travel_m` is not in the Python `StrokeResult` today. The pipeline already computes it
(`distance` in `_compute`); exposing it is a one-line addition, made when the UI is wired to
real results.

### 3.2 API

```c
void pl_ui_init(lv_display_t *display);            /* builds screens, shows idle */
void pl_ui_show_result(const pl_ui_result *r);     /* face-angle screen, animations start */
void pl_ui_next(void);                             /* swipe left  */
void pl_ui_prev(void);                             /* swipe right */
void pl_ui_idle(void);                             /* wordmark only */
int  pl_ui_current_screen(void);                   /* for tests */
```

On the device, swipe gestures call `pl_ui_next` and `pl_ui_prev`; that wiring belongs to the
board bring-up, not here.

### 3.3 Invariant 8 stays in the app task

Nothing may render between arm and follow-through. That is enforced by the application task
not calling `lv_timer_handler()` in that window, not by the UI. The UI is passive: it creates
no timers of its own, and its animations start only in `pl_ui_show_result`, which the app
calls after DONE.

## 4. Screens

Palette and typography are the studies': amber `#E8B44A` on near-black `#0A0B0D`, foreground
`#F2EFE9`, dim `#6E747B`, faint `#2A2E34`, steel `#8E959D`. All drawing is in the golfer's
frame: **left is the target, bottom is the golfer**, and the stroke runs right to left.

Named constants are **display ranges, not measurement thresholds**, and invariant 5 does not
apply to them. Each is commented as such in the code.

### 4.1 Face angle

A blade from above, heel nearest the golfer, with the face as the lit amber edge. A dashed
line shows where square sits. The hero number is `|angle|` to one decimal with `°`, and the
label is `OPEN · REL. ADDRESS` or `CLOSED · REL. ADDRESS`.

- Drawn rotation = **5 × angle**, clamped at **±45°** on screen. The number is never clamped.
- Open turns the toe toward the target: **counter-clockwise on screen**.
- Below **0.05°** in magnitude the label reads `SQUARE · REL. ADDRESS`. That is the rounding
  limit of a one-decimal number, not a judgement (no verdicts, per the studies).
- Motion: the head swings from square to its angle in ~0.85 s, ease-out.

### 4.2 Tempo

A backswing bar grows **rightward** from the ball line, then the through bar grows
**leftward**, both at the same px/s, so bar length is duration. The hero reads `2.04 : 1`,
with the label `BACK : THROUGH`, and there is no "ideal" marker.

- Scale **118 px/s**, shared by both bars. If the backswing would overrun the aperture, both
  bars shrink by the same factor, so the ratio stays exact.
- Motion: each bar draws in real time (0.70 s then 0.34 s for the study's stroke), then the
  number fades in.

### 4.3 Path

Dashed target line, `OUTSIDE` above, `INSIDE · YOU` below, and a head that retraces the
stroke right to left along a straight segment (the studies wrote a quadratic, but its control
point was the chord midpoint, which is a straight line). The text reads `OUT TO IN`, `IN TO OUT` or
`STRAIGHT`, and **no arc figure is ever shown**.

- The curve spans a fixed 152 px horizontally, standing for `path_travel_m`.
- Lateral end offset in px = **4 × arc** × (152 px / travel), clamped at ±50 px. The sign
  follows the direction: out-to-in starts above (outside) and ends below; in-to-out the
  reverse; straight is flat.
- Motion: a trail and a dot traverse the curve in ~1.1 s, then the text fades in.

### 4.4 Impact speed

A ring around the aperture fills anticlockwise from the right, following the stroke toward the
target. The hero is the speed to two decimals, with the label `IMPACT SPEED` and the unit
`M / S`. It is clubhead speed, not ball speed.

- Ring full scale **3.0 m/s**, which puts typical putts at a third to two-thirds. Above it
  the ring is full and the number stays exact.
- Motion: the ring fills in ~0.8 s.

### 4.5 Idle and navigation

Before the first stroke, the `PLUMB` wordmark alone. After a stroke the last result stays
until the next one. Swipes move through face, tempo, path and speed, wrapping, with page dots
at the bottom edge. A new result always returns to face angle, the primary metric.

## 5. Fonts

Saira Condensed (Google Fonts, SIL Open Font License 1.1), converted with `lv_font_conv`
(npm) to LVGL bitmap fonts at four sizes: **64 px hero**, **30 px word** (path direction),
**13 px labels** and **11 px faint markers** (TOE, HEEL, TARGET, BALL, OUTSIDE). Each is subset
to exactly the glyphs its screens print: digits, uppercase
A–Z, `° : . · —` and space.

`analysis/tools/fonts.py` regenerates them. The generated `.c` files and `OFL.txt` are
committed, so a build needs neither npm nor the network. On the device the font data
belongs in PSRAM (invariant 7); on the host it does not matter.

## 6. LVGL and the host build

- LVGL is a **git submodule** at `firmware/third_party/lvgl`, pinned to the newest 9.x
  release tag (at least 9.2, per parent §6.1). Clone with `--recursive`.
- There is one `lv_conf.h`, at `firmware/components/plumb_ui/lv_conf.h`: RGB565, software
  renderer, no OS, and only the widgets used.
- `analysis/tools/cbuild.py` gains a UI target, which compiles LVGL, `plumb_ui` and a host
  program `firmware/test/uisnap.c` with the compiler it already discovers (MSVC here).
  LVGL's object files are cached, so only the first build is slow.
- `uisnap` creates a 240 × 240 display whose flush callback writes into memory, and advances
  LVGL's tick itself, so rendering is deterministic and never waits on a clock. It reads
  commands on stdin, like `portcheck`: a result, a screen, a time in ms. It writes the frame
  as raw RGB565, and Python converts that to PNG.

## 7. Tests

`analysis/tests/test_ui.py` drives `uisnap`. **The rule is the project's: check what is drawn
against what was measured, not against a stored copy of itself.**

1. **Geometry, no rendering:** 5× and its ±45° clamp; 4× arc with travel scaling and its
   ±50 px clamp; tempo bar lengths and the shared shrink; ring fraction and its full-scale
   cap. Each is checked against values derived by hand in the test.
2. **Face angle, measured from pixels:** fit a line to the amber face-edge pixels; its angle
   must equal 5 × the input within a pixel's worth. Open and closed must rotate in opposite
   directions, open counter-clockwise.
3. **Tempo, measured from pixels:** the two amber bars' lengths are in the input ratio, and
   the backswing bar lies right of the ball line.
4. **Path:** the traced curve's end offsets have the sign the direction requires, and a
   straight path is flat.
5. **Aperture:** nothing lit outside the 240 px circle on any screen.
6. **Glyphs:** every string any screen can print renders only glyphs present in its font. A
   missing glyph draws as a silent blank.
7. **Invalid metrics:** `— —` and no graphic.
8. **Navigation:** next and prev wrap, and a new result returns to face angle.
9. **Motion order:** mid-animation frames show the backswing bar before the through bar, and
   the path dot moving right to left.

PNGs of every screen at its final frame are written to a scratch directory on each run, as the
review artifact. They are not committed as golden images, for the reason in rule 1.

## 8. Risks

- **LVGL on MSVC.** LVGL supports it, but warning levels differ. LVGL is compiled with its own
  flags, and `-W4 -WX` applies to our code only.
- **Font size.** A 64 px bitmap font subset to ~50 glyphs is tens of KB. Measure it rather
  than assume, since PSRAM is 2 MB and shared.
- **The studies were SVG.** Anti-aliased thin dashed lines and small letter-spaced labels will
  not look identical in LVGL. Fidelity is judged on the PNGs, with Will.
