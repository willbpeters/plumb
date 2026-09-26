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
