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
