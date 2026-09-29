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

## Measured size

2026-09-26, MSVC x64. Object sizes include relocation and symbol data, so
they are an upper bound on the flash cost; bitmap bytes are the glyph data
itself, counted from the generated arrays.

| Symbol | Object | Glyph bitmaps |
|---|---|---|
| `pl_font_hero` | 24,443 B | 15,098 B |
| `pl_font_word` | 7,131 B | 3,745 B |
| `pl_font_label` | 2,956 B | 697 B |
| `pl_font_marker` | 2,721 B | 534 B |
| **total** | **37,251 B** | **20,074 B** |

About 1.8% of the 2 MB PSRAM at the upper bound.
