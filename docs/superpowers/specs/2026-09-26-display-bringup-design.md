# Display bring-up — design

**Date:** 2026-09-26
**Parent spec:** `2026-09-15-putting-analyzer-design.md` (§4.2, §6.2, §6.3, §13 Phase 0; invariants 7, 8)
**UI:** `2026-09-25-ui-screens-design.md` — the screens this puts on the panel
**Status:** approved with Will, 2026-09-26

## Goal

Get the round GC9A01 display and the CST816S touch controller working on the board, and show
the host-proven `plumb_ui` screens on them. This closes Phase 0's "display renders" item. It is
a bring-up instrument beside `imu_stream`, not product firmware; the ESP-IDF project remains
a separate, later piece of work.

**Done means:** with Will at the board, the test pattern shows correct colours and orientation,
the four screens match the host renders, a left or right swipe changes the screen, and the
render time, the working SPI clock and the pin map are recorded in `docs/bringup-results.md`.

## Pins (from the Waveshare ESP32-S3-Touch-LCD-1.28 Rev3 schematic)

| Net | GPIO | Net | GPIO |
|---|---|---|---|
| LCD_DC | 8 | I2C1 SDA (touch + IMU) | 6 |
| LCD_CS | 9 | I2C1 SCL (touch + IMU) | 7 |
| LCD_CLK | 10 | TP_INT | 5 |
| LCD_MOSI | 11 | TP_RST | 13 |
| LCD_MISO | 12 (unused) | IMU_INT1 / INT2 | 4 / 3 |
| LCD_RST | 14 | BAT_ADC | 1 |
| LCD_BL | 2, low-side MOSFET Q1, active high | | |

## Architecture

`firmware/bringup-arduino/display/`, built with `arduino-cli` like `imu_stream`:

- **`gc9a01.{h,c}`**: ESP-IDF `esp_lcd` SPI panel I/O with DMA on SPI2. The GC9A01 init
  sequence is Espressif's `esp_lcd_gc9a01` default vendor table (Apache-2.0, credited in the
  file). Orientation (`MADCTL`) and colour inversion can be changed at runtime, so the test
  pattern can settle them without a rebuild. The SPI clock is a compile-time constant, 80 MHz
  per parent §6.3, and 40 MHz if 80 shows artifacts.
- **`cst816s.{h,cpp}`**: touch coordinates over `Wire` at 0x15 on the shared bus. A NACK reads as
  "not touched", because the controller sleeps when idle.
- **`display.ino`**: LVGL glue and a serial console.
  - Two 240×60 RGB565 partial buffers from `MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA`
    (invariant 7).
  - `lv_draw_rgb565_swap` in the flush, because the panel wants big-endian and LVGL renders
    little-endian; this keeps `lv_conf.h` shared with the host bench.
  - Flush completion signalled from the DMA-done callback. LVGL's tick from `millis()`.
  - Everything in `loop()`, which is core 1 in this core (invariant 8's half that applies;
    there is no IMU here).
  - A pointer input device, and LVGL's `LV_EVENT_GESTURE` on the screen mapping left to
    `pl_ui_next` and right to `pl_ui_prev`.
- **`pl_*.c` wrappers**: one line each, `#include "src/ui.c"` and so on, resolved through `-I`
  on the component. The UI source is compiled from its one location in the repo; nothing is
  copied.
- **`analysis/tools/board_ui.py`**: the `arduino-cli` compile and upload, with LVGL passed as
  a library and the include paths passed as build properties, into a fixed build directory
  so rebuilds are incremental. Also `send`, which writes console commands and prints the reply.

## Console

`t` test pattern · `r` example result (face screen) · `n` / `p` next / previous · `i` idle ·
`v` toggle inversion · `g` toggle BGR · `x` / `y` toggle mirror X / Y · `s` toggle swap XY ·
`?` status. Every command that redraws prints `render+flush N us` for a full-screen refresh.
Touch prints `touch x y` on press and `gesture LEFT|RIGHT` when LVGL detects one.

## Verification (with Will at the board)

1. **Test pattern.** Red, green, blue and white quadrants, labelled, with TOP, BOTTOM, LEFT and
   RIGHT at the edges. Held with USB-C toward the golfer, does every label read correctly and
   sit where it claims? Wrong colours are fixed with `g` or `v`, mirroring with `x` / `y` / `s`,
   and the values that work become the defaults.
2. **Screens.** `r`, then `n` three times, compared against the host contact sheet.
3. **Touch.** A swipe left and a swipe right: the serial log shows the coordinates and the
   gesture, and the screen changes.
4. **Record.** Pins, `MADCTL`, inversion, SPI clock and render times go in
   `docs/bringup-results.md`.

The final rotation is not decided here. It depends on how the board sits in the grip, which
waits for a printed base.

## Also recorded: charger current

The same schematic shows an **ETA6098** charger (parent §4.2 and §4.4 say ETA6096), with its
ISET resistor R15 = 160 kΩ, which the schematic's own table maps to **1 A**. On the parent
spec's 400 mAh cell that is 2.5C, over §4.4's ≤ 1C limit. It goes in `bringup-results.md` and
`HANDOFF.md` before a cell is bought. The value is read from the schematic table, not the
chip datasheet, and the board's revision still needs confirming against its silkscreen.
