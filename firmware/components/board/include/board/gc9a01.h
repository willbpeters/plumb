// GC9A01 round panel over ESP-IDF esp_lcd, for the display bring-up.
// Pins are the Waveshare ESP32-S3-Touch-LCD-1.28 Rev3 schematic's
// (docs/superpowers/specs/2026-09-26-display-bringup-design.md).
#ifndef PLUMB_GC9A01_H
#define PLUMB_GC9A01_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// MADCTL bits (GC9A01A datasheet, 36h).
#define GC9A01_MADCTL_MY 0x80
#define GC9A01_MADCTL_MX 0x40
#define GC9A01_MADCTL_MV 0x20
#define GC9A01_MADCTL_BGR 0x08

// Called from the SPI DMA completion interrupt when a draw has left the bus.
typedef void (*gc9a01_done_cb)(void *ctx);

// Bring the panel up: SPI bus, reset, the vendor init sequence, display on,
// backlight on. `max_transfer_bytes` is the largest single draw.
esp_err_t gc9a01_init(uint32_t spi_hz, uint8_t madctl, bool invert,
                      size_t max_transfer_bytes, gc9a01_done_cb done, void *ctx);

// Queue a draw of the inclusive rectangle (x1,y1)-(x2,y2). Pixels must be
// big-endian RGB565 in DMA-capable memory, and must stay valid until the
// done callback fires.
esp_err_t gc9a01_draw(int x1, int y1, int x2, int y2, const void *pixels);

esp_err_t gc9a01_set_madctl(uint8_t madctl);
esp_err_t gc9a01_set_invert(bool invert);

#ifdef __cplusplus
}
#endif

#endif // PLUMB_GC9A01_H
