// See gc9a01.h.
//
// The vendor init table is Espressif's esp_lcd_gc9a01 default
// (github.com/espressif/esp-bsp, components/lcd/esp_lcd_gc9a01,
// Copyright 2021-2026 Espressif Systems, Apache-2.0). Most of it is
// undocumented vendor registers; it is carried over verbatim rather than
// "tidied", for the same reason the C port keeps the Python's arithmetic
// order -- a change nobody can explain is a defect nobody can find.

#include "board/gc9a01.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIN_DC 8
#define PIN_CS 9
#define PIN_CLK 10
#define PIN_MOSI 11
#define PIN_RST 14
#define PIN_BL 2
#define HOST SPI2_HOST

typedef struct {
    uint8_t cmd;
    uint8_t data[12];
    uint8_t len;
} init_cmd;

static const init_cmd VENDOR_INIT[] = {
    {0xfe, {0}, 0}, {0xef, {0}, 0},
    {0xeb, {0x14}, 1}, {0x84, {0x60}, 1}, {0x85, {0xff}, 1}, {0x86, {0xff}, 1},
    {0x87, {0xff}, 1}, {0x8e, {0xff}, 1}, {0x8f, {0xff}, 1}, {0x88, {0x0a}, 1},
    {0x89, {0x23}, 1}, {0x8a, {0x00}, 1}, {0x8b, {0x80}, 1}, {0x8c, {0x01}, 1},
    {0x8d, {0x03}, 1},
    {0x90, {0x08, 0x08, 0x08, 0x08}, 4},
    {0xff, {0x60, 0x01, 0x04}, 3},
    {0xc3, {0x13}, 1}, {0xc4, {0x13}, 1}, {0xc9, {0x30}, 1}, {0xbe, {0x11}, 1},
    {0xe1, {0x10, 0x0e}, 2},
    {0xdf, {0x21, 0x0c, 0x02}, 3},
    // gamma
    {0xf0, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6},
    {0xf1, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6},
    {0xf2, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2a}, 6},
    {0xf3, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6f}, 6},
    {0xed, {0x1b, 0x0b}, 2}, {0xae, {0x77}, 1}, {0xcd, {0x63}, 1},
    {0x70, {0x07, 0x07, 0x04, 0x0e, 0x0f, 0x09, 0x07, 0x08, 0x03}, 9},
    {0xe8, {0x34}, 1},  // 4 dot inversion
    {0x60, {0x38, 0x0b, 0x6d, 0x6d, 0x39, 0xf0, 0x6d, 0x6d}, 8},
    {0x61, {0x38, 0xf4, 0x6d, 0x6d, 0x38, 0xf7, 0x6d, 0x6d}, 8},
    {0x62, {0x38, 0x0d, 0x71, 0xed, 0x70, 0x70, 0x38, 0x0f, 0x71, 0xef, 0x70, 0x70}, 12},
    {0x63, {0x38, 0x11, 0x71, 0xf1, 0x70, 0x70, 0x38, 0x13, 0x71, 0xf3, 0x70, 0x70}, 12},
    {0x64, {0x28, 0x29, 0xf1, 0x01, 0xf1, 0x00, 0x07}, 7},
    {0x66, {0x3c, 0x00, 0xcd, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00}, 10},
    {0x67, {0x00, 0x3c, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98}, 10},
    {0x74, {0x10, 0x45, 0x80, 0x00, 0x00, 0x4e, 0x00}, 7},
    {0x98, {0x3e, 0x07}, 2}, {0x99, {0x3e, 0x07}, 2},
};

static esp_lcd_panel_io_handle_t s_io;
static gc9a01_done_cb s_done;
static void *s_ctx;

static bool on_color_done(esp_lcd_panel_io_handle_t io,
                          esp_lcd_panel_io_event_data_t *event, void *user)
{
    (void)io;
    (void)event;
    (void)user;
    if (s_done) {
        s_done(s_ctx);
    }
    return false;
}

static esp_err_t cmd(uint8_t c, const uint8_t *data, size_t len)
{
    return esp_lcd_panel_io_tx_param(s_io, c, data, len);
}

esp_err_t gc9a01_init(uint32_t spi_hz, uint8_t madctl, bool invert,
                      size_t max_transfer_bytes, gc9a01_done_cb done, void *ctx)
{
    esp_err_t err;
    size_t i;
    const uint8_t colmod = 0x55;  // 16 bits per pixel, RGB565

    s_done = done;
    s_ctx = ctx;

    spi_bus_config_t bus = {0};
    bus.sclk_io_num = PIN_CLK;
    bus.mosi_io_num = PIN_MOSI;
    bus.miso_io_num = -1;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = (int)max_transfer_bytes + 16;
    err = spi_bus_initialize(HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    esp_lcd_panel_io_spi_config_t io = {0};
    io.dc_gpio_num = PIN_DC;
    io.cs_gpio_num = PIN_CS;
    io.pclk_hz = spi_hz;
    io.lcd_cmd_bits = 8;
    io.lcd_param_bits = 8;
    io.spi_mode = 0;
    io.trans_queue_depth = 10;
    io.on_color_trans_done = on_color_done;
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)HOST, &io, &s_io);
    if (err != ESP_OK) return err;

    gpio_config_t out = {0};
    out.pin_bit_mask = (1ULL << PIN_RST) | (1ULL << PIN_BL);
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);
    gpio_set_level(PIN_BL, 0);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    cmd(0x11, NULL, 0);  // sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    cmd(0x36, &madctl, 1);
    cmd(0x3a, &colmod, 1);
    for (i = 0; i < sizeof(VENDOR_INIT) / sizeof(VENDOR_INIT[0]); i++) {
        cmd(VENDOR_INIT[i].cmd, VENDOR_INIT[i].data, VENDOR_INIT[i].len);
    }
    cmd(invert ? 0x21 : 0x20, NULL, 0);
    cmd(0x29, NULL, 0);  // display on
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_BL, 1);
    return ESP_OK;
}

esp_err_t gc9a01_draw(int x1, int y1, int x2, int y2, const void *pixels)
{
    const uint8_t caset[4] = {(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
    const uint8_t raset[4] = {(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
    const size_t bytes = (size_t)(x2 - x1 + 1) * (size_t)(y2 - y1 + 1) * 2u;
    cmd(0x2a, caset, 4);
    cmd(0x2b, raset, 4);
    return esp_lcd_panel_io_tx_color(s_io, 0x2c, pixels, bytes);
}

esp_err_t gc9a01_set_madctl(uint8_t madctl)
{
    return cmd(0x36, &madctl, 1);
}

esp_err_t gc9a01_set_invert(bool invert)
{
    return cmd(invert ? 0x21 : 0x20, NULL, 0);
}
