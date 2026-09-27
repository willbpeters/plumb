#include "ui_port.h"

#include "board/cst816.h"
#include "board/gc9a01.h"
#include "board/i2c_bus.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "gate.h"
#include "lvgl.h"
#include "plumb/ui.h"
#include "uart_io.h"

#define W 240
#define H 240
/* Parent spec 6.3: 2 x (240 x 60) RGB565, internal DMA-capable SRAM. */
#define BUF_LINES 60
#define BUF_BYTES (W * BUF_LINES * 2)
#define SPI_HZ (80u * 1000u * 1000u)
#define EXERCISE_MS 100

static lv_display_t *s_display;
static lv_timer_t *s_exercise;
static bool s_exercising;
static bool s_touch_ok;

static void flush_done(void *ctx)
{
    (void)ctx;
    /* From the SPI DMA interrupt: the buffer is LVGL's again. */
    lv_display_flush_ready(s_display);
}

static void flush(lv_display_t *display, const lv_area_t *area, uint8_t *pixels)
{
    (void)display;
    /* LVGL renders little-endian RGB565; the panel reads big-endian. Swapped
     * here, not in lv_conf.h, which the host bench shares. */
    lv_draw_rgb565_swap(pixels, lv_area_get_size(area));
    gc9a01_draw(area->x1, area->y1, area->x2, area->y2, pixels);
}

static uint32_t tick(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void touch_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    uint16_t x, y;
    /* lv_timer_handler() is not called under the gate, so this cannot run
     * then. If that ever changes, the bus still stays the IMU's. */
    if (!gate_armed() && s_touch_ok && cst816_read(&x, &y)) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void gesture(lv_event_t *event)
{
    (void)event;
    switch (lv_indev_get_gesture_dir(lv_indev_active())) {
    case LV_DIR_LEFT:
        pl_ui_next();
        uart_io_printf("# gesture left: screen %d\n", pl_ui_current_screen());
        break;
    case LV_DIR_RIGHT:
        pl_ui_prev();
        uart_io_printf("# gesture right: screen %d\n", pl_ui_current_screen());
        break;
    default:
        break;
    }
}

static void exercise_step(lv_timer_t *timer)
{
    (void)timer;
    pl_ui_next();
}

esp_err_t ui_port_init(void)
{
    uint8_t chip = 0;
    esp_err_t err = cst816_init(board_i2c_bus(), &chip);
    s_touch_ok = (err == ESP_OK);
    uart_io_printf("# touch: CST816 %s, chip id 0x%02X\n",
                   s_touch_ok ? "answered" : "DID NOT ANSWER", chip);

    /* Called on core 1, so the SPI interrupt is allocated here too. */
    err = gc9a01_init(SPI_HZ, GC9A01_MADCTL_MX | GC9A01_MADCTL_BGR, true,
                      BUF_BYTES, flush_done, NULL);
    if (err != ESP_OK) {
        return err;
    }

    lv_init();
    lv_tick_set_cb(tick);
    s_display = lv_display_create(W, H);
    /* Invariant 7: draw buffers in internal SRAM, never PSRAM. */
    void *buf1 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (buf1 == NULL || buf2 == NULL) {
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_buffers(s_display, buf1, buf2, BUF_BYTES,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(s_display, flush);

    lv_indev_t *touch = lv_indev_create();
    lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch, touch_read);

    pl_ui_init(s_display);
    lv_obj_add_event_cb(lv_screen_active(), gesture, LV_EVENT_GESTURE, NULL);

    s_exercise = lv_timer_create(exercise_step, EXERCISE_MS, NULL);
    lv_timer_pause(s_exercise);
    return ESP_OK;
}

void ui_port_service(void)
{
    if (!gate_armed()) {
        lv_timer_handler();
    }
}

void ui_port_example_result(void)
{
    /* The bring-up's example (display.ino), so the two can be compared. */
    pl_ui_result r = {0};
    r.face_valid = true;
    r.face_angle_deg = 1.8f;
    r.tempo_valid = true;
    r.backswing_s = 0.70f;
    r.downswing_s = 0.34f;
    r.path_valid = true;
    r.path_dir = PL_PATH_OUT_TO_IN;
    r.path_arc_m = 0.006f;
    r.path_travel_m = 0.30f;
    r.speed_valid = true;
    r.impact_speed_mps = 1.62f;
    pl_ui_show_result(&r);
}

void ui_port_next(void)
{
    pl_ui_next();
}

void ui_port_prev(void)
{
    pl_ui_prev();
}

void ui_port_set_exercise(bool on)
{
    s_exercising = on;
    if (on) {
        ui_port_example_result(); /* cycling needs result screens to cycle */
        lv_timer_resume(s_exercise);
    } else {
        lv_timer_pause(s_exercise);
    }
}

bool ui_port_exercise(void)
{
    return s_exercising;
}

void ui_port_status(void)
{
    uart_io_printf("# ui: screen %d, missing glyphs %lu, exercise %s, touch %s\n",
                   pl_ui_current_screen(), (unsigned long)pl_ui_missing_glyphs(),
                   s_exercising ? "on" : "off", s_touch_ok ? "ok" : "absent");
    uart_io_printf("# heap: internal free %u B, PSRAM free %u B\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
