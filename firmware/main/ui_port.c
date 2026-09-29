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

/* How long lv_timer_handler() takes, so rendering speed is a measurement --
 * the trade made when LVGL's code left IRAM for the stroke pipeline's memory
 * (spec 6.3, amended 2026-09-28). A call over RENDER_MIN_US drew something;
 * the rest only checked timers. Reset by ui_port_render_reset(). */
#define RENDER_MIN_US 2000
static uint32_t s_render_max_us;
static uint64_t s_render_total_us;
static uint32_t s_renders;

void ui_port_service(void)
{
    if (!gate_armed()) {
        const int64_t t0 = esp_timer_get_time();
        lv_timer_handler();
        const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
        if (us > s_render_max_us) {
            s_render_max_us = us;
        }
        if (us > RENDER_MIN_US) {
            s_render_total_us += us;
            s_renders++;
        }
    }
}

void ui_port_render_reset(void)
{
    s_render_max_us = 0;
    s_render_total_us = 0;
    s_renders = 0;
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

/* The pipeline's face angle is the twist about measured gravity, and at rest
 * the accelerometer reads specific force UPWARD, so positive is counter-
 * clockwise seen from above -- whatever way the board sits in the grip, since
 * it is a rotation about a world axis. plumb_ui draws + as OPEN. For a
 * right-handed golfer in the UI's frame (target left, golfer at the bottom),
 * open points right of the target, away from the golfer: CLOCKWISE from
 * above, negative. Hence -1.
 *
 * DERIVED, NOT MEASURED (2026-09-28). No test has checked it: open the face at
 * address on the bench, make a stroke, read the label. A left-handed golfer
 * flips it, and nothing on the device knows handedness yet. */
#define FACE_OPEN_SIGN (-1.0f)

void ui_port_show_stroke(const pl_stroke_result *s)
{
    pl_ui_result r = {0};
    r.face_valid = s->face_valid != 0;
    r.face_angle_deg = FACE_OPEN_SIGN * (float)s->face_angle_deg;
    r.tempo_valid = true;
    r.backswing_s = (float)s->backswing_s;
    r.downswing_s = (float)s->downswing_s;
    r.path_valid = s->path_valid != 0;
    /* pl_direction's values are pl_path_dir's, by construction (pipeline.h). */
    r.path_dir = (pl_path_dir)s->path_direction;
    r.path_arc_m = (float)s->path_arc_m;
    r.path_travel_m = (float)s->path_travel_m;
    r.speed_valid = s->speed_valid != 0;
    r.impact_speed_mps = (float)s->impact_speed_mps;
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
    uart_io_printf("# render: %lu draws, mean %.2f ms, max %.2f ms (lv_timer_handler)\n",
                   (unsigned long)s_renders,
                   s_renders ? (double)s_render_total_us / s_renders / 1000.0 : 0.0,
                   s_render_max_us / 1000.0);
    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    uart_io_printf("# lvgl heap: %u B total, %u B in use now, %u B at most (%u%%)\n",
                   (unsigned)mon.total_size, (unsigned)(mon.total_size - mon.free_size),
                   (unsigned)mon.max_used, (unsigned)mon.used_pct);
    uart_io_printf("# heap: internal free %u B (largest block %u B), PSRAM free %u B\n",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
