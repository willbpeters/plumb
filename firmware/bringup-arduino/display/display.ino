// Plumb display bring-up.
//
// Puts the host-proven plumb_ui screens on the round GC9A01 panel, with the
// CST816S touch controller driving swipes. A bring-up instrument, not product
// firmware (docs/superpowers/specs/2026-09-26-display-bringup-design.md).
//
// Console, single characters at 115200:
//   t  test pattern (colours and orientation)
//   r  show the example result (face screen)    n / p  next / previous
//   i  idle                                     m      measure a full refresh
//   v  toggle inversion   g  toggle BGR   x / y  toggle mirror X / Y
//   s  toggle swap XY     ?  status

#include <Wire.h>
#include <esp_heap_caps.h>
#include <lvgl.h>

#include "cst816s.h"
#include "gc9a01.h"

extern "C" {
#include "plumb/ui.h"
}

static const int W = 240;
static const int H = 240;
// Parent spec 6.3: 2 x (240 x 60) RGB565, internal DMA-capable SRAM.
static const int BUF_LINES = 60;
static const size_t BUF_BYTES = (size_t)W * BUF_LINES * 2;
// Parent spec 6.3 says 80 MHz. Drop to 40 MHz if the image shows artifacts.
static const uint32_t SPI_HZ = 80u * 1000u * 1000u;

static const int PIN_SDA = 6;  // shared with the IMU (spec 4.2)
static const int PIN_SCL = 7;

static lv_display_t *s_display;
static lv_obj_t *s_ui_screen;
static lv_obj_t *s_test_screen;
// Verified with Will at the board, 2026-09-26: MX + BGR with inversion on
// gives correct colours and unmirrored text, USB-C toward the viewer. BGR
// alone rendered every glyph mirrored left to right.
static uint8_t s_madctl = GC9A01_MADCTL_MX | GC9A01_MADCTL_BGR;
static bool s_invert = true;
static uint32_t s_last_refresh_us;

// -- display ----------------------------------------------------------------

static void flush_done(void *) {
  // From the SPI DMA interrupt: the buffer is free for LVGL again.
  lv_display_flush_ready(s_display);
}

static void flush(lv_display_t *, const lv_area_t *area, uint8_t *pixels) {
  // LVGL renders little-endian RGB565; the panel reads big-endian. Swapped
  // here rather than in lv_conf.h, which the host bench shares.
  lv_draw_rgb565_swap(pixels, lv_area_get_size(area));
  gc9a01_draw(area->x1, area->y1, area->x2, area->y2, pixels);
}

static uint32_t tick() { return millis(); }

// -- touch --------------------------------------------------------------------

static void touch_read(lv_indev_t *, lv_indev_data_t *data) {
  static bool pressed;
  static uint16_t last_x, last_y;
  uint16_t x, y;
  if (cst816s_read(x, y)) {
    data->point.x = x;
    data->point.y = y;
    data->state = LV_INDEV_STATE_PRESSED;
    if (!pressed) Serial.printf("touch %u %u\n", x, y);
    pressed = true;
    last_x = x;
    last_y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
    if (pressed) Serial.printf("release %u %u\n", last_x, last_y);
    pressed = false;
  }
}

static void gesture(lv_event_t *) {
  switch (lv_indev_get_gesture_dir(lv_indev_active())) {
    case LV_DIR_LEFT:
      Serial.println("gesture LEFT -> next");
      pl_ui_next();
      break;
    case LV_DIR_RIGHT:
      Serial.println("gesture RIGHT -> previous");
      pl_ui_prev();
      break;
    case LV_DIR_TOP:
      Serial.println("gesture UP");
      break;
    case LV_DIR_BOTTOM:
      Serial.println("gesture DOWN");
      break;
    default:
      break;
  }
}

// -- test pattern ---------------------------------------------------------------

static lv_obj_t *square(lv_obj_t *parent, uint32_t colour, int x, int y,
                        const char *text, uint32_t text_colour) {
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_remove_style_all(box);
  lv_obj_set_size(box, 72, 72);
  lv_obj_set_pos(box, x, y);
  lv_obj_set_style_bg_color(box, lv_color_hex(colour), 0);
  lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
  lv_obj_t *label = lv_label_create(box);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(text_colour), 0);
  lv_obj_center(label);
  return box;
}

static void edge(lv_obj_t *parent, const char *text, lv_align_t align, int dx, int dy) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label, align, dx, dy);
}

static lv_obj_t *build_test_pattern() {
  lv_obj_t *screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
  lv_obj_set_scrollable(screen, false);
  square(screen, 0xFF0000, 44, 44, "RED", 0xFFFFFF);
  square(screen, 0x00FF00, 124, 44, "GREEN", 0x000000);
  square(screen, 0x0000FF, 44, 124, "BLUE", 0xFFFFFF);
  square(screen, 0xFFFFFF, 124, 124, "WHITE", 0x000000);
  edge(screen, "TOP", LV_ALIGN_TOP_MID, 0, 16);
  edge(screen, "BOTTOM", LV_ALIGN_BOTTOM_MID, 0, -16);
  edge(screen, "LEFT", LV_ALIGN_LEFT_MID, 10, 0);
  edge(screen, "RIGHT", LV_ALIGN_RIGHT_MID, -10, 0);
  return screen;
}

// -- console --------------------------------------------------------------------

static void measure(const char *what) {
  const uint32_t t0 = micros();
  lv_obj_invalidate(lv_screen_active());
  lv_refr_now(s_display);
  s_last_refresh_us = micros() - t0;
  Serial.printf("%s: full refresh (render + flush) %lu us\n", what,
                (unsigned long)s_last_refresh_us);
}

static void show_ui() {
  if (lv_screen_active() != s_ui_screen) lv_screen_load(s_ui_screen);
}

static void apply_madctl() {
  gc9a01_set_madctl(s_madctl);
  Serial.printf("MADCTL 0x%02X (MY %d MX %d MV %d BGR %d)\n", s_madctl,
                !!(s_madctl & GC9A01_MADCTL_MY), !!(s_madctl & GC9A01_MADCTL_MX),
                !!(s_madctl & GC9A01_MADCTL_MV), !!(s_madctl & GC9A01_MADCTL_BGR));
  measure("redraw");
}

static void status() {
  Serial.printf("plumb display: MADCTL 0x%02X, invert %s, SPI %lu Hz, core %d\n",
                s_madctl, s_invert ? "on" : "off", (unsigned long)SPI_HZ,
                xPortGetCoreID());
  Serial.printf("  screen %d, last full refresh %lu us, missing glyphs %lu\n",
                pl_ui_current_screen(), (unsigned long)s_last_refresh_us,
                (unsigned long)pl_ui_missing_glyphs());
  Serial.printf("  internal heap free %u B, PSRAM free %u B\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

static void example_result() {
  pl_ui_result r = {};
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
  show_ui();
  pl_ui_show_result(&r);
  Serial.println("result shown: face screen");
}

static void console(char c) {
  switch (c) {
    case 't':
      lv_screen_load(s_test_screen);
      measure("test pattern");
      break;
    case 'r': example_result(); break;
    case 'n': show_ui(); pl_ui_next(); Serial.printf("screen %d\n", pl_ui_current_screen()); break;
    case 'p': show_ui(); pl_ui_prev(); Serial.printf("screen %d\n", pl_ui_current_screen()); break;
    case 'i': show_ui(); pl_ui_idle(); Serial.println("idle"); break;
    case 'm': measure("current screen"); break;
    case 'v':
      s_invert = !s_invert;
      gc9a01_set_invert(s_invert);
      Serial.printf("invert %s\n", s_invert ? "on" : "off");
      measure("redraw");
      break;
    case 'g': s_madctl ^= GC9A01_MADCTL_BGR; apply_madctl(); break;
    case 'x': s_madctl ^= GC9A01_MADCTL_MX; apply_madctl(); break;
    case 'y': s_madctl ^= GC9A01_MADCTL_MY; apply_madctl(); break;
    case 's': s_madctl ^= GC9A01_MADCTL_MV; apply_madctl(); break;
    case '?': status(); break;
    default: break;
  }
}

// -- setup / loop -----------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nplumb display bring-up");

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  cst816s_begin();

  esp_err_t err = gc9a01_init(SPI_HZ, s_madctl, s_invert, BUF_BYTES, flush_done, nullptr);
  Serial.printf("panel: gc9a01_init %s\n", err == ESP_OK ? "ok" : esp_err_to_name(err));

  lv_init();
  lv_tick_set_cb(tick);
  s_display = lv_display_create(W, H);
  // Invariant 7: draw buffers in internal SRAM, never PSRAM.
  void *buf1 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  void *buf2 = heap_caps_malloc(BUF_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  Serial.printf("buffers: 2 x %u B internal DMA %s\n", (unsigned)BUF_BYTES,
                (buf1 && buf2) ? "ok" : "ALLOCATION FAILED");
  lv_display_set_buffers(s_display, buf1, buf2, BUF_BYTES, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(s_display, flush);

  lv_indev_t *touch = lv_indev_create();
  lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch, touch_read);

  pl_ui_init(s_display);
  s_ui_screen = lv_screen_active();
  lv_obj_add_event_cb(s_ui_screen, gesture, LV_EVENT_GESTURE, nullptr);
  s_test_screen = build_test_pattern();

  measure("idle");
  status();
  Serial.println("ready: t r n p i m v g x y s ?");
}

void loop() {
  while (Serial.available()) console((char)Serial.read());
  lv_timer_handler();
  delay(2);
}
