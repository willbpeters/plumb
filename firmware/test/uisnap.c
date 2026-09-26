/* Headless renderer for plumb_ui: the UI's test bench.
 *
 * Reads commands on stdin, one per line, and answers on stdout -- the same
 * shape as portcheck.c. analysis/tools/uisnap.py drives it.
 *
 * LVGL's clock is advanced here, 1 ms at a time, and nothing reads a wall
 * clock, so every frame is a pure function of the commands before it.
 *
 *   result F face T back thru P dir arc travel S speed   show a result
 *   next | prev | idle                                   navigate
 *   advance MS                                           run the clock
 *   snap PATH                                            write RGB565 frame
 *   screen                                               print current index
 *   missing                                              print glyph misses
 *   probe TEXT                                           misses in hero font
 *   facerot DEG | faceside DEG | tempobars B T |
 *   pathends DIR ARC TRAVEL | speedfrac MPS              geometry
 */

#include <stdio.h>
#include <string.h>

#include "lvgl.h"
#include "plumb/ui.h"
#include "geometry.h"
#include "style.h"

#define W 240
#define H 240
#define LINE_MAX_CHARS 640

static uint16_t draw_buffer[W * H];
static uint16_t frame[W * H];

static void flush(lv_display_t *display, const lv_area_t *area, uint8_t *pixels)
{
    const uint16_t *src = (const uint16_t *)pixels;
    const int32_t width = lv_area_get_width(area);
    int32_t y;
    for (y = area->y1; y <= area->y2; y++) {
        memcpy(&frame[y * W + area->x1], &src[(y - area->y1) * width],
               (size_t)width * sizeof(uint16_t));
    }
    lv_display_flush_ready(display);
}

static void advance(unsigned ms)
{
    unsigned i;
    for (i = 0; i < ms; i++) {
        lv_tick_inc(1);
        lv_timer_handler();
    }
}

static int snap(const char *path)
{
    FILE *f;
    lv_obj_invalidate(lv_screen_active());
    lv_refr_now(NULL);
    f = fopen(path, "wb");
    if (f == NULL) {
        return 0;
    }
    fwrite(frame, sizeof(uint16_t), W * H, f);
    fclose(f);
    return 1;
}

int main(void)
{
    char line[LINE_MAX_CHARS];
    lv_display_t *display;

    lv_init();
    display = lv_display_create(W, H);
    lv_display_set_buffers(display, draw_buffer, NULL, sizeof(draw_buffer),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(display, flush);
    pl_ui_init(display);

    while (fgets(line, sizeof(line), stdin) != NULL) {
        char op[32];
        char text[LINE_MAX_CHARS];
        float a, b, c;
        int flag;

        if (sscanf(line, "%31s", op) != 1) {
            continue;
        }

        if (strcmp(op, "result") == 0) {
            pl_ui_result r;
            int fv, tv, pv, sv, dir;
            memset(&r, 0, sizeof(r));
            if (sscanf(line, "%31s %d %f %d %f %f %d %d %f %f %d %f", op,
                       &fv, &r.face_angle_deg, &tv, &r.backswing_s,
                       &r.downswing_s, &pv, &dir, &r.path_arc_m,
                       &r.path_travel_m, &sv, &r.impact_speed_mps) != 12) {
                fprintf(stderr, "bad result: %s", line);
                return 1;
            }
            r.face_valid = fv != 0;
            r.tempo_valid = tv != 0;
            r.path_valid = pv != 0;
            r.path_dir = (pl_path_dir)dir;
            r.speed_valid = sv != 0;
            pl_ui_show_result(&r);
        } else if (strcmp(op, "next") == 0) {
            pl_ui_next();
        } else if (strcmp(op, "prev") == 0) {
            pl_ui_prev();
        } else if (strcmp(op, "idle") == 0) {
            pl_ui_idle();
        } else if (strcmp(op, "advance") == 0) {
            unsigned ms;
            if (sscanf(line, "%31s %u", op, &ms) != 2) {
                fprintf(stderr, "bad advance: %s", line);
                return 1;
            }
            advance(ms);
        } else if (strcmp(op, "snap") == 0) {
            if (sscanf(line, "%31s %639[^\r\n]", op, text) != 2 || !snap(text)) {
                fprintf(stderr, "bad snap: %s", line);
                return 1;
            }
        } else if (strcmp(op, "screen") == 0) {
            printf("%d\n", pl_ui_current_screen());
        } else if (strcmp(op, "missing") == 0) {
            printf("%u\n", (unsigned)pl_ui_missing_glyphs());
        } else if (strcmp(op, "probe") == 0) {
            if (sscanf(line, "%31s %639[^\r\n]", op, text) != 2) {
                fprintf(stderr, "bad probe: %s", line);
                return 1;
            }
            printf("%u\n", (unsigned)pl_ui_count_missing(&pl_font_hero, text));
        } else if (strcmp(op, "facerot") == 0 && sscanf(line, "%31s %f", op, &a) == 2) {
            printf("%.6f\n", (double)pl_ui_face_rotation_cw_deg(a));
        } else if (strcmp(op, "faceside") == 0 && sscanf(line, "%31s %f", op, &a) == 2) {
            printf("%d\n", (int)pl_ui_face_side_of(a));
        } else if (strcmp(op, "tempobars") == 0
                   && sscanf(line, "%31s %f %f", op, &a, &b) == 3) {
            float back, thru;
            pl_ui_tempo_bars(a, b, &back, &thru);
            printf("%.6f %.6f\n", (double)back, (double)thru);
        } else if (strcmp(op, "pathends") == 0
                   && sscanf(line, "%31s %d %f %f", op, &flag, &a, &b) == 4) {
            float x0, y0, x1, y1;
            pl_ui_path_ends((pl_path_dir)flag, a, b, &x0, &y0, &x1, &y1);
            printf("%.6f %.6f %.6f %.6f\n", (double)x0, (double)y0,
                   (double)x1, (double)y1);
        } else if (strcmp(op, "speedfrac") == 0 && sscanf(line, "%31s %f", op, &c) == 2) {
            printf("%.6f\n", (double)pl_ui_speed_fraction(c));
        } else {
            fprintf(stderr, "unknown or malformed: %s", line);
            return 1;
        }
        fflush(stdout);
    }
    return 0;
}
