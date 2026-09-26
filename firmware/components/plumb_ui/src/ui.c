/* See plumb/ui.h. One full-screen object draws whichever screen is current,
 * so every pixel on the display comes from a draw callback we wrote. */

#include "plumb/ui.h"

#include "screens.h"
#include "style.h"

static const pl_ui_screen *const SCREENS[PL_UI_SCREEN_COUNT] = {
    &pl_ui_screen_face,
    &pl_ui_screen_tempo,
    &pl_ui_screen_path,
    &pl_ui_screen_speed,
};

static lv_obj_t *s_canvas;
static pl_ui_result s_result;
static bool s_has_result;
static int s_screen = PL_UI_IDLE;
static int32_t s_t_ms;

static void set_t(void *var, int32_t value)
{
    (void)var;
    s_t_ms = value;
    lv_obj_invalidate(s_canvas);
}

/* The motion clock. An lv_anim, which runs inside lv_timer_handler() and
 * therefore never during a stroke (invariant 8 is the app's to enforce). */
static void start_motion(void)
{
    lv_anim_delete(&s_t_ms, set_t);
    s_t_ms = 0;
    if (s_screen != PL_UI_IDLE) {
        const uint32_t duration = SCREENS[s_screen]->duration_ms(&s_result);
        if (duration > 0) {
            lv_anim_t a;
            lv_anim_init(&a);
            lv_anim_set_var(&a, &s_t_ms);
            lv_anim_set_exec_cb(&a, set_t);
            lv_anim_set_values(&a, 0, (int32_t)duration);
            lv_anim_set_duration(&a, duration);
            lv_anim_start(&a);
        }
    }
    lv_obj_invalidate(s_canvas);
}

static void draw_dots(lv_layer_t *layer)
{
    int i;
    for (i = 0; i < PL_UI_SCREEN_COUNT; i++) {
        pl_ui_dot(layer, i == s_screen ? PL_UI_ACCENT : PL_UI_FAINT,
                  (float)(102 + 12 * i), 232.0f, 3);
    }
}

static void draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    if (s_screen == PL_UI_IDLE) {
        pl_ui_text(layer, &pl_font_word, PL_UI_FG, LV_OPA_COVER, "PLUMB",
                   120, 131, PL_UI_ALIGN_CENTER, 6);
        return;
    }
    SCREENS[s_screen]->draw(layer, &s_result, (uint32_t)s_t_ms);
    draw_dots(layer);
}

void pl_ui_init(lv_display_t *display)
{
    lv_obj_t *screen = lv_display_get_screen_active(display);
    lv_obj_set_style_bg_color(screen, PL_UI_BG, 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_scrollable(screen, false);

    s_canvas = lv_obj_create(screen);
    lv_obj_remove_style_all(s_canvas);
    lv_obj_set_pos(s_canvas, 0, 0);
    lv_obj_set_size(s_canvas, 240, 240);
    lv_obj_add_event_cb(s_canvas, draw_cb, LV_EVENT_DRAW_MAIN, NULL);

    s_has_result = false;
    s_screen = PL_UI_IDLE;
    start_motion();
}

void pl_ui_show_result(const pl_ui_result *result)
{
    s_result = *result;
    s_has_result = true;
    s_screen = PL_UI_SCREEN_FACE;   /* the primary metric, after every stroke */
    start_motion();
}

void pl_ui_next(void)
{
    if (!s_has_result) {
        return;
    }
    s_screen = s_screen == PL_UI_IDLE ? PL_UI_SCREEN_FACE
                                      : (s_screen + 1) % PL_UI_SCREEN_COUNT;
    start_motion();
}

void pl_ui_prev(void)
{
    if (!s_has_result) {
        return;
    }
    s_screen = s_screen == PL_UI_IDLE
                   ? PL_UI_SCREEN_FACE
                   : (s_screen + PL_UI_SCREEN_COUNT - 1) % PL_UI_SCREEN_COUNT;
    start_motion();
}

void pl_ui_idle(void)
{
    s_screen = PL_UI_IDLE;
    start_motion();
}

int pl_ui_current_screen(void)
{
    return s_screen;
}
