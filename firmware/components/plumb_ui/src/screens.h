/* A screen is a pure function of the result and the ms since it appeared. */
#ifndef PL_UI_SCREENS_H
#define PL_UI_SCREENS_H

#include <stdint.h>

#include "lvgl.h"
#include "plumb/ui.h"

typedef struct {
    void (*draw)(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms);
    /* How long its motion runs; 0 for none. */
    uint32_t (*duration_ms)(const pl_ui_result *result);
} pl_ui_screen;

extern const pl_ui_screen pl_ui_screen_face;
extern const pl_ui_screen pl_ui_screen_tempo;
extern const pl_ui_screen pl_ui_screen_path;
extern const pl_ui_screen pl_ui_screen_speed;

#endif /* PL_UI_SCREENS_H */
