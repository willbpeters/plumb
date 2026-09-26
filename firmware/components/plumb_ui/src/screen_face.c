/* Face angle. Stub until Task 4. */
#include "screens.h"
#include "style.h"

static void draw(lv_layer_t *layer, const pl_ui_result *result, uint32_t t_ms)
{
    (void)result;
    (void)t_ms;
    pl_ui_text(layer, &pl_font_label, PL_UI_DIM, LV_OPA_COVER, "FACE ANGLE",
               120, 124, PL_UI_ALIGN_CENTER, 2);
}

static uint32_t duration_ms(const pl_ui_result *result)
{
    (void)result;
    return 0;
}

const pl_ui_screen pl_ui_screen_face = { draw, duration_ms };
