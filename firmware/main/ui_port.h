/* LVGL, the GC9A01 panel and the CST816 touch controller, glued to plumb_ui.
 * Runs on the app task, core 1. Serviced only while the gate is open. */
#ifndef UI_PORT_H
#define UI_PORT_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t ui_port_init(void);
/* lv_timer_handler(), unless the gate is armed. */
void ui_port_service(void);

void ui_port_example_result(void);
void ui_port_next(void);
void ui_port_prev(void);
/* Cycle the result screens every 100 ms: the jitter test's UI load. */
void ui_port_set_exercise(bool on);
bool ui_port_exercise(void);
void ui_port_status(void);

#endif /* UI_PORT_H */
