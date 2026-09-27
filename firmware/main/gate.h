/* Invariant 8: nothing renders between arm and follow-through.
 *
 * While the gate is armed the app task does not call lv_timer_handler(), so
 * LVGL neither renders nor reads touch, and the touch controller's I2C
 * traffic stops (parent spec 6.4). A console command arms it for now; the
 * ported stroke state machine will arm it later, through the same call. A
 * panel transfer already on the SPI bus when the gate arms runs to completion
 * by DMA; it touches neither core 0 nor the I2C bus. */
#ifndef GATE_H
#define GATE_H

#include <stdbool.h>

void gate_arm(void);
void gate_open(void);
bool gate_armed(void);

#endif /* GATE_H */
