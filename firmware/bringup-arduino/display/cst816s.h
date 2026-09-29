// CST816S touch controller on the shared I2C bus (GPIO6/7, with the IMU),
// reset on GPIO13. Coordinates only; gestures are LVGL's, not the chip's.
#ifndef PLUMB_CST816S_H
#define PLUMB_CST816S_H

#include <stdint.h>

// Reset the controller and try to keep it awake. The Wire bus must already be
// started. Returns true if the chip answered at 0x15.
bool cst816s_begin();

// One read. False means no finger -- including a NACK, because the chip
// sleeps when idle and stops answering until it is touched.
bool cst816s_read(uint16_t &x, uint16_t &y);

#endif  // PLUMB_CST816S_H
