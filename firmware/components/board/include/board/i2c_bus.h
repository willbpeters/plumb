/* The one I2C bus, shared by the IMU and the touch controller.
 *
 * The master driver serialises transactions from different tasks with its own
 * bus lock. Only the acquisition task reads the IMU; touch is read by the app
 * task, and only while the invariant-8 gate is open, so under the gate the IMU
 * has the bus to itself. */
#ifndef BOARD_I2C_BUS_H
#define BOARD_I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

/* Create the bus. Its interrupt is allocated on the calling core: call it
 * from core 0, where acquisition runs. */
esp_err_t board_i2c_init(void);

i2c_master_bus_handle_t board_i2c_bus(void);

/* Bus-clear diagnostics from init: SDA before the clear (0: a device was
 * holding it, a read cut off by the last reset), the SCL clocks it took to
 * release it, and SDA after the clear and STOP. -1 before init. */
int board_i2c_sda_at_boot(void);
int board_i2c_clear_clocks(void);
int board_i2c_sda_after_clear(void);

#endif /* BOARD_I2C_BUS_H */
