/* CST816 capacitive touch controller (chip id 0xB5 on this board, measured). */
#ifndef BOARD_CST816_H
#define BOARD_CST816_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

/* Reset, read the chip id, keep it awake. */
esp_err_t cst816_init(i2c_master_bus_handle_t bus, uint8_t *chip_id);

/* True with a position while a finger is down. */
bool cst816_read(uint16_t *x, uint16_t *y);

#endif /* BOARD_CST816_H */
