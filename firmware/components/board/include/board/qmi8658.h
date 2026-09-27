/* QMI8658 IMU in SyncSample mode, paced by DRDY on INT2.
 *
 * Configuration is the stroke configuration proven by bringup-arduino/
 * imu_stream: 896.8 Hz nominal (906.86 Hz measured on this unit), +-16 g,
 * +-256 dps, gyro low-pass on, FIFO bypassed. What is new is the pacing:
 * the part's own data-ready line, and the lock that makes a slow I2C read
 * safe (datasheet rev A 6.1, 6.3, 13.2). */
#ifndef BOARD_QMI8658_H
#define BOARD_QMI8658_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define QMI8658_STATUSINT_AVAIL 0x01u
#define QMI8658_STATUSINT_LOCKED 0x02u

typedef struct {
    uint32_t timestamp; /* 24-bit sample counter */
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t statusint;  /* STATUSINT when Avail was first seen */
    uint8_t polls;      /* STATUSINT reads it took */
} qmi8658_sample;

/* Reset, verify and configure the part, leaving DRDY pulsing on INT2. */
esp_err_t qmi8658_init(i2c_master_bus_handle_t bus);

/* One locked read (13.2.3): STATUSINT starts the lock, the burst through GZ_H
 * releases it. ESP_ERR_NOT_FOUND if Avail never came up. */
esp_err_t qmi8658_read_locked(qmi8658_sample *out);

#endif /* BOARD_QMI8658_H */
