/* Waveshare ESP32-S3-Touch-LCD-1.28, Rev3 schematic
 * (docs/bringup-results.md, "Pins"). The panel's SPI pins live with its
 * driver, src/gc9a01.c, which is carried over verbatim from the bring-up. */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#define BOARD_PIN_I2C_SDA 6 /* shared: IMU and touch (parent spec 4.2) */
#define BOARD_PIN_I2C_SCL 7
#define BOARD_PIN_IMU_INT1 4
#define BOARD_PIN_IMU_INT2 3 /* DRDY in SyncSample mode (datasheet rev A 6.1) */
#define BOARD_PIN_TP_INT 5
#define BOARD_PIN_TP_RST 13

/* The QMI8658 is a fast-mode (400 kHz) I2C part. */
#define BOARD_I2C_HZ 400000

#endif /* BOARD_PINS_H */
