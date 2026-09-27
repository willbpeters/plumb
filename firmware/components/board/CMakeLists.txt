# Board drivers on ESP-IDF: the shared I2C bus, the QMI8658 IMU, the CST816
# touch controller and the GC9A01 panel. Hardware only; no UI, no algorithm.
idf_component_register(
    SRCS "src/i2c_bus.c" "src/qmi8658.c" "src/cst816.c" "src/gc9a01.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_spi esp_lcd)
