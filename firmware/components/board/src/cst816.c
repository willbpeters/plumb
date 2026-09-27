/* See cst816.h. Registers from the CST816S register description (Hynitron),
 * as used by bringup-arduino/display/cst816s.cpp. */

#include "board/cst816.h"

#include "board/pins.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADDRESS 0x15
#define TIMEOUT_MS 10
#define REG_FINGER_NUM 0x02 /* then XH, XL, YH, YL */
#define REG_CHIP_ID 0xA7
#define REG_DIS_AUTO_SLEEP 0xFE

static i2c_master_dev_handle_t s_dev;

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, TIMEOUT_MS);
}

esp_err_t cst816_init(i2c_master_bus_handle_t bus, uint8_t *chip_id)
{
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADDRESS,
        .scl_speed_hz = BOARD_I2C_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &device, &s_dev);
    if (err != ESP_OK) {
        return err;
    }
    const gpio_config_t reset = {
        .pin_bit_mask = 1ULL << BOARD_PIN_TP_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&reset);
    gpio_set_level(BOARD_PIN_TP_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BOARD_PIN_TP_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(60)); /* answers for a while after reset */

    *chip_id = 0;
    err = read_regs(REG_CHIP_ID, chip_id, 1);
    /* Keep it awake so an idle read is "no finger", not a NACK. Best effort. */
    const uint8_t awake[2] = {REG_DIS_AUTO_SLEEP, 0x01};
    i2c_master_transmit(s_dev, awake, sizeof(awake), TIMEOUT_MS);
    return err;
}

bool cst816_read(uint16_t *x, uint16_t *y)
{
    uint8_t d[5];
    if (read_regs(REG_FINGER_NUM, d, sizeof(d)) != ESP_OK || d[0] == 0) {
        return false;
    }
    *x = (uint16_t)(((d[1] & 0x0F) << 8) | d[2]);
    *y = (uint16_t)(((d[3] & 0x0F) << 8) | d[4]);
    return true;
}
