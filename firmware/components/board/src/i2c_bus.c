#include "board/i2c_bus.h"

#include "board/pins.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define HALF_PERIOD_US 5 /* 100 kHz: slow enough for anything on the bus */
#define MAX_CLOCKS 9

static i2c_master_bus_handle_t s_bus;
static int s_sda_at_boot = -1;
static int s_clear_clocks = -1;
static int s_sda_after_clear = -1;

/* The MCU can reset in the middle of an IMU read (the bus is busy about 45%
 * of the time at 906.86 Hz), and the IMU is not reset with it. It then holds
 * SDA low, part-way through a byte, waiting for clocks that never come.
 * Measured before this existed: every boot that found SDA low failed the IMU
 * init, and ESP-IDF's own bus clear (i2c_master_bus_reset) left SDA low.
 *
 * So the standard recovery is done by hand, before the driver owns the pins:
 * clock SCL until the device lets SDA go (at most one byte and its ACK slot),
 * then a STOP. */
static void clear_bus(void)
{
    const gpio_config_t pins = {
        .pin_bit_mask = (1ULL << BOARD_PIN_I2C_SDA) | (1ULL << BOARD_PIN_I2C_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&pins);
    gpio_set_level(BOARD_PIN_I2C_SDA, 1);
    gpio_set_level(BOARD_PIN_I2C_SCL, 1);
    esp_rom_delay_us(HALF_PERIOD_US);
    s_sda_at_boot = gpio_get_level(BOARD_PIN_I2C_SDA);

    /* All nine clocks, with SDA released, whatever SDA reads on the way.
     * Stopping at the first high SDA was measured to fail: a high can be a 1
     * data bit mid-byte, and the next SCL edge lets the device drive a 0
     * again (4 of 40 resets still failed that way). Nine clocks always reach
     * an ACK slot, where the released SDA is a NACK and the device ends its
     * read (NXP UM10204, 3.1.16). s_clear_clocks records when SDA first read
     * high, for the diagnostic only. */
    int released_at = s_sda_at_boot ? 0 : -1;
    for (int clock = 1; clock <= MAX_CLOCKS; clock++) {
        gpio_set_level(BOARD_PIN_I2C_SCL, 0);
        esp_rom_delay_us(HALF_PERIOD_US);
        gpio_set_level(BOARD_PIN_I2C_SCL, 1);
        esp_rom_delay_us(HALF_PERIOD_US);
        if (released_at < 0 && gpio_get_level(BOARD_PIN_I2C_SDA)) {
            released_at = clock;
        }
    }
    const int clocks = released_at;
    /* STOP: SDA low to high while SCL is high. */
    gpio_set_level(BOARD_PIN_I2C_SCL, 0);
    esp_rom_delay_us(HALF_PERIOD_US);
    gpio_set_level(BOARD_PIN_I2C_SDA, 0);
    esp_rom_delay_us(HALF_PERIOD_US);
    gpio_set_level(BOARD_PIN_I2C_SCL, 1);
    esp_rom_delay_us(HALF_PERIOD_US);
    gpio_set_level(BOARD_PIN_I2C_SDA, 1);
    esp_rom_delay_us(HALF_PERIOD_US);

    s_clear_clocks = clocks;
    s_sda_after_clear = gpio_get_level(BOARD_PIN_I2C_SDA);
    gpio_reset_pin(BOARD_PIN_I2C_SDA);
    gpio_reset_pin(BOARD_PIN_I2C_SCL);
}

esp_err_t board_i2c_init(void)
{
    clear_bus();
    const i2c_master_bus_config_t config = {
        .i2c_port = -1,
        .sda_io_num = BOARD_PIN_I2C_SDA,
        .scl_io_num = BOARD_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, &s_bus);
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return s_bus;
}

int board_i2c_sda_at_boot(void)
{
    return s_sda_at_boot;
}

int board_i2c_clear_clocks(void)
{
    return s_clear_clocks;
}

int board_i2c_sda_after_clear(void)
{
    return s_sda_after_clear;
}
