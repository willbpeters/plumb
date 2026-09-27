/* See qmi8658.h. Register facts are from the QMI8658C datasheet rev A (QST
 * document 13-52-27) and were proven on this board by bringup-arduino/
 * imu_stream/qmi8658.cpp, whose comments carry the reasoning. */

#include "board/qmi8658.h"

#include "board/pins.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADDRESS 0x6B
#define TIMEOUT_MS 10

#define REG_WHO_AM_I 0x00
#define REG_CTRL1 0x02
#define REG_CTRL2 0x03
#define REG_CTRL3 0x04
#define REG_CTRL5 0x06
#define REG_CTRL7 0x08
#define REG_CTRL9 0x0A
#define REG_CAL1_L 0x0B
#define REG_FIFO_CTRL 0x14
#define REG_STATUSINT 0x2D
#define REG_TIMESTAMP_L 0x30
#define REG_GZ_H 0x40
#define REG_RESET_RESULT 0x4D
#define REG_RESET 0x60

#define WHO_AM_I_VALUE 0x05
#define RESET_COMMAND 0xB0
#define RESET_RESULT_OK 0x80
#define CTRL1_ADDR_AI 0x40
#define CTRL2_ACCEL_16G_896HZ ((0x03 << 4) | 0x03)
#define CTRL3_GYRO_256DPS_896HZ ((0x04 << 4) | 0x03)
#define CTRL5_STROKE_FILTERS 0x10 /* gyro LPF on, mode 00; accel LPF off */
#define CTRL7_SYNC_6DOF 0x83      /* SyncSample, DRDY_DIS=0, gEN, aEN (13.2.2) */
#define FIFO_CTRL_BYPASS 0x00     /* bypass: DRDY enabled (6.3) */
#define CTRL9_CMD_ACK 0x00
#define CTRL9_CMD_AHB_CLOCK_GATING 0x12
#define STATUSINT_CMD_DONE 0x80
#define CTRL9_POLLS 50

#define SYSTEM_TURN_ON_MS 15
#define GYRO_TURN_ON_MS 150
#define DATA_LOCK_DELAY_US 6 /* Table 40, ODR setting 3 */
#define AVAIL_POLLS 4
#define BLOCK_BYTES (REG_GZ_H - REG_TIMESTAMP_L + 1) /* 17 */
#define SAMPLE_OFFSET 5 /* past TIMESTAMP (3) and TEMP (2) */

static i2c_master_dev_handle_t s_dev;

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, n, TIMEOUT_MS);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), TIMEOUT_MS);
}

/* Write, then read back: a configuration write that did not land is found
 * here, not as a wrong number a week later. */
static esp_err_t write_verified(uint8_t reg, uint8_t value)
{
    esp_err_t err = write_reg(reg, value);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t back = 0;
    err = read_regs(reg, &back, 1);
    if (err != ESP_OK) {
        return err;
    }
    return back == value ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/* The CTRL9 protocol (5.9): write the command, wait for STATUSINT.CmdDone,
 * acknowledge. CTRL9 reads back whatever was written whether or not the
 * command ran, so CmdDone is the only confirmation there is. */
static esp_err_t ctrl9(uint8_t command)
{
    esp_err_t err = write_reg(REG_CTRL9, command);
    if (err != ESP_OK) {
        return err;
    }
    bool done = false;
    for (int i = 0; i < CTRL9_POLLS && !done; i++) {
        uint8_t status = 0;
        err = read_regs(REG_STATUSINT, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        done = (status & STATUSINT_CMD_DONE) != 0;
    }
    if (!done) {
        return ESP_ERR_TIMEOUT;
    }
    return write_reg(REG_CTRL9, CTRL9_CMD_ACK);
}

esp_err_t qmi8658_init(i2c_master_bus_handle_t bus)
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

    /* Soft reset, so every boot starts from the same state; the MCU resets
     * without power-cycling the sensor. 0x4D reads 0x80 after a good reset
     * and may be overwritten later, so it is checked now and never again. */
    err = write_reg(REG_RESET, RESET_COMMAND);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_TURN_ON_MS));
    uint8_t value = 0;
    if ((err = read_regs(REG_RESET_RESULT, &value, 1)) != ESP_OK) return err;
    if (value != RESET_RESULT_OK) return ESP_ERR_INVALID_STATE;
    if ((err = read_regs(REG_WHO_AM_I, &value, 1)) != ESP_OK) return err;
    if (value != WHO_AM_I_VALUE) return ESP_ERR_NOT_FOUND;

    /* ADDR_AI by read-modify-write: CTRL1.BE is left as reset sets it, the
     * setting the little-endian unpack was proven under. */
    uint8_t ctrl1 = 0;
    if ((err = read_regs(REG_CTRL1, &ctrl1, 1)) != ESP_OK) return err;
    if ((err = write_verified(REG_CTRL1, (uint8_t)(ctrl1 | CTRL1_ADDR_AI))) != ESP_OK) return err;

    if ((err = write_verified(REG_CTRL2, CTRL2_ACCEL_16G_896HZ)) != ESP_OK) return err;
    if ((err = write_verified(REG_CTRL3, CTRL3_GYRO_256DPS_896HZ)) != ESP_OK) return err;
    if ((err = write_verified(REG_CTRL5, CTRL5_STROKE_FILTERS)) != ESP_OK) return err;
    if ((err = write_verified(REG_FIFO_CTRL, FIFO_CTRL_BYPASS)) != ESP_OK) return err;

    /* 13.2.1: over I2C the internal AHB clock gating must be off for the lock
     * to be trustworthy. */
    if ((err = write_verified(REG_CAL1_L, 0x01)) != ESP_OK) return err;
    if ((err = ctrl9(CTRL9_CMD_AHB_CLOCK_GATING)) != ESP_OK) return err;

    if ((err = write_verified(REG_CTRL7, CTRL7_SYNC_6DOF)) != ESP_OK) return err;
    /* Gyro start-up: nothing sampled before this is believed. */
    vTaskDelay(pdMS_TO_TICKS(GYRO_TURN_ON_MS));
    return ESP_OK;
}

esp_err_t qmi8658_read_locked(qmi8658_sample *out)
{
    uint8_t status = 0;
    uint8_t polls = 0;
    esp_err_t err;
    while (polls < AVAIL_POLLS) {
        polls++;
        if ((err = read_regs(REG_STATUSINT, &status, 1)) != ESP_OK) {
            return err;
        }
        if (status & QMI8658_STATUSINT_AVAIL) {
            break;
        }
    }
    if (!(status & QMI8658_STATUSINT_AVAIL)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (!(status & QMI8658_STATUSINT_LOCKED)) {
        /* Locking is in progress and completes within Data_Lock_Delay. */
        esp_rom_delay_us(DATA_LOCK_DELAY_US);
    }

    uint8_t block[BLOCK_BYTES];
    if ((err = read_regs(REG_TIMESTAMP_L, block, sizeof(block))) != ESP_OK) {
        return err;
    }
    out->timestamp = (uint32_t)block[0] | ((uint32_t)block[1] << 8) |
                     ((uint32_t)block[2] << 16);
    const uint8_t *s = block + SAMPLE_OFFSET;
    for (int axis = 0; axis < 3; axis++) {
        out->accel[axis] = (int16_t)((uint16_t)s[2 * axis + 1] << 8 | s[2 * axis]);
        out->gyro[axis] = (int16_t)((uint16_t)s[6 + 2 * axis + 1] << 8 | s[6 + 2 * axis]);
    }
    out->statusint = status;
    out->polls = polls;
    return ESP_OK;
}
