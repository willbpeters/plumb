#include "acq/acq.h"

#include <string.h>

#include "board/i2c_bus.h"
#include "board/pins.h"
#include "board/qmi8658.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Above esp_timer's task (22), below the IPC tasks (24). */
#define ACQ_PRIORITY (configMAX_PRIORITIES - 2)
#define ACQ_CORE 0
#define ACQ_STACK 4096
#define DRDY_TIMEOUT_MS 100

static acq_ring s_ring;
static volatile acq_counters s_counters;
static TaskHandle_t s_task;
static TaskHandle_t s_starter;
static esp_err_t s_init_result;
static volatile uint32_t s_edge_us;

static uint16_t clamp16(uint32_t us)
{
    return us > 0xFFFFu ? 0xFFFFu : (uint16_t)us;
}

static void drdy_isr(void *arg)
{
    (void)arg;
    s_edge_us = (uint32_t)esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    portYIELD_FROM_ISR(woken);
}

static esp_err_t start_drdy(void)
{
    const gpio_config_t pin = {
        .pin_bit_mask = 1ULL << BOARD_PIN_IMU_INT2,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&pin);
    if (err != ESP_OK) {
        return err;
    }
    /* Installed from this task, so the GPIO interrupt lives on core 0. */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    return gpio_isr_handler_add(BOARD_PIN_IMU_INT2, drdy_isr, NULL);
}

static void acq_task(void *arg)
{
    (void)arg;
    s_init_result = qmi8658_init(board_i2c_bus());
    if (s_init_result == ESP_OK) {
        s_init_result = start_drdy();
    }
    xTaskNotifyGive(s_starter);
    if (s_init_result != ESP_OK) {
        vTaskDelete(NULL);
    }

    for (;;) {
        const uint32_t edges = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DRDY_TIMEOUT_MS));
        if (edges == 0) {
            s_counters.drdy_timeouts++;
            continue;
        }
        const uint32_t wake_us = (uint32_t)esp_timer_get_time();
        const uint32_t edge_us = s_edge_us;
        qmi8658_sample sample;
        const esp_err_t err = qmi8658_read_locked(&sample);
        const uint32_t done_us = (uint32_t)esp_timer_get_time();
        if (edges > 1) {
            s_counters.missed_edges += edges - 1;
        }
        if (err == ESP_ERR_NOT_FOUND) {
            s_counters.not_available++;
            continue;
        }
        if (err != ESP_OK) {
            s_counters.read_errors++;
            continue;
        }
        if (!(sample.statusint & QMI8658_STATUSINT_LOCKED)) {
            s_counters.unlocked++;
        }

        acq_record record;
        memset(&record, 0, sizeof(record));
        record.timestamp = sample.timestamp;
        memcpy(record.accel, sample.accel, sizeof(record.accel));
        memcpy(record.gyro, sample.gyro, sizeof(record.gyro));
        record.edge_us = edge_us;
        record.done_us = done_us;
        record.wake_us = clamp16(wake_us - edge_us);
        record.status_us = clamp16(sample.status_done_us - edge_us);
        record.edges = (uint8_t)(edges > 255 ? 255 : edges);
        record.statusint = sample.statusint;
        record.polls = sample.polls;
        s_counters.reads++;
        acq_ring_push(&s_ring, &record);
    }
}

esp_err_t acq_start(void)
{
    acq_ring_init(&s_ring);
    s_starter = xTaskGetCurrentTaskHandle();
    if (xTaskCreatePinnedToCore(acq_task, "acq", ACQ_STACK, NULL, ACQ_PRIORITY,
                                &s_task, ACQ_CORE) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return s_init_result;
}

acq_ring *acq_ring_handle(void)
{
    return &s_ring;
}

void acq_counters_get(acq_counters *out)
{
    /* Each field is one aligned 32-bit word written by one task; a copy may
     * mix fields a sample apart, which is fine for a diagnostic. */
    out->reads = s_counters.reads;
    out->read_errors = s_counters.read_errors;
    out->not_available = s_counters.not_available;
    out->unlocked = s_counters.unlocked;
    out->missed_edges = s_counters.missed_edges;
    out->drdy_timeouts = s_counters.drdy_timeouts;
}
