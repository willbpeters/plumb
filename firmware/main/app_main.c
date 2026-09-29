/* Plumb product firmware: the skeleton
 * (docs/superpowers/specs/2026-09-27-firmware-skeleton-design.md).
 *
 * Core 0: the acquisition task, woken by the IMU's data-ready edge.
 * Core 1: this app task. It drains the ring, streams to the host, and runs
 * LVGL only while the invariant-8 gate is open.
 *
 * Console, single characters at 921600 (imu_stream's letters where they
 * overlap, so tools/capture.py works unchanged):
 *   s  start / stop streaming      b / c  binary / CSV
 *   a  arm the gate                o      open the gate
 *   x  toggle the UI exercise      j      jitter report, then a new window
 *   r  example result   n / p  next / previous screen      ?  status
 *   m  measure the IMU's sample rate again (also done at boot)
 *   1, d  acknowledged: the only rate and read path this firmware has
 *   9, f  refused: max rate and the FIFO path are not on this firmware
 */

#include "acq/acq.h"
#include "board/i2c_bus.h"
#include "board/qmi8658.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gate.h"
#include "stream.h"
#include "stroke.h"
#include "uart_io.h"
#include "ui_port.h"

#define APP_CORE 1
#define APP_PRIORITY 5
#define APP_STACK 16384

static esp_err_t s_imu_result = ESP_FAIL;
static esp_err_t s_ui_result = ESP_FAIL;

static void status(void)
{
    stream_status();
    stroke_status();
    uart_io_printf("# imu %s (init step %d, ctrl9 %ld us, SDA at boot %d, clear clocks %d, SDA after clear %d), ui %s\n",
                   esp_err_to_name(s_imu_result), qmi8658_init_step(),
                   (long)qmi8658_ctrl9_us(), board_i2c_sda_at_boot(),
                   board_i2c_clear_clocks(), board_i2c_sda_after_clear(),
                   esp_err_to_name(s_ui_result));
    if (s_ui_result == ESP_OK) {
        ui_port_status();
    }
}

static void console(char c)
{
    const bool ui = (s_ui_result == ESP_OK);
    switch (c) {
    case 's': stream_toggle(); break;
    case 'b': stream_set_binary(true); uart_io_printf("# format binary\n"); break;
    case 'c': stream_set_binary(false); uart_io_printf("# format csv\n"); break;
    case '1': uart_io_printf("# rate stroke\n"); break;
    case 'd': uart_io_printf("# path direct\n"); break;
    case '9': uart_io_printf("# rate max UNSUPPORTED on this firmware\n"); break;
    case 'f': uart_io_printf("# path fifo UNSUPPORTED on this firmware\n"); break;
    case 'a': gate_arm(); uart_io_printf("# gate armed\n"); break;
    case 'o': gate_open(); uart_io_printf("# gate open\n"); break;
    case 'j': stream_jitter_report(); break;
    case 'm': stream_measure_rate(); uart_io_printf("# measuring sample rate\n"); break;
    case '?': status(); break;
    case 'x':
        ui_port_render_reset();
        if (ui) {
            ui_port_set_exercise(!ui_port_exercise());
            uart_io_printf("# exercise %s\n", ui_port_exercise() ? "on" : "off");
        }
        break;
    case 'r':
        if (ui) {
            ui_port_example_result();
            uart_io_printf("# result shown\n");
        }
        break;
    case 'n': if (ui) ui_port_next(); break;
    case 'p': if (ui) ui_port_prev(); break;
    default: break;
    }
}

static void app_task(void *arg)
{
    (void)arg;
    s_ui_result = ui_port_init();
    if (s_ui_result != ESP_OK) {
        uart_io_printf("# UI init failed: %s\n", esp_err_to_name(s_ui_result));
    }
    stream_init();
    status();
    uart_io_printf("# ready: s b c a o x j m r n p ?\n");
    for (;;) {
        int c;
        while ((c = uart_io_getc()) >= 0) {
            console((char)c);
        }
        stream_service();
        if (s_ui_result == ESP_OK) {
            ui_port_service();
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    /* Nothing but this task's own output on the wire once it is up. */
    esp_log_level_set("*", ESP_LOG_WARN);
    ESP_ERROR_CHECK(uart_io_init());
    uart_io_printf("\n# plumb firmware skeleton\n");

    /* app_main runs on core 0: the bus interrupt and, inside acq_start, the
     * DRDY interrupt are allocated here, beside the acquisition task. */
    s_imu_result = board_i2c_init();
    if (s_imu_result == ESP_OK) {
        s_imu_result = acq_start();
    }
    if (s_imu_result != ESP_OK) {
        uart_io_printf("# IMU init failed: %s\n", esp_err_to_name(s_imu_result));
    }
    xTaskCreatePinnedToCore(app_task, "app", APP_STACK, NULL, APP_PRIORITY,
                            NULL, APP_CORE);
}
