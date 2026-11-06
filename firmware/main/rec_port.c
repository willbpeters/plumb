/* The corpus log on the device (components/rec on the `corpus` partition).
 *
 * Runs on the app task, core 1, beside the stroke pipeline, which it does not
 * touch: recording changes nothing about what the pipeline sees or shows.
 *
 * Console: l starts or stops a session; e asks to erase the log and Y
 * confirms (any other key cancels); B toggles record-from-boot. A session
 * cannot start before the sample rate is measured, because the rate goes in
 * its header, exactly as the pipeline waits for it.
 *
 * ~5 KB of static internal RAM, almost all the 4 KB stage (rec/log.h).
 */

#include "rec_port.h"

#include <stdio.h>
#include <string.h>

#include "board/qmi8658.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "rec/log.h"
#include "stream.h"
#include "uart_io.h"

#define PARTITION_SUBTYPE 0x40
#define NVS_NAMESPACE "rec"
#define NVS_KEY_BOOT "from_boot"

static const esp_partition_t *s_part;
static rec_log s_log;
static bool s_ready;

static bool s_from_boot;      /* the NVS flag */
static bool s_boot_started;   /* this boot's automatic start has happened */
static bool s_start_pending;  /* l pressed before the rate existed */
static bool s_erase_asked;
static bool s_erasing;

static bool f_read(void *ctx, uint32_t off, void *dst, uint32_t len)
{
    return esp_partition_read(ctx, off, dst, len) == ESP_OK;
}

static bool f_write(void *ctx, uint32_t off, const void *src, uint32_t len)
{
    return esp_partition_write(ctx, off, src, len) == ESP_OK;
}

static bool f_erase(void *ctx, uint32_t off, uint32_t len)
{
    return esp_partition_erase_range(ctx, off, len) == ESP_OK;
}

static bool nvs_open_ok(nvs_handle_t *h, nvs_open_mode_t mode)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* The NVS partition is in a layout this IDF cannot use. It holds
         * nothing but this flag, so starting it over loses nothing else. */
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    return err == ESP_OK && nvs_open(NVS_NAMESPACE, mode, h) == ESP_OK;
}

void rec_port_init(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      (esp_partition_subtype_t)PARTITION_SUBTYPE,
                                      "corpus");
    if (s_part == NULL) {
        uart_io_printf("# rec: no corpus partition -- flash the partition table\n");
        return;
    }
    const rec_flash flash = {f_read, f_write, f_erase, (void *)s_part,
                             (uint32_t)s_part->size / REC_SECTOR_BYTES
                                 * REC_SECTOR_BYTES};
    const rec_status s = rec_log_open(&s_log, &flash);
    if (s != REC_OK) {
        uart_io_printf("# rec: open failed (%d)\n", (int)s);
        return;
    }
    s_ready = true;

    nvs_handle_t h;
    if (nvs_open_ok(&h, NVS_READONLY)) {
        uint8_t v = 0;
        if (nvs_get_u8(h, NVS_KEY_BOOT, &v) == ESP_OK) {
            s_from_boot = v != 0;
        }
        nvs_close(h);
    }
}

static void start(void)
{
    acq_rate_estimate rate;
    if (!stream_sample_rate(&rate)) {
        s_start_pending = true;
        uart_io_printf("# rec: starts when the sample rate is measured\n");
        return;
    }
    s_start_pending = false;

    rec_session_info info;
    memset(&info, 0, sizeof(info));
    info.start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    info.rate_hz = rate.hz;
    info.rate_se_hz = rate.hz_se;
    info.gyro_dps = QMI8658_GYRO_DPS;
    info.accel_g = QMI8658_ACCEL_G;
    info.putter_id = 0;   /* profiles do not exist yet */
    info.calibrated = 0;  /* the accelerometer calibration is not in NVS yet */
    for (int i = 0; i < 3; i++) {
        info.accel_gain[i] = 1.0f;
    }
    strncpy(info.firmware, esp_app_get_description()->version,
            REC_FIRMWARE_CHARS);

    const rec_status s = rec_log_start(&s_log, &info);
    if (s == REC_OK) {
        uart_io_printf("# rec: session %lu started at %lu of %lu bytes\n",
                       (unsigned long)s_log.sessions, (unsigned long)s_log.end,
                       (unsigned long)s_log.flash.size);
    } else {
        uart_io_printf("# rec: cannot start (%s)\n",
                       s == REC_ERR_FULL ? "log full -- dump, then e Y" : "error");
    }
}

static void report_stop(void)
{
    uart_io_printf("# rec: session %lu stopped%s: %lu samples, %lu lost, "
                   "%lu refused, %.2fx\n",
                   (unsigned long)s_log.sessions, s_log.full ? " (LOG FULL)" : "",
                   (unsigned long)s_log.samples, (unsigned long)s_log.lost,
                   (unsigned long)s_log.refused,
                   s_log.bytes ? 12.0 * s_log.samples / s_log.bytes : 0.0);
}

void rec_port_feed(uint32_t index, const acq_record *r)
{
    if (!s_ready) {
        return;
    }
    if (!s_log.recording && !s_erasing
        && (s_start_pending || (s_from_boot && !s_boot_started))) {
        acq_rate_estimate rate;
        if (stream_sample_rate(&rate)) {
            s_boot_started = true;
            start();
        }
    }
    const bool was = s_log.recording;
    rec_log_feed(&s_log, index, r->accel, r->gyro);
    if (was && !s_log.recording) {
        report_stop();
    }
}

void rec_port_service(void)
{
    if (!s_ready) {
        return;
    }
    if (s_erasing) {
        bool done = false;
        if (rec_log_erase_step(&s_log, &done) != REC_OK) {
            s_erasing = false;
            uart_io_printf("# rec: erase failed\n");
        } else if (done) {
            s_erasing = false;
            uart_io_printf("# rec: log erased\n");
        }
        return;
    }
    const bool was = s_log.recording;
    if (rec_log_service(&s_log) != REC_OK) {
        uart_io_printf("# rec: flash write failed, session stopped\n");
    }
    if (was && !s_log.recording) {
        report_stop();
    }
}

int rec_port_console(char c)
{
    if (s_erase_asked) {
        s_erase_asked = false;
        if (c == 'Y') {
            if (s_log.recording) {
                uart_io_printf("# rec: stop the session first\n");
            } else {
                s_erasing = true;
                uart_io_printf("# rec: erasing %lu bytes\n",
                               (unsigned long)s_log.flash.size);
            }
        } else {
            uart_io_printf("# rec: erase cancelled\n");
        }
        return 1;
    }
    switch (c) {
    case 'l':
        if (!s_ready || s_erasing) {
            uart_io_printf("# rec: not available\n");
        } else if (s_log.recording) {
            rec_log_stop(&s_log);
            report_stop();
        } else if (s_start_pending) {
            s_start_pending = false;
            uart_io_printf("# rec: start cancelled\n");
        } else {
            start();
        }
        return 1;
    case 'e':
        s_erase_asked = true;
        uart_io_printf("# rec: erase the whole log? Y to confirm, any key cancels\n");
        return 1;
    case 'B': {
        nvs_handle_t h;
        bool saved = false;
        if (nvs_open_ok(&h, NVS_READWRITE)) {
            saved = nvs_set_u8(h, NVS_KEY_BOOT, s_from_boot ? 0 : 1) == ESP_OK
                    && nvs_commit(h) == ESP_OK;
            nvs_close(h);
        }
        if (saved) {
            s_from_boot = !s_from_boot;
            uart_io_printf("# rec: record from boot %s\n", s_from_boot ? "ON" : "off");
        } else {
            uart_io_printf("# rec: could not save the flag\n");
        }
        return 1;
    }
    default:
        return 0;
    }
}

void rec_port_status(void)
{
    if (!s_ready) {
        uart_io_printf("# rec: not available\n");
        return;
    }
    /* Sessions are counted on the first start, which reads the log. */
    char sessions[16] = "?";
    if (s_log.sessions_counted) {
        snprintf(sessions, sizeof(sessions), "%lu", (unsigned long)s_log.sessions);
    }
    uart_io_printf("# rec: %s, %lu of %lu bytes used, %s sessions, "
                   "record from boot %s\n",
                   s_erasing ? "ERASING" : s_log.recording ? "RECORDING" : "idle",
                   (unsigned long)s_log.end, (unsigned long)s_log.flash.size,
                   sessions, s_from_boot ? "ON" : "off");
    if (s_log.recording || s_log.samples) {
        uart_io_printf("# rec: session %lu: %lu samples, %lu lost, %lu refused, "
                       "%.2fx\n",
                       (unsigned long)s_log.sessions, (unsigned long)s_log.samples,
                       (unsigned long)s_log.lost, (unsigned long)s_log.refused,
                       s_log.bytes ? 12.0 * s_log.samples / s_log.bytes : 0.0);
    }
}
