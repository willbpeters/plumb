#include "uart_io.h"

#include <stdarg.h>
#include <stdio.h>

#include "driver/uart.h"

#define PORT UART_NUM_0 /* the CH343 bridge */
#define BAUD 921600     /* imu_stream's rate; capture.py's default */
#define RX_BUFFER 1024
#define TX_BUFFER 8192

esp_err_t uart_io_init(void)
{
    const uart_config_t config = {
        .baud_rate = BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(PORT, RX_BUFFER, TX_BUFFER, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    return uart_param_config(PORT, &config);
}

void uart_io_write(const void *data, size_t len)
{
    uart_write_bytes(PORT, data, len);
}

void uart_io_printf(const char *fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (n < 0) {
        return;
    }
    if ((size_t)n >= sizeof(line)) {
        n = sizeof(line) - 1;
    }
    uart_io_write(line, (size_t)n);
}

int uart_io_getc(void)
{
    uint8_t c;
    return uart_read_bytes(PORT, &c, 1, 0) == 1 ? c : -1;
}
