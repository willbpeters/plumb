/* The one serial path, text and binary alike, owned by the app task. One
 * writer means a status line can never land in the middle of a frame. */
#ifndef UART_IO_H
#define UART_IO_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t uart_io_init(void);
void uart_io_write(const void *data, size_t len);
void uart_io_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* The next received byte, or -1. Never blocks. */
int uart_io_getc(void);

#endif /* UART_IO_H */
