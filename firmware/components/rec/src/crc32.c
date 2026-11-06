#include "rec/crc32.h"

/* Bitwise rather than a table: a block is a few hundred bytes every ~70 ms,
 * and 1 KB of table is internal SRAM the pipeline needs more. */
uint32_t rec_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}
