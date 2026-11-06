/* CRC-32 as zlib computes it (reflected 0xEDB88320, init and final XOR
 * 0xFFFFFFFF), so Python's binascii.crc32 checks it. Pure C. */
#ifndef REC_CRC32_H
#define REC_CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t rec_crc32(const uint8_t *data, size_t len);

#endif /* REC_CRC32_H */
