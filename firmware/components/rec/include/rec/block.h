/* One block record: up to REC_BLOCK_MAX consecutive samples, compressed
 * losslessly (rec/format.h, and the design document for why lossless).
 *
 * Per axis: the first value raw in the header, then each difference from the
 * previous sample, zigzag-mapped to unsigned and Rice-coded with the k that
 * makes that axis's block smallest. Stateless. Pure C.
 */
#ifndef REC_BLOCK_H
#define REC_BLOCK_H

#include <stddef.h>
#include <stdint.h>

#include "rec/format.h"

/* Encode `count` (1..REC_BLOCK_MAX) samples, each in axis order accel X Y Z,
 * gyro X Y Z, the first at sensor index first_index. `out` must hold
 * REC_BLOCK_MAX_BYTES. Returns the bytes written, CRC included. */
size_t rec_block_encode(uint8_t *out, uint32_t first_index,
                        const int16_t samples[][REC_AXES], uint8_t count);

#endif /* REC_BLOCK_H */
