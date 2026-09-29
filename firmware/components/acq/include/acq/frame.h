/* imu_stream's binary frame, so analysis/tools/capture.py reads this firmware
 * unchanged.
 *
 *   A5 5A | first_seq u32 | count u8 | flags u8 | micros u32 | count x 6 x i16 | xor
 *
 * Little-endian. The XOR covers everything between the sync word and itself.
 * The layout's other implementations are bringup-arduino/imu_stream/framing.cpp
 * and capture.py; tests/test_acq.py decodes this one with capture.py.
 */
#ifndef ACQ_FRAME_H
#define ACQ_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "acq/record.h"

#define ACQ_FRAME_MAX_SAMPLES 16u
#define ACQ_FRAME_FLAG_OVERFLOW 0x01u   /* samples were dropped before this frame */
#define ACQ_FRAME_FLAG_SENSOR_SEQ 0x02u /* seq comes from the sensor's own counter */
#define ACQ_FRAME_BYTES(n) (2u + 10u + 12u * (n) + 1u)

/* Encode `count` records (1..ACQ_FRAME_MAX_SAMPLES) with consecutive sequence
 * numbers from first_seq. Returns the bytes written to out, which must hold
 * ACQ_FRAME_BYTES(count). */
size_t acq_frame_encode(uint8_t *out, uint32_t first_seq, uint8_t flags,
                        uint32_t micros, const acq_record *records,
                        uint8_t count);

#endif /* ACQ_FRAME_H */
