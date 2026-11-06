/* The corpus log's on-flash format, version 1.
 *
 * docs/superpowers/specs/2026-09-29-stroke-logging-design.md is the reference;
 * analysis/plumb/corpus.py reads it, written from that document rather than
 * from this code, so the two are checked against each other
 * (analysis/tests/test_rec.py). Little-endian throughout.
 *
 * Pure C, no ESP-IDF: the host harness shares it.
 */
#ifndef REC_FORMAT_H
#define REC_FORMAT_H

#define REC_FORMAT_VERSION 1u

#define REC_PAGE_BYTES 256u     /* one flash page program */
#define REC_SECTOR_BYTES 4096u  /* the erase unit */

/* Session record: page-aligned, fixed length. */
#define REC_SESSION_MAGIC 0x534D4C50u  /* "PLMS" */
#define REC_SESSION_BYTES 100u
#define REC_FIRMWARE_CHARS 32u

/* Block record: up to REC_BLOCK_MAX consecutive samples. */
#define REC_BLOCK_MAGIC 0x4B42u        /* "BK" */
#define REC_BLOCK_MAX 64u
#define REC_BLOCK_HEADER_BYTES 28u
#define REC_AXES 6u                    /* accel X Y Z, gyro X Y Z */

/* Rice coding of each zigzagged difference. A quotient of REC_RICE_ESCAPE or
 * more is written as that many one-bits and then the value in
 * REC_RICE_RAW_BITS: a difference can span full scale to full scale, whose
 * zigzag (131,070) is far too long for unary. */
#define REC_RICE_K_MAX 16u
#define REC_RICE_ESCAPE 24u
#define REC_RICE_RAW_BITS 17u

/* The largest block, all escapes: header, payload, CRC. */
#define REC_BLOCK_MAX_BYTES                                                   \
    (REC_BLOCK_HEADER_BYTES                                                   \
     + (REC_AXES * (REC_BLOCK_MAX - 1u) * (REC_RICE_ESCAPE + REC_RICE_RAW_BITS) \
        + 7u) / 8u                                                            \
     + 4u)

#endif /* REC_FORMAT_H */
