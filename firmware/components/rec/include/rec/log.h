/* The corpus log: an append-only sequence of session and block records on a
 * raw flash partition (rec/format.h).
 *
 * WHY THE WRITES ARE SHAPED LIKE THIS. A flash write or erase disables the
 * cache on both cores of the ESP32-S3, and the IMU is read directly with no
 * FIFO behind it, so a stall longer than one sample period (~1.1 ms) loses
 * samples. So while recording this only ever PROGRAMS, one aligned 256-byte
 * page per rec_log_service() call, and erases only when told to, at rest.
 * Whether one page program fits inside a sample period is for the board to
 * say; if it does not, the sensor's own counter shows it as a gap in the log.
 *
 * Every loss, whatever its cause, is a gap in the sample index between two
 * blocks: nothing here counts loss any other way.
 *
 * Pure C, no ESP-IDF: flash is reached only through the callbacks, and the
 * host harness supplies a RAM flash that behaves like NOR.
 */
#ifndef REC_LOG_H
#define REC_LOG_H

#include <stdbool.h>
#include <stdint.h>

#include "rec/format.h"

/* Encoded bytes waiting for flash. Must hold a page plus the largest block,
 * and is sized so the log can fall a little behind without refusing. */
#define REC_STAGE_BYTES 4096u

typedef struct {
    bool (*read)(void *ctx, uint32_t offset, void *dst, uint32_t len);
    bool (*write)(void *ctx, uint32_t offset, const void *src, uint32_t len);
    /* offset and len are multiples of REC_SECTOR_BYTES */
    bool (*erase)(void *ctx, uint32_t offset, uint32_t len);
    void *ctx;
    uint32_t size;  /* a multiple of REC_SECTOR_BYTES */
} rec_flash;

/* What a session record carries besides its framing. */
typedef struct {
    uint32_t start_ms;
    double rate_hz;
    double rate_se_hz;
    uint16_t gyro_dps;
    uint16_t accel_g;
    uint8_t putter_id;
    uint8_t calibrated;  /* 1 if the offsets and gains below are real */
    float accel_offset_mps2[3];
    float accel_gain[3];
    char firmware[REC_FIRMWARE_CHARS];
} rec_session_info;

typedef enum {
    REC_OK = 0,
    REC_ERR_IO,         /* a flash callback failed */
    REC_ERR_FULL,       /* no room for another session */
    REC_ERR_STATE,      /* recording when it must not be, or the reverse */
} rec_status;

typedef struct {
    rec_flash flash;
    uint32_t end;           /* flash offset of stage[0]; page-aligned */
    uint32_t sessions;      /* in the log, counted on the first start */
    bool sessions_counted;
    bool recording;
    bool full;              /* the last session stopped because flash ran out */
    bool io_error;

    uint8_t stage[REC_STAGE_BYTES];
    uint32_t stage_len;

    int16_t block[REC_BLOCK_MAX][REC_AXES];
    uint8_t block_count;
    uint32_t block_first;
    uint32_t next_index;    /* the index the next sample should have */
    bool have_next;

    /* The current (or last) session. */
    uint32_t samples;       /* accepted into blocks */
    uint32_t lost;          /* index gaps between accepted samples */
    uint32_t refused;       /* dropped here because the stage was full */
    uint32_t bytes;         /* encoded, headers included */

    uint32_t erase_next;    /* rec_log_erase_step() progress */
} rec_log;

/* Find where the log ends. No flash is written. */
rec_status rec_log_open(rec_log *log, const rec_flash *flash);

/* Begin a session at the next page boundary. Counts the sessions already in
 * the log the first time it is called, which reads their record headers. */
rec_status rec_log_start(rec_log *log, const rec_session_info *info);

/* One sample at its unwrapped sensor index. Nothing happens unless
 * recording. Stops the session itself when flash runs out (log->full). */
void rec_log_feed(rec_log *log, uint32_t index, const int16_t accel[3],
                  const int16_t gyro[3]);

/* Write at most one full page. Call once per pass of the app loop. */
rec_status rec_log_service(rec_log *log);

/* Close the block, write everything staged, end on a page boundary. */
rec_status rec_log_stop(rec_log *log);

/* Erase the whole log one 64 KB step per call; true when done. Refused while
 * recording. */
rec_status rec_log_erase_step(rec_log *log, bool *done);

#endif /* REC_LOG_H */
