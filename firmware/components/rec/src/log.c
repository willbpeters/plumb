#include "rec/log.h"

#include <string.h>

#include "rec/block.h"
#include "rec/crc32.h"

#define ERASE_STEP_BYTES (64u * 1024u)

static uint32_t round_up_page(uint32_t n)
{
    return (n + REC_PAGE_BYTES - 1u) / REC_PAGE_BYTES * REC_PAGE_BYTES;
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v);
    put16(p + 2, (uint16_t)(v >> 16));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16
           | (uint32_t)p[3] << 24;
}

/* Both targets are little-endian, so a float or double's bytes are already in
 * the format's order. */
static void put_bytes(uint8_t *p, const void *v, size_t n)
{
    memcpy(p, v, n);
}

/* A sector is erased when every byte reads 0xFF. The stage doubles as the
 * buffer: it is exactly a sector, and nothing is staged while opening. */
static bool sector_erased(rec_log *log, uint32_t sector, bool *erased)
{
    if (!log->flash.read(log->flash.ctx, sector * REC_SECTOR_BYTES, log->stage,
                         REC_SECTOR_BYTES)) {
        return false;
    }
    *erased = true;
    for (uint32_t i = 0; i < REC_SECTOR_BYTES; i++) {
        if (log->stage[i] != 0xFFu) {
            *erased = false;
            break;
        }
    }
    return true;
}

rec_status rec_log_open(rec_log *log, const rec_flash *flash)
{
    memset(log, 0, sizeof(*log));
    log->flash = *flash;

    /* The log is written front to back and erased whole, so its sectors are
     * some written ones then only erased ones: binary-search for the first
     * erased sector. The last written byte is in the sector before it -- the
     * gap a session's page rounding leaves is always under a sector. */
    const uint32_t sectors = flash->size / REC_SECTOR_BYTES;
    uint32_t lo = 0, hi = sectors;
    while (lo < hi) {
        const uint32_t mid = lo + (hi - lo) / 2u;
        bool erased;
        if (!sector_erased(log, mid, &erased)) {
            return REC_ERR_IO;
        }
        if (erased) {
            hi = mid;
        } else {
            lo = mid + 1u;
        }
    }
    if (lo > 0) {
        bool erased;
        if (!sector_erased(log, lo - 1u, &erased)) {
            return REC_ERR_IO;
        }
        uint32_t last = REC_SECTOR_BYTES;
        while (last > 0 && log->stage[last - 1u] == 0xFFu) {
            last--;
        }
        /* Round up: whatever wrote the last byte, torn or not, the next
         * session must not share its page. */
        log->end = round_up_page((lo - 1u) * REC_SECTOR_BYTES + last);
    }
    return REC_OK;
}

/* Sessions start on page boundaries, so only those are examined. A block can
 * begin on one too, but its magic differs, and a payload that happens to hold
 * a session magic also needs a matching CRC over 96 bytes. */
static rec_status count_sessions(rec_log *log)
{
    uint8_t rec[REC_SESSION_BYTES];
    log->sessions = 0;
    for (uint32_t pos = 0; pos + REC_SESSION_BYTES <= log->end;
         pos += REC_PAGE_BYTES) {
        if (!log->flash.read(log->flash.ctx, pos, rec, REC_SESSION_BYTES)) {
            return REC_ERR_IO;
        }
        if (get32(rec) == REC_SESSION_MAGIC
            && get32(rec + REC_SESSION_BYTES - 4u)
                   == rec_crc32(rec, REC_SESSION_BYTES - 4u)) {
            log->sessions++;
        }
    }
    log->sessions_counted = true;
    return REC_OK;
}

rec_status rec_log_start(rec_log *log, const rec_session_info *info)
{
    if (log->recording) {
        return REC_ERR_STATE;
    }
    if (!log->sessions_counted) {
        const rec_status s = count_sessions(log);
        if (s != REC_OK) {
            return s;
        }
    }
    if (log->end + REC_SESSION_BYTES + REC_BLOCK_MAX_BYTES > log->flash.size) {
        log->full = true;
        return REC_ERR_FULL;
    }

    uint8_t *p = log->stage;
    memset(p, 0, REC_SESSION_BYTES);
    put32(p, REC_SESSION_MAGIC);
    put16(p + 4, (uint16_t)REC_FORMAT_VERSION);
    put16(p + 6, (uint16_t)REC_SESSION_BYTES);
    put32(p + 8, log->sessions + 1u);
    put32(p + 12, info->start_ms);
    put_bytes(p + 16, &info->rate_hz, 8);
    put_bytes(p + 24, &info->rate_se_hz, 8);
    put16(p + 32, info->gyro_dps);
    put16(p + 34, info->accel_g);
    p[36] = info->putter_id;
    p[37] = info->calibrated;
    put_bytes(p + 40, info->accel_offset_mps2, 12);
    put_bytes(p + 52, info->accel_gain, 12);
    memcpy(p + 64, info->firmware, REC_FIRMWARE_CHARS);
    put32(p + 96, rec_crc32(p, 96));

    log->stage_len = REC_SESSION_BYTES;
    log->sessions++;
    log->recording = true;
    log->full = false;
    log->io_error = false;
    log->block_count = 0;
    log->have_next = false;
    log->samples = 0;
    log->lost = 0;
    log->refused = 0;
    log->bytes = REC_SESSION_BYTES;
    return REC_OK;
}

static rec_status write_page(rec_log *log)
{
    if (!log->flash.write(log->flash.ctx, log->end, log->stage, REC_PAGE_BYTES)) {
        log->io_error = true;
        log->recording = false;
        return REC_ERR_IO;
    }
    log->stage_len -= REC_PAGE_BYTES;
    memmove(log->stage, log->stage + REC_PAGE_BYTES, log->stage_len);
    log->end += REC_PAGE_BYTES;
    return REC_OK;
}

static rec_status flush_and_finish(rec_log *log)
{
    while (log->stage_len >= REC_PAGE_BYTES) {
        const rec_status s = write_page(log);
        if (s != REC_OK) {
            return s;
        }
    }
    if (log->stage_len > 0) {
        /* A partial page. NOR only clears bits, so the rest of the page stays
         * erased and nothing else will ever be written to it. */
        if (!log->flash.write(log->flash.ctx, log->end, log->stage,
                              log->stage_len)) {
            log->io_error = true;
            log->recording = false;
            return REC_ERR_IO;
        }
        log->end = round_up_page(log->end + log->stage_len);
        log->stage_len = 0;
    }
    log->recording = false;
    return REC_OK;
}

/* Encode the open block into the stage. False when the flash is full. */
static bool close_block(rec_log *log)
{
    if (log->block_count == 0) {
        return true;
    }
    const uint8_t count = log->block_count;
    log->block_count = 0;
    if (log->end + log->stage_len + REC_BLOCK_MAX_BYTES > log->flash.size) {
        log->refused += count;
        return false;
    }
    if (REC_STAGE_BYTES - log->stage_len < REC_BLOCK_MAX_BYTES) {
        /* Flash is falling behind. The samples are dropped here, and show in
         * the log as a gap in the index like any other loss. */
        log->refused += count;
        return true;
    }
    const size_t n = rec_block_encode(log->stage + log->stage_len,
                                      log->block_first,
                                      (const int16_t(*)[REC_AXES])log->block,
                                      count);
    log->stage_len += (uint32_t)n;
    log->bytes += (uint32_t)n;
    log->samples += count;
    return true;
}

void rec_log_feed(rec_log *log, uint32_t index, const int16_t accel[3],
                  const int16_t gyro[3])
{
    if (!log->recording) {
        return;
    }
    /* Against the previous sample, not the open block: a gap can fall just
     * after a block closed at REC_BLOCK_MAX. */
    if (log->have_next && index != log->next_index) {
        log->lost += index - log->next_index;
        if (!close_block(log)) {
            log->full = true;
            (void)flush_and_finish(log);
            return;
        }
    }
    log->next_index = index + 1u;
    log->have_next = true;
    if (log->block_count == 0) {
        log->block_first = index;
    }
    int16_t *s = log->block[log->block_count++];
    memcpy(s, accel, 3 * sizeof(int16_t));
    memcpy(s + 3, gyro, 3 * sizeof(int16_t));
    if (log->block_count == REC_BLOCK_MAX) {
        if (!close_block(log)) {
            log->full = true;
            (void)flush_and_finish(log);
        }
    }
}

rec_status rec_log_service(rec_log *log)
{
    if (log->stage_len < REC_PAGE_BYTES || !log->recording) {
        return REC_OK;
    }
    return write_page(log);
}

rec_status rec_log_stop(rec_log *log)
{
    if (!log->recording) {
        return REC_ERR_STATE;
    }
    if (!close_block(log)) {
        log->full = true;
    }
    return flush_and_finish(log);
}

rec_status rec_log_erase_step(rec_log *log, bool *done)
{
    *done = false;
    if (log->recording) {
        return REC_ERR_STATE;
    }
    uint32_t len = log->flash.size - log->erase_next;
    if (len > ERASE_STEP_BYTES) {
        len = ERASE_STEP_BYTES;
    }
    if (len > 0 && !log->flash.erase(log->flash.ctx, log->erase_next, len)) {
        return REC_ERR_IO;
    }
    log->erase_next += len;
    if (log->erase_next >= log->flash.size) {
        log->erase_next = 0;
        log->end = 0;
        log->sessions = 0;
        log->sessions_counted = true;
        log->full = false;
        *done = true;
    }
    return REC_OK;
}
