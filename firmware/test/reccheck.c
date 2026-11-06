/* Host harness for the corpus log (components/rec).
 *
 * Driven by analysis/tests/test_rec.py, which decodes the image this writes
 * with analysis/plumb/corpus.py -- a decoder written from the design document,
 * not from this code. argv[1] is the image file written at the end.
 *
 * The flash is RAM that behaves like NOR: erased to 0xFF, a write can only
 * clear bits (it ANDs), and `tear N` lets exactly N more bytes through, then
 * fails every write -- power lost mid-page. `reopen` is the power coming back.
 *
 * Commands on stdin, whitespace-separated:
 *   flash SIZE                    fresh erased flash, then open
 *   start MS RATE SE PUTTER CAL O0 O1 O2 G0 G1 G2 FIRMWARE
 *   feed N  then N x (index a0 a1 a2 g0 g1 g2)   service after each sample
 *   stop
 *   tear N
 *   reopen
 *   erase
 * Every command prints one line of state.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rec/log.h"

static uint8_t *s_mem;
static uint32_t s_size;
static long long s_budget = -1; /* bytes writes may still land; -1 unlimited */

static bool f_read(void *ctx, uint32_t off, void *dst, uint32_t len)
{
    (void)ctx;
    if (off + len > s_size) {
        return false;
    }
    memcpy(dst, s_mem + off, len);
    return true;
}

static bool f_write(void *ctx, uint32_t off, const void *src, uint32_t len)
{
    (void)ctx;
    if (off + len > s_size) {
        fprintf(stderr, "reccheck: write past the end\n");
        exit(3);
    }
    const uint8_t *p = src;
    for (uint32_t i = 0; i < len; i++) {
        if (s_budget == 0) {
            return false;
        }
        s_mem[off + i] &= p[i];
        if (s_budget > 0) {
            s_budget--;
        }
    }
    return true;
}

static bool f_erase(void *ctx, uint32_t off, uint32_t len)
{
    (void)ctx;
    if (off % REC_SECTOR_BYTES || len % REC_SECTOR_BYTES || off + len > s_size) {
        fprintf(stderr, "reccheck: misaligned erase\n");
        exit(3);
    }
    memset(s_mem + off, 0xFF, len);
    return true;
}

static rec_log s_log;
static rec_flash s_flash = {f_read, f_write, f_erase, NULL, 0};

static long long next(void)
{
    long long v;
    if (scanf("%lld", &v) != 1) {
        fprintf(stderr, "reccheck: input ended early\n");
        exit(2);
    }
    return v;
}

static double next_double(void)
{
    double v;
    if (scanf("%lf", &v) != 1) {
        fprintf(stderr, "reccheck: input ended early\n");
        exit(2);
    }
    return v;
}

static void state(const char *what, rec_status s)
{
    printf("%s status %d end %lu sessions %lu recording %d full %d samples %lu "
           "lost %lu refused %lu bytes %lu\n",
           what, (int)s, (unsigned long)s_log.end, (unsigned long)s_log.sessions,
           s_log.recording, s_log.full, (unsigned long)s_log.samples,
           (unsigned long)s_log.lost, (unsigned long)s_log.refused,
           (unsigned long)s_log.bytes);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: reccheck IMAGE < commands\n");
        return 2;
    }
    char cmd[32];
    while (scanf("%31s", cmd) == 1) {
        if (strcmp(cmd, "flash") == 0) {
            s_size = (uint32_t)next();
            free(s_mem);
            s_mem = malloc(s_size);
            memset(s_mem, 0xFF, s_size);
            s_flash.size = s_size;
            s_budget = -1;
            state("flash", rec_log_open(&s_log, &s_flash));
        } else if (strcmp(cmd, "start") == 0) {
            rec_session_info info;
            memset(&info, 0, sizeof(info));
            info.start_ms = (uint32_t)next();
            info.rate_hz = next_double();
            info.rate_se_hz = next_double();
            info.gyro_dps = 256;
            info.accel_g = 16;
            info.putter_id = (uint8_t)next();
            info.calibrated = (uint8_t)next();
            for (int i = 0; i < 3; i++) {
                info.accel_offset_mps2[i] = (float)next_double();
            }
            for (int i = 0; i < 3; i++) {
                info.accel_gain[i] = (float)next_double();
            }
            if (scanf("%31s", info.firmware) != 1) {
                return 2;
            }
            state("start", rec_log_start(&s_log, &info));
        } else if (strcmp(cmd, "feed") == 0) {
            const long long n = next();
            rec_status worst = REC_OK;
            for (long long i = 0; i < n; i++) {
                const uint32_t index = (uint32_t)next();
                int16_t a[3], g[3];
                for (int k = 0; k < 3; k++) {
                    a[k] = (int16_t)next();
                }
                for (int k = 0; k < 3; k++) {
                    g[k] = (int16_t)next();
                }
                rec_log_feed(&s_log, index, a, g);
                const rec_status s = rec_log_service(&s_log);
                if (s != REC_OK) {
                    worst = s;
                }
            }
            state("feed", worst);
        } else if (strcmp(cmd, "stop") == 0) {
            state("stop", rec_log_stop(&s_log));
        } else if (strcmp(cmd, "tear") == 0) {
            s_budget = next();
            state("tear", REC_OK);
        } else if (strcmp(cmd, "reopen") == 0) {
            s_budget = -1;
            state("reopen", rec_log_open(&s_log, &s_flash));
        } else if (strcmp(cmd, "erase") == 0) {
            bool done = false;
            rec_status s = REC_OK;
            while (!done && s == REC_OK) {
                s = rec_log_erase_step(&s_log, &done);
            }
            state("erase", s);
        } else {
            fprintf(stderr, "reccheck: unknown command %s\n", cmd);
            return 2;
        }
    }
    FILE *f = fopen(argv[1], "wb");
    if (!f || fwrite(s_mem, 1, s_size, f) != s_size) {
        return 3;
    }
    fclose(f);
    return 0;
}
