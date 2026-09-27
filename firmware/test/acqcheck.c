/* Host harness for the acquisition core's pure-C modules.
 *
 * Driven by analysis/tests/test_acq.py, which checks each module against an
 * independent reference. The first argument picks the module; the input is
 * whitespace-separated integers on stdin.
 *
 *   hist  : n, n values, k, k per-mille ranks
 *   seq   : n, n counter values
 *   ring  : n, n ops (1 push the next value, 0 pop)
 *   frame : first_seq, flags, micros, n, n x 6 samples
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "acq/frame.h"
#include "acq/jitter.h"
#include "acq/ring.h"
#include "acq/seqcount.h"

static long long next(void)
{
    long long v;
    if (scanf("%lld", &v) != 1) {
        fprintf(stderr, "acqcheck: input ended early\n");
        exit(2);
    }
    return v;
}

static int run_hist(void)
{
    static acq_hist h;
    acq_hist_reset(&h);
    const long long n = next();
    for (long long i = 0; i < n; i++) {
        acq_hist_add(&h, (uint32_t)next());
    }
    printf("count %lu over %lu min %lu max %lu\n", (unsigned long)h.count,
           (unsigned long)h.over, (unsigned long)h.min, (unsigned long)h.max);
    const long long k = next();
    for (long long i = 0; i < k; i++) {
        const uint32_t pm = (uint32_t)next();
        printf("p %lu %lu\n", (unsigned long)pm,
               (unsigned long)acq_hist_percentile(&h, pm));
    }
    return 0;
}

static int run_seq(void)
{
    acq_seqcount s;
    acq_seqcount_reset(&s);
    const long long n = next();
    for (long long i = 0; i < n; i++) {
        uint32_t index = 0;
        const acq_seq_result r = acq_seqcount_update(&s, (uint32_t)next(), &index);
        printf("r %d %lu\n", (int)r, (unsigned long)index);
    }
    printf("totals received %lu lost %lu duplicated %lu backward %lu produced %lu\n",
           (unsigned long)s.received, (unsigned long)s.lost,
           (unsigned long)s.duplicated, (unsigned long)s.backward,
           (unsigned long)acq_seqcount_produced(&s));
    return 0;
}

static int run_ring(void)
{
    static acq_ring ring;
    acq_ring_init(&ring);
    const long long n = next();
    uint32_t value = 0;
    for (long long i = 0; i < n; i++) {
        if (next() == 1) {
            acq_record r;
            memset(&r, 0, sizeof(r));
            r.timestamp = ++value;
            puts(acq_ring_push(&ring, &r) ? "push ok" : "push full");
        } else {
            acq_record r;
            if (acq_ring_pop(&ring, &r)) {
                printf("pop %lu\n", (unsigned long)r.timestamp);
            } else {
                puts("pop empty");
            }
        }
    }
    printf("dropped %lu\n", (unsigned long)acq_ring_dropped(&ring));
    return 0;
}

static int run_frame(void)
{
    const uint32_t first_seq = (uint32_t)next();
    const uint8_t flags = (uint8_t)next();
    const uint32_t micros = (uint32_t)next();
    const uint8_t n = (uint8_t)next();
    acq_record records[ACQ_FRAME_MAX_SAMPLES];
    memset(records, 0, sizeof(records));
    for (uint8_t i = 0; i < n; i++) {
        for (int axis = 0; axis < 3; axis++) {
            records[i].accel[axis] = (int16_t)next();
        }
        for (int axis = 0; axis < 3; axis++) {
            records[i].gyro[axis] = (int16_t)next();
        }
    }
    uint8_t out[ACQ_FRAME_BYTES(ACQ_FRAME_MAX_SAMPLES)];
    const size_t len = acq_frame_encode(out, first_seq, flags, micros, records, n);
    for (size_t i = 0; i < len; i++) {
        printf("%02x", out[i]);
    }
    printf("\n");
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: acqcheck hist|seq|ring|frame < numbers\n");
        return 2;
    }
    if (strcmp(argv[1], "hist") == 0) return run_hist();
    if (strcmp(argv[1], "seq") == 0) return run_seq();
    if (strcmp(argv[1], "ring") == 0) return run_ring();
    if (strcmp(argv[1], "frame") == 0) return run_frame();
    fprintf(stderr, "acqcheck: unknown mode %s\n", argv[1]);
    return 2;
}
