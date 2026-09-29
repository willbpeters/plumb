/* A 1 us histogram with exact extremes and nearest-rank percentiles.
 *
 * Used for two timings per sample: the interval between successive DRDY
 * edges, and the latency from an edge to the end of its read. No pass or fail
 * threshold is attached to either (the skeleton spec, invariant 5's spirit):
 * the histogram reports, a person decides.
 *
 * Values at or beyond ACQ_HIST_BINS microseconds are counted, and their
 * maximum kept exactly, but they are not binned. A percentile that falls
 * among them is ACQ_HIST_UNKNOWN rather than a made-up number.
 */
#ifndef ACQ_JITTER_H
#define ACQ_JITTER_H

#include <stdint.h>

#define ACQ_HIST_BINS 2048u
#define ACQ_HIST_UNKNOWN 0xFFFFFFFFu

typedef struct {
    uint32_t bins[ACQ_HIST_BINS];
    uint32_t count;
    uint32_t over; /* values >= ACQ_HIST_BINS */
    uint32_t min;
    uint32_t max;
} acq_hist;

void acq_hist_reset(acq_hist *h);
void acq_hist_add(acq_hist *h, uint32_t us);

/* The smallest recorded value v such that at least ceil(per_mille/1000 *
 * count) values are <= v. per_mille in 1..1000: 500 is the median, 990 p99. */
uint32_t acq_hist_percentile(const acq_hist *h, uint32_t per_mille);

#endif /* ACQ_JITTER_H */
