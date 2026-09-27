#include "acq/jitter.h"

#include <string.h>

void acq_hist_reset(acq_hist *h)
{
    memset(h, 0, sizeof(*h));
    h->min = ACQ_HIST_UNKNOWN;
}

void acq_hist_add(acq_hist *h, uint32_t us)
{
    h->count++;
    if (us < h->min) {
        h->min = us;
    }
    if (us > h->max) {
        h->max = us;
    }
    if (us < ACQ_HIST_BINS) {
        h->bins[us]++;
    } else {
        h->over++;
    }
}

uint32_t acq_hist_percentile(const acq_hist *h, uint32_t per_mille)
{
    if (h->count == 0) {
        return ACQ_HIST_UNKNOWN;
    }
    /* Integer ceiling: the rank is exact, with no floating point to round. */
    uint64_t rank = ((uint64_t)per_mille * h->count + 999u) / 1000u;
    if (rank == 0) {
        rank = 1;
    }
    uint64_t seen = 0;
    for (uint32_t us = 0; us < ACQ_HIST_BINS; us++) {
        seen += h->bins[us];
        if (seen >= rank) {
            return us;
        }
    }
    return ACQ_HIST_UNKNOWN;
}
