#include "acq/seqcount.h"

#include <string.h>

#define MASK24 0x00FFFFFFu
#define HALF24 0x00800000u

void acq_seqcount_reset(acq_seqcount *s)
{
    memset(s, 0, sizeof(*s));
}

acq_seq_result acq_seqcount_update(acq_seqcount *s, uint32_t timestamp,
                                   uint32_t *index)
{
    timestamp &= MASK24;
    s->received++;
    if (!s->started) {
        s->started = true;
        s->last = timestamp;
        s->index = 0;
        *index = 0;
        return ACQ_SEQ_NEW;
    }

    const uint32_t step = (timestamp - s->last) & MASK24;
    if (step == 0) {
        s->duplicated++;
        *index = s->index;
        return ACQ_SEQ_DUPLICATE;
    }
    if (step >= HALF24) {
        s->backward++;
        return ACQ_SEQ_BACKWARD;
    }
    s->lost += step - 1u;
    s->index += step;
    s->last = timestamp;
    *index = s->index;
    return ACQ_SEQ_NEW;
}

uint32_t acq_seqcount_produced(const acq_seqcount *s)
{
    return s->started ? s->index + 1u : 0u;
}
