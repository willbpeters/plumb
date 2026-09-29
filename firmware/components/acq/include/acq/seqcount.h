/* Sample accounting from the sensor's own 24-bit sample counter.
 *
 * This is the method that found the FIFO path's 21.9% loss: the part numbers
 * every sample it produces, so a step of more than one between consecutive
 * reads is a sample produced and never read, and a step of zero is the same
 * sample read twice. Counting what arrived cannot see either (imu_stream's
 * "dropped: 0" alongside 20% loss).
 *
 * A step of more than half the counter's range is taken as the counter going
 * backwards rather than as eight million lost samples.
 */
#ifndef ACQ_SEQCOUNT_H
#define ACQ_SEQCOUNT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ACQ_SEQ_NEW = 0,       /* a sample not seen before; index is its position */
    ACQ_SEQ_DUPLICATE = 1, /* the same counter value as the previous record */
    ACQ_SEQ_BACKWARD = 2,  /* the counter went back; not placed */
} acq_seq_result;

typedef struct {
    bool started;
    uint32_t last;       /* last counter value accepted, 24 bits */
    uint32_t index;      /* unwrapped position of that sample since the first */
    uint32_t received;   /* every record fed in */
    uint32_t lost;       /* samples the counter says were produced and never read */
    uint32_t duplicated;
    uint32_t backward;
} acq_seqcount;

void acq_seqcount_reset(acq_seqcount *s);

/* Feed one record's counter. *index receives the sample's position since the
 * first record for ACQ_SEQ_NEW and ACQ_SEQ_DUPLICATE. */
acq_seq_result acq_seqcount_update(acq_seqcount *s, uint32_t timestamp,
                                   uint32_t *index);

/* Samples the sensor produced from the first record to the last, inclusive. */
uint32_t acq_seqcount_produced(const acq_seqcount *s);

#endif /* ACQ_SEQCOUNT_H */
