/* The stroke pipeline on the device, and the render gate it drives. */
#ifndef STROKE_H
#define STROKE_H

#include <stdint.h>

#include "acq/record.h"

/* One new sample, as acq_seqcount placed it. Call from the ring drain, for
 * ACQ_SEQ_NEW records only, in order. Does nothing until the sample rate has
 * been measured. */
void stroke_feed(uint32_t index, const acq_record *r);

void stroke_status(void);

#endif /* STROKE_H */
