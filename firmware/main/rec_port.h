/* The corpus log on the device: the `corpus` partition, the console's control
 * of it, and the record-from-boot flag. docs/superpowers/specs/
 * 2026-09-29-stroke-logging-design.md. */
#ifndef REC_PORT_H
#define REC_PORT_H

#include <stdint.h>

#include "acq/record.h"

/* Find the partition and the log's end. Call before the ring is drained. */
void rec_port_init(void);

/* One new sample, as acq_seqcount placed it. Call from the ring drain, for
 * ACQ_SEQ_NEW records only, in order. */
void rec_port_feed(uint32_t index, const acq_record *r);

/* At most one page write, or one 64 KB erase step. Every pass of the app loop. */
void rec_port_service(void);

/* Console. Returns 1 if the key was the log's. */
int rec_port_console(char c);

void rec_port_status(void);

#endif /* REC_PORT_H */
