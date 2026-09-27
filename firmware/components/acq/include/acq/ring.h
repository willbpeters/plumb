/* Single-producer, single-consumer ring of acq_records.
 *
 * The producer is the acquisition task on core 0 and the consumer the app task
 * on core 1 (parent spec 6.2: "a lock-free ring buffer"). No lock and no
 * critical section: a FreeRTOS queue would take a spinlock that both cores
 * contend for, and that is jitter the acquisition core does not need.
 *
 * head and tail are free-running counters; the slot is the counter modulo the
 * capacity. The producer alone writes head and dropped, the consumer alone
 * writes tail.
 */
#ifndef ACQ_RING_H
#define ACQ_RING_H

#include <stdbool.h>
#include <stdint.h>

#include "acq/record.h"

/* A power of two. 512 records is 0.56 s at 906.86 Hz, which covers the
 * longest the app task blocks: one full-screen LVGL render. */
#define ACQ_RING_CAPACITY 512u

typedef struct {
    acq_record slots[ACQ_RING_CAPACITY];
    uint32_t head;
    uint32_t tail;
    uint32_t dropped; /* records refused because the ring was full */
} acq_ring;

void acq_ring_init(acq_ring *ring);

/* Producer. False, and counted in dropped, when the ring is full. */
bool acq_ring_push(acq_ring *ring, const acq_record *record);

/* Consumer. False when the ring is empty. */
bool acq_ring_pop(acq_ring *ring, acq_record *out);

/* Either side. Monotonic. */
uint32_t acq_ring_dropped(const acq_ring *ring);

#endif /* ACQ_RING_H */
