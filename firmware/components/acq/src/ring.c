#include "acq/ring.h"

#include <string.h>

/* Acquire/release ordering between the cores. On the device this is GCC's
 * __atomic builtins, which emit the memory barriers the two LX7 cores need.
 * The host harness is built with MSVC and used from one thread, so there it
 * degrades to plain accesses: the host test proves ordering of the queue and
 * the overflow accounting, not the memory model, and says so here. */
#if defined(__GNUC__)
#define LOAD_ACQUIRE(p) __atomic_load_n((p), __ATOMIC_ACQUIRE)
#define STORE_RELEASE(p, v) __atomic_store_n((p), (v), __ATOMIC_RELEASE)
#else
#define LOAD_ACQUIRE(p) (*(p))
#define STORE_RELEASE(p, v) (*(p) = (v))
#endif

#define MASK (ACQ_RING_CAPACITY - 1u)

void acq_ring_init(acq_ring *ring)
{
    memset(ring, 0, sizeof(*ring));
}

bool acq_ring_push(acq_ring *ring, const acq_record *record)
{
    const uint32_t head = ring->head;
    const uint32_t tail = LOAD_ACQUIRE(&ring->tail);
    if (head - tail >= ACQ_RING_CAPACITY) {
        STORE_RELEASE(&ring->dropped, ring->dropped + 1u);
        return false;
    }
    ring->slots[head & MASK] = *record;
    STORE_RELEASE(&ring->head, head + 1u);
    return true;
}

bool acq_ring_pop(acq_ring *ring, acq_record *out)
{
    const uint32_t tail = ring->tail;
    const uint32_t head = LOAD_ACQUIRE(&ring->head);
    if (head == tail) {
        return false;
    }
    *out = ring->slots[tail & MASK];
    STORE_RELEASE(&ring->tail, tail + 1u);
    return true;
}

uint32_t acq_ring_dropped(const acq_ring *ring)
{
    return LOAD_ACQUIRE(&ring->dropped);
}
