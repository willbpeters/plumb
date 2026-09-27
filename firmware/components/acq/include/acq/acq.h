/* The acquisition task: core 0, highest application priority, woken by the
 * IMU's data-ready edge (parent spec 6.2 as amended, invariant 8). */
#ifndef ACQ_ACQ_H
#define ACQ_ACQ_H

#include <stdint.h>

#include "acq/ring.h"
#include "esp_err.h"

typedef struct {
    uint32_t reads;          /* records pushed or refused */
    uint32_t read_errors;    /* I2C errors; no record */
    uint32_t not_available;  /* woken, but STATUSINT never showed Avail; no record */
    uint32_t unlocked;       /* Avail seen before Locked: waited Data_Lock_Delay */
    uint32_t missed_edges;   /* DRDY edges beyond one per read: the task fell behind */
    uint32_t drdy_timeouts;  /* 100 ms with no edge at all */
} acq_counters;

/* Start the task pinned to core 0 and wait for the IMU to configure. Must be
 * called after board_i2c_init(). */
esp_err_t acq_start(void);

/* The ring the task fills. The app task is its only reader. */
acq_ring *acq_ring_handle(void);

void acq_counters_get(acq_counters *out);

#endif /* ACQ_ACQ_H */
