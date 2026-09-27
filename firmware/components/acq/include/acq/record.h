/* One IMU sample as the acquisition task read it.
 *
 * Raw counts and raw times only: the corpus stores raw samples, never derived
 * metrics (CLAUDE.md), and this is the record everything downstream is built
 * from. Pure C, no ESP-IDF, so the host harness shares it.
 */
#ifndef ACQ_RECORD_H
#define ACQ_RECORD_H

#include <stdint.h>

typedef struct {
    uint32_t timestamp;  /* the sensor's 24-bit sample counter, TIMESTAMP_L..H */
    int16_t accel[3];    /* AX, AY, AZ, raw counts */
    int16_t gyro[3];     /* GX, GY, GZ, raw counts */
    uint32_t edge_us;    /* esp_timer at the DRDY rising edge, low 32 bits */
    uint32_t done_us;    /* esp_timer when the burst read completed */
    uint8_t edges;       /* DRDY edges since the previous read: 1 unless the task fell behind */
    uint8_t statusint;   /* STATUSINT as first seen with Avail set: bit 0 Avail, bit 1 Locked */
    uint8_t polls;       /* STATUSINT reads it took to see Avail */
    uint8_t reserved;
} acq_record;

#endif /* ACQ_RECORD_H */
