/* The IMU's real sample rate, measured on the board at startup.
 *
 * Will's decision, 2026-09-27. The QMI8658's ODR comes from its own MEMS
 * oscillator, and this unit runs at 906.86 Hz against 896.8 nominal -- 1.12%,
 * which goes straight into every integrated angle as a scale error. It is a
 * property of the part, not the part number, so no constant can carry it.
 *
 * THE MEASUREMENT. A least-squares line through (sample index, DRDY edge
 * time). The index is the sensor's own counter, unwrapped by acq_seqcount, so
 * a lost sample thins the fit instead of reading as a slower rate. The edge
 * time is esp_timer in the ISR. The slope is the period.
 *
 * TWO GUARDS, because a wrong pairing of time and index is ~one period off
 * the line against a few microseconds of jitter:
 *   - records with edges != 1 are skipped: the task fell behind, so the stored
 *     edge time may belong to a later sample than the one read;
 *   - after a first fit, points more than REJECT_SIGMAS robust standard
 *     deviations from the line are dropped and the line refitted.
 *
 * WHAT THE UNCERTAINTY COVERS. hz_se is the statistical error of the fit,
 * from its own residuals. It does NOT include the ESP32-S3's crystal, which
 * esp_timer counts against -- that is a separate, systematic term, stated
 * where the result is used.
 *
 * Pure C, no ESP-IDF: the host harness shares it (analysis/tests/test_acq.py).
 */
#ifndef ACQ_RATE_H
#define ACQ_RATE_H

#include <stdbool.h>
#include <stdint.h>

/* A chosen false-alarm criterion for dropping a point, not a tuned threshold:
 * at 5 robust sigmas a Gaussian point is dropped about once in 1.7 million.
 * A mispaired edge sits hundreds of sigmas out. */
#define ACQ_RATE_REJECT_SIGMAS 5.0

/* esp_timer's resolution. The robust scale is floored here so a run of
 * exactly-on-the-line points cannot make every rounding look like an outlier. */
#define ACQ_RATE_SCALE_FLOOR_US 1.0

typedef struct {
    uint32_t index;     /* unwrapped sample index, acq_seqcount */
    uint32_t edge_us;   /* esp_timer at the DRDY edge, low 32 bits */
    float work;         /* residual scratch for the solve */
} acq_rate_point;

typedef struct {
    acq_rate_point *points;
    uint32_t capacity;
    uint32_t count;
    uint32_t skipped;   /* records with edges != 1 */
} acq_rate;

typedef struct {
    double hz;
    double hz_se;           /* statistical only; see above */
    double rms_us;          /* residual RMS of the points kept */
    double max_us;          /* largest |residual| kept */
    double span_s;          /* first to last point accepted */
    uint32_t accepted;      /* points stored */
    uint32_t used;          /* points kept by the fit */
    uint32_t rejected;      /* dropped as off the line */
    uint32_t skipped;
} acq_rate_estimate;

/* `storage` holds `capacity` points and must outlive the measurement. */
void acq_rate_init(acq_rate *r, acq_rate_point *storage, uint32_t capacity);

/* One record, as acq_seqcount placed it. Returns false once full. */
bool acq_rate_add(acq_rate *r, uint32_t index, uint32_t edge_us,
                  uint8_t edges);

bool acq_rate_full(const acq_rate *r);

/* Fit. Returns false with fewer than three points left to fit. Reorders the
 * stored points. */
bool acq_rate_solve(acq_rate *r, acq_rate_estimate *out);

#endif /* ACQ_RATE_H */
