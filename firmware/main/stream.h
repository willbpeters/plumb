/* Drains the acquisition ring on the app task: sample accounting since boot,
 * the jitter histograms, and the stream to the host in imu_stream's framing. */
#ifndef STREAM_H
#define STREAM_H

#include <stdbool.h>

#include "acq/rate.h"

void stream_init(void);
/* Drain everything in the ring. Call every pass of the app loop. */
void stream_service(void);

void stream_toggle(void);
bool stream_running(void);
void stream_set_binary(bool binary);
bool stream_binary(void);

/* Print the jitter window since the last report, then start a new one. */
void stream_jitter_report(void);
void stream_status(void);

/* The IMU's measured sample rate (acq/rate.h). Measured from the first
 * samples after stream_init() and again on stream_measure_rate(). False until
 * a measurement has finished. */
bool stream_sample_rate(acq_rate_estimate *out);
void stream_measure_rate(void);

#endif /* STREAM_H */
