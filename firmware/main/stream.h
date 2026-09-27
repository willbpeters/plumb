/* Drains the acquisition ring on the app task: sample accounting since boot,
 * the jitter histograms, and the stream to the host in imu_stream's framing. */
#ifndef STREAM_H
#define STREAM_H

#include <stdbool.h>

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

#endif /* STREAM_H */
