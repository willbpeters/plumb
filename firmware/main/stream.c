#include "stream.h"

#include <string.h>

#include "acq/acq.h"
#include "acq/frame.h"
#include "acq/jitter.h"
#include "acq/seqcount.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "gate.h"
#include "uart_io.h"

static acq_ring *s_ring;

/* Since boot: every record, whether or not anything is streaming. */
static acq_seqcount s_boot;
static uint32_t s_prev_index;
static uint32_t s_prev_edge_us;
static bool s_have_prev;

/* The jitter window. Two histograms (8 KB each, internal RAM). */
static acq_hist s_interval;
static acq_hist s_latency;
static uint32_t s_window_start_us;
static bool s_window_armed;
static bool s_window_mixed;
/* The latency, split into its three consecutive segments: edge to the task
 * running, the STATUSINT read, and the lock wait plus the burst. Rendering
 * lengthens the total; these say which part. Allocated in PSRAM at init. */
static acq_hist *s_wake;
static acq_hist *s_statusint;
static acq_hist *s_burst;

/* The stream to the host. */
static bool s_streaming;
static bool s_binary;
static acq_seqcount s_stream;
static uint32_t s_stream_start_us;
static acq_record s_pending[ACQ_FRAME_MAX_SAMPLES];
static uint8_t s_pending_n;
static uint32_t s_pending_first;
static uint32_t s_dropped_seen;

static void window_reset(void)
{
    acq_hist_reset(&s_interval);
    acq_hist_reset(&s_latency);
    if (s_wake) {
        acq_hist_reset(s_wake);
        acq_hist_reset(s_statusint);
        acq_hist_reset(s_burst);
    }
    s_window_start_us = (uint32_t)esp_timer_get_time();
    s_window_armed = gate_armed();
    s_window_mixed = false;
}

void stream_init(void)
{
    s_ring = acq_ring_handle();
    acq_seqcount_reset(&s_boot);
    /* PSRAM: diagnostic counts, written a few times a millisecond by core 1,
     * kept out of the internal budget the acquisition path lives in. */
    s_wake = heap_caps_malloc(sizeof(acq_hist), MALLOC_CAP_SPIRAM);
    s_statusint = heap_caps_malloc(sizeof(acq_hist), MALLOC_CAP_SPIRAM);
    s_burst = heap_caps_malloc(sizeof(acq_hist), MALLOC_CAP_SPIRAM);
    if (!s_wake || !s_statusint || !s_burst) {
        uart_io_printf("# latency segments unavailable: out of PSRAM\n");
        heap_caps_free(s_wake);
        heap_caps_free(s_statusint);
        heap_caps_free(s_burst);
        s_wake = s_statusint = s_burst = NULL;
    }
    window_reset();
}

static void flush_frame(void)
{
    if (s_pending_n == 0) {
        return;
    }
    const uint32_t dropped = acq_ring_dropped(s_ring);
    uint8_t flags = ACQ_FRAME_FLAG_SENSOR_SEQ;
    if (dropped != s_dropped_seen) {
        flags |= ACQ_FRAME_FLAG_OVERFLOW;
        s_dropped_seen = dropped;
    }
    uint8_t out[ACQ_FRAME_BYTES(ACQ_FRAME_MAX_SAMPLES)];
    const size_t len = acq_frame_encode(out, s_pending_first, flags,
                                        s_pending[0].edge_us, s_pending,
                                        s_pending_n);
    uart_io_write(out, len);
    s_pending_n = 0;
}

static void emit(const acq_record *r)
{
    uint32_t index;
    if (acq_seqcount_update(&s_stream, r->timestamp, &index) != ACQ_SEQ_NEW) {
        return; /* counted, not sent: a repeat is not a measurement */
    }
    if (!s_binary) {
        uart_io_printf("%lu,%d,%d,%d,%d,%d,%d\n", (unsigned long)index,
                       r->accel[0], r->accel[1], r->accel[2],
                       r->gyro[0], r->gyro[1], r->gyro[2]);
        return;
    }
    /* A frame's samples are numbered consecutively from first_seq, so a gap
     * ends the frame: the host sees the gap as a gap. */
    if (s_pending_n > 0 && (index != s_pending_first + s_pending_n ||
                            s_pending_n == ACQ_FRAME_MAX_SAMPLES)) {
        flush_frame();
    }
    if (s_pending_n == 0) {
        s_pending_first = index;
    }
    s_pending[s_pending_n++] = *r;
}

void stream_service(void)
{
    if (gate_armed() != s_window_armed) {
        s_window_mixed = true;
    }
    acq_record r;
    while (acq_ring_pop(s_ring, &r)) {
        uint32_t index;
        if (acq_seqcount_update(&s_boot, r.timestamp, &index) == ACQ_SEQ_NEW) {
            /* Edge-to-edge only across consecutive samples: a lost sample
             * would read as a doubled interval, and loss is counted apart. */
            if (s_have_prev && index == s_prev_index + 1) {
                acq_hist_add(&s_interval, r.edge_us - s_prev_edge_us);
            }
            acq_hist_add(&s_latency, r.done_us - r.edge_us);
            if (s_wake) {
                acq_hist_add(s_wake, r.wake_us);
                acq_hist_add(s_statusint, (uint32_t)(r.status_us - r.wake_us));
                acq_hist_add(s_burst, (r.done_us - r.edge_us) - r.status_us);
            }
            s_prev_index = index;
            s_prev_edge_us = r.edge_us;
            s_have_prev = true;
        }
        if (s_streaming) {
            emit(&r);
        }
    }
    flush_frame();
}

void stream_toggle(void)
{
    flush_frame();
    s_streaming = !s_streaming;
    acq_seqcount_reset(&s_stream);
    s_pending_n = 0;
    s_dropped_seen = acq_ring_dropped(s_ring);
    s_stream_start_us = (uint32_t)esp_timer_get_time();
    uart_io_printf("# streaming %d\n", s_streaming);
}

bool stream_running(void)
{
    return s_streaming;
}

void stream_set_binary(bool binary)
{
    s_binary = binary;
}

bool stream_binary(void)
{
    return s_binary;
}

static void print_hist(const char *name, const acq_hist *h)
{
    char p[3][12];
    const uint32_t ranks[3] = {500, 990, 999};
    for (int i = 0; i < 3; i++) {
        const uint32_t v = acq_hist_percentile(h, ranks[i]);
        if (v == ACQ_HIST_UNKNOWN) {
            strcpy(p[i], h->count ? ">=2048" : "-");
        } else {
            snprintf(p[i], sizeof(p[i]), "%lu", (unsigned long)v);
        }
    }
    uart_io_printf("# %s n=%lu min=%lu p50=%s p99=%s p99.9=%s max=%lu over2048=%lu\n",
                   name, (unsigned long)h->count,
                   (unsigned long)(h->count ? h->min : 0), p[0], p[1], p[2],
                   (unsigned long)h->max, (unsigned long)h->over);
}

void stream_jitter_report(void)
{
    const uint32_t now = (uint32_t)esp_timer_get_time();
    const char *gate = s_window_mixed ? "CHANGED DURING WINDOW"
                       : s_window_armed ? "armed throughout" : "open throughout";
    uart_io_printf("# jitter window %.2f s, gate %s\n",
                   (now - s_window_start_us) / 1e6, gate);
    print_hist("interval_us", &s_interval);
    print_hist("latency_us", &s_latency);
    if (s_wake) {
        print_hist("  wake_us", s_wake);
        print_hist("  statusint_us", s_statusint);
        print_hist("  burst_us", s_burst);
    }
    window_reset();
}

void stream_status(void)
{
    acq_counters c;
    acq_counters_get(&c);
    const uint32_t elapsed = (uint32_t)esp_timer_get_time() - s_stream_start_us;
    const uint32_t produced = acq_seqcount_produced(&s_stream);
    uart_io_printf("# plumb firmware: format=%s streaming=%d gate=%s\n",
                   s_binary ? "binary" : "csv", s_streaming,
                   gate_armed() ? "armed" : "open");
    uart_io_printf("# since boot: produced=%lu lost=%lu duplicated=%lu backward=%lu ring_dropped=%lu\n",
                   (unsigned long)acq_seqcount_produced(&s_boot),
                   (unsigned long)s_boot.lost, (unsigned long)s_boot.duplicated,
                   (unsigned long)s_boot.backward,
                   (unsigned long)acq_ring_dropped(s_ring));
    uart_io_printf("# acq: reads=%lu read_errors=%lu not_available=%lu unlocked=%lu missed_edges=%lu drdy_timeouts=%lu\n",
                   (unsigned long)c.reads, (unsigned long)c.read_errors,
                   (unsigned long)c.not_available, (unsigned long)c.unlocked,
                   (unsigned long)c.missed_edges, (unsigned long)c.drdy_timeouts);
    if (s_streaming && elapsed > 0) {
        uart_io_printf("# stream: produced=%lu lost=%lu produced_hz=%.2f\n",
                       (unsigned long)produced, (unsigned long)s_stream.lost,
                       produced * 1e6 / elapsed);
    }
}
