/* The stroke pipeline on the device: samples in, results to the screens.
 *
 * Runs on the app task, core 1. Parent spec 6.2: "Nothing else runs on [core
 * 0] during a stroke" -- so the pipeline steps here, as stream_service drains
 * the acquisition ring, and its end-of-stroke computation (two passes over up
 * to 3 s of track) never delays a sample being read.
 *
 * It owns the render gate (invariant 8). Armed from BACKSWING through
 * FOLLOWTHROUGH -- Will's decision, 2026-09-28: at ADDRESS the screen would
 * freeze whenever the putter lay still, since still is what ADDRESS means. The
 * console's a / o still work, and are overridden on the next sample.
 */

#include "stroke.h"

#include <stdbool.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "gate.h"
#include "plumb/session.h"
#include "stream.h"
#include "stroke_config.h"
#include "uart_io.h"
#include "ui_port.h"

/* ~157 KB in single precision, almost all of it the path ring
 * (PL_PIPELINE_TRACK_MAX). Allocated at startup from internal SRAM rather than
 * declared static: the static data region cannot hold it (the link overflowed
 * dram0_0_seg by 143,912 bytes), and the heap can reach more of the SRAM. If
 * even that is too little, the pipeline does not run, and says so. */
static pl_session *s_session_mem;
#define s_session (*s_session_mem)
static pl_track s_track;
static bool s_no_memory;

/* Largest first: the internal heap is several regions, and the 98 KB of
 * matrices needs the biggest block before anything smaller splits it. */
static bool allocate(void)
{
    s_track.m = heap_caps_malloc(sizeof(pl_real[PL_PIPELINE_TRACK_MAX][9]),
                                 MALLOC_CAP_INTERNAL);
    s_track.v = heap_caps_malloc(sizeof(pl_real[PL_PIPELINE_TRACK_MAX][3]),
                                 MALLOC_CAP_INTERNAL);
    s_session_mem = heap_caps_malloc(sizeof(pl_session), MALLOC_CAP_INTERNAL);
    if (s_track.m && s_track.v && s_session_mem) {
        return true;
    }
    heap_caps_free(s_track.m);
    heap_caps_free(s_track.v);
    heap_caps_free(s_session_mem);
    s_track.m = NULL;
    s_track.v = NULL;
    s_session_mem = NULL;
    return false;
}
static bool s_running;
static double s_rate_hz;

/* Samples the sensor produced and the pipeline never saw, while a stroke was
 * in progress. The pipeline cannot see a gap -- it integrates the next sample
 * as if it followed directly -- and one missed sample at 50 dps is 0.05 deg of
 * face angle. So a gap is counted and reported with the stroke, not hidden. */
static uint32_t s_prev_index;
static bool s_have_prev;
static uint32_t s_lost_in_stroke;
static uint32_t s_lost_total;

/* The cost of a step, the whole point of measuring on the board. */
static uint32_t s_step_max_us;
static uint64_t s_step_total_us;
static uint32_t s_steps;
static uint32_t s_done_us;          /* the step that produced the last result */

static const char *state_name(pl_state s)
{
    static const char *const names[] = {
        "IDLE", "ADDRESS", "BACKSWING", "DOWNSWING", "IMPACT",
        "FOLLOWTHROUGH", "DONE", "ABANDONED",
    };
    return (unsigned)s < sizeof(names) / sizeof(names[0]) ? names[s] : "?";
}

static bool in_stroke(pl_state s)
{
    return s == PL_STATE_BACKSWING || s == PL_STATE_DOWNSWING
           || s == PL_STATE_IMPACT || s == PL_STATE_FOLLOWTHROUGH;
}

static void report(const pl_stroke_result *r)
{
    /* Face angle relative to ADDRESS, never absolute (invariant 3). */
    uart_io_printf("# stroke %lu: face %+.2f deg rel. address, tempo %.2f "
                   "(%.3f / %.3f s)",
                   (unsigned long)s_session.strokes, r->face_valid ? r->face_angle_deg : 0.0,
                   r->tempo_ratio, r->backswing_s, r->downswing_s);
    if (r->speed_valid) {
        uart_io_printf(", speed %.2f m/s", r->impact_speed_mps);
    } else {
        uart_io_printf(", speed unavailable");
    }
    if (r->path_valid) {
        uart_io_printf(", path arc %.1f mm dir %d", 1e3 * r->path_arc_m,
                       (int)r->path_direction);
    } else {
        uart_io_printf(", path unavailable");
    }
    uart_io_printf("%s, lost in stroke %lu, compute %lu us\n",
                   r->face_valid ? "" : ", FACE UNAVAILABLE",
                   (unsigned long)s_lost_in_stroke, (unsigned long)s_done_us);
}

void stroke_feed(uint32_t index, const acq_record *r)
{
    if (!s_running) {
        acq_rate_estimate rate;
        if (s_no_memory || !stream_sample_rate(&rate)) {
            return; /* dt comes from the measured rate; nothing until it exists */
        }
        if (s_session_mem == NULL && !allocate()) {
            s_no_memory = true;
            uart_io_printf("# stroke: NOT RUNNING -- %u + %u + %u bytes do not fit "
                           "in internal SRAM (largest free block %u B)\n",
                           (unsigned)sizeof(pl_real[PL_PIPELINE_TRACK_MAX][9]),
                           (unsigned)sizeof(pl_real[PL_PIPELINE_TRACK_MAX][3]),
                           (unsigned)sizeof(pl_session),
                           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
            return;
        }
        pl_pipeline_config cfg;
        stroke_config_fill(&cfg, rate.hz);
        if (!pl_session_init(&s_session, &cfg, &s_track)) {
            uart_io_printf("# stroke: config refused by the pipeline\n");
            return;
        }
        s_rate_hz = rate.hz;
        s_running = true;
        uart_io_printf("# stroke: pipeline running at %.4f Hz\n", rate.hz);
    }

    const pl_state before = pl_session_state(&s_session);
    if (s_have_prev && index != s_prev_index + 1) {
        const uint32_t gap = index - s_prev_index - 1;
        s_lost_total += gap;
        if (in_stroke(before)) {
            s_lost_in_stroke += gap;
        }
    }
    s_prev_index = index;
    s_have_prev = true;

    pl_stroke_result result;
    const int64_t t0 = esp_timer_get_time();
    const int produced = pl_session_step(&s_session, r->gyro, r->accel, &result);
    const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    s_step_total_us += us;
    s_steps++;
    if (us > s_step_max_us) {
        s_step_max_us = us;
    }

    const pl_state after = pl_session_state(&s_session);
    if (in_stroke(after)) {
        gate_arm();
    } else {
        gate_open();
    }
    if (!in_stroke(before) && in_stroke(after)) {
        s_lost_in_stroke = 0;
    }

    if (s_session.abandoned_this_step) {
        uart_io_printf("# stroke abandoned (%s) from %s\n",
                       s_session.last_abandon_reason == PL_ABANDON_REST ? "rest" : "timeout",
                       state_name(s_session.last_abandoned_from));
    }
    if (produced) {
        s_done_us = us;
        report(&result);
        ui_port_show_stroke(&result);
    }
}

void stroke_status(void)
{
    if (!s_running) {
        uart_io_printf("# stroke: %s (session %u B)\n",
                       s_no_memory ? "NOT RUNNING, out of internal SRAM"
                                   : "waiting for the sample rate",
                       (unsigned)sizeof(pl_session));
        return;
    }
    uart_io_printf("# stroke: state %s, strokes %lu, abandoned %lu, rate %.4f Hz, "
                   "lost %lu (in last stroke %lu)\n",
                   state_name(pl_session_state(&s_session)),
                   (unsigned long)s_session.strokes, (unsigned long)s_session.abandoned,
                   s_rate_hz, (unsigned long)s_lost_total,
                   (unsigned long)s_lost_in_stroke);
    uart_io_printf("# stroke step: mean %.1f us, max %lu us over %lu steps "
                   "(%s precision, %u-byte session)\n",
                   s_steps ? (double)s_step_total_us / s_steps : 0.0,
                   (unsigned long)s_step_max_us, (unsigned long)s_steps,
                   PL_REAL_NAME, (unsigned)sizeof(s_session));
}
