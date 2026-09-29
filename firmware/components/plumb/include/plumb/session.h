/* The device loop: one stroke after another. Translation of
 * analysis/plumb/session.py.
 *
 * A pl_pipeline is one stroke, and DONE and ABANDONED are terminal. The device
 * runs indefinitely, so this starts the next one -- and carries the pivot
 * calibration across, because the pivot is the golfer's, learned over a
 * session (parent spec 8.4). Pure C99, no allocation: the caller holds one of
 * these, statically; it is a pl_pipeline and a little more.
 */

#ifndef PLUMB_SESSION_H
#define PLUMB_SESSION_H

#include <stdint.h>

#include "plumb/pipeline.h"
#include "plumb/pivot.h"

typedef struct {
    pl_pipeline pipeline;
    pl_pivot_calibration calibration;
    pl_pipeline_config cfg;
    pl_track track;
    uint32_t strokes;               /* results produced */
    uint32_t abandoned;             /* strokes abandoned */
    /* Set on the step a stroke was abandoned, cleared on the next. */
    int abandoned_this_step;
    pl_abandon_reason last_abandon_reason;
    pl_state last_abandoned_from;
} pl_session;

/* 0 if the pipeline refuses the config (pl_pipeline_init). */
int pl_session_init(pl_session *s, const pl_pipeline_config *cfg,
                    const pl_track *track);

/* One sample. Returns 1 and fills `out` on the sample a stroke finishes. */
int pl_session_step(pl_session *s, const int16_t gyro_counts[3],
                    const int16_t accel_counts[3], pl_stroke_result *out);

/* The state of the stroke in progress, for whoever drives the render gate. */
pl_state pl_session_state(const pl_session *s);

#endif /* PLUMB_SESSION_H */
