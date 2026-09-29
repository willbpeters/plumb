/* See plumb/session.h. Translation of analysis/plumb/session.py. */

#include "plumb/session.h"

int pl_session_init(pl_session *s, const pl_pipeline_config *cfg,
                    const pl_track *track)
{
    s->cfg = *cfg;
    s->track = *track;
    s->strokes = 0;
    s->abandoned = 0;
    s->abandoned_this_step = 0;
    s->last_abandon_reason = PL_ABANDON_NONE;
    s->last_abandoned_from = PL_STATE_IDLE;
    pl_pivot_calibration_init(&s->calibration);
    return pl_pipeline_init(&s->pipeline, &s->cfg, &s->calibration, &s->track);
}

int pl_session_step(pl_session *s, const int16_t gyro_counts[3],
                    const int16_t accel_counts[3], pl_stroke_result *out)
{
    const int produced = pl_pipeline_step(&s->pipeline, gyro_counts,
                                          accel_counts, out);
    s->abandoned_this_step = 0;
    if (s->pipeline.state == PL_STATE_ABANDONED) {
        s->abandoned++;
        s->abandoned_this_step = 1;
        s->last_abandon_reason = s->pipeline.abandon_reason;
        s->last_abandoned_from = s->pipeline.abandoned_from;
    }
    if (produced) {
        s->strokes++;
    }
    if (s->pipeline.state == PL_STATE_DONE
        || s->pipeline.state == PL_STATE_ABANDONED) {
        /* The config was accepted once already, so this cannot refuse it. */
        (void)pl_pipeline_init(&s->pipeline, &s->cfg, &s->calibration,
                               &s->track);
    }
    return produced;
}

pl_state pl_session_state(const pl_session *s)
{
    return s->pipeline.state;
}
