#include "acq/rate.h"

#include <math.h>
#include <stdlib.h>

/* 1.4826 x MAD estimates a Gaussian standard deviation. */
#define MAD_TO_SIGMA 1.4826

typedef struct {
    double slope;       /* us per sample */
    double intercept;   /* us at x = 0 */
    double sxx;
    double ssr;
    uint32_t n;
} line_fit;

/* Offsets from whichever point is first in the array. Signed 32-bit
 * differences: a 32-bit esp_timer wrap inside the window, and a point that
 * sorting has put ahead of earlier ones, both come out right for any window
 * under 35 minutes. A line fit does not care where its origin is. */
static double x_of(const acq_rate *r, const acq_rate_point *p)
{
    return (double)(int32_t)(p->index - r->points[0].index);
}

static double y_of(const acq_rate *r, const acq_rate_point *p)
{
    return (double)(int32_t)(p->edge_us - r->points[0].edge_us);
}

/* Two-pass least squares over the first n points. */
static bool fit(const acq_rate *r, uint32_t n, line_fit *out)
{
    double mx = 0.0, my = 0.0, sxx = 0.0, sxy = 0.0, ssr = 0.0;
    uint32_t i;
    if (n < 3) {
        return false;
    }
    for (i = 0; i < n; i++) {
        mx += x_of(r, &r->points[i]);
        my += y_of(r, &r->points[i]);
    }
    mx /= n;
    my /= n;
    for (i = 0; i < n; i++) {
        const double dx = x_of(r, &r->points[i]) - mx;
        sxx += dx * dx;
        sxy += dx * (y_of(r, &r->points[i]) - my);
    }
    if (sxx <= 0.0) {
        return false;
    }
    out->slope = sxy / sxx;
    out->intercept = my - out->slope * mx;
    for (i = 0; i < n; i++) {
        const double e = y_of(r, &r->points[i])
                       - (out->intercept + out->slope * x_of(r, &r->points[i]));
        ssr += e * e;
    }
    out->sxx = sxx;
    out->ssr = ssr;
    out->n = n;
    return true;
}

static int by_work(const void *a, const void *b)
{
    const float wa = ((const acq_rate_point *)a)->work;
    const float wb = ((const acq_rate_point *)b)->work;
    return (wa > wb) - (wa < wb);
}

static float median_of_work(acq_rate_point *points, uint32_t n)
{
    qsort(points, n, sizeof(points[0]), by_work);
    return (n % 2) ? points[n / 2].work
                   : 0.5f * (points[n / 2 - 1].work + points[n / 2].work);
}

void acq_rate_init(acq_rate *r, acq_rate_point *storage, uint32_t capacity)
{
    r->points = storage;
    r->capacity = capacity;
    r->count = 0;
    r->skipped = 0;
}

bool acq_rate_add(acq_rate *r, uint32_t index, uint32_t edge_us, uint8_t edges)
{
    if (r->count >= r->capacity) {
        return false;
    }
    if (edges != 1) {
        r->skipped++;
        return true;
    }
    r->points[r->count].index = index;
    r->points[r->count].edge_us = edge_us;
    r->points[r->count].work = 0.0f;
    r->count++;
    return true;
}

bool acq_rate_full(const acq_rate *r)
{
    return r->count >= r->capacity;
}

bool acq_rate_solve(acq_rate *r, acq_rate_estimate *out)
{
    const uint32_t n = r->count;
    line_fit first, final;
    float median, limit;
    double scale, max_us = 0.0;
    uint32_t kept, i;

    if (!fit(r, n, &first)) {
        return false;
    }
    out->span_s = (double)(uint32_t)(r->points[n - 1].edge_us
                                     - r->points[0].edge_us) / 1e6;

    /* Residuals from the first fit, taken before anything is reordered:
     * first's intercept is relative to the current points[0]. */
    for (i = 0; i < n; i++) {
        r->points[i].work = (float)(y_of(r, &r->points[i])
                                    - (first.intercept
                                       + first.slope * x_of(r, &r->points[i])));
    }

    /* Robust scale of the residuals: 1.4826 x median |r - median r|. */
    median = median_of_work(r->points, n);
    for (i = 0; i < n; i++) {
        r->points[i].work = fabsf(r->points[i].work - median);
    }
    scale = MAD_TO_SIGMA * median_of_work(r->points, n);
    if (scale < ACQ_RATE_SCALE_FLOOR_US) {
        scale = ACQ_RATE_SCALE_FLOOR_US;
    }
    limit = (float)(ACQ_RATE_REJECT_SIGMAS * scale);

    /* Sorted by distance from the median residual, so the kept points are a
     * prefix of the array. */
    for (kept = 0; kept < n && r->points[kept].work <= limit; kept++) {
    }
    if (kept < 3) {
        return false;
    }
    if (!fit(r, kept, &final)) {
        return false;
    }
    for (i = 0; i < kept; i++) {
        const double e = fabs(y_of(r, &r->points[i])
                              - (final.intercept + final.slope * x_of(r, &r->points[i])));
        if (e > max_us) {
            max_us = e;
        }
    }

    {
        const double variance = final.ssr / (double)(final.n - 2);
        const double slope_se = sqrt(variance / final.sxx);
        out->hz = 1e6 / final.slope;
        out->hz_se = out->hz * slope_se / final.slope;
        out->rms_us = sqrt(final.ssr / (double)final.n);
    }
    out->max_us = max_us;
    out->accepted = n;
    out->used = kept;
    out->rejected = n - kept;
    out->skipped = r->skipped;
    return true;
}
