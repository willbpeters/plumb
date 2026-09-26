/* The one LVGL configuration for Plumb: host bench and device alike.
 *
 * Only overrides are listed; LVGL's lv_conf_internal.h supplies a default for
 * everything else. Kept short on purpose, so every line here is a decision.
 *
 * LV_COLOR_FORMAT_DEFAULT, not LV_COLOR_DEPTH: 9.6 deprecates the latter, and
 * its fallback path hits a #warning that MSVC's C compiler treats as a fatal
 * error (C1021). Measured, 2026-09-26.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

/* The GC9A01 takes RGB565 (parent spec 6.3). */
#define LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565

/* No OS inside the UI: the app task owns scheduling, and invariant 8 is
 * enforced there by not calling lv_timer_handler() during a stroke. */
#define LV_USE_OS LV_OS_NONE

/* LVGL's own heap, for objects and draw tasks. Not the draw buffers, which the
 * caller supplies (internal SRAM on the device, invariant 7). */
#define LV_MEM_SIZE (96 * 1024)

/* Parent spec 6.3: display refresh period 10 ms. Also the animation tick, so
 * the host bench resolves motion to 10 ms. */
#define LV_DEF_REFR_PERIOD 10

#define LV_USE_LOG 0
#define LV_BUILD_EXAMPLES 0
#define LV_BUILD_DEMOS 0

#endif /* LV_CONF_H */
