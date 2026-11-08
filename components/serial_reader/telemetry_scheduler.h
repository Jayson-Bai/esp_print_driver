#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Telemetry scheduler: periodic sampling of STAT/ESTAT/USTAT.
 * Replaces the old single 1 Hz status_task loop with a deadline-based
 * multi-rate scheduler.
 *
 * Rates:
 *   - ESTAT full   : 1000 ms per monitored tool (also counts as the hot sample)
 *   - ESTAT hot    : 200 ms while the tool is active (5 Hz incl. full frame)
 *   - USTAT        : 1000 ms, independent of extrusion state
 *   - STAT         : 1000 ms (UI/ready path, unchanged format)
 *
 * Activity is evaluated per tool from the extruder snapshot, not only
 * state == RUN, so decel/queue-drain/reset still counts as active.
 */

void telemetry_scheduler_start(void);

#ifdef __cplusplus
}
#endif
