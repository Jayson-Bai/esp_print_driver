#include "telemetry_scheduler.h"

#include "extruder_ctrl.h"
#include "fan.h"
#include "motor.h"
#include "pid_ctrl.h"
#include "serial_reader.h"
#include "temperature.h"
#include "uart_tx_dispatch.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/*
 * Deadline-driven telemetry scheduler.
 *
 * Sampling happens at a 50 ms service tick; each stream has its own
 * next-due timestamp so a slow stream does not delay the others.
 * A full ESTAT frame doubles as the hot sample at that slot so hot fields
 * stay at 5 Hz during activity without a duplicate line.
 */

#define TELEM_SERVICE_TICK_MS      50
#define TELEM_FULL_PERIOD_MS     1000
#define TELEM_HOT_PERIOD_MS       200
#define TELEM_USTAT_PERIOD_MS    1000
#define TELEM_STAT_PERIOD_MS     1000

/* Active->idle hysteresis so the 200 ms cadence does not flap at edges. */
#define TELEM_ACTIVE_HOLD_US     500000LL

/* Stale telemetry older than this is dropped by the TX layer instead of sent. */
#define TELEM_DEADLINE_US         80000LL

#define TELEM_FRAME_MAX           512

#define EXTRUDER_TOOL_COUNT        2

extern volatile int g_current_tool_id;

/* Shared sampling/sequence state. */
static uint32_t g_boot_id = 0;
static portMUX_TYPE g_sched_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t g_estat_sample_seq[EXTRUDER_TOOL_COUNT] = {0, 0};
static uint32_t g_ustat_sample_seq = 0;
static int64_t  g_estat_next_full_us[EXTRUDER_TOOL_COUNT] = {0, 0};
static int64_t  g_estat_next_hot_us[EXTRUDER_TOOL_COUNT] = {0, 0};
static int64_t  g_ustat_next_us = 0;
static int64_t  g_stat_next_us = 0;
static int64_t  g_last_active_us[EXTRUDER_TOOL_COUNT] = {0, 0};

static const char *extruder_state_name(extruder_state_t state)
{
    switch (state) {
        case EXTRUDER_STATE_IDLE:        return "IDLE";
        case EXTRUDER_STATE_ENABLE_WAIT: return "ENABLE_WAIT";
        case EXTRUDER_STATE_ACCEL:       return "ACCEL";
        case EXTRUDER_STATE_RUN:         return "RUN";
        case EXTRUDER_STATE_DECEL:       return "DECEL";
        case EXTRUDER_STATE_DIR_CHANGE:  return "DIR_CHANGE";
        case EXTRUDER_STATE_HOLD:        return "HOLD";
        case EXTRUDER_STATE_FAULT:       return "FAULT";
        case EXTRUDER_STATE_RESETTING:   return "RESETTING";
        default:                         return "UNKNOWN";
    }
}

/* Evaluate whether a tool snapshot counts as active. */
static bool tool_is_active(const extruder_status_t *s, int64_t now_us,
                           int tool_id)
{
    bool moving =
        s->state == EXTRUDER_STATE_ENABLE_WAIT ||
        s->state == EXTRUDER_STATE_ACCEL ||
        s->state == EXTRUDER_STATE_RUN ||
        s->state == EXTRUDER_STATE_DECEL ||
        s->state == EXTRUDER_STATE_DIR_CHANGE ||
        s->state == EXTRUDER_STATE_RESETTING;
    bool backlog = s->target_steps != s->submitted_steps ||
                   s->submitted_steps != s->completed_steps ||
                   s->queue_depth > 0;
    bool speed = s->velocity_steps_s > 0.5f || s->velocity_steps_s < -0.5f;

    bool recent;
    portENTER_CRITICAL(&g_sched_mux);
    int64_t last = g_last_active_us[tool_id - 1];
    portEXIT_CRITICAL(&g_sched_mux);
    recent = (now_us - last) < TELEM_ACTIVE_HOLD_US;

    bool active = moving || backlog || speed || recent;
    if (active) {
        portENTER_CRITICAL(&g_sched_mux);
        g_last_active_us[tool_id - 1] = now_us;
        portEXIT_CRITICAL(&g_sched_mux);
    }
    return active;
}

static void submit_telemetry(const char *frame)
{
    size_t len = strlen(frame);
    int64_t deadline = esp_timer_get_time() + TELEM_DEADLINE_US;
    uart_tx_submit_telemetry(frame, len, deadline);
}

static void send_stat_frame(void)
{
    float temp_cf = get_temperature1();
    float temp_resin = get_temperature2();
    float target_cf = pid_get_target(1);
    float target_resin = pid_get_target(2);
    int fan_ok_cf = fan_get_state(FAN1) ? 1 : 0;
    int fan_ok_resin = fan_get_state(FAN2) ? 1 : 0;
    int sw1 = gpio_get_level(STOP_PIN_1);
    int sw2 = gpio_get_level(STOP_PIN_2);
    int tool = (sw1 == 0 && sw2 == 0) ? 0 : (sw1 == 0 ? 1 : (sw2 == 0 ? 2 : 0));
    int err = (sw1 == 0 && sw2 == 0) ? 1 : 0;

    bool last_valid;
    uint32_t last_seq;
    int last_tool;
    int64_t last_total_um;
    int64_t last_us;
    get_last_e_status_public(&last_valid, &last_seq, &last_tool, &last_total_um, &last_us);

    char last_seq_text[16];
    if (last_valid) {
        snprintf(last_seq_text, sizeof(last_seq_text), "%" PRIu32, last_seq);
    } else {
        memcpy(last_seq_text, "-1", 3);
    }

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
                       "STAT temp_cf=%.1f temp_resin=%.1f target_cf=%.1f "
                       "target_resin=%.1f fan_ok_cf=%d fan_ok_resin=%d "
                       "tool=%d err=%d last_e_seq=%s last_e_tool=%d "
                       "last_e_abs=%.6f last_e_us=%" PRId64 "\n",
                       temp_cf, temp_resin, target_cf, target_resin,
                       fan_ok_cf, fan_ok_resin, tool, err,
                       last_seq_text, last_tool,
                       (double)last_total_um / 1000000.0, last_us);
    if (len > 0 && len < (int)sizeof(buf)) {
        submit_telemetry(buf);
    }
}

static void send_estat_frame(int tool, bool hot_only, int64_t sample_us)
{
    extruder_status_t s;
    memset(&s, 0, sizeof(s));
    if (!extruder_get_status(tool, &s)) {
        return;
    }

    portENTER_CRITICAL(&g_sched_mux);
    uint32_t seq = ++g_estat_sample_seq[tool - 1];
    portEXIT_CRITICAL(&g_sched_mux);

    char buf[TELEM_FRAME_MAX];
    int len;
    if (hot_only) {
        len = snprintf(buf, sizeof(buf),
                       "ESTAT_H tool=%d state=%s target_steps=%" PRId64
                       " submitted_steps=%" PRId64 " last_seq=%" PRIu32
                       " ff_sps=%.3f v_sps=%.3f fault=0x%08" PRIx32
                       " boot_id=%" PRIu32 " sample_seq=%" PRIu32
                       " sample_us=%" PRId64 " reset_epoch=%" PRIu32
                       " active=%d\n",
                       tool, extruder_state_name(s.state),
                       s.target_steps, s.submitted_steps, s.last_seq,
                       s.feedforward_steps_s, s.velocity_steps_s,
                       s.fault_flags, g_boot_id, seq, sample_us,
                       s.reset_generation, 1);
    } else {
        len = snprintf(buf, sizeof(buf),
                       "ESTAT tool=%d state=%s target_steps=%" PRId64
                       " submitted_steps=%" PRId64 " completed_steps=%" PRId64
                       " host_total_um=%" PRId64 " last_cmd_us=%" PRId64
                       " ff_until_us=%" PRId64 " ff_sps=%.3f v_sps=%.3f"
                       " phase=%.6f last_seq=%" PRIu32
                       " fault=0x%08" PRIx32 " hard_fault=0x%08" PRIx32
                       " rmt_submit_err=%" PRIu32
                       " rmt_done_err=%" PRIu32 " qdepth=%" PRIu32
                       " dir=%d enabled=%d hold_latched=%d host_valid=%d"
                       " boot_id=%" PRIu32 " sample_seq=%" PRIu32
                       " sample_us=%" PRId64 " reset_epoch=%" PRIu32
                       " active=%d\n",
                       tool, extruder_state_name(s.state),
                       s.target_steps, s.submitted_steps, s.completed_steps,
                       s.host_total_um, s.last_command_us,
                       s.feedforward_valid_until_us,
                       s.feedforward_steps_s, s.velocity_steps_s, s.phase,
                       s.last_seq, s.fault_flags, s.hard_fault_flags,
                       s.rmt_submit_errors, s.rmt_done_errors, s.queue_depth,
                       s.direction_sign, s.enabled ? 1 : 0,
                       s.host_hold_active ? 1 : 0,
                       s.host_sample_valid ? 1 : 0,
                       g_boot_id, seq, sample_us,
                       s.reset_generation,
                       tool_is_active(&s, sample_us, tool) ? 1 : 0);
    }
    if (len > 0 && len < (int)sizeof(buf)) {
        submit_telemetry(buf);
    }
}

static void send_ustat_frame(int64_t sample_us)
{
    /* UART counters live in serial_reader.c; expose a getter. */
    uint32_t fifo, buf, line, cmd, old_seq, gap, perr;
    get_uart_counters(&fifo, &buf, &line, &cmd, &old_seq, &gap, &perr);

    portENTER_CRITICAL(&g_sched_mux);
    uint32_t seq = ++g_ustat_sample_seq;
    portEXIT_CRITICAL(&g_sched_mux);

    char out[160];
    int len = snprintf(out, sizeof(out),
                       "USTAT fifo_ovf=%" PRIu32 " buffer_full=%" PRIu32
                       " line_drop=%" PRIu32 " cmd_drop=%" PRIu32
                       " old_seq=%" PRIu32 " seq_gap=%" PRIu32
                       " parse_err=%" PRIu32 " boot_id=%" PRIu32
                       " sample_seq=%" PRIu32 " sample_us=%" PRId64 "\n",
                       fifo, buf, line, cmd, old_seq, gap, perr,
                       g_boot_id, seq, sample_us);
    if (len > 0 && len < (int)sizeof(out)) {
        submit_telemetry(out);
    }
}

static void telemetry_task(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    /* Stable boot id from a monotonic source; not globally unique but enough
     * to distinguish a reboot within a host session. */
    g_boot_id = (uint32_t)(esp_timer_get_time() & 0x7fffffff) + 1;

    int64_t now = esp_timer_get_time();
    for (int i = 0; i < EXTRUDER_TOOL_COUNT; ++i) {
        g_estat_next_full_us[i] = now;
        g_estat_next_hot_us[i] = now;
        g_last_active_us[i] = 0;
    }
    g_ustat_next_us = now;
    g_stat_next_us = now;

    while (1) {
        now = esp_timer_get_time();

        /* Per-tool full + hot cadence. */
        for (int tool = 1; tool <= EXTRUDER_TOOL_COUNT; ++tool) {
            if (now >= g_estat_next_full_us[tool - 1]) {
                send_estat_frame(tool, false, now);
                g_estat_next_full_us[tool - 1] += TELEM_FULL_PERIOD_MS * 1000LL;
                /* Full frame doubles as the hot sample at this slot. */
                g_estat_next_hot_us[tool - 1] = now + TELEM_HOT_PERIOD_MS * 1000LL;
                continue;
            }
            extruder_status_t s;
            memset(&s, 0, sizeof(s));
            if (extruder_get_status(tool, &s) &&
                tool_is_active(&s, now, tool) &&
                now >= g_estat_next_hot_us[tool - 1]) {
                send_estat_frame(tool, true, now);
                g_estat_next_hot_us[tool - 1] += TELEM_HOT_PERIOD_MS * 1000LL;
            }
        }

        if (now >= g_ustat_next_us) {
            send_ustat_frame(now);
            g_ustat_next_us += TELEM_USTAT_PERIOD_MS * 1000LL;
        }

        if (now >= g_stat_next_us) {
            send_stat_frame();
            g_stat_next_us += TELEM_STAT_PERIOD_MS * 1000LL;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEM_SERVICE_TICK_MS));
    }
}

void telemetry_scheduler_start(void)
{
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096,
                            NULL, 3, NULL, 1);
}
