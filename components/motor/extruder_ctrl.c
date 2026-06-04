#include "extruder_ctrl.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include <math.h>

#define EXTRUDER_MAX_STEPS 400        // 单片最大步数上限，可调
#define EXTRUDER_SLICE_US 500         // 每片输出的最大时间(us)，可调
#define RMT_RESOLUTION_HZ 5000000     // RMT 计时分辨率(Hz)，可调
#define EXTRUDER_ERR_K 5.0f           // 误差修正增益(1/s)，可调

static float g_last_abs_mm[2] = {0.0f, 0.0f};
static float g_target_steps_total[2] = {0.0f, 0.0f};
static float g_emit_steps_total[2] = {0.0f, 0.0f};
static float g_phase_accum[2] = {0.0f, 0.0f};
static int g_phase_dir[2] = {0, 0};
static float g_ui_abs_mm[2] = {0.0f, 0.0f};
static int g_ui_dir[2] = {0, 0};
static bool g_ui_active[2] = {false, false};
static float g_host_steps_per_s[2] = {0.0f, 0.0f};
static bool g_stop_request[2] = {false, false};
static portMUX_TYPE g_extruder_mux = portMUX_INITIALIZER_UNLOCKED;
static bool g_extruder_task_started = false;
static TaskHandle_t g_extruder_task_handle = NULL;
static rmt_channel_handle_t g_rmt_chan[2] = {NULL, NULL};
static rmt_encoder_handle_t g_rmt_encoder = NULL;

typedef struct {
    uint32_t high_ticks;
    uint32_t low_ticks;
    uint32_t pulses;
} rmt_step_payload_t;

static size_t rmt_step_encode_cb(const void *data, size_t data_size,
                                 size_t symbols_written, size_t symbols_free,
                                 rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    (void)data_size;
    (void)arg;
    const rmt_step_payload_t *payload = (const rmt_step_payload_t *)data;
    if (symbols_written >= payload->pulses) {
        *done = true;
        return 0;
    }

    size_t remaining = payload->pulses - symbols_written;
    size_t count = remaining;
    if (count > symbols_free) {
        count = symbols_free;
    }

    rmt_symbol_word_t symbol = {
        .level0 = 1,
        .duration0 = payload->high_ticks,
        .level1 = 0,
        .duration1 = payload->low_ticks,
    };

    for (size_t i = 0; i < count; i++) {
        symbols[i] = symbol;
    }

    if (symbols_written + count >= payload->pulses) {
        *done = true;
    }

    return count;
}

static void rmt_init_if_needed(int tool_id, int step_pin)
{
    int idx = tool_id - 1;
    if (g_rmt_chan[idx] != NULL) {
        return;
    }

    if (g_rmt_encoder == NULL) {
        rmt_simple_encoder_config_t enc_cfg = {
            .callback = rmt_step_encode_cb,
            .arg = NULL,
            .min_chunk_size = 32,
        };
        rmt_new_simple_encoder(&enc_cfg, &g_rmt_encoder);
    }

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = step_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 2,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
        },
    };
    if (rmt_new_tx_channel(&tx_cfg, &g_rmt_chan[idx]) == ESP_OK) {
        rmt_enable(g_rmt_chan[idx]);
    }
}

static float steps_per_mm_for_tool(int tool_id)
{
    if (tool_id == 1) {
        return (200.0f * (float)motor3_microstep) / (3.14159f * 10.0f);
    }
    return (200.0f * (float)motor4_microstep) / (3.14159f * 10.0f);
}

static uint32_t step_delay_for_tool(int tool_id, bool retract)
{
    if (tool_id == 1) {
        return retract ? motor3_get_retract_step_delay_us() : motor3_get_step_delay_us();
    }
    return retract ? motor4_get_retract_step_delay_us() : motor4_get_step_delay_us();
}

static void pulse_steps(int tool_id, motor_dir_t direction, int steps)
{
    int step_pin = (tool_id == 1) ? STEP_PIN_3 : STEP_PIN_4;
    int dir_pin = (tool_id == 1) ? DIR_PIN_3 : DIR_PIN_4;
    bool retract = (direction == CW);
    uint32_t delay_us = step_delay_for_tool(tool_id, retract);
    int idx = tool_id - 1;

    rmt_init_if_needed(tool_id, step_pin);

    gpio_set_level(dir_pin, (direction == CCW) ? 1 : 0);

    int steps_remaining = steps;
    while (steps_remaining > 0) {
        if (g_stop_request[idx]) {
            break;
        }
        int steps_chunk = steps_remaining;
        if (steps_chunk > EXTRUDER_MAX_STEPS) {
            steps_chunk = EXTRUDER_MAX_STEPS;
        }
        rmt_step_payload_t payload = {
            .high_ticks = delay_us,
            .low_ticks = delay_us,
            .pulses = (uint32_t)steps_chunk,
        };
        rmt_transmit_config_t tx_cfg = {
            .loop_count = 0,
            .flags = {
                .eot_level = 0,
                .queue_nonblocking = 0,
            },
        };
        rmt_transmit(g_rmt_chan[idx], g_rmt_encoder, &payload, sizeof(payload), &tx_cfg);
        rmt_tx_wait_all_done(g_rmt_chan[idx], -1);
        steps_remaining -= steps_chunk;
    }

}

void extruder_set_absolute(int tool_id, float total_mm)
{
    if (tool_id != 1 && tool_id != 2) {
        return;
    }

    int idx = tool_id - 1;

    portENTER_CRITICAL(&g_extruder_mux);
    float delta = total_mm - g_last_abs_mm[idx];
    g_last_abs_mm[idx] = total_mm;
    float delta_steps = delta * steps_per_mm_for_tool(tool_id);
    g_target_steps_total[idx] += delta_steps;
    g_host_steps_per_s[idx] = delta_steps / 0.004f;
    portEXIT_CRITICAL(&g_extruder_mux);

}

void extruder_reset_absolute(int tool_id)
{
    if (tool_id != 1 && tool_id != 2) {
        return;
    }

    int idx = tool_id - 1;
    portENTER_CRITICAL(&g_extruder_mux);
    g_last_abs_mm[idx] = 0.0f;
    g_target_steps_total[idx] = 0.0f;
    g_emit_steps_total[idx] = 0.0f;
    g_phase_accum[idx] = 0.0f;
    g_phase_dir[idx] = 0;
    g_ui_abs_mm[idx] = 0.0f;
    g_host_steps_per_s[idx] = 0.0f;
    g_stop_request[idx] = true;
    portEXIT_CRITICAL(&g_extruder_mux);
}

static void extruder_task(void *pvParameters)
{
    (void)pvParameters;

    motor3_gpio_init();
    motor4_gpio_init();

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t slice_ticks = pdMS_TO_TICKS(EXTRUDER_SLICE_US / 1000);
    if (slice_ticks == 0) {
        slice_ticks = 1;
    }

    while (1) {
        vTaskDelayUntil(&last_wake, slice_ticks);

        for (int tool_id = 1; tool_id <= 2; tool_id++) {
            int idx = tool_id - 1;
            int en_pin = (tool_id == 1) ? EN_PIN_3 : EN_PIN_4;
            int ui_dir_sign = g_ui_dir[idx];

            if (g_ui_active[idx] && ui_dir_sign != 0) {
                float speed = (tool_id == 1)
                    ? (ui_dir_sign > 0 ? motor3_get_speed_mm_per_s() : motor3_get_retract_speed_mm_per_s())
                    : (ui_dir_sign > 0 ? motor4_get_speed_mm_per_s() : motor4_get_retract_speed_mm_per_s());
                if (speed > 0.0f) {
                    float delta_mm = speed * ((float)EXTRUDER_SLICE_US / 1000000.0f) * (float)ui_dir_sign;
                    g_ui_abs_mm[idx] += delta_mm;
                    extruder_set_absolute(tool_id, g_ui_abs_mm[idx]);
                }
            }

            portENTER_CRITICAL(&g_extruder_mux);
            ui_dir_sign = g_ui_dir[idx];
            float target_steps = g_target_steps_total[idx];
            float host_steps_per_s = g_host_steps_per_s[idx];
            portEXIT_CRITICAL(&g_extruder_mux);

            float error = target_steps - g_emit_steps_total[idx];
            float abs_error = fabsf(error);
            if (g_stop_request[idx]) {
                portENTER_CRITICAL(&g_extruder_mux);
                g_stop_request[idx] = false;
                portEXIT_CRITICAL(&g_extruder_mux);
                gpio_set_level(en_pin, 1);
                continue;
            }
            if (abs_error <= 0.0f) {
                gpio_set_level(en_pin, 1);
                continue;
            }

            int dir_sign = (error > 0.0f) ? 1 : -1;
            if (g_phase_dir[idx] != dir_sign) {
                g_phase_dir[idx] = dir_sign;
                g_phase_accum[idx] = 0.0f;
            }

            motor_dir_t dir = (dir_sign > 0) ? CCW : CW;
            float base_steps_per_s;
            if (g_ui_active[idx]) {
                float ui_mm_per_s = (tool_id == 1)
                    ? (dir_sign > 0 ? motor3_get_speed_mm_per_s() : motor3_get_retract_speed_mm_per_s())
                    : (dir_sign > 0 ? motor4_get_speed_mm_per_s() : motor4_get_retract_speed_mm_per_s());
                base_steps_per_s = ui_mm_per_s * steps_per_mm_for_tool(tool_id);
            } else {
                base_steps_per_s = fabsf(host_steps_per_s);
            }
            float steps_per_s = base_steps_per_s + EXTRUDER_ERR_K * abs_error;
            if (steps_per_s <= 0.0f) {
                continue;
            }
            float inc = steps_per_s * ((float)EXTRUDER_SLICE_US / 1000000.0f);
            g_phase_accum[idx] += inc;

            int steps_to_send = (int)g_phase_accum[idx];
            if (steps_to_send > (int)abs_error) {
                steps_to_send = (int)abs_error;
            }
            if (steps_to_send > EXTRUDER_MAX_STEPS) {
                steps_to_send = EXTRUDER_MAX_STEPS;
            }
            g_phase_accum[idx] -= (float)steps_to_send;
            if (steps_to_send <= 0) {
                continue;
            }

            gpio_set_level(en_pin, 0);
            pulse_steps(tool_id, dir, steps_to_send);

            if (dir == CCW) {
                g_emit_steps_total[idx] += (float)steps_to_send;
            } else {
                g_emit_steps_total[idx] -= (float)steps_to_send;
            }

            portENTER_CRITICAL(&g_extruder_mux);
            g_stop_request[idx] = false;
            portEXIT_CRITICAL(&g_extruder_mux);
        }
    }
}

void extruder_task_start(void)
{
    if (g_extruder_task_started) {
        return;
    }
    g_extruder_task_started = true;
    xTaskCreatePinnedToCore(extruder_task, "extruder_rt", 4096, NULL, 6, &g_extruder_task_handle, 0);
}

void extruder_ui_start(int tool_id, int dir_sign)
{
    if (tool_id != 1 && tool_id != 2) {
        return;
    }
    int idx = tool_id - 1;
    portENTER_CRITICAL(&g_extruder_mux);
    g_ui_dir[idx] = (dir_sign >= 0) ? 1 : -1;
    g_ui_active[idx] = true;
    portEXIT_CRITICAL(&g_extruder_mux);
}

void extruder_ui_stop(int tool_id)
{
    if (tool_id != 1 && tool_id != 2) {
        return;
    }
    int idx = tool_id - 1;
    portENTER_CRITICAL(&g_extruder_mux);
    g_ui_active[idx] = false;
    g_ui_dir[idx] = 0;
    portEXIT_CRITICAL(&g_extruder_mux);
}
