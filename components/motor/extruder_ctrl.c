#include "extruder_ctrl.h"
#include "motor.h"

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*
 * 第一阶段挤出运动规划器。
 *
 * UART 下发的是绝对挤出位置。target_steps 表示当前目标位置，
 * submitted_steps 表示已经提交给异步 RMT 队列的脉冲终点，
 * completed_steps 表示 RMT 已实际发送完成的脉冲终点。
 * 三套计数刻意分离，避免丢包或队列延迟演变为无上限的追赶突发，
 * 同时保证复位只在在途脉冲全部结束后才成为真正的同步屏障。
 */
#define EXTRUDER_TOOL_COUNT                 2
/* MCU 规划周期及上位机 UART 前馈采样、保持时序。 */
#define EXTRUDER_CONTROL_PERIOD_MS          2
#define EXTRUDER_HOST_PERIOD_US             4000LL
#define EXTRUDER_FEEDFORWARD_MIN_DT_US      1000LL
#define EXTRUDER_FEEDFORWARD_MAX_ANCHOR_US  30000000LL
#define EXTRUDER_FEEDFORWARD_MAX_REFRESH_US 250000LL
#define EXTRUDER_FEEDFORWARD_MIN_HOLD_US    12000LL
#define EXTRUDER_FEEDFORWARD_MAX_HOLD_US    100000LL
/*
 * 驱动器使能、空闲释放、方向建立和 STEP 高电平时序。
 * STEP 高电平固定为 4 us，低电平随规划步频自适应；STEP 占空比不负责设置
 * TMC2209 的线圈电流或扭矩，实际电流仍由驱动板 VREF/IRUN 等硬件配置决定。
 */
#define EXTRUDER_ENABLE_SETUP_US            2000LL
#define EXTRUDER_ENABLE_IDLE_TIMEOUT_US      500000LL
#define EXTRUDER_DIR_SETUP_US               5U
#define EXTRUDER_STEP_HIGH_US               4U
/* RMT 时基、异步队列容量及单批脉冲的最长占用时间。 */
#define EXTRUDER_RMT_RESOLUTION_HZ          1000000U
#define EXTRUDER_RMT_QUEUE_DEPTH            4U
#define EXTRUDER_RMT_MEM_SYMBOLS            64U
#define EXTRUDER_RMT_MAX_DURATION_TICKS     32767U
#define EXTRUDER_MAX_BATCH_US               4000U
/* MCU 内部速度规划、加减速和有限位置追赶参数。 */
#define EXTRUDER_MAX_STEP_RATE_SPS          1800.0f
#define EXTRUDER_MAX_ACCEL_SPS2             5000.0f
#define EXTRUDER_POSITION_K                 5.0f
#define EXTRUDER_CATCHUP_MAX_SPS            270.0f
#define EXTRUDER_SPEED_EPSILON_SPS          0.5f
#define EXTRUDER_MAX_DT_S                   0.010f
#define EXTRUDER_MAX_ABS_UM                 1000000000000LL
#define EXTRUDER_MAX_LAG_STEPS              500LL
#define EXTRUDER_LAG_FAULT_DELAY_US         250000LL
/* 送料轮有效直径；实物直径或传动比不同时必须重新标定，否则只影响挤出量换算。 */
#define EXTRUDER_ROLLER_DIAMETER_MM         10.0
#define EXTRUDER_PI                         3.14159265358979323846
#define EXTRUDER_HARD_FAULT_MASK            (EXTRUDER_FAULT_RMT_SUBMIT | \
                                              EXTRUDER_FAULT_RESET_TIMEOUT | \
                                              EXTRUDER_FAULT_RMT_DONE)

static const char *TAG = "extruder";

typedef struct {
    uint32_t high_ticks;
    uint32_t low_ticks;
    uint32_t pulses;
} rmt_step_payload_t;

typedef struct {
    rmt_step_payload_t payload;
    int64_t signed_steps;
    bool in_use;
} rmt_step_slot_t;

typedef struct {
    int tool_id;
    int step_pin;
    int dir_pin;
    int en_pin;

    /* 每个挤出通道独立持有 RMT 通道和编码器。 */
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    bool initialized;

    /* 目标、已提交和已完成计数共同组成异步脉冲账本。 */
    int64_t target_steps;
    int64_t submitted_steps;
    int64_t completed_steps;
    /* UART 绝对位置及前馈锚点以微米计，各类控制时刻以微秒计。 */
    int64_t host_total_um;
    int64_t last_command_us;
    int64_t feedforward_valid_until_us;
    int64_t feedforward_anchor_total_q;
    int64_t feedforward_anchor_rx_us;
    int64_t enable_ready_us;
    int64_t last_active_us;
    int64_t lag_exceeded_since_us;

    /* UART 序号和相邻不同 E 值共同估算前馈速度。 */
    uint32_t last_seq;
    uint32_t feedforward_anchor_seq;
    bool host_sample_valid;
    bool feedforward_anchor_valid;
    bool host_hold_active;
    float feedforward_steps_s;

    /* MCU 基础运动状态：速度、DDA 相位、当前/待切换方向及反转制动。 */
    float speed_sps;
    float phase;
    int direction_sign;
    int pending_direction;
    bool reverse_braking;
    bool enabled;
    extruder_state_t state;

    bool ui_active;
    int ui_direction;
    double ui_target_steps_exact;

    bool reset_pending;
    uint32_t reset_generation;
    uint32_t fault_flags;
    uint32_t rmt_submit_errors;
    uint32_t rmt_done_errors;

    /* RMT 异步事务环形队列；每个槽记录该批脉冲对应的有符号步数。 */
    rmt_step_slot_t slots[EXTRUDER_RMT_QUEUE_DEPTH];
    uint32_t slot_head;
    uint32_t slot_tail;
    uint32_t queued_count;
} extruder_runtime_t;

typedef struct {
    int64_t target_steps;
    int64_t submitted_steps;
    int64_t completed_steps;
    int64_t last_command_us;
    int64_t feedforward_valid_until_us;
    int64_t enable_ready_us;
    int64_t last_active_us;
    int64_t lag_exceeded_since_us;
    float feedforward_steps_s;
    float speed_sps;
    float phase;
    int direction_sign;
    int pending_direction;
    bool reverse_braking;
    bool enabled;
    bool ui_active;
    bool host_hold_active;
    bool host_sample_valid;
    bool reset_pending;
    uint32_t fault_flags;
    uint32_t queued_count;
} planner_snapshot_t;

static extruder_runtime_t g_tools[EXTRUDER_TOOL_COUNT] = {
    {
        .tool_id = 1,
        .step_pin = STEP_PIN_3,
        .dir_pin = DIR_PIN_3,
        .en_pin = EN_PIN_3,
        .state = EXTRUDER_STATE_IDLE,
    },
    {
        .tool_id = 2,
        .step_pin = STEP_PIN_4,
        .dir_pin = DIR_PIN_4,
        .en_pin = EN_PIN_4,
        .state = EXTRUDER_STATE_IDLE,
    },
};

static portMUX_TYPE g_extruder_mux = portMUX_INITIALIZER_UNLOCKED;
static bool g_extruder_task_started;
static TaskHandle_t g_extruder_task_handle;

static extruder_runtime_t *runtime_for_tool(int tool_id)
{
    if (tool_id < 1 || tool_id > EXTRUDER_TOOL_COUNT) {
        return NULL;
    }
    return &g_tools[tool_id - 1];
}

static double steps_per_mm_for_tool(int tool_id)
{
    int microsteps = (tool_id == 1) ? motor3_microstep : motor4_microstep;
    return (200.0 * (double)microsteps) /
           (EXTRUDER_PI * EXTRUDER_ROLLER_DIAMETER_MM);
}

static int64_t absolute_um_to_steps(int tool_id, int64_t total_um)
{
    double steps = ((double)total_um * steps_per_mm_for_tool(tool_id)) / 1000000.0;
    return (int64_t)llround(steps);
}

static int sign_i64(int64_t value)
{
    return (value > 0) - (value < 0);
}

static int sign_float(float value)
{
    if (value > EXTRUDER_SPEED_EPSILON_SPS) {
        return 1;
    }
    if (value < -EXTRUDER_SPEED_EPSILON_SPS) {
        return -1;
    }
    return 0;
}

static int64_t abs_i64(int64_t value)
{
    return (value >= 0) ? value : -value;
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool has_hard_fault(uint32_t fault_flags)
{
    return (fault_flags & EXTRUDER_HARD_FAULT_MASK) != 0;
}

static int64_t clamp_i64(int64_t value, int64_t minimum, int64_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int64_t estimate_feedforward_dt_us(uint32_t seq, uint32_t previous_seq,
                                          int64_t rx_us,
                                          int64_t previous_rx_us,
                                          int64_t maximum_dt_us)
{
    int64_t actual_dt_us = rx_us - previous_rx_us;
    if (actual_dt_us > maximum_dt_us) {
        return 0;
    }

    uint32_t seq_delta = seq - previous_seq;
    if (seq_delta > 0 && seq_delta <= UINT32_MAX / 2U &&
        seq_delta <= (uint32_t)(maximum_dt_us /
                                EXTRUDER_HOST_PERIOD_US)) {
        int64_t seq_dt_us = (int64_t)seq_delta * EXTRUDER_HOST_PERIOD_US;
        if (seq_dt_us >= EXTRUDER_FEEDFORWARD_MIN_DT_US) {
            return seq_dt_us;
        }
    }

    if (actual_dt_us >= EXTRUDER_FEEDFORWARD_MIN_DT_US &&
        actual_dt_us <= maximum_dt_us) {
        return actual_dt_us;
    }
    return 0;
}

static float move_towards(float current, float target, float maximum_delta)
{
    if (current < target) {
        current += maximum_delta;
        return (current > target) ? target : current;
    }
    if (current > target) {
        current -= maximum_delta;
        return (current < target) ? target : current;
    }
    return current;
}

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
    size_t count = (remaining < symbols_free) ? remaining : symbols_free;
    rmt_symbol_word_t symbol = {
        .level0 = 1,
        .duration0 = payload->high_ticks,
        .level1 = 0,
        .duration1 = payload->low_ticks,
    };

    for (size_t i = 0; i < count; ++i) {
        symbols[i] = symbol;
    }
    *done = (symbols_written + count >= payload->pulses);
    return count;
}

static bool IRAM_ATTR rmt_trans_done_cb(rmt_channel_handle_t channel,
                                        const rmt_tx_done_event_data_t *edata,
                                        void *user_ctx)
{
    (void)channel;
    extruder_runtime_t *runtime = (extruder_runtime_t *)user_ctx;

    portENTER_CRITICAL_ISR(&g_extruder_mux);
    if (runtime->queued_count == 0) {
        runtime->rmt_done_errors++;
        runtime->fault_flags |= EXTRUDER_FAULT_RMT_DONE;
        runtime->state = EXTRUDER_STATE_FAULT;
        portEXIT_CRITICAL_ISR(&g_extruder_mux);
        return false;
    }

    rmt_step_slot_t *slot = &runtime->slots[runtime->slot_head];
    size_t expected_symbols = (size_t)slot->payload.pulses + 1U;
    if (!slot->in_use || edata == NULL || edata->num_symbols != expected_symbols) {
        runtime->rmt_done_errors++;
        runtime->fault_flags |= EXTRUDER_FAULT_RMT_DONE;
        runtime->state = EXTRUDER_STATE_FAULT;
    }

    /*
     * 收到事务完成回调，表示预留的整批脉冲已经由 RMT 外设发送完毕。
     * 即使符号数量诊断不一致，也必须结算该槽位对应的 completed_steps；
     * 同时锁存的硬故障会阻止后续规划，避免同一批脉冲被重复记账。
     */
    runtime->completed_steps += slot->signed_steps;
    slot->in_use = false;
    runtime->slot_head = (runtime->slot_head + 1U) % EXTRUDER_RMT_QUEUE_DEPTH;
    runtime->queued_count--;
    portEXIT_CRITICAL_ISR(&g_extruder_mux);
    return false;
}

static bool init_rmt(extruder_runtime_t *runtime)
{
    rmt_simple_encoder_config_t encoder_config = {
        .callback = rmt_step_encode_cb,
        .arg = runtime,
        .min_chunk_size = 32,
    };
    esp_err_t err = rmt_new_simple_encoder(&encoder_config, &runtime->encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tool %d encoder init failed: %s",
                 runtime->tool_id, esp_err_to_name(err));
        return false;
    }

    rmt_tx_channel_config_t channel_config = {
        .gpio_num = runtime->step_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXTRUDER_RMT_RESOLUTION_HZ,
        .mem_block_symbols = EXTRUDER_RMT_MEM_SYMBOLS,
        .trans_queue_depth = EXTRUDER_RMT_QUEUE_DEPTH,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
        },
    };
    err = rmt_new_tx_channel(&channel_config, &runtime->channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tool %d RMT channel init failed: %s",
                 runtime->tool_id, esp_err_to_name(err));
        return false;
    }

    rmt_tx_event_callbacks_t callbacks = {
        .on_trans_done = rmt_trans_done_cb,
    };
    err = rmt_tx_register_event_callbacks(runtime->channel, &callbacks, runtime);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tool %d RMT callback init failed: %s",
                 runtime->tool_id, esp_err_to_name(err));
        return false;
    }

    err = rmt_enable(runtime->channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tool %d RMT enable failed: %s",
                 runtime->tool_id, esp_err_to_name(err));
        return false;
    }
    return true;
}

static void snapshot_runtime(extruder_runtime_t *runtime, planner_snapshot_t *snapshot)
{
    portENTER_CRITICAL(&g_extruder_mux);
    snapshot->target_steps = runtime->target_steps;
    snapshot->submitted_steps = runtime->submitted_steps;
    snapshot->completed_steps = runtime->completed_steps;
    snapshot->last_command_us = runtime->last_command_us;
    snapshot->feedforward_valid_until_us = runtime->feedforward_valid_until_us;
    snapshot->enable_ready_us = runtime->enable_ready_us;
    snapshot->last_active_us = runtime->last_active_us;
    snapshot->lag_exceeded_since_us = runtime->lag_exceeded_since_us;
    snapshot->feedforward_steps_s = runtime->feedforward_steps_s;
    snapshot->speed_sps = runtime->speed_sps;
    snapshot->phase = runtime->phase;
    snapshot->direction_sign = runtime->direction_sign;
    snapshot->pending_direction = runtime->pending_direction;
    snapshot->reverse_braking = runtime->reverse_braking;
    snapshot->enabled = runtime->enabled;
    snapshot->ui_active = runtime->ui_active;
    snapshot->host_hold_active = runtime->host_hold_active;
    snapshot->host_sample_valid = runtime->host_sample_valid;
    snapshot->reset_pending = runtime->reset_pending;
    snapshot->fault_flags = runtime->fault_flags;
    snapshot->queued_count = runtime->queued_count;
    portEXIT_CRITICAL(&g_extruder_mux);
}

static void set_motion_state(extruder_runtime_t *runtime, float speed_sps,
                             float phase, extruder_state_t state)
{
    if (phase < 0.0f || phase >= 1.0f) {
        phase -= floorf(phase);
        if (phase < 0.0f) {
            phase = 0.0f;
        }
    }
    portENTER_CRITICAL(&g_extruder_mux);
    runtime->speed_sps = clamp_float(speed_sps, 0.0f, EXTRUDER_MAX_STEP_RATE_SPS);
    runtime->phase = phase;
    if (runtime->reset_pending) {
        runtime->state = EXTRUDER_STATE_RESETTING;
    } else if (has_hard_fault(runtime->fault_flags)) {
        runtime->state = EXTRUDER_STATE_FAULT;
    } else {
        runtime->state = state;
    }
    portEXIT_CRITICAL(&g_extruder_mux);
}

static void set_enabled(extruder_runtime_t *runtime, bool enabled, int64_t now_us)
{
    bool changed;
    portENTER_CRITICAL(&g_extruder_mux);
    changed = (runtime->enabled != enabled);
    portEXIT_CRITICAL(&g_extruder_mux);
    if (!changed) {
        return;
    }

    gpio_set_level(runtime->en_pin, enabled ? 0 : 1);
    portENTER_CRITICAL(&g_extruder_mux);
    runtime->enabled = enabled;
    runtime->enable_ready_us = enabled ? now_us + EXTRUDER_ENABLE_SETUP_US : 0;
    portEXIT_CRITICAL(&g_extruder_mux);
}

static bool change_direction_if_safe(extruder_runtime_t *runtime, int direction)
{
    bool safe;
    portENTER_CRITICAL(&g_extruder_mux);
    safe = direction != 0 &&
           runtime->queued_count == 0 &&
           runtime->submitted_steps == runtime->completed_steps &&
           !runtime->reset_pending &&
           !has_hard_fault(runtime->fault_flags);
    portEXIT_CRITICAL(&g_extruder_mux);
    if (!safe) {
        return false;
    }

    gpio_set_level(runtime->dir_pin, (direction > 0) ? 1 : 0);
    esp_rom_delay_us(EXTRUDER_DIR_SETUP_US);

    portENTER_CRITICAL(&g_extruder_mux);
    runtime->direction_sign = direction;
    runtime->pending_direction = 0;
    runtime->reverse_braking = false;
    runtime->phase = 0.0f;
    runtime->state = EXTRUDER_STATE_DIR_CHANGE;
    portEXIT_CRITICAL(&g_extruder_mux);
    return true;
}

static uint32_t pulse_period_us(float speed_sps)
{
    if (speed_sps < 1.0f) {
        speed_sps = 1.0f;
    }
    /*
     * DDA 负责保证长期累计步数。批内周期向下取整而不是向上取整，
     * 因此量化误差最多只会在下一规划边界形成极短的低电平空隙，
     * 不会让每一批都逐渐延后并最终填满异步 RMT 队列。
     */
    uint32_t period_us = (uint32_t)floorf(1000000.0f / speed_sps);
    uint32_t minimum_period = EXTRUDER_STEP_HIGH_US + 1U;
    uint32_t maximum_period = EXTRUDER_STEP_HIGH_US +
                              EXTRUDER_RMT_MAX_DURATION_TICKS;
    if (period_us < minimum_period) {
        period_us = minimum_period;
    }
    if (period_us > maximum_period) {
        period_us = maximum_period;
    }
    return period_us;
}

static uint32_t limit_batch_steps(uint32_t requested_steps, float speed_sps)
{
    uint32_t period_us = pulse_period_us(speed_sps);
    uint32_t maximum_steps = EXTRUDER_MAX_BATCH_US / period_us;
    if (maximum_steps == 0) {
        maximum_steps = 1;
    }
    return (requested_steps < maximum_steps) ? requested_steps : maximum_steps;
}

static bool submit_steps(extruder_runtime_t *runtime, int direction,
                         uint32_t requested_steps, float pulse_speed_sps)
{
    if (direction == 0 || requested_steps == 0) {
        return false;
    }

    requested_steps = limit_batch_steps(requested_steps, pulse_speed_sps);
    uint32_t period_us = pulse_period_us(pulse_speed_sps);
    uint32_t slot_index;
    int64_t signed_steps = (int64_t)direction * (int64_t)requested_steps;

    portENTER_CRITICAL(&g_extruder_mux);
    if (runtime->queued_count >= EXTRUDER_RMT_QUEUE_DEPTH ||
        runtime->reset_pending ||
        has_hard_fault(runtime->fault_flags) ||
        !runtime->enabled) {
        portEXIT_CRITICAL(&g_extruder_mux);
        return false;
    }

    slot_index = runtime->slot_tail;
    rmt_step_slot_t *slot = &runtime->slots[slot_index];
    if (slot->in_use) {
        runtime->rmt_submit_errors++;
        runtime->fault_flags |= EXTRUDER_FAULT_RMT_SUBMIT;
        runtime->state = EXTRUDER_STATE_FAULT;
        portEXIT_CRITICAL(&g_extruder_mux);
        return false;
    }

    slot->payload.high_ticks = EXTRUDER_STEP_HIGH_US;
    slot->payload.low_ticks = period_us - EXTRUDER_STEP_HIGH_US;
    slot->payload.pulses = requested_steps;
    slot->signed_steps = signed_steps;
    slot->in_use = true;
    runtime->slot_tail = (runtime->slot_tail + 1U) % EXTRUDER_RMT_QUEUE_DEPTH;
    runtime->queued_count++;
    runtime->submitted_steps += signed_steps;
    portEXIT_CRITICAL(&g_extruder_mux);

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 1,
        },
    };
    esp_err_t err = rmt_transmit(runtime->channel, runtime->encoder,
                                 &runtime->slots[slot_index].payload,
                                 sizeof(runtime->slots[slot_index].payload),
                                 &transmit_config);
    if (err == ESP_OK) {
        return true;
    }

    /*
     * RMT 拒绝事务后不会产生完成回调。规划器是该队列唯一的生产者，
     * 因此即使此时回调已经释放了更早的队首槽位，仍可安全回滚刚预留的
     * 队尾槽位及 submitted_steps，避免提交失败的脉冲留在账本中。
     */
    portENTER_CRITICAL(&g_extruder_mux);
    rmt_step_slot_t *rollback_slot = &runtime->slots[slot_index];
    if (rollback_slot->in_use) {
        rollback_slot->in_use = false;
        runtime->slot_tail = slot_index;
        runtime->queued_count--;
        runtime->submitted_steps -= signed_steps;
    }
    runtime->rmt_submit_errors++;
    runtime->fault_flags |= EXTRUDER_FAULT_RMT_SUBMIT;
    runtime->state = EXTRUDER_STATE_FAULT;
    portEXIT_CRITICAL(&g_extruder_mux);
    ESP_LOGE(TAG, "tool %d RMT submit failed: %s",
             runtime->tool_id, esp_err_to_name(err));
    return false;
}

static void update_ui_target(extruder_runtime_t *runtime, float elapsed_s,
                             int64_t now_us)
{
    int ui_direction;
    bool ui_active;
    portENTER_CRITICAL(&g_extruder_mux);
    ui_direction = runtime->ui_direction;
    ui_active = runtime->ui_active;
    portEXIT_CRITICAL(&g_extruder_mux);
    if (!ui_active || ui_direction == 0) {
        return;
    }

    float speed_mm_s;
    if (runtime->tool_id == 1) {
        speed_mm_s = (ui_direction > 0) ? motor3_get_speed_mm_per_s()
                                        : motor3_get_retract_speed_mm_per_s();
    } else {
        speed_mm_s = (ui_direction > 0) ? motor4_get_speed_mm_per_s()
                                        : motor4_get_retract_speed_mm_per_s();
    }
    if (!isfinite(speed_mm_s) || speed_mm_s < 0.0f) {
        speed_mm_s = 0.0f;
    }

    float signed_speed_sps = (float)(speed_mm_s *
                              steps_per_mm_for_tool(runtime->tool_id)) *
                              (float)ui_direction;
    signed_speed_sps = clamp_float(signed_speed_sps,
                                  -EXTRUDER_MAX_STEP_RATE_SPS,
                                  EXTRUDER_MAX_STEP_RATE_SPS);

    portENTER_CRITICAL(&g_extruder_mux);
    if (runtime->ui_active && runtime->ui_direction == ui_direction) {
        runtime->ui_target_steps_exact += (double)signed_speed_sps * elapsed_s;
        runtime->target_steps = (int64_t)llround(runtime->ui_target_steps_exact);
        runtime->feedforward_steps_s = signed_speed_sps;
        runtime->last_command_us = now_us;
    }
    portEXIT_CRITICAL(&g_extruder_mux);
}

static void complete_reset_if_drained(extruder_runtime_t *runtime, int64_t now_us)
{
    bool drained;
    portENTER_CRITICAL(&g_extruder_mux);
    drained = runtime->reset_pending &&
              runtime->queued_count == 0 &&
              runtime->submitted_steps == runtime->completed_steps;
    portEXIT_CRITICAL(&g_extruder_mux);
    if (!drained) {
        return;
    }

    if (runtime->channel != NULL &&
        rmt_tx_wait_all_done(runtime->channel, 0) != ESP_OK) {
        return;
    }

    set_enabled(runtime, false, now_us);
    gpio_set_level(runtime->dir_pin, 0);

    portENTER_CRITICAL(&g_extruder_mux);
    if (runtime->reset_pending &&
        runtime->queued_count == 0 &&
        runtime->submitted_steps == runtime->completed_steps) {
        runtime->target_steps = 0;
        runtime->submitted_steps = 0;
        runtime->completed_steps = 0;
        runtime->host_total_um = 0;
        runtime->last_command_us = 0;
        runtime->feedforward_valid_until_us = 0;
        runtime->feedforward_anchor_total_q = 0;
        runtime->feedforward_anchor_rx_us = 0;
        runtime->last_seq = 0;
        runtime->feedforward_anchor_seq = 0;
        runtime->host_sample_valid = false;
        runtime->feedforward_anchor_valid = false;
        runtime->host_hold_active = false;
        runtime->feedforward_steps_s = 0.0f;
        runtime->speed_sps = 0.0f;
        runtime->phase = 0.0f;
        runtime->direction_sign = 0;
        runtime->pending_direction = 0;
        runtime->reverse_braking = false;
        runtime->ui_active = false;
        runtime->ui_direction = 0;
        runtime->ui_target_steps_exact = 0.0;
        runtime->lag_exceeded_since_us = 0;
        runtime->last_active_us = now_us;
        runtime->fault_flags = EXTRUDER_FAULT_NONE;
        runtime->reset_pending = false;
        runtime->reset_generation++;
        runtime->state = EXTRUDER_STATE_IDLE;
        memset(runtime->slots, 0, sizeof(runtime->slots));
        runtime->slot_head = 0;
        runtime->slot_tail = 0;
    }
    portEXIT_CRITICAL(&g_extruder_mux);
}

static void process_reset(extruder_runtime_t *runtime, int64_t now_us)
{
    portENTER_CRITICAL(&g_extruder_mux);
    runtime->speed_sps = 0.0f;
    runtime->phase = 0.0f;
    runtime->reverse_braking = false;
    runtime->pending_direction = 0;
    runtime->state = EXTRUDER_STATE_RESETTING;
    portEXIT_CRITICAL(&g_extruder_mux);
    complete_reset_if_drained(runtime, now_us);
}

static void process_fault(extruder_runtime_t *runtime,
                          const planner_snapshot_t *snapshot,
                          float elapsed_s, int64_t now_us)
{
    portENTER_CRITICAL(&g_extruder_mux);
    runtime->host_hold_active = false;
    portEXIT_CRITICAL(&g_extruder_mux);

    float speed = move_towards(snapshot->speed_sps, 0.0f,
                               EXTRUDER_MAX_ACCEL_SPS2 * elapsed_s);
    set_motion_state(runtime, speed, 0.0f, EXTRUDER_STATE_FAULT);

    if (snapshot->queued_count == 0 &&
        now_us - snapshot->last_active_us >= EXTRUDER_ENABLE_IDLE_TIMEOUT_US) {
        set_enabled(runtime, false, now_us);
    }
}

static void update_lag_diagnostic(extruder_runtime_t *runtime,
                                  int64_t planning_error, int64_t now_us)
{
    portENTER_CRITICAL(&g_extruder_mux);
    if (abs_i64(planning_error) > EXTRUDER_MAX_LAG_STEPS) {
        if (runtime->lag_exceeded_since_us == 0) {
            runtime->lag_exceeded_since_us = now_us;
        } else if (now_us - runtime->lag_exceeded_since_us >=
                   EXTRUDER_LAG_FAULT_DELAY_US) {
            runtime->fault_flags |= EXTRUDER_FAULT_LAG_LIMIT;
        }
    } else {
        runtime->lag_exceeded_since_us = 0;
    }
    portEXIT_CRITICAL(&g_extruder_mux);
}

static void process_reverse_brake(extruder_runtime_t *runtime,
                                  const planner_snapshot_t *snapshot,
                                  float elapsed_s, int64_t now_us)
{
    if (snapshot->direction_sign == 0) {
        portENTER_CRITICAL(&g_extruder_mux);
        runtime->reverse_braking = false;
        portEXIT_CRITICAL(&g_extruder_mux);
        return;
    }

    if (!snapshot->enabled) {
        set_enabled(runtime, true, now_us);
        set_motion_state(runtime, 0.0f, 0.0f, EXTRUDER_STATE_ENABLE_WAIT);
        return;
    }
    if (now_us < snapshot->enable_ready_us) {
        set_motion_state(runtime, 0.0f, 0.0f, EXTRUDER_STATE_ENABLE_WAIT);
        return;
    }

    float old_speed = snapshot->speed_sps;
    float new_speed = move_towards(old_speed, 0.0f,
                                   EXTRUDER_MAX_ACCEL_SPS2 * elapsed_s);
    float phase = snapshot->phase +
                  0.5f * (old_speed + new_speed) * elapsed_s;
    uint32_t due_steps = (uint32_t)floorf(phase);
    phase -= (float)due_steps;

    set_motion_state(runtime, new_speed, phase, EXTRUDER_STATE_DECEL);
    if (due_steps > 0) {
        float pulse_speed = fmaxf(0.5f * (old_speed + new_speed), 1.0f);
        submit_steps(runtime, snapshot->direction_sign, due_steps, pulse_speed);
    }

    if (new_speed <= EXTRUDER_SPEED_EPSILON_SPS) {
        bool drained;
        portENTER_CRITICAL(&g_extruder_mux);
        runtime->speed_sps = 0.0f;
        runtime->phase = 0.0f;
        drained = runtime->queued_count == 0 &&
                  runtime->submitted_steps == runtime->completed_steps;
        portEXIT_CRITICAL(&g_extruder_mux);
        if (drained) {
            change_direction_if_safe(runtime, snapshot->pending_direction);
        }
    }
}

static void planner_step(extruder_runtime_t *runtime, float elapsed_s,
                         int64_t now_us)
{
    update_ui_target(runtime, elapsed_s, now_us);

    planner_snapshot_t snapshot;
    snapshot_runtime(runtime, &snapshot);

    if (snapshot.reset_pending) {
        process_reset(runtime, now_us);
        return;
    }
    if (has_hard_fault(snapshot.fault_flags)) {
        process_fault(runtime, &snapshot, elapsed_s, now_us);
        return;
    }

    int64_t planning_error = snapshot.target_steps - snapshot.submitted_steps;
    update_lag_diagnostic(runtime, planning_error, now_us);

    bool host_fresh = snapshot.host_sample_valid &&
                      now_us <= snapshot.feedforward_valid_until_us;
    bool session_alive = snapshot.ui_active || snapshot.host_hold_active;

    if (session_alive || planning_error != 0 || snapshot.queued_count > 0 ||
        snapshot.speed_sps > EXTRUDER_SPEED_EPSILON_SPS ||
        snapshot.reverse_braking) {
        portENTER_CRITICAL(&g_extruder_mux);
        runtime->last_active_us = now_us;
        portEXIT_CRITICAL(&g_extruder_mux);
    }

    float feedforward = (snapshot.ui_active || host_fresh)
                      ? snapshot.feedforward_steps_s : 0.0f;
    float correction = clamp_float((float)planning_error * EXTRUDER_POSITION_K,
                                   -EXTRUDER_CATCHUP_MAX_SPS,
                                   EXTRUDER_CATCHUP_MAX_SPS);
    float desired_signed_speed = feedforward + correction;

    if (fabsf(feedforward) > EXTRUDER_SPEED_EPSILON_SPS &&
        desired_signed_speed * feedforward < 0.0f) {
        desired_signed_speed = 0.0f;
    }

    if (fabsf(feedforward) <= EXTRUDER_SPEED_EPSILON_SPS) {
        float stop_limit = sqrtf(2.0f * EXTRUDER_MAX_ACCEL_SPS2 *
                                 (float)abs_i64(planning_error));
        desired_signed_speed = clamp_float(desired_signed_speed,
                                           -stop_limit, stop_limit);
    }
    desired_signed_speed = clamp_float(desired_signed_speed,
                                       -EXTRUDER_MAX_STEP_RATE_SPS,
                                       EXTRUDER_MAX_STEP_RATE_SPS);
    int desired_direction = sign_float(desired_signed_speed);

    /*
     * 绝对目标尚未跨过一个完整步进时，量化后的前馈速度可能已经变号。
     * 只有该方向存在可执行的位置误差时才允许物理反转，避免因亚步进
     * 数值抖动让电机在正反方向之间来回切换。
     */
    int error_direction = sign_i64(planning_error);
    if (desired_direction != 0 &&
        ((error_direction != 0 && desired_direction != error_direction) ||
         (error_direction == 0 && snapshot.direction_sign != 0 &&
          desired_direction != snapshot.direction_sign))) {
        desired_signed_speed = 0.0f;
        desired_direction = 0;
    }

    if (snapshot.reverse_braking) {
        if (desired_direction == snapshot.direction_sign) {
            portENTER_CRITICAL(&g_extruder_mux);
            runtime->reverse_braking = false;
            runtime->pending_direction = 0;
            portEXIT_CRITICAL(&g_extruder_mux);
        } else {
            process_reverse_brake(runtime, &snapshot, elapsed_s, now_us);
            return;
        }
    }

    if (snapshot.direction_sign != 0 && desired_direction != 0 &&
        desired_direction != snapshot.direction_sign) {
        portENTER_CRITICAL(&g_extruder_mux);
        runtime->reverse_braking = true;
        runtime->pending_direction = desired_direction;
        portEXIT_CRITICAL(&g_extruder_mux);
        process_reverse_brake(runtime, &snapshot, elapsed_s, now_us);
        return;
    }

    bool need_enable = session_alive || planning_error != 0 ||
                       snapshot.speed_sps > EXTRUDER_SPEED_EPSILON_SPS ||
                       snapshot.queued_count > 0;
    if (need_enable && !snapshot.enabled) {
        set_enabled(runtime, true, now_us);
        set_motion_state(runtime, 0.0f, 0.0f, EXTRUDER_STATE_ENABLE_WAIT);
        return;
    }
    if (snapshot.enabled && now_us < snapshot.enable_ready_us) {
        set_motion_state(runtime, 0.0f, 0.0f, EXTRUDER_STATE_ENABLE_WAIT);
        return;
    }

    if (desired_direction != 0 && snapshot.direction_sign == 0) {
        if (!change_direction_if_safe(runtime, desired_direction)) {
            set_motion_state(runtime, 0.0f, 0.0f, EXTRUDER_STATE_DIR_CHANGE);
            return;
        }
        snapshot.direction_sign = desired_direction;
    }

    float desired_speed = fabsf(desired_signed_speed);
    float old_speed = snapshot.speed_sps;
    float new_speed = move_towards(old_speed, desired_speed,
                                   EXTRUDER_MAX_ACCEL_SPS2 * elapsed_s);
    float phase = snapshot.phase +
                  0.5f * (old_speed + new_speed) * elapsed_s;
    uint32_t due_steps = (uint32_t)floorf(phase);
    phase -= (float)due_steps;

    extruder_state_t next_state;
    if (new_speed > old_speed + EXTRUDER_SPEED_EPSILON_SPS) {
        next_state = EXTRUDER_STATE_ACCEL;
    } else if (new_speed + EXTRUDER_SPEED_EPSILON_SPS < old_speed) {
        next_state = EXTRUDER_STATE_DECEL;
    } else if (new_speed > EXTRUDER_SPEED_EPSILON_SPS) {
        next_state = EXTRUDER_STATE_RUN;
    } else if (session_alive) {
        next_state = EXTRUDER_STATE_HOLD;
    } else {
        next_state = EXTRUDER_STATE_IDLE;
    }
    set_motion_state(runtime, new_speed, phase, next_state);

    int move_direction = snapshot.direction_sign;
    int64_t available_signed = snapshot.target_steps - snapshot.submitted_steps;
    if (due_steps > 0 && move_direction != 0 &&
        sign_i64(available_signed) == move_direction) {
        uint64_t available = (uint64_t)abs_i64(available_signed);
        if ((uint64_t)due_steps > available) {
            due_steps = (uint32_t)available;
        }
        if (due_steps > 0) {
            float pulse_speed = fmaxf(0.5f * (old_speed + new_speed), 1.0f);
            submit_steps(runtime, move_direction, due_steps, pulse_speed);
        }
    }

    if (!need_enable && snapshot.enabled &&
        snapshot.queued_count == 0 &&
        now_us - snapshot.last_active_us >= EXTRUDER_ENABLE_IDLE_TIMEOUT_US) {
        set_enabled(runtime, false, now_us);
    }
}

static void extruder_task(void *argument)
{
    (void)argument;

    motor3_gpio_init();
    motor4_gpio_init();
    int64_t now_us = esp_timer_get_time();

    for (int i = 0; i < EXTRUDER_TOOL_COUNT; ++i) {
        extruder_runtime_t *runtime = &g_tools[i];
        bool initialized = init_rmt(runtime);
        portENTER_CRITICAL(&g_extruder_mux);
        runtime->initialized = initialized;
        runtime->last_active_us = now_us;
        if (!initialized) {
            runtime->fault_flags |= EXTRUDER_FAULT_RMT_SUBMIT;
            runtime->state = EXTRUDER_STATE_FAULT;
        }
        portEXIT_CRITICAL(&g_extruder_mux);
    }

    motor3_ready = g_tools[0].initialized;
    motor4_ready = g_tools[1].initialized;

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t period_ticks = pdMS_TO_TICKS(EXTRUDER_CONTROL_PERIOD_MS);
    if (period_ticks == 0) {
        period_ticks = 1;
    }
    int64_t last_loop_us = esp_timer_get_time();

    while (true) {
        vTaskDelayUntil(&last_wake, period_ticks);
        now_us = esp_timer_get_time();
        float elapsed_s = (float)(now_us - last_loop_us) / 1000000.0f;
        last_loop_us = now_us;
        elapsed_s = clamp_float(elapsed_s, 0.000001f, EXTRUDER_MAX_DT_S);

        for (int i = 0; i < EXTRUDER_TOOL_COUNT; ++i) {
            if (g_tools[i].initialized) {
                planner_step(&g_tools[i], elapsed_s, now_us);
            }
        }
    }
}

void extruder_task_start(void)
{
    portENTER_CRITICAL(&g_extruder_mux);
    if (g_extruder_task_started) {
        portEXIT_CRITICAL(&g_extruder_mux);
        return;
    }
    g_extruder_task_started = true;
    portEXIT_CRITICAL(&g_extruder_mux);

    BaseType_t created = xTaskCreatePinnedToCore(extruder_task, "extruder_rt",
                                                 6144, NULL, 7,
                                                 &g_extruder_task_handle, 0);
    if (created != pdPASS) {
        portENTER_CRITICAL(&g_extruder_mux);
        g_extruder_task_started = false;
        for (int i = 0; i < EXTRUDER_TOOL_COUNT; ++i) {
            g_tools[i].fault_flags |= EXTRUDER_FAULT_RMT_SUBMIT;
            g_tools[i].state = EXTRUDER_STATE_FAULT;
        }
        portEXIT_CRITICAL(&g_extruder_mux);
    }
}

bool extruder_set_absolute_um_at(int tool_id, int64_t total_um, uint32_t seq,
                                 int64_t rx_us)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return false;
    }
    if (total_um < -EXTRUDER_MAX_ABS_UM || total_um > EXTRUDER_MAX_ABS_UM) {
        portENTER_CRITICAL(&g_extruder_mux);
        runtime->fault_flags |= EXTRUDER_FAULT_PROTOCOL;
        portEXIT_CRITICAL(&g_extruder_mux);
        return false;
    }
    if (rx_us <= 0) {
        rx_us = esp_timer_get_time();
    }

    int64_t target_steps = absolute_um_to_steps(tool_id, total_um);
    portENTER_CRITICAL(&g_extruder_mux);
    if (runtime->reset_pending) {
        portEXIT_CRITICAL(&g_extruder_mux);
        return false;
    }

    /*
     * 上位机在浮点数默认格式化和串行化之前去重。绝对 E 较大时，
     * 多个新序号报文中的 ASCII E 可能解析为同一个数值。因此单独
     * 保留“上次不同 E 值”锚点，让这些重复量化报文共同延长一次
     * 平均速度的计算区间，避免前馈速度出现 0、0、尖峰、0 的
     * 跳变。
     */
    float feedforward = runtime->feedforward_steps_s;
    int64_t sample_dt_us = 0;
    if (!runtime->feedforward_anchor_valid) {
        feedforward = 0.0f;
        runtime->feedforward_anchor_total_q = total_um;
        runtime->feedforward_anchor_seq = seq;
        runtime->feedforward_anchor_rx_us = rx_us;
        runtime->feedforward_anchor_valid = true;
    } else if (total_um != runtime->feedforward_anchor_total_q) {
        sample_dt_us = estimate_feedforward_dt_us(
            seq, runtime->feedforward_anchor_seq,
            rx_us, runtime->feedforward_anchor_rx_us,
            EXTRUDER_FEEDFORWARD_MAX_ANCHOR_US);
        int64_t delta_um = total_um - runtime->feedforward_anchor_total_q;
        feedforward = 0.0f;
        if (sample_dt_us > 0 && delta_um != 0) {
            double elapsed_s = (double)sample_dt_us / 1000000.0;
            double calculated = ((double)delta_um *
                                 steps_per_mm_for_tool(tool_id) / 1000000.0) /
                                elapsed_s;
            if (isfinite(calculated)) {
                feedforward = clamp_float((float)calculated,
                                          -EXTRUDER_MAX_STEP_RATE_SPS,
                                          EXTRUDER_MAX_STEP_RATE_SPS);
            }
        }
        runtime->feedforward_anchor_total_q = total_um;
        runtime->feedforward_anchor_seq = seq;
        runtime->feedforward_anchor_rx_us = rx_us;
    } else if (runtime->host_sample_valid) {
        /* 刷新当前前馈估计的有效期，但不移动“上次不同 E 值”锚点。 */
        sample_dt_us = estimate_feedforward_dt_us(
            seq, runtime->last_seq, rx_us, runtime->last_command_us,
            EXTRUDER_FEEDFORWARD_MAX_REFRESH_US);
        if (sample_dt_us == 0) {
            feedforward = 0.0f;
        }
    }

    int64_t hold_us = (sample_dt_us > 0) ? sample_dt_us * 3LL
                                         : EXTRUDER_FEEDFORWARD_MIN_HOLD_US;
    hold_us = clamp_i64(hold_us, EXTRUDER_FEEDFORWARD_MIN_HOLD_US,
                        EXTRUDER_FEEDFORWARD_MAX_HOLD_US);

    runtime->target_steps = target_steps;
    runtime->host_total_um = total_um;
    runtime->last_seq = seq;
    runtime->last_command_us = rx_us;
    runtime->feedforward_valid_until_us = rx_us + hold_us;
    runtime->feedforward_steps_s = feedforward;
    runtime->host_sample_valid = true;
    runtime->host_hold_active = !has_hard_fault(runtime->fault_flags);

    /* 收到正式串口打印数据后，立即接管可能忘记停止的 UI 手动点动。 */
    runtime->ui_active = false;
    runtime->ui_direction = 0;
    runtime->ui_target_steps_exact = (double)target_steps;
    portEXIT_CRITICAL(&g_extruder_mux);
    return true;
}

bool extruder_set_absolute_um(int tool_id, int64_t total_um, uint32_t seq)
{
    return extruder_set_absolute_um_at(tool_id, total_um, seq,
                                       esp_timer_get_time());
}

void extruder_set_absolute(int tool_id, float total_mm)
{
    if (!isfinite(total_mm)) {
        return;
    }
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return;
    }

    uint32_t seq;
    portENTER_CRITICAL(&g_extruder_mux);
    seq = runtime->host_sample_valid ? runtime->last_seq + 1U : 0U;
    portEXIT_CRITICAL(&g_extruder_mux);
    int64_t total_um = (int64_t)llround((double)total_mm * 1000000.0);
    (void)extruder_set_absolute_um(tool_id, total_um, seq);
}

void extruder_reset_absolute(int tool_id)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return;
    }
    portENTER_CRITICAL(&g_extruder_mux);
    runtime->reset_pending = true;
    runtime->state = EXTRUDER_STATE_RESETTING;
    portEXIT_CRITICAL(&g_extruder_mux);
}

bool extruder_reset_absolute_wait(int tool_id, uint32_t timeout_ms)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return false;
    }

    uint32_t start_generation;
    portENTER_CRITICAL(&g_extruder_mux);
    start_generation = runtime->reset_generation;
    runtime->reset_pending = true;
    runtime->state = EXTRUDER_STATE_RESETTING;
    portEXIT_CRITICAL(&g_extruder_mux);

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    if (timeout_ticks == 0) {
        timeout_ticks = 1;
    }

    while ((xTaskGetTickCount() - start_tick) <= timeout_ticks) {
        bool complete;
        portENTER_CRITICAL(&g_extruder_mux);
        complete = runtime->reset_generation != start_generation;
        portEXIT_CRITICAL(&g_extruder_mux);
        if (complete) {
            return true;
        }
        vTaskDelay(1);
    }

    bool completed_during_timeout_edge;
    portENTER_CRITICAL(&g_extruder_mux);
    completed_during_timeout_edge = runtime->reset_generation != start_generation;
    if (!completed_during_timeout_edge) {
        /*
         * reset 超时后只撤销请求，不清空任何脉冲账本。这样迟到的 RMT
         * 完成回调仍可按旧原点正确结算，避免把尚未完成的旧脉冲错误地
         * 计入一个被虚假确认的新原点。
         */
        runtime->reset_pending = false;
        runtime->fault_flags |= EXTRUDER_FAULT_RESET_TIMEOUT;
        runtime->state = EXTRUDER_STATE_FAULT;
    }
    portEXIT_CRITICAL(&g_extruder_mux);
    return completed_during_timeout_edge;
}

void extruder_ui_start(int tool_id, int dir_sign)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_extruder_mux);
    if (!runtime->reset_pending && !has_hard_fault(runtime->fault_flags)) {
        runtime->ui_direction = (dir_sign >= 0) ? 1 : -1;
        runtime->ui_active = true;
        runtime->ui_target_steps_exact = (double)runtime->target_steps;
        runtime->host_sample_valid = false;
        runtime->feedforward_anchor_valid = false;
        runtime->feedforward_valid_until_us = 0;
        runtime->host_hold_active = false;
        runtime->feedforward_steps_s = 0.0f;
        runtime->last_command_us = esp_timer_get_time();
    }
    portEXIT_CRITICAL(&g_extruder_mux);
}

void extruder_ui_stop(int tool_id)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return;
    }
    portENTER_CRITICAL(&g_extruder_mux);
    runtime->ui_active = false;
    runtime->ui_direction = 0;
    runtime->host_hold_active = false;
    runtime->feedforward_steps_s = 0.0f;
    runtime->last_command_us = esp_timer_get_time();
    portEXIT_CRITICAL(&g_extruder_mux);
}

void extruder_notify_transport_fault(uint32_t fault_flags)
{
    if (fault_flags == 0) {
        return;
    }
    portENTER_CRITICAL(&g_extruder_mux);
    for (int i = 0; i < EXTRUDER_TOOL_COUNT; ++i) {
        g_tools[i].fault_flags |= fault_flags;
    }
    portEXIT_CRITICAL(&g_extruder_mux);
}

bool extruder_get_status(int tool_id, extruder_status_t *status)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL || status == NULL) {
        return false;
    }

    portENTER_CRITICAL(&g_extruder_mux);
    status->target_steps = runtime->target_steps;
    status->submitted_steps = runtime->submitted_steps;
    status->completed_steps = runtime->completed_steps;
    status->host_total_um = runtime->host_total_um;
    status->last_command_us = runtime->last_command_us;
    status->feedforward_valid_until_us = runtime->feedforward_valid_until_us;
    status->feedforward_steps_s = runtime->feedforward_steps_s;
    status->velocity_steps_s = runtime->speed_sps *
                               (float)runtime->direction_sign;
    status->phase = runtime->phase;
    status->last_seq = runtime->last_seq;
    status->fault_flags = runtime->fault_flags;
    status->hard_fault_flags = runtime->fault_flags & EXTRUDER_HARD_FAULT_MASK;
    status->rmt_submit_errors = runtime->rmt_submit_errors;
    status->rmt_done_errors = runtime->rmt_done_errors;
    status->queue_depth = runtime->queued_count;
    status->direction_sign = runtime->direction_sign;
    status->enabled = runtime->enabled;
    status->host_hold_active = runtime->host_hold_active;
    status->host_sample_valid = runtime->host_sample_valid;
    status->state = runtime->state;
    portEXIT_CRITICAL(&g_extruder_mux);
    return true;
}

bool extruder_is_ready(int tool_id)
{
    extruder_runtime_t *runtime = runtime_for_tool(tool_id);
    if (runtime == NULL) {
        return false;
    }
    bool ready;
    portENTER_CRITICAL(&g_extruder_mux);
    ready = g_extruder_task_started && runtime->initialized &&
            runtime->channel != NULL && runtime->encoder != NULL;
    portEXIT_CRITICAL(&g_extruder_mux);
    return ready;
}
