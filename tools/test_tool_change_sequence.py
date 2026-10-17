#!/usr/bin/env python3
"""Compile the real switch/motion C functions with a deterministic GPIO/RTOS harness."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

from test_limit_interrupt_regression import c_function_body

ROOT = Path(__file__).resolve().parents[1]


class ToolChangeSequenceTest(unittest.TestCase):
    def test_real_sequence_all_endpoints_delays_failures_and_entry_points(self):
        source = (ROOT / 'components/motor/motor.c').read_text()
        header = (ROOT / 'components/motor/motor.h').read_text()
        constants = '\n'.join(re.findall(
            r'^#define (?:STEP_PIN_[12]|DIR_PIN_[12]|EN_PIN_[12]|STOP_PIN_[12]|'
            r'motor[12]_microstep|SAFE_TARGET_MM|TOOL_CHANGE_SETTLE_MS|'
            r'SWITCH_TIMEOUT_MS|HOME_TIMEOUT_MS|CHECK_INTERVAL_MS|STEP_DELAY_US)\s+.*$', header, re.M))
        declarations = '\n'.join(re.search(
            rf'typedef\s+(?:enum|struct)\s*\{{[^}}]*\}}\s*{name};', header).group()
            for name in ('motor_dir_t', 'motor_id_t', 'motor_cmd_type_t', 'motor_cmd_t'))
        signatures = [
            'static int axis_stop_pin(', 'static bool axis_is_at_work_position(',
            'static bool axis_limit_stably_active(', 'static void update_axis_ready_flags(',
            'static void sync_axis_origin_from_limit(', 'static void axis_set_endpoint(',
            'static bool axis_sync_endpoint(', 'void motor_startup_check(',
            'static bool axis_sequence_take(', 'static void axis_sequence_give(',
            'void motor_move_loop(', 'void motor_position_impl(',
            'static bool motor_position_wait(', 'void motor_position(',
            'static bool motor_homing_wait(', 'void motor_homing(',
            'static bool switch_to_axis(', 'bool switch_to_CF(',
            'bool switch_to_RESIN(', 'bool Motor_INI(',
        ]
        functions = '\n'.join(signature + source[source.index(signature) + len(signature):
                          source.index('{', source.index(signature))] + '{' +
                          c_function_body(source, signature) + '}' for signature in signatures)
        harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
typedef unsigned TickType_t;
#define pdTRUE 1
#define pdMS_TO_TICKS(ms) (ms)
#define portMAX_DELAY 0xffffffffU
static void log_ignore(const char *tag, const char *fmt, ...) { (void)tag; (void)fmt; }
#define ESP_LOGI(...) log_ignore(__VA_ARGS__)
#define ESP_LOGE(...) log_ignore(__VA_ARGS__)
''' + constants + '\n' + declarations + r'''
volatile bool motor1_ready, motor2_ready, motor3_ready, motor4_ready;
volatile bool limit_triggered_1, limit_triggered_2;
bool motor1_homed, motor2_homed;
volatile int motor1_position_steps, motor2_position_steps;
volatile motor_dir_t motor1_current_dir, motor2_current_dir;
static volatile bool g_axes_startup_checked, g_axis_sequence_active;
static SemaphoreHandle_t g_axis_sequence_mutex;
QueueHandle_t motor1_queue, motor2_queue;
static int height[3], direction[3], pulses[3], sensor_high[3];
static bool mutex_taken, pending, fail_queue, fail_notify;
static unsigned notifications;
static motor_cmd_t pending_cmd, history[8];
static int pending_axis, history_axis[8], command_count, settle_count;
static uint64_t now_us, ascent_done_us;
static TaskHandle_t caller = (void *)99;

int gpio_get_level(int pin) {
    int axis=pin==STOP_PIN_1 ? 1 : 2;
    return sensor_high[axis] || height[axis]>0;
}
void gpio_set_level(int pin, int level) {
    for (int axis=1; axis<=2; axis++) {
        int dir_pin=axis==1 ? DIR_PIN_1 : DIR_PIN_2;
        int step_pin=axis==1 ? STEP_PIN_1 : STEP_PIN_2;
        if (pin==dir_pin) direction[axis]=level;
        if (pin==step_pin && level) {
            pulses[axis]++;
            height[axis]+=direction[axis] ? 1 : -1;
        }
    }
}
void esp_rom_delay_us(unsigned us) { now_us+=us; }
void vTaskDelay(unsigned ms) {
    if (ms==TOOL_CHANGE_SETTLE_MS) {
        assert(ascent_done_us && !pending);
        settle_count++;
    }
    now_us+=(uint64_t)ms*1000;
}
void stop_gpio_init(void) {}
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
int xSemaphoreTake(SemaphoreHandle_t semaphore, unsigned timeout) {
    (void)semaphore; (void)timeout;
    if (mutex_taken) return 0;
    mutex_taken=true;
    return pdTRUE;
}
int xSemaphoreGive(SemaphoreHandle_t semaphore) {
    (void)semaphore; assert(mutex_taken); mutex_taken=false; return pdTRUE;
}
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return caller; }
void xTaskNotifyGive(TaskHandle_t task) { assert(task==caller); notifications++; }
bool extruder_is_ready(int tool) { return tool==1 || tool==2; }
int xQueueSend(QueueHandle_t q, const motor_cmd_t *cmd, unsigned timeout);
unsigned ulTaskNotifyTake(int clear, unsigned timeout);
bool switch_to_CF(void);
bool switch_to_RESIN(void);
''' + functions + r'''
int xQueueSend(QueueHandle_t q, const motor_cmd_t *cmd, unsigned timeout) {
    assert(timeout!=portMAX_DELAY);
    if (fail_queue) return 0;
    assert(!pending && command_count<8);
    if (cmd->cmd_type==MOTOR_CMD_POSITION && ascent_done_us) {
        assert(now_us>=ascent_done_us+(uint64_t)TOOL_CHANGE_SETTLE_MS*1000);
    }
    assert(cmd->completion_task==caller);
    pending=true;
    pending_cmd=history[command_count]=*cmd;
    pending_axis=history_axis[command_count++]=(int)(intptr_t)q;
    return pdTRUE;
}
unsigned ulTaskNotifyTake(int clear, unsigned timeout) {
    if (timeout && pending && !fail_notify) {
        int axis=pending_axis;
        motor_cmd_t cmd=pending_cmd;
        pending=false;
        if (cmd.cmd_type==MOTOR_CMD_MOVE_STEPS) {
            assert(cmd.direction==CCW && cmd.steps==48000);
            motor_move_loop("test", &cmd,
                axis==1 ? STEP_PIN_1 : STEP_PIN_2, axis==1 ? DIR_PIN_1 : DIR_PIN_2,
                axis==1 ? EN_PIN_1 : EN_PIN_2, axis==1 ? STOP_PIN_1 : STOP_PIN_2,
                axis==1 ? &limit_triggered_1 : &limit_triggered_2,
                axis==1 ? &motor1_current_dir : &motor2_current_dir);
            ascent_done_us=now_us;
        } else {
            assert(cmd.cmd_type==MOTOR_CMD_POSITION);
            motor_position_impl(axis);
        }
        xTaskNotifyGive(cmd.completion_task);
    }
    unsigned result=notifications;
    if (clear) notifications=0;
    if (timeout && !result) now_us+=(uint64_t)timeout*1000;
    return result;
}
static void reset(int mask, int stale) {
    height[1]=(mask&1) ? 0 : 48000;
    height[2]=(mask&2) ? 0 : 48000;
    direction[1]=direction[2]=pulses[1]=pulses[2]=0;
    sensor_high[1]=sensor_high[2]=0;
    motor1_homed=motor2_homed=!stale;
    motor1_ready=motor2_ready=false;
    motor1_position_steps=motor2_position_steps=stale ? -123 : 0;
    limit_triggered_1=limit_triggered_2=stale;
    motor1_current_dir=motor2_current_dir=MOTOR_STOP;
    g_axes_startup_checked=g_axis_sequence_active=false;
    g_axis_sequence_mutex=NULL;
    motor1_queue=(void *)1; motor2_queue=(void *)2;
    mutex_taken=pending=fail_queue=fail_notify=false;
    notifications=command_count=settle_count=0;
    now_us=ascent_done_us=0;
}
int main(void) {
    int combinations=0;
    for (int mask=0; mask<3; mask++) for (int target=1; target<=2; target++) for (int stale=0; stale<2; stale++) {
        reset(mask, stale);
        bool target_at_work=(mask & (1<<(target-1)))!=0;
        int other=target==1 ? 2 : 1;
        bool other_at_work=(mask & (1<<(other-1)))!=0;
        assert(switch_to_axis(target));
        assert(height[target]==0 && height[other]==48000);
        assert(command_count==(other_at_work ? 1 : 0)+(target_at_work ? 0 : 1));
        assert(settle_count==(other_at_work ? 1 : 0));
        assert(motor1_ready && motor2_ready && !mutex_taken && !g_axis_sequence_active);
        if (target_at_work) assert(pulses[target]==0);
        if (other_at_work) assert(history_axis[0]==other && history[0].cmd_type==MOTOR_CMD_MOVE_STEPS);
        int previous_commands=command_count;
        assert(target==1 ? switch_to_CF() : switch_to_RESIN());
        assert(command_count==previous_commands);
        combinations++;
    }
    reset(0, 1);
    motor_startup_check();
    assert(motor1_position_steps==48000 && motor2_position_steps==48000);
    assert(Motor_INI() && height[1]==48000 && height[2]==0 && command_count==1);
    assert(motor3_ready && motor4_ready);
    reset(1, 1); motor_position(MOTOR2);
    assert(height[1]==48000 && height[2]==0 && command_count==2);
    reset(0, 1); motor_homing(MOTOR1); assert(command_count==0);
    reset(1, 1); motor_homing(MOTOR1); assert(height[1]==48000 && command_count==1);
    reset(2, 1); fail_queue=true;
    assert(!switch_to_CF() && command_count==0 && !mutex_taken);
    reset(2, 1); fail_notify=true;
    assert(!switch_to_CF() && command_count==1 && !mutex_taken);
    reset(0, 1); motor1_queue=NULL;
    assert(!switch_to_CF() && command_count==0 && !mutex_taken);
    reset(0, 1); sensor_high[1]=1;
    assert(!switch_to_CF() && command_count==1 && !motor1_ready && !mutex_taken);
    reset(0, 1); assert(!switch_to_axis(99) && command_count==0);
    printf("12 physically valid endpoint/cache combinations, repeated selection, shared entries, delay ordering and bounded failures passed (%d).\n", combinations);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            code = Path(directory) / 'switch.c'
            binary = Path(directory) / 'switch'
            code.write_text(harness)
            subprocess.run(['gcc', '-std=c11', '-O2', '-Wall', '-Wextra', str(code), '-lm', '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
