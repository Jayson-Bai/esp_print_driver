#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

//引脚
#define K1_PIN      16
#define K2_PIN      17
#define CUT_PIN     15
#define SW1_PIN     19
#define SW2_PIN     20

typedef struct {
    int pin;       // 只能是 K1_PIN 或 K2_PIN
    bool state;    // false 触发
    TaskHandle_t completion_task; // 可选的同步等待方
} cutter_command_t;

typedef void (*cutter_cut_done_cb_t)(void *ctx, bool ok);

void cutter_gpio_init(void);
void cutter_task(void *pvParameters);
void cutter_send_command(int PIN_NUM, bool state);
bool cutter_send_command_wait(int PIN_NUM, bool state, uint32_t timeout_ms);
bool cutter_start_cut_sequence_task(void);
bool cutter_start_cut_sequence_async(cutter_cut_done_cb_t done_cb, void *done_ctx);
bool cutter_run_cut_sequence_if_idle(void);

