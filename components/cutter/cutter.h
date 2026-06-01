#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

//引脚
#define K1_PIN      16
#define K2_PIN      17
#define SW1_PIN     19
#define SW2_PIN     20

typedef struct {
    int pin;       // 只能是 K1_PIN 或 K2_PIN
    bool state;    // false 触发
} cutter_command_t;

void cutter_gpio_init(void);
void cutter_task(void *pvParameters);
void cutter_send_command(int PIN_NUM, bool state);

