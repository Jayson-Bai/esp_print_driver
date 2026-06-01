#ifndef FAN_H
#define FAN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>


#define FAN1_PIN 12 //纤维引脚
#define FAN2_PIN 10 //树脂引脚

extern volatile bool fan1_ready;
extern volatile bool fan2_ready;

void Fan_INI(void);

typedef enum {
    FAN1, // 纤维
    FAN2  // 树脂
} fan_id_t;


typedef struct {
    fan_id_t id;     
    bool enable; // true = 打开, false = 关闭
} fan_cmd_t;


extern QueueHandle_t fan_queue;


void fan_task(void *pvParameters);


void fan_send_command(fan_id_t id);
void fan_set_state(fan_id_t id, bool enable);

bool fan_get_state(fan_id_t id);

#endif // FAN_H
