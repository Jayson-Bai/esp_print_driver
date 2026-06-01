#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_rom_sys.h"

// 引脚
// X轴 纤维                     MOTOR1
#define STEP_PIN_1      1   // 脉冲
#define DIR_PIN_1       2   // 方向
#define STOP_PIN_1      13  // 限位  
#define EN_PIN_1        42  // 使能
// Y轴 树脂                     MOTOR2
#define STEP_PIN_2      38  // 脉冲
#define DIR_PIN_2       0   // 方向
#define STOP_PIN_2      14  // 限位  
#define EN_PIN_2        41  // 使能
// 纤维挤出                     MOTOR3
#define STEP_PIN_3      45  // 脉冲
#define DIR_PIN_3       48  // 方向 
#define EN_PIN_3        40  // 使能
// 树脂挤出                     MOTOR4
#define STEP_PIN_4      47  // 脉冲
#define DIR_PIN_4       21  // 方向 
#define EN_PIN_4        39  // 使能

// 转速
#define STEP_DELAY_US  75   // 微秒 (1 毫秒 = 1000 微秒)  

//微步
#define motor1_microstep 8
#define motor2_microstep 8
#define motor3_microstep 16
#define motor4_microstep 16

#define SAFE_TARGET_MM         70.0f  // 动作目标：抬升到 70mm
#define SAFE_SWITCH_HEIGHT_MM  60.0f    // 定义安全切换的阈值高度
#define SWITCH_TIMEOUT_MS      30000    // 超时时间 5000ms (5秒)
#define HOME_TIMEOUT_MS      30000
#define CHECK_INTERVAL_MS      20      // 检测间隔 20ms

void Motor_INI(void);

extern volatile bool motor1_ready;
extern volatile bool motor2_ready;
extern volatile bool motor3_ready;
extern volatile bool motor4_ready;

// Motor3（纤维挤出机）步进延时
void motor3_set_step_delay_us(uint32_t delay_us);
uint32_t motor3_get_step_delay_us(void);
uint32_t motor3_get_retract_step_delay_us(void);

// Motor4（树脂挤出机）步进延时
void motor4_set_step_delay_us(uint32_t delay_us);
uint32_t motor4_get_step_delay_us(void);
uint32_t motor4_get_retract_step_delay_us(void);

// 挤出机进料速度（耗材前进速度，mm/s）
void motor3_set_speed_mm_per_s(float v_mm_s);
void motor4_set_speed_mm_per_s(float v_mm_s);

float motor3_get_speed_mm_per_s(void);
float motor4_get_speed_mm_per_s(void);

// 回抽时耗材退回速度（mm/s）
void motor3_set_retract_speed_mm_per_s(float v_mm_s);
float motor3_get_retract_speed_mm_per_s(void);

void motor4_set_retract_speed_mm_per_s(float v_mm_s);
float motor4_get_retract_speed_mm_per_s(void);



//从右往左：黑 绿 蓝 红，对应TMC2209 从左往右 的接线情况下：
typedef enum {
    CW,   // CW =0, 顺时针，向下
    CCW,   // CCW=1, 逆时针，向上
    MOTOR_STOP  //停止标志
} motor_dir_t;

typedef enum {
    MOTOR_CMD_MOVE_STEPS = 0,   // 按 steps 运动
    MOTOR_CMD_MOVE_TO_MM,       // 绝对位置就位 motor_move_to_mm()
    MOTOR_CMD_POSITION,         // 找工作位 motor_positon()
    MOTOR_CMD_HOMING            // 上升到安全位置 motor_homing()
} motor_cmd_type_t;

typedef struct {
    motor_dir_t direction;   // 0向下，1向上，2停止
    int steps;
    uint32_t step_delay_us;
    motor_cmd_type_t cmd_type;
    float   target_mm;  //MOVE_TO_MM / HOMING 用
} motor_cmd_t;



typedef enum {
    MOTOR1 = 1,  
    MOTOR2 = 2,   
    MOTOR3 = 3,
    MOTOR4 = 4
} motor_id_t;

extern volatile motor_dir_t motor1_current_dir;
extern volatile motor_dir_t motor2_current_dir;


extern QueueHandle_t motor1_queue;
extern QueueHandle_t motor2_queue;
extern QueueHandle_t motor3_queue;
extern QueueHandle_t motor4_queue;

void motor1_gpio_init(void);
void motor2_gpio_init(void);
void motor3_gpio_init(void);
void motor4_gpio_init(void);
void stop_gpio_init(void);
void stop_monitor_task(void *arg);
void motor_move_loop(const char *TAG, motor_cmd_t *cmd, int STEP_PIN, int DIR_PIN, int EN_PIN, int STOP_PIN, volatile bool *limit_triggered, volatile motor_dir_t *current_dir);
void motor1_task(void *pvParameters);
void motor2_task(void *pvParameters);
void motor3_task(void *pvParameters);
void motor4_task(void *pvParameters);
void motor_send_command(motor_id_t motor_id, motor_dir_t direction, int steps);
void motor_send_command_mm(motor_id_t motor_id, motor_dir_t direction, float mm);
float motor1_position_mm(void);
float motor2_position_mm(void);
void motor_homing_impl(motor_id_t motor_id);
void motor_position_impl(motor_id_t motor_id);
void switch_to_CF(void);
void switch_to_RESIN(void);
void extruder_parameters_init(void);
float get_X_position_mm(void);
float get_Y_position_mm(void);
void motor_move_to_mm(motor_id_t motor_id, float target_mm);
void motor_homing(motor_id_t motor_id);
void motor_position(motor_id_t motor_id);
void motor_move_to_mm_impl(motor_id_t motor_id, float target_mm);




