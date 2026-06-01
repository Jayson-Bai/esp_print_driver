//42步进电机

//丝杠导程2mm 
//步距角1.8°    （200步/圈）


#include "motor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>
#include "esp_task_wdt.h" 

//全局标志 打印前询问
volatile bool motor1_ready = false;  // 纤维Z轴
volatile bool motor2_ready = false;  // 树脂Z轴
volatile bool motor3_ready = false;  // 纤维挤出机
volatile bool motor4_ready = false;  // 树脂挤出机

volatile bool limit_triggered_1 = false;
volatile bool limit_triggered_2 = false;

//电机复位标志
bool motor1_homed = false;
bool motor2_homed = false;


// 电机限位监测任务
volatile motor_dir_t motor1_current_dir = MOTOR_STOP; 
volatile motor_dir_t motor2_current_dir = MOTOR_STOP; 

volatile int motor1_position_steps = 0;
volatile int motor2_position_steps = 0;

// MOTOR3（挤出机：纤维）的当前步进延时
static volatile uint32_t g_motor3_step_delay_us = STEP_DELAY_US;
// MOTOR4（挤出机：树脂）的当前步进延时
static volatile uint32_t g_motor4_step_delay_us = STEP_DELAY_US;
// 回抽时的步进延时
static volatile uint32_t g_motor3_retract_step_delay_us = STEP_DELAY_US;
static volatile uint32_t g_motor4_retract_step_delay_us = STEP_DELAY_US;

static float g_motor3_extrude_speed_mm_s  = 0.0f;
static float g_motor3_retract_speed_mm_s  = 0.0f;
static float g_motor4_extrude_speed_mm_s  = 0.0f;
static float g_motor4_retract_speed_mm_s  = 0.0f;

// Motor3 接口
void motor3_set_step_delay_us(uint32_t delay_us)
{
    // 简单保护一下，防止设置过小
    if (delay_us < 100) {
        delay_us = 100;
    }
    g_motor3_step_delay_us = delay_us;
}

uint32_t motor3_get_step_delay_us(void)
{
    return g_motor3_step_delay_us;
}

uint32_t motor3_get_retract_step_delay_us(void)
{
    return g_motor3_retract_step_delay_us;
}

// Motor4 接口
void motor4_set_step_delay_us(uint32_t delay_us)
{
    if (delay_us < 100) {
        delay_us = 100;
    }
    g_motor4_step_delay_us = delay_us;
}

uint32_t motor4_get_step_delay_us(void)
{
    return g_motor4_step_delay_us;
}

uint32_t motor4_get_retract_step_delay_us(void)
{
    return g_motor4_retract_step_delay_us;
}

static void update_ready_if_safe(void)
{
    float x = motor1_position_mm();
    float y = motor2_position_mm();
    if (motor1_homed && motor2_homed &&
        x >= SAFE_SWITCH_HEIGHT_MM && y >= SAFE_SWITCH_HEIGHT_MM) {
        motor1_ready = true;
        motor2_ready = true;
    }
}

// 检查另一个轴是否处于安全位置
static bool is_other_axis_safe(motor_id_t self_id)
{
    //  SAFE_SWITCH_HEIGHT_MM (70.0f)
    float safe_threshold = SAFE_SWITCH_HEIGHT_MM; 
    float other_pos = 0.0f;
    bool other_homed = false;

    if (self_id == MOTOR1) {
        other_homed = motor2_homed;
        other_pos = motor2_position_mm();
    } else {
        other_homed = motor1_homed;
        other_pos = motor1_position_mm();
    }

    if (!other_homed) {
        // ESP_LOGE("SAFETY", "互锁拒绝：对方轴未归零，位置未知！");
        return false;
    }

    if (other_pos < safe_threshold) {
        // ESP_LOGE("SAFETY", "互锁拒绝：对方轴位于低处 (%.2f mm < %.2f mm)，存在撞机风险！", other_pos, safe_threshold);
        return false;
    }
    return true;
}

// 初始化MOTOR1控制引脚   纤维轴
void motor1_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_PIN_1) | (1ULL << DIR_PIN_1) | (1ULL << EN_PIN_1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);
    gpio_set_level(STEP_PIN_1, 0);
    gpio_set_level(DIR_PIN_1, 0);
    gpio_set_level(EN_PIN_1, 1);  // TMC2209低电平使能, 初始化时默认关闭使能
}

// 初始化MOTOR2控制引脚   树脂轴
void motor2_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_PIN_2) | (1ULL << DIR_PIN_2) | (1ULL << EN_PIN_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(STEP_PIN_2, 0);
    gpio_set_level(DIR_PIN_2, 0);
    gpio_set_level(EN_PIN_2, 1);  
}
    
// 初始化MOTOR1、MOTOR2限位引脚
void stop_gpio_init(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        //.pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    // 初始化 STOP_PIN_1
    io_conf.pin_bit_mask = (1ULL << STOP_PIN_1);
    gpio_config(&io_conf);

    // 初始化 STOP_PIN_2
    io_conf.pin_bit_mask = (1ULL << STOP_PIN_2);
    gpio_config(&io_conf);
}

static TaskHandle_t g_stop_task_handle = NULL;
static volatile int64_t g_stop1_last_isr_us = 0;
static volatile int64_t g_stop2_last_isr_us = 0;
#define STOP_DEBOUNCE_US 3000

static void IRAM_ATTR stop_gpio_isr_handler(void *arg)
{
    int pin = (int)(intptr_t)arg;
    int64_t now_us = esp_timer_get_time();

    if (pin == STOP_PIN_1) {
        if (now_us - g_stop1_last_isr_us < STOP_DEBOUNCE_US) {
            return;
        }
        g_stop1_last_isr_us = now_us;
        if (motor1_current_dir == CW) {
            limit_triggered_1 = true;
            gpio_set_level(EN_PIN_1, 1);
        }
    } else if (pin == STOP_PIN_2) {
        if (now_us - g_stop2_last_isr_us < STOP_DEBOUNCE_US) {
            return;
        }
        g_stop2_last_isr_us = now_us;
        if (motor2_current_dir == CW) {
            limit_triggered_2 = true;
            gpio_set_level(EN_PIN_2, 1);
        }
    }

    if (g_stop_task_handle != NULL) {
        BaseType_t higher_woken = pdFALSE;
        vTaskNotifyGiveFromISR(g_stop_task_handle, &higher_woken);
        if (higher_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}





void stop_monitor_task(void *arg) {

    stop_gpio_init(); 
    g_stop_task_handle = xTaskGetCurrentTaskHandle();

    static bool isr_installed = false;
    if (!isr_installed) {
        esp_err_t err = gpio_install_isr_service(0);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            gpio_isr_handler_add(STOP_PIN_1, stop_gpio_isr_handler, (void *)(intptr_t)STOP_PIN_1);
            gpio_isr_handler_add(STOP_PIN_2, stop_gpio_isr_handler, (void *)(intptr_t)STOP_PIN_2);
            isr_installed = true;
        }
    }

    // 用于边沿检测（防止重复触发）
    int last_level_1 = 1;
    int last_level_2 = 1;

    while (1) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        // ---- MOTOR1 限位检测 ----
        int level1 = gpio_get_level(STOP_PIN_1);

        if (last_level_1 == 1 && level1 == 0) {  // 下降沿触发
            if (!limit_triggered_1 && motor1_current_dir == CW) {
                limit_triggered_1 = true;
                gpio_set_level(EN_PIN_1, 1);  
                motor1_position_steps = 0;
                motor1_homed = true;
                // ESP_LOGW("STOP", "MOTOR1 限位触发, 禁止继续向下运动");
            }
        }
        last_level_1 = level1;

        // ---- MOTOR2 限位检测 ----
        int level2 = gpio_get_level(STOP_PIN_2);

        if (last_level_2 == 1 && level2 == 0) {  // 下降沿触发
            if (!limit_triggered_2 && motor2_current_dir == CW) {
                limit_triggered_2 = true;
                gpio_set_level(EN_PIN_2, 1); 
                motor2_position_steps = 0; 
                motor2_homed = true;
                // ESP_LOGW("STOP", "MOTOR2 限位触发, 禁止继续向下运动");
            }
        }
        last_level_2 = level2;
    }
}


//电机位置转换
float motor1_position_mm(void) 
{
    float steps_per_mm = (200 * motor1_microstep) / 2.0f;   //每mm需要的步数
    return (float)motor1_position_steps / steps_per_mm;     //运行距离mm
}

float motor2_position_mm(void) 
{
    float steps_per_mm = (200 * motor2_microstep) / 2.0f;
    return (float)motor2_position_steps / steps_per_mm;
}

// MOTOR1、MOTOR2 通用步数运动函数
void motor_move_loop(const char *TAG, motor_cmd_t *cmd, int STEP_PIN, int DIR_PIN, int EN_PIN, int STOP_PIN, volatile bool *limit_triggered, volatile motor_dir_t *current_dir) 
{
    // 1. 更新全局方向
    *current_dir = cmd->direction;

    if(cmd->direction == MOTOR_STOP) {
        gpio_set_level(EN_PIN, 1); 
        // ESP_LOGI(TAG, "Received STOP command.");
        return;
    }

    // --- 启动前检查 ---
    if (cmd->direction == CW) {
        if (*limit_triggered || gpio_get_level(STOP_PIN) == 0) {
            // ESP_LOGW(TAG, "启动被拒绝：限位已触发！");
            *limit_triggered = true;
            *current_dir = MOTOR_STOP;
            return; 
        }
    }

    if(cmd->direction == CCW && *limit_triggered) {
        // ESP_LOGI(TAG, "Moving CCW (Up), releasing limit status.");
        *limit_triggered = false;
    }

    // 2. 开启硬件
    gpio_set_level(EN_PIN, 0); 
    gpio_set_level(DIR_PIN, cmd->direction == CCW ? 1 : 0); 

    // 3. 脉冲循环
    for(int i = 0; i < cmd->steps; i++) {

        // 监测 Monitor 任务标志位
        if (*limit_triggered && cmd->direction == CW) {
            // ESP_LOGW(TAG, "监测到限位标志触发，停止脉冲输出！");
            break; 
        }

        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_DELAY_US); 
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(STEP_DELAY_US); 

        // 坐标更新
        if (STEP_PIN == STEP_PIN_1) {
            if (cmd->direction == CCW) motor1_position_steps++;
            else motor1_position_steps--;
        } else if (STEP_PIN == STEP_PIN_2) {
            if (cmd->direction == CCW) motor2_position_steps++;
            else motor2_position_steps--;
        }
    }

    // 5. 结束清理
    gpio_set_level(EN_PIN, 1); 
    *current_dir = MOTOR_STOP; 
    // ESP_LOGD(TAG, "Motion complete."); 
}

//电机1主任务
QueueHandle_t motor1_queue = NULL;

void motor1_task(void *pvParameters) 
{
    motor1_gpio_init();
    motor1_queue = xQueueCreate(5, sizeof(motor_cmd_t));
    motor_cmd_t cmd;

    while (1) {
        if (xQueueReceive(motor1_queue, &cmd, portMAX_DELAY)) {
            switch (cmd.cmd_type) {
                case MOTOR_CMD_MOVE_STEPS:
                    //传入 STOP_PIN_1
                    motor_move_loop("MOTOR1", &cmd,
                                    STEP_PIN_1, DIR_PIN_1, EN_PIN_1, STOP_PIN_1,
                                    &limit_triggered_1, &motor1_current_dir);
                    break;
                case MOTOR_CMD_MOVE_TO_MM:
                    motor_move_to_mm_impl(MOTOR1, cmd.target_mm);
                    break;
                case MOTOR_CMD_POSITION:
                    motor_position_impl(MOTOR1); 
                    break;
                case MOTOR_CMD_HOMING:
                    motor_homing_impl(MOTOR1);
                    break;
                default: break;
            }
        }
    }
}


//电机2主任务
QueueHandle_t motor2_queue = NULL;

void motor2_task(void *pvParameters) 
{
    motor2_gpio_init();
    motor2_queue = xQueueCreate(5, sizeof(motor_cmd_t));
    motor_cmd_t cmd;

    while (1) {
        if (xQueueReceive(motor2_queue, &cmd, portMAX_DELAY)) {
            switch (cmd.cmd_type) {
                case MOTOR_CMD_MOVE_STEPS:
                    // 传入 STOP_PIN_2
                    motor_move_loop("MOTOR2", &cmd,
                                    STEP_PIN_2, DIR_PIN_2, EN_PIN_2, STOP_PIN_2,
                                    &limit_triggered_2, &motor2_current_dir);
                    break;
                case MOTOR_CMD_MOVE_TO_MM:
                    motor_move_to_mm_impl(MOTOR2, cmd.target_mm);
                    break;
                case MOTOR_CMD_POSITION:
                    motor_position_impl(MOTOR2);
                    break;
                case MOTOR_CMD_HOMING:
                    motor_homing_impl(MOTOR2);
                    break;
                default: break;
            }
        }
    }
}

//1234电机发送命令
//16细分（3200步/圈）：1mm = 0.5圈 = 1600步    8细分（1600步/圈）：1mm = 0.5圈 = 800步
void motor_send_command(motor_id_t motor_id, motor_dir_t direction, int steps)
{
    motor_cmd_t cmd = 
    {
        .cmd_type  = MOTOR_CMD_MOVE_STEPS,   
        .direction = direction,
        .steps     = steps,
        .target_mm = 0.0f                    // 防止脏数据
    };

    QueueHandle_t target_queue = NULL;

    if (motor_id == MOTOR1) 
    {
        target_queue = motor1_queue;
    } 
    else if (motor_id == MOTOR2) 
    {
        target_queue = motor2_queue;
    }
    else if (motor_id == MOTOR3) 
    {
        target_queue = motor3_queue;
    }
    else if (motor_id == MOTOR4) 
    {
        target_queue = motor4_queue;
    }

    if (target_queue) 
    {
        xQueueSend(target_queue, &cmd, 0);
    } 
    else 
    {
        // ESP_LOGE("MOTOR_CMD", "Invalid motor ID or queue not ready.");
    }
}

//mm输入控制1234
void motor_send_command_mm(motor_id_t motor_id, motor_dir_t direction, float mm)
{
    float steps_per_mm;
    //motor1 motor2丝杠控制
    if (motor_id == MOTOR1)
        steps_per_mm = (200 * motor1_microstep) / 2.0f;

    else if (motor_id == MOTOR2)
        steps_per_mm = (200 * motor2_microstep) / 2.0f;

    //motor3 motor4挤出机控制
    else if (motor_id == MOTOR3)
        steps_per_mm = (200*motor3_microstep) / (3.14159f * 10.0f);

    else if (motor_id == MOTOR4)
        steps_per_mm = (200*motor4_microstep) / (3.14159f * 10.0f);

    else 
    {
        // ESP_LOGE("MOTOR_CMD", "Invalid motor ID for mm conversion.");
        return;
    }

    int steps = (int)(mm * steps_per_mm);  
    motor_send_command(motor_id, direction, steps);
}



//XY轴电机就位专用函数（含有绝对位置信息）
void motor_move_to_mm_impl(motor_id_t motor_id, float target_mm)
{
    float steps_per_mm = 0.0f;
    float current_mm = 0.0f;

    if (motor_id == MOTOR1) 
    {
        steps_per_mm = (200 * motor1_microstep) / 2.0f;
        current_mm = motor1_position_mm();
    } 
    else if (motor_id == MOTOR2) 
    {
        steps_per_mm = (200 * motor2_microstep) / 2.0f;
        current_mm = motor2_position_mm();
    }

    float delta_mm = target_mm - current_mm;
    int steps_needed = (int)(delta_mm * steps_per_mm);

    if (steps_needed == 0) return;

    motor_dir_t dir = steps_needed > 0 ? CCW : CW; //需要步数＞0时，CCW（向上

    motor_send_command(motor_id, dir, abs(steps_needed));//复用

    // ESP_LOGI("MOVE_TO_MM", "MOTOR%d move from %.2f mm to %.2f mm (%d steps)", motor_id == MOTOR1 ? 1 : 2, current_mm, target_mm, steps_needed);
}

void motor_move_to_mm(motor_id_t motor_id, float target_mm)
{
    // 只支持 MOTOR1 / MOTOR2
    if (motor_id != MOTOR1 && motor_id != MOTOR2) {
        // ESP_LOGE("MOVE_TO_MM", "Invalid motor ID: %d", motor_id);
        return;
    }

    QueueHandle_t q = (motor_id == MOTOR1) ? motor1_queue : motor2_queue;

    if (q == NULL) {
        // ESP_LOGE("MOVE_TO_MM", "Queue not ready");
        return;
    }

    motor_cmd_t cmd = {
        .cmd_type  = MOTOR_CMD_MOVE_TO_MM,
        .direction = MOTOR_STOP,   // 无关
        .steps     = 0,
        .target_mm = target_mm
    };

    xQueueSend(q, &cmd, portMAX_DELAY);

    // 如果想完全阻塞，等动作完成，你可以简单等一段时间或者检查位置：
    // （简易方案：轮询当前位置，直到接近 target_mm）
    while (1) {
        float pos = (motor_id == MOTOR1) ? motor1_position_mm() : motor2_position_mm();
        if (fabsf(pos - target_mm) < 0.01f) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// --- 任务内部实现 ---
void motor_position_impl(motor_id_t motor_id)
{
    // 变量指针绑定 
    int STEP_PIN, DIR_PIN, EN_PIN, STOP_PIN;
    volatile int *position_steps;
    volatile bool *limit_triggered;
    bool *motor_homed;
    volatile motor_dir_t *current_dir_ptr;
    int microstep;

    if (motor_id == MOTOR1) {
        STEP_PIN = STEP_PIN_1; 
        DIR_PIN = DIR_PIN_1; 
        EN_PIN = EN_PIN_1; 
        STOP_PIN = STOP_PIN_1;
        position_steps = &motor1_position_steps; 
        limit_triggered = &limit_triggered_1;
        motor_homed = &motor1_homed; 
        current_dir_ptr = &motor1_current_dir;
        microstep = motor1_microstep;
    } else {
        STEP_PIN = STEP_PIN_2; 
        DIR_PIN = DIR_PIN_2; 
        EN_PIN = EN_PIN_2; 
        STOP_PIN = STOP_PIN_2;
        position_steps = &motor2_position_steps; 
        limit_triggered = &limit_triggered_2;
        motor_homed = &motor2_homed; 
        current_dir_ptr = &motor2_current_dir;
        microstep = motor2_microstep;
    }

    // 1. 初始化
    *limit_triggered = false; 
    *current_dir_ptr = CW; // 标记正在向下

    // 2. 启动前检查：如果已经在限位上，直接成功返回
    if (gpio_get_level(STOP_PIN) == 0) {
        // ESP_LOGI("POSITION_IMPL", "启动时检测到限位已闭合，直接设为原点。");
        *position_steps = 0;
        *limit_triggered = true;
        *motor_homed = true;
        *current_dir_ptr = MOTOR_STOP;
        return; // 不执行任何移动
    }

    gpio_set_level(EN_PIN, 0);
    gpio_set_level(DIR_PIN, 0); // CW 向下

    int max_steps = (int)(130.0f * 200 * microstep / 2.0f);
    
    // 3. 运动循环
    for (int i = 0; i < max_steps; i++) {
        // 实时检测限位
        if (gpio_get_level(STOP_PIN) == 0) {
            // ESP_LOGI("POSITION_IMPL", "触碰限位，停止。");
            *limit_triggered = true;
            break;
        }
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_DELAY_US);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(STEP_DELAY_US);
    }

    // 4. 结算
    if (*limit_triggered) {
        *position_steps = 0;
        *motor_homed = true;
    } else {
        // ESP_LOGE("POSITION_IMPL", "找零失败：行程超出最大范围");
        *motor_homed = false;
    }

    *current_dir_ptr = MOTOR_STOP;
    gpio_set_level(EN_PIN, 1);
}

// --- 外部调用封装 ---
void motor_position(motor_id_t motor_id)
{
    if (motor_id != MOTOR1 && motor_id != MOTOR2) return;

    // 1. 互锁安全检查
    if (!is_other_axis_safe(motor_id)) return;

    // 2. 清除标志位，防止 Race 
    if (motor_id == MOTOR1) motor1_homed = false;
    else motor2_homed = false;

    QueueHandle_t q = (motor_id == MOTOR1) ? motor1_queue : motor2_queue;
    motor_cmd_t cmd = { .cmd_type = MOTOR_CMD_POSITION, .direction = MOTOR_STOP };
    xQueueSend(q, &cmd, portMAX_DELAY);

    // 3. 阻塞等待
    int timeout_ms = 15000;
    int waited = 0;
    while (1) {
        bool homed = (motor_id == MOTOR1) ? motor1_homed : motor2_homed;
        if (homed) break;
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
        if (waited > timeout_ms) {
            // ESP_LOGE("POSITION", "指令超时！");
            break;
        }
    }
}


// --- 任务内部实现 ---
void motor_homing_impl(motor_id_t motor_id)
{
    bool homed = (motor_id == MOTOR1) ? motor1_homed : motor2_homed;
    
    // 必须已归零才能基于坐标移动
    if (!homed) {
        // ESP_LOGW("HOMING_IMPL", "未归零，忽略抬起指令");
        return;
    }

    // 直接调用 _impl 函数，向队列追加移动指令
    motor_move_to_mm_impl(motor_id, SAFE_TARGET_MM); 
}

// --- 外部调用封装 ---
void motor_homing(motor_id_t motor_id)
{
    if (motor_id != MOTOR1 && motor_id != MOTOR2) return;

    // 1. 定义临时指针和变量，用于操作对应电机的状态
    bool *homed_ptr;
    volatile int *steps_ptr;
    int STOP_PIN;

    if (motor_id == MOTOR1) {
        homed_ptr = &motor1_homed;
        steps_ptr = &motor1_position_steps;
        STOP_PIN  = STOP_PIN_1;
    } else {
        homed_ptr = &motor2_homed;
        steps_ptr = &motor2_position_steps;
        STOP_PIN  = STOP_PIN_2;
    }

    // 软件标志
    if (!(*homed_ptr)) {
        // 但是物理引脚显示已压住限位
        if (gpio_get_level(STOP_PIN) == 0) {
            // ESP_LOGW("HOMING", "上电检测到位于限位处，自动建立坐标系。");
            // 强制校准坐标
            *steps_ptr = 0;
            *homed_ptr = true; 
            // 此时状态已修复，程序会继续往下执行发送指令
        } 
        else {
            // 既没归零，也没在限位上，那才是真正的“位置未知”
            // ESP_LOGW("HOMING", "未归零且不在限位，拒绝指令");
            return;
        }
    }

    // 3. 发送指令
    QueueHandle_t q = (motor_id == MOTOR1) ? motor1_queue : motor2_queue;
    motor_cmd_t cmd = { .cmd_type = MOTOR_CMD_HOMING };
    xQueueSend(q, &cmd, portMAX_DELAY);

    // 4. 阻塞等待到位 (保持不变)
    int timeout_ms = HOME_TIMEOUT_MS;
    int waited = 0;
    while (1) {
        float pos = (motor_id == MOTOR1) ? motor1_position_mm() : motor2_position_mm();
        
        if (pos >= SAFE_SWITCH_HEIGHT_MM) break;

        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
        waited += CHECK_INTERVAL_MS;
        
        if (waited > timeout_ms) {
             // ESP_LOGE("HOMING", "抬起超时！当前位置: %.2f", pos);
             break;
        }
    }

    update_ready_if_safe();
}


// 读取当前 X 轴实际位置 (mm)
float get_X_position_mm(void)
{
    
     if (!motor1_homed) { return 0; } 

    return motor1_position_mm();
}

// 读取当前 Y 轴实际位置 (mm)
float get_Y_position_mm(void)
{
     if (!motor2_homed) { return 0; }

    return motor2_position_mm();
}

static bool axes_ready_for_switch(void)
{
    update_ready_if_safe();
    return motor1_ready && motor2_ready;
}

// 切换到纤维轴 (使用 Motor1)
void switch_to_CF(void)
{
    // 1. 初始化检查：必须两个轴都 ready
    if (!motor1_ready || !motor2_ready) 
    {
        if (!axes_ready_for_switch()) {
            // ESP_LOGE("SWITCH", 
                     // "切换到纤维轴失败：电机未完成初始化 (M1_ready=%d, M2_ready=%d)",
                     // motor1_ready, motor2_ready);
            return;
        }
    }

    // 2. 读取两个限位开关电平：0 = 触发 = 在工作位(position)
    int sw1_level = gpio_get_level(STOP_PIN_1);
    int sw2_level = gpio_get_level(STOP_PIN_2);

    bool motor1_at_work = (sw1_level == 0); // Motor1 压在限位开关上
    bool motor2_at_work = (sw2_level == 0); // Motor2 压在限位开关上

    // 3. 如果目标轴 Motor1 已经在工作位，直接提示并返回
    if (motor1_at_work)
    {
        // ESP_LOGI("SWITCH",
                 // "切换到纤维轴：Motor1 已经在工作位(限位开关闭合)，无需动作。");
        return;
    }

    // 4. 如果另一个轴 Motor2 在工作位，先抬起再切换
    if (motor2_at_work)
    {
        // ESP_LOGI("SWITCH",
                 // "切换到纤维轴：Motor2 在工作位(限位开关闭合)，先执行 homing 抬起避让。");
        motor_homing(2);   // 抬到 SAFE_TARGET_MM；内部自己等待到 SAFE_SWITCH_HEIGHT_MM
    }

    // 5. 现在可以安全地下探 Motor1 到工作位
    // ESP_LOGI("SWITCH", "切换到纤维轴：开始执行 Motor1 的 position 寻找工作位。");
    motor_position(1);     // 内部有 is_other_axis_safe 互锁检查
}

// 切换到树脂轴 
void switch_to_RESIN(void)
{
    // 1. 初始化检查：必须两个轴都 ready
    if (!motor1_ready || !motor2_ready) 
    {
        if (!axes_ready_for_switch()) {
            // ESP_LOGE("SWITCH", 
                     // "切换到树脂轴失败：电机未完成初始化 (M1_ready=%d, M2_ready=%d)",
                     // motor1_ready, motor2_ready);
            return;
        }
    }

    // 2. 读取两个限位开关电平：0 = 触发 = 在工作位(position)
    int sw1_level = gpio_get_level(STOP_PIN_1);
    int sw2_level = gpio_get_level(STOP_PIN_2);

    bool motor1_at_work = (sw1_level == 0); // Motor1 压在限位开关上
    bool motor2_at_work = (sw2_level == 0); // Motor2 压在限位开关上

    // 3. 如果目标轴 Motor2 已经在工作位，直接提示并返回
    if (motor2_at_work)
    {
        // ESP_LOGI("SWITCH",
                 // "切换到树脂轴：Motor2 已经在工作位(限位开关闭合)，无需动作。");
        return;
    }

    // 4. 如果另一个轴 Motor1 在工作位，先抬起再切换
    if (motor1_at_work)
    {
        // ESP_LOGI("SWITCH",
                 // "切换到树脂轴：Motor1 在工作位(限位开关闭合)，先执行 homing 抬起避让。");
        motor_homing(1);
    }

    // 5. 现在可以安全地下探 Motor2 到工作位
    // ESP_LOGI("SWITCH", "切换到树脂轴：开始执行 Motor2 的 position 寻找工作位。");
    motor_position(2);
}

// 电机初始化 完成后XY轴都在安全位置 返回ready标志
void Motor_INI(void)
{
    // ESP_LOGI("INIT", "开始执行电机初始化...");

    motor1_ready = false;
    motor2_ready = false;
    motor3_ready = false;
    motor4_ready = false;

    // 1. 读取两个限位开关的物理电平 (假设 0 为触发/闭合)
    int level1 = gpio_get_level(STOP_PIN_1);
    int level2 = gpio_get_level(STOP_PIN_2);
    
    bool sw1_triggered = (level1 == 0);
    bool sw2_triggered = (level2 == 0);

    // ESP_LOGI("INIT", "传感器状态: M1=%d, M2=%d", sw1_triggered, sw2_triggered);

    // 2. 异常情况：没有任何一个轴在限位上
    // 此时软件不敢盲目乱动，必须人工介入。
    if (!sw1_triggered && !sw2_triggered)
    {
        // ESP_LOGE("INIT", "【危险】系统复位失败！两个限位均未触发。");
        // ESP_LOGE("INIT", "位置未知，为防止撞机，请手动将至少一个轴旋至限位处");
        return; // 直接退出，不做任何电机动作
    }

    // 3. 处理逻辑
    // 策略：优先处理“已经触发限位”的那个轴，因为它位置已知，可以安全抬起。
    
    // --- 情况 A: Motor 1 在限位处 ---
    if (sw1_triggered)
    {
        // ESP_LOGI("INIT", "检测到 Motor1 在限位处。步骤1：抬起 Motor1。");

        // 手动建立 M1 坐标系
        // 因为刚上电 homed 为 false，不手动置位的话 motor_homing 会拒绝执行
        motor1_position_steps = 0;
        motor1_homed = true; 
        
        // 1. 让 Motor1 抬起
        motor_homing(1);

        // 2. 等待 Motor1 到达安全高度 (防撞)
        int timeout = 0;
        while (motor1_position_mm() < SAFE_SWITCH_HEIGHT_MM)
        {
            vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
            if (++timeout > (HOME_TIMEOUT_MS / CHECK_INTERVAL_MS)) {
                // ESP_LOGE("INIT", "Motor1 抬起超时，初始化中止！");
                return;
            }
        }
        // ESP_LOGI("INIT", "Motor1 已安全抬起。步骤2：复位 Motor2。");

        // 3. 处理 Motor2：先找零，再抬起
        // 此时 Motor1 已在上面，Motor2 可以放心大胆地向下找零
        motor_position(2); // 找零 (阻塞执行)
        
        if (motor2_homed) {
             // ESP_LOGI("INIT", "Motor2 找零成功，正在抬起...");
             motor_homing(2); // 抬起 (非阻塞)
             // (可选) 如果需要在这里死等 M2 到位，可以加 while 循环，
             // 但通常初始化发令后即可结束函数，让电机在后台跑。
        } else {
             // ESP_LOGE("INIT", "Motor2 找零失败！");
        }
    }
    // --- 情况 B: Motor 2 在限位处 (且 M1 没在) ---
    else if (sw2_triggered)
    {
        // ESP_LOGI("INIT", "检测到 Motor2 在限位处。步骤1：抬起 Motor2。");

        // [关键步骤] 手动建立 M2 坐标系
        motor2_position_steps = 0;
        motor2_homed = true;

        // 1. 让 Motor2 抬起
        motor_homing(2);

        // 2. 等待 Motor2 到达安全高度
        int timeout = 0;
        while (motor2_position_mm() < SAFE_SWITCH_HEIGHT_MM)
        {
            vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
            if (++timeout > (HOME_TIMEOUT_MS / CHECK_INTERVAL_MS)) {
                // ESP_LOGE("INIT", "Motor2 抬起超时，初始化中止！");
                return;
            }
        }
        // ESP_LOGI("INIT", "Motor2 已安全抬起。步骤2：复位 Motor1。");

        // 3. 处理 Motor1：先找零，再抬起
        motor_position(1); // 找零
        
        if (motor1_homed) {
             // ESP_LOGI("INIT", "Motor1 找零成功，正在抬起...");
             motor_homing(1); // 抬起
        } else {
             // ESP_LOGE("INIT", "Motor1 找零失败！");
        }
    }

    // ESP_LOGI("INIT", "系统初始化指令发送完成。双轴将停留在安全高度。");

    float X = motor1_position_mm();
    motor1_ready = motor1_homed && (X >= SAFE_SWITCH_HEIGHT_MM);

    float Y = motor2_position_mm();
    motor2_ready = motor2_homed && (Y >= SAFE_SWITCH_HEIGHT_MM);

    // Motor3 / Motor4：只要队列存在（任务已经启动），就认为 ready
    motor3_ready = (motor3_queue != NULL);
    motor4_ready = (motor4_queue != NULL);
    
    // ESP_LOGI("INIT",
        // "系统初始化完成: M1_ready=%d, M2_ready=%d, M3_ready=%d, M4_ready=%d",
        // motor1_ready, motor2_ready, motor3_ready, motor4_ready);
}

//纤维挤出参数mm
float fiber_w = 2.0f;//线宽
float fiber_h = 0.2f;//层高
float fiber_A_target; //= fiber_w * fiber_h; //目标打印截面积（近似矩形）
float fiber_d = 0.4f; //纤维耗材线径
float fiber_A_filament; //= 3.14159f * (fiber_d / 2.0f) * (fiber_d / 2.0f);//纤维耗材截面积（圆形）
float fiber_extrusion_ratio; //= fiber_A_target / fiber_A_filament;//挤出比例

//纤维挤出机 motor3
//滚轴直径5mm 步距角1.8° 
//每圈步数200*motor3_microstep  每圈挤出长度3.14159f * 5.0f(轴直径5mm)
//steps_per_mm = (200*motor3_microstep) / (3.14159f * 5.0f)

//const float MOTOR_EXTRUDER_DIAMETER_MM = 5.0f;

//树脂挤出参数
float resin_w = 2.0f;
float resin_h = 0.2f;
float resin_A_target;//= resin_w * resin_h;
float resin_d = 3.0f;
float resin_A_filament;//= 3.14159f * (resin_d / 2.0f) * (resin_d / 2.0f);
float resin_extrusion_ratio; //= resin_A_target / resin_A_filament;

//树脂挤出机 motor4 
//每圈步数200*motor4_microstep
//steps_per_mm = (200*motor4_microstep) / (3.14159f * 5.0f)

void extruder_parameters_init(void)
{
    static const char *TAG = "EXTRUDER_PARAMS"; // Log tag
    (void)TAG;

    // --- 3. 计算纤维参数 ---
    fiber_A_target = fiber_w * fiber_h;
    fiber_A_filament = 3.14159f * (fiber_d / 2.0f) * (fiber_d / 2.0f);
    
    // (安全检查，防止除以零)
    if (fiber_A_filament > 0.001f) {
        fiber_extrusion_ratio = fiber_A_target / fiber_A_filament;
    } else {
        fiber_extrusion_ratio = 0.0f;
    }

    // --- 4. 计算树脂参数 ---
    resin_A_target = resin_w * resin_h;
    resin_A_filament = 3.14159f * (resin_d / 2.0f) * (resin_d / 2.0f);
    
    if (resin_A_filament > 0.001f) {
        resin_extrusion_ratio = resin_A_target / resin_A_filament;
    } else {
        resin_extrusion_ratio = 0.0f;
    }

    // --- 5. 打印结果用于调试 ---
    // ESP_LOGI(TAG, "挤出机参数已计算:");
    // ESP_LOGI(TAG, "  Fiber (M3): W=%.2f, H=%.2f, D=%.2f", fiber_w, fiber_h, fiber_d);
    // ESP_LOGI(TAG, "  Fiber Ratio: %.4f (Target A: %.4f, Filament A: %.4f)", 
             // fiber_extrusion_ratio, fiber_A_target, fiber_A_filament);
    
    if (fiber_d < 1.0f) {
        // ESP_LOGW(TAG, "  [警告!] 纤维耗材直径 (fiber_d) = %.2f mm. 这看起来太小了!", fiber_d);
        // ESP_LOGW(TAG, "  请确认这不是喷嘴直径。常见的耗材直径是 1.75 mm。");
    }
    
    // ESP_LOGI(TAG, "  Resin (M4): W=%.2f, H=%.2f, D=%.2f", resin_w, resin_h, resin_d);
    // ESP_LOGI(TAG, "  Resin Ratio: %.4f (Target A: %.4f, Filament A: %.4f)",
             // resin_extrusion_ratio, resin_A_target, resin_A_filament);
}


QueueHandle_t motor3_queue = NULL;
QueueHandle_t motor4_queue = NULL;

void motor3_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_PIN_3) | (1ULL << DIR_PIN_3) | (1ULL << EN_PIN_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(STEP_PIN_3, 0);
    gpio_set_level(DIR_PIN_3, 0);
    gpio_set_level(EN_PIN_3, 1);  // TMC2209低电平使能, 初始化时默认关闭使能
}

void motor3_task(void *pvParameters) 
{
    motor3_gpio_init();
    motor3_queue = xQueueCreate(5, sizeof(motor_cmd_t));
    
    motor_cmd_t cmd;      // 用于接收启动命令
    motor_cmd_t stop_cmd; // 用于检查中断命令

    while(1) 
    {

        if(xQueueReceive(motor3_queue, &cmd, portMAX_DELAY)) 
        {
            // 2. 收到命令，检查它
            
            // 如果是 STOP 命令, 或者 0 步, 忽略它, 继续空闲
            if (cmd.direction == MOTOR_STOP || cmd.steps <= 0) {
                gpio_set_level(EN_PIN_3, 1); // 保持电机关闭
                continue; 
            }

            // 3. 状态: 运动 (MOVING)
            // ESP_LOGI("MOTOR3", "开始运动: %d 步, 方向: %d", cmd.steps, cmd.direction);
            
            gpio_set_level(EN_PIN_3, 0); // 使能电机
            gpio_set_level(DIR_PIN_3, cmd.direction == CCW ? 1 : 0); 
            
            int steps_remaining = cmd.steps;
            //判断挤出还是回抽，选择速度
            uint32_t delay_us = (cmd.direction == CCW) ? g_motor3_step_delay_us : g_motor3_retract_step_delay_us;//motor3插头线序特殊，控制方向与其他电机相反

            // 4. 可中断的脉冲循环
            while (steps_remaining > 0) 
            {
                // 4a. 发送一个脉冲
                gpio_set_level(STEP_PIN_3, 1);
                esp_rom_delay_us(delay_us); 
                gpio_set_level(STEP_PIN_3, 0);
                esp_rom_delay_us(delay_us); 
                
                steps_remaining--;

                // 4b. (实时停止) 非阻塞地检查队列
                if (xQueueReceive(motor3_queue, &stop_cmd, 0) == pdTRUE) 
                {
                    
                    if (stop_cmd.direction == MOTOR_STOP) {
                        // ESP_LOGW("MOTOR3", "运动被 STOP 命令中断!");
                        steps_remaining = 0; // 强制退出循环
                        // 不需要 break, 循环会在下次检查时自动停止
                    } 
                }
            } 

          
            
            gpio_set_level(EN_PIN_3, 1); 
            // ESP_LOGI("MOTOR3", "运动完成。");
            
        }
    }
}

void motor4_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_PIN_4) | (1ULL << DIR_PIN_4) | (1ULL << EN_PIN_4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(STEP_PIN_4, 0);
    gpio_set_level(DIR_PIN_4, 0);
    gpio_set_level(EN_PIN_4, 1); 
}

void motor4_task(void *pvParameters) 
{
    motor4_gpio_init();
    motor4_queue = xQueueCreate(5, sizeof(motor_cmd_t));
    
    motor_cmd_t cmd;
    motor_cmd_t stop_cmd; 

    while(1) 
    {
        // 1. 状态: 空闲
        if(xQueueReceive(motor4_queue, &cmd, portMAX_DELAY)) 
        {
            // 2. 检查命令
            if (cmd.direction == MOTOR_STOP || cmd.steps <= 0) {
                gpio_set_level(EN_PIN_4, 1);
                continue;
            }

            // 3. 状态: 运动
            // ESP_LOGI("MOTOR4", "开始运动: %d 步, 方向: %d", cmd.steps, cmd.direction);
            gpio_set_level(EN_PIN_4, 0); 
            gpio_set_level(DIR_PIN_4, cmd.direction == CCW ? 1 : 0); 
            
            int steps_remaining = cmd.steps;

            //判断挤出还是回抽，选择速度
            uint32_t delay_us = (cmd.direction == CCW) ? g_motor4_step_delay_us : g_motor4_retract_step_delay_us;//树脂挤出机，逆时针挤出，顺时针回抽

            // 4. (关键) 可中断的脉冲循环
            while (steps_remaining > 0) 
            {
                // 4a. 发送一个脉冲
                gpio_set_level(STEP_PIN_4, 1);
                esp_rom_delay_us(delay_us); 
                gpio_set_level(STEP_PIN_4, 0);
                esp_rom_delay_us(delay_us); 
                steps_remaining--;

                // 4b. (实时停止) 非阻塞地检查队列
                if (xQueueReceive(motor4_queue, &stop_cmd, 0) == pdTRUE) 
                {
                    if (stop_cmd.direction == MOTOR_STOP) {
                        // ESP_LOGW("MOTOR4", "运动被 STOP 命令中断!");
                        steps_remaining = 0; 
                    } 
                }
            } 

            // 5. 状态: 清理
            gpio_set_level(EN_PIN_4, 1); 
            // ESP_LOGI("MOTOR4", "运动完成。");
        }
    }
}

// 从速度换算 delay_us
static uint32_t calc_step_delay_from_speed(float v_mm_s, int microstep)
{
    // 防止 0 或负数
    if (v_mm_s <= 0.0f) {
    
        return STEP_DELAY_US;
    }

    // 每 mm 的步数：挤出机结构写死：200 步/圈，轴直径 10mm
    float steps_per_mm = (200.0f * (float)microstep) / (3.14159f * 10.0f);

    float f_step = v_mm_s * steps_per_mm;   // 步/秒

    // 限制最大步频，防止 delay 太小（
    if (f_step <= 0.0f) {
        return STEP_DELAY_US;
    }

    // 限制最大步频
    float f_step_max = 50000.0f;
    if (f_step > f_step_max) {
        f_step = f_step_max;
    }

    // delay_us = 1e6 / (2 * f_step)
    float delay_f = 1000000.0f / (2.0f * f_step);

    // 限制最小/最大 delay
    if (delay_f < 100.0f) {   
        delay_f = 100.0f;
    }
    if (delay_f > 1000000.0f) { // 太慢上限
        delay_f = 1000000.0f;
    }

    return (uint32_t)delay_f;
}


// --- 对外接口：按进料速度设置 motor3 ---
void motor3_set_speed_mm_per_s(float v_mm_s)
{
    g_motor3_extrude_speed_mm_s  = v_mm_s;
    uint32_t delay_us = calc_step_delay_from_speed(v_mm_s, motor3_microstep);
    motor3_set_step_delay_us(delay_us);
}

float motor3_get_speed_mm_per_s(void)
{
    return g_motor3_extrude_speed_mm_s ;
}


// --- 对外接口：按进料速度设置 motor4 ---
void motor4_set_speed_mm_per_s(float v_mm_s)
{
    g_motor4_extrude_speed_mm_s  = v_mm_s;
    uint32_t delay_us = calc_step_delay_from_speed(v_mm_s, motor4_microstep);
    motor4_set_step_delay_us(delay_us);
}

float motor4_get_speed_mm_per_s(void)
{
    return g_motor4_extrude_speed_mm_s ;
}

//纤维回抽接口
void motor3_set_retract_speed_mm_per_s(float v_mm_s)
{
    g_motor3_retract_speed_mm_s = v_mm_s;
    uint32_t delay_us = calc_step_delay_from_speed(v_mm_s, motor3_microstep);
    g_motor3_retract_step_delay_us = delay_us;
}

float motor3_get_retract_speed_mm_per_s(void)
{
    return g_motor3_retract_speed_mm_s;
}

//树脂回抽接口
void motor4_set_retract_speed_mm_per_s(float v_mm_s)
{
    g_motor4_retract_speed_mm_s = v_mm_s;
    uint32_t delay_us = calc_step_delay_from_speed(v_mm_s, motor4_microstep);
    g_motor4_retract_step_delay_us = delay_us;
}

float motor4_get_retract_speed_mm_per_s(void)
{
    return g_motor4_retract_speed_mm_s;
}
