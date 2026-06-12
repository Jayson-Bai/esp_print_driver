/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <sys/param.h>
#include "esp_check.h"
#include "esp_log.h"
#include "pid_ctrl.h"

static const char *TAG = "pid_ctrl";

typedef struct pid_ctrl_block_t pid_ctrl_block_t;
typedef float (*pid_cal_func_t)(pid_ctrl_block_t *pid, float error);

//PID参数
struct pid_ctrl_block_t {
    float Kp; // PID Kp value
    float Ki; // PID Ki value
    float Kd; // PID Kd value
    float previous_err1; // e(k-1)
    float previous_err2; // e(k-2)
    float integral_err;  // Sum of error
    float last_output;  // PID output in last control period
    float max_output;   // PID maximum output limitation
    float min_output;   // PID minimum output limitation
    float max_integral; // PID maximum integral value limitation
    float min_integral; // PID minimum integral value limitation
    pid_cal_func_t calculate_func; // calculation function, depends on actual PID type set by user
};

//位置式PID计算
static float pid_calc_positional(pid_ctrl_block_t *pid, float error)
{
    float output = 0;
    /* Add current error to the integral error */
    pid->integral_err += error;
    /* If the integral error is out of the range, it will be limited */
    pid->integral_err = MIN(pid->integral_err, pid->max_integral);
    pid->integral_err = MAX(pid->integral_err, pid->min_integral);

    /* Calculate the pid control value by location formula */
    /* u(k) = e(k)*Kp + (e(k)-e(k-1))*Kd + integral*Ki */
    output = error * pid->Kp +
             (error - pid->previous_err1) * pid->Kd +
             pid->integral_err * pid->Ki;

    /* If the output is out of the range, it will be limited */
    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    /* Update previous error */
    pid->previous_err1 = error;

    return output;
}

//增量式PID
static float pid_calc_incremental(pid_ctrl_block_t *pid, float error)
{
    float output = 0;

    /* Calculate the pid control value by increment formula */
    /* du(k) = (e(k)-e(k-1))*Kp + (e(k)-2*e(k-1)+e(k-2))*Kd + e(k)*Ki */
    /* u(k) = du(k) + u(k-1) */
    output = (error - pid->previous_err1) * pid->Kp +
             (error - 2 * pid->previous_err1 + pid->previous_err2) * pid->Kd +
             error * pid->Ki +
             pid->last_output;

    /* If the output is beyond the range, it will be limited */
    output = MIN(output, pid->max_output);
    output = MAX(output, pid->min_output);

    /* Update previous error */
    pid->previous_err2 = pid->previous_err1;
    pid->previous_err1 = error;

    /* Update last output */
    pid->last_output = output;

    return output;
}

esp_err_t pid_new_control_block(const pid_ctrl_config_t *config, pid_ctrl_block_handle_t *ret_pid)
{
    esp_err_t ret = ESP_OK;
    pid_ctrl_block_t *pid = NULL;
    /* Check the input pointer */
    ESP_GOTO_ON_FALSE(config && ret_pid, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

    pid = calloc(1, sizeof(pid_ctrl_block_t));
    ESP_GOTO_ON_FALSE(pid, ESP_ERR_NO_MEM, err, TAG, "no mem for PID control block");
    ESP_GOTO_ON_ERROR(pid_update_parameters(pid, &config->init_param), err, TAG, "init PID parameters failed");
    *ret_pid = pid;
    return ret;

err:
    if (pid) {
        free(pid);
    }
    return ret;
}

esp_err_t pid_del_control_block(pid_ctrl_block_handle_t pid)
{
    ESP_RETURN_ON_FALSE(pid, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    free(pid);
    return ESP_OK;
}

esp_err_t pid_compute(pid_ctrl_block_handle_t pid, float input_error, float *ret_result)
{
    ESP_RETURN_ON_FALSE(pid && ret_result, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    *ret_result = pid->calculate_func(pid, input_error);
    return ESP_OK;
}

esp_err_t pid_update_parameters(pid_ctrl_block_handle_t pid, const pid_ctrl_parameter_t *params)
{
    ESP_RETURN_ON_FALSE(pid && params, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    pid->Kp = params->kp;
    pid->Ki = params->ki;
    pid->Kd = params->kd;
    pid->max_output = params->max_output;
    pid->min_output = params->min_output;
    pid->max_integral = params->max_integral;
    pid->min_integral = params->min_integral;
    /* Set the calculate function according to the PID type */
    switch (params->cal_type) {
    case PID_CAL_TYPE_INCREMENTAL:
        pid->calculate_func = pid_calc_incremental;
        break;
    case PID_CAL_TYPE_POSITIONAL:
        pid->calculate_func = pid_calc_positional;
        break;
    default:
        ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_ARG, TAG, "invalid PID calculation type:%d", params->cal_type);
    }
    return ESP_OK;
}

esp_err_t pid_reset_ctrl_block(pid_ctrl_block_handle_t pid)
{
    ESP_RETURN_ON_FALSE(pid, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    pid->integral_err = 0;
    pid->previous_err1 = 0;
    pid->previous_err2 = 0;
    pid->last_output = 0;
    return ESP_OK;
}



//PWM初始化配置
void PWM1_init()
{   
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HEATER_PIN_1,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // 初始拉低，避免误触发
    gpio_set_level(HEATER_PIN_1, 0);


    //timber初始化，两通道可共用
    ledc_timer_config_t timer_cfg = 
    {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_RESOLUTION,
    .timer_num = LEDC_TIMER_1,
    .freq_hz = 1000,    // PWM频率(Hz) 
    .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t err = ledc_timer_config(&timer_cfg);

    if (err != ESP_OK) 
    {
        // ESP_LOGE("PWM", "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return;
    }   
    else 
    {
    // ESP_LOGI("PWM", "PWM timer frequency: %lu Hz", timer_cfg.freq_hz);
    };


    //纤维通道ledc初始化
    ledc_channel_config_t channel1_cfg = 
    {
        .gpio_num = HEATER_PIN_1 ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL_1,    
        .intr_type = LEDC_INTR_DISABLE, 
        .timer_sel = LEDC_TIMER_1,     
        .duty = 0,                    // 初始占空比     范围[0,1 << LEDC_RESOLUTION]
        .hpoint = 0,      
    };

    err = ledc_channel_config(&channel1_cfg);

    if (err != ESP_OK) 
    {
        // ESP_LOGE("PWM", "Failed to configure LEDC channel: %s", esp_err_to_name(err));
        return;
    }
    else
    {
        // ESP_LOGI("PWM", "PWM channel duty: %lu", channel1_cfg.duty);
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_1);
        if (err != ESP_OK) 
        {
            // ESP_LOGE("PWM", "Failed to update LEDC duty: %s", esp_err_to_name(err));
            return;
        };
    };

    ledc_set_pin(HEATER_PIN_1, LEDC_LOW_SPEED_MODE, PWM_CHANNEL_1);

}


void PWM2_init()
{   
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << HEATER_PIN_2,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // 初始拉低，避免误触发
    gpio_set_level(HEATER_PIN_2, 0);

    //timber初始化，两通道可共用
    ledc_timer_config_t timer_cfg = 
    {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_RESOLUTION,
    .timer_num = LEDC_TIMER_1,
    .freq_hz = 1000,    // PWM频率(Hz) 
    .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t err = ledc_timer_config(&timer_cfg);

    if (err != ESP_OK) 
    {
        // ESP_LOGE("PWM", "Failed to configure LEDC timer: %s", esp_err_to_name(err));
        return;
    }   
    else 
    {
    // ESP_LOGI("PWM", "PWM timer frequency: %lu Hz", timer_cfg.freq_hz);
    };

    //树脂通道ledc初始化
    ledc_channel_config_t channel2_cfg = 
    {
        .gpio_num = HEATER_PIN_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL_2,               // 不同channel
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,              //共用timer
        .duty = 0,                           
        .hpoint = 0,
    };

    err = ledc_channel_config(&channel2_cfg);

    if (err != ESP_OK) 
    {
        // ESP_LOGE("PWM", "Failed to configure LEDC channel 2: %s", esp_err_to_name(err));
        return;
    } 
    else 
    {
        // ESP_LOGI("PWM", "PWM channel 2 duty: %lu", channel2_cfg.duty);
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_2);
        if (err != ESP_OK) 
        {
            // ESP_LOGE("PWM", "Failed to update LEDC duty (channel 2): %s", esp_err_to_name(err));
            return;
        }; 
    }

    ledc_set_pin(HEATER_PIN_2, LEDC_LOW_SPEED_MODE, PWM_CHANNEL_2);

}



// PID控制PWM占空比
esp_err_t pid_control_pwm_duty(
    pid_ctrl_block_handle_t *pid_handle,
    const pid_ctrl_parameter_t *init_param,
    float target_value,
    float current_value,
    ledc_mode_t speed_mode,
    ledc_channel_t channel,
    uint32_t *duty_out)
{
    ESP_RETURN_ON_FALSE(duty_out, ESP_ERR_INVALID_ARG, TAG, "Invalid duty pointer");
    ESP_RETURN_ON_FALSE(init_param, ESP_ERR_INVALID_ARG, TAG, "Invalid PID init parameter");

    esp_err_t err;

    if (*pid_handle == NULL) {
        pid_ctrl_config_t pid_config = {
            .init_param = *init_param,
        };
        err = pid_new_control_block(&pid_config, pid_handle);
        if (err != ESP_OK) {
            return err;
        }
    } else {
        pid_update_parameters(*pid_handle, init_param);
    }

    // 计算误差
    float error = target_value - current_value;
    float result = 0.0f;

    err = pid_compute(*pid_handle, error, &result);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "PID compute failed");
        return err;
    }

    // --------------------------------------------------------
    // 1. 输出限幅：保证在 [min_output, max_output] 之间
    // --------------------------------------------------------
    if (result > (*pid_handle)->max_output) {
        result = (*pid_handle)->max_output;
    } else if (result < (*pid_handle)->min_output) {
        result = (*pid_handle)->min_output;
    }

    // --------------------------------------------------------
    // 2. 绝对映射：PID输出(0~max_output) → PWM(0~(2^RES-1))
    //
    // 注意：这里使用 max_output 作为满量程，不再减去 min_output，
    // 所以：
    //   result = max_output  → 100% 占空比
    //   result = 0          → 0% 占空比（但通常被 min_output 限住）
    //   result = min_output → (min_output / max_output) 的占空比
    // --------------------------------------------------------
    float ratio = 0.0f;
    if ((*pid_handle)->max_output > 0.0f) {
        ratio = result / (*pid_handle)->max_output;
    }

    float full_scale = (float)((1 << LEDC_RESOLUTION) - 1);
    float mapped_duty_f = ratio * full_scale;

    // 安全检查
    if (mapped_duty_f > full_scale) {
        mapped_duty_f = full_scale;
    } else if (mapped_duty_f < 0.0f) {
        mapped_duty_f = 0.0f;
    }

    uint32_t duty = (uint32_t)mapped_duty_f;

    // --------------------------------------------------------
    // 3. 写入PWM
    // --------------------------------------------------------
    err = ledc_set_duty(speed_mode, channel, duty);
    if (err != ESP_OK) {
        return err;
    }
    err = ledc_update_duty(speed_mode, channel);
    if (err != ESP_OK) {
        return err;
    }

    *duty_out = duty;
    return ESP_OK;
}




//任务封装
QueueHandle_t PID1_queue = NULL;

extern float get_temperature1();

static portMUX_TYPE g_pid_spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile float g_pid_target_cf = 0.0f;
static volatile float g_pid_target_resin = 0.0f;
static volatile bool g_pid_enable_cf = false;
static volatile bool g_pid_enable_resin = false;

static pid_ctrl_parameter_t g_pid_params_cf = {
    .kp = 2.0f, .ki = 0.85f, .kd = 40.0f,
    .max_output = 100.0f, .min_output = 82.0f,
    .max_integral = 95.0f, .min_integral = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL
};

static pid_ctrl_parameter_t g_pid_params_resin = {
    .kp = 1.782f, .ki = 0.6f, .kd = 150.0f,
    .max_output = 100.0f, .min_output = 63.18f,
    .max_integral = 105.3f, .min_integral = 0.0f,
    .cal_type = PID_CAL_TYPE_POSITIONAL
};

static portMUX_TYPE g_pid_param_mux = portMUX_INITIALIZER_UNLOCKED;

void PID1_Task(void *pvParameters) {

    PID1_queue = xQueueCreate(5, sizeof(PID_cmd_t));

    PWM1_init();

    PID_cmd_t cmd;
    pid_ctrl_block_handle_t pid = NULL;
    float current;
    uint32_t duty;

    while (1) {
        
        if (xQueueReceive(PID1_queue, &cmd, portMAX_DELAY)) //阻塞等待
        {

            if (!cmd.enable) 
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_1, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_1);
                continue;
            }

            float target_temp = cmd.target_temp;
            // ESP_LOGI(TAG, "Start heating to %.2f ℃", target_temp);

            pid_ctrl_parameter_t local_params;

            TickType_t last_wake = xTaskGetTickCount();
            while (1) 
            {
                portENTER_CRITICAL(&g_pid_param_mux);
                local_params = g_pid_params_cf;
                portEXIT_CRITICAL(&g_pid_param_mux);

                current = get_temperature1();

                pid_control_pwm_duty(&pid,
                                     &local_params,
                                     target_temp,
                                     current,
                                     LEDC_LOW_SPEED_MODE,
                                     PWM_CHANNEL_1,
                                     &duty);

                // 检查是否收到停止加热命令（非阻塞,不影响PID的实时性）
                PID_cmd_t stop_cmd;
                if (xQueueReceive(PID1_queue, &stop_cmd, 0) == pdTRUE) {
                    if (!stop_cmd.enable) {
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_1, 0);
                        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_1);
                        break;
                    }
                    target_temp = stop_cmd.target_temp;
                }

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
            }
        }
    }
}


//PID2任务封装
QueueHandle_t PID2_queue = NULL;

extern float get_temperature2();

void PID2_Task(void *pvParameters) {

    PID2_queue = xQueueCreate(5, sizeof(PID_cmd_t));

    PWM2_init();

    PID_cmd_t cmd;
    pid_ctrl_block_handle_t pid = NULL;
    float current;
    uint32_t duty;

    while (1) {
        if (xQueueReceive(PID2_queue, &cmd, portMAX_DELAY)) {
            if (!cmd.enable) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_2, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_2);
                continue;
            }

            float target_temp = cmd.target_temp;
            // ESP_LOGI("PID2", "Start heating to %.2f ℃", target_temp);

            pid_ctrl_parameter_t local_params;

            TickType_t last_wake = xTaskGetTickCount();
            while (1) {
                portENTER_CRITICAL(&g_pid_param_mux);
                local_params = g_pid_params_resin;
                portEXIT_CRITICAL(&g_pid_param_mux);

                current = get_temperature2();

                pid_control_pwm_duty(&pid,
                                     &local_params,
                                     target_temp,
                                     current,
                                     LEDC_LOW_SPEED_MODE,
                                     PWM_CHANNEL_2,
                                     &duty);

                PID_cmd_t stop_cmd;
                if (xQueueReceive(PID2_queue, &stop_cmd, 0) == pdTRUE) {
                    if (!stop_cmd.enable) {
                        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_2, 0);
                        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_2);
                        break;
                    }
                    target_temp = stop_cmd.target_temp;
                }

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
            }
        }
    }
}



void PID_send_command(int channel, float target_temp, bool enable) //1纤维 2树脂
{
    PID_cmd_t cmd = {
        .target_temp = target_temp,
        .enable = enable,
    };

    QueueHandle_t target_queue = NULL;

    switch (channel) {
        case 1:
            target_queue = PID1_queue;
            portENTER_CRITICAL(&g_pid_spinlock);
            g_pid_target_cf = target_temp;
            g_pid_enable_cf = enable;
            portEXIT_CRITICAL(&g_pid_spinlock);
            break;
        case 2:
            target_queue = PID2_queue;
            portENTER_CRITICAL(&g_pid_spinlock);
            g_pid_target_resin = target_temp;
            g_pid_enable_resin = enable;
            portEXIT_CRITICAL(&g_pid_spinlock);
            break;
        default:
            // ESP_LOGE("PID", "Invalid channel number: %d", channel);
            return;
    }

    if (target_queue != NULL) {
        xQueueSend(target_queue, &cmd, 0);
    } else {
        // ESP_LOGW("PID", "Queue for channel %d is not ready", channel);
    }
}

void pid_set_params(int channel, float kp, float ki, float kd,
                    float max_output, float min_output,
                    float max_integral, float min_integral)
{
    portENTER_CRITICAL(&g_pid_param_mux);
    pid_ctrl_parameter_t *params;
    if (channel == 1) {
        params = &g_pid_params_cf;
    } else if (channel == 2) {
        params = &g_pid_params_resin;
    } else {
        portEXIT_CRITICAL(&g_pid_param_mux);
        return;
    }
    params->kp = kp;
    params->ki = ki;
    params->kd = kd;
    params->max_output = max_output;
    params->min_output = min_output;
    params->max_integral = max_integral;
    params->min_integral = min_integral;
    portEXIT_CRITICAL(&g_pid_param_mux);
}

float pid_get_target(int channel)
{
    float ret;
    portENTER_CRITICAL(&g_pid_spinlock);
    if (channel == 1) {
        ret = g_pid_target_cf;
    } else if (channel == 2) {
        ret = g_pid_target_resin;
    } else {
        ret = 0.0f;
    }
    portEXIT_CRITICAL(&g_pid_spinlock);
    return ret;
}

bool pid_get_enable(int channel)
{
    bool ret;
    portENTER_CRITICAL(&g_pid_spinlock);
    if (channel == 1) {
        ret = g_pid_enable_cf;
    } else if (channel == 2) {
        ret = g_pid_enable_resin;
    } else {
        ret = false;
    }
    portEXIT_CRITICAL(&g_pid_spinlock);
    return ret;
}
