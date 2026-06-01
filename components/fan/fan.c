#include "fan.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "FAN";

// 队列定义
QueueHandle_t fan_queue = NULL;

//风扇状态
static bool fan1_current_state = false;
static bool fan2_current_state = false;

// 风扇 ready 标志
volatile bool fan1_ready = false;
volatile bool fan2_ready = false;

static void fan_gpio_init(void)
{
    // 配置 FAN1_PIN
    gpio_config_t io_conf_fan1 = {
        .pin_bit_mask = (1ULL << FAN1_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_fan1);
    gpio_set_level(FAN1_PIN, 0); // 默认关闭

    // 配置 FAN2_PIN
    gpio_config_t io_conf_fan2 = {
        .pin_bit_mask = (1ULL << FAN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_fan2);
    gpio_set_level(FAN2_PIN, 0); // 默认关闭

    ESP_LOGI(TAG, "风扇 GPIO (Fan1: %d, Fan2: %d) 初始化完成", FAN1_PIN, FAN2_PIN);
}

bool fan_get_state(fan_id_t id)
{
    if (id == FAN1) {
        return fan1_current_state;
    } else if (id == FAN2) {
        return fan2_current_state;
    }
    return false; // 默认返回 false
}

void fan_task(void *pvParameters)
{
    fan_gpio_init();
  
    fan_queue = xQueueCreate(5, sizeof(fan_cmd_t)); 
    if (fan_queue == NULL) {
        ESP_LOGE(TAG, "创建风扇队列失败!");
        vTaskDelete(NULL);
    }

    fan_cmd_t cmd; 

    while (1) {
        // 3. 阻塞等待队列命令 
        if (xQueueReceive(fan_queue, &cmd, portMAX_DELAY)) {
            if (cmd.id == FAN1) {
                fan1_current_state = cmd.enable;
                gpio_set_level(FAN1_PIN, fan1_current_state ? 1 : 0);
                ESP_LOGI(TAG, "Fan 1 (纤维) 设置为: %s", fan1_current_state ? "ON" : "OFF");
            } else if (cmd.id == FAN2) {
                fan2_current_state = cmd.enable;
                gpio_set_level(FAN2_PIN, fan2_current_state ? 1 : 0);
                ESP_LOGI(TAG, "Fan 2 (树脂) 设置为: %s", fan2_current_state ? "ON" : "OFF");
            }
        }
    }
}

void fan_send_command(fan_id_t id) 
{
    if (fan_queue == NULL) {
        ESP_LOGE(TAG, "风扇队列未初始化!");
        return;
    }

    bool current = fan_get_state(id);
    fan_cmd_t cmd = {
        .id = id,
        .enable = !current
    };

    if (xQueueSend(fan_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "风扇队列已满, 命令发送失败!");
    }
}

void fan_set_state(fan_id_t id, bool enable)
{
    if (fan_queue == NULL) {
        ESP_LOGE(TAG, "风扇队列未初始化!");
        return;
    }

    fan_cmd_t cmd = {
        .id = id,
        .enable = enable
    };

    if (xQueueSend(fan_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "风扇队列已满, 命令发送失败!");
    }
}

void Fan_INI(void)
{
    // 判断 fan_task 是否已经创建并运行（fan_queue 是否创建成功）
    if (fan_queue == NULL) {
        ESP_LOGE(TAG, "Fan_INI: 风扇任务/队列未初始化，风扇未 ready!");
        fan1_ready = false;
        fan2_ready = false;
        return;
    }else{
        fan1_ready = true;
        fan2_ready = true;

        ESP_LOGI(TAG, "Fan_INI: 风扇初始化完成，Fan1_ready=%d, Fan2_ready=%d",
            fan1_ready, fan2_ready);
    }
    
}
