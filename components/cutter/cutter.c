#include "cutter.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Cutter";

static QueueHandle_t cutter_queue = NULL;

//GPIO初始化
void cutter_gpio_init(void) {

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_num_t output_pins[] = { K1_PIN, K2_PIN, CUT_PIN, SW1_PIN, SW2_PIN };
    for (int i = 0; i < sizeof(output_pins) / sizeof(output_pins[0]); i++) {
        io_conf.pin_bit_mask = 1ULL << output_pins[i];
        gpio_config(&io_conf);
    }

    gpio_set_level(K1_PIN, 1);  //控制模块默认低电平触发
    gpio_set_level(K2_PIN, 1);
    gpio_set_level(CUT_PIN, 0); // MOS 默认关闭，高电平触发
    gpio_set_level(SW1_PIN, 1); 
    gpio_set_level(SW2_PIN, 1);

    ESP_LOGI(TAG, "GPIO 初始化完成：所有引脚默认高电平");
}


static bool sw1_locked = false;
static bool sw2_locked = false;


void cutter_task(void *pvParameters) {
    cutter_gpio_init();

    cutter_queue = xQueueCreate(10, sizeof(cutter_command_t));
    if (cutter_queue == NULL) {
        ESP_LOGE(TAG, "命令队列创建失败！");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Cutter 任务启动，等待命令...");

    cutter_command_t cmd;
    while (1) {
        if (xQueueReceive(cutter_queue, &cmd, portMAX_DELAY)) {
            gpio_num_t pin = cmd.pin;

            bool is_valid_pin =
                (pin == K1_PIN || pin == K2_PIN || pin == SW1_PIN || pin == SW2_PIN);

            if (!is_valid_pin) {
                ESP_LOGW(TAG, "无效 GPIO%d, 忽略命令", pin);
                continue;
            }

            // 处理 SW 引脚：触发后保持低电平，不自动恢复
            if (pin == SW1_PIN) {
                gpio_set_level(SW1_PIN, 0);  // 拉低保持
                sw1_locked = true;
                ESP_LOGI(TAG, "SW1 触发，保持低电平，等待解锁");
                continue;
            } else if (pin == SW2_PIN) {
                gpio_set_level(SW2_PIN, 0);  // 拉低保持
                sw2_locked = true;
                ESP_LOGI(TAG, "SW2 触发，保持低电平，等待解锁");
                continue;
            }

            // 处理 K1/K2 前，先解锁对应的 SW
            if (pin == K1_PIN && sw1_locked) {
                gpio_set_level(SW1_PIN, 1);  // 解锁
                sw1_locked = false;
                ESP_LOGI(TAG, "动作 K1 前解锁 SW1");
            }
            if (pin == K2_PIN && sw2_locked) {
                gpio_set_level(SW2_PIN, 1);  // 解锁
                sw2_locked = false;
                ESP_LOGI(TAG, "动作 K2 前解锁 SW2");
            }

            // 拉低 K 引脚 100ms，然后恢复
            ESP_LOGI(TAG, "触发 GPIO%d, 拉低 100ms", pin);
            gpio_set_level(pin, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(pin, 1);
            ESP_LOGI(TAG, "GPIO%d 恢复为高电平", pin);
        }
    }
}


void cutter_send_command(int PIN_NUM, bool state) {
    if (cutter_queue == NULL) {
        ESP_LOGW(TAG, "命令队列未初始化，忽略发送");
        return;
    }

    cutter_command_t cmd = {
        .pin = PIN_NUM,
        .state = state  
    };

    if (xQueueSend(cutter_queue, &cmd, 0) != pdPASS) {
        ESP_LOGW(TAG, "队列已满，无法发送 GPIO%d 的命令", PIN_NUM);
    }
}