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
static volatile bool cut_sequence_running = false;
static portMUX_TYPE cut_sequence_mux = portMUX_INITIALIZER_UNLOCKED;
static cutter_cut_done_cb_t cut_sequence_done_cb = NULL;
static void *cut_sequence_done_ctx = NULL;

static bool cutter_try_begin_cut_sequence(cutter_cut_done_cb_t done_cb, void *done_ctx)
{
    bool started = false;

    portENTER_CRITICAL(&cut_sequence_mux);
    if (!cut_sequence_running) {
        cut_sequence_running = true;
        cut_sequence_done_cb = done_cb;
        cut_sequence_done_ctx = done_ctx;
        started = true;
    }
    portEXIT_CRITICAL(&cut_sequence_mux);

    return started;
}

static void cutter_run_cut_sequence(void)
{
    cutter_send_command(K1_PIN, false);
    vTaskDelay(pdMS_TO_TICKS(7000));

    gpio_set_level(CUT_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(CUT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    cutter_send_command(K2_PIN, false);
}

static void cutter_finish_cut_sequence(void)
{
    cutter_cut_done_cb_t done_cb = NULL;
    void *done_ctx = NULL;

    portENTER_CRITICAL(&cut_sequence_mux);
    done_cb = cut_sequence_done_cb;
    done_ctx = cut_sequence_done_ctx;
    cut_sequence_done_cb = NULL;
    cut_sequence_done_ctx = NULL;
    cut_sequence_running = false;
    portEXIT_CRITICAL(&cut_sequence_mux);

    if (done_cb != NULL) {
        done_cb(done_ctx);
    }
}

static void cutter_cut_sequence_task(void *pvParameters)
{
    (void)pvParameters;
    cutter_run_cut_sequence();
    cutter_finish_cut_sequence();
    vTaskDelete(NULL);
}

bool cutter_start_cut_sequence_async(cutter_cut_done_cb_t done_cb, void *done_ctx)
{
    if (!cutter_try_begin_cut_sequence(done_cb, done_ctx)) {
        return false;
    }

    if (xTaskCreate(cutter_cut_sequence_task, "cut_sequence", 2048, NULL, 2, NULL) != pdPASS) {
        portENTER_CRITICAL(&cut_sequence_mux);
        cut_sequence_done_cb = NULL;
        cut_sequence_done_ctx = NULL;
        cut_sequence_running = false;
        portEXIT_CRITICAL(&cut_sequence_mux);
        return false;
    }

    return true;
}

bool cutter_start_cut_sequence_task(void)
{
    return cutter_start_cut_sequence_async(NULL, NULL);
}

bool cutter_run_cut_sequence_if_idle(void)
{
    if (!cutter_try_begin_cut_sequence(NULL, NULL)) {
        return false;
    }

    cutter_run_cut_sequence();
    cutter_finish_cut_sequence();
    return true;
}


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
