#include "motor.h"
#include "serial_reader.h"
#include "pid_ctrl.h"
#include "fan.h"
#include "temperature.h"
#include "extruder_ctrl.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/queue.h"

// 从 uart0 读取数据
int uart0_read(uint8_t *buf, int max_len, TickType_t ticks_to_wait)
{
    int len = uart_read_bytes(UART_PORT, buf, max_len - 1, ticks_to_wait);
    if (len > 0) {
        buf[len] = '\0'; // 确保以字符串结尾
    }
    return len;
}

// 向 uart0 发送数据
esp_err_t uart0_write(const uint8_t *data, int len)
{
    int res = uart_write_bytes(UART_PORT, (const char*)data, len);
    return (res == len) ? ESP_OK : ESP_FAIL;
}

static volatile int g_current_tool_id = 1; // 1=纤维, 2=树脂

typedef enum {
    UART_CMD_EXTRUDE = 0,
    UART_CMD_EVENT = 1,
} uart_cmd_type_t;

typedef struct {
    char line[256];
} uart_line_t;

typedef struct {
    uart_cmd_type_t type;
    int seq;
    int tool_id;
    float total_mm;
    char event_type[32];
    char arg[96];
} uart_cmd_t;

static QueueHandle_t uart_cmd_queue = NULL;
static QueueHandle_t uart_line_queue = NULL;
static QueueHandle_t uart_event_queue = NULL;
static volatile uint32_t uart_cmd_drop_count = 0;
static volatile uint32_t uart_line_drop_count = 0;

// 初始化 uart0
void uart0_init(int baud_rate)
{
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 30, &uart_event_queue, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ---------------- 工具函数：去尾空白 ----------------
static void trim(char *s)
{
    if (!s) return;
    int n = strlen(s);
    while (n > 0) {
        char c = s[n - 1];
        if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
            s[n - 1] = '\0';
            n--;
        } else {
            break;
        }
    }
}

static bool parse_extrude_cmd(const char *cmd, int *seq, int *tool_id, float *len_mm)
{
    if (!cmd || cmd[0] != 'E' || cmd[1] != ' ') {
        return false;
    }
    cmd += 2;
    char *end;

    *seq = (int)strtol(cmd, &end, 10);
    if (end == cmd || *end != ' ') return false;
    cmd = end + 1;

    *tool_id = (int)strtol(cmd, &end, 10);
    if (end == cmd || *end != ' ') return false;
    cmd = end + 1;

    *len_mm = strtof(cmd, &end);
    if (end == cmd) return false;

    return true;
}

static bool parse_event_cmd(const char *cmd, int *seq, char *event_type, char *arg)
{
    if (!cmd || strncmp(cmd, "EV ", 3) != 0) {
        return false;
    }
    cmd += 3;
    char *end;
    long s = strtol(cmd, &end, 10);
    if (end > cmd && *end == ' ') {
        *seq = (int)s;
        cmd = end + 1;
    }
    int i = 0;
    while (*cmd && *cmd != ' ' && i < 31) {
        event_type[i++] = *cmd++;
    }
    event_type[i] = '\0';
    if (*cmd != ' ') return false;
    cmd++;
    i = 0;
    while (*cmd && i < 95) {
        arg[i++] = *cmd++;
    }
    arg[i] = '\0';
    return true;
}

static void handle_event(const char *event_type, const char *arg)
{
    if (!event_type || !arg) {
        return;
    }

    if (strcmp(event_type, "heat_cf") == 0) {
        float temp = strtof(arg, NULL);
        PID_send_command(1, temp, temp > 0.0f);
        return;
    }
    if (strcmp(event_type, "heat_resin") == 0) {
        float temp = strtof(arg, NULL);
        PID_send_command(2, temp, temp > 0.0f);
        return;
    }
    if (strcmp(event_type, "tool_change_cf") == 0) {
        switch_to_CF();
        g_current_tool_id = 1;
        return;
    }
    if (strcmp(event_type, "tool_change_resin") == 0) {
        switch_to_RESIN();
        g_current_tool_id = 2;
        return;
    }
    if (strcmp(event_type, "fan_cf") == 0) {
        bool enable = (strtol(arg, NULL, 10) != 0);
        fan_set_state(FAN1, enable);
        return;
    }
    if (strcmp(event_type, "fan_resin") == 0) {
        bool enable = (strtol(arg, NULL, 10) != 0);
        fan_set_state(FAN2, enable);
        return;
    }
    if (strcmp(event_type, "extrude_reset") == 0) {
        int tool_id = (int)strtol(arg, NULL, 10);
        extruder_reset_absolute(tool_id);
        return;
    }
    if (strcmp(event_type, "pid_set_cf") == 0) {
        float kp, ki, kd, max_o, min_o, max_i, min_i;
        if (sscanf(arg, "%f %f %f %f %f %f %f",
                   &kp, &ki, &kd, &max_o, &min_o, &max_i, &min_i) == 7) {
            pid_set_params(1, kp, ki, kd, max_o, min_o, max_i, min_i);
        }
        return;
    }
    if (strcmp(event_type, "pid_set_resin") == 0) {
        float kp, ki, kd, max_o, min_o, max_i, min_i;
        if (sscanf(arg, "%f %f %f %f %f %f %f",
                   &kp, &ki, &kd, &max_o, &min_o, &max_i, &min_i) == 7) {
            pid_set_params(2, kp, ki, kd, max_o, min_o, max_i, min_i);
        }
        return;
    }
}

static void handle_extrude_cmd(int tool_id, float total_mm)
{
    if (tool_id != 1 && tool_id != 2) {
        return;
    }

    extruder_set_absolute(tool_id, total_mm);
}

static void handle_line(char *cmd)
{
    if (!cmd) {
        return;
    }

    trim(cmd);
    if (cmd[0] == '\0') {
        return;
    }

    if (uart_cmd_queue == NULL) {
        return;
    }

    uart_cmd_t msg;
    memset(&msg, 0, sizeof(msg));

    if (parse_extrude_cmd(cmd, &msg.seq, &msg.tool_id, &msg.total_mm)) {
        msg.type = UART_CMD_EXTRUDE;
        if (xQueueSend(uart_cmd_queue, &msg, 0) != pdTRUE) {
            uart_cmd_drop_count++;
        }
        return;
    }

    if (parse_event_cmd(cmd, &msg.seq, msg.event_type, msg.arg)) {
        msg.type = UART_CMD_EVENT;
        if (xQueueSend(uart_cmd_queue, &msg, 0) != pdTRUE) {
            uart_cmd_drop_count++;
        }
        return;
    }
}

int get_current_tool_id(void)
{
    return g_current_tool_id;
}

// 串口接收任务
void serial_rx_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t data[128];
    char line_buf[256];
    size_t line_len = 0;
    bool overflow = false;

    while (1) {
        uart_event_t event;
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (event.type == UART_DATA) {
            int remaining = event.size;
            while (remaining > 0) {
                int to_read = remaining;
                if (to_read > (int)sizeof(data)) {
                    to_read = sizeof(data);
                }
                int len = uart_read_bytes(UART_PORT, data, to_read, 0);
                if (len <= 0) {
                    break;
                }
                remaining -= len;
                for (int i = 0; i < len; i++) {
                    char c = (char)data[i];
                    if (c == '\n') {
                        if (!overflow && line_len > 0) {
                            uart_line_t msg;
                            size_t copy_len = line_len;
                            if (copy_len >= sizeof(msg.line)) {
                                copy_len = sizeof(msg.line) - 1;
                            }
                            memcpy(msg.line, line_buf, copy_len);
                            msg.line[copy_len] = '\0';
                            if (uart_line_queue != NULL) {
                                if (xQueueSend(uart_line_queue, &msg, 0) != pdTRUE) {
                                    uart_line_drop_count++;
                                }
                            }
                        }
                        line_len = 0;
                        overflow = false;
                    } else if (c != '\r') {
                        if (!overflow) {
                            if (line_len < sizeof(line_buf) - 1) {
                                line_buf[line_len++] = c;
                            } else {
                                overflow = true;
                                line_len = 0;
                            }
                        }
                    }
                }
            }
            continue;
        }

        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            uart_flush_input(UART_PORT);
            uart_event_t dummy;
            while (xQueueReceive(uart_event_queue, &dummy, 0) == pdTRUE);
            line_len = 0;
            overflow = false;
            continue;
        }
    }
}

// 串口解析任务
void serial_parse_task(void *pvParameters)
{
    (void)pvParameters;
    uart_line_t msg;
    while (1) {
        if (xQueueReceive(uart_line_queue, &msg, portMAX_DELAY) == pdTRUE) {
            handle_line(msg.line);
        }
    }
}

static void command_dispatch_task(void *pvParameters)
{
    (void)pvParameters;

    uart_cmd_t msg;
    while (1) {
        if (xQueueReceive(uart_cmd_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.type == UART_CMD_EXTRUDE) {
                handle_extrude_cmd(msg.tool_id, msg.total_mm);
            } else if (msg.type == UART_CMD_EVENT) {
                handle_event(msg.event_type, msg.arg);
            }
        }
    }
}

void command_dispatch_task_start(void)
{
    if (uart_cmd_queue == NULL) {
        uart_cmd_queue = xQueueCreate(128, sizeof(uart_cmd_t));
    }
    xTaskCreatePinnedToCore(command_dispatch_task, "cmd_dispatch", 4096, NULL, 6, NULL, 1);
}

void serial_tasks_start(void)
{
    if (uart_line_queue == NULL) {
        uart_line_queue = xQueueCreate(128, sizeof(uart_line_t));
    }
    if (uart_cmd_queue == NULL) {
        uart_cmd_queue = xQueueCreate(128, sizeof(uart_cmd_t));
    }
    xTaskCreatePinnedToCore(serial_rx_task, "serial_rx", 4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(serial_parse_task, "serial_parse", 4096, NULL, 6, NULL, 1);
}

static void status_task(void *pvParameters)
{
    (void)pvParameters;

    char buf[192];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float temp_cf = get_temperature1();
        float temp_resin = get_temperature2();
        float target_cf = pid_get_target(1);
        float target_resin = pid_get_target(2);
        int fan_ok_cf = fan_get_state(FAN1) ? 1 : 0;
        int fan_ok_resin = fan_get_state(FAN2) ? 1 : 0;
        int tool = 0;
        int err = 0;
        int sw1 = gpio_get_level(STOP_PIN_1);
        int sw2 = gpio_get_level(STOP_PIN_2);

        if (sw1 == 0 && sw2 == 0) {
            tool = 0;
            err = 1;
        } else if (sw1 == 0) {
            tool = 1;
        } else if (sw2 == 0) {
            tool = 2;
        } else {
            tool = 0;
        }

        int len = snprintf(buf, sizeof(buf),
                           "STAT temp_cf=%.1f temp_resin=%.1f target_cf=%.1f target_resin=%.1f "
                           "fan_ok_cf=%d fan_ok_resin=%d tool=%d err=%d\n",
                           temp_cf, temp_resin, target_cf, target_resin,
                           fan_ok_cf, fan_ok_resin, tool, err);
        if (len > 0) {
            if (len > (int)sizeof(buf)) {
                len = sizeof(buf);
            }
            uart0_write((const uint8_t *)buf, len);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

void status_task_start(void)
{
    xTaskCreatePinnedToCore(status_task, "status_pub", 4096, NULL, 1, NULL, 1);
}
