#include "motor.h"
#include "serial_reader.h"
#include "pid_ctrl.h"
#include "fan.h"
#include "cutter.h"
#include "temperature.h"
#include "extruder_ctrl.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t g_uart_tx_mutex = NULL;

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
    if (data == NULL || len <= 0) {
        return ESP_OK;
    }

    if (g_uart_tx_mutex != NULL) {
        xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }
    int res = uart_write_bytes(UART_PORT, (const char*)data, len);
    if (g_uart_tx_mutex != NULL) {
        xSemaphoreGive(g_uart_tx_mutex);
    }
    return (res == len) ? ESP_OK : ESP_FAIL;
}

static volatile int g_current_tool_id = 1; // 1=纤维, 2=树脂

typedef enum {
    UART_CMD_EXTRUDE = 0,
    UART_CMD_EVENT = 1,
} uart_cmd_type_t;

typedef struct {
    char line[256];
    int64_t rx_us;
} uart_line_t;

typedef struct {
    uart_cmd_type_t type;
    uint32_t seq;
    int tool_id;
    /* 累计绝对 E，单位为 10^-6 mm；变量名保留为 total_um 以兼容现有接口。 */
    int64_t total_um;
    /* 接收任务读到本行换行符时的 MCU 单调微秒时间。 */
    int64_t rx_us;
    char event_type[32];
    char arg[96];
} uart_cmd_t;

static QueueHandle_t uart_cmd_queue = NULL;
static QueueHandle_t uart_line_queue = NULL;
static QueueHandle_t uart_event_queue = NULL;
static volatile uint32_t uart_cmd_drop_count = 0;
static volatile uint32_t uart_line_drop_count = 0;
static volatile uint32_t uart_fifo_ovf_count = 0;
static volatile uint32_t uart_buffer_full_count = 0;
static volatile uint32_t uart_parse_error_count = 0;
static volatile uint32_t uart_old_seq_count = 0;
static volatile uint32_t uart_seq_gap_count = 0;

static portMUX_TYPE g_e_status_mux = portMUX_INITIALIZER_UNLOCKED;
static bool g_last_e_valid = false;
static uint32_t g_last_e_seq = 0;
static int g_last_e_tool = 0;
static int64_t g_last_e_total_um = 0;
static int64_t g_last_e_us = 0;
static int64_t g_last_eack_us = 0;
#define RESET_EVENT_CACHE_DEPTH 8
static portMUX_TYPE g_reset_event_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_reset_event_done_seq[2][RESET_EVENT_CACHE_DEPTH];
static uint8_t g_reset_event_done_count[2] = {0, 0};
static uint8_t g_reset_event_done_next[2] = {0, 0};

static bool reset_event_was_completed(int tool_id, uint32_t seq)
{
    int index = tool_id - 1;
    bool completed = false;
    portENTER_CRITICAL(&g_reset_event_mux);
    for (uint8_t i = 0; i < g_reset_event_done_count[index]; ++i) {
        if (g_reset_event_done_seq[index][i] == seq) {
            completed = true;
            break;
        }
    }
    portEXIT_CRITICAL(&g_reset_event_mux);
    return completed;
}

static void mark_reset_event_completed(int tool_id, uint32_t seq)
{
    int index = tool_id - 1;
    portENTER_CRITICAL(&g_reset_event_mux);
    uint8_t slot = g_reset_event_done_next[index];
    g_reset_event_done_seq[index][slot] = seq;
    g_reset_event_done_next[index] = (uint8_t)((slot + 1U) %
                                                RESET_EVENT_CACHE_DEPTH);
    if (g_reset_event_done_count[index] < RESET_EVENT_CACHE_DEPTH) {
        g_reset_event_done_count[index]++;
    }
    portEXIT_CRITICAL(&g_reset_event_mux);
}

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
    if (g_uart_tx_mutex == NULL) {
        g_uart_tx_mutex = xSemaphoreCreateMutex();
    }
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

static bool parse_extrude_cmd(const char *cmd, uint32_t *seq, int *tool_id,
                              int64_t *total_um)
{
    /*
     * 挤出报文格式：E <uint32序号> <工具号> <累计绝对E(mm)>\n
     * 此处只完成严格语法检查和定点化；工具号 1/2 的语义由执行层检查。
     */
    if (!cmd || cmd[0] != 'E' || cmd[1] != ' ') {
        return false;
    }
    cmd += 2;
    char *end = NULL;

    errno = 0;
    unsigned long long seq_value = strtoull(cmd, &end, 10);
    if (end == cmd || errno == ERANGE || seq_value > UINT32_MAX || *end != ' ') {
        return false;
    }
    *seq = (uint32_t)seq_value;
    cmd = end + 1;

    errno = 0;
    long tool_value = strtol(cmd, &end, 10);
    if (end == cmd || errno == ERANGE || tool_value < INT_MIN ||
        tool_value > INT_MAX || *end != ' ') {
        return false;
    }
    *tool_id = (int)tool_value;
    cmd = end + 1;

    errno = 0;
    double len_mm = strtod(cmd, &end);
    if (end == cmd || *end != '\0' || errno == ERANGE || !isfinite(len_mm)) {
        return false;
    }

    /* 内部保留到 10^-6 mm；序列化之前已经损失的精度无法在 MCU 侧恢复。 */
    double scaled_um = len_mm * 1000000.0;
    if (!isfinite(scaled_um) || scaled_um < -1000000000000.0 ||
        scaled_um > 1000000000000.0) {
        return false;
    }
    *total_um = llround(scaled_um);

    return true;
}

static bool parse_event_cmd(const char *cmd, uint32_t *seq, char *event_type, char *arg)
{
    /* 通用事件格式：EV <uint32序号> <事件类型> [参数]\n */
    if (!cmd || strncmp(cmd, "EV ", 3) != 0) {
        return false;
    }
    cmd += 3;
    char *end = NULL;
    errno = 0;
    unsigned long long s = strtoull(cmd, &end, 10);
    if (end == cmd || *end != ' ' || errno == ERANGE || s > UINT32_MAX) {
        return false;
    }
    *seq = (uint32_t)s;
    cmd = end + 1;
    int i = 0;
    while (*cmd && *cmd != ' ' && i < 31) {
        event_type[i++] = *cmd++;
    }
    event_type[i] = '\0';
    if (*cmd == '\0') {
        arg[0] = '\0';
        return true;
    }
    if (*cmd != ' ') return false;
    cmd++;
    i = 0;
    while (*cmd && i < 95) {
        arg[i++] = *cmd++;
    }
    arg[i] = '\0';
    return true;
}

static void send_uart_line(const char *line)
{
    if (line == NULL) {
        return;
    }
    uart0_write((const uint8_t *)line, strlen(line));
}

static void update_last_e_status(uint32_t seq, int tool_id, int64_t total_um,
                                 int64_t mcu_us)
{
    portENTER_CRITICAL(&g_e_status_mux);
    g_last_e_valid = true;
    g_last_e_seq = seq;
    g_last_e_tool = tool_id;
    g_last_e_total_um = total_um;
    g_last_e_us = mcu_us;
    portEXIT_CRITICAL(&g_e_status_mux);
}

static void reset_last_e_status(void)
{
    portENTER_CRITICAL(&g_e_status_mux);
    g_last_e_valid = false;
    g_last_e_seq = 0;
    g_last_e_tool = 0;
    g_last_e_total_um = 0;
    g_last_e_us = 0;
    g_last_eack_us = 0;
    portEXIT_CRITICAL(&g_e_status_mux);
}

static void get_last_e_status(bool *valid, uint32_t *seq, int *tool_id,
                              int64_t *total_um, int64_t *mcu_us)
{
    portENTER_CRITICAL(&g_e_status_mux);
    *valid = g_last_e_valid;
    *seq = g_last_e_seq;
    *tool_id = g_last_e_tool;
    *total_um = g_last_e_total_um;
    *mcu_us = g_last_e_us;
    portEXIT_CRITICAL(&g_e_status_mux);
}

typedef struct {
    uint32_t seq;
    char event_type[32];
    bool active;
} cut_event_done_ctx_t;

static portMUX_TYPE g_cut_event_done_mux = portMUX_INITIALIZER_UNLOCKED;
static cut_event_done_ctx_t g_cut_event_done_ctx;

static void clear_cut_event_done_context(void)
{
    portENTER_CRITICAL(&g_cut_event_done_mux);
    g_cut_event_done_ctx.active = false;
    portEXIT_CRITICAL(&g_cut_event_done_mux);
}

static void prepare_cut_event_done_context(uint32_t seq, const char *event_type)
{
    (void)event_type;

    portENTER_CRITICAL(&g_cut_event_done_mux);
    g_cut_event_done_ctx.seq = seq;
    g_cut_event_done_ctx.event_type[0] = 'c';
    g_cut_event_done_ctx.event_type[1] = 'u';
    g_cut_event_done_ctx.event_type[2] = 't';
    g_cut_event_done_ctx.event_type[3] = '\0';
    g_cut_event_done_ctx.active = true;
    portEXIT_CRITICAL(&g_cut_event_done_mux);
}

static void cut_event_done_callback(void *ctx)
{
    cut_event_done_ctx_t done_ctx;
    cut_event_done_ctx_t *src = (cut_event_done_ctx_t *)ctx;

    if (src == NULL) {
        return;
    }

    portENTER_CRITICAL(&g_cut_event_done_mux);
    done_ctx = *src;
    src->active = false;
    portEXIT_CRITICAL(&g_cut_event_done_mux);

    if (!done_ctx.active) {
        return;
    }

    char done_msg[96];
    int64_t done_us = esp_timer_get_time();
    int done_len = snprintf(done_msg, sizeof(done_msg), "EVDONE %" PRIu32 " %s done %" PRId64 "\n",
                            done_ctx.seq, done_ctx.event_type, done_us);
    if (done_len > 0) {
        send_uart_line(done_msg);
    }
}

static void handle_event(uint32_t seq, const char *event_type, const char *arg)
{
    if (!event_type || !arg) {
        return;
    }

    char ack[96];
    int64_t ack_us = esp_timer_get_time();
    int ack_len = snprintf(ack, sizeof(ack), "EVACK %" PRIu32 " %s accepted %" PRId64 "\n",
                           seq, event_type, ack_us);
    if (ack_len > 0) {
        send_uart_line(ack);
    }

    if (strcmp(event_type, "heat_cf") == 0) {
        float temp = strtof(arg, NULL);
        PID_send_command(1, temp, temp > 0.0f);
        goto done;
    }
    if (strcmp(event_type, "heat_resin") == 0) {
        float temp = strtof(arg, NULL);
        PID_send_command(2, temp, temp > 0.0f);
        goto done;
    }
    if (strcmp(event_type, "tool_change_cf") == 0) {
        switch_to_CF();
        g_current_tool_id = 1;
        goto done;
    }
    if (strcmp(event_type, "tool_change_resin") == 0) {
        switch_to_RESIN();
        g_current_tool_id = 2;
        goto done;
    }
    if (strcmp(event_type, "fan_cf") == 0) {
        bool enable = (strtol(arg, NULL, 10) != 0);
        fan_set_state(FAN1, enable);
        goto done;
    }
    if (strcmp(event_type, "fan_resin") == 0) {
        bool enable = (strtol(arg, NULL, 10) != 0);
        fan_set_state(FAN2, enable);
        goto done;
    }
    if (strcmp(event_type, "cut") == 0) {
        prepare_cut_event_done_context(seq, event_type);
        if (!cutter_start_cut_sequence_async(cut_event_done_callback, &g_cut_event_done_ctx)) {
            clear_cut_event_done_context();
            char err_msg[96];
            int64_t err_us = esp_timer_get_time();
            int err_len = snprintf(err_msg, sizeof(err_msg),
                                   "EVERR %" PRIu32 " %s busy %" PRId64 "\n",
                                   seq, event_type, err_us);
            if (err_len > 0) {
                send_uart_line(err_msg);
            }
            return;
        }
        return;
    }
    if (strcmp(event_type, "extrude_reset") == 0) {
        int tool_id = (int)strtol(arg, NULL, 10);
        if (tool_id != 1 && tool_id != 2) {
            char err_msg[96];
            int64_t err_us = esp_timer_get_time();
            int err_len = snprintf(err_msg, sizeof(err_msg),
                                   "EVERR %" PRIu32 " %s invalid_tool %d %" PRId64 "\n",
                                   seq, event_type, tool_id, err_us);
            if (err_len > 0) {
                send_uart_line(err_msg);
            }
            return;
        }
        /*
         * 当前上位机对所有非切割事件都使用 event_seq == 0。因此只有非零事件号
         * 才能安全地作为重放去重标识；若对 0 也去重，之后同一工具的合法复位会被误吞。
         */
        if (seq != 0 && reset_event_was_completed(tool_id, seq)) {
            goto done;
        }
        if (!extruder_reset_absolute_wait(tool_id, 150)) {
            char err_msg[112];
            int64_t err_us = esp_timer_get_time();
            int err_len = snprintf(err_msg, sizeof(err_msg),
                                   "EVERR %" PRIu32 " %s reset_timeout %" PRId64 "\n",
                                   seq, event_type, err_us);
            if (err_len > 0) {
                send_uart_line(err_msg);
            }
            return;
        }
        if (seq != 0) {
            mark_reset_event_completed(tool_id, seq);
        }
        reset_last_e_status();
        goto done;
    }
    if (strcmp(event_type, "pid_set_cf") == 0) {
        float kp, ki, kd, max_o, min_o, max_i, min_i;
        if (sscanf(arg, "%f %f %f %f %f %f %f",
                   &kp, &ki, &kd, &max_o, &min_o, &max_i, &min_i) != 7) {
            char err_msg[96];
            int64_t err_us = esp_timer_get_time();
            int err_len = snprintf(err_msg, sizeof(err_msg),
                                   "EVERR %" PRIu32 " %s invalid_args %" PRId64 "\n",
                                   seq, event_type, err_us);
            if (err_len > 0) {
                send_uart_line(err_msg);
            }
            return;
        }
        pid_set_params(1, kp, ki, kd, max_o, min_o, max_i, min_i);
        goto done;
    }
    if (strcmp(event_type, "pid_set_resin") == 0) {
        float kp, ki, kd, max_o, min_o, max_i, min_i;
        if (sscanf(arg, "%f %f %f %f %f %f %f",
                   &kp, &ki, &kd, &max_o, &min_o, &max_i, &min_i) != 7) {
            char err_msg[96];
            int64_t err_us = esp_timer_get_time();
            int err_len = snprintf(err_msg, sizeof(err_msg),
                                   "EVERR %" PRIu32 " %s invalid_args %" PRId64 "\n",
                                   seq, event_type, err_us);
            if (err_len > 0) {
                send_uart_line(err_msg);
            }
            return;
        }
        pid_set_params(2, kp, ki, kd, max_o, min_o, max_i, min_i);
        goto done;
    }

    {
        char err_msg[96];
        int64_t err_us = esp_timer_get_time();
        int err_len = snprintf(err_msg, sizeof(err_msg),
                               "EVERR %" PRIu32 " %s unknown_event %" PRId64 "\n",
                               seq, event_type, err_us);
        if (err_len > 0) {
            send_uart_line(err_msg);
        }
        return;
    }

done:
    {
        char done_msg[96];
        int64_t done_us = esp_timer_get_time();
        int done_len = snprintf(done_msg, sizeof(done_msg), "EVDONE %" PRIu32 " %s done %" PRId64 "\n",
                                seq, event_type, done_us);
        if (done_len > 0) {
            send_uart_line(done_msg);
        }
    }
}

static void handle_extrude_cmd(uint32_t seq, int tool_id, int64_t total_um,
                               int64_t rx_us)
{
    if (tool_id != 1 && tool_id != 2) {
        uart_parse_error_count++;
        return;
    }

    bool last_valid;
    uint32_t last_seq;
    int last_tool;
    int64_t last_total_um;
    int64_t last_us;
    get_last_e_status(&last_valid, &last_seq, &last_tool, &last_total_um, &last_us);
    (void)last_tool;
    (void)last_total_um;
    (void)last_us;

    if (last_valid) {
        uint32_t seq_delta = seq - last_seq;
        if (seq_delta == 0) {
            uart_old_seq_count++;
        } else if (seq_delta > (UINT32_MAX / 2U)) {
            uart_old_seq_count++;
        } else if (seq_delta > 1U) {
            uint32_t missed = seq_delta - 1U;
            if (UINT32_MAX - uart_seq_gap_count < missed) {
                uart_seq_gap_count = UINT32_MAX;
            } else {
                uart_seq_gap_count += missed;
            }
        }
    }

    /*
     * seq 在这里刻意只用于估速和诊断，不作为拒收门禁。现有上位机可能对 E 去重、
     * 暂停发送、复用轨迹序号或跳号恢复；真正的执行依据始终是累计绝对 E 和 MCU
     * 本地运动规划。只要之后还有绝对 E，新报文可覆盖中间丢包；若最后一个 E
     * 永久丢失且没有后续报文，则固件无法知道缺失的最终目标。
     */
    if (!extruder_set_absolute_um_at(tool_id, total_um, seq, rx_us)) {
        return;
    }

    int64_t mcu_us = esp_timer_get_time();
    update_last_e_status(seq, tool_id, total_um, rx_us);

    /* EACK 最多约 20 Hz，用于观测而非逐包闭环；电机执行不等待上位机反馈。 */
    if (g_last_eack_us == 0 || mcu_us - g_last_eack_us >= 50000) {
        char ack[112];
        int len = snprintf(ack, sizeof(ack),
                           "EACK %" PRIu32 " %d %.6f %" PRId64 "\n",
                           seq, tool_id, (double)total_um / 1000000.0, mcu_us);
        if (len > 0) {
            send_uart_line(ack);
            g_last_eack_us = mcu_us;
        }
    }
}

static void handle_line(char *cmd, int64_t rx_us)
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

    if (parse_extrude_cmd(cmd, &msg.seq, &msg.tool_id, &msg.total_um)) {
        msg.type = UART_CMD_EXTRUDE;
        msg.rx_us = rx_us;
        if (xQueueSend(uart_cmd_queue, &msg, 0) != pdTRUE) {
            uart_cmd_drop_count++;
            extruder_notify_transport_fault(EXTRUDER_FAULT_CMD_QUEUE_FULL);
        }
        return;
    }

    if (parse_event_cmd(cmd, &msg.seq, msg.event_type, msg.arg)) {
        msg.type = UART_CMD_EVENT;
        if (xQueueSend(uart_cmd_queue, &msg, 0) != pdTRUE) {
            uart_cmd_drop_count++;
            extruder_notify_transport_fault(EXTRUDER_FAULT_CMD_QUEUE_FULL);
        }
        return;
    }

    uart_parse_error_count++;
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
                            /* 以完整行到达时刻估速，避免解析/调度排队时间污染 4 ms 周期。 */
                            msg.rx_us = esp_timer_get_time();
                            if (uart_line_queue != NULL) {
                                if (xQueueSend(uart_line_queue, &msg, 0) != pdTRUE) {
                                    uart_line_drop_count++;
                                    extruder_notify_transport_fault(EXTRUDER_FAULT_LINE_QUEUE_FULL);
                                }
                            }
                        } else if (overflow) {
                            uart_parse_error_count++;
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
            /*
             * 溢出后输入流的行边界已不可信，因此清空接收侧并记录本地故障。
             * 固件不会等待上位机确认，也不会把未知的缺失 E 自行臆测为新目标。
             */
            if (event.type == UART_FIFO_OVF) {
                uart_fifo_ovf_count++;
                extruder_notify_transport_fault(EXTRUDER_FAULT_UART_FIFO_OVF);
            } else {
                uart_buffer_full_count++;
                extruder_notify_transport_fault(EXTRUDER_FAULT_UART_BUFFER_FULL);
            }
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
            handle_line(msg.line, msg.rx_us);
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
                handle_extrude_cmd(msg.seq, msg.tool_id, msg.total_um, msg.rx_us);
            } else if (msg.type == UART_CMD_EVENT) {
                handle_event(msg.seq, msg.event_type, msg.arg);
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

static const char *extruder_state_name(extruder_state_t state)
{
    switch (state) {
        case EXTRUDER_STATE_IDLE:        return "IDLE";
        case EXTRUDER_STATE_ENABLE_WAIT: return "ENABLE_WAIT";
        case EXTRUDER_STATE_ACCEL:       return "ACCEL";
        case EXTRUDER_STATE_RUN:         return "RUN";
        case EXTRUDER_STATE_DECEL:       return "DECEL";
        case EXTRUDER_STATE_DIR_CHANGE:  return "DIR_CHANGE";
        case EXTRUDER_STATE_HOLD:        return "HOLD";
        case EXTRUDER_STATE_FAULT:       return "FAULT";
        case EXTRUDER_STATE_RESETTING:   return "RESETTING";
        default:                         return "UNKNOWN";
    }
}

static void send_status_buffer(char *buf, size_t capacity, int formatted_len)
{
    if (buf == NULL || capacity < 2 || formatted_len <= 0) {
        return;
    }

    size_t send_len = (size_t)formatted_len;
    if (send_len >= capacity) {
        buf[capacity - 2] = '\n';
        buf[capacity - 1] = '\0';
        send_len = capacity - 1;
    }
    uart0_write((const uint8_t *)buf, (int)send_len);
}

static void status_task(void *pvParameters)
{
    (void)pvParameters;

    char buf[512];
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

        bool last_valid;
        uint32_t last_seq;
        int last_tool;
        int64_t last_total_um;
        int64_t last_us;
        get_last_e_status(&last_valid, &last_seq, &last_tool, &last_total_um, &last_us);

        char last_seq_text[16];
        if (last_valid) {
            snprintf(last_seq_text, sizeof(last_seq_text), "%" PRIu32, last_seq);
        } else {
            memcpy(last_seq_text, "-1", 3);
        }

        /* STAT 保持原有 UI 状态格式；last_e_abs 仍以 mm 输出。 */
        int len = snprintf(buf, sizeof(buf),
                           "STAT temp_cf=%.1f temp_resin=%.1f target_cf=%.1f target_resin=%.1f "
                           "fan_ok_cf=%d fan_ok_resin=%d tool=%d err=%d "
                           "last_e_seq=%s last_e_tool=%d last_e_abs=%.6f last_e_us=%" PRId64 "\n",
                           temp_cf, temp_resin, target_cf, target_resin,
                           fan_ok_cf, fan_ok_resin, tool, err,
                           last_seq_text, last_tool,
                           (double)last_total_um / 1000000.0, last_us);
        send_status_buffer(buf, sizeof(buf), len);

        int status_tool = g_current_tool_id;
        if (status_tool != 1 && status_tool != 2) {
            status_tool = 1;
        }
        extruder_status_t e_status;
        memset(&e_status, 0, sizeof(e_status));
        /* ESTAT 是 MCU 本地规划/脉冲账本快照，仅用于监测，不构成执行握手。 */
        if (extruder_get_status(status_tool, &e_status)) {
            len = snprintf(buf, sizeof(buf),
                           "ESTAT tool=%d state=%s target_steps=%" PRId64
                           " submitted_steps=%" PRId64 " completed_steps=%" PRId64
                           " host_total_um=%" PRId64 " last_cmd_us=%" PRId64
                           " ff_until_us=%" PRId64 " ff_sps=%.3f v_sps=%.3f"
                           " phase=%.6f last_seq=%" PRIu32
                           " fault=0x%08" PRIx32 " hard_fault=0x%08" PRIx32
                           " rmt_submit_err=%" PRIu32
                           " rmt_done_err=%" PRIu32 " qdepth=%" PRIu32
                           " dir=%d enabled=%d hold_latched=%d host_valid=%d\n",
                           status_tool, extruder_state_name(e_status.state),
                           e_status.target_steps, e_status.submitted_steps,
                           e_status.completed_steps, e_status.host_total_um,
                           e_status.last_command_us,
                           e_status.feedforward_valid_until_us,
                           e_status.feedforward_steps_s,
                           e_status.velocity_steps_s, e_status.phase, e_status.last_seq,
                           e_status.fault_flags, e_status.hard_fault_flags,
                           e_status.rmt_submit_errors,
                           e_status.rmt_done_errors, e_status.queue_depth,
                           e_status.direction_sign, e_status.enabled ? 1 : 0,
                           e_status.host_hold_active ? 1 : 0,
                           e_status.host_sample_valid ? 1 : 0);
            send_status_buffer(buf, sizeof(buf), len);
        }

        /* USTAT 汇总 UART 与软件队列异常，便于无逻辑分析仪时定位链路丢包。 */
        len = snprintf(buf, sizeof(buf),
                       "USTAT fifo_ovf=%" PRIu32 " buffer_full=%" PRIu32
                       " line_drop=%" PRIu32 " cmd_drop=%" PRIu32
                       " old_seq=%" PRIu32 " seq_gap=%" PRIu32
                       " parse_err=%" PRIu32 "\n",
                       uart_fifo_ovf_count, uart_buffer_full_count,
                       uart_line_drop_count, uart_cmd_drop_count,
                       uart_old_seq_count, uart_seq_gap_count,
                       uart_parse_error_count);
        send_status_buffer(buf, sizeof(buf), len);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
    }
}

void status_task_start(void)
{
    xTaskCreatePinnedToCore(status_task, "status_pub", 4096, NULL, 1, NULL, 1);
}
