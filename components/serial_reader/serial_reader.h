#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <stdint.h>

#define UART_PORT UART_NUM_0
#define UART_TX   43
#define UART_RX   44
#define BUF_SIZE  1024

// UART 初始化
void uart0_init(int baud_rate);

// UART 读写
int uart0_read(uint8_t *buf, int max_len, TickType_t ticks_to_wait);

// 串口任务
void serial_rx_task(void *pvParameters);
void serial_parse_task(void *pvParameters);
void serial_tasks_start(void);

/* Telemetry scheduler + unified TX owner live in the serial_reader component. */
void uart_tx_dispatch_start(void);
void command_dispatch_task_start(void);
int get_current_tool_id(void);
void get_last_e_status_public(bool *valid, uint32_t *seq, int *tool_id,
                              int64_t *total_um, int64_t *mcu_us);
void get_uart_counters(uint32_t *fifo, uint32_t *buf, uint32_t *line,
                       uint32_t *cmd, uint32_t *old_seq, uint32_t *gap,
                       uint32_t *perr);

#endif // SERIAL_READER_H
