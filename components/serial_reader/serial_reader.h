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
esp_err_t uart0_write(const uint8_t *data, int len);

// 串口任务
void serial_rx_task(void *pvParameters);
void serial_parse_task(void *pvParameters);
void serial_tasks_start(void);
void status_task_start(void);
void command_dispatch_task_start(void);
int get_current_tool_id(void);

#endif // SERIAL_READER_H
