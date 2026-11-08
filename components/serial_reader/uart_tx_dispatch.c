#include "uart_tx_dispatch.h"

#include "driver/uart.h"
#include "esp_timer.h"
#include "serial_reader.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <string.h>

/*
 * Bounded TX queue: each slot carries one complete NUL-terminated frame plus
 * its priority and generation time. Depth 64 absorbs event bursts without
 * flooding the driver.
 */

#define UART_TX_QUEUE_DEPTH   64U
#define UART_TX_FRAME_MAX     384U
#define UART_TX_TASK_STACK    4096U
#define UART_TX_TASK_PRIO     4

typedef struct {
    uart_tx_prio_t prio;
    int64_t generated_us;
    int64_t deadline_us;
    uint16_t len;
    char frame[UART_TX_FRAME_MAX];
} uart_tx_slot_t;

static QueueHandle_t g_tx_queue = NULL;
static TaskHandle_t g_tx_task = NULL;
static uart_tx_health_t g_tx_health = {0};
static portMUX_TYPE g_tx_health_mux = portMUX_INITIALIZER_UNLOCKED;

static void health_inc(volatile uint32_t *field)
{
    portENTER_CRITICAL(&g_tx_health_mux);
    (*field)++;
    portEXIT_CRITICAL(&g_tx_health_mux);
}

static void health_update_max_depth(void)
{
    portENTER_CRITICAL(&g_tx_health_mux);
    UBaseType_t depth = g_tx_queue ? uxQueueMessagesWaiting(g_tx_queue) : 0;
    if (depth > g_tx_health.max_queue_depth) {
        g_tx_health.max_queue_depth = depth;
    }
    portEXIT_CRITICAL(&g_tx_health_mux);
}

bool uart_tx_submit(uart_tx_prio_t prio, const char *frame, size_t len,
                    int64_t generated_us, int64_t deadline_us)
{
    if (g_tx_queue == NULL || frame == NULL || len == 0 ||
        len >= UART_TX_FRAME_MAX) {
        return false;
    }
    uart_tx_slot_t slot;
    slot.prio = prio;
    slot.generated_us = generated_us;
    slot.deadline_us = deadline_us;
    slot.len = (uint16_t)len;
    memcpy(slot.frame, frame, len);
    slot.frame[len] = '\0';
    if (xQueueSend(g_tx_queue, &slot, 0) != pdTRUE) {
        health_inc(&g_tx_health.dropped_queue_full);
        return false;
    }
    switch (prio) {
        case UART_TX_PRIO_EVENT: health_inc(&g_tx_health.submitted_event); break;
        case UART_TX_PRIO_EACK:  health_inc(&g_tx_health.submitted_eack); break;
        default:                 health_inc(&g_tx_health.submitted_telemetry); break;
    }
    health_update_max_depth();
    return true;
}

bool uart_tx_submit_event(const char *frame, size_t len)
{
    return uart_tx_submit(UART_TX_PRIO_EVENT, frame, len,
                          esp_timer_get_time(), 0);
}

bool uart_tx_submit_eack(const char *frame, size_t len)
{
    return uart_tx_submit(UART_TX_PRIO_EACK, frame, len,
                          esp_timer_get_time(), 0);
}

bool uart_tx_submit_telemetry(const char *frame, size_t len, int64_t deadline_us)
{
    return uart_tx_submit(UART_TX_PRIO_TELEM, frame, len,
                          esp_timer_get_time(), deadline_us);
}

void uart_tx_get_health(uart_tx_health_t *out)
{
    if (out == NULL) {
        return;
    }
    portENTER_CRITICAL(&g_tx_health_mux);
    *out = g_tx_health;
    portEXIT_CRITICAL(&g_tx_health_mux);
}

/*
 * Pop the next frame to send. The queue is FIFO; event frames are submitted
 * through their own entry point and normally arrive singly, while telemetry
 * carries a deadline so stale telemetry is discarded here before sending.
 * This guarantees an event never waits behind a backlog of stale telemetry.
 * Events themselves have no deadline and are never dropped.
 */
static bool tx_queue_pop(uart_tx_slot_t *out, TickType_t wait)
{
    if (xQueueReceive(g_tx_queue, out, wait) != pdTRUE) {
        return false;
    }
    int64_t now = esp_timer_get_time();
    while (out->prio == UART_TX_PRIO_TELEM &&
           out->deadline_us > 0 && now > out->deadline_us) {
        health_inc(&g_tx_health.dropped_expired);
        if (xQueueReceive(g_tx_queue, out, 0) != pdTRUE) {
            return false;
        }
    }
    return true;
}

static void uart_tx_task(void *pvParameters)
{
    (void)pvParameters;
    uart_tx_slot_t slot;
    while (1) {
        if (!tx_queue_pop(&slot, portMAX_DELAY)) {
            continue;
        }
        int written = uart_write_bytes(UART_PORT, slot.frame, (int)slot.len);
        if (written == (int)slot.len) {
            switch (slot.prio) {
                case UART_TX_PRIO_EVENT: health_inc(&g_tx_health.sent_event); break;
                case UART_TX_PRIO_EACK:  health_inc(&g_tx_health.sent_eack); break;
                default:                 health_inc(&g_tx_health.sent_telemetry); break;
            }
        } else {
            health_inc(&g_tx_health.send_errors);
        }
    }
}

void uart_tx_dispatch_start(void)
{
    if (g_tx_queue != NULL) {
        return;
    }
    g_tx_queue = xQueueCreate(UART_TX_QUEUE_DEPTH, sizeof(uart_tx_slot_t));
    if (g_tx_queue == NULL) {
        return;
    }
    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", UART_TX_TASK_STACK,
                            NULL, UART_TX_TASK_PRIO, &g_tx_task, 1);
}
