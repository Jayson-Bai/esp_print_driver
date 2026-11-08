#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Unified UART TX owner: only uart_tx_task calls uart_write_bytes.
 * Other tasks submit complete NUL-terminated frames via uart_tx_submit_*;
 * the TX task serialises them by priority.
 *
 * Priority: event(EVACK/EVDONE/EVERR) > eack > telemetry(STAT/ESTAT/USTAT).
 * Telemetry frames carry a generation time and a deadline; expired telemetry
 * may be dropped but is counted.
 */

typedef enum {
    UART_TX_PRIO_EVENT = 0,
    UART_TX_PRIO_EACK  = 1,
    UART_TX_PRIO_TELEM = 2,
} uart_tx_prio_t;

bool uart_tx_submit(uart_tx_prio_t prio, const char *frame, size_t len,
                    int64_t generated_us, int64_t deadline_us);
bool uart_tx_submit_event(const char *frame, size_t len);
bool uart_tx_submit_eack(const char *frame, size_t len);
bool uart_tx_submit_telemetry(const char *frame, size_t len, int64_t deadline_us);

typedef struct {
    uint32_t submitted_event;
    uint32_t submitted_eack;
    uint32_t submitted_telemetry;
    uint32_t sent_event;
    uint32_t sent_eack;
    uint32_t sent_telemetry;
    uint32_t dropped_queue_full;
    uint32_t dropped_expired;
    uint32_t send_errors;
    uint32_t max_queue_depth;
} uart_tx_health_t;

void uart_tx_get_health(uart_tx_health_t *out);
void uart_tx_dispatch_start(void);

#ifdef __cplusplus
}
#endif
