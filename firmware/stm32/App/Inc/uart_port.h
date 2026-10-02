#ifndef UART_PORT_H
#define UART_PORT_H

/*
 * Thread-safe, interrupt-driven UART transmit.
 *
 * Each port has:
 *   - a mutex, so lines from different tasks never interleave on the wire
 *     (priority inheritance keeps a low-priority writer from blocking a
 *     high-priority one for long);
 *   - a binary semaphore given by the TX-complete ISR, so the writing task
 *     sleeps during the transfer instead of busy-waiting on TXE.
 *
 * Before the scheduler starts, writes fall back to polling so the boot banner
 * still works.
 */

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

typedef struct {
  UART_HandleTypeDef *huart;
  SemaphoreHandle_t   mutex;
  SemaphoreHandle_t   tx_done;
  volatile uint32_t   errors;
  volatile uint32_t   timeouts;
} uart_port_t;

extern uart_port_t g_console;  /* USART2 -> ST-LINK VCP */
extern uart_port_t g_uplink;   /* USART1 -> ESP32       */

void uart_port_init(void);

/* Lock/write/unlock, for callers that format into a shared buffer. */
BaseType_t uart_port_lock(uart_port_t *p, uint32_t timeout_ms);
void       uart_port_unlock(uart_port_t *p);
int        uart_port_write_locked(uart_port_t *p, const void *data, size_t len, uint32_t timeout_ms);

/* Convenience: lock + write + unlock. Returns 0 on success. */
int uart_port_write(uart_port_t *p, const void *data, size_t len, uint32_t timeout_ms);

#endif /* UART_PORT_H */
