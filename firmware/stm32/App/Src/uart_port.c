#include "uart_port.h"

#include "board.h"
#include "task.h"

uart_port_t g_console = { .huart = &huart_console };
uart_port_t g_uplink  = { .huart = &huart_uplink  };

static int scheduler_running(void)
{
  return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

void uart_port_init(void)
{
  uart_port_t *ports[] = { &g_console, &g_uplink };
  for (size_t i = 0; i < sizeof ports / sizeof ports[0]; i++) {
    ports[i]->mutex   = xSemaphoreCreateMutex();
    ports[i]->tx_done = xSemaphoreCreateBinary();
    configASSERT(ports[i]->mutex != NULL && ports[i]->tx_done != NULL);
  }
  vQueueAddToRegistry(g_console.mutex, "con_mtx");
  vQueueAddToRegistry(g_uplink.mutex, "upl_mtx");
}

BaseType_t uart_port_lock(uart_port_t *p, uint32_t timeout_ms)
{
  if (!scheduler_running() || p->mutex == NULL) {
    return pdTRUE; /* single-threaded boot phase */
  }
  return xSemaphoreTake(p->mutex, pdMS_TO_TICKS(timeout_ms));
}

void uart_port_unlock(uart_port_t *p)
{
  if (scheduler_running() && p->mutex != NULL) {
    xSemaphoreGive(p->mutex);
  }
}

int uart_port_write_locked(uart_port_t *p, const void *data, size_t len, uint32_t timeout_ms)
{
  if (len == 0) {
    return 0;
  }
  if (len > UINT16_MAX) {
    return -1;
  }

  if (!scheduler_running() || p->tx_done == NULL) {
    return (HAL_UART_Transmit(p->huart, (const uint8_t *)data, (uint16_t)len, timeout_ms) == HAL_OK) ? 0 : -1;
  }

  (void)xSemaphoreTake(p->tx_done, 0); /* discard a stale completion */

  if (HAL_UART_Transmit_IT(p->huart, (const uint8_t *)data, (uint16_t)len) != HAL_OK) {
    p->errors++;
    return -1;
  }
  if (xSemaphoreTake(p->tx_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    HAL_UART_AbortTransmit_IT(p->huart);
    p->timeouts++;
    return -1;
  }
  return 0;
}

int uart_port_write(uart_port_t *p, const void *data, size_t len, uint32_t timeout_ms)
{
  if (uart_port_lock(p, timeout_ms) != pdTRUE) {
    p->timeouts++;
    return -1;
  }
  int rc = uart_port_write_locked(p, data, len, timeout_ms);
  uart_port_unlock(p);
  return rc;
}

/* ---- HAL callbacks (ISR context) --------------------------------------- */

static uart_port_t *port_for(UART_HandleTypeDef *h)
{
  if (h == g_console.huart) return &g_console;
  if (h == g_uplink.huart)  return &g_uplink;
  return NULL;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
  uart_port_t *p = port_for(h);
  if (p != NULL && p->tx_done != NULL) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(p->tx_done, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
  uart_port_t *p = port_for(h);
  if (p != NULL) {
    p->errors++;
  }
}
