/*
 * Uplink task: serialises samples and status to the ESP32 gateway over USART1
 * as checksummed ASCII lines (see docs/protocols.md). Runs below the sensor
 * task so a slow or disconnected gateway can never delay acquisition.
 */

#include "app.h"
#include "log.h"
#include "uart_port.h"

#define TAG "uplink"
#define TX_TIMEOUT_MS       50u
#define HEARTBEAT_PERIOD_MS 500u

void uplink_task(void *arg)
{
  (void)arg;
  char line[UPLINK_LINE_MAX];
  uint32_t failures = 0;

  for (;;) {
    uplink_msg_t m;
    BaseType_t got = xQueueReceive(q_uplink, &m, pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    xEventGroupSetBits(eg_heartbeat, HB_UPLINK);
    if (got != pdTRUE) {
      continue;
    }

    size_t n = (m.kind == UPLINK_ENV)
      ? protocol_format_env_line(&m.u.env, line, sizeof line)
      : protocol_format_status_line(&m.u.status, line, sizeof line);
    if (n == 0) {
      continue;
    }

    if (uart_port_write(&g_uplink, line, n, TX_TIMEOUT_MS) != 0) {
      if (++failures % 10u == 1u) {
        LOGW(TAG, "USART1 write failed (%lu total)", (unsigned long)failures);
      }
    }
  }
}
