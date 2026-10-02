#include "log.h"

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "uart_port.h"

#define LOG_LINE_MAX     160u
#define LOG_LOCK_TIMEOUT 50u   /* ms; drop rather than stall a real-time task */
#define LOG_TX_TIMEOUT   50u   /* 160 chars @ 115200 baud ~= 14 ms */

/* Shared format buffer -- only touched while holding the console mutex. */
static char s_line[LOG_LINE_MAX];
static volatile uint32_t s_dropped;

static const char k_level_char[] = { 'D', 'I', 'W', 'E' };

void log_init(void)
{
  s_dropped = 0;
}

uint32_t log_dropped(void)
{
  return s_dropped;
}

void log_printf(log_level_t level, const char *tag, const char *fmt, ...)
{
  if (uart_port_lock(&g_console, LOG_LOCK_TIMEOUT) != pdTRUE) {
    s_dropped++;
    return;
  }

  uint32_t ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
  int n = snprintf(s_line, sizeof s_line, "[%5lu.%03lu] %c %s: ",
                   (unsigned long)(ms / 1000u), (unsigned long)(ms % 1000u),
                   k_level_char[level & 3], tag);
  if (n < 0) n = 0;

  va_list ap;
  va_start(ap, fmt);
  int m = vsnprintf(&s_line[n], sizeof s_line - (size_t)n, fmt, ap);
  va_end(ap);
  if (m < 0) m = 0;

  size_t len = (size_t)n + (size_t)m;
  if (len > sizeof s_line - 3u) {
    len = sizeof s_line - 3u; /* truncated: keep room for CRLF */
  }
  s_line[len++] = '\r';
  s_line[len++] = '\n';

  (void)uart_port_write_locked(&g_console, s_line, len, LOG_TX_TIMEOUT);
  uart_port_unlock(&g_console);
}
