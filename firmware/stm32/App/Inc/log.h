#ifndef LOG_H
#define LOG_H

/*
 * Mutex-protected console logging (USART2 / ST-LINK VCP).
 *
 *   [   12.345] I sensor: T=23.45C RH=41.20% P=101325Pa
 *
 * Task context only -- never call from an ISR (it takes a mutex and blocks
 * on the UART). ISRs count events; tasks report them.
 */

#include <stdint.h>

typedef enum { LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR } log_level_t;

void log_init(void);
void log_printf(log_level_t level, const char *tag, const char *fmt, ...)
  __attribute__((format(printf, 3, 4)));

#define LOGD(tag, ...) log_printf(LOG_DEBUG, tag, __VA_ARGS__)
#define LOGI(tag, ...) log_printf(LOG_INFO,  tag, __VA_ARGS__)
#define LOGW(tag, ...) log_printf(LOG_WARN,  tag, __VA_ARGS__)
#define LOGE(tag, ...) log_printf(LOG_ERROR, tag, __VA_ARGS__)

/* Number of log lines dropped because the console mutex could not be taken in time. */
uint32_t log_dropped(void);

#endif /* LOG_H */
