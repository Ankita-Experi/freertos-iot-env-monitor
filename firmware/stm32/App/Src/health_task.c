/*
 * Health task: software watchdog supervisor + diagnostics.
 *
 *  - Every monitored task sets its bit in eg_heartbeat each loop. Once a second
 *    the health task reads-and-clears the bits and checks each task against a
 *    deadline. Only if every task is alive does it refresh the hardware IWDG;
 *    a hung task therefore causes a clean reset within ~4 s, and the next boot
 *    reports "reset cause: IWDG timeout" (also flagged in NODE_STATUS).
 *  - Every 5 s (or immediately when the CAN task sends a task notification on
 *    a status request) it publishes NODE_STATUS on CAN and to the uplink.
 *  - Every 30 s it logs stack high-water marks, heap and driver counters.
 *
 * Runs at the lowest application priority: if it is starved, that itself is
 * a fault the IWDG should catch.
 */

#include "app.h"
#include "can_bus.h"
#include "i2c_bus.h"
#include "log.h"
#include "uart_port.h"

#define TAG "health"
#define CHECK_PERIOD_MS     1000u
#define STATUS_PERIOD_S     5u
#define REPORT_PERIOD_S     30u
#define FAST_TASK_DEADLINE  1500u   /* can/uplink heartbeat every 500 ms */

typedef struct {
  EventBits_t bit;
  const char *name;
  TickType_t  last_seen;
} hb_entry_t;

static uint8_t build_flags(const can_error_state_t *es, uint32_t busoff_delta, int stalled)
{
  uint8_t flags = 0;
  if (g_sensor_ok)                             flags |= STATUS_FLAG_SENSOR_OK;
  if (es->warning)                             flags |= STATUS_FLAG_CAN_WARNING;
  if (es->passive)                             flags |= STATUS_FLAG_CAN_PASSIVE;
  if (es->bus_off || busoff_delta)             flags |= STATUS_FLAG_CAN_BUSOFF;
  if (g_clock_source == CLOCK_SRC_HSI_FALLBACK) flags |= STATUS_FLAG_HSI_FALLBACK;
  if (g_reset_cause == RESET_CAUSE_IWDG)       flags |= STATUS_FLAG_WDT_RESET;
  if (stalled)                                 flags |= STATUS_FLAG_TASK_STALL;
  return flags;
}

static void publish_status(uint8_t flags, const can_error_state_t *es)
{
  size_t heap = xPortGetFreeHeapSize();
  node_status_t st = {
    .flags           = flags,
    .tec             = es->tec,
    .rec             = es->rec,
    .free_heap_bytes = (uint16_t)((heap > 0xFFFFu) ? 0xFFFFu : heap),
    .uptime_s        = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ),
  };

  can_frame_t f = { .std_id = CAN_ID_NODE_STATUS, .dlc = 8 };
  protocol_pack_status(&st, f.data);
  if (xQueueSend(q_can_tx, &f, 0) != pdTRUE) g_queue_drops++;

  uplink_msg_t m = { .kind = UPLINK_STATUS, .u.status = st };
  if (xQueueSend(q_uplink, &m, 0) != pdTRUE) g_queue_drops++;
}

static void report(void)
{
  can_stats_t cs;
  i2c_bus_stats_t is;
  can_error_state_t es;
  can_bus_get_stats(&cs);
  i2c_bus_get_stats(&is);
  can_bus_get_error_state(&es);

  LOGI(TAG, "stack free (words): can=%lu sensor=%lu uplink=%lu health=%lu",
       (unsigned long)uxTaskGetStackHighWaterMark(h_task_can),
       (unsigned long)uxTaskGetStackHighWaterMark(h_task_sensor),
       (unsigned long)uxTaskGetStackHighWaterMark(h_task_uplink),
       (unsigned long)uxTaskGetStackHighWaterMark(NULL));
  LOGI(TAG, "heap free=%u min=%u | queue drops=%lu log drops=%lu",
       (unsigned)xPortGetFreeHeapSize(), (unsigned)xPortGetMinimumEverFreeHeapSize(),
       (unsigned long)g_queue_drops, (unsigned long)log_dropped());
  LOGI(TAG, "CAN tx=%lu txto=%lu rx=%lu rxdrop=%lu ovr=%lu ewg=%lu epv=%lu boff=%lu TEC=%u REC=%u",
       (unsigned long)cs.tx_ok, (unsigned long)cs.tx_timeout, (unsigned long)cs.rx_ok,
       (unsigned long)cs.rx_queue_full, (unsigned long)cs.rx_fifo_overrun,
       (unsigned long)cs.err_warning, (unsigned long)cs.err_passive,
       (unsigned long)cs.bus_off, es.tec, es.rec);
  LOGI(TAG, "I2C xfer=%lu err=%lu timeout=%lu | sensor errors=%lu | uplink uart err=%lu to=%lu",
       (unsigned long)is.transfers, (unsigned long)is.errors, (unsigned long)is.timeouts,
       (unsigned long)g_sensor_errors, (unsigned long)g_uplink.errors,
       (unsigned long)g_uplink.timeouts);
}

void health_task(void *arg)
{
  (void)arg;

  TickType_t now = xTaskGetTickCount();
  hb_entry_t hb[] = {
    { HB_SENSOR, "sensor", now },
    { HB_CAN,    "can",    now },
    { HB_UPLINK, "uplink", now },
  };

  uint32_t seconds = 0;
  uint32_t last_busoff = 0;
  int stall_reported = 0;

  for (;;) {
    /* Wake every second, or early if a status request arrives. */
    uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CHECK_PERIOD_MS));
    now = xTaskGetTickCount();

    /* xEventGroupClearBits returns the bits *before* clearing: atomic read-and-clear. */
    EventBits_t bits = xEventGroupClearBits(eg_heartbeat, HB_ALL);

    int stalled = 0;
    uint32_t sensor_deadline = 2u * g_sample_period_ms + 1000u;
    for (size_t i = 0; i < sizeof hb / sizeof hb[0]; i++) {
      if (bits & hb[i].bit) {
        hb[i].last_seen = now;
      }
      uint32_t deadline = (hb[i].bit == HB_SENSOR) ? sensor_deadline : FAST_TASK_DEADLINE;
      uint32_t silent_ms = (uint32_t)(now - hb[i].last_seen) * portTICK_PERIOD_MS;
      if (silent_ms > deadline) {
        stalled = 1;
        if (!stall_reported) {
          LOGE(TAG, "task '%s' silent for %lu ms - withholding watchdog refresh",
               hb[i].name, (unsigned long)silent_ms);
        }
      }
    }

    if (!stalled) {
      board_iwdg_refresh();
      stall_reported = 0;
    } else {
      stall_reported = 1;   /* IWDG will reset us within ~4 s */
    }

    if (notified == 0) {
      seconds++;
      board_led_toggle();   /* 0.5 Hz blink = scheduler and health task alive */
    }

    if (notified || (seconds % STATUS_PERIOD_S) == 0u) {
      can_stats_t cs;
      can_error_state_t es;
      can_bus_get_stats(&cs);
      can_bus_get_error_state(&es);
      uint32_t busoff_delta = cs.bus_off - last_busoff;
      last_busoff = cs.bus_off;
      publish_status(build_flags(&es, busoff_delta, stalled), &es);
    }

    if (notified == 0 && (seconds % REPORT_PERIOD_S) == 0u) {
      report();
    }
  }
}
