#include "diag.h"

#include <stdio.h>
#include <string.h>

#include "app.h"
#include "isotp.h"
#include "log.h"
#include "uds.h"

#define TAG "uds"
#define SW_VERSION       "ENVMON-1.2.0"
#define TX_TIMEOUT_MS    20u
#define RESET_DELAY_MS   50u     /* let the positive response reach the bus first */

static isotp_link_t s_link;
static uds_server_t s_uds;
static uint8_t      s_resp[ISOTP_MAX_PAYLOAD];
static uint32_t     s_rng;
static uint32_t     s_last_busoff;
static uint8_t      s_reset_pending;
static uint32_t     s_reset_at_ms;

static uint32_t now_ms(void)
{
  return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static int reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

static void send_frame(const isotp_frame_t *f)
{
  can_frame_t cf = { .std_id = CAN_ID_UDS_RESPONSE, .dlc = f->dlc };
  memcpy(cf.data, f->data, sizeof cf.data);
  (void)can_bus_transmit(&cf, TX_TIMEOUT_MS);
}

/* ===========================================================================
 * Data identifiers
 * ======================================================================== */

static int read_did(void *ctx, uint16_t did, uint8_t *out, uint16_t max)
{
  (void)ctx;
  switch (did) {
    case UDS_DID_ECU_SERIAL: {                      /* 96-bit factory unique ID as 24 hex chars */
      if (max < 24) return -1;
      char hex[25];
      snprintf(hex, sizeof hex, "%08lX%08lX%08lX", (unsigned long)HAL_GetUIDw2(),
               (unsigned long)HAL_GetUIDw1(), (unsigned long)HAL_GetUIDw0());
      memcpy(out, hex, 24);
      return 24;
    }
    case UDS_DID_SW_VERSION: {
      size_t n = sizeof SW_VERSION - 1u;
      if (max < n) return -1;
      memcpy(out, SW_VERSION, n);
      return (int)n;
    }
    case UDS_DID_ENV_DATA: {
      if (max < 8) return -1;
      env_sample_t s;
      taskENTER_CRITICAL();
      s = g_last_sample;
      taskEXIT_CRITICAL();
      protocol_pack_env(&s, out);
      return 8;
    }
    case UDS_DID_SAMPLE_PERIOD: {
      if (max < 2) return -1;
      uint32_t p = g_sample_period_ms;
      out[0] = (uint8_t)(p >> 8);                   /* UDS data is big-endian */
      out[1] = (uint8_t)p;
      return 2;
    }
    case UDS_DID_NODE_STATUS: {
      if (max < 8) return -1;
      node_status_t st;
      taskENTER_CRITICAL();
      st = g_last_status;
      taskEXIT_CRITICAL();
      protocol_pack_status(&st, out);
      return 8;
    }
    default:
      return -1;
  }
}

static uint8_t write_did(void *ctx, uint16_t did, const uint8_t *in, uint16_t len)
{
  (void)ctx;
  if (did != UDS_DID_SAMPLE_PERIOD) return UDS_NRC_REQUEST_OUT_OF_RANGE;   /* others read-only */
  if (len != 2) return UDS_NRC_INCORRECT_LENGTH;
  uint16_t v = (uint16_t)((in[0] << 8) | in[1]);
  if (v < SAMPLE_PERIOD_MIN_MS || v > SAMPLE_PERIOD_MAX_MS) return UDS_NRC_REQUEST_OUT_OF_RANGE;
  g_sample_period_ms = v;
  return 0;
}

static uint32_t random32(void *ctx)
{
  (void)ctx;
  /* xorshift32, re-stirred with the SysTick counter so seeds depend on request timing. */
  s_rng ^= SysTick->VAL ^ xTaskGetTickCount();
  s_rng ^= s_rng << 13;
  s_rng ^= s_rng >> 17;
  s_rng ^= s_rng << 5;
  return s_rng;
}

/* ===========================================================================
 * DTCs: re-evaluated on every poll, from state the other tasks already publish.
 * ======================================================================== */

static void update_dtcs(void)
{
  can_stats_t cs;
  can_error_state_t es;
  can_bus_get_stats(&cs);
  can_bus_get_error_state(&es);

  uds_set_dtc(&s_uds, UDS_DTC_SENSOR_COMM, g_sensor_errors > 0 && !g_sensor_ok);
  uds_set_dtc(&s_uds, UDS_DTC_CAN_BUS_OFF, es.bus_off || cs.bus_off != s_last_busoff);
  s_last_busoff = cs.bus_off;
  uds_set_dtc(&s_uds, UDS_DTC_CLOCK_FALLBACK, g_clock_source == CLOCK_SRC_HSI_FALLBACK);
  uds_set_dtc(&s_uds, UDS_DTC_TASK_STALL, g_task_stalled);
}

/* ===========================================================================
 * Public API
 * ======================================================================== */

void diag_init(void)
{
  /* We advertise BS=0 (no further FC needed) and STmin=0 to testers. */
  isotp_init(&s_link, 0, 0);

  const uds_callbacks_t cb = { read_did, write_did, random32, NULL };
  uds_init(&s_uds, &cb);
  s_rng = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2() ^ 0x9E3779B9u;

  /* A watchdog reset is an event: record it once, as history, at boot. */
  if (g_reset_cause == RESET_CAUSE_IWDG) {
    uds_set_dtc(&s_uds, UDS_DTC_WATCHDOG_RESET, 1);
    uds_set_dtc(&s_uds, UDS_DTC_WATCHDOG_RESET, 0);
  }
}

int diag_accepts(uint32_t std_id)
{
  return std_id == CAN_ID_UDS_REQUEST_PHYS || std_id == CAN_ID_UDS_REQUEST_FUNC;
}

void diag_on_frame(const can_frame_t *f)
{
  int functional = (f->std_id == CAN_ID_UDS_REQUEST_FUNC);
  if (functional && (f->dlc < 1 || (f->data[0] >> 4) != 0)) {
    return;   /* functional addressing carries single frames only */
  }

  uint32_t now = now_ms();
  isotp_frame_t reply;
  unsigned ev = isotp_on_frame(&s_link, f->data, f->dlc, now, &reply);
  if (ev & ISOTP_EV_REPLY) {
    send_frame(&reply);                             /* Flow Control to the tester */
  }
  if (!(ev & ISOTP_EV_MESSAGE)) {
    return;
  }

  uint32_t actions;
  uint8_t sid = s_link.rx_buf[0];
  uint16_t n = uds_handle(&s_uds, s_link.rx_buf, s_link.rx_len, functional, now,
                          s_resp, sizeof s_resp, &actions);
  if (n > 0) {
    isotp_frame_t first;
    if (isotp_send(&s_link, s_resp, n, now, &first) == 0) {
      send_frame(&first);
    } else {
      LOGW(TAG, "response to 0x%02X dropped: transmitter busy", sid);
    }
  }

  if (n == 0) {
    LOGI(TAG, "0x%02X%s -> (no response)", sid, functional ? " (functional)" : "");
  } else if (s_resp[0] == UDS_SID_NEGATIVE_RESPONSE) {
    LOGI(TAG, "0x%02X -> NRC 0x%02X", sid, s_resp[2]);
  } else {
    LOGI(TAG, "0x%02X -> 0x%02X, %u bytes", sid, s_resp[0], n);
  }

  if (actions & UDS_ACTION_ECU_RESET) {
    s_reset_pending = 1;
    s_reset_at_ms = now + RESET_DELAY_MS;
  }
}

uint32_t diag_poll(void)
{
  uint32_t now = now_ms();
  isotp_frame_t f;
  while (isotp_poll(&s_link, now, &f)) {
    send_frame(&f);
    now = now_ms();
  }

  uds_tick(&s_uds, now);
  update_dtcs();

  if (s_reset_pending && reached(now, s_reset_at_ms) && !isotp_tx_busy(&s_link)) {
    LOGW(TAG, "ECU reset requested by tester");
    vTaskDelay(pdMS_TO_TICKS(10));                  /* drain the log line */
    NVIC_SystemReset();
  }

  uint32_t next = isotp_next_event_ms(&s_link, now);
  if (s_reset_pending && next > 10u) {
    next = 10u;
  }
  return next;
}
