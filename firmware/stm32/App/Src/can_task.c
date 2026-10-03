/*
 * CAN task: single owner of the bxCAN transmit path, the command handler and
 * the UDS diagnostic server.
 *
 * Blocks on a queue set containing both the outgoing-frame queue (fed by the
 * sensor and health tasks) and the incoming-frame queue (fed by the RX ISR),
 * so one task services both directions without polling. The block time is
 * shortened whenever ISO-TP has a Consecutive Frame or timeout coming due.
 */

#include "app.h"
#include "can_bus.h"
#include "diag.h"
#include "log.h"

#define TAG "can"
#define TX_TIMEOUT_MS       20u   /* ~10 frame times at 500 kbit/s */
#define HEARTBEAT_PERIOD_MS 500u

static void handle_frame(const can_frame_t *f)
{
  can_command_t cmd;
  if (protocol_parse_command(f->std_id, f->data, f->dlc, &cmd) != 0) {
    LOGW(TAG, "ignored frame 0x%03lX dlc=%u", (unsigned long)f->std_id, f->dlc);
    return;
  }

  switch (cmd.id) {
    case CAN_CMD_SET_PERIOD:
      g_sample_period_ms = cmd.period_ms;
      LOGI(TAG, "cmd: sample period -> %u ms", cmd.period_ms);
      break;
    case CAN_CMD_REQUEST_STATUS:
      xTaskNotifyGive(h_task_health);   /* health task sends NODE_STATUS now */
      LOGI(TAG, "cmd: status requested");
      break;
    default:
      break;
  }
}

void can_task(void *arg)
{
  (void)arg;

  if (can_bus_start(q_can_rx) != 0) {
    LOGE(TAG, "bxCAN start failed");
    board_fatal();
  }
  diag_init();
  LOGI(TAG, "bxCAN up: 500 kbit/s, filters 0x%03X/0x%03X + UDS 0x%03X/0x%03X, %s",
       CAN_ID_CMD_BASE, CAN_ID_CMD_MASK, CAN_ID_UDS_REQUEST_PHYS, CAN_ID_UDS_REQUEST_FUNC,
#if defined(CAN_SELF_TEST)
       "silent-loopback"
#else
       "normal mode"
#endif
  );

  uint32_t consecutive_tx_fail = 0;

  for (;;) {
    uint32_t wait_ms = diag_poll();
    if (wait_ms > HEARTBEAT_PERIOD_MS) {
      wait_ms = HEARTBEAT_PERIOD_MS;
    }

    QueueSetMemberHandle_t ready = xQueueSelectFromSet(qs_can, pdMS_TO_TICKS(wait_ms));
    xEventGroupSetBits(eg_heartbeat, HB_CAN);

    can_frame_t f;
    if (ready == q_can_rx) {
      if (xQueueReceive(q_can_rx, &f, 0) == pdTRUE) {
        if (diag_accepts(f.std_id)) {
          diag_on_frame(&f);
        } else {
          handle_frame(&f);
        }
      }
    } else if (ready == q_can_tx) {
      if (xQueueReceive(q_can_tx, &f, 0) == pdTRUE) {
        if (can_bus_transmit(&f, TX_TIMEOUT_MS) == 0) {
          if (consecutive_tx_fail >= 3u) {
            LOGI(TAG, "TX recovered after %lu failures", (unsigned long)consecutive_tx_fail);
          }
          consecutive_tx_fail = 0;
        } else if (++consecutive_tx_fail == 3u) {
          /* Log once per outage, not once per frame. */
          can_error_state_t es;
          can_bus_get_error_state(&es);
          LOGW(TAG, "TX stalled: TEC=%u REC=%u LEC=%s%s - no other node ACKing? check termination",
               es.tec, es.rec, can_bus_lec_str(es.lec), es.bus_off ? " BUS-OFF" : "");
        }
      }
    }
  }
}
