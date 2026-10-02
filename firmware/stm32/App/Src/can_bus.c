#include "can_bus.h"

#include <string.h>

#include "board.h"
#include "protocol.h"
#include "semphr.h"
#include "task.h"

#define CAN_TX_MAILBOXES 3u

static QueueHandle_t     s_rx_queue;
static SemaphoreHandle_t s_tx_credits;
static can_stats_t       s_stats;

static void configure_filter(void)
{
  /*
   * 32-bit mask mode. In the 32-bit filter register layout:
   *   [31:21] STID[10:0]  [20:3] EXID[17:0]  [2] IDE  [1] RTR  [0] 0
   * FilterIdHigh/MaskHigh hold bits [31:16], so STID sits at << 5.
   * The low half's mask also checks IDE=0 and RTR=0 -> only standard *data*
   * frames get through; extended or remote frames are rejected in hardware.
   */
  CAN_FilterTypeDef f = {0};
  f.FilterBank           = 0;
  f.FilterMode           = CAN_FILTERMODE_IDMASK;
  f.FilterScale          = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh         = (uint32_t)(CAN_ID_CMD_BASE << 5);
  f.FilterIdLow          = 0x0000;                 /* IDE = 0, RTR = 0 */
  f.FilterMaskIdHigh     = (uint32_t)(CAN_ID_CMD_MASK << 5);
  f.FilterMaskIdLow      = 0x0006;                 /* compare IDE and RTR */
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation     = ENABLE;
  f.SlaveStartFilterBank = 14;                     /* banks 14-27 belong to CAN2 */
  if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) {
    board_fatal();
  }
}

int can_bus_start(QueueHandle_t rx_queue)
{
  s_rx_queue   = rx_queue;
  s_tx_credits = xSemaphoreCreateCounting(CAN_TX_MAILBOXES, CAN_TX_MAILBOXES);
  configASSERT(s_tx_credits != NULL);

  configure_filter();

  if (HAL_CAN_ActivateNotification(&hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
        CAN_IT_TX_MAILBOX_EMPTY |
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK) {
    return -1;
  }
  return (HAL_CAN_Start(&hcan) == HAL_OK) ? 0 : -1;
}

int can_bus_transmit(const can_frame_t *f, uint32_t timeout_ms)
{
  if (xSemaphoreTake(s_tx_credits, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    /* All three mailboxes still pending: nobody is ACKing or we are bus-off.
     * Abort them; each abort completes via the ISR and returns its credit. */
    s_stats.tx_timeout++;
    HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    return -1;
  }

  CAN_TxHeaderTypeDef hdr = {0};
  hdr.StdId              = f->std_id & 0x7FFu;
  hdr.IDE                = CAN_ID_STD;
  hdr.RTR                = CAN_RTR_DATA;
  hdr.DLC                = (f->dlc > 8u) ? 8u : f->dlc;
  hdr.TransmitGlobalTime = DISABLE;

  uint32_t mailbox;
  if (HAL_CAN_AddTxMessage(&hcan, &hdr, (uint8_t *)f->data, &mailbox) != HAL_OK) {
    xSemaphoreGive(s_tx_credits); /* credit/mailbox accounting drifted; hand it back */
    return -1;
  }
  return 0;
}

void can_bus_get_stats(can_stats_t *out)
{
  taskENTER_CRITICAL();
  *out = s_stats;
  taskEXIT_CRITICAL();
}

void can_bus_get_error_state(can_error_state_t *out)
{
  uint32_t esr = hcan.Instance->ESR;
  out->tec     = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
  out->rec     = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
  out->lec     = (uint8_t)((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
  out->warning = (esr & CAN_ESR_EWGF) ? 1u : 0u;
  out->passive = (esr & CAN_ESR_EPVF) ? 1u : 0u;
  out->bus_off = (esr & CAN_ESR_BOFF) ? 1u : 0u;
}

const char *can_bus_lec_str(uint8_t lec)
{
  static const char *const names[] = {
    "none", "stuff", "form", "ack", "bit-recessive", "bit-dominant", "crc", "sw",
  };
  return names[lec & 7u];
}

/* ===========================================================================
 * HAL callbacks -- ISR context
 * ======================================================================== */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
  BaseType_t woken = pdFALSE;
  CAN_RxHeaderTypeDef hdr;
  can_frame_t f;

  /* Drain everything the FIFO holds (up to 3) in one interrupt. */
  while (HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0) > 0u) {
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &hdr, f.data) != HAL_OK) {
      break;
    }
    f.std_id = hdr.StdId;
    f.dlc    = (uint8_t)hdr.DLC;
    s_stats.rx_ok++;
    if (s_rx_queue == NULL || xQueueSendFromISR(s_rx_queue, &f, &woken) != pdTRUE) {
      s_stats.rx_queue_full++;
    }
  }
  portYIELD_FROM_ISR(woken);
}

static void give_tx_credit(void)
{
  BaseType_t woken = pdFALSE;
  if (s_tx_credits != NULL) {
    xSemaphoreGiveFromISR(s_tx_credits, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h) { (void)h; s_stats.tx_ok++; give_tx_credit(); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h) { (void)h; s_stats.tx_ok++; give_tx_credit(); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h) { (void)h; s_stats.tx_ok++; give_tx_credit(); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *h)    { (void)h; s_stats.tx_aborted++; give_tx_credit(); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *h)    { (void)h; s_stats.tx_aborted++; give_tx_credit(); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *h)    { (void)h; s_stats.tx_aborted++; give_tx_credit(); }

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h)
{
  uint32_t e = h->ErrorCode;

  if (e & HAL_CAN_ERROR_EWG)     s_stats.err_warning++;
  if (e & HAL_CAN_ERROR_EPV)     s_stats.err_passive++;
  if (e & HAL_CAN_ERROR_BOF)     s_stats.bus_off++;
  if (e & HAL_CAN_ERROR_RX_FOV0) s_stats.rx_fifo_overrun++;

  /* A mailbox that ended in error (arbitration lost / transmit error) still
   * completed its request -- return its credit so accounting stays exact. */
  const uint32_t tx_fail = HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0 |
                           HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1 |
                           HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2;
  if (e & tx_fail) {
    if (e & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0)) give_tx_credit();
    if (e & (HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1)) give_tx_credit();
    if (e & (HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2)) give_tx_credit();
  }

  HAL_CAN_ResetError(h);
}
