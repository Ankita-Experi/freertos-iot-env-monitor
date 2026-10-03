#ifndef CAN_BUS_H
#define CAN_BUS_H

/*
 * bxCAN driver layer for FreeRTOS.
 *
 *  RX : FIFO0 message-pending ISR drains the hardware FIFO into a FreeRTOS
 *       queue with xQueueSendFromISR(); the CAN task processes frames later.
 *  TX : a counting semaphore mirrors the 3 hardware TX mailboxes. A task takes
 *       one "credit" per frame; the TX-complete / abort ISRs give it back.
 *       If no credit appears in time (no ACK on the bus, bus-off) the pending
 *       mailboxes are aborted so the task never blocks forever.
 *  ERR: warning / passive / bus-off are counted from the SCE interrupt.
 *       Last-error-code interrupts are deliberately NOT enabled: with no other
 *       node to ACK, every automatic retransmission raises an ACK error, which
 *       turns into an interrupt storm (~10 k IRQ/s). LEC is sampled by polling.
 *
 *  Filter: bank 0, 32-bit mask mode, accepts standard data frames 0x200-0x20F;
 *          bank 1, 32-bit list mode, accepts UDS requests 0x7E0 and 0x7DF.
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
  uint32_t std_id;   /* 11-bit */
  uint8_t  dlc;
  uint8_t  data[8];
} can_frame_t;

typedef struct {
  uint32_t tx_ok;
  uint32_t tx_timeout;       /* mailbox credit not returned in time -> aborted */
  uint32_t tx_aborted;
  uint32_t rx_ok;
  uint32_t rx_queue_full;    /* frame dropped: task not keeping up */
  uint32_t rx_fifo_overrun;  /* hardware FIFO overflowed before the ISR ran */
  uint32_t err_warning;      /* entered error-warning (TEC/REC >= 96)   */
  uint32_t err_passive;      /* entered error-passive (TEC/REC >= 128)  */
  uint32_t bus_off;          /* entered bus-off (TEC > 255)             */
} can_stats_t;

typedef struct {
  uint8_t tec;
  uint8_t rec;
  uint8_t lec;              /* last error code: 0 none,1 stuff,2 form,3 ack,4 rec,5 dom,6 crc */
  uint8_t warning  : 1;
  uint8_t passive  : 1;
  uint8_t bus_off  : 1;
} can_error_state_t;

/* Configure the acceptance filter, enable interrupts and start the controller.
 * Received frames that pass the filter are posted to rx_queue (item = can_frame_t). */
int  can_bus_start(QueueHandle_t rx_queue);

/* Queue a frame in a hardware mailbox. Task context only. Returns 0 on success. */
int  can_bus_transmit(const can_frame_t *f, uint32_t timeout_ms);

void can_bus_get_stats(can_stats_t *out);
void can_bus_get_error_state(can_error_state_t *out);
const char *can_bus_lec_str(uint8_t lec);

#endif /* CAN_BUS_H */
