#ifndef ISOTP_H
#define ISOTP_H

/*
 * ISO-TP (ISO 15765-2) transport layer for classic CAN, normal addressing.
 *
 * Splits messages longer than 7 bytes into a First Frame plus Consecutive
 * Frames, and reassembles them on receive, with Flow Control in both
 * directions. Pure C, no HAL or RTOS: the caller passes in received frames
 * and a millisecond timestamp, and sends whatever frames this module hands back.
 *
 *   PCI type (high nibble of byte 0)
 *     0 Single Frame       [0x0L] data...              L = 1..7
 *     1 First Frame        [0x1L LL] data...           12-bit length, 8..4095
 *     2 Consecutive Frame  [0x2N] data...              N = sequence 0..15
 *     3 Flow Control       [0x3S BS STmin]             S: 0 CTS, 1 WAIT, 2 OVERFLOW
 *
 * Outgoing frames are always 8 bytes, padded with 0xCC.
 */

#include <stdint.h>

#define ISOTP_MAX_PAYLOAD   128u    /* largest UDS message this node sends or accepts */
#define ISOTP_PAD_BYTE      0xCCu
#define ISOTP_TIMEOUT_MS    1000u   /* N_Bs (wait for FC) and N_Cr (wait for next CF) */

typedef struct {
  uint8_t data[8];
  uint8_t dlc;
} isotp_frame_t;

typedef enum {
  ISOTP_TX_IDLE = 0,
  ISOTP_TX_WAIT_FC,     /* First Frame sent, waiting for the receiver's Flow Control */
  ISOTP_TX_SENDING,     /* Consecutive Frames due, paced by STmin */
} isotp_tx_state_t;

typedef struct {
  /* Our Flow Control parameters, advertised when we receive a multi-frame message. */
  uint8_t  fc_block_size;   /* 0 = send all CFs without further FC */
  uint8_t  fc_stmin;        /* minimum gap between CFs we ask the sender for, ms */

  /* Receive side */
  uint8_t  rx_buf[ISOTP_MAX_PAYLOAD];
  uint16_t rx_len;          /* bytes received so far */
  uint16_t rx_expected;     /* total length announced in the First Frame */
  uint8_t  rx_next_sn;
  uint8_t  rx_active;
  uint8_t  rx_bs_count;     /* CFs left before we must send another FC (0 = unlimited) */
  uint32_t rx_deadline_ms;

  /* Transmit side */
  uint8_t  tx_buf[ISOTP_MAX_PAYLOAD];
  uint16_t tx_len;
  uint16_t tx_off;
  uint8_t  tx_next_sn;
  isotp_tx_state_t tx_state;
  uint8_t  tx_block_size;   /* from the receiver's FC */
  uint8_t  tx_bs_count;     /* CFs left in the current block */
  uint32_t tx_stmin_ms;
  uint32_t tx_next_ms;      /* when the next CF may go out */
  uint32_t tx_deadline_ms;  /* N_Bs timeout while waiting for FC */

  /* Diagnostics */
  uint32_t rx_errors;       /* wrong sequence number, overflow, malformed */
  uint32_t timeouts;
} isotp_link_t;

/* Result flags from isotp_on_frame(). */
#define ISOTP_EV_NONE     0u
#define ISOTP_EV_MESSAGE  1u   /* a complete message is in link->rx_buf / rx_len */
#define ISOTP_EV_REPLY    2u   /* *reply holds a Flow Control frame to send now */

void isotp_init(isotp_link_t *link, uint8_t block_size, uint8_t stmin_ms);

/* Feed one received CAN frame. Returns a combination of ISOTP_EV_* flags. */
unsigned isotp_on_frame(isotp_link_t *link, const uint8_t *data, uint8_t dlc, uint32_t now_ms,
                        isotp_frame_t *reply);

/* Start sending a message. *first receives the Single or First Frame to transmit now.
 * Returns 0 on success, -1 if the length is invalid or a transmission is already running. */
int isotp_send(isotp_link_t *link, const uint8_t *payload, uint16_t len, uint32_t now_ms,
               isotp_frame_t *first);

/* Call regularly. Returns 1 and fills *out when a Consecutive Frame is due;
 * also expires timed-out transfers. */
int isotp_poll(isotp_link_t *link, uint32_t now_ms, isotp_frame_t *out);

/* Milliseconds until isotp_poll() next has work, or UINT32_MAX if idle. */
uint32_t isotp_next_event_ms(const isotp_link_t *link, uint32_t now_ms);

int isotp_tx_busy(const isotp_link_t *link);

/* STmin byte -> milliseconds (0xF1-0xF9 = 100-900 us, rounded up to 1 ms). */
uint32_t isotp_stmin_to_ms(uint8_t stmin);

#endif /* ISOTP_H */
