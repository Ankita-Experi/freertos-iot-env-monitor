#include "isotp.h"

#include <string.h>

#define PCI_SF 0x0u
#define PCI_FF 0x1u
#define PCI_CF 0x2u
#define PCI_FC 0x3u

#define FC_CTS      0x0u
#define FC_WAIT     0x1u
#define FC_OVERFLOW 0x2u

/* Wrap-safe "now has reached t". */
static int reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

static void frame_init(isotp_frame_t *f)
{
  memset(f->data, ISOTP_PAD_BYTE, sizeof f->data);
  f->dlc = 8;
}

static void make_fc(isotp_frame_t *f, uint8_t status, uint8_t bs, uint8_t stmin)
{
  frame_init(f);
  f->data[0] = (uint8_t)((PCI_FC << 4) | status);
  f->data[1] = bs;
  f->data[2] = stmin;
}

uint32_t isotp_stmin_to_ms(uint8_t stmin)
{
  if (stmin <= 0x7Fu) return stmin;
  if (stmin >= 0xF1u && stmin <= 0xF9u) return 1;   /* 100-900 us: our tick is 1 ms */
  return 0x7Fu;                                     /* reserved values: use the maximum */
}

void isotp_init(isotp_link_t *link, uint8_t block_size, uint8_t stmin_ms)
{
  memset(link, 0, sizeof *link);
  link->fc_block_size = block_size;
  link->fc_stmin = stmin_ms;
}

int isotp_tx_busy(const isotp_link_t *link)
{
  return link->tx_state != ISOTP_TX_IDLE;
}

/* ===========================================================================
 * Receive
 * ======================================================================== */

static unsigned on_flow_control(isotp_link_t *link, const uint8_t *d, uint8_t dlc, uint32_t now)
{
  if (link->tx_state != ISOTP_TX_WAIT_FC || dlc < 3) {
    return ISOTP_EV_NONE;   /* unexpected FC: ignore, as the standard requires */
  }
  switch (d[0] & 0x0Fu) {
    case FC_CTS:
      link->tx_block_size = d[1];
      link->tx_bs_count   = d[1];
      link->tx_stmin_ms   = isotp_stmin_to_ms(d[2]);
      link->tx_state      = ISOTP_TX_SENDING;
      link->tx_next_ms    = now;               /* first CF may go immediately */
      break;
    case FC_WAIT:
      link->tx_deadline_ms = now + ISOTP_TIMEOUT_MS;
      break;
    default:                                   /* overflow or invalid: abort */
      link->tx_state = ISOTP_TX_IDLE;
      break;
  }
  return ISOTP_EV_NONE;
}

unsigned isotp_on_frame(isotp_link_t *link, const uint8_t *d, uint8_t dlc, uint32_t now,
                        isotp_frame_t *reply)
{
  if (dlc < 1) {
    return ISOTP_EV_NONE;
  }

  switch (d[0] >> 4) {
    case PCI_SF: {
      uint8_t len = d[0] & 0x0Fu;
      if (len == 0 || len > 7 || len > dlc - 1) {
        link->rx_errors++;
        return ISOTP_EV_NONE;
      }
      link->rx_active = 0;                     /* a new SF aborts any reception in progress */
      memcpy(link->rx_buf, &d[1], len);
      link->rx_len = len;
      return ISOTP_EV_MESSAGE;
    }

    case PCI_FF: {
      if (dlc < 8) {
        link->rx_errors++;
        return ISOTP_EV_NONE;
      }
      uint16_t total = (uint16_t)(((d[0] & 0x0Fu) << 8) | d[1]);
      if (total < 8) {                         /* would have fitted in a Single Frame */
        link->rx_errors++;
        return ISOTP_EV_NONE;
      }
      if (total > ISOTP_MAX_PAYLOAD) {
        link->rx_errors++;
        link->rx_active = 0;
        make_fc(reply, FC_OVERFLOW, 0, 0);
        return ISOTP_EV_REPLY;
      }
      memcpy(link->rx_buf, &d[2], 6);
      link->rx_len         = 6;
      link->rx_expected    = total;
      link->rx_next_sn     = 1;
      link->rx_active      = 1;
      link->rx_bs_count    = link->fc_block_size;
      link->rx_deadline_ms = now + ISOTP_TIMEOUT_MS;
      make_fc(reply, FC_CTS, link->fc_block_size, link->fc_stmin);
      return ISOTP_EV_REPLY;
    }

    case PCI_CF: {
      if (!link->rx_active) {
        return ISOTP_EV_NONE;                  /* stray CF: ignore */
      }
      if ((d[0] & 0x0Fu) != link->rx_next_sn) {
        link->rx_errors++;
        link->rx_active = 0;                   /* sequence error: abort the message */
        return ISOTP_EV_NONE;
      }
      uint16_t remaining = (uint16_t)(link->rx_expected - link->rx_len);
      uint16_t n = remaining < 7u ? remaining : 7u;
      if (dlc < n + 1u) {
        link->rx_errors++;
        link->rx_active = 0;
        return ISOTP_EV_NONE;
      }
      memcpy(&link->rx_buf[link->rx_len], &d[1], n);
      link->rx_len         = (uint16_t)(link->rx_len + n);
      link->rx_next_sn     = (uint8_t)((link->rx_next_sn + 1u) & 0x0Fu);
      link->rx_deadline_ms = now + ISOTP_TIMEOUT_MS;

      if (link->rx_len >= link->rx_expected) {
        link->rx_active = 0;
        return ISOTP_EV_MESSAGE;
      }
      if (link->fc_block_size != 0 && --link->rx_bs_count == 0) {
        link->rx_bs_count = link->fc_block_size;
        make_fc(reply, FC_CTS, link->fc_block_size, link->fc_stmin);
        return ISOTP_EV_REPLY;
      }
      return ISOTP_EV_NONE;
    }

    case PCI_FC:
      return on_flow_control(link, d, dlc, now);

    default:
      return ISOTP_EV_NONE;
  }
}

/* ===========================================================================
 * Transmit
 * ======================================================================== */

int isotp_send(isotp_link_t *link, const uint8_t *payload, uint16_t len, uint32_t now,
               isotp_frame_t *first)
{
  if (len == 0 || len > ISOTP_MAX_PAYLOAD || link->tx_state != ISOTP_TX_IDLE) {
    return -1;
  }
  frame_init(first);

  if (len <= 7u) {
    first->data[0] = (uint8_t)((PCI_SF << 4) | len);
    memcpy(&first->data[1], payload, len);
    return 0;
  }

  memcpy(link->tx_buf, payload, len);
  link->tx_len = len;
  first->data[0] = (uint8_t)((PCI_FF << 4) | ((len >> 8) & 0x0Fu));
  first->data[1] = (uint8_t)(len & 0xFFu);
  memcpy(&first->data[2], payload, 6);
  link->tx_off         = 6;
  link->tx_next_sn     = 1;
  link->tx_state       = ISOTP_TX_WAIT_FC;
  link->tx_deadline_ms = now + ISOTP_TIMEOUT_MS;
  return 0;
}

int isotp_poll(isotp_link_t *link, uint32_t now, isotp_frame_t *out)
{
  if (link->rx_active && reached(now, link->rx_deadline_ms)) {
    link->rx_active = 0;                       /* N_Cr: sender went quiet mid-message */
    link->timeouts++;
  }

  if (link->tx_state == ISOTP_TX_WAIT_FC) {
    if (reached(now, link->tx_deadline_ms)) {
      link->tx_state = ISOTP_TX_IDLE;          /* N_Bs: no Flow Control arrived */
      link->timeouts++;
    }
    return 0;
  }
  if (link->tx_state != ISOTP_TX_SENDING || !reached(now, link->tx_next_ms)) {
    return 0;
  }

  frame_init(out);
  out->data[0] = (uint8_t)((PCI_CF << 4) | link->tx_next_sn);
  uint16_t remaining = (uint16_t)(link->tx_len - link->tx_off);
  uint16_t n = remaining < 7u ? remaining : 7u;
  memcpy(&out->data[1], &link->tx_buf[link->tx_off], n);
  link->tx_off     = (uint16_t)(link->tx_off + n);
  link->tx_next_sn = (uint8_t)((link->tx_next_sn + 1u) & 0x0Fu);
  link->tx_next_ms = now + link->tx_stmin_ms;

  if (link->tx_off >= link->tx_len) {
    link->tx_state = ISOTP_TX_IDLE;
  } else if (link->tx_block_size != 0 && --link->tx_bs_count == 0) {
    link->tx_state       = ISOTP_TX_WAIT_FC;   /* block finished: wait for the next FC */
    link->tx_deadline_ms = now + ISOTP_TIMEOUT_MS;
  }
  return 1;
}

uint32_t isotp_next_event_ms(const isotp_link_t *link, uint32_t now)
{
  uint32_t best = UINT32_MAX;
  uint32_t t;

  if (link->rx_active) {
    t = reached(now, link->rx_deadline_ms) ? 0 : link->rx_deadline_ms - now;
    if (t < best) best = t;
  }
  if (link->tx_state == ISOTP_TX_WAIT_FC) {
    t = reached(now, link->tx_deadline_ms) ? 0 : link->tx_deadline_ms - now;
    if (t < best) best = t;
  } else if (link->tx_state == ISOTP_TX_SENDING) {
    t = reached(now, link->tx_next_ms) ? 0 : link->tx_next_ms - now;
    if (t < best) best = t;
  }
  return best;
}
