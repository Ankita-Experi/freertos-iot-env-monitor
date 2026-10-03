/*
 * Host shim: the firmware's ISO-TP + UDS stack behind a frame-in / frames-out
 * C API, built as libdiag.so. tools/test_tools.py loads it with ctypes and runs
 * tools/uds_client.py against it, so the Python tester is checked against the
 * exact C code that runs on the STM32 -- no hardware needed.
 */

#include <string.h>

#include "isotp.h"
#include "uds.h"

#define ID_PHYS 0x7E0u
#define ID_FUNC 0x7DFu

static isotp_link_t s_link;
static uds_server_t s_uds;
static uint8_t      s_resp[ISOTP_MAX_PAYLOAD];
static uint16_t     s_period = 1000;
static int          s_reset_requested;
static uint32_t     s_rng = 0x2545F491u;

static int read_did(void *ctx, uint16_t did, uint8_t *out, uint16_t max)
{
  (void)ctx;
  if (did == UDS_DID_SAMPLE_PERIOD && max >= 2) {
    out[0] = (uint8_t)(s_period >> 8);
    out[1] = (uint8_t)s_period;
    return 2;
  }
  if (did == UDS_DID_SW_VERSION && max >= 12) {
    memcpy(out, "ENVMON-1.2.0", 12);
    return 12;
  }
  if (did == UDS_DID_ENV_DATA && max >= 8) {
    /* 23.45 C, 41.20 %RH, 101325 Pa, seq 7 -- little-endian, as on CAN */
    const uint8_t env[8] = { 0x29, 0x09, 0x18, 0x10, 0xCD, 0x8B, 0x01, 0x07 };
    memcpy(out, env, 8);
    return 8;
  }
  return -1;
}

static uint8_t write_did(void *ctx, uint16_t did, const uint8_t *in, uint16_t len)
{
  (void)ctx;
  if (did != UDS_DID_SAMPLE_PERIOD) return UDS_NRC_REQUEST_OUT_OF_RANGE;
  if (len != 2) return UDS_NRC_INCORRECT_LENGTH;
  uint16_t v = (uint16_t)((in[0] << 8) | in[1]);
  if (v < 100 || v > 10000) return UDS_NRC_REQUEST_OUT_OF_RANGE;
  s_period = v;
  return 0;
}

static uint32_t random32(void *ctx)
{
  (void)ctx;
  s_rng ^= s_rng << 13;
  s_rng ^= s_rng >> 17;
  s_rng ^= s_rng << 5;
  return s_rng;
}

static int emit(const isotp_frame_t *f, uint8_t *out, int n, int max)
{
  if (n < max) memcpy(&out[n * 8], f->data, 8);
  return n + 1;
}

void shim_init(void)
{
  isotp_init(&s_link, 0, 0);
  const uds_callbacks_t cb = { read_did, write_did, random32, NULL };
  uds_init(&s_uds, &cb);
  s_period = 1000;
  s_reset_requested = 0;
}

/* Feed one frame from the tester. Writes up to max response frames (8 bytes each)
 * to out and returns how many there are. */
int shim_rx(uint32_t can_id, const uint8_t *data, int dlc, uint32_t now, uint8_t *out, int max)
{
  int n = 0;
  int functional = (can_id == ID_FUNC);
  if (can_id != ID_PHYS && !functional) return 0;
  if (functional && (data[0] >> 4) != 0) return 0;

  isotp_frame_t reply;
  unsigned ev = isotp_on_frame(&s_link, data, (uint8_t)dlc, now, &reply);
  if (ev & ISOTP_EV_REPLY) n = emit(&reply, out, n, max);
  if (ev & ISOTP_EV_MESSAGE) {
    uint32_t actions;
    uint16_t len = uds_handle(&s_uds, s_link.rx_buf, s_link.rx_len, functional, now,
                              s_resp, sizeof s_resp, &actions);
    if (actions & UDS_ACTION_ECU_RESET) s_reset_requested = 1;
    isotp_frame_t first;
    if (len > 0 && isotp_send(&s_link, s_resp, len, now, &first) == 0) n = emit(&first, out, n, max);
  }
  isotp_frame_t cf;
  while (isotp_poll(&s_link, now, &cf)) n = emit(&cf, out, n, max);   /* CFs released by an FC */
  return n < max ? n : max;
}

void shim_set_dtc(int dtc, int failed) { uds_set_dtc(&s_uds, (uds_dtc_t)dtc, failed); }
int  shim_reset_requested(void)        { return s_reset_requested; }
int  shim_period(void)                 { return s_period; }
int  shim_session(void)                { return s_uds.session; }
