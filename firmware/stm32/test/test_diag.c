/*
 * Host unit tests for the diagnostic stack: ISO-TP (ISO 15765-2) and UDS (ISO 14229-1).
 *   make -C firmware/stm32/test
 */

#include <stdio.h>
#include <string.h>

#include "isotp.h"
#include "uds.h"

static int g_failures;
static int g_checks;

#define CHECK(cond, ...)                                       \
  do {                                                         \
    g_checks++;                                                \
    if (!(cond)) {                                             \
      g_failures++;                                            \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);              \
      printf(__VA_ARGS__);                                     \
      printf("\n");                                            \
    }                                                          \
  } while (0)

static int bytes_eq(const uint8_t *a, const uint8_t *b, size_t n) { return memcmp(a, b, n) == 0; }

/* ===========================================================================
 * ISO-TP
 * ======================================================================== */

static void test_isotp_single_frame(void)
{
  isotp_link_t l;
  isotp_frame_t f, reply;
  isotp_init(&l, 0, 0);

  const uint8_t msg[] = { 0x22, 0xF1, 0x95 };
  CHECK(isotp_send(&l, msg, 3, 0, &f) == 0, "SF send");
  const uint8_t expect[8] = { 0x03, 0x22, 0xF1, 0x95, 0xCC, 0xCC, 0xCC, 0xCC };
  CHECK(f.dlc == 8 && bytes_eq(f.data, expect, 8), "SF bytes + 0xCC padding");
  CHECK(!isotp_tx_busy(&l), "SF leaves TX idle");

  unsigned ev = isotp_on_frame(&l, f.data, f.dlc, 0, &reply);
  CHECK(ev == ISOTP_EV_MESSAGE && l.rx_len == 3 && bytes_eq(l.rx_buf, msg, 3), "SF receive");

  const uint8_t bad[2] = { 0x05, 0x11 };   /* claims 5 bytes in a 2-byte frame */
  CHECK(isotp_on_frame(&l, bad, 2, 0, &reply) == ISOTP_EV_NONE, "short SF rejected");
}

/* Send len bytes from a to b with the given receiver FC settings; checks pacing. */
static int transfer(uint16_t len, uint8_t bs, uint8_t stmin, uint32_t *elapsed_ms)
{
  isotp_link_t tx, rx;
  isotp_init(&tx, 0, 0);
  isotp_init(&rx, bs, stmin);

  uint8_t msg[ISOTP_MAX_PAYLOAD];
  for (uint16_t i = 0; i < len; i++) msg[i] = (uint8_t)(i * 7 + 3);

  uint32_t now = 1000;
  isotp_frame_t f, fc;
  if (isotp_send(&tx, msg, len, now, &f) != 0) return -1;
  if (isotp_on_frame(&rx, f.data, f.dlc, now, &fc) != ISOTP_EV_REPLY) return -2;   /* FF -> FC */
  isotp_on_frame(&tx, fc.data, fc.dlc, now, &f);

  int done = 0;
  for (int step = 0; step < 5000 && !done; step++) {
    isotp_frame_t cf;
    if (isotp_poll(&tx, now, &cf)) {
      unsigned ev = isotp_on_frame(&rx, cf.data, cf.dlc, now, &fc);
      if (ev & ISOTP_EV_REPLY) isotp_on_frame(&tx, fc.data, fc.dlc, now, &f);
      if (ev & ISOTP_EV_MESSAGE) done = 1;
    } else {
      now++;                                   /* nothing due yet: let 1 ms pass */
    }
  }
  *elapsed_ms = now - 1000;
  if (!done) return -3;
  if (rx.rx_len != len || memcmp(rx.rx_buf, msg, len) != 0) return -4;
  return 0;
}

static void test_isotp_multi_frame(void)
{
  uint32_t ms;
  /* 128 bytes = FF(6) + 18 CFs: sequence number wraps 15 -> 0 */
  CHECK(transfer(128, 0, 0, &ms) == 0, "128-byte transfer, BS=0 STmin=0");
  CHECK(ms == 0, "no pacing delay with STmin=0 (took %u ms)", (unsigned)ms);

  int rc = transfer(50, 2, 5, &ms);          /* 50 bytes -> 7 CFs, FC every 2 CFs */
  CHECK(rc == 0, "50-byte transfer, BS=2 STmin=5 (rc=%d)", rc);
  /* 7 CFs: first immediately after each FC, others >= 5 ms apart: CF1 t0, CF2 +5, FC, CF3, CF4 +5 ... */
  CHECK(ms >= 15, "STmin honoured: %u ms for 7 CFs in blocks of 2", (unsigned)ms);

  CHECK(transfer(8, 0, 0, &ms) == 0, "8-byte boundary (smallest multi-frame)");
}

static void test_isotp_errors(void)
{
  isotp_link_t l;
  isotp_frame_t reply, out;

  /* First Frame announcing more than we can hold -> FC overflow. */
  isotp_init(&l, 0, 0);
  const uint8_t big_ff[8] = { 0x10, 0xC8, 1, 2, 3, 4, 5, 6 };   /* 200 bytes */
  CHECK(isotp_on_frame(&l, big_ff, 8, 0, &reply) == ISOTP_EV_REPLY && reply.data[0] == 0x32,
        "oversized FF answered with FC overflow (0x32), got 0x%02X", reply.data[0]);

  /* Wrong sequence number aborts reception. */
  isotp_init(&l, 0, 0);
  const uint8_t ff[8] = { 0x10, 0x0A, 1, 2, 3, 4, 5, 6 };      /* 10 bytes */
  isotp_on_frame(&l, ff, 8, 0, &reply);
  const uint8_t cf_bad[8] = { 0x22, 7, 8, 9, 10, 0xCC, 0xCC, 0xCC };
  CHECK(isotp_on_frame(&l, cf_bad, 8, 0, &reply) == ISOTP_EV_NONE && !l.rx_active && l.rx_errors == 1,
        "bad sequence number aborts");

  /* N_Cr timeout. */
  isotp_init(&l, 0, 0);
  isotp_on_frame(&l, ff, 8, 0, &reply);
  CHECK(isotp_next_event_ms(&l, 0) == ISOTP_TIMEOUT_MS, "N_Cr deadline reported");
  isotp_poll(&l, ISOTP_TIMEOUT_MS, &out);
  CHECK(!l.rx_active && l.timeouts == 1, "N_Cr timeout aborts reception");

  /* N_Bs timeout: no Flow Control after a First Frame. */
  isotp_init(&l, 0, 0);
  uint8_t msg[20] = { 0 };
  isotp_send(&l, msg, 20, 0, &out);
  CHECK(isotp_send(&l, msg, 5, 0, &out) == -1, "second send rejected while busy");
  isotp_poll(&l, ISOTP_TIMEOUT_MS, &out);
  CHECK(!isotp_tx_busy(&l) && l.timeouts == 1, "N_Bs timeout frees the transmitter");

  CHECK(isotp_stmin_to_ms(0xF5) == 1 && isotp_stmin_to_ms(0x14) == 20 && isotp_stmin_to_ms(0x90) == 127,
        "STmin decoding");
}

/* ===========================================================================
 * UDS
 * ======================================================================== */

static uint16_t g_period = 1000;
static uint32_t g_rand = 0xC0FFEE01u;

static int fake_read(void *ctx, uint16_t did, uint8_t *out, uint16_t max)
{
  (void)ctx;
  if (did == UDS_DID_SAMPLE_PERIOD && max >= 2) {
    out[0] = (uint8_t)(g_period >> 8);
    out[1] = (uint8_t)g_period;
    return 2;
  }
  if (did == UDS_DID_SW_VERSION && max >= 5) {
    memcpy(out, "1.2.0", 5);
    return 5;
  }
  return -1;
}

static uint8_t fake_write(void *ctx, uint16_t did, const uint8_t *in, uint16_t len)
{
  (void)ctx;
  if (did != UDS_DID_SAMPLE_PERIOD) return UDS_NRC_REQUEST_OUT_OF_RANGE;
  if (len != 2) return UDS_NRC_INCORRECT_LENGTH;
  uint16_t v = (uint16_t)((in[0] << 8) | in[1]);
  if (v < 100 || v > 10000) return UDS_NRC_REQUEST_OUT_OF_RANGE;
  g_period = v;
  return 0;
}

static uint32_t fake_random(void *ctx) { (void)ctx; return g_rand; }

static uds_server_t g_srv;
static uint8_t  g_resp[ISOTP_MAX_PAYLOAD];
static uint32_t g_actions;

static uint16_t req(const uint8_t *r, uint16_t n, uint32_t now)
{
  return uds_handle(&g_srv, r, n, 0, now, g_resp, sizeof g_resp, &g_actions);
}

#define REQ(now, ...) req((const uint8_t[]){ __VA_ARGS__ }, sizeof((const uint8_t[]){ __VA_ARGS__ }), now)

static int is_nrc(uint16_t n, uint8_t sid, uint8_t nrc)
{
  return n == 3 && g_resp[0] == 0x7F && g_resp[1] == sid && g_resp[2] == nrc;
}

static void uds_setup(void)
{
  uds_callbacks_t cb = { fake_read, fake_write, fake_random, NULL };
  uds_init(&g_srv, &cb);
  g_period = 1000;
}

static void unlock(uint32_t now)
{
  REQ(now, 0x10, 0x03);
  REQ(now, 0x27, 0x01);
  uint32_t k = uds_compute_key(g_rand);
  REQ(now, 0x27, 0x02, (uint8_t)(k >> 24), (uint8_t)(k >> 16), (uint8_t)(k >> 8), (uint8_t)k);
}

static void test_uds_session_and_basics(void)
{
  uds_setup();
  uint16_t n = REQ(0, 0x10, 0x03);
  const uint8_t expect[6] = { 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4 };   /* P2 50 ms, P2* 5000 ms */
  CHECK(n == 6 && bytes_eq(g_resp, expect, 6), "session control 0x03 response");

  n = REQ(0, 0x22, 0xF1, 0x86);
  CHECK(n == 4 && g_resp[3] == 0x03, "DID F186 reports extended session");

  CHECK(REQ(0, 0x3E, 0x80) == 0, "TesterPresent with suppressPosRsp sends nothing");
  CHECK(REQ(0, 0x3E, 0x00) == 2 && g_resp[0] == 0x7E, "TesterPresent positive response");

  CHECK(is_nrc(REQ(0, 0x10, 0x02), 0x10, 0x12), "programming session not supported -> 0x12");
  CHECK(is_nrc(REQ(0, 0x10), 0x10, 0x13), "missing sub-function -> 0x13");
  CHECK(is_nrc(REQ(0, 0x85, 0x01), 0x85, 0x11), "unknown service -> 0x11");

  uint32_t act;
  const uint8_t unk[2] = { 0x85, 0x01 };
  CHECK(uds_handle(&g_srv, unk, 2, 1, 0, g_resp, sizeof g_resp, &act) == 0,
        "functional request: unsupported service stays silent");
}

static void test_uds_s3_timeout(void)
{
  uds_setup();
  unlock(0);
  CHECK(g_srv.unlocked && g_srv.session == UDS_SESSION_EXTENDED, "unlocked in extended session");
  REQ(4000, 0x3E, 0x00);                         /* keeps the session alive */
  uds_tick(&g_srv, 8999);
  CHECK(g_srv.session == UDS_SESSION_EXTENDED, "TesterPresent restarted S3");
  uds_tick(&g_srv, 9000);
  CHECK(g_srv.session == UDS_SESSION_DEFAULT && !g_srv.unlocked, "S3 expiry -> default session, relocked");
}

static void test_uds_read_did(void)
{
  uds_setup();
  uint16_t n = REQ(0, 0x22, 0x01, 0x01, 0xF1, 0x95);
  const uint8_t expect[] = { 0x62, 0x01, 0x01, 0x03, 0xE8, 0xF1, 0x95, '1', '.', '2', '.', '0' };
  CHECK(n == sizeof expect && bytes_eq(g_resp, expect, sizeof expect), "multi-DID read");

  n = REQ(0, 0x22, 0x01, 0x01, 0x12, 0x34);      /* one supported, one not */
  CHECK(n == 5, "unsupported DID skipped when another is supported");
  CHECK(is_nrc(REQ(0, 0x22, 0x12, 0x34), 0x22, 0x31), "only unsupported DID -> 0x31");
  CHECK(is_nrc(REQ(0, 0x22, 0x01), 0x22, 0x13), "odd DID length -> 0x13");
}

static void test_uds_security_and_write(void)
{
  uds_setup();
  CHECK(is_nrc(REQ(0, 0x2E, 0x01, 0x01, 0x01, 0xF4), 0x2E, 0x7F), "write in default session -> 0x7F");
  CHECK(is_nrc(REQ(0, 0x27, 0x01), 0x27, 0x7F), "security access in default session -> 0x7F");

  REQ(0, 0x10, 0x03);
  CHECK(is_nrc(REQ(0, 0x2E, 0x01, 0x01, 0x01, 0xF4), 0x2E, 0x33), "write while locked -> 0x33");
  CHECK(is_nrc(REQ(0, 0x27, 0x02, 0, 0, 0, 0), 0x27, 0x24), "key before seed -> 0x24");

  uint16_t n = REQ(0, 0x27, 0x01);
  CHECK(n == 6 && g_resp[0] == 0x67 && g_resp[2] == 0xC0 && g_resp[5] == 0x01, "seed returned");
  uint32_t k = uds_compute_key(g_rand);
  n = REQ(0, 0x27, 0x02, (uint8_t)(k >> 24), (uint8_t)(k >> 16), (uint8_t)(k >> 8), (uint8_t)k);
  CHECK(n == 2 && g_resp[0] == 0x67 && g_srv.unlocked, "correct key unlocks");

  n = REQ(0, 0x27, 0x01);
  CHECK(n == 6 && g_resp[2] == 0 && g_resp[5] == 0, "seed is zero when already unlocked");

  n = REQ(0, 0x2E, 0x01, 0x01, 0x01, 0xF4);       /* 500 ms */
  CHECK(n == 3 && g_resp[0] == 0x6E && g_period == 500, "sample period written");
  CHECK(is_nrc(REQ(0, 0x2E, 0x01, 0x01, 0x00, 0x32), 0x2E, 0x31), "50 ms rejected -> 0x31");
  CHECK(is_nrc(REQ(0, 0x2E, 0xF1, 0x95, 0x41), 0x2E, 0x31), "read-only DID -> 0x31");
}

static void test_uds_lockout(void)
{
  uds_setup();
  REQ(0, 0x10, 0x03);
  uint8_t nrcs[3];
  for (int i = 0; i < 3; i++) {
    REQ(100, 0x27, 0x01);
    REQ(100, 0x27, 0x02, 0xDE, 0xAD, 0xBE, 0xEF);
    nrcs[i] = g_resp[2];
  }
  CHECK(nrcs[0] == 0x35 && nrcs[1] == 0x35 && nrcs[2] == 0x36, "invalid key x2 -> 0x35, third -> 0x36");
  CHECK(is_nrc(REQ(5000, 0x27, 0x01), 0x27, 0x37), "seed during lockout -> 0x37");
  REQ(9000, 0x3E, 0x00);                          /* keep session alive */
  CHECK(REQ(10100, 0x27, 0x01) == 6, "seed available after the 10 s delay");
}

static void test_uds_dtcs(void)
{
  uds_setup();
  uint16_t n = REQ(0, 0x19, 0x01, 0xFF);
  const uint8_t none[6] = { 0x59, 0x01, 0x2F, 0x01, 0x00, 0x00 };
  CHECK(n == 6 && bytes_eq(g_resp, none, 6), "no DTCs at start");

  uds_set_dtc(&g_srv, UDS_DTC_SENSOR_COMM, 1);
  uds_set_dtc(&g_srv, UDS_DTC_CAN_BUS_OFF, 1);
  uds_set_dtc(&g_srv, UDS_DTC_CAN_BUS_OFF, 0);    /* fault went away: history stays */

  n = REQ(0, 0x19, 0x01, 0x01);                   /* currently failing only */
  CHECK(n == 6 && g_resp[5] == 1, "one DTC currently failing");
  n = REQ(0, 0x19, 0x02, 0x08);                   /* confirmed */
  const uint8_t list[] = { 0x59, 0x02, 0x2F, 0xA1, 0x01, 0x01, 0x2F, 0xA1, 0x02, 0x01, 0x2E };
  CHECK(n == sizeof list && bytes_eq(g_resp, list, sizeof list), "confirmed DTC list with status bytes");

  n = REQ(0, 0x19, 0x0A);
  CHECK(n == 3 + 4 * UDS_DTC_COUNT, "0x0A lists all %d supported DTCs", UDS_DTC_COUNT);

  CHECK(is_nrc(REQ(0, 0x14, 0x12, 0x34, 0x56), 0x14, 0x31), "clear unknown DTC -> 0x31");
  CHECK(REQ(0, 0x14, 0xA1, 0x02, 0x01) == 1 && g_resp[0] == 0x54, "clear one DTC");
  CHECK(REQ(0, 0x14, 0xFF, 0xFF, 0xFF) == 1, "clear all DTCs");
  REQ(0, 0x19, 0x01, 0xFF);
  CHECK(g_resp[5] == 0, "no DTCs after clear");
  CHECK(is_nrc(REQ(0, 0x19, 0x04, 0, 0, 0, 0), 0x19, 0x12), "unsupported report type -> 0x12");
}

static void test_uds_ecu_reset(void)
{
  uds_setup();
  uint16_t n = REQ(0, 0x11, 0x01);
  CHECK(n == 2 && g_resp[0] == 0x51 && (g_actions & UDS_ACTION_ECU_RESET), "hard reset: response + action");
  n = REQ(0, 0x11, 0x83);
  CHECK(n == 0 && (g_actions & UDS_ACTION_ECU_RESET), "suppressed soft reset still resets");
  CHECK(is_nrc(REQ(0, 0x11, 0x02), 0x11, 0x12), "key-off-on reset not supported -> 0x12");
}

static void test_uds_over_isotp(void)
{
  /* Full path: multi-frame request in, multi-frame response out. */
  uds_setup();
  uds_set_dtc(&g_srv, UDS_DTC_SENSOR_COMM, 1);
  uds_set_dtc(&g_srv, UDS_DTC_WATCHDOG_RESET, 1);
  uds_set_dtc(&g_srv, UDS_DTC_TASK_STALL, 1);

  isotp_link_t server, tester;
  isotp_init(&server, 0, 0);
  isotp_init(&tester, 0, 0);
  isotp_frame_t f, fc;

  const uint8_t rq[3] = { 0x19, 0x02, 0xFF };
  isotp_send(&tester, rq, 3, 0, &f);
  CHECK(isotp_on_frame(&server, f.data, f.dlc, 0, &fc) == ISOTP_EV_MESSAGE, "request reassembled");

  uint32_t act;
  uint16_t n = uds_handle(&g_srv, server.rx_buf, server.rx_len, 0, 0, g_resp, sizeof g_resp, &act);
  CHECK(n == 15, "3 DTCs -> 15-byte response (multi-frame)");
  isotp_send(&server, g_resp, n, 0, &f);
  isotp_on_frame(&tester, f.data, f.dlc, 0, &fc);      /* FF -> tester's FC */
  isotp_on_frame(&server, fc.data, fc.dlc, 0, &f);
  isotp_frame_t cf;
  unsigned ev = 0;
  for (int i = 0; i < 4 && !(ev & ISOTP_EV_MESSAGE); i++) {
    if (isotp_poll(&server, 0, &cf)) ev = isotp_on_frame(&tester, cf.data, cf.dlc, 0, &fc);
  }
  CHECK((ev & ISOTP_EV_MESSAGE) && tester.rx_len == 15 && bytes_eq(tester.rx_buf, g_resp, 15),
        "tester received the full response");
}

int main(void)
{
  printf("Running diagnostic stack tests\n");
  test_isotp_single_frame();
  test_isotp_multi_frame();
  test_isotp_errors();
  test_uds_session_and_basics();
  test_uds_s3_timeout();
  test_uds_read_did();
  test_uds_security_and_write();
  test_uds_lockout();
  test_uds_dtcs();
  test_uds_ecu_reset();
  test_uds_over_isotp();
  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
