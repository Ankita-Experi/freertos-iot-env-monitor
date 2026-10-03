#include "uds.h"

#include <string.h>

/* Project-specific 3-byte DTC numbers (documented in docs/protocols.md). */
static const uint32_t k_dtc_codes[UDS_DTC_COUNT] = {
  [UDS_DTC_SENSOR_COMM]    = 0xA10101u,
  [UDS_DTC_CAN_BUS_OFF]    = 0xA10201u,
  [UDS_DTC_WATCHDOG_RESET] = 0xA10301u,
  [UDS_DTC_CLOCK_FALLBACK] = 0xA10401u,
  [UDS_DTC_TASK_STALL]     = 0xA10501u,
};

static int reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

uint32_t uds_dtc_code(uds_dtc_t dtc)
{
  return (dtc < UDS_DTC_COUNT) ? k_dtc_codes[dtc] : 0;
}

uint32_t uds_compute_key(uint32_t seed)
{
  uint32_t x = seed ^ 0x5A3C96E1u;
  x = (x << 7) | (x >> 25);
  return x + 0x1234ABCDu;
}

void uds_init(uds_server_t *srv, const uds_callbacks_t *cb)
{
  memset(srv, 0, sizeof *srv);
  srv->cb = *cb;
  srv->session = UDS_SESSION_DEFAULT;
}

static void enter_default_session(uds_server_t *srv)
{
  srv->session     = UDS_SESSION_DEFAULT;
  srv->unlocked    = 0;   /* security is always relocked when leaving a session */
  srv->seed_issued = 0;
}

void uds_tick(uds_server_t *srv, uint32_t now)
{
  if (srv->session != UDS_SESSION_DEFAULT && reached(now, srv->last_request_ms + UDS_S3_SERVER_MS)) {
    enter_default_session(srv);
  }
  if (srv->lockout_active && reached(now, srv->lockout_until_ms)) {
    srv->lockout_active  = 0;
    srv->failed_attempts = 0;
  }
}

void uds_set_dtc(uds_server_t *srv, uds_dtc_t dtc, int failed)
{
  if (dtc >= UDS_DTC_COUNT) return;
  if (failed) {
    srv->dtc_status[dtc] |= UDS_DTC_TEST_FAILED | UDS_DTC_TEST_FAILED_THIS_CYCLE | UDS_DTC_PENDING |
                            UDS_DTC_CONFIRMED | UDS_DTC_TEST_FAILED_SINCE_CLEAR;
  } else {
    srv->dtc_status[dtc] &= (uint8_t)~UDS_DTC_TEST_FAILED;   /* history bits stay until cleared */
  }
}

/* ===========================================================================
 * Response helpers
 * ======================================================================== */

typedef struct {
  uint8_t *buf;
  uint16_t max;
  uint16_t len;
  int      overflow;
} out_t;

static void put(out_t *o, uint8_t b)
{
  if (o->len < o->max) o->buf[o->len++] = b;
  else o->overflow = 1;
}

static uint16_t negative(uint8_t *resp, uint8_t sid, uint8_t nrc)
{
  resp[0] = UDS_SID_NEGATIVE_RESPONSE;
  resp[1] = sid;
  resp[2] = nrc;
  return 3;
}

/* ===========================================================================
 * Services. Each returns a response length, or 0x8000|NRC for a negative response.
 * ======================================================================== */

#define NRC(code) ((uint16_t)(0x8000u | (code)))

static uint16_t svc_session_control(uds_server_t *srv, const uint8_t *req, uint16_t len, out_t *o)
{
  if (len != 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
  uint8_t sub = req[1] & 0x7Fu;
  if (sub != UDS_SESSION_DEFAULT && sub != UDS_SESSION_EXTENDED) {
    return NRC(UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  enter_default_session(srv);   /* any session change relocks security */
  srv->session = sub;

  put(o, UDS_SID_SESSION_CONTROL + UDS_POSITIVE_OFFSET);
  put(o, sub);
  put(o, (uint8_t)(UDS_P2_SERVER_MS >> 8));
  put(o, (uint8_t)(UDS_P2_SERVER_MS & 0xFF));
  put(o, (uint8_t)((UDS_P2_STAR_SERVER_MS / 10u) >> 8));      /* P2* in 10 ms units */
  put(o, (uint8_t)((UDS_P2_STAR_SERVER_MS / 10u) & 0xFF));
  return o->len;
}

static uint16_t svc_ecu_reset(const uint8_t *req, uint16_t len, out_t *o, uint32_t *actions)
{
  if (len != 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
  uint8_t sub = req[1] & 0x7Fu;
  if (sub != 0x01u && sub != 0x03u) return NRC(UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
  *actions |= UDS_ACTION_ECU_RESET;
  put(o, UDS_SID_ECU_RESET + UDS_POSITIVE_OFFSET);
  put(o, sub);
  return o->len;
}

static uint16_t svc_tester_present(const uint8_t *req, uint16_t len, out_t *o)
{
  if (len != 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
  if ((req[1] & 0x7Fu) != 0x00u) return NRC(UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
  put(o, UDS_SID_TESTER_PRESENT + UDS_POSITIVE_OFFSET);
  put(o, 0x00);
  return o->len;
}

static uint16_t svc_read_did(uds_server_t *srv, const uint8_t *req, uint16_t len, out_t *o)
{
  if (len < 3 || ((len - 1u) % 2u) != 0) return NRC(UDS_NRC_INCORRECT_LENGTH);

  put(o, UDS_SID_READ_DID + UDS_POSITIVE_OFFSET);
  int supported = 0;
  for (uint16_t i = 1; i + 1u < len; i += 2) {
    uint16_t did = (uint16_t)((req[i] << 8) | req[i + 1]);
    uint8_t  tmp[40];
    int n;
    if (did == UDS_DID_ACTIVE_SESSION) {
      tmp[0] = srv->session;
      n = 1;
    } else {
      n = srv->cb.read_did ? srv->cb.read_did(srv->cb.ctx, did, tmp, sizeof tmp) : -1;
    }
    if (n < 0) continue;            /* unsupported DIDs are skipped... */
    supported = 1;
    put(o, req[i]);
    put(o, req[i + 1]);
    for (int k = 0; k < n; k++) put(o, tmp[k]);
  }
  if (!supported) return NRC(UDS_NRC_REQUEST_OUT_OF_RANGE);   /* ...unless none were supported */
  if (o->overflow) return NRC(UDS_NRC_RESPONSE_TOO_LONG);
  return o->len;
}

static uint16_t svc_write_did(uds_server_t *srv, const uint8_t *req, uint16_t len, out_t *o)
{
  if (srv->session == UDS_SESSION_DEFAULT) return NRC(UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
  if (len < 4) return NRC(UDS_NRC_INCORRECT_LENGTH);
  if (!srv->unlocked) return NRC(UDS_NRC_SECURITY_ACCESS_DENIED);

  uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
  uint8_t nrc = srv->cb.write_did ? srv->cb.write_did(srv->cb.ctx, did, &req[3], (uint16_t)(len - 3u))
                                  : UDS_NRC_REQUEST_OUT_OF_RANGE;
  if (nrc != 0) return NRC(nrc);

  put(o, UDS_SID_WRITE_DID + UDS_POSITIVE_OFFSET);
  put(o, req[1]);
  put(o, req[2]);
  return o->len;
}

static uint16_t svc_security_access(uds_server_t *srv, const uint8_t *req, uint16_t len,
                                    uint32_t now, out_t *o)
{
  if (srv->session == UDS_SESSION_DEFAULT) return NRC(UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION);
  if (len < 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
  uint8_t sub = req[1] & 0x7Fu;

  if (sub == 0x01u) {                                       /* requestSeed */
    if (len != 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
    if (srv->lockout_active) return NRC(UDS_NRC_TIME_DELAY_NOT_EXPIRED);
    uint32_t seed = 0;                                      /* already unlocked -> zero seed */
    if (!srv->unlocked) {
      do {
        seed = srv->cb.random32 ? srv->cb.random32(srv->cb.ctx) : 0x12345678u;
      } while (seed == 0);
      srv->seed = seed;
      srv->seed_issued = 1;
    }
    put(o, UDS_SID_SECURITY_ACCESS + UDS_POSITIVE_OFFSET);
    put(o, sub);
    put(o, (uint8_t)(seed >> 24));
    put(o, (uint8_t)(seed >> 16));
    put(o, (uint8_t)(seed >> 8));
    put(o, (uint8_t)seed);
    return o->len;
  }

  if (sub == 0x02u) {                                       /* sendKey */
    if (len != 6) return NRC(UDS_NRC_INCORRECT_LENGTH);
    if (!srv->seed_issued) return NRC(UDS_NRC_REQUEST_SEQUENCE_ERROR);
    srv->seed_issued = 0;                                   /* each seed is good for one try */
    uint32_t key = ((uint32_t)req[2] << 24) | ((uint32_t)req[3] << 16) | ((uint32_t)req[4] << 8) | req[5];
    if (key != uds_compute_key(srv->seed)) {
      if (++srv->failed_attempts >= UDS_SECURITY_MAX_TRIES) {
        srv->lockout_active   = 1;
        srv->lockout_until_ms = now + UDS_SECURITY_DELAY_MS;
        return NRC(UDS_NRC_EXCEEDED_ATTEMPTS);
      }
      return NRC(UDS_NRC_INVALID_KEY);
    }
    srv->unlocked = 1;
    srv->failed_attempts = 0;
    put(o, UDS_SID_SECURITY_ACCESS + UDS_POSITIVE_OFFSET);
    put(o, sub);
    return o->len;
  }

  return NRC(UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
}

static void put_dtc(out_t *o, unsigned i, uint8_t status)
{
  put(o, (uint8_t)(k_dtc_codes[i] >> 16));
  put(o, (uint8_t)(k_dtc_codes[i] >> 8));
  put(o, (uint8_t)k_dtc_codes[i]);
  put(o, status);
}

static uint16_t svc_read_dtc(uds_server_t *srv, const uint8_t *req, uint16_t len, out_t *o)
{
  if (len < 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
  uint8_t sub = req[1] & 0x7Fu;

  switch (sub) {
    case 0x01: case 0x02: {                                 /* by status mask */
      if (len != 3) return NRC(UDS_NRC_INCORRECT_LENGTH);
      uint8_t mask = req[2] & UDS_DTC_AVAILABILITY_MASK;
      put(o, UDS_SID_READ_DTC + UDS_POSITIVE_OFFSET);
      put(o, sub);
      put(o, UDS_DTC_AVAILABILITY_MASK);
      if (sub == 0x01) {
        uint16_t count = 0;
        for (unsigned i = 0; i < UDS_DTC_COUNT; i++) {
          if (srv->dtc_status[i] & mask) count++;
        }
        put(o, 0x01);                                       /* ISO 14229-1 DTC format */
        put(o, (uint8_t)(count >> 8));
        put(o, (uint8_t)count);
      } else {
        for (unsigned i = 0; i < UDS_DTC_COUNT; i++) {
          if (srv->dtc_status[i] & mask) put_dtc(o, i, srv->dtc_status[i]);
        }
      }
      break;
    }
    case 0x0A:                                              /* all supported DTCs */
      if (len != 2) return NRC(UDS_NRC_INCORRECT_LENGTH);
      put(o, UDS_SID_READ_DTC + UDS_POSITIVE_OFFSET);
      put(o, sub);
      put(o, UDS_DTC_AVAILABILITY_MASK);
      for (unsigned i = 0; i < UDS_DTC_COUNT; i++) put_dtc(o, i, srv->dtc_status[i]);
      break;
    default:
      return NRC(UDS_NRC_SUBFUNCTION_NOT_SUPPORTED);
  }
  if (o->overflow) return NRC(UDS_NRC_RESPONSE_TOO_LONG);
  return o->len;
}

static uint16_t svc_clear_dtc(uds_server_t *srv, const uint8_t *req, uint16_t len, out_t *o)
{
  if (len != 4) return NRC(UDS_NRC_INCORRECT_LENGTH);
  uint32_t group = ((uint32_t)req[1] << 16) | ((uint32_t)req[2] << 8) | req[3];
  int matched = 0;
  for (unsigned i = 0; i < UDS_DTC_COUNT; i++) {
    if (group == 0xFFFFFFu || group == k_dtc_codes[i]) {
      srv->dtc_status[i] = 0;
      matched = 1;
    }
  }
  if (!matched) return NRC(UDS_NRC_REQUEST_OUT_OF_RANGE);
  put(o, UDS_SID_CLEAR_DTC + UDS_POSITIVE_OFFSET);
  return o->len;
}

/* ===========================================================================
 * Dispatcher
 * ======================================================================== */

static int has_subfunction(uint8_t sid)
{
  return sid == UDS_SID_SESSION_CONTROL || sid == UDS_SID_ECU_RESET || sid == UDS_SID_READ_DTC ||
         sid == UDS_SID_SECURITY_ACCESS || sid == UDS_SID_TESTER_PRESENT;
}

uint16_t uds_handle(uds_server_t *srv, const uint8_t *req, uint16_t len, int functional,
                    uint32_t now, uint8_t *resp, uint16_t resp_max, uint32_t *actions)
{
  *actions = UDS_ACTION_NONE;
  if (len == 0 || resp_max < 3) return 0;

  uds_tick(srv, now);            /* expire S3 / lockout before acting on the request */
  srv->last_request_ms = now;

  uint8_t sid = req[0];
  out_t o = { resp, resp_max, 0, 0 };
  uint16_t r;

  switch (sid) {
    case UDS_SID_SESSION_CONTROL: r = svc_session_control(srv, req, len, &o); break;
    case UDS_SID_ECU_RESET:       r = svc_ecu_reset(req, len, &o, actions); break;
    case UDS_SID_CLEAR_DTC:       r = svc_clear_dtc(srv, req, len, &o); break;
    case UDS_SID_READ_DTC:        r = svc_read_dtc(srv, req, len, &o); break;
    case UDS_SID_READ_DID:        r = svc_read_did(srv, req, len, &o); break;
    case UDS_SID_SECURITY_ACCESS: r = svc_security_access(srv, req, len, now, &o); break;
    case UDS_SID_WRITE_DID:       r = svc_write_did(srv, req, len, &o); break;
    case UDS_SID_TESTER_PRESENT:  r = svc_tester_present(req, len, &o); break;
    default:                      r = NRC(UDS_NRC_SERVICE_NOT_SUPPORTED); break;
  }

  if (r & 0x8000u) {
    uint8_t nrc = (uint8_t)(r & 0xFFu);
    /* ISO 14229-1 7.5: on functional requests, stay silent for "not supported" style NRCs. */
    if (functional && (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED || nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED ||
                       nrc == UDS_NRC_REQUEST_OUT_OF_RANGE)) {
      return 0;
    }
    return negative(resp, sid, nrc);
  }

  /* Positive response suppressed on request (sub-function bit 7). Actions still apply. */
  if (len >= 2 && has_subfunction(sid) && (req[1] & UDS_SUPPRESS_POS_RSP)) {
    return 0;
  }
  return r;
}
