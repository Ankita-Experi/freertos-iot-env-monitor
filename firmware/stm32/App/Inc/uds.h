#ifndef UDS_H
#define UDS_H

/*
 * UDS (ISO 14229-1) diagnostic server -- the subset an ECU of this size needs.
 *
 *   0x10 DiagnosticSessionControl   default (0x01), extended (0x03)
 *   0x11 ECUReset                   hard (0x01), soft (0x03)
 *   0x14 ClearDiagnosticInformation all DTCs (0xFFFFFF) or one DTC
 *   0x19 ReadDTCInformation         0x01 count by mask, 0x02 by mask, 0x0A supported
 *   0x22 ReadDataByIdentifier       one or more DIDs per request
 *   0x27 SecurityAccess             level 1: seed 0x01 / key 0x02  (extended session)
 *   0x2E WriteDataByIdentifier      extended session + security unlocked
 *   0x3E TesterPresent              keeps a non-default session alive
 *
 * Pure C: transport (ISO-TP), data sources and actions are supplied by the caller,
 * so the whole server is unit-tested on the host.
 */

#include <stdint.h>

/* ---- Service IDs ----------------------------------------------------------- */
#define UDS_SID_SESSION_CONTROL   0x10u
#define UDS_SID_ECU_RESET         0x11u
#define UDS_SID_CLEAR_DTC         0x14u
#define UDS_SID_READ_DTC          0x19u
#define UDS_SID_READ_DID          0x22u
#define UDS_SID_SECURITY_ACCESS   0x27u
#define UDS_SID_WRITE_DID         0x2Eu
#define UDS_SID_TESTER_PRESENT    0x3Eu
#define UDS_SID_NEGATIVE_RESPONSE 0x7Fu
#define UDS_POSITIVE_OFFSET       0x40u
#define UDS_SUPPRESS_POS_RSP      0x80u   /* sub-function bit 7 */

/* ---- Negative response codes ------------------------------------------------ */
#define UDS_NRC_SERVICE_NOT_SUPPORTED           0x11u
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED       0x12u
#define UDS_NRC_INCORRECT_LENGTH                0x13u
#define UDS_NRC_RESPONSE_TOO_LONG               0x14u
#define UDS_NRC_CONDITIONS_NOT_CORRECT          0x22u
#define UDS_NRC_REQUEST_SEQUENCE_ERROR          0x24u
#define UDS_NRC_REQUEST_OUT_OF_RANGE            0x31u
#define UDS_NRC_SECURITY_ACCESS_DENIED          0x33u
#define UDS_NRC_INVALID_KEY                     0x35u
#define UDS_NRC_EXCEEDED_ATTEMPTS               0x36u
#define UDS_NRC_TIME_DELAY_NOT_EXPIRED          0x37u
#define UDS_NRC_SERVICE_NOT_SUPPORTED_IN_SESSION 0x7Fu

/* ---- Sessions ---------------------------------------------------------------- */
#define UDS_SESSION_DEFAULT   0x01u
#define UDS_SESSION_EXTENDED  0x03u

/* ---- Timing (reported in the session-control response) ----------------------- */
#define UDS_P2_SERVER_MS        50u
#define UDS_P2_STAR_SERVER_MS   5000u
#define UDS_S3_SERVER_MS        5000u    /* non-default session times out without requests */
#define UDS_SECURITY_DELAY_MS   10000u   /* lockout after too many invalid keys */
#define UDS_SECURITY_MAX_TRIES  3u

/* ---- Data identifiers ---------------------------------------------------------- */
#define UDS_DID_ACTIVE_SESSION  0xF186u  /* handled inside the server */
#define UDS_DID_ECU_SERIAL      0xF18Cu
#define UDS_DID_SW_VERSION      0xF195u
#define UDS_DID_ENV_DATA        0x0100u  /* same 8-byte layout as CAN ENV_DATA */
#define UDS_DID_SAMPLE_PERIOD   0x0101u  /* uint16 ms, big-endian; writable */
#define UDS_DID_NODE_STATUS     0x0102u  /* same 8-byte layout as CAN NODE_STATUS */

/* ---- DTCs ----------------------------------------------------------------------- */
typedef enum {
  UDS_DTC_SENSOR_COMM = 0,   /* BME280 not responding */
  UDS_DTC_CAN_BUS_OFF,       /* CAN controller entered bus-off */
  UDS_DTC_WATCHDOG_RESET,    /* last reset was an IWDG timeout */
  UDS_DTC_CLOCK_FALLBACK,    /* HSE failed, running from HSI */
  UDS_DTC_TASK_STALL,        /* a FreeRTOS task missed its heartbeat deadline */
  UDS_DTC_COUNT
} uds_dtc_t;

/* DTC status bits (ISO 14229-1 D.2) this server maintains. */
#define UDS_DTC_TEST_FAILED              0x01u
#define UDS_DTC_TEST_FAILED_THIS_CYCLE   0x02u
#define UDS_DTC_PENDING                  0x04u
#define UDS_DTC_CONFIRMED                0x08u
#define UDS_DTC_TEST_FAILED_SINCE_CLEAR  0x20u
#define UDS_DTC_AVAILABILITY_MASK        0x2Fu

/* Actions the application must perform after the response has been sent. */
#define UDS_ACTION_NONE        0u
#define UDS_ACTION_ECU_RESET   1u

typedef struct {
  /* Read a DID into out. Return the length written, or -1 if the DID is not supported. */
  int     (*read_did)(void *ctx, uint16_t did, uint8_t *out, uint16_t max);
  /* Write a DID. Return 0 on success or a negative-response code. */
  uint8_t (*write_did)(void *ctx, uint16_t did, const uint8_t *in, uint16_t len);
  /* Source of security seeds. */
  uint32_t (*random32)(void *ctx);
  void *ctx;
} uds_callbacks_t;

typedef struct {
  uds_callbacks_t cb;
  uint8_t  session;
  uint8_t  unlocked;
  uint8_t  seed_issued;
  uint32_t seed;
  uint8_t  failed_attempts;
  uint32_t lockout_until_ms;
  uint8_t  lockout_active;
  uint32_t last_request_ms;
  uint8_t  dtc_status[UDS_DTC_COUNT];
} uds_server_t;

void uds_init(uds_server_t *srv, const uds_callbacks_t *cb);

/*
 * Handle one complete request. functional = 1 if it arrived on the functional
 * (broadcast) address. Writes the response into resp and returns its length;
 * 0 means "send nothing". *actions receives UDS_ACTION_* flags.
 */
uint16_t uds_handle(uds_server_t *srv, const uint8_t *req, uint16_t len, int functional,
                    uint32_t now_ms, uint8_t *resp, uint16_t resp_max, uint32_t *actions);

/* Call periodically: expires the S3 session timer. */
void uds_tick(uds_server_t *srv, uint32_t now_ms);

/* Report the current result of a fault check. */
void uds_set_dtc(uds_server_t *srv, uds_dtc_t dtc, int failed);

uint32_t uds_dtc_code(uds_dtc_t dtc);

/* Demo seed/key algorithm, shared with tools/uds_client.py. NOT a secure algorithm. */
uint32_t uds_compute_key(uint32_t seed);

#endif /* UDS_H */
