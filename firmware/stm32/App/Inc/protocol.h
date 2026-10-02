#ifndef PROTOCOL_H
#define PROTOCOL_H

/*
 * Wire formats shared by the STM32 node, the ESP32 gateway and tools/.
 * Full description: docs/protocols.md.
 *
 * Everything here is pure (no HAL, no RTOS) so it is unit-tested on the host.
 */

#include <stddef.h>
#include <stdint.h>

/* ---- Telemetry sample -------------------------------------------------- */
typedef struct {
  uint32_t seq;                  /* increments per successful sample; wraps */
  int32_t  temperature_c_x100;   /* 0.01 degC  */
  uint32_t humidity_rh_x100;     /* 0.01 %RH   */
  uint32_t pressure_pa;          /* 1 Pa       */
  uint32_t timestamp_ms;         /* FreeRTOS tick at acquisition */
} env_sample_t;

/* ---- CAN (11-bit standard IDs, 500 kbit/s) ----------------------------- */
#define CAN_ID_ENV_DATA      0x101u  /* node -> bus, every sample period          */
#define CAN_ID_NODE_STATUS   0x102u  /* node -> bus, every 5 s or on request      */
#define CAN_ID_CMD_BASE      0x200u  /* bus -> node, 0x200-0x20F accepted by filter */
#define CAN_ID_CMD_MASK      0x7F0u

#define CAN_CMD_SET_PERIOD     0x01u /* data[1..2] = period ms (LE), 100..10000 */
#define CAN_CMD_REQUEST_STATUS 0x02u /* reply with one NODE_STATUS frame now      */

#define SAMPLE_PERIOD_MIN_MS  100u
#define SAMPLE_PERIOD_MAX_MS  10000u

/* Node status flags (NODE_STATUS byte 0). */
#define STATUS_FLAG_SENSOR_OK     0x01u
#define STATUS_FLAG_CAN_WARNING   0x02u  /* TEC or REC >= 96  */
#define STATUS_FLAG_CAN_PASSIVE   0x04u  /* TEC or REC >= 128 */
#define STATUS_FLAG_CAN_BUSOFF    0x08u  /* seen since last status frame */
#define STATUS_FLAG_HSI_FALLBACK  0x10u  /* running without HSE -> CAN timing marginal */
#define STATUS_FLAG_WDT_RESET     0x20u  /* last reset was an IWDG timeout */
#define STATUS_FLAG_TASK_STALL    0x40u  /* a task missed its heartbeat deadline */

typedef struct {
  uint8_t  flags;
  uint8_t  tec;                  /* transmit error counter */
  uint8_t  rec;                  /* receive error counter  */
  uint16_t free_heap_bytes;      /* saturates at 65535 */
  uint32_t uptime_s;             /* 24-bit on the wire  */
} node_status_t;

typedef struct {
  uint8_t  id;                   /* CAN_CMD_* */
  uint16_t period_ms;            /* for CAN_CMD_SET_PERIOD */
} can_command_t;

/* ENV_DATA: [0-1] int16 temp 0.01C | [2-3] uint16 RH 0.01% | [4-6] uint24 Pa | [7] seq & 0xFF */
void protocol_pack_env(const env_sample_t *s, uint8_t data[8]);
void protocol_unpack_env(const uint8_t data[8], env_sample_t *s);

/* NODE_STATUS: [0] flags | [1] TEC | [2] REC | [3-4] uint16 heap | [5-7] uint24 uptime s */
void protocol_pack_status(const node_status_t *st, uint8_t data[8]);

/* Returns 0 and fills *cmd on a valid command, -1 otherwise. */
int protocol_parse_command(uint32_t std_id, const uint8_t *data, uint8_t dlc, can_command_t *cmd);

/* ---- UART uplink to ESP32 (NMEA-style ASCII lines) ---------------------
 *   $ENV,<seq>,<temp_c_x100>,<rh_x100>,<pressure_pa>*HH\r\n
 *   $STA,<flags_hex>,<tec>,<rec>,<free_heap>,<uptime_s>*HH\r\n
 * HH = XOR of all characters between '$' and '*', two upper-case hex digits.
 */
#define UPLINK_LINE_MAX 72u

uint8_t protocol_nmea_checksum(const char *body, size_t len);
/* Return number of characters written (excluding NUL), or 0 if buf too small. */
size_t  protocol_format_env_line(const env_sample_t *s, char *buf, size_t buflen);
size_t  protocol_format_status_line(const node_status_t *st, char *buf, size_t buflen);

#endif /* PROTOCOL_H */
