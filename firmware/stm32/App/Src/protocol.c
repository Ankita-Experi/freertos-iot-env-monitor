#include "protocol.h"

#include <stdio.h>
#include <string.h>

static int16_t clamp_s16(int32_t v)
{
  if (v > INT16_MAX) return INT16_MAX;
  if (v < INT16_MIN) return INT16_MIN;
  return (int16_t)v;
}

static uint32_t clamp_u(uint32_t v, uint32_t max) { return (v > max) ? max : v; }

void protocol_pack_env(const env_sample_t *s, uint8_t d[8])
{
  uint16_t t = (uint16_t)clamp_s16(s->temperature_c_x100);
  uint16_t h = (uint16_t)clamp_u(s->humidity_rh_x100, 10000u);
  uint32_t p = clamp_u(s->pressure_pa, 0xFFFFFFu);

  d[0] = (uint8_t)(t & 0xFF);
  d[1] = (uint8_t)(t >> 8);
  d[2] = (uint8_t)(h & 0xFF);
  d[3] = (uint8_t)(h >> 8);
  d[4] = (uint8_t)(p & 0xFF);
  d[5] = (uint8_t)((p >> 8) & 0xFF);
  d[6] = (uint8_t)((p >> 16) & 0xFF);
  d[7] = (uint8_t)(s->seq & 0xFF);
}

void protocol_unpack_env(const uint8_t d[8], env_sample_t *s)
{
  memset(s, 0, sizeof *s);
  s->temperature_c_x100 = (int16_t)(d[0] | (d[1] << 8));
  s->humidity_rh_x100   = (uint32_t)(d[2] | (d[3] << 8));
  s->pressure_pa        = (uint32_t)d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16);
  s->seq                = d[7];
}

void protocol_pack_status(const node_status_t *st, uint8_t d[8])
{
  uint32_t up = clamp_u(st->uptime_s, 0xFFFFFFu);
  d[0] = st->flags;
  d[1] = st->tec;
  d[2] = st->rec;
  d[3] = (uint8_t)(st->free_heap_bytes & 0xFF);
  d[4] = (uint8_t)(st->free_heap_bytes >> 8);
  d[5] = (uint8_t)(up & 0xFF);
  d[6] = (uint8_t)((up >> 8) & 0xFF);
  d[7] = (uint8_t)((up >> 16) & 0xFF);
}

int protocol_parse_command(uint32_t std_id, const uint8_t *d, uint8_t dlc, can_command_t *cmd)
{
  if ((std_id & CAN_ID_CMD_MASK) != CAN_ID_CMD_BASE || dlc < 1 || d == NULL || cmd == NULL) {
    return -1;
  }
  memset(cmd, 0, sizeof *cmd);
  cmd->id = d[0];

  switch (cmd->id) {
    case CAN_CMD_SET_PERIOD:
      if (dlc < 3) return -1;
      cmd->period_ms = (uint16_t)(d[1] | (d[2] << 8));
      if (cmd->period_ms < SAMPLE_PERIOD_MIN_MS || cmd->period_ms > SAMPLE_PERIOD_MAX_MS) {
        return -1;
      }
      return 0;
    case CAN_CMD_REQUEST_STATUS:
      return 0;
    default:
      return -1;
  }
}

uint8_t protocol_nmea_checksum(const char *body, size_t len)
{
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) {
    x ^= (uint8_t)body[i];
  }
  return x;
}

/* Wrap "<body>" as "$<body>*HH\r\n". */
static size_t finish_line(char *buf, size_t buflen, int body_len)
{
  /* body was written at buf+1; need room for "*HH\r\n" + NUL */
  if (body_len <= 0 || (size_t)body_len + 1u + 6u > buflen) {
    return 0;
  }
  buf[0] = '$';
  uint8_t cs = protocol_nmea_checksum(&buf[1], (size_t)body_len);
  int n = snprintf(&buf[1 + body_len], buflen - 1u - (size_t)body_len, "*%02X\r\n", cs);
  if (n != 5) {
    return 0;
  }
  return 1u + (size_t)body_len + 5u;
}

size_t protocol_format_env_line(const env_sample_t *s, char *buf, size_t buflen)
{
  if (buflen < 8u) return 0;
  int n = snprintf(&buf[1], buflen - 1u, "ENV,%lu,%ld,%lu,%lu",
                   (unsigned long)s->seq, (long)s->temperature_c_x100,
                   (unsigned long)s->humidity_rh_x100, (unsigned long)s->pressure_pa);
  return finish_line(buf, buflen, n);
}

size_t protocol_format_status_line(const node_status_t *st, char *buf, size_t buflen)
{
  if (buflen < 8u) return 0;
  int n = snprintf(&buf[1], buflen - 1u, "STA,%02X,%u,%u,%u,%lu",
                   st->flags, st->tec, st->rec, st->free_heap_bytes,
                   (unsigned long)st->uptime_s);
  return finish_line(buf, buflen, n);
}
