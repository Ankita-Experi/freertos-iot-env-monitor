# Wire protocols

## CAN — 500 kbit/s, 11-bit standard identifiers

All multi-byte fields are **little-endian**.

### `0x101` ENV_DATA (node → bus, every sample period)

| Byte | Type | Field | Unit |
|------|------|-------|------|
| 0–1 | int16 | temperature | 0.01 °C |
| 2–3 | uint16 | relative humidity | 0.01 %RH (0–10000) |
| 4–6 | uint24 | pressure | 1 Pa |
| 7 | uint8 | sequence number (low byte) | — |

Example: `D9 F9 04 12 CD 8B 01 34` → −15.75 °C, 46.12 %RH, 1013.25 hPa, seq 0x34.
Values saturate rather than wrap. A receiver detects lost frames from gaps in the sequence byte.

### `0x102` NODE_STATUS (node → bus, every 5 s or on request)

| Byte | Type | Field |
|------|------|-------|
| 0 | uint8 | flags (below) |
| 1 | uint8 | CAN transmit error counter (TEC) |
| 2 | uint8 | CAN receive error counter (REC) |
| 3–4 | uint16 | free FreeRTOS heap, bytes (saturates at 65535) |
| 5–7 | uint24 | uptime, seconds |

| Flag bit | Name | Meaning |
|----------|------|---------|
| 0x01 | SENSOR_OK | Last BME280 measurement succeeded |
| 0x02 | CAN_WARNING | TEC or REC ≥ 96 |
| 0x04 | CAN_PASSIVE | TEC or REC ≥ 128 |
| 0x08 | CAN_BUSOFF | Bus-off now, or since the previous status frame |
| 0x10 | HSI_FALLBACK | HSE failed at boot; running from internal RC (CAN timing marginal) |
| 0x20 | WDT_RESET | Last reset was caused by the independent watchdog |
| 0x40 | TASK_STALL | A task is currently missing its heartbeat deadline |

### `0x200`–`0x20F` COMMAND (bus → node)

The node's hardware acceptance filter (bank 0, 32-bit mask mode) passes only
**standard data frames** with `ID & 0x7F0 == 0x200`. Extended and remote frames are
rejected in hardware, never reaching software.

| Byte 0 | Command | Payload | Effect |
|--------|---------|---------|--------|
| `0x01` | SET_PERIOD | bytes 1–2: uint16 period ms, 100–10000 | Changes the sample period from the next cycle |
| `0x02` | REQUEST_STATUS | — | Node sends one NODE_STATUS frame immediately |

```bash
# Linux can-utils examples
cansend can0 200#01F401      # sample every 500 ms
cansend can0 200#02          # request status
candump can0,101:7FF,102:7FF
```

## UART uplink — STM32 USART1 → ESP32 UART2, 115200 8N1

NMEA-style ASCII lines, one per message:

```
$ENV,<seq>,<temp_c_x100>,<rh_x100>,<pressure_pa>*HH\r\n
$STA,<flags_hex>,<tec>,<rec>,<free_heap>,<uptime_s>*HH\r\n
```

`HH` is the XOR of every character between `$` and `*`, as two upper-case hex
digits. Example:

```
$ENV,42,2345,4120,101325*58
```

Why ASCII rather than binary: lines are human-readable on a USB-serial adapter
during bring-up, the `$` resynchronises the receiver after any corruption, and at
one line per second the overhead is irrelevant. The full 32-bit sequence number
(vs. 8 bits on CAN) lets the gateway count losses across long gaps.
