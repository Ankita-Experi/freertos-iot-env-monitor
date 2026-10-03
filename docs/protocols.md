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

## UDS diagnostics — ISO 14229-1 over ISO-TP (ISO 15765-2)

The node runs a UDS diagnostic server inside the CAN task (`App/Src/diag.c`,
`uds.c`, `isotp.c`). Any standard UDS tester works; `tools/uds_client.py` is a
small one built on python-can.

### Addressing and transport

| CAN ID | Direction | Use |
|--------|-----------|-----|
| `0x7E0` | tester → node | physical requests (single- or multi-frame) |
| `0x7DF` | tester → all nodes | functional requests (single frames only) |
| `0x7E8` | node → tester | responses |

- ISO-TP normal addressing, classic CAN, frames padded to 8 bytes with `0xCC`.
- The node advertises Flow Control **BS = 0, STmin = 0** and honours the tester's BS/STmin when it sends.
- Maximum message size: 128 bytes. N_Bs / N_Cr timeouts: 1000 ms.
- A second bxCAN filter bank (32-bit **ID-list mode**) admits exactly `0x7E0` and `0x7DF`.

### Services

| SID | Service | Session | Notes |
|-----|---------|---------|-------|
| `0x10` | DiagnosticSessionControl | any | `0x01` default, `0x03` extended. Response carries P2 = 50 ms, P2* = 5000 ms. |
| `0x11` | ECUReset | any | `0x01` hard, `0x03` soft. The node resets ~50 ms after the positive response. |
| `0x14` | ClearDiagnosticInformation | any | `FFFFFF` = all DTCs, or one 3-byte DTC. |
| `0x19` | ReadDTCInformation | any | `0x01` count by status mask, `0x02` list by status mask, `0x0A` all supported DTCs. |
| `0x22` | ReadDataByIdentifier | any | Several DIDs per request; unsupported DIDs are skipped (`0x31` only if none are supported). |
| `0x27` | SecurityAccess | extended | Level 1: `0x01` request seed, `0x02` send key. 3 invalid keys → `0x36`, then `0x37` for 10 s. |
| `0x2E` | WriteDataByIdentifier | extended + unlocked | Only DID `0x0101` is writable. |
| `0x3E` | TesterPresent | any | Keeps a non-default session alive. |

- The **suppressPosRspMsgIndicationBit** (sub-function bit 7) is honoured.
- An extended session falls back to default after **5 s (S3)** without a request, and security relocks.
- On functional requests, NRCs `0x11`, `0x12` and `0x31` are suppressed, as ISO 14229-1 requires.

> The seed/key algorithm (`uds_compute_key()`) is a demonstration function, not a
> secure one; production ECUs use a secret, OEM-specific algorithm.

### Data identifiers (big-endian unless noted)

| DID | Access | Content |
|-----|--------|---------|
| `0xF186` | R | Active diagnostic session (1 byte) |
| `0xF18C` | R | ECU serial number: the STM32 96-bit unique ID as 24 ASCII hex characters |
| `0xF195` | R | Software version, ASCII (`ENVMON-1.2.0`) |
| `0x0100` | R | Latest sample: same 8-byte layout as the CAN `ENV_DATA` frame (little-endian) |
| `0x0101` | R/W | Sample period, uint16 ms, 100–10000 |
| `0x0102` | R | Node status: same 8-byte layout as the CAN `NODE_STATUS` frame (little-endian) |

### DTCs

| DTC | Meaning | Set when |
|-----|---------|----------|
| `0xA10101` | Sensor communication failure | BME280 not answering on I2C |
| `0xA10201` | CAN bus-off | the controller entered bus-off |
| `0xA10301` | Watchdog reset | the last reset was an IWDG timeout (recorded once at boot) |
| `0xA10401` | Clock failure | HSE failed and the MCU runs from HSI |
| `0xA10501` | Task stall | a FreeRTOS task missed its heartbeat deadline |

Status byte bits maintained (availability mask `0x2F`): `0x01` testFailed,
`0x02` testFailedThisOperationCycle, `0x04` pendingDTC, `0x08` confirmedDTC,
`0x20` testFailedSinceLastClear. When a fault clears, only `testFailed` drops;
the history bits stay until a ClearDiagnosticInformation request.

### Example session

```text
$ python3 tools/uds_client.py set-period 250
  -> 10 03                     DiagnosticSessionControl, extended
  <- 50 03 00 32 01 F4         P2 = 50 ms, P2* = 5000 ms
  -> 27 01                     SecurityAccess, request seed
  <- 67 01 8C 1D 52 E7         seed (random each time)
  -> 27 02 xx xx xx xx         key = f(seed)
  <- 67 02                     unlocked
  -> 2E 01 01 00 FA            WriteDataByIdentifier 0x0101 = 250 ms
  <- 6E 01 01
```

With Linux can-utils and the kernel ISO-TP module:

```bash
isotprecv -s 7E0 -d 7E8 can0 &                    # listen for the response first
echo "22 F1 95" | isotpsend -s 7E0 -d 7E8 -p CC can0
# isotprecv prints: 62 F1 95 45 4E 56 4D 4F 4E 2D 31 2E 32 2E 30   ("ENVMON-1.2.0")
```
