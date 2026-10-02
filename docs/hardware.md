# Hardware

## Bill of materials

| Qty | Part | Notes |
|-----|------|-------|
| 1 | NUCLEO-F446RE | STM32F446RE, Cortex-M4F, 180 MHz, on-board ST-LINK/V2-1 |
| 1 | BME280 breakout (3.3 V) | I2C; make sure it is a **BME**280 (chip ID 0x60), not a BMP280 (0x58, no humidity) |
| 1 | SN65HVD230 CAN transceiver breakout | 3.3 V transceiver; many breakouts already include a 120 Ω terminator |
| 1 | ESP32 dev kit (e.g. ESP32-DevKitC) | Wi-Fi gateway |
| 1 | USB-CAN adapter (CANable, PCAN-USB, …) | Second bus node for ACKs + monitoring from a PC |
| 2 | 120 Ω resistors | Only if neither end already has termination |
| — | Twisted pair for CANH/CANL, jumper wires | |

## Wiring

```
NUCLEO-F446RE                         BME280
  3V3  ─────────────────────────────── VIN
  GND  ─────────────────────────────── GND
  PB8 / D15 (I2C1_SCL) ─────────────── SCL
  PB9 / D14 (I2C1_SDA) ─────────────── SDA
                                       SDO → GND  (address 0x76)

NUCLEO-F446RE                         SN65HVD230
  3V3  ─────────────────────────────── 3V3
  GND  ─────────────────────────────── GND
  PA12 / CN10-12 (CAN1_TX) ─────────── CTX (D)
  PA11 / CN10-14 (CAN1_RX) ─────────── CRX (R)
                                       CANH ═╗ twisted pair to the USB-CAN adapter
                                       CANL ═╝

NUCLEO-F446RE                         ESP32
  PA9  / D8 (USART1_TX) ────────────── GPIO16 (UART2 RX)
  PA10 / D2 (USART1_RX) ────────────── GPIO17 (UART2 TX)
  GND  ─────────────────────────────── GND   ← required, or the UART will be garbage
```

The debug console (USART2) is routed through the ST-LINK USB cable as a virtual COM
port — no extra wiring. Open it at **115200 8N1**.

### CAN termination

A CAN bus needs exactly **two 120 Ω terminators, one at each physical end**. With
everything powered off, measure resistance between CANH and CANL:

| Reading | Meaning |
|---------|---------|
| ≈ 60 Ω | Correct: two 120 Ω in parallel |
| ≈ 120 Ω | Only one terminator — add one at the other end |
| ≈ 40 Ω | Three terminators — remove one (check breakout boards for an on-board resistor) |
| open | No termination — bus will not work reliably even on a bench |

### Single-board testing without a second CAN node

CAN requires another node to acknowledge every frame. With the transceiver alone,
every transmission fails with an ACK error and the TX error counter climbs to
error-passive. To test the firmware with just the Nucleo, build with
`-DCAN_SELF_TEST=ON`, which puts bxCAN in **silent-loopback** mode: frames are
looped back internally and nothing is driven onto the pins.

### Clock source

The firmware runs the PLL from the 8 MHz MCO output of the on-board ST-LINK (HSE
bypass), which is the default on NUCLEO-F446RE. If that clock is missing (MCO solder
bridges open on some board revisions — see UM1724), it falls back to the internal HSI and
reports `HSI_FALLBACK` — usable for development, but HSI's ±1 % tolerance is outside
what CAN timing should rely on.
