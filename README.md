# FreeRTOS IoT Environmental Monitor with CAN Bus

[![CI](https://github.com/Ankita-Experi/freertos-iot-env-monitor/actions/workflows/ci.yml/badge.svg)](https://github.com/Ankita-Experi/freertos-iot-env-monitor/actions/workflows/ci.yml)

A real-time environmental monitoring node on an **STM32 Nucleo-F446RE (ARM Cortex-M4F)**
running **FreeRTOS**. It samples a Bosch **BME280** over interrupt-driven I2C, broadcasts
readings on a **500 kbit/s CAN bus**, and forwards them through an **ESP32 Wi-Fi gateway**
to a live **ThingSpeak** cloud dashboard — sensor to cloud, with a watchdog-supervised
health monitor keeping every stage honest.

```
BME280 ──I2C──▶ STM32F446RE (FreeRTOS, 4 tasks) ──CAN 500k──▶ bus / USB-CAN adapter
                        │
                        └──UART──▶ ESP32 ──Wi-Fi/HTTP──▶ ThingSpeak dashboard
```

## Highlights

- **Four concurrent FreeRTOS tasks** with deliberate priority assignment —
  CAN (4) › sensor (3) › uplink (2) › health (1). [Why that order →](docs/architecture.md#why-these-priorities)
- **ISR-to-task signalling with binary semaphores**: I2C and UART transfers are
  interrupt-driven; the task sleeps until the completion ISR calls `xSemaphoreGiveFromISR`.
- **ISR-safe queues**: the CAN RX interrupt drains the hardware FIFO into a FreeRTOS
  queue with `xQueueSendFromISR`; a **queue set** lets one task service TX and RX.
- **Mutex-protected UART logging** with priority inheritance, so console output from
  any task never interleaves and never causes unbounded priority inversion.
- **bxCAN at 500 kbit/s**, interrupt-driven RX, **11-bit hardware ID filtering**
  (32-bit mask mode, rejects extended/remote frames), TX mailbox flow control via a
  counting semaphore, error-warning/passive/bus-off tracking, auto bus-off recovery.
- **Health monitoring**: per-task heartbeats in an event group gate the **independent
  watchdog** — a hung task causes a clean reset, reported on the next boot. Stack
  high-water marks, heap, and CAN/I2C error counters are logged and published.
- **Register-level BME280 driver** with the datasheet's integer compensation (no FPU
  needed), platform-independent and **unit-tested on the host** against Bosch's
  published worked example and the float reference formulas.
- **ESP32 gateway**: checksum-validated UART framing, per-window averaging,
  sample-loss detection, Wi-Fi reconnect with exponential back-off, ThingSpeak rate-limit aware.

## Repository layout

```
firmware/
  stm32/                     STM32CubeF4 HAL + FreeRTOS, CMake build
    App/                     application: tasks, drivers, protocol
      Src/sensor_task.c      BME280 acquisition (xTaskDelayUntil)
      Src/can_task.c         queue-set TX/RX + command handling
      Src/uplink_task.c      UART framing to the ESP32
      Src/health_task.c      heartbeat supervision, IWDG, diagnostics
      Src/can_bus.c          bxCAN driver: filter, ISRs, mailbox credits
      Src/i2c_bus.c          interrupt-driven I2C + binary semaphore
      Src/uart_port.c        interrupt-driven UART + mutex
      Src/bme280.c           portable BME280 driver
      Src/protocol.c         CAN/UART wire formats
    Core/                    board bring-up: clocks, pins, IRQs, HAL timebase
    test/                    host unit tests (BME280 math, protocol)
    lib/                     git submodules: FreeRTOS-Kernel, STM32 HAL, CMSIS
  esp32-gateway/             Arduino sketch: UART → HTTP → ThingSpeak
tools/
  can_bittiming.py           bxCAN prescaler / segment calculator
  can_monitor.py             python-can decoder + command sender
docs/
  architecture.md            task model, sync primitives, IRQ priorities, fault handling
  protocols.md               CAN frame layout and UART line format
  hardware.md                BOM, wiring, termination
  bring-up.md                bring-up checklist, oscilloscope checks, fault injection
```

## Hardware

NUCLEO-F446RE, BME280 breakout, SN65HVD230 CAN transceiver, ESP32 dev kit, and a
USB-CAN adapter as the second bus node. Full wiring in [docs/hardware.md](docs/hardware.md).

| Function | STM32 pins |
|----------|------------|
| BME280 (I2C1, 400 kHz) | PB8 SCL / PB9 SDA |
| CAN1 → SN65HVD230 | PA12 TX / PA11 RX |
| ESP32 uplink (USART1) | PA9 TX / PA10 RX |
| Debug console (USART2) | ST-LINK virtual COM port |

## Building the STM32 firmware

Requirements: CMake ≥ 3.20, [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi-gcc`).

```bash
git clone --recurse-submodules --shallow-submodules https://github.com/Ankita-Experi/freertos-iot-env-monitor.git
cd freertos-iot-env-monitor/firmware/stm32

cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
cmake --build build
```

Options:

| CMake option | Default | Effect |
|--------------|---------|--------|
| `-DCAN_SELF_TEST=ON` | OFF | bxCAN silent-loopback — test with the Nucleo alone, no transceiver |
| `-DBME280_I2C_ADDR=0x77` | `0x76` | Sensor address (SDO tied high) |
| `-DCMAKE_BUILD_TYPE=Release` | Debug | `-O2` |

**Flashing:** copy `build/env_monitor.bin` onto the `NOD_F446RE` USB drive, or use
STM32CubeProgrammer / OpenOCD:

```bash
openocd -f board/st_nucleo_f4.cfg -c "program build/env_monitor.elf verify reset exit"
```

STM32CubeIDE users can import the folder as a CMake project (File › Import › CMake).

## ESP32 gateway

1. Create a ThingSpeak channel with fields 1–7 (see the header of the sketch for the mapping).
2. `cp firmware/esp32-gateway/secrets.h.example firmware/esp32-gateway/secrets.h` and fill it in.
3. Open `esp32-gateway.ino` in the Arduino IDE (ESP32 board package installed), select your board, upload.

## Tests

```bash
make -C firmware/stm32/test        # BME280 compensation + protocol, with ASan/UBSan
python3 -m pytest tools            # bit-timing solver + CAN decoder
```

CI builds both firmware variants and runs all tests on every push.

## Console output

The format of the debug console at 115200 baud (values will vary):

```
[    0.000] I boot: FreeRTOS IoT environmental monitor  (built ...)
[    0.000] I boot: SYSCLK 180 MHz from HSE 8 MHz, reset cause: power-on
[    0.001] I can: bxCAN up: 500 kbit/s, filter 0x200/0x7F0, normal mode
[    0.012] I sensor: BME280 found at 0x76, conversion <= 10 ms
[    0.025] I sensor: #0 T=23.41C RH=41.87% P=100912Pa
[    1.025] I sensor: #1 T=23.42C RH=41.85% P=100913Pa
...
[   30.000] I health: stack free (words): can=... sensor=... uplink=... health=...
[   30.000] I health: CAN tx=... txto=0 rx=0 ... TEC=0 REC=0
```

## Design notes

- **Why forced mode for the BME280?** The sensor sleeps between samples (≈0.1 µA),
  self-heating is negligible, and every reading is a fresh, coherent T/P/H triple
  read in one 8-byte burst.
- **Why drop instead of block on full queues?** A late sample is worth less than the
  next on-time one; producers count drops instead of stalling, and the count is reported.
- **Why no LEC interrupt on CAN?** With no node to ACK, every automatic retransmission
  raises an ACK error — enabling the last-error-code interrupt turns that into an
  interrupt storm. LEC is polled from `CAN_ESR` instead.
- **Why TIM6 for the HAL tick?** FreeRTOS owns SysTick; sharing it breaks either the
  kernel tick or HAL timeouts.

More in [docs/architecture.md](docs/architecture.md).

## Roadmap

- [ ] SAE J1939-style 29-bit identifiers and PGNs for the telemetry frames
- [ ] DMA for USART TX; tickless idle for low-power operation
- [ ] TLS with a pinned CA for the gateway → ThingSpeak link (currently plain HTTP)
- [ ] On-device anomaly detection with TensorFlow Lite for Microcontrollers

## License

MIT — see [LICENSE](LICENSE). Third-party code in `firmware/stm32/lib/` keeps its own
license (FreeRTOS: MIT; STM32Cube HAL / CMSIS: BSD-3-Clause / Apache-2.0).
