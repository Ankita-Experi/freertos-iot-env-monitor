# FreeRTOS IoT Environmental Monitor with CAN Bus

[![CI](https://github.com/Ankita-Experi/freertos-iot-env-monitor/actions/workflows/ci.yml/badge.svg)](https://github.com/Ankita-Experi/freertos-iot-env-monitor/actions/workflows/ci.yml)

A real-time environmental monitoring node on an **STM32 Nucleo-F446RE (ARM Cortex-M4F)**
running **FreeRTOS**. It samples a Bosch **BME280** over interrupt-driven I2C, broadcasts
readings on a **500 kbit/s CAN bus**, and forwards them through an **ESP32 Wi-Fi gateway**
to a live **ThingSpeak** cloud dashboard — sensor to cloud, with a watchdog-supervised
health monitor keeping every stage honest. A **UDS (ISO 14229) diagnostic server over
ISO-TP** lets any standard tester read live data, manage trouble codes and reset the node.

```
BME280 ──I2C──▶ STM32F446RE (FreeRTOS, 4 tasks) ◀──CAN 500k──▶ bus / USB-CAN adapter
                        │                         telemetry + UDS diagnostics
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
- **UDS diagnostics (ISO 14229-1) over ISO-TP (ISO 15765-2)**: session control, security
  access (seed/key with attempt lockout), read/write data by identifier, DTC reporting and
  clearing, ECU reset, S3 session timeout. Multi-frame transport with flow control, block
  size and STmin pacing. A second CAN filter bank in ID-list mode admits only `0x7E0`/`0x7DF`.
  [Protocol details →](docs/protocols.md#uds-diagnostics--iso-14229-1-over-iso-tp-iso-15765-2)
- **Health monitoring**: per-task heartbeats in an event group gate the **independent
  watchdog** — a hung task causes a clean reset, reported on the next boot. Stack
  high-water marks, heap, and CAN/I2C error counters are logged and published.
- **Register-level BME280 driver** with the datasheet's integer compensation (no FPU
  needed), platform-independent and **unit-tested on the host** against Bosch's
  published worked example and the float reference formulas.
- **CI/CD in two systems**: a **Jenkins** declarative pipeline (`Jenkinsfile`) and **GitHub
  Actions** both run the C and Python unit tests and cross-compile both firmware variants;
  Jenkins also publishes JUnit results and archives the firmware images. [Run Jenkins locally →](docs/jenkins.md)
- **ESP32 gateway**: checksum-validated UART framing, per-window averaging,
  sample-loss detection, Wi-Fi reconnect with exponential back-off, ThingSpeak rate-limit aware.

## Repository layout

```
firmware/
  stm32/                     STM32CubeF4 HAL + FreeRTOS, CMake build
    App/                     application: tasks, drivers, protocol
      Src/sensor_task.c      BME280 acquisition (xTaskDelayUntil)
      Src/can_task.c         queue-set TX/RX, command handling, UDS routing
      Src/diag.c             UDS server integration: DIDs, DTCs, ECU reset
      Src/uds.c              portable UDS (ISO 14229-1) service layer
      Src/isotp.c            portable ISO-TP (ISO 15765-2) transport
      Src/uplink_task.c      UART framing to the ESP32
      Src/health_task.c      heartbeat supervision, IWDG, diagnostics
      Src/can_bus.c          bxCAN driver: filter, ISRs, mailbox credits
      Src/i2c_bus.c          interrupt-driven I2C + binary semaphore
      Src/uart_port.c        interrupt-driven UART + mutex
      Src/bme280.c           portable BME280 driver
      Src/protocol.c         CAN/UART wire formats
    Core/                    board bring-up: clocks, pins, IRQs, HAL timebase
    test/                    host unit tests (BME280 math, protocol, ISO-TP, UDS)
    lib/                     git submodules: FreeRTOS-Kernel, STM32 HAL, CMSIS
  esp32-gateway/             Arduino sketch: UART → HTTP → ThingSpeak
tools/
  can_bittiming.py           bxCAN prescaler / segment calculator
  can_monitor.py             python-can decoder + command sender
  uds_client.py              UDS tester: info, live data, DTCs, set-period, reset
ci/jenkins/                  Jenkins image with the ARM toolchain and plugins
Jenkinsfile                  declarative pipeline
docs/
  architecture.md            task model, sync primitives, IRQ priorities, fault handling
  protocols.md               CAN frames, UDS services/DIDs/DTCs, UART line format
  jenkins.md                 running the Jenkins pipeline locally
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

## UDS diagnostics

With a USB-CAN adapter on the bus (`pip install python-can`):

```bash
python3 tools/uds_client.py info              # software version, ECU serial, session
python3 tools/uds_client.py live              # live sensor data and node status
python3 tools/uds_client.py dtc               # stored trouble codes with status bits
python3 tools/uds_client.py set-period 250    # extended session -> security access -> write DID
python3 tools/uds_client.py reset
```

## Tests

```bash
make -C firmware/stm32/test        # BME280 math, protocol, ISO-TP, UDS (85 checks, ASan/UBSan)
python3 -m pytest tools            # bit timing, CAN decoder, UDS client end-to-end against the C server
```

The UDS end-to-end tests compile the firmware's ISO-TP and UDS sources into a host shared
library and drive them with `tools/uds_client.py`, so the tester and the server are checked
against each other without hardware. GitHub Actions and Jenkins both run everything on each change.

## Console output

The format of the debug console at 115200 baud (values will vary):

```
[    0.000] I boot: FreeRTOS IoT environmental monitor  (built ...)
[    0.000] I boot: SYSCLK 180 MHz from HSE 8 MHz, reset cause: power-on
[    0.001] I can: bxCAN up: 500 kbit/s, filters 0x200/0x7F0 + UDS 0x7E0/0x7DF, normal mode
[    0.012] I sensor: BME280 found at 0x76, conversion <= 10 ms
[    0.025] I sensor: #0 T=23.41C RH=41.87% P=100912Pa
[    1.025] I sensor: #1 T=23.42C RH=41.85% P=100913Pa
[    4.310] I uds: 0x10 -> 0x50, 6 bytes
[    4.318] I uds: 0x22 -> 0x62, 15 bytes
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
- **Why run UDS inside the CAN task?** That task already owns CAN TX, so the ISO-TP and
  UDS state need no locks, and responses can't interleave with telemetry frames mid-message.
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
