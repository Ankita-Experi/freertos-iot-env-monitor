# Bring-up and debugging procedure

A step-by-step checklist for bringing the node up on real hardware, with the expected
measurement at each step. Scope and logic-analyzer captures from this procedure belong
in [`docs/images/`](images/).

## 1. Power, clock and console

1. Flash the Debug build. LD2 should blink at **0.5 Hz** (toggled by the health task —
   so a steady blink proves the scheduler and the lowest-priority task are both running).
2. Open the ST-LINK virtual COM port at 115200 8N1. Expected banner:
   ```
   [    0.000] I boot: FreeRTOS IoT environmental monitor  (built ...)
   [    0.000] I boot: SYSCLK 180 MHz from HSE 8 MHz, reset cause: power-on
   ```
3. If you see `HSI (HSE failed!)`, check the ST-LINK MCO solder bridge before
   trusting any CAN timing measurement.
4. Fast LED blink (~10 Hz) = `board_fatal()`. Halt in the debugger and inspect
   `g_assert_file`/`g_assert_line`, `g_overflow_task`, or `g_fault_cfsr`.

## 2. I2C / BME280

| Symptom on console | Likely cause |
|--------------------|--------------|
| `BME280 init failed (-1) - check wiring/pull-ups` | No ACK: wiring, wrong address (SDO pin), or missing pull-ups |
| `wrong chip ID (BMP280?)` | Board is a BMP280 — no humidity sensor |
| Periodic `measurement failed (-1)` | Marginal pull-ups at 400 kHz — try 4.7 kΩ, or lower `ClockSpeed` to 100 kHz |

**Logic analyzer check** (SCL = PB8, SDA = PB9): decode one sample cycle. Expect a
1-byte write to `0xF4` (forced mode), status reads of `0xF3`, then an 8-byte burst
read starting at `0xF7`. SCL period ≈ 2.5 µs at 400 kHz. Rise time should be well
under the 300 ns fast-mode limit; slow, rounded edges mean the pull-ups are too weak.

## 3. CAN physical layer

Measure with the bus **powered off** first: CANH–CANL ≈ 60 Ω (see
[hardware.md](hardware.md#can-termination)).

Then power up, connect the USB-CAN adapter as the second node, and probe CANH and
CANL to GND (or use a differential probe).

| Measurement | Expected |
|-------------|----------|
| Recessive level, CANH and CANL | both ≈ 2.3 V (SN65HVD230 at 3.3 V); differential ≈ 0 V |
| Dominant differential (CANH − CANL) | ≈ 2 V (spec: 1.5 – 3.0 V) |
| Bit time (narrowest pulse) | **2.000 µs** at 500 kbit/s |
| Ringing on edges | Should settle well before the sample point (88.9 % of the bit ≈ 1.78 µs in) |

Trigger on the CAN TX pin (PA12) falling edge — the start-of-frame — to get a stable
capture. Many scopes can decode CAN directly; set 500 kbit/s and confirm ID `0x101`
with DLC 8.

## 4. CAN bit timing

The configured timing is: 45 MHz APB1 / prescaler 5 → 9 MHz time-quantum clock,
1 + 15 + 2 = 18 tq per bit, sample point 88.9 %, SJW 2 tq.
`python3 tools/can_bittiming.py` shows the alternatives.

Symptoms of a timing mismatch between nodes, and what to check:

| Symptom | Check |
|---------|-------|
| Adapter shows bursts of error frames; node TEC climbing; `LEC=form` or `LEC=stuff` | Bit-rate mismatch. Measure the bit time on the scope — it must be 2.000 µs. A common cause is computing the prescaler from the 90 MHz APB1 *timer* clock instead of the 45 MHz PCLK1 that clocks bxCAN. |
| Works on a short bench bus, fails on a longer cable | Sample point too early for the propagation delay, or reflections from missing termination |
| `LEC=ack`, TEC rising to 128+, `TX stalled` on console | No other node ACKing: adapter not connected, at a different bit rate, or in listen-only mode |
| Works with HSE, fails with `HSI_FALLBACK` | Oscillator tolerance — HSI drift is too large for reliable CAN |

The node reads TEC, REC and the last error code (LEC) from the `CAN_ESR` register and
reports them in the console's 30-second diagnostic line and in every NODE_STATUS frame,
so timing problems are visible without a debugger attached.

## 5. End-to-end

```bash
python3 tools/can_monitor.py --interface socketcan --channel can0
python3 tools/can_monitor.py --interface socketcan --channel can0 --set-period 250
```

Expect ENV frames at the new rate within one period, and `lost 0` on the monitor.
On the ESP32 serial monitor, expect an `upload: ok, entry N` line every 20 s and the
values on the ThingSpeak channel.

## 6. Fault-injection checks

| Action | Expected behaviour |
|--------|--------------------|
| Unplug the BME280 while running | `measurement failed` ×3 → re-init attempts each period; `SENSOR_OK` clears; CAN/uplink keep running |
| Re-plug the BME280 | `BME280 found` and samples resume, without a reset |
| Disconnect the USB-CAN adapter | One `TX stalled … no other node ACKing?` warning (not one per frame); sampling unaffected |
| Short CANH to CANL briefly | Bus-off; hardware auto-recovery; `CAN_BUSOFF` flag in the next status frame |
| Temporarily add `vTaskSuspend(NULL);` inside the uplink task loop | Health reports `task 'uplink' silent …`, withholds the refresh, board resets within ~4 s, next boot shows `reset cause: IWDG timeout` and `WDT_RESET` in NODE_STATUS |
| Unplug the ESP32 | Uplink write failures logged every 10th failure; sensor timing unaffected |
