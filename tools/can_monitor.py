#!/usr/bin/env python3
"""
CAN bus monitor and command tool for the environmental monitor node.

Decodes ENV_DATA (0x101) and NODE_STATUS (0x102) frames and can send commands.
Works with any python-can interface: SocketCAN (Linux + USB adapter such as a
CANable / PCAN-USB), slcan, pcan, etc.

    pip install python-can

    # Linux SocketCAN, 500 kbit/s:
    sudo ip link set can0 up type can bitrate 500000
    python3 tools/can_monitor.py --interface socketcan --channel can0

    # Windows/macOS with a CANable in slcan mode:
    python3 tools/can_monitor.py --interface slcan --channel COM5 --bitrate 500000

    # Commands
    python3 tools/can_monitor.py ... --set-period 250     # sample every 250 ms
    python3 tools/can_monitor.py ... --request-status
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

ID_ENV_DATA = 0x101
ID_NODE_STATUS = 0x102
ID_CMD = 0x200
CMD_SET_PERIOD = 0x01
CMD_REQUEST_STATUS = 0x02

STATUS_FLAGS = {
    0x01: "SENSOR_OK",
    0x02: "CAN_WARNING",
    0x04: "CAN_PASSIVE",
    0x08: "CAN_BUSOFF",
    0x10: "HSI_FALLBACK",
    0x20: "WDT_RESET",
    0x40: "TASK_STALL",
}


def decode_env(data: bytes) -> dict:
    """ENV_DATA: int16 temp 0.01C | uint16 RH 0.01% | uint24 Pa | uint8 seq (little-endian)."""
    if len(data) != 8:
        raise ValueError(f"ENV_DATA must be 8 bytes, got {len(data)}")
    temp, rh = struct.unpack_from("<hH", data, 0)
    pressure = data[4] | (data[5] << 8) | (data[6] << 16)
    return {"temp_c": temp / 100, "rh_pct": rh / 100, "pressure_hpa": pressure / 100, "seq": data[7]}


def decode_status(data: bytes) -> dict:
    """NODE_STATUS: flags | TEC | REC | uint16 free heap | uint24 uptime s."""
    if len(data) != 8:
        raise ValueError(f"NODE_STATUS must be 8 bytes, got {len(data)}")
    flags, tec, rec, heap = struct.unpack_from("<BBBH", data, 0)
    uptime = data[5] | (data[6] << 8) | (data[7] << 16)
    names = [n for bit, n in STATUS_FLAGS.items() if flags & bit] or ["-"]
    return {"flags": flags, "flag_names": names, "tec": tec, "rec": rec,
            "free_heap": heap, "uptime_s": uptime}


def encode_set_period(period_ms: int) -> bytes:
    if not 100 <= period_ms <= 10000:
        raise ValueError("period must be 100..10000 ms")
    return struct.pack("<BH", CMD_SET_PERIOD, period_ms)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--set-period", type=int, metavar="MS")
    ap.add_argument("--request-status", action="store_true")
    args = ap.parse_args()

    try:
        import can  # python-can
    except ImportError:
        print("python-can is required: pip install python-can", file=sys.stderr)
        return 1

    with can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate) as bus:
        if args.set_period is not None:
            bus.send(can.Message(arbitration_id=ID_CMD, data=encode_set_period(args.set_period),
                                 is_extended_id=False))
            print(f"sent SET_PERIOD {args.set_period} ms")
        if args.request_status:
            bus.send(can.Message(arbitration_id=ID_CMD, data=bytes([CMD_REQUEST_STATUS]),
                                 is_extended_id=False))
            print("sent REQUEST_STATUS")

        last_seq = None
        lost = 0
        t0 = time.monotonic()
        print("listening... (Ctrl+C to stop)")
        try:
            for msg in bus:
                if msg is None or msg.is_extended_id or msg.is_error_frame:
                    continue
                t = time.monotonic() - t0
                if msg.arbitration_id == ID_ENV_DATA:
                    e = decode_env(bytes(msg.data))
                    if last_seq is not None:
                        lost += (e["seq"] - last_seq - 1) & 0xFF
                    last_seq = e["seq"]
                    print(f"{t:9.3f}  ENV  #{e['seq']:3d}  {e['temp_c']:7.2f} C  {e['rh_pct']:6.2f} %RH  "
                          f"{e['pressure_hpa']:8.2f} hPa  (lost {lost})")
                elif msg.arbitration_id == ID_NODE_STATUS:
                    s = decode_status(bytes(msg.data))
                    print(f"{t:9.3f}  STA  {','.join(s['flag_names'])}  TEC={s['tec']} REC={s['rec']}  "
                          f"heap={s['free_heap']}  up={s['uptime_s']} s")
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
