#!/usr/bin/env python3
"""
UDS (ISO 14229-1) tester for the environmental monitor node.

Talks to the node's diagnostic server over ISO-TP (ISO 15765-2) on CAN:
request ID 0x7E0, response ID 0x7E8, 500 kbit/s. Uses python-can for the bus
and a small built-in ISO-TP implementation, so the only dependency is python-can.

    pip install python-can
    sudo ip link set can0 up type can bitrate 500000      # Linux SocketCAN

    python3 tools/uds_client.py info                  # software version, serial, session
    python3 tools/uds_client.py live                  # live sensor data + node status
    python3 tools/uds_client.py set-period 250        # extended session + security + write
    python3 tools/uds_client.py dtc                   # stored trouble codes
    python3 tools/uds_client.py clear-dtc
    python3 tools/uds_client.py reset                 # hard reset

    # other adapters, e.g. a CANable in slcan mode on Windows:
    python3 tools/uds_client.py --interface slcan --channel COM5 info
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Callable, Optional

REQ_ID = 0x7E0
FUNC_ID = 0x7DF
RESP_ID = 0x7E8
PAD = 0xCC

# Data identifiers (docs/protocols.md)
DID_ACTIVE_SESSION = 0xF186
DID_ECU_SERIAL = 0xF18C
DID_SW_VERSION = 0xF195
DID_ENV_DATA = 0x0100
DID_SAMPLE_PERIOD = 0x0101
DID_NODE_STATUS = 0x0102

DTC_NAMES = {
    0xA10101: "Sensor communication failure (BME280)",
    0xA10201: "CAN bus-off",
    0xA10301: "Watchdog reset",
    0xA10401: "HSE clock failure, running on HSI",
    0xA10501: "FreeRTOS task stall",
}

NRC_NAMES = {
    0x11: "serviceNotSupported", 0x12: "subFunctionNotSupported", 0x13: "incorrectMessageLength",
    0x14: "responseTooLong", 0x22: "conditionsNotCorrect", 0x24: "requestSequenceError",
    0x31: "requestOutOfRange", 0x33: "securityAccessDenied", 0x35: "invalidKey",
    0x36: "exceededNumberOfAttempts", 0x37: "requiredTimeDelayNotExpired", 0x78: "responsePending",
    0x7F: "serviceNotSupportedInActiveSession",
}

SendFn = Callable[[int, bytes], None]
RecvFn = Callable[[float], Optional[tuple[int, bytes]]]


class NegativeResponse(Exception):
    def __init__(self, sid: int, nrc: int):
        self.sid, self.nrc = sid, nrc
        super().__init__(f"service 0x{sid:02X}: NRC 0x{nrc:02X} {NRC_NAMES.get(nrc, 'unknown')}")


def compute_key(seed: int) -> int:
    """Demo seed/key algorithm, identical to uds_compute_key() in the firmware. Not secure."""
    x = (seed ^ 0x5A3C96E1) & 0xFFFFFFFF
    x = ((x << 7) | (x >> 25)) & 0xFFFFFFFF
    return (x + 0x1234ABCD) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# ISO-TP (tester side)
# ---------------------------------------------------------------------------

def _pad(b: bytes) -> bytes:
    return b + bytes([PAD]) * (8 - len(b))


def single_frame(payload: bytes) -> bytes:
    if not 1 <= len(payload) <= 7:
        raise ValueError("single frame carries 1-7 bytes")
    return _pad(bytes([len(payload)]) + payload)


def first_frame(payload: bytes) -> bytes:
    n = len(payload)
    if not 8 <= n <= 4095:
        raise ValueError("first frame announces 8-4095 bytes")
    return bytes([0x10 | (n >> 8), n & 0xFF]) + payload[:6]


def consecutive_frames(payload: bytes) -> list[bytes]:
    frames, sn = [], 1
    for off in range(6, len(payload), 7):
        frames.append(_pad(bytes([0x20 | sn]) + payload[off:off + 7]))
        sn = (sn + 1) & 0x0F
    return frames


def stmin_seconds(stmin: int) -> float:
    if stmin <= 0x7F:
        return stmin / 1000
    if 0xF1 <= stmin <= 0xF9:
        return (stmin - 0xF0) / 10000
    return 0x7F / 1000


class IsoTp:
    """Blocking ISO-TP client: one request out, one response back."""

    def __init__(self, send: SendFn, recv: RecvFn, tx_id: int = REQ_ID, rx_id: int = RESP_ID,
                 timeout: float = 1.0):
        self.send, self.recv = send, recv
        self.tx_id, self.rx_id, self.timeout = tx_id, rx_id, timeout

    def _next(self, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise TimeoutError("no response from node")
            msg = self.recv(left)
            if msg is not None and msg[0] == self.rx_id:
                return msg[1]

    def send_message(self, payload: bytes, functional: bool = False) -> None:
        if len(payload) <= 7:
            self.send(FUNC_ID if functional else self.tx_id, single_frame(payload))
            return
        if functional:
            raise ValueError("functional requests must fit in a single frame")
        self.send(self.tx_id, first_frame(payload))
        cfs = consecutive_frames(payload)
        i = 0
        while i < len(cfs):
            fc = self._next(self.timeout)
            if fc[0] >> 4 != 3:
                continue
            status = fc[0] & 0x0F
            if status == 1:                      # WAIT
                continue
            if status != 0:
                raise IOError("receiver reported overflow")
            bs, gap = fc[1], stmin_seconds(fc[2])
            block = cfs[i:] if bs == 0 else cfs[i:i + bs]
            for j, cf in enumerate(block):
                if j:
                    time.sleep(gap)
                self.send(self.tx_id, cf)
            i += len(block)

    def receive_message(self, timeout: float) -> bytes:
        while True:
            d = self._next(timeout)
            pci = d[0] >> 4
            if pci == 0:
                return d[1:1 + (d[0] & 0x0F)]
            if pci != 1:
                continue                         # stray CF / FC: ignore
            total = ((d[0] & 0x0F) << 8) | d[1]
            data = bytearray(d[2:8])
            self.send(self.tx_id, _pad(bytes([0x30, 0x00, 0x00])))   # CTS, no blocks, no gap
            sn = 1
            while len(data) < total:
                cf = self._next(self.timeout)
                if cf[0] >> 4 != 2:
                    continue
                if cf[0] & 0x0F != sn:
                    raise IOError(f"ISO-TP sequence error: expected {sn}, got {cf[0] & 0x0F}")
                data += cf[1:1 + min(7, total - len(data))]
                sn = (sn + 1) & 0x0F
            return bytes(data)


# ---------------------------------------------------------------------------
# UDS client
# ---------------------------------------------------------------------------

class UdsClient:
    P2_STAR = 5.0   # server's extended response timeout, seconds

    def __init__(self, tp: IsoTp):
        self.tp = tp

    def request(self, payload: bytes, functional: bool = False, expect_response: bool = True) -> bytes:
        self.tp.send_message(payload, functional)
        if not expect_response:
            return b""
        timeout = self.tp.timeout
        while True:
            resp = self.tp.receive_message(timeout)
            if resp[:1] == b"\x7F" and len(resp) >= 3:
                if resp[2] == 0x78:              # responsePending: keep waiting
                    timeout = self.P2_STAR
                    continue
                raise NegativeResponse(resp[1], resp[2])
            if resp[0] != payload[0] + 0x40:
                raise IOError(f"unexpected response 0x{resp[0]:02X} to service 0x{payload[0]:02X}")
            return resp

    def session(self, session: int) -> tuple[int, int]:
        r = self.request(bytes([0x10, session]))
        p2_ms, p2_star_10ms = struct.unpack(">HH", r[2:6])
        return p2_ms, p2_star_10ms * 10

    def tester_present(self) -> None:
        self.request(bytes([0x3E, 0x00]))

    def ecu_reset(self, kind: int = 0x01) -> None:
        self.request(bytes([0x11, kind]))

    def read_did(self, did: int) -> bytes:
        r = self.request(bytes([0x22]) + struct.pack(">H", did))
        if struct.unpack(">H", r[1:3])[0] != did:
            raise IOError("response echoes a different DID")
        return r[3:]

    def write_did(self, did: int, data: bytes) -> None:
        self.request(bytes([0x2E]) + struct.pack(">H", did) + data)

    def unlock(self) -> None:
        seed = struct.unpack(">I", self.request(bytes([0x27, 0x01]))[2:6])[0]
        if seed == 0:
            return                               # already unlocked
        self.request(bytes([0x27, 0x02]) + struct.pack(">I", compute_key(seed)))

    def read_dtcs(self, mask: int = 0xFF) -> list[tuple[int, int]]:
        r = self.request(bytes([0x19, 0x02, mask]))
        return parse_dtc_records(r[3:])

    def clear_dtcs(self, group: int = 0xFFFFFF) -> None:
        self.request(bytes([0x14]) + group.to_bytes(3, "big"))


def parse_dtc_records(data: bytes) -> list[tuple[int, int]]:
    return [(int.from_bytes(data[i:i + 3], "big"), data[i + 3]) for i in range(0, len(data) - 3, 4)]


def describe_status(status: int) -> str:
    bits = [(0x01, "failing now"), (0x04, "pending"), (0x08, "confirmed"), (0x20, "failed since clear")]
    return ", ".join(name for bit, name in bits if status & bit) or "-"


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def _bus_transport(args) -> tuple[IsoTp, object]:
    import can  # python-can

    bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate,
                  can_filters=[{"can_id": RESP_ID, "can_mask": 0x7FF, "extended": False}])

    def send(can_id: int, data: bytes) -> None:
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))

    def recv(timeout: float):
        m = bus.recv(timeout)
        return None if m is None else (m.arbitration_id, bytes(m.data))

    return IsoTp(send, recv), bus


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=500000)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("live")
    sp = sub.add_parser("set-period")
    sp.add_argument("ms", type=int)
    sub.add_parser("dtc")
    sub.add_parser("clear-dtc")
    rp = sub.add_parser("reset")
    rp.add_argument("kind", nargs="?", choices=["hard", "soft"], default="hard")
    args = ap.parse_args()

    try:
        tp, bus = _bus_transport(args)
    except ImportError:
        print("python-can is required: pip install python-can", file=sys.stderr)
        return 1

    from can_monitor import decode_env, decode_status

    uds = UdsClient(tp)
    try:
        if args.cmd == "info":
            print("Software version:", uds.read_did(DID_SW_VERSION).decode())
            print("ECU serial      :", uds.read_did(DID_ECU_SERIAL).decode())
            print("Active session  : 0x%02X" % uds.read_did(DID_ACTIVE_SESSION)[0])
        elif args.cmd == "live":
            e = decode_env(uds.read_did(DID_ENV_DATA))
            s = decode_status(uds.read_did(DID_NODE_STATUS))
            period = struct.unpack(">H", uds.read_did(DID_SAMPLE_PERIOD))[0]
            print(f"{e['temp_c']:.2f} C  {e['rh_pct']:.2f} %RH  {e['pressure_hpa']:.2f} hPa  (sample #{e['seq']})")
            print(f"status: {','.join(s['flag_names'])}  TEC={s['tec']} REC={s['rec']}  "
                  f"heap={s['free_heap']}  up={s['uptime_s']} s  period={period} ms")
        elif args.cmd == "set-period":
            uds.session(0x03)
            uds.unlock()
            uds.write_did(DID_SAMPLE_PERIOD, struct.pack(">H", args.ms))
            print(f"sample period set to {args.ms} ms")
        elif args.cmd == "dtc":
            dtcs = uds.read_dtcs()
            if not dtcs:
                print("no stored DTCs")
            for code, status in dtcs:
                print(f"0x{code:06X}  status 0x{status:02X} ({describe_status(status)})  "
                      f"{DTC_NAMES.get(code, '')}")
        elif args.cmd == "clear-dtc":
            uds.clear_dtcs()
            print("DTCs cleared")
        elif args.cmd == "reset":
            uds.ecu_reset(0x01 if args.kind == "hard" else 0x03)
            print("node is resetting")
    except (NegativeResponse, TimeoutError, IOError) as exc:
        print("error:", exc, file=sys.stderr)
        return 2
    finally:
        bus.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
