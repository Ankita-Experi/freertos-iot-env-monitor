"""Tests for tools/ (no hardware or python-can needed): python3 -m pytest tools"""

import collections
import ctypes
import pathlib
import struct
import subprocess
import time

import pytest

from can_bittiming import solve
from can_monitor import decode_env, decode_status, encode_set_period
from uds_client import (IsoTp, NegativeResponse, UdsClient, compute_key, consecutive_frames,
                        first_frame, parse_dtc_records, single_frame, stmin_seconds)


def test_firmware_timing_is_a_valid_solution():
    # board.c: 45 MHz APB1, prescaler 5, BS1 15, BS2 2 -> 500 kbit/s, 88.9 %
    sols = solve(45e6, 500e3, 0.875)
    t = next(s for s in sols if (s.prescaler, s.bs1, s.bs2) == (5, 15, 2))
    assert abs(t.bitrate - 500e3) < 1e-3
    assert abs(t.sample_point - 16 / 18) < 1e-9
    assert t.max_sjw >= 2  # firmware uses SJW = 2


def test_all_solutions_are_exact_and_in_window():
    for s in solve(45e6, 500e3, 0.875):
        assert abs(s.bitrate - 500e3) < 1e-3
        assert 0.75 <= s.sample_point <= 0.90


def test_decode_env_matches_firmware_encoding():
    # Same vector as firmware/stm32/test/test_host.c
    frame = bytes([0xD9, 0xF9, 0x04, 0x12, 0xCD, 0x8B, 0x01, 0x34])
    e = decode_env(frame)
    assert e == {"temp_c": -15.75, "rh_pct": 46.12, "pressure_hpa": 1013.25, "seq": 0x34}


def test_decode_status():
    frame = bytes([0x21, 8, 0]) + struct.pack("<H", 17000) + (3600).to_bytes(3, "little")
    s = decode_status(frame)
    assert s["flag_names"] == ["SENSOR_OK", "WDT_RESET"]
    assert (s["tec"], s["rec"], s["free_heap"], s["uptime_s"]) == (8, 0, 17000, 3600)


def test_encode_set_period():
    assert encode_set_period(500) == bytes([0x01, 0xF4, 0x01])


# ---------------------------------------------------------------------------
# UDS / ISO-TP client
# ---------------------------------------------------------------------------


def test_isotp_frame_builders():
    assert single_frame(bytes([0x22, 0xF1, 0x95])) == bytes([0x03, 0x22, 0xF1, 0x95, 0xCC, 0xCC, 0xCC, 0xCC])
    payload = bytes(range(20))
    assert first_frame(payload) == bytes([0x10, 20, 0, 1, 2, 3, 4, 5])
    cfs = consecutive_frames(payload)
    assert [cf[0] for cf in cfs] == [0x21, 0x22]
    assert cfs[1] == bytes([0x22, 13, 14, 15, 16, 17, 18, 19])
    assert len(consecutive_frames(bytes(128))) == 18 and consecutive_frames(bytes(128))[15][0] == 0x20  # SN wraps


def test_stmin_and_key():
    assert stmin_seconds(20) == 0.020 and stmin_seconds(0xF5) == 0.0005
    # Same vector as the firmware: C and Python must agree bit for bit.
    x = ((0xC0FFEE01 ^ 0x5A3C96E1) << 7 | (0xC0FFEE01 ^ 0x5A3C96E1) >> 25) & 0xFFFFFFFF
    assert compute_key(0xC0FFEE01) == (x + 0x1234ABCD) & 0xFFFFFFFF


def test_parse_dtc_records():
    assert parse_dtc_records(bytes([0xA1, 0x01, 0x01, 0x2F, 0xA1, 0x02, 0x01, 0x2E])) == [
        (0xA10101, 0x2F), (0xA10201, 0x2E)]
    assert parse_dtc_records(b"") == []


# ---- End to end: Python client <-> the firmware's C UDS server (libdiag.so) ----

ROOT = pathlib.Path(__file__).resolve().parent.parent
TEST_DIR = ROOT / "firmware" / "stm32" / "test"


@pytest.fixture(scope="module")
def libdiag():
    try:
        subprocess.run(["make", "-s", "-C", str(TEST_DIR), "libdiag.so"], check=True, capture_output=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"cannot build libdiag.so: {exc}")
    lib = ctypes.CDLL(str(TEST_DIR / "libdiag.so"))
    lib.shim_rx.argtypes = [ctypes.c_uint32, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32,
                            ctypes.c_char_p, ctypes.c_int]
    return lib


@pytest.fixture
def uds(libdiag):
    libdiag.shim_init()
    inbox = collections.deque()
    t0 = time.monotonic()

    def send(can_id, data):
        out = ctypes.create_string_buffer(8 * 32)
        now = int((time.monotonic() - t0) * 1000) & 0xFFFFFFFF
        n = libdiag.shim_rx(can_id, bytes(data), len(data), now, out, 32)
        for i in range(n):
            inbox.append((0x7E8, out.raw[i * 8:(i + 1) * 8]))

    def recv(timeout):
        return inbox.popleft() if inbox else None

    client = UdsClient(IsoTp(send, recv, timeout=0.2))
    client.lib = libdiag
    return client


def test_e2e_read_dids(uds):
    assert uds.read_did(0xF195) == b"ENVMON-1.2.0"         # 15-byte response: multi-frame
    assert uds.read_did(0x0101) == bytes([0x03, 0xE8])
    assert decode_env(uds.read_did(0x0100)) == {"temp_c": 23.45, "rh_pct": 41.2, "pressure_hpa": 1013.25, "seq": 7}
    # Four DIDs in one request = 9 bytes: a multi-frame *request*
    r = uds.request(bytes([0x22, 0x01, 0x01, 0xF1, 0x86, 0x01, 0x01, 0xF1, 0x86]))
    assert r == bytes([0x62, 0x01, 0x01, 0x03, 0xE8, 0xF1, 0x86, 0x01, 0x01, 0x01, 0x03, 0xE8, 0xF1, 0x86, 0x01])


def test_e2e_security_and_write(uds):
    with pytest.raises(NegativeResponse) as e:
        uds.write_did(0x0101, bytes([0x01, 0xF4]))
    assert e.value.nrc == 0x7F                               # wrong session
    assert uds.session(0x03) == (50, 5000)
    with pytest.raises(NegativeResponse) as e:
        uds.write_did(0x0101, bytes([0x01, 0xF4]))
    assert e.value.nrc == 0x33                               # locked
    uds.unlock()
    uds.write_did(0x0101, bytes([0x01, 0xF4]))
    assert uds.lib.shim_period() == 500
    with pytest.raises(NegativeResponse) as e:
        uds.write_did(0x0101, bytes([0x00, 0x32]))
    assert e.value.nrc == 0x31                               # 50 ms out of range


def test_e2e_dtcs_and_reset(uds):
    assert uds.read_dtcs() == []
    uds.lib.shim_set_dtc(0, 1)                               # sensor failure
    uds.lib.shim_set_dtc(2, 1)                               # watchdog reset
    assert uds.read_dtcs() == [(0xA10101, 0x2F), (0xA10301, 0x2F)]
    uds.clear_dtcs()
    assert uds.read_dtcs() == []
    uds.ecu_reset()
    assert uds.lib.shim_reset_requested() == 1


def test_e2e_functional_tester_present(uds):
    uds.request(bytes([0x3E, 0x80]), functional=True, expect_response=False)
    with pytest.raises(TimeoutError):                         # suppressed: node stays silent
        uds.tp.receive_message(0.05)
