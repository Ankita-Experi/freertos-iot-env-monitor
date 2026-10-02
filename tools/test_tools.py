"""Tests for tools/ (no hardware or python-can needed): python3 -m pytest tools"""

import struct

from can_bittiming import solve
from can_monitor import decode_env, decode_status, encode_set_period


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
