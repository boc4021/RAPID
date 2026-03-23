"""
test_protocol.py - Unit tests for helpers in tests/protocol.py.

Covers: crc8_xor, build_frame, r_um_to_steps (boundary values, clamping,
rounding, mid-range accuracy).

Run:
    python -m pytest tests/test_protocol.py -v
    # or
    python tests/test_protocol.py
"""

import sys
import os
import struct

sys.path.insert(0, os.path.dirname(__file__))

from protocol import (
    crc8_xor, build_frame, r_um_to_steps,
    SOF1, SOF2, TYPE_POINT, TYPE_END, TYPE_ACK,
    POINT_LEN, BAUD_RATE, MAX_STEPS, DISC_RADIUS_UM,
)


# ---------------------------------------------------------------------------
# crc8_xor
# ---------------------------------------------------------------------------

def test_crc_empty():
    assert crc8_xor(b"") == 0


def test_crc_single_byte():
    assert crc8_xor(bytes([0x42])) == 0x42


def test_crc_known_vector():
    # TYPE_POINT(0x01) ^ LEN(0x08) ^ 8 zero bytes == 0x09
    data = bytes([0x01, 0x08]) + bytes(8)
    assert crc8_xor(data) == 0x09


def test_crc_self_cancels():
    # XOR the same byte twice → 0
    assert crc8_xor(bytes([0xAB, 0xAB])) == 0


def test_crc_appended_sum_is_zero():
    data = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    crc  = crc8_xor(data)
    assert crc8_xor(data + bytes([crc])) == 0


# ---------------------------------------------------------------------------
# build_frame
# ---------------------------------------------------------------------------

def test_build_frame_sof_bytes():
    frame = build_frame(TYPE_END, b"")
    assert frame[0] == SOF1
    assert frame[1] == SOF2


def test_build_frame_type_and_len():
    payload = b"\x01\x02\x03"
    frame = build_frame(TYPE_POINT, payload)
    assert frame[2] == TYPE_POINT
    assert frame[3] == len(payload)


def test_build_frame_payload_embedded():
    payload = bytes([0xAA, 0x55, 0xFF])
    frame = build_frame(TYPE_ACK, payload)
    assert frame[4:7] == payload


def test_build_frame_crc_correct():
    payload = bytes([0x11, 0x22])
    frame = build_frame(TYPE_ACK, payload)
    # CRC covers [type, len, payload...]
    data_for_crc = bytes([frame[2], frame[3]]) + payload
    expected_crc = crc8_xor(data_for_crc)
    assert frame[-1] == expected_crc


def test_build_frame_total_length():
    payload = bytes(POINT_LEN)
    frame = build_frame(TYPE_POINT, payload)
    # SOF(2) + TYPE(1) + LEN(1) + PAYLOAD(POINT_LEN) + CRC(1)
    assert len(frame) == 2 + 1 + 1 + POINT_LEN + 1


def test_build_frame_empty_payload():
    frame = build_frame(TYPE_END, b"")
    assert len(frame) == 5            # SOF(2) + TYPE + LEN=0 + CRC
    assert frame[3] == 0              # LEN = 0
    # CRC = TYPE ^ 0x00 = TYPE_END
    assert frame[4] == TYPE_END


# ---------------------------------------------------------------------------
# r_um_to_steps
# ---------------------------------------------------------------------------

def test_steps_at_origin():
    assert r_um_to_steps(0) == 0


def test_steps_at_full_radius():
    assert r_um_to_steps(DISC_RADIUS_UM) == MAX_STEPS


def test_steps_at_midpoint():
    # 16500 µm = exactly half of 33000 → half of 8500 = 4250
    assert r_um_to_steps(16500) == 4250


def test_steps_clamp_negative():
    assert r_um_to_steps(-1)      == 0
    assert r_um_to_steps(-100000) == 0


def test_steps_clamp_over_range():
    assert r_um_to_steps(DISC_RADIUS_UM + 1)      == MAX_STEPS
    assert r_um_to_steps(DISC_RADIUS_UM + 100000)  == MAX_STEPS


def test_steps_rounding_down():
    # 1 µm → 1/33000 * 8500 ≈ 0.2576 → rounds to 0
    assert r_um_to_steps(1) == 0


def test_steps_rounding_up():
    # Find the smallest r that rounds to 1:
    #   r/33000 * 8500 >= 0.5 → r >= 33000*0.5/8500 ≈ 1.94 → r = 2
    assert r_um_to_steps(2) == 1


def test_steps_quarter_radius():
    # 8250 µm = ¼ of 33000 → ¼ of 8500 = 2125
    assert r_um_to_steps(8250) == 2125


def test_steps_result_always_in_range():
    for r in range(0, DISC_RADIUS_UM + 1, 1000):
        s = r_um_to_steps(r)
        assert 0 <= s <= MAX_STEPS, f"r_um_to_steps({r}) = {s} out of [0, {MAX_STEPS}]"


def test_steps_monotone():
    """Larger radius must always produce the same or greater step count."""
    prev = r_um_to_steps(0)
    for r in range(0, DISC_RADIUS_UM + 1, 100):
        s = r_um_to_steps(r)
        assert s >= prev, f"Non-monotone at r={r}: {s} < {prev}"
        prev = s


# ---------------------------------------------------------------------------
# Protocol constants sanity
# ---------------------------------------------------------------------------

def test_constants_non_zero():
    assert SOF1           == 0xAA
    assert SOF2           == 0x55
    assert TYPE_POINT     == 0x01
    assert TYPE_END       == 0x03
    assert TYPE_ACK       == 0x81
    assert POINT_LEN      == 8
    assert BAUD_RATE      == 115200
    assert DISC_RADIUS_UM == 33000
    assert MAX_STEPS      == 8500


# ---------------------------------------------------------------------------
# Standalone runner
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import traceback

    tests = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v)]
    passed = failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except Exception as exc:
            print(f"  FAIL  {fn.__name__}: {exc}")
            traceback.print_exc()
            failed += 1

    print(f"\n{passed} / {passed + failed} tests passed.")
    sys.exit(0 if failed == 0 else 1)
