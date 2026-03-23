"""
protocol.py - Canonical Python mirror of src/protocol.h.

Single source of truth for wire-protocol and disc-geometry constants
used by fpga_sim.py and e2e_test.py.

Keep in sync with src/protocol.h via:  make check-proto
"""

import struct

# ---- Frame delimiters --------------------------------------------------------
SOF1           = 0xAA
SOF2           = 0x55

# ---- Packet type codes -------------------------------------------------------
TYPE_POINT     = 0x01   # PC -> FPGA: polar point     (LEN = POINT_LEN)
TYPE_END       = 0x03   # PC -> FPGA: end of sequence (LEN = 0)
TYPE_ACK       = 0x81   # FPGA -> PC: ACK, echoes incoming payload
TYPE_DEBUG     = 0xF0   # FPGA -> PC: UTF-8 debug / status string

# ---- Payload sizes -----------------------------------------------------------
POINT_LEN      = 8      # r_um (int32 LE) + theta_deg (float32 LE)

# ---- Transport ---------------------------------------------------------------
BAUD_RATE      = 115200

# ---- Physical disc geometry --------------------------------------------------
DISC_RADIUS_UM = 33000  # outer radius in um  (33 mm standard CD)
MAX_STEPS      = 8500   # stepper steps from home to outer edge


def crc8_xor(data: bytes) -> int:
    """XOR-fold CRC-8.  Mirrors crc8_xor() in src/framing.c."""
    c = 0
    for b in data:
        c ^= b
    return c


def build_frame(pkt_type: int, payload: bytes) -> bytes:
    """Build a complete framed packet.  Mirrors send_frame() in vitis/framing.c."""
    length = len(payload)
    crc = crc8_xor(bytes([pkt_type, length]) + payload)
    return bytes([SOF1, SOF2, pkt_type, length]) + payload + bytes([crc])


def r_um_to_steps(r_um: int) -> int:
    """Map physical radius (um) to stepper step count.  Mirrors vitis_workspace/systemControl/main.c."""
    target = int(r_um / DISC_RADIUS_UM * MAX_STEPS + 0.5)
    return max(0, min(target, MAX_STEPS))
