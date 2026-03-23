"""
fpga_sim.py - Software FPGA simulator for RAPID end-to-end testing.

Simulates vitis_workspace/systemControl/main.c: opens a serial COM port and acts as the FPGA,
parsing framed polar-coordinate packets and sending ACK responses.

No real motors, laser, GPIO, or I2C hardware is required.  The simulator
logs what the physical hardware *would* do at each step.

Wire protocol (shared with src/main.c / vitis_workspace/systemControl/main.c):
  SOF (0xAA 0x55) | TYPE (1 B) | LEN (1 B) | PAYLOAD (LEN B) | CRC8

  TYPE_POINT 0x01, LEN=8: r_um (int32 LE) + theta_deg (float32 LE)
  TYPE_END   0x03, LEN=0: end of sequence
  TYPE_ACK   0x81:        echo of incoming payload

Usage:
    python tests/fpga_sim.py <COM_PORT>

Requirements:
    pip install pyserial
    A virtual COM port pair connected to the port used by RAPID.exe.
    On Windows: com0com (https://com0com.sourceforge.net/) creates pairs.
    Example: com0com creates COM4 <-> COM6
      Simulator: python tests/fpga_sim.py COM4
      RAPID.exe: RAPID_INIT_WAIT_MS=0 build/RAPID.exe COM6 tests/fixtures/e2e_small.gds
"""

import sys
import struct
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed.  Run: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# ---- Protocol & motor constants -------------------------------------------
# Canonical source of truth: src/protocol.h  (mirrored in tests/protocol.py)
from protocol import (
    SOF1, SOF2, TYPE_POINT, TYPE_END, TYPE_ACK,
    POINT_LEN, BAUD_RATE, MAX_STEPS, DISC_RADIUS_UM,
    crc8_xor, r_um_to_steps,
)


def _read_exact(port: "serial.Serial", n: int) -> bytes:
    """Read exactly n bytes; raise TimeoutError if fewer arrive."""
    data = port.read(n)
    if len(data) < n:
        raise TimeoutError(f"Short read: expected {n} bytes, got {len(data)}")
    return data


def send_frame(port: "serial.Serial", pkt_type: int, payload: bytes) -> None:
    crc   = crc8_xor(bytes([pkt_type, len(payload)]) + payload)
    frame = bytes([SOF1, SOF2, pkt_type, len(payload)]) + payload + bytes([crc])
    port.write(frame)


def receive_packet(port: "serial.Serial") -> tuple[int, bytes]:
    """Block until one valid framed packet arrives.  Returns (type, payload)."""
    while True:
        b = _read_exact(port, 1)
        if b[0] != SOF1:
            continue

        b = _read_exact(port, 1)
        if b[0] != SOF2:
            continue

        pkt_type = _read_exact(port, 1)[0]
        length   = _read_exact(port, 1)[0]
        payload  = _read_exact(port, length) if length else b""
        rx_crc   = _read_exact(port, 1)[0]

        expected_crc = crc8_xor(bytes([pkt_type, length]) + payload)
        if expected_crc != rx_crc:
            print(f"[SIM] CRC mismatch: expected 0x{expected_crc:02X}, got 0x{rx_crc:02X}",
                  flush=True)
            continue

        return pkt_type, payload


def run(port_name: str) -> int:
    """Open port, simulate FPGA behaviour.  Returns 0 on success."""
    print(f"[SIM] Opening {port_name} at {BAUD_RATE} baud...", flush=True)
    try:
        port = serial.Serial(port_name, BAUD_RATE, timeout=120)
    except serial.SerialException as e:
        print(f"[SIM] ERROR: could not open {port_name}: {e}", file=sys.stderr)
        return 1

    print("[SIM] Ready. Simulating FPGA (main.c) behaviour.", flush=True)
    print(f"[SIM] Disc radius: {DISC_RADIUS_UM} um, max steps: {MAX_STEPS}", flush=True)

    # Simulated hardware state
    current_step  = 0
    laser_on      = False
    point_count   = 0
    start_time    = time.time()

    try:
        while True:
            pkt_type, payload = receive_packet(port)

            if pkt_type == TYPE_POINT:
                if len(payload) < POINT_LEN:
                    print(f"[SIM] POINT payload too short: {len(payload)} bytes — ignored",
                          flush=True)
                    continue

                r_um,      = struct.unpack_from("<i", payload, 0)
                theta_deg, = struct.unpack_from("<f", payload, 4)

                target_step = r_um_to_steps(r_um)
                # T-PAT-03: assert simulator invariants so violations fail the test
                assert 0 <= target_step <= MAX_STEPS, (
                    f"[SIM] ASSERT: target_step {target_step} out of [0, {MAX_STEPS}] "
                    f"(r_um={r_um})")
                assert r_um >= 0, f"[SIM] ASSERT: r_um={r_um} is negative"
                delta       = abs(target_step - current_step)
                direction   = "outward" if target_step >= current_step else "inward"

                if delta > 0:
                    # Note: the real FPGA sleeps for the move duration (usleep);
                    # the simulator only logs it.  This is intentional — the
                    # simulator runs without real-time delays so E2E tests
                    # complete in seconds rather than minutes.
                    move_ms = delta * 2.0          # 500 Hz step rate => 2 ms/step
                    print(f"[SIM] POINT {point_count+1}: r={r_um} um, theta={theta_deg:.2f} deg "
                          f"-> step {current_step} -> {target_step} "
                          f"({delta} steps {direction}, ~{move_ms:.0f} ms simulated)",
                          flush=True)
                    current_step = target_step
                else:
                    print(f"[SIM] POINT {point_count+1}: r={r_um} um, theta={theta_deg:.2f} deg "
                          f"-> already at step {current_step} (no move)", flush=True)

                # Turn laser on after first move (mirrors main.c first_point flag)
                if not laser_on:
                    laser_on = True
                    print("[SIM] Laser ON", flush=True)

                point_count += 1

                # ACK: echo payload back to PC
                send_frame(port, TYPE_ACK, payload)
                print(f"[SIM] ACK #{point_count} sent", flush=True)

            elif pkt_type == TYPE_END:
                # T-PAT-03: if points were received, laser must have been turned on
                if point_count > 0:
                    assert laser_on, (
                        "[SIM] ASSERT: TYPE_END received after points but laser was never ON")
                if laser_on:
                    laser_on = False
                    print("[SIM] Laser OFF", flush=True)

                elapsed = time.time() - start_time
                print(f"[SIM] Pattern complete: {point_count} point(s) processed "
                      f"in {elapsed:.1f}s", flush=True)
                print(f"[SIM] Final stepper position: step {current_step} "
                      f"({current_step / MAX_STEPS * DISC_RADIUS_UM:.0f} um)", flush=True)

                # ACK end packet with empty payload
                send_frame(port, TYPE_ACK, b"")
                print("[SIM] End ACK sent. Shutting down.", flush=True)
                break

            else:
                print(f"[SIM] Unknown packet type: 0x{pkt_type:02X} — ignored", flush=True)

    except TimeoutError as e:
        print(f"[SIM] ERROR: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("[SIM] Interrupted.", flush=True)
    finally:
        port.close()

    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <COM_PORT>")
        print("Example: python tests/fpga_sim.py COM4")
        sys.exit(1)

    sys.exit(run(sys.argv[1]))
