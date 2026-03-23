"""
fpga_sink.py - Silent COM port sink for ACK timeout testing.

Opens the specified COM port and reads all incoming bytes without sending
any response.  Used by test_ack_timeout.py to verify that RAPID.exe exits
non-zero when no ACK arrives within ACK_TIMEOUT (2000 ms in src/main.c).

Usage:
    python tests/fpga_sink.py <COM_PORT>

Requirements:
    pip install pyserial
"""

import sys

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed.  Run: pip install pyserial", file=sys.stderr)
    sys.exit(1)

from protocol import BAUD_RATE


def run(port_name: str) -> int:
    try:
        port = serial.Serial(port_name, BAUD_RATE, timeout=60)
    except serial.SerialException as e:
        print(f"[SINK] ERROR: could not open {port_name}: {e}", file=sys.stderr)
        return 1

    print(f"[SINK] {port_name} open at {BAUD_RATE} baud — discarding input, never responding.",
          flush=True)
    try:
        while True:
            port.read(256)   # drain inbound bytes; send nothing
    except KeyboardInterrupt:
        print("[SINK] Interrupted.", flush=True)
    finally:
        port.close()

    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <COM_PORT>")
        sys.exit(1)
    sys.exit(run(sys.argv[1]))
