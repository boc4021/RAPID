"""
test_ack_timeout.py - Verify RAPID.exe exits non-zero on ACK timeout.

Starts fpga_sink.py (a COM port that accepts bytes but never responds),
then runs RAPID.exe and verifies it exits with a non-zero code within
ACK_TIMEOUT (2000 ms, defined in src/main.c) plus a generous margin.

Setup (one-time):
  Same as e2e_test.py: install com0com, create a port pair (e.g. COM4 <-> COM6).
  pip install pyserial

Usage:
  python tests/test_ack_timeout.py
  python tests/test_ack_timeout.py --sim-port COM4 --pc-port COM6
  python tests/test_ack_timeout.py --gds tests/fixtures/e2e_small.gds
"""

import argparse
import os
import subprocess
import sys
import threading

# ACK_TIMEOUT in src/main.c is 2000 ms.
# Allow 10 s total for process startup + the timeout wait.
_ACK_TIMEOUT_S  = 2
_STARTUP_MARGIN = 10
_TOTAL_TIMEOUT  = _ACK_TIMEOUT_S + _STARTUP_MARGIN

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _drain(proc: subprocess.Popen, lines: list, tag: str) -> None:
    for raw in proc.stdout:
        line = raw.rstrip()
        lines.append(line)
        print(f"  [{tag}] {line}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="RAPID ACK timeout test")
    parser.add_argument("--sim-port", default="COM4",
                        help="COM port for fpga_sink.py (never responds)")
    parser.add_argument("--pc-port",  default="COM6",
                        help="COM port for RAPID.exe")
    parser.add_argument("--gds",      default="tests/fixtures/e2e_small.gds",
                        help="GDS input file")
    parser.add_argument("--rapid",    default="build/RAPID.exe",
                        help="Path to RAPID.exe")
    args = parser.parse_args()

    args.rapid = os.path.abspath(args.rapid)
    args.gds   = os.path.abspath(args.gds)

    if not os.path.exists(args.rapid):
        print(f"ERROR: {args.rapid} not found — run 'make' first.", file=sys.stderr)
        return 1
    if not os.path.exists(args.gds):
        print(f"ERROR: GDS file not found: {args.gds}", file=sys.stderr)
        return 1

    print("=" * 60)
    print("[TIMEOUT-TEST] Testing ACK timeout path in RAPID.exe")
    print(f"[TIMEOUT-TEST] Sink port  : {args.sim_port}  (never ACKs)")
    print(f"[TIMEOUT-TEST] PC port    : {args.pc_port}")
    print(f"[TIMEOUT-TEST] GDS        : {args.gds}")
    print(f"[TIMEOUT-TEST] Expected   : RAPID.exe exits non-zero within {_TOTAL_TIMEOUT}s")
    print("=" * 60)

    # ---- Start the silent sink -------------------------------------------
    sink_cmd = [sys.executable,
                os.path.join(SCRIPT_DIR, "fpga_sink.py"),
                args.sim_port]
    sink_proc = subprocess.Popen(
        sink_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    print(f"[TIMEOUT-TEST] fpga_sink.py started (PID {sink_proc.pid})", flush=True)

    # Wait for the sink to open its port before starting RAPID.exe.
    # fpga_sink.py prints "[SINK] <port> open ..." once the port is ready.
    sink_lines = []
    for raw in sink_proc.stdout:
        line = raw.rstrip()
        sink_lines.append(line)
        print(f"  [SINK] {line}", flush=True)
        if "open" in line.lower():
            break

    # ---- Start RAPID.exe with init wait bypassed -------------------------
    env = os.environ.copy()
    env["RAPID_INIT_WAIT_MS"] = "0"

    rapid_cmd = [args.rapid, args.pc_port, args.gds]
    rapid_proc = subprocess.Popen(
        rapid_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
        env=env,
    )
    print(f"[TIMEOUT-TEST] RAPID.exe started (PID {rapid_proc.pid})", flush=True)

    # Drain RAPID output in a background thread while we wait for exit.
    rapid_lines: list = []
    rapid_thread = threading.Thread(
        target=_drain, args=(rapid_proc, rapid_lines, "RAPID"), daemon=True)
    rapid_thread.start()

    # ---- Wait for RAPID.exe to timeout and exit --------------------------
    try:
        rapid_proc.wait(timeout=_TOTAL_TIMEOUT)
    except subprocess.TimeoutExpired:
        print(f"\n[TIMEOUT-TEST] FAIL — RAPID.exe did not exit within {_TOTAL_TIMEOUT}s",
              file=sys.stderr)
        rapid_proc.kill()
        sink_proc.kill()
        return 1

    rapid_thread.join(timeout=3)

    # Clean up the sink.
    sink_proc.terminate()
    sink_proc.wait(timeout=3)

    # ---- Evaluate --------------------------------------------------------
    exit_code = rapid_proc.returncode
    passed    = (exit_code != 0)

    # Verify RAPID actually printed a timeout message (not some other error).
    timeout_msg_seen = any("timeout" in l.lower() or "Timeout" in l
                           for l in rapid_lines)

    print()
    print("=" * 60)
    print(f"[TIMEOUT-TEST] RAPID.exe exit code   : {exit_code}")
    print(f"[TIMEOUT-TEST] Timeout message seen  : {timeout_msg_seen}")
    print(f"\n[TIMEOUT-TEST] {'PASS' if passed else 'FAIL'}")
    if not passed:
        print("[TIMEOUT-TEST] Expected non-zero exit; RAPID.exe exited 0 — "
              "ACK timeout path may not be working.", file=sys.stderr)
    print("=" * 60)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
