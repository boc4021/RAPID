"""
e2e_test.py - End-to-end test for the RAPID PC <-> FPGA protocol.

Launches fpga_sim.py (FPGA side) and RAPID.exe (PC side) as subprocesses,
connects them through a virtual COM port pair, and verifies that every point
in the GDS file is acknowledged correctly.

Setup (one-time):
  1. Install com0com: https://com0com.sourceforge.net/
     In the com0com Setup UI, create a pair — e.g. COM10 <-> COM11.
  2. pip install pyserial

Usage:
  python tests/e2e_test.py                              # defaults below
  python tests/e2e_test.py --sim-port COM10 --pc-port COM11
  python tests/e2e_test.py --gds tests/fixtures/e2e_small.gds

The env var RAPID_INIT_WAIT_MS is set to 0 automatically so RAPID.exe skips
its 32-second hardware init wait and sends packets immediately.
"""

import argparse
import os
import re
import subprocess
import sys
import threading
import time

# Regex matching a point ACK line printed by pcCommunication.c
ACK_RE = re.compile(r"\[ACK\]\s+r=-?\d+\s+um")
# Regex for the final summary line
DONE_RE = re.compile(r"Done\s+-\s+(\d+)\s+points sent,\s+(\d+)\s+ACKs received")

COORD_RE = re.compile(r"-?\d+\s*:\s*-?\d+")


def count_gds_points(gds_path: str) -> int:
    """Count coordinate pairs in a GDS file using the same logic as inputParser.c."""
    count   = 0
    reading = False
    with open(gds_path) as f:
        for line in f:
            if not reading and line.startswith("XY"):
                reading = True
                line = line[2:]          # strip leading "XY"
            if reading:
                if "ENDEL" in line:
                    break
                if COORD_RE.search(line):
                    count += 1
    return count


def drain(proc: subprocess.Popen, lines: list, prefix: str) -> None:
    """Read all stdout lines from proc, print with prefix, append to lines."""
    for raw in proc.stdout:
        line = raw.rstrip()
        lines.append(line)
        # fpga_sim.py already tags its own lines with [SIM]; RAPID.exe does not
        tagged = line if prefix == "[SIM]" else f"[RAPID] {line}"
        print(f"  {tagged}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="RAPID end-to-end test")
    parser.add_argument("--sim-port", default="COM10",
                        help="COM port for fpga_sim.py (FPGA side)")
    parser.add_argument("--pc-port",  default="COM11",
                        help="COM port for RAPID.exe  (PC side)")
    parser.add_argument("--gds",      default="tests/fixtures/e2e_small.gds",
                        help="GDS input file")
    parser.add_argument("--rapid",    default="build/RAPID.exe",
                        help="Path to RAPID.exe")
    args = parser.parse_args()

    # ---- Validate --------------------------------------------------------
    if not os.path.exists(args.rapid):
        print(f"ERROR: {args.rapid} not found — run 'make' first.", file=sys.stderr)
        return 1

    if not os.path.exists(args.gds):
        print(f"ERROR: GDS file not found: {args.gds}", file=sys.stderr)
        return 1

    expected = count_gds_points(args.gds)
    if expected == 0:
        print(f"ERROR: No coordinates found in {args.gds}", file=sys.stderr)
        return 1

    print("=" * 60)
    print(f"[E2E] GDS file   : {args.gds}  ({expected} points)")
    print(f"[E2E] Sim port   : {args.sim_port}  (FPGA side)")
    print(f"[E2E] PC port    : {args.pc_port}   (RAPID.exe side)")
    print(f"[E2E] RAPID_INIT_WAIT_MS=0  (hardware init wait bypassed)")
    print("=" * 60)

    # ---- Start FPGA simulator --------------------------------------------
    sim_cmd = [sys.executable, "tests/fpga_sim.py", args.sim_port]
    sim_proc = subprocess.Popen(
        sim_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    print(f"[E2E] fpga_sim.py started (PID {sim_proc.pid})", flush=True)
    time.sleep(1)      # give the simulator a moment to open the port

    # ---- Start RAPID.exe -------------------------------------------------
    env = os.environ.copy()
    env["RAPID_INIT_WAIT_MS"] = "0"

    rapid_cmd = [args.rapid, args.pc_port, args.gds]
    rapid_proc = subprocess.Popen(
        rapid_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
        env=env,
    )
    print(f"[E2E] RAPID.exe started (PID {rapid_proc.pid})", flush=True)

    # ---- Drain output from both processes in parallel --------------------
    sim_lines   = []
    rapid_lines = []

    sim_thread   = threading.Thread(target=drain, args=(sim_proc,   sim_lines,   "[SIM]"),   daemon=True)
    rapid_thread = threading.Thread(target=drain, args=(rapid_proc, rapid_lines, "[RAPID]"), daemon=True)
    sim_thread.start()
    rapid_thread.start()

    # ---- Wait for RAPID.exe to finish ------------------------------------
    # Timeout: per-point ACK (generous 10 s each) + small fixed overhead
    timeout_s = expected * 10 + 30
    try:
        rapid_proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        print(f"\n[E2E] TIMEOUT after {timeout_s}s — killing processes", file=sys.stderr)
        rapid_proc.kill()
        sim_proc.kill()
        return 1

    rapid_thread.join(timeout=5)

    # Clean up simulator
    sim_proc.terminate()
    sim_thread.join(timeout=5)

    # ---- Evaluate --------------------------------------------------------
    ack_count = sum(1 for l in rapid_lines if ACK_RE.search(l))
    exit_code = rapid_proc.returncode

    print()
    print("=" * 60)
    print(f"[E2E] Expected point ACKs : {expected}")
    print(f"[E2E] Received point ACKs : {ack_count}")
    print(f"[E2E] RAPID.exe exit code : {exit_code}")

    # Also check the final summary line for sanity
    for line in rapid_lines:
        m = DONE_RE.search(line)
        if m:
            pts_sent = int(m.group(1))
            acks_rcvd = int(m.group(2))
            print(f"[E2E] RAPID summary       : {pts_sent} sent, {acks_rcvd} ACKs")

    passed = (exit_code == 0 and ack_count == expected)
    print(f"\n[E2E] {'PASS' if passed else 'FAIL'}")
    print("=" * 60)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
