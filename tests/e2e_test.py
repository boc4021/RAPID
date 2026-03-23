"""
e2e_test.py - End-to-end test for the RAPID PC <-> FPGA protocol.

Launches fpga_sim.py (FPGA side) and RAPID.exe (PC side) as subprocesses,
connects them through a virtual COM port pair, and verifies that every point
in the GDS file is acknowledged correctly.

Setup (one-time):
  1. Install com0com: https://com0com.sourceforge.net/
     In the com0com Setup UI, create a pair — e.g. COM4 <-> COM6.
  2. pip install pyserial

Usage:
  python tests/e2e_test.py                              # defaults below
  python tests/e2e_test.py --sim-port COM4 --pc-port COM6
  python tests/e2e_test.py --gds tests/fixtures/e2e_small.gds

The env var RAPID_INIT_WAIT_MS is set to 0 automatically so RAPID.exe skips
its 32-second hardware init wait and sends packets immediately.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time

# Matches the structured JSON event lines emitted by src/main.c:
#   >> {"type":"ack", "r_um":..., "theta_deg":...}
#   >> {"type":"done", "points_sent":..., "acks_received":...}
JSON_LINE_PREFIX = ">> "

COORD_RE = re.compile(r"-?\d+\s*:\s*-?\d+")


def count_gds_points(gds_path: str) -> int:
    """Count coordinate pairs in a GDS file using the same logic as inputParser.c.

    Keep in sync with getCoordinates() in src/inputParser.c — both use
    sscanf(line, "%d : %d", ...) / COORD_RE to detect valid coordinate lines,
    and both break on "ENDEL" after the "XY" header."""
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


def evaluate_output(rapid_lines: list, expected: int) -> tuple[int, dict | None]:
    """Parse structured JSON events from RAPID.exe output lines.

    Returns (ack_count, done_evt) where done_evt is the final summary dict or
    None if no "done" event was found.
    """
    ack_count = 0
    done_evt  = None
    for line in rapid_lines:
        if not line.startswith(JSON_LINE_PREFIX):
            continue
        try:
            evt = json.loads(line[len(JSON_LINE_PREFIX):])
        except json.JSONDecodeError as exc:
            # T-QUA-03: log malformed JSON rather than swallowing it silently
            print(f"[E2E] WARNING: malformed JSON event: {line!r} ({exc})", flush=True)
            continue
        if evt.get("type") == "ack":
            ack_count += 1
        elif evt.get("type") == "done":
            done_evt = evt
    return ack_count, done_evt


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
    parser.add_argument("--sim-port", default="COM4",
                        help="COM port for fpga_sim.py (FPGA side)")
    parser.add_argument("--pc-port",  default="COM6",
                        help="COM port for RAPID.exe  (PC side)")
    parser.add_argument("--gds",      default="tests/fixtures/e2e_small.gds",
                        help="GDS input file")
    parser.add_argument("--rapid",    default="build/RAPID.exe",
                        help="Path to RAPID.exe")
    args = parser.parse_args()

    # ---- Validate --------------------------------------------------------
    args.rapid = os.path.abspath(args.rapid)
    args.gds   = os.path.abspath(args.gds)

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
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sim_cmd = [sys.executable, os.path.join(script_dir, "fpga_sim.py"), args.sim_port]
    sim_proc = subprocess.Popen(
        sim_cmd,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    print(f"[E2E] fpga_sim.py started (PID {sim_proc.pid})", flush=True)

    # T-QUA-01: wait for the simulator's "Ready." line instead of sleeping.
    # fpga_sim.py prints "[SIM] Ready." once the port is open and accepting.
    sim_lines = []
    _sim_deadline = time.time() + 15.0
    for raw in sim_proc.stdout:
        line = raw.rstrip()
        sim_lines.append(line)
        print(f"  [SIM] {line}", flush=True)
        if "Ready." in line:
            break
        if time.time() > _sim_deadline:
            print("[E2E] ERROR: simulator never printed 'Ready.' within 15 s",
                  file=sys.stderr)
            sim_proc.kill()
            return 1

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
    # sim_lines already pre-populated with startup lines read above.
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
    # Parse structured JSON events from the >> lines (the machine-readable contract).
    ack_count, done_evt = evaluate_output(rapid_lines, expected)
    exit_code = rapid_proc.returncode

    # T-QUA-02: use done_evt["points_sent"] as the authoritative point count
    # (it comes directly from inputParser as used by RAPID.exe, removing the
    # risk of count_gds_points() drifting from the C parser).
    expected_from_exe = done_evt["points_sent"] if done_evt else None

    if expected_from_exe is not None and expected_from_exe != expected:
        print(f"[E2E] WARNING: count_gds_points={expected} but "
              f"done.points_sent={expected_from_exe} — GDS parser may have drifted",
              flush=True)

    print()
    print("=" * 60)
    print(f"[E2E] Expected point ACKs : {expected}")
    print(f"[E2E] Received point ACKs : {ack_count}")
    print(f"[E2E] RAPID.exe exit code : {exit_code}")
    if done_evt:
        print(f"[E2E] RAPID summary       : {done_evt['points_sent']} sent, "
              f"{done_evt['acks_received']} ACKs")

    # T-COV-04: validate done_evt field values, not just presence.
    # acks_received == points_sent + 1 because the end-packet ACK is also counted.
    passed = (
        exit_code == 0
        and done_evt is not None
        and ack_count == done_evt.get("points_sent", -1)
        and done_evt.get("acks_received", -1) == done_evt.get("points_sent", -1) + 1
    )
    print(f"\n[E2E] {'PASS' if passed else 'FAIL'}")
    print("=" * 60)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
