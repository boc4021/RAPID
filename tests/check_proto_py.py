"""
check_proto_py.py - Verify tests/protocol.py constants match src/protocol.h.

Called by: make check-proto
"""
import re
import sys

src = open("src/protocol.h").read()
py  = open("tests/protocol.py").read()

defs = dict(re.findall(r'#define\s+(\w+)\s+(0x[0-9A-Fa-f]+u?|\d+u?)', src))

pymap = {
    "SOF_BYTE_1": "SOF1",
    "SOF_BYTE_2": "SOF2",
    "TYPE_POINT": "TYPE_POINT",
    "TYPE_END":   "TYPE_END",
    "TYPE_ACK":   "TYPE_ACK",
    "TYPE_DEBUG": "TYPE_DEBUG",
    "POINT_LEN":  "POINT_LEN",
    "BAUD_RATE":  "BAUD_RATE",
    "MAX_STEPS":  "MAX_STEPS",
    "DISC_RADIUS_UM": "DISC_RADIUS_UM",
}

ok = True
for cname, pyname in pymap.items():
    cv = int(defs[cname].rstrip('u'), 0)
    m  = re.search(rf'{pyname}\s*=\s*(0x[0-9A-Fa-f]+|\d+)', py)
    pv = int(m.group(1), 0) if m else None
    if cv != pv:
        print(f"MISMATCH {cname}: src/protocol.h={cv}  tests/protocol.py ({pyname})={pv}")
        ok = False

sys.exit(0 if ok else 1)
