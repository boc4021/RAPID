"""
test_protocol_parser.py - Unit tests for ProtocolParser in src/gui.py.

Run:
    python -m pytest tests/test_protocol_parser.py -v
    # or
    python tests/test_protocol_parser.py

No GUI dependencies are exercised — only ProtocolParser is imported.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from gui import ProtocolParser


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_parser(*registrations):
    """Create a ProtocolParser with zero or more (type, handler) pairs."""
    p = ProtocolParser()
    for event_type, fn in registrations:
        p.register(event_type, fn)
    return p


# ---------------------------------------------------------------------------
# Registration and dispatch
# ---------------------------------------------------------------------------

def test_registered_handler_called_with_event_dict():
    calls = []
    p = make_parser(("ack", calls.append))
    p.feed_line('>> {"type":"ack","r_um":16500,"theta_deg":0.0}')
    assert len(calls) == 1
    assert calls[0]["r_um"] == 16500
    assert calls[0]["theta_deg"] == 0.0


def test_second_registration_overwrites_first():
    first, second = [], []
    p = ProtocolParser()
    p.register("ack", first.append)
    p.register("ack", second.append)   # overrides previous
    p.feed_line('>> {"type":"ack","r_um":0,"theta_deg":0}')
    assert first  == []
    assert len(second) == 1


def test_multiple_event_types_dispatched_independently():
    acks, errs = [], []
    p = make_parser(("ack", acks.append), ("crc_error", errs.append))
    p.feed_line('>> {"type":"ack","r_um":100,"theta_deg":45.0}')
    p.feed_line('>> {"type":"crc_error"}')
    assert len(acks) == 1
    assert len(errs) == 1


def test_unregistered_type_is_silently_ignored():
    calls = []
    p = make_parser(("ack", calls.append))
    p.feed_line('>> {"type":"done","points_sent":3,"acks_received":4}')
    assert calls == []   # "done" not registered — no error, no dispatch


def test_non_prefix_line_not_dispatched():
    calls = []
    p = make_parser(("ack", calls.append))
    p.feed_line("[FPGA] some debug message from the firmware")
    p.feed_line("Sending 3 polar points over COM25...")
    p.feed_line("")
    assert calls == []


def test_prefix_only_line_ignored():
    """'>> ' with no JSON payload must not raise."""
    p = make_parser(("ack", lambda e: None))
    p.feed_line(">> ")   # no JSON, must not raise


# ---------------------------------------------------------------------------
# JSON parsing robustness
# ---------------------------------------------------------------------------

def test_malformed_json_silently_ignored():
    calls = []
    p = make_parser(("ack", calls.append))
    p.feed_line(">> {not valid json at all}")
    assert calls == []   # must not raise


def test_valid_json_but_missing_type_key():
    calls = []
    p = make_parser(("ack", calls.append))
    p.feed_line('>> {"r_um":100,"theta_deg":0}')   # no "type" key
    assert calls == []


def test_type_is_null_not_dispatched():
    calls = []
    p = make_parser(("ack", calls.append))
    p.feed_line('>> {"type":null,"r_um":100}')
    assert calls == []


def test_empty_json_object_ignored():
    p = make_parser(("ack", lambda e: (_ for _ in ()).throw(AssertionError("should not call"))))
    p.feed_line('>> {}')   # valid JSON, no "type" key


def test_json_array_at_top_level_ignored():
    """A JSON array (not an object) must not crash."""
    p = ProtocolParser()
    p.feed_line('>> [1, 2, 3]')   # evt.get("type") would fail on a list


def test_handler_receives_full_event_dict():
    evts = []
    p = make_parser(("ack", evts.append))
    p.feed_line('>> {"type":"ack","r_um":33000,"theta_deg":180.5,"extra":"field"}')
    assert evts[0]["r_um"]      == 33000
    assert evts[0]["theta_deg"] == 180.5
    assert evts[0]["extra"]     == "field"


# ---------------------------------------------------------------------------
# State isolation between calls
# ---------------------------------------------------------------------------

def test_feed_line_multiple_times_accumulates(self=None):
    calls = []
    p = make_parser(("ack", calls.append))
    for r in [100, 200, 300]:
        p.feed_line(f'{{"type":"ack","r_um":{r},"theta_deg":0.0}}')
        # Note: no ">> " prefix — should all be ignored
    assert calls == []   # none had the ">> " prefix

    for r in [100, 200, 300]:
        p.feed_line(f'>> {{"type":"ack","r_um":{r},"theta_deg":0.0}}')
    assert [e["r_um"] for e in calls] == [100, 200, 300]


# ---------------------------------------------------------------------------
# Standalone runner (no pytest required)
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
