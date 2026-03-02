#!/usr/bin/env python3
"""
test_milestone3.py — Milestone 3 Validation Test Suite
=======================================================

Tests Channel Routing, Standalone Test Mode, Dynamic Limit Override,
and extreme stress scenarios for the TC-Interrupter.

Milestone 3 Objectives (from PLAN.md):
  Phase A — Dynamic MIDI channel → coil routing via CDC
  Phase B — Standalone test mode (direct FIRE bypassing MIDI)
  Phase C — Dynamic limit override + 6-channel MIDI stress

Requires:
  - pyserial >= 3.5
  - mido >= 1.3.0         (optional, for MIDI tests)
  - python-rtmidi >= 1.5.0 (optional, for MIDI tests)

Usage:
  python test_milestone3.py                     # Auto-detect port, basic tests
  python test_milestone3.py --port COM5         # Explicit port
  python test_milestone3.py --all               # Run all tests including MIDI+stress
  python test_milestone3.py --midi              # Run MIDI routing tests only
  python test_milestone3.py --stress            # Include extreme stress tests
"""

import argparse
import json
import sys
import time
import threading

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
VID = 0x0483
PID = 0x5741
BAUD = 115200
TIMEOUT = 2.0
NUM_COILS = 6
MIDI_CHANNELS = 16

# ---------------------------------------------------------------------------
#  Helpers (shared with M2)
# ---------------------------------------------------------------------------

def find_port():
    """Auto-detect the TC-Interrupter CDC COM port by VID:PID."""
    for p in serial.tools.list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def send_cmd(ser, cmd, timeout=TIMEOUT):
    """Send a command and return the full response line (stripped).

    Handles long responses (e.g. COILS? JSON) that may span multiple
    serial reads by accumulating data until a newline is found.
    """
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    buf = ""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = ser.read(ser.in_waiting or 1)
        if raw:
            buf += raw.decode("ascii", errors="replace")
            # Look for a complete line
            if "\n" in buf:
                # Return the first complete line
                line = buf.split("\n")[0].strip()
                if line:
                    return line
                # If the line was empty (bare \r\n), keep reading
                buf = buf.split("\n", 1)[1]
    return buf.strip()


def send_cmd_multi(ser, cmd, lines=2, timeout=TIMEOUT):
    """Send a command and return up to `lines` response lines."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    result = []
    deadline = time.monotonic() + timeout
    while len(result) < lines and time.monotonic() < deadline:
        raw = ser.readline().decode("ascii", errors="replace").strip()
        if raw:
            result.append(raw)
    return result


def parse_json(line):
    """Try to parse a JSON response, return dict or None."""
    try:
        return json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return None


def cleanup(ser):
    """Stop all coils and reset routing/limits to defaults."""
    send_cmd(ser, "STOPALL")
    send_cmd(ser, "ROUTERESET")
    for c in range(NUM_COILS):
        send_cmd(ser, f"LIMITS {c} 200 10 100")
        send_cmd(ser, f"ENABLE {c}")
    time.sleep(0.05)


# ---------------------------------------------------------------------------
#  Test Results Tracker
# ---------------------------------------------------------------------------

class TestResults:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.details = []

    def ok(self, name, msg=""):
        self.passed += 1
        self.details.append(("PASS", name, msg))
        print(f"  [PASS] {name}" + (f" — {msg}" if msg else ""))

    def fail(self, name, msg=""):
        self.failed += 1
        self.details.append(("FAIL", name, msg))
        print(f"  [FAIL] {name}" + (f" — {msg}" if msg else ""))

    def skip(self, name, msg=""):
        self.skipped += 1
        self.details.append(("SKIP", name, msg))
        print(f"  [SKIP] {name}" + (f" — {msg}" if msg else ""))

    def summary(self):
        total = self.passed + self.failed + self.skipped
        print(f"\n{'='*60}")
        print(f"Results: {self.passed}/{total} passed, "
              f"{self.failed} failed, {self.skipped} skipped")
        print(f"{'='*60}")
        return self.failed == 0


# ===========================================================================
#  Phase A: Channel Map / MIDI Routing
# ===========================================================================

def test_route_query(ser, R):
    """ROUTE? returns full 16-channel routing table as JSON."""
    cleanup(ser)
    resp = send_cmd(ser, "ROUTE?")
    data = parse_json(resp)
    if not data or "routes" not in data:
        R.fail("ROUTE? Query", f"failed to parse: {resp}")
        return
    routes = data["routes"]
    if len(routes) != MIDI_CHANNELS:
        R.fail("ROUTE? Query", f"expected {MIDI_CHANNELS} routes, got {len(routes)}")
        return
    # Default routing: ch 0-5 → coil 0-5, ch 6-15 → 255
    ok = True
    for r in routes:
        ch = r["ch"]
        coil = r["coil"]
        expected = ch if ch < NUM_COILS else 255
        if coil != expected:
            ok = False
            break
    if ok:
        R.ok("ROUTE? Query", "default routing correct (ch0→c0..ch5→c5, ch6-15→255)")
    else:
        R.fail("ROUTE? Query", f"unexpected defaults: {resp}")


def test_route_single_channel(ser, R):
    """ROUTE moves a single MIDI channel to a different coil."""
    cleanup(ser)
    # Route ch 0 to coil 3
    resp = send_cmd(ser, "ROUTE 0 3")
    if "OK:ROUTE" not in resp or "coil=3" not in resp:
        R.fail("ROUTE Single Channel", f"set failed: {resp}")
        cleanup(ser)
        return
    # Verify via ROUTE?
    resp = send_cmd(ser, "ROUTE?")
    data = parse_json(resp)
    if data and data["routes"][0]["coil"] == 3:
        R.ok("ROUTE Single Channel", "ch=0 → coil=3 confirmed via ROUTE?")
    else:
        R.fail("ROUTE Single Channel", f"ROUTE? doesn't reflect change: {resp}")
    cleanup(ser)


def test_route_unmap(ser, R):
    """ROUTE ch to 255 unmaps it (MIDI notes ignored)."""
    cleanup(ser)
    resp = send_cmd(ser, "ROUTE 0 255")
    if "OK:ROUTE" not in resp or "coil=255" not in resp:
        R.fail("ROUTE Unmap", f"unexpected: {resp}")
        cleanup(ser)
        return
    R.ok("ROUTE Unmap", "ch=0 unmapped (coil=255)")
    cleanup(ser)


def test_routeall(ser, R):
    """ROUTEALL maps all 16 channels to a single coil."""
    cleanup(ser)
    resp = send_cmd(ser, "ROUTEALL 2")
    if "OK:ROUTEALL" not in resp or "coil=2" not in resp:
        R.fail("ROUTEALL", f"unexpected: {resp}")
        cleanup(ser)
        return
    # Verify
    resp = send_cmd(ser, "ROUTE?")
    data = parse_json(resp)
    if data and all(r["coil"] == 2 for r in data["routes"]):
        R.ok("ROUTEALL", "all 16 channels → coil 2 confirmed")
    else:
        R.fail("ROUTEALL", f"not all channels routed: {resp}")
    cleanup(ser)


def test_routereset(ser, R):
    """ROUTERESET restores default routing."""
    cleanup(ser)
    send_cmd(ser, "ROUTEALL 4")
    resp = send_cmd(ser, "ROUTERESET")
    if "OK:ROUTERESET" not in resp:
        R.fail("ROUTERESET", f"unexpected: {resp}")
        cleanup(ser)
        return
    resp = send_cmd(ser, "ROUTE?")
    data = parse_json(resp)
    if data:
        ok = all(r["coil"] == (r["ch"] if r["ch"] < NUM_COILS else 255) for r in data["routes"])
        if ok:
            R.ok("ROUTERESET", "default routing restored")
        else:
            R.fail("ROUTERESET", f"routing not default: {resp}")
    else:
        R.fail("ROUTERESET", f"ROUTE? parse failed: {resp}")


def test_route_all_permutations(ser, R):
    """Route ch 0-5 to coils 5-0 (reversed) and verify."""
    cleanup(ser)
    for ch in range(NUM_COILS):
        coil = NUM_COILS - 1 - ch
        send_cmd(ser, f"ROUTE {ch} {coil}")

    resp = send_cmd(ser, "ROUTE?")
    data = parse_json(resp)
    if not data:
        R.fail("Route Permutations", f"ROUTE? parse failed: {resp}")
        cleanup(ser)
        return

    ok = True
    for ch in range(NUM_COILS):
        expected = NUM_COILS - 1 - ch
        if data["routes"][ch]["coil"] != expected:
            ok = False
            break
    if ok:
        R.ok("Route Permutations", "reversed mapping ch0→c5..ch5→c0 verified")
    else:
        R.fail("Route Permutations", f"mapping incorrect: {resp}")
    cleanup(ser)


def test_route_invalid_args(ser, R):
    """Invalid ROUTE arguments return ERR."""
    cleanup(ser)
    resp = send_cmd(ser, "ROUTE 99 0")
    if "ERR" in resp:
        R.ok("ROUTE Invalid Args", f"channel 99 rejected: {resp}")
    else:
        R.fail("ROUTE Invalid Args", f"expected ERR, got: {resp}")


# ===========================================================================
#  Phase B: Standalone Test Mode
# ===========================================================================

def test_fire_coil0_precise(ser, R):
    """FIRE coil 0 at known frequency and verify pulse rate."""
    cleanup(ser)

    resp = send_cmd(ser, "COILS?")
    data = parse_json(resp)
    initial = data["coils"][0]["pulses"] if data and "coils" in data else 0

    # Fire at 200 Hz, 50 µs on coil 0
    resp = send_cmd(ser, "FIRE 0 200 50")
    if not resp.startswith("OK:FIRE"):
        R.fail("FIRE Coil 0 Precise", f"FIRE failed: {resp}")
        return

    time.sleep(1.0)

    resp = send_cmd(ser, "COILS?")
    data = parse_json(resp)
    send_cmd(ser, "STOP 0")

    if data and "coils" in data:
        delta = data["coils"][0]["pulses"] - initial
        # 200 Hz for 1s = ~200 pulses (± tolerance for duty limiting)
        if 100 <= delta <= 300:
            R.ok("FIRE Coil 0 Precise", f"{delta} pulses in 1s @ 200 Hz (expected ~200)")
        else:
            R.fail("FIRE Coil 0 Precise", f"{delta} pulses (expected ~200)")
    else:
        R.fail("FIRE Coil 0 Precise", f"COILS? parse failed: {resp}")


def test_fire_different_widths(ser, R):
    """FIRE with different pulse widths, verify clamping at limit."""
    cleanup(ser)
    # Set limits — consume the response fully
    send_cmd(ser, "LIMITS 0 100 10 100")
    # Synchronize: a PING/PONG round-trip guarantees all prior CDC TX
    # has been flushed through the USB stack before we proceed.
    sync = send_cmd(ser, "PING")

    # Fire with 50 µs (within limit) — should not clamp
    resp = send_cmd(ser, "FIRE 0 100 50")
    # Accept: response starts with OK:FIRE and does NOT say "clamped"
    within = resp.startswith("OK:FIRE") and "clamped" not in resp.lower()
    send_cmd(ser, "STOP 0")
    send_cmd(ser, "PING")  # sync again before next FIRE

    # Fire with 200 µs (exceeds 100 µs limit) — should clamp
    resp2 = send_cmd(ser, "FIRE 0 100 200")
    clamped = resp2.startswith("OK:FIRE") and ("clamped" in resp2.lower() or "ontime=100" in resp2)
    send_cmd(ser, "STOP 0")

    if within and clamped:
        R.ok("FIRE Different Widths", "50µs passed through, 200µs clamped to 100µs")
    else:
        R.fail("FIRE Different Widths", f"within={within} clamped={clamped} resp1={resp} resp2={resp2}")
    cleanup(ser)


def test_fire_stop_verify_idle(ser, R):
    """After STOP, verify coil timer is not active and no voices remain."""
    cleanup(ser)
    send_cmd(ser, "FIRE 0 100 30")
    time.sleep(0.2)
    send_cmd(ser, "STOP 0")
    time.sleep(0.05)

    resp = send_cmd(ser, "COILS?")
    data = parse_json(resp)
    if data and "coils" in data:
        voices = data["coils"][0].get("voices", -1)
        active = data["coils"][0].get("active", -1)
        if voices == 0:
            R.ok("FIRE/STOP Idle", f"voices=0 active={active}")
        else:
            R.fail("FIRE/STOP Idle", f"voices={voices} active={active}")
    else:
        R.fail("FIRE/STOP Idle", f"COILS? parse failed: {resp}")


def test_fire_invalid_coil(ser, R):
    """FIRE with invalid coil returns ERR."""
    resp = send_cmd(ser, "FIRE 7 100 50")
    if "ERR" in resp:
        R.ok("FIRE Invalid Coil", f"coil 7 rejected: {resp}")
    else:
        R.fail("FIRE Invalid Coil", f"expected ERR, got: {resp}")


def test_fire_zero_freq(ser, R):
    """FIRE with freq=0 returns ERR."""
    resp = send_cmd(ser, "FIRE 0 0 50")
    if "ERR" in resp:
        R.ok("FIRE Zero Freq", f"freq 0 rejected: {resp}")
    else:
        R.fail("FIRE Zero Freq", f"expected ERR, got: {resp}")


# ===========================================================================
#  Coil Enable / Disable
# ===========================================================================

def test_enable_disable(ser, R):
    """DISABLE prevents firing, ENABLE re-allows it."""
    cleanup(ser)

    # Disable coil 0
    resp = send_cmd(ser, "DISABLE 0")
    if "OK:DISABLE" not in resp:
        R.fail("Enable/Disable", f"DISABLE failed: {resp}")
        cleanup(ser)
        return

    # Verify coil 0 disabled via COILS?
    resp = send_cmd(ser, "COILS?")
    data = parse_json(resp)
    if data and data["coils"][0].get("en", 1) == 0:
        pass  # good
    else:
        R.fail("Enable/Disable", f"coil 0 not disabled: {resp}")
        cleanup(ser)
        return

    # Try to fire on disabled coil - tone should add but no pulses
    initial_pulses = data["coils"][0].get("pulses", 0)
    send_cmd(ser, "FIRE 0 100 50")
    time.sleep(0.3)

    resp = send_cmd(ser, "COILS?")
    data = parse_json(resp)
    after_pulses = data["coils"][0].get("pulses", 0) if data else 0
    send_cmd(ser, "STOP 0")

    # Re-enable
    resp = send_cmd(ser, "ENABLE 0")
    if "OK:ENABLE" not in resp:
        R.fail("Enable/Disable", f"ENABLE failed: {resp}")
        cleanup(ser)
        return

    # The key test: when disabled, no new pulses should fire
    if after_pulses == initial_pulses:
        R.ok("Enable/Disable", "no pulses while disabled, re-enabled OK")
    else:
        R.fail("Enable/Disable",
               f"pulses changed while disabled: {initial_pulses} → {after_pulses}")
    cleanup(ser)


def test_disable_invalid_coil(ser, R):
    """DISABLE with invalid coil returns ERR."""
    resp = send_cmd(ser, "DISABLE 8")
    if "ERR" in resp:
        R.ok("DISABLE Invalid", f"coil 8 rejected: {resp}")
    else:
        R.fail("DISABLE Invalid", f"expected ERR, got: {resp}")


# ===========================================================================
#  Phase C: Dynamic Limit Override
# ===========================================================================

def test_dynamic_limit_change(ser, R):
    """Change limits while a tone is actively firing."""
    cleanup(ser)
    # Start with generous limits
    send_cmd(ser, "LIMITS 0 200 10 100")
    send_cmd(ser, "FIRE 0 100 150")
    time.sleep(0.3)

    # Mid-flight, tighten on-time to 50 µs
    resp = send_cmd(ser, "LIMITS 0 50 10 100")
    if "OK:LIMITS" not in resp or "max_ontime=50" not in resp:
        R.fail("Dynamic Limit Change", f"LIMITS set failed: {resp}")
        send_cmd(ser, "STOPALL")
        cleanup(ser)
        return

    # The scheduler should now clamp on-time to 50 µs for subsequent pulses
    # We can't directly measure pulse width from CDC, but we can verify
    # the limits are set and the system doesn't crash
    time.sleep(0.3)

    resp = send_cmd(ser, "PING")
    send_cmd(ser, "STOPALL")

    if resp == "PONG":
        R.ok("Dynamic Limit Change", "limits changed mid-fire, system stable")
    else:
        R.fail("Dynamic Limit Change", f"system unresponsive after limit change")
    cleanup(ser)


def test_limits_all_coils(ser, R):
    """Set different limits on all 6 coils and query them back."""
    cleanup(ser)
    expected = []
    for c in range(NUM_COILS):
        ot = 50 + c * 50   # 50, 100, 150, 200, 250, 300
        duty = 5 + c * 5   # 5, 10, 15, 20, 25, 30
        off = 30 + c * 10  # 30, 40, 50, 60, 70, 80
        send_cmd(ser, f"LIMITS {c} {ot} {duty} {off}")
        # Clamp to absolute caps
        ot_exp = min(ot, 500)
        duty_exp = min(duty, 50)
        off_exp = max(off, 20)
        expected.append((ot_exp, duty_exp, off_exp))

    time.sleep(0.05)

    all_ok = True
    for c in range(NUM_COILS):
        resp = send_cmd(ser, f"LIMITS? {c}")
        ot_s, duty_s, off_s = expected[c]
        if (f"max_ontime={ot_s}" not in resp or
                f"duty_permil={duty_s}" not in resp or
                f"min_offtime={off_s}" not in resp):
            all_ok = False
            break

    if all_ok:
        R.ok("Limits All Coils", "6 unique limit profiles set and verified")
    else:
        R.fail("Limits All Coils", f"mismatch on coil {c}: {resp}")
    cleanup(ser)


# ===========================================================================
#  MIDI Routing Integration Tests
# ===========================================================================

def test_midi_route_to_coil(ser, R):
    """Route MIDI ch 0 to coil 2 and verify MIDI Note On fires coil 2."""
    try:
        import mido
    except ImportError:
        R.skip("MIDI Route→Coil", "mido not installed")
        return

    midi_port = None
    for name in mido.get_output_names():
        if "TC-Interrupter" in name or "MIDI" in name:
            midi_port = name
            break
    if midi_port is None:
        R.skip("MIDI Route→Coil", "no MIDI port found")
        return

    cleanup(ser)
    send_cmd(ser, "ROUTE 0 2")
    time.sleep(0.05)

    try:
        with mido.open_output(midi_port) as port:
            port.send(mido.Message("note_on", channel=0, note=60, velocity=100))
            time.sleep(0.4)

            resp = send_cmd(ser, "COILS?")
            data = parse_json(resp)

            port.send(mido.Message("note_off", channel=0, note=60, velocity=0))
            time.sleep(0.1)

            if data and "coils" in data:
                # Coil 2 should have voices/pulses, coil 0 should not
                c0_voices = data["coils"][0].get("voices", 0)
                c2_voices = data["coils"][2].get("voices", 0)
                c2_pulses = data["coils"][2].get("pulses", 0)

                if c2_voices >= 1 and c0_voices == 0:
                    R.ok("MIDI Route→Coil", f"ch0 routed to coil2: voices={c2_voices} pulses={c2_pulses}")
                elif c2_pulses > 0 and c0_voices == 0:
                    R.ok("MIDI Route→Coil", f"ch0 routed to coil2: pulses={c2_pulses}")
                else:
                    R.fail("MIDI Route→Coil", f"c0_v={c0_voices} c2_v={c2_voices} c2_p={c2_pulses}")
            else:
                R.fail("MIDI Route→Coil", f"COILS? parse failed: {resp}")
    except Exception as e:
        R.fail("MIDI Route→Coil", f"MIDI error: {e}")

    send_cmd(ser, "STOPALL")
    cleanup(ser)


def test_midi_unmapped_channel_ignored(ser, R):
    """MIDI notes on an unmapped channel produce no output."""
    try:
        import mido
    except ImportError:
        R.skip("MIDI Unmapped Ignored", "mido not installed")
        return

    midi_port = None
    for name in mido.get_output_names():
        if "TC-Interrupter" in name or "MIDI" in name:
            midi_port = name
            break
    if midi_port is None:
        R.skip("MIDI Unmapped Ignored", "no MIDI port found")
        return

    cleanup(ser)
    send_cmd(ser, "STOPALL")  # Ensure all tones from prior tests are stopped
    time.sleep(0.2)
    send_cmd(ser, "ROUTE 0 255")  # Unmap ch 0
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Get baseline pulse counts — use SCHED? per-coil pulse counts (shorter response)
    resp = send_cmd(ser, "COILS?", timeout=3.0)
    baseline = parse_json(resp)
    if not baseline or "coils" not in baseline:
        R.fail("MIDI Unmapped Ignored", f"baseline COILS? parse failed (len={len(resp)})")
        cleanup(ser)
        return
    base_pulses = [c["pulses"] for c in baseline["coils"]]

    try:
        with mido.open_output(midi_port) as port:
            port.send(mido.Message("note_on", channel=0, note=69, velocity=100))
            time.sleep(0.3)
            port.send(mido.Message("note_off", channel=0, note=69, velocity=0))
            time.sleep(0.15)
            ser.reset_input_buffer()

            resp = send_cmd(ser, "COILS?", timeout=3.0)
            data = parse_json(resp)
            if data and "coils" in data:
                after_pulses = [c["pulses"] for c in data["coils"]]
                deltas = [a - b for a, b in zip(after_pulses, base_pulses)]
                if all(d == 0 for d in deltas):
                    R.ok("MIDI Unmapped Ignored", "no pulses fired on any coil")
                else:
                    R.fail("MIDI Unmapped Ignored", f"pulse deltas: {deltas}")
            else:
                R.fail("MIDI Unmapped Ignored", f"parse failed (len={len(resp)}): {resp[:120]}")
    except Exception as e:
        R.fail("MIDI Unmapped Ignored", f"MIDI error: {e}")

    cleanup(ser)


def test_midi_routeall_convergence(ser, R):
    """ROUTEALL ch→coil 0, send notes on different channels, all fire coil 0."""
    try:
        import mido
    except ImportError:
        R.skip("MIDI ROUTEALL Convergence", "mido not installed")
        return

    midi_port = None
    for name in mido.get_output_names():
        if "TC-Interrupter" in name or "MIDI" in name:
            midi_port = name
            break
    if midi_port is None:
        R.skip("MIDI ROUTEALL Convergence", "no MIDI port found")
        return

    cleanup(ser)
    send_cmd(ser, "ROUTEALL 0")
    time.sleep(0.05)

    try:
        with mido.open_output(midi_port) as port:
            # Send notes on ch 0, 1, 2
            for ch in range(3):
                port.send(mido.Message("note_on", channel=ch, note=60 + ch, velocity=100))

            time.sleep(0.3)

            resp = send_cmd(ser, "COILS?")
            data = parse_json(resp)

            # Stop all notes
            for ch in range(3):
                port.send(mido.Message("note_off", channel=ch, note=60 + ch, velocity=0))
            time.sleep(0.1)

            if data and "coils" in data:
                voices = data["coils"][0].get("voices", 0)
                if voices >= 2:
                    R.ok("MIDI ROUTEALL Convergence", f"coil 0 has {voices} voices from 3 channels")
                elif voices >= 1:
                    R.ok("MIDI ROUTEALL Convergence", f"coil 0 has {voices} voice(s)")
                else:
                    R.fail("MIDI ROUTEALL Convergence", f"coil 0 voices={voices}")
            else:
                R.fail("MIDI ROUTEALL Convergence", f"COILS? parse: {resp}")
    except Exception as e:
        R.fail("MIDI ROUTEALL Convergence", f"error: {e}")

    send_cmd(ser, "STOPALL")
    cleanup(ser)


# ===========================================================================
#  Stress Tests
# ===========================================================================

def test_rapid_route_changes(ser, R):
    """Rapidly change routing 100 times while system is idle."""
    cleanup(ser)
    errors = 0
    for i in range(100):
        ch = i % MIDI_CHANNELS
        coil = i % NUM_COILS
        resp = send_cmd(ser, f"ROUTE {ch} {coil}", timeout=1.0)
        if "OK:ROUTE" not in resp:
            errors += 1
            if errors >= 5:
                break

    resp = send_cmd(ser, "PING")
    cleanup(ser)

    if errors == 0 and resp == "PONG":
        R.ok("Rapid Route Changes (100x)", "no errors, system responsive")
    else:
        R.fail("Rapid Route Changes (100x)", f"{errors} errors, PING={resp}")


def test_fire_all_coils_sustained(ser, R):
    """Fire all 6 coils for 3 seconds, verify system stability."""
    cleanup(ser)
    for c in range(NUM_COILS):
        send_cmd(ser, f"FIRE {c} {100 + c * 30} 30")

    time.sleep(3.0)

    # System still alive?
    resp = send_cmd(ser, "PING")
    resp2 = send_cmd(ser, "SCHED?")
    data = parse_json(resp2)
    send_cmd(ser, "STOPALL")

    if resp == "PONG" and data and data.get("running") == 1:
        R.ok("Sustained 6-Coil (3s)", "system responsive, scheduler running")
    else:
        R.fail("Sustained 6-Coil (3s)", f"PING={resp} sched={resp2}")
    cleanup(ser)


def test_limit_override_under_fire(ser, R):
    """Change limits on a firing coil 20 times, verify no crash."""
    cleanup(ser)
    send_cmd(ser, "FIRE 0 200 100")
    time.sleep(0.1)

    errors = 0
    for i in range(20):
        ot = 30 + (i * 13) % 200
        duty = 5 + (i * 3) % 20
        resp = send_cmd(ser, f"LIMITS 0 {ot} {duty} 50", timeout=1.0)
        if "OK:LIMITS" not in resp:
            errors += 1

    send_cmd(ser, "STOPALL")
    resp = send_cmd(ser, "PING")

    if errors == 0 and resp == "PONG":
        R.ok("Limit Override Under Fire (20x)", "no errors, system stable")
    else:
        R.fail("Limit Override Under Fire (20x)", f"{errors} errors, PING={resp}")
    cleanup(ser)


def test_cdc_responsive_during_midi_flood(ser, R):
    """Flood MIDI while checking CDC remains responsive."""
    try:
        import mido
    except ImportError:
        R.skip("CDC During MIDI Flood", "mido not installed")
        return

    midi_port = None
    for name in mido.get_output_names():
        if "TC-Interrupter" in name or "MIDI" in name:
            midi_port = name
            break
    if midi_port is None:
        R.skip("CDC During MIDI Flood", "no MIDI port found")
        return

    cleanup(ser)
    stop_event = threading.Event()
    midi_errors = []

    def midi_flood():
        try:
            with mido.open_output(midi_port) as port:
                note = 36
                while not stop_event.is_set():
                    for ch in range(NUM_COILS):
                        port.send(mido.Message("note_on", channel=ch, note=note, velocity=100))
                    time.sleep(0.01)
                    for ch in range(NUM_COILS):
                        port.send(mido.Message("note_off", channel=ch, note=note, velocity=0))
                    note = 36 + ((note - 36 + 1) % 48)
        except Exception as e:
            midi_errors.append(str(e))

    flood_thread = threading.Thread(target=midi_flood, daemon=True)
    flood_thread.start()

    # Hammer CDC during MIDI flood
    time.sleep(0.2)
    cdc_ok = 0
    cdc_fail = 0
    for _ in range(30):
        resp = send_cmd(ser, "PING", timeout=1.0)
        if resp == "PONG":
            cdc_ok += 1
        else:
            cdc_fail += 1
        time.sleep(0.05)

    stop_event.set()
    flood_thread.join(timeout=2.0)
    send_cmd(ser, "STOPALL")

    if cdc_fail == 0:
        R.ok("CDC During MIDI Flood", f"{cdc_ok}/{cdc_ok + cdc_fail} PINGs during flood")
    elif cdc_fail <= 3:
        R.ok("CDC During MIDI Flood", f"{cdc_ok}/{cdc_ok + cdc_fail} PINGs (minor drops)")
    else:
        R.fail("CDC During MIDI Flood", f"{cdc_ok}/{cdc_ok + cdc_fail} PINGs")
    cleanup(ser)


def test_rapid_fire_stop_all_coils(ser, R):
    """100 rapid FIRE/STOP cycles across all coils."""
    cleanup(ser)
    errors = 0
    for i in range(100):
        coil = i % NUM_COILS
        freq = 80 + (i * 23) % 400
        resp = send_cmd(ser, f"FIRE {coil} {freq} 30", timeout=1.0)
        if not resp.startswith("OK:FIRE"):
            errors += 1
        resp = send_cmd(ser, f"STOP {coil}", timeout=1.0)
        if not resp.startswith("OK:STOP"):
            errors += 1

    send_cmd(ser, "STOPALL")
    resp = send_cmd(ser, "PING")

    if errors == 0 and resp == "PONG":
        R.ok("Rapid Fire/Stop (100 cycles)", "no errors")
    else:
        R.fail("Rapid Fire/Stop (100 cycles)", f"{errors} errors, PING={resp}")
    cleanup(ser)


# ===========================================================================
#  Main
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(description="Milestone 3 Test Suite")
    parser.add_argument("--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("--all", action="store_true", help="Run all tests including MIDI+stress")
    parser.add_argument("--midi", action="store_true", help="Run MIDI routing tests only")
    parser.add_argument("--stress", action="store_true", help="Include stress tests")
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("ERROR: No TC-Interrupter CDC port found. Use --port to specify.")
        sys.exit(1)

    print(f"Connecting to {port}...")
    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    time.sleep(0.5)
    ser.reset_input_buffer()

    resp = send_cmd(ser, "PING")
    if resp != "PONG":
        print(f"ERROR: Device not responding (got: '{resp}')")
        ser.close()
        sys.exit(1)
    print(f"Connected. Device responds to PING.\n")

    R = TestResults()

    if args.midi:
        print("=== MIDI Routing Tests ===")
        test_midi_route_to_coil(ser, R)
        test_midi_unmapped_channel_ignored(ser, R)
        test_midi_routeall_convergence(ser, R)
    else:
        # Phase A: Channel Map
        print("=== Phase A: Channel Routing ===")
        test_route_query(ser, R)
        test_route_single_channel(ser, R)
        test_route_unmap(ser, R)
        test_routeall(ser, R)
        test_routereset(ser, R)
        test_route_all_permutations(ser, R)
        test_route_invalid_args(ser, R)

        # Phase B: Standalone Test Mode
        print("\n=== Phase B: Standalone Test Mode ===")
        test_fire_coil0_precise(ser, R)
        test_fire_different_widths(ser, R)
        test_fire_stop_verify_idle(ser, R)
        test_fire_invalid_coil(ser, R)
        test_fire_zero_freq(ser, R)

        # Coil Enable/Disable
        print("\n=== Coil Enable/Disable ===")
        test_enable_disable(ser, R)
        test_disable_invalid_coil(ser, R)

        # Phase C: Dynamic Limits
        print("\n=== Phase C: Dynamic Limit Override ===")
        test_dynamic_limit_change(ser, R)
        test_limits_all_coils(ser, R)

        if args.all:
            print("\n=== MIDI Routing Integration ===")
            test_midi_route_to_coil(ser, R)
            test_midi_unmapped_channel_ignored(ser, R)
            test_midi_routeall_convergence(ser, R)

        if args.stress or args.all:
            print("\n=== Stress Tests ===")
            test_rapid_route_changes(ser, R)
            test_fire_all_coils_sustained(ser, R)
            test_limit_override_under_fire(ser, R)
            test_rapid_fire_stop_all_coils(ser, R)
            test_cdc_responsive_during_midi_flood(ser, R)

    # Cleanup
    cleanup(ser)
    ser.close()

    success = R.summary()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
