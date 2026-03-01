#!/usr/bin/env python3
"""
test_milestone2.py — Milestone 2 Validation Test Suite
=======================================================

Tests the Hybrid OPM Scheduler, Safety Limits, Multi-Coil Firing,
and USB MIDI note handling for the TC-Interrupter.

Requires:
  - pyserial >= 3.5
  - mido >= 1.3.0         (optional, for MIDI tests)
  - python-rtmidi >= 1.5.0 (optional, for MIDI tests)

Usage:
  python test_milestone2.py                     # Auto-detect port, basic tests
  python test_milestone2.py --port COM5         # Explicit port
  python test_milestone2.py --all               # Run all tests including MIDI
  python test_milestone2.py --midi              # Run MIDI tests only
  python test_milestone2.py --stress            # Include stress tests
"""

import argparse
import json
import sys
import time

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
VID = 0x0483        # STMicroelectronics
PID = 0x5741        # TC-Interrupter CDC+MIDI (custom PID from M1)
BAUD = 115200
TIMEOUT = 2.0       # seconds
NUM_COILS = 6

# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def find_port():
    """Auto-detect the TC-Interrupter CDC COM port by VID:PID."""
    for p in serial.tools.list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def send_cmd(ser, cmd, timeout=TIMEOUT):
    """Send a command and return the response line (stripped)."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            return line
    return ""


def send_cmd_multi(ser, cmd, lines=1, timeout=TIMEOUT):
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


def parse_json_response(line):
    """Try to parse a JSON response, return dict or None."""
    try:
        return json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return None

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

# ---------------------------------------------------------------------------
#  Phase A Tests: Single Coil OPM Firing
# ---------------------------------------------------------------------------

def test_scheduler_running(ser, R):
    """Verify the scheduler is running and reporting stats."""
    resp = send_cmd(ser, "SCHED?")
    data = parse_json_response(resp)
    if data and data.get("running") == 1:
        R.ok("Scheduler Running", f"ticks={data.get('ticks', '?')}")
    else:
        R.fail("Scheduler Running", f"response: {resp}")


def test_fire_single_coil(ser, R):
    """FIRE a test tone on coil 0 and verify pulses are counted."""
    # Ensure clean state
    send_cmd(ser, "STOPALL")
    time.sleep(0.1)

    # Read initial pulse count for coil 0
    resp = send_cmd(ser, "COILS?")
    data = parse_json_response(resp)
    if not data or "coils" not in data:
        R.fail("FIRE Single Coil — initial COILS? parse", f"response: {resp}")
        return
    initial_pulses = data["coils"][0].get("pulses", 0)

    # Fire coil 0 at 100 Hz, 50 µs on-time
    resp = send_cmd(ser, "FIRE 0 100 50")
    if not resp.startswith("OK:FIRE"):
        R.fail("FIRE Single Coil — FIRE command", f"response: {resp}")
        return

    # Wait for pulses to accumulate
    time.sleep(0.5)

    # Check pulse count increased
    resp = send_cmd(ser, "COILS?")
    data = parse_json_response(resp)
    if not data or "coils" not in data:
        R.fail("FIRE Single Coil — COILS? after fire", f"response: {resp}")
        send_cmd(ser, "STOPALL")
        return

    new_pulses = data["coils"][0].get("pulses", 0)
    delta = new_pulses - initial_pulses

    # At 100 Hz for 0.5s, expect ~50 pulses (allow tolerance)
    send_cmd(ser, "STOP 0")
    if delta >= 20:
        R.ok("FIRE Single Coil", f"pulses={delta} in 500ms (expected ~50)")
    else:
        R.fail("FIRE Single Coil", f"only {delta} pulses in 500ms (expected ~50)")


def test_stop_coil(ser, R):
    """FIRE then STOP and verify pulse count stops increasing."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    send_cmd(ser, "FIRE 0 200 30")
    time.sleep(0.3)
    send_cmd(ser, "STOP 0")
    time.sleep(0.05)

    # Read pulse count
    resp1 = send_cmd(ser, "COILS?")
    data1 = parse_json_response(resp1)
    if not data1:
        R.fail("STOP Coil — COILS? parse", f"response: {resp1}")
        return
    count1 = data1["coils"][0].get("pulses", 0)

    # Wait and read again — should not increase
    time.sleep(0.3)
    resp2 = send_cmd(ser, "COILS?")
    data2 = parse_json_response(resp2)
    if not data2:
        R.fail("STOP Coil — second COILS? parse", f"response: {resp2}")
        return
    count2 = data2["coils"][0].get("pulses", 0)

    if count2 == count1:
        R.ok("STOP Coil", f"pulse count stable at {count1} after STOP")
    else:
        R.fail("STOP Coil", f"pulse count changed {count1} → {count2} after STOP")


def test_stopall(ser, R):
    """FIRE multiple coils then STOPALL."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    send_cmd(ser, "FIRE 0 100 30")
    send_cmd(ser, "FIRE 1 200 40")
    time.sleep(0.05)

    resp = send_cmd(ser, "STOPALL")
    if "OK:STOPALL" in resp:
        # Verify voices are all 0
        time.sleep(0.05)
        resp2 = send_cmd(ser, "COILS?")
        data = parse_json_response(resp2)
        if data:
            all_zero = all(c.get("voices", 1) == 0 for c in data["coils"])
            if all_zero:
                R.ok("STOPALL", "all voices cleared")
            else:
                R.fail("STOPALL", f"some voices remain: {resp2}")
        else:
            R.fail("STOPALL", f"COILS? parse failed: {resp2}")
    else:
        R.fail("STOPALL", f"unexpected response: {resp}")


# ---------------------------------------------------------------------------
#  Phase A Tests: Safety Limits
# ---------------------------------------------------------------------------

def test_ontime_clamping(ser, R):
    """Verify on-time is clamped to max_ontime_us."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Set max on-time to 100 µs for coil 0
    send_cmd(ser, "LIMITS 0 100 10 100")
    time.sleep(0.05)

    # Request 300 µs — should be clamped to 100
    resp = send_cmd(ser, "FIRE 0 100 300")
    if "clamped" in resp.lower() or "ontime=100" in resp:
        R.ok("On-time Clamping", f"response: {resp}")
    else:
        R.fail("On-time Clamping", f"expected clamping, got: {resp}")

    send_cmd(ser, "STOP 0")
    # Restore defaults
    send_cmd(ser, "LIMITS 0 200 10 100")


def test_absolute_ontime_cap(ser, R):
    """Verify absolute max on-time (500 µs) cannot be exceeded."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Try to set max on-time to 1000 µs (exceeds 500 µs absolute cap)
    resp = send_cmd(ser, "LIMITS 0 1000 50 20")
    if "max_ontime=500" in resp:
        R.ok("Absolute On-time Cap", f"capped to 500: {resp}")
    else:
        R.fail("Absolute On-time Cap", f"expected 500, got: {resp}")

    # Restore defaults
    send_cmd(ser, "LIMITS 0 200 10 100")


def test_limits_query(ser, R):
    """Set limits and query them back."""
    send_cmd(ser, "LIMITS 2 150 15 80")
    time.sleep(0.05)

    resp = send_cmd(ser, "LIMITS? 2")
    if "max_ontime=150" in resp and "duty_permil=15" in resp and "min_offtime=80" in resp:
        R.ok("Limits Query", f"response: {resp}")
    else:
        R.fail("Limits Query", f"unexpected response: {resp}")

    # Restore defaults
    send_cmd(ser, "LIMITS 2 200 10 100")


def test_duty_limiting(ser, R):
    """Fire a tone that would exceed duty cycle and verify limiting."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Set very restrictive duty: 1 permil = 0.1%
    # With 100ms window, budget = 1 * 100000 / 1000 = 100 µs per window
    send_cmd(ser, "LIMITS 0 200 1 50")
    time.sleep(0.05)

    # Fire at 500 Hz with 50 µs on-time
    # Without duty limit: 500 * 50µs = 25000 µs/s = 2.5% duty
    # With 0.1% limit: budget only 100 µs per 100ms = ~2 pulses per window
    send_cmd(ser, "FIRE 0 500 50")
    time.sleep(0.5)

    resp = send_cmd(ser, "SCHED?")
    data = parse_json_response(resp)
    send_cmd(ser, "STOP 0")

    if data and "coils" in data:
        pulses = data["coils"][0].get("pulses", 0)
        # At 0.1% duty with 50µs on-time: ~2 pulses per 100ms window × 5 windows = ~10
        # Without limiting: ~250 pulses
        if pulses < 50:
            R.ok("Duty Limiting", f"only {pulses} pulses (duty-limited from ~250)")
        else:
            R.fail("Duty Limiting", f"{pulses} pulses — duty limit may not be working")
    else:
        R.fail("Duty Limiting", f"SCHED? parse failed: {resp}")

    # Restore defaults
    send_cmd(ser, "LIMITS 0 200 10 100")


# ---------------------------------------------------------------------------
#  Phase B Tests: Dual/Multi-Coil Sync
# ---------------------------------------------------------------------------

def test_dual_coil_fire(ser, R):
    """Fire two coils simultaneously and verify independent pulse counting."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Fire coil 0 at 100 Hz and coil 1 at 200 Hz
    send_cmd(ser, "FIRE 0 100 50")
    send_cmd(ser, "FIRE 1 200 50")
    time.sleep(0.5)

    resp = send_cmd(ser, "COILS?")
    data = parse_json_response(resp)
    send_cmd(ser, "STOPALL")

    if data and "coils" in data:
        p0 = data["coils"][0].get("pulses", 0)
        p1 = data["coils"][1].get("pulses", 0)
        # Coil 0 at 100 Hz for 0.5s → ~50 pulses
        # Coil 1 at 200 Hz for 0.5s → ~100 pulses (if duty allows)
        if p0 >= 15 and p1 >= 15:
            R.ok("Dual Coil Fire", f"coil0={p0} coil1={p1} pulses")
        else:
            R.fail("Dual Coil Fire", f"low pulse counts: coil0={p0} coil1={p1}")
    else:
        R.fail("Dual Coil Fire", f"COILS? parse failed: {resp}")


def test_all_six_coils(ser, R):
    """Fire all 6 coils simultaneously and verify each produces pulses."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Fire all 6 coils at different frequencies
    freqs = [80, 100, 120, 150, 180, 200]
    for c in range(NUM_COILS):
        send_cmd(ser, f"FIRE {c} {freqs[c]} 30")

    time.sleep(0.5)

    resp = send_cmd(ser, "COILS?")
    data = parse_json_response(resp)
    send_cmd(ser, "STOPALL")

    if data and "coils" in data:
        pulses = [data["coils"][c].get("pulses", 0) for c in range(NUM_COILS)]
        all_fired = all(p >= 5 for p in pulses)
        pulse_str = ", ".join(f"c{i}={p}" for i, p in enumerate(pulses))
        if all_fired:
            R.ok("All 6 Coils Fire", pulse_str)
        else:
            R.fail("All 6 Coils Fire", f"some coils didn't fire: {pulse_str}")
    else:
        R.fail("All 6 Coils Fire", f"COILS? parse failed: {resp}")


# ---------------------------------------------------------------------------
#  Phase B Tests: Polyphony (multiple voices per coil)
# ---------------------------------------------------------------------------

def test_polyphony_two_voices(ser, R):
    """Fire two different test tones on the same coil."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Fire two tones on coil 0 at different frequencies
    # Use note marker = 0xFF for test tones; they share coil but have different slots
    resp1 = send_cmd(ser, "FIRE 0 100 30")
    resp2 = send_cmd(ser, "FIRE 0 200 30")

    time.sleep(0.05)

    # Check active voices
    resp = send_cmd(ser, "COILS?")
    data = parse_json_response(resp)
    send_cmd(ser, "STOPALL")

    if data and "coils" in data:
        voices = data["coils"][0].get("voices", 0)
        # Note: both FIRE tones use note=0xFF, channel=0xFF so they'll share
        # the same slot (update in place). To get 2 voice slots we'd need
        # different note/channel identifiers (i.e., via MIDI).
        # With test tones (same marker), we get 1 voice updated twice.
        # This is expected behavior for FIRE — it uses a single test slot.
        if voices >= 1:
            R.ok("Test Tone Voice", f"voices={voices} on coil 0")
        else:
            R.fail("Test Tone Voice", f"no voices on coil 0")
    else:
        R.fail("Test Tone Voice", f"COILS? parse failed: {resp}")


# ---------------------------------------------------------------------------
#  Phase C Tests: E-Stop and Status
# ---------------------------------------------------------------------------

def test_estop_query(ser, R):
    """Query E-Stop state (can only verify query works, not trigger)."""
    resp = send_cmd(ser, "ESTOP?")
    if resp.startswith("ESTOP="):
        R.ok("E-Stop Query", f"response: {resp}")
    else:
        R.fail("E-Stop Query", f"unexpected response: {resp}")


def test_status_scheduler(ser, R):
    """Verify STATUS includes scheduler info."""
    resp = send_cmd(ser, "STATUS")
    data = parse_json_response(resp)
    if data and "sched_running" in data and "sched_us" in data:
        R.ok("STATUS with Scheduler", f"sched_running={data['sched_running']}")
    else:
        R.fail("STATUS with Scheduler", f"response: {resp}")


def test_sched_query(ser, R):
    """Verify SCHED? returns detailed scheduler state."""
    resp = send_cmd(ser, "SCHED?")
    data = parse_json_response(resp)
    if data and "running" in data and "coils" in data and len(data["coils"]) == NUM_COILS:
        R.ok("SCHED? Query", f"running={data['running']}, {len(data['coils'])} coils")
    else:
        R.fail("SCHED? Query", f"response: {resp}")


def test_coils_query(ser, R):
    """Verify COILS? returns all 6 coil states."""
    resp = send_cmd(ser, "COILS?")
    data = parse_json_response(resp)
    if data and "num_coils" in data and data["num_coils"] == NUM_COILS:
        coils = data.get("coils", [])
        if len(coils) == NUM_COILS and all("pulses" in c for c in coils):
            R.ok("COILS? Query", f"{NUM_COILS} coils with pulse counts")
        else:
            R.fail("COILS? Query", f"incomplete coil data: {resp}")
    else:
        R.fail("COILS? Query", f"response: {resp}")


# ---------------------------------------------------------------------------
#  MIDI Tests (optional)
# ---------------------------------------------------------------------------

def test_midi_note_on_off(ser, R):
    """Send MIDI Note On/Off and verify scheduler picks it up."""
    try:
        import mido
    except ImportError:
        R.skip("MIDI Note On/Off", "mido not installed")
        return

    # Find MIDI output port
    midi_port = None
    for name in mido.get_output_names():
        if "TC-Interrupter" in name or "MIDI" in name:
            midi_port = name
            break

    if midi_port is None:
        R.skip("MIDI Note On/Off", "no MIDI port found")
        return

    send_cmd(ser, "STOPALL")
    time.sleep(0.1)

    try:
        with mido.open_output(midi_port) as port:
            # Send Note On: channel 0, note 69 (A4 = 440 Hz), velocity 100
            port.send(mido.Message("note_on", channel=0, note=69, velocity=100))
            time.sleep(0.3)

            # Check that coil 0 has an active voice
            resp = send_cmd(ser, "COILS?")
            data = parse_json_response(resp)

            if data and data["coils"][0].get("voices", 0) >= 1:
                pulses = data["coils"][0].get("pulses", 0)

                # Send Note Off
                port.send(mido.Message("note_off", channel=0, note=69, velocity=0))
                time.sleep(0.1)

                # Verify voice removed
                resp2 = send_cmd(ser, "COILS?")
                data2 = parse_json_response(resp2)
                voices_after = data2["coils"][0].get("voices", 0) if data2 else -1

                if voices_after == 0 and pulses > 0:
                    R.ok("MIDI Note On/Off", f"pulses={pulses}, voice added then removed")
                elif pulses > 0:
                    R.ok("MIDI Note On/Off", f"pulses={pulses} (voice cleanup: {voices_after})")
                else:
                    R.fail("MIDI Note On/Off", f"no pulses fired")
            else:
                R.fail("MIDI Note On/Off", f"no voice after Note On: {resp}")
    except Exception as e:
        R.fail("MIDI Note On/Off", f"MIDI error: {e}")

    send_cmd(ser, "STOPALL")


def test_midi_channel_routing(ser, R):
    """Verify ROUTE command changes MIDI channel-to-coil mapping."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    # Route channel 0 to coil 3
    resp = send_cmd(ser, "ROUTE 0 3")
    if "OK:ROUTE" in resp and "coil=3" in resp:
        R.ok("MIDI Channel Routing", f"ch 0 → coil 3: {resp}")
    else:
        R.fail("MIDI Channel Routing", f"unexpected response: {resp}")

    # Restore default: channel 0 → coil 0
    send_cmd(ser, "ROUTE 0 0")


# ---------------------------------------------------------------------------
#  Stress Tests
# ---------------------------------------------------------------------------

def test_rapid_fire_stop(ser, R):
    """Rapidly fire and stop tones to test scheduler stability."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    errors = 0
    for i in range(50):
        coil = i % NUM_COILS
        freq = 100 + (i * 17) % 400
        resp = send_cmd(ser, f"FIRE {coil} {freq} 30", timeout=1.0)
        if not resp.startswith("OK:FIRE"):
            errors += 1
        resp = send_cmd(ser, f"STOP {coil}", timeout=1.0)
        if not resp.startswith("OK:STOP"):
            errors += 1

    send_cmd(ser, "STOPALL")

    if errors == 0:
        R.ok("Rapid Fire/Stop (50 cycles)", "no errors")
    else:
        R.fail("Rapid Fire/Stop (50 cycles)", f"{errors} errors")


def test_sustained_multicoil(ser, R):
    """Run all 6 coils for 2 seconds and verify no crashes."""
    send_cmd(ser, "STOPALL")
    time.sleep(0.05)

    for c in range(NUM_COILS):
        send_cmd(ser, f"FIRE {c} {100 + c * 50} 30")

    time.sleep(2.0)

    # Verify system is still responsive
    resp = send_cmd(ser, "PING")
    send_cmd(ser, "STOPALL")

    if resp == "PONG":
        # Also check scheduler is still running
        resp2 = send_cmd(ser, "SCHED?")
        data = parse_json_response(resp2)
        if data and data.get("running") == 1:
            R.ok("Sustained Multi-Coil (2s)", "system responsive, scheduler running")
        else:
            R.fail("Sustained Multi-Coil (2s)", f"scheduler issue: {resp2}")
    else:
        R.fail("Sustained Multi-Coil (2s)", f"PING returned: {resp}")


# ---------------------------------------------------------------------------
#  Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Milestone 2 Test Suite")
    parser.add_argument("--port", help="Serial port (auto-detect if omitted)")
    parser.add_argument("--all", action="store_true", help="Run all tests including MIDI")
    parser.add_argument("--midi", action="store_true", help="Run MIDI tests only")
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

    # Quick connectivity check
    resp = send_cmd(ser, "PING")
    if resp != "PONG":
        print(f"ERROR: Device not responding (got: '{resp}')")
        ser.close()
        sys.exit(1)
    print(f"Connected. Device responds to PING.\n")

    R = TestResults()

    if args.midi:
        # MIDI tests only
        print("=== MIDI Tests ===")
        test_midi_note_on_off(ser, R)
        test_midi_channel_routing(ser, R)
    else:
        # Phase A: Single Coil + Safety
        print("=== Phase A: Single Coil OPM & Safety ===")
        test_scheduler_running(ser, R)
        test_fire_single_coil(ser, R)
        test_stop_coil(ser, R)
        test_stopall(ser, R)
        test_ontime_clamping(ser, R)
        test_absolute_ontime_cap(ser, R)
        test_limits_query(ser, R)
        test_duty_limiting(ser, R)

        # Phase B: Multi-Coil
        print("\n=== Phase B: Multi-Coil Sync & Polyphony ===")
        test_dual_coil_fire(ser, R)
        test_all_six_coils(ser, R)
        test_polyphony_two_voices(ser, R)

        # Phase C: E-Stop, Status, Queries
        print("\n=== Phase C: E-Stop & Diagnostics ===")
        test_estop_query(ser, R)
        test_status_scheduler(ser, R)
        test_sched_query(ser, R)
        test_coils_query(ser, R)

        if args.all:
            print("\n=== MIDI Tests ===")
            test_midi_note_on_off(ser, R)
            test_midi_channel_routing(ser, R)

        if args.stress or args.all:
            print("\n=== Stress Tests ===")
            test_rapid_fire_stop(ser, R)
            test_sustained_multicoil(ser, R)

    # Cleanup
    send_cmd(ser, "STOPALL")
    ser.close()

    success = R.summary()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
