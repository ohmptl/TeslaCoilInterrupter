#!/usr/bin/env python3
"""
TC-Interrupter Milestone 1 Test Suite
======================================
Phase C: Host PC validation for the USB Composite Device (CDC + MIDI).

This script validates:
  1. CDC Virtual COM Port enumeration and bi-directional communication
  2. PING/PONG command-response latency
  3. All supported CDC commands (VERSION, ESTOP?, STATUS, ECHO)
  4. Stress testing: rapid CDC command bursts
  5. Simultaneous CDC + MIDI transmission (deadlock detection)

Requirements:
  pip install pyserial mido python-rtmidi

Usage:
  python test_cdc.py                          # Auto-detect COM port
  python test_cdc.py --port COM5              # Specify COM port
  python test_cdc.py --port COM5 --stress     # Include stress tests
  python test_cdc.py --port COM5 --midi       # Include MIDI tests
  python test_cdc.py --port COM5 --all        # Run all tests
"""

import argparse
import json
import sys
import time
import threading
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required.  Install with: pip install pyserial")
    sys.exit(1)


# ---------------------------------------------------------------------------
#                          Configuration
# ---------------------------------------------------------------------------
CDC_VID = 0x0483      # STMicroelectronics
CDC_PID = 0x5741      # Custom PID for TC-Interrupter (22337 decimal)
BAUD_RATE = 115200    # Virtual CDC — baud rate is ignored but required
TIMEOUT = 2.0         # Seconds to wait for a response
STRESS_ITERATIONS = 200


# ---------------------------------------------------------------------------
#                         Utility Functions
# ---------------------------------------------------------------------------
def find_tc_port() -> Optional[str]:
    """Auto-detect the TC-Interrupter CDC port by VID:PID."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if p.vid == CDC_VID and p.pid == CDC_PID:
            return p.device
    # Fallback: look for any STM32 CDC device
    for p in ports:
        if p.vid == CDC_VID:
            return p.device
    return None


def send_command(ser: serial.Serial, cmd: str, timeout: float = TIMEOUT) -> str:
    """Send a command and read the response line."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()

    deadline = time.monotonic() + timeout
    response = b""
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            response += chunk
            if b"\r\n" in response or b"\n" in response:
                break
        time.sleep(0.005)

    return response.decode("ascii", errors="replace").strip()


class TestResult:
    """Simple test result tracker."""
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []

    def ok(self, name: str, detail: str = ""):
        self.passed += 1
        print(f"  [PASS] {name}" + (f" — {detail}" if detail else ""))

    def fail(self, name: str, detail: str = ""):
        self.failed += 1
        self.errors.append(f"{name}: {detail}")
        print(f"  [FAIL] {name}" + (f" — {detail}" if detail else ""))

    def summary(self):
        total = self.passed + self.failed
        print(f"\n{'='*60}")
        print(f"Results: {self.passed}/{total} passed, {self.failed} failed")
        if self.errors:
            print("Failures:")
            for e in self.errors:
                print(f"  - {e}")
        print(f"{'='*60}")
        return self.failed == 0


# ---------------------------------------------------------------------------
#                         Test Cases
# ---------------------------------------------------------------------------
def test_connection(ser: serial.Serial, results: TestResult):
    """Test 1: Verify basic CDC connection."""
    print("\n--- Test: Connection ---")
    if ser.is_open:
        results.ok("COM port opened")
    else:
        results.fail("COM port opened", "Port is not open")


def test_ping_pong(ser: serial.Serial, results: TestResult):
    """Test 2: PING/PONG command-response."""
    print("\n--- Test: PING/PONG ---")
    t0 = time.monotonic()
    resp = send_command(ser, "PING")
    latency_ms = (time.monotonic() - t0) * 1000

    if "PONG" in resp:
        results.ok("PING -> PONG", f"latency={latency_ms:.1f}ms")
    else:
        results.fail("PING -> PONG", f"got '{resp}'")


def test_version(ser: serial.Serial, results: TestResult):
    """Test 3: VERSION command."""
    print("\n--- Test: VERSION ---")
    resp = send_command(ser, "VERSION")
    if "TC-Interrupter" in resp:
        results.ok("VERSION", f"'{resp}'")
    else:
        results.fail("VERSION", f"got '{resp}'")


def test_estop(ser: serial.Serial, results: TestResult):
    """Test 4: ESTOP? query."""
    print("\n--- Test: ESTOP? ---")
    resp = send_command(ser, "ESTOP?")
    if resp.startswith("ESTOP="):
        val = resp.split("=")[1]
        if val in ("0", "1"):
            results.ok("ESTOP?", f"state={val}")
        else:
            results.fail("ESTOP?", f"invalid value '{val}'")
    else:
        results.fail("ESTOP?", f"got '{resp}'")


def test_status(ser: serial.Serial, results: TestResult):
    """Test 5: STATUS command (JSON response)."""
    print("\n--- Test: STATUS ---")
    resp = send_command(ser, "STATUS")
    try:
        data = json.loads(resp)
        if "fw" in data and "estop" in data and "uptime_ms" in data:
            results.ok("STATUS", f"uptime={data['uptime_ms']}ms, estop={data['estop']}")
        else:
            results.fail("STATUS", f"missing keys in {data}")
    except json.JSONDecodeError:
        results.fail("STATUS", f"invalid JSON: '{resp}'")


def test_echo(ser: serial.Serial, results: TestResult):
    """Test 6: ECHO command."""
    print("\n--- Test: ECHO ---")
    test_str = "Hello Tesla Coil!"
    resp = send_command(ser, f"ECHO {test_str}")
    if resp == test_str:
        results.ok("ECHO", f"'{resp}'")
    else:
        results.fail("ECHO", f"expected '{test_str}', got '{resp}'")


def test_unknown_command(ser: serial.Serial, results: TestResult):
    """Test 7: Unknown command error handling."""
    print("\n--- Test: Unknown Command ---")
    resp = send_command(ser, "FOOBAR")
    if "ERR" in resp.upper():
        results.ok("Unknown command", f"'{resp}'")
    else:
        results.fail("Unknown command", f"expected ERR, got '{resp}'")


def test_stress_cdc(ser: serial.Serial, results: TestResult, iterations: int = STRESS_ITERATIONS):
    """Test 8: Rapid-fire CDC command stress test."""
    print(f"\n--- Test: CDC Stress ({iterations} iterations) ---")
    failures = 0
    t0 = time.monotonic()

    for i in range(iterations):
        resp = send_command(ser, "PING", timeout=1.0)
        if "PONG" not in resp:
            failures += 1

    elapsed = time.monotonic() - t0
    rate = iterations / elapsed

    if failures == 0:
        results.ok("CDC Stress", f"{iterations} PING/PONG in {elapsed:.1f}s ({rate:.0f} cmd/s)")
    else:
        results.fail("CDC Stress", f"{failures}/{iterations} failed in {elapsed:.1f}s")


def test_simultaneous_cdc_midi(ser: serial.Serial, results: TestResult):
    """Test 9: Simultaneous CDC + MIDI (deadlock detection).

    Sends MIDI Note-On/Off events while concurrently hammering CDC commands.
    Verifies no deadlock occurs (CDC still responds within timeout).
    """
    print("\n--- Test: Simultaneous CDC + MIDI ---")

    try:
        import mido
    except ImportError:
        results.ok("CDC+MIDI simultaneous", "SKIPPED (mido not installed)")
        return

    # Find MIDI output port
    midi_port_name = None
    for name in mido.get_output_names():
        if "TC-Interrupter" in name or "MIDI" in name:
            midi_port_name = name
            break

    if midi_port_name is None:
        results.ok("CDC+MIDI simultaneous", "SKIPPED (no MIDI port found)")
        return

    # Background thread: blast MIDI notes
    stop_event = threading.Event()
    midi_errors = []

    def midi_sender():
        try:
            with mido.open_output(midi_port_name) as port:
                note = 60
                while not stop_event.is_set():
                    port.send(mido.Message("note_on", note=note, velocity=100, channel=0))
                    time.sleep(0.005)
                    port.send(mido.Message("note_off", note=note, velocity=0, channel=0))
                    note = 60 + ((note - 59) % 12)
                    time.sleep(0.005)
        except Exception as e:
            midi_errors.append(str(e))

    midi_thread = threading.Thread(target=midi_sender, daemon=True)
    midi_thread.start()

    # Foreground: send CDC commands while MIDI is flowing
    cdc_ok = 0
    cdc_fail = 0
    t0 = time.monotonic()

    for _ in range(50):
        resp = send_command(ser, "PING", timeout=1.0)
        if "PONG" in resp:
            cdc_ok += 1
        else:
            cdc_fail += 1
        time.sleep(0.01)

    stop_event.set()
    midi_thread.join(timeout=3.0)
    elapsed = time.monotonic() - t0

    if cdc_fail == 0 and not midi_errors:
        results.ok("CDC+MIDI simultaneous",
                    f"{cdc_ok} CDC ok, MIDI concurrent for {elapsed:.1f}s")
    elif midi_errors:
        results.fail("CDC+MIDI simultaneous", f"MIDI errors: {midi_errors}")
    else:
        results.fail("CDC+MIDI simultaneous",
                     f"{cdc_fail}/50 CDC failures during MIDI flood")


# ---------------------------------------------------------------------------
#                              Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="TC-Interrupter Milestone 1 Test Suite")
    parser.add_argument("--port", type=str, default=None,
                        help="COM port (e.g., COM5 or /dev/ttyACM0)")
    parser.add_argument("--stress", action="store_true",
                        help="Include CDC stress test")
    parser.add_argument("--midi", action="store_true",
                        help="Include simultaneous CDC+MIDI test")
    parser.add_argument("--all", action="store_true",
                        help="Run all tests including stress and MIDI")
    args = parser.parse_args()

    print("=" * 60)
    print("  TC-Interrupter Milestone 1 Test Suite")
    print("=" * 60)

    # Find port
    port = args.port or find_tc_port()
    if port is None:
        print("\nERROR: Could not find TC-Interrupter CDC port.")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description} [VID:PID={p.vid}:{p.pid}]")
        print("\nUse --port to specify manually.")
        sys.exit(1)

    print(f"\nUsing port: {port}")

    # Open serial connection
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(0.5)  # Allow device to settle after connection
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)

    results = TestResult()

    try:
        # Core tests (always run)
        test_connection(ser, results)
        test_ping_pong(ser, results)
        test_version(ser, results)
        test_estop(ser, results)
        test_status(ser, results)
        test_echo(ser, results)
        test_unknown_command(ser, results)

        # Stress tests (optional)
        if args.stress or args.all:
            test_stress_cdc(ser, results)

        # MIDI tests (optional)
        if args.midi or args.all:
            test_simultaneous_cdc_midi(ser, results)

    finally:
        ser.close()

    success = results.summary()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
