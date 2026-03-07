#!/usr/bin/env python3
"""
TC-Interrupter Test GUI
========================

A quick tkinter GUI for testing the TC-Interrupter hardware.
Two primary modes:
  1. Static Pulse Mode — Send direct FIRE commands (freq/ontime/coil)
  2. MIDI File Player   — Load and play .mid files through the device

Requires:
  - pyserial >= 3.5
  - mido >= 1.3.0
  - python-rtmidi >= 1.5.0

Usage:
  python tc_gui.py                  # Auto-detect port
  python tc_gui.py --port COM5     # Explicit port
"""

import argparse
import json
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
VID = 0x0483
PID = 0x5741
BAUD = 115200
NUM_COILS = 6
APP_TITLE = "TC-Interrupter Control Panel"
POLL_INTERVAL_MS = 500    # Status polling interval


# ---------------------------------------------------------------------------
#  Serial Communication Layer
# ---------------------------------------------------------------------------
class DeviceLink:
    """Thread-safe serial communication with the TC-Interrupter."""

    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    @staticmethod
    def find_port():
        for p in serial.tools.list_ports.comports():
            if p.vid == VID and p.pid == PID:
                return p.device
        return None

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, BAUD, timeout=1.5)
            time.sleep(0.3)
            self.ser.reset_input_buffer()

    def disconnect(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = None

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, cmd, timeout=2.0):
        """Send command, return response line."""
        with self.lock:
            if not self.connected:
                return ""
            try:
                self.ser.reset_input_buffer()
                self.ser.write((cmd + "\r\n").encode("ascii"))
                self.ser.flush()
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    line = self.ser.readline().decode("ascii", errors="replace").strip()
                    if line:
                        return line
            except (serial.SerialException, OSError):
                pass
            return ""

    def send_json(self, cmd, timeout=2.0):
        """Send command, parse JSON response."""
        line = self.send(cmd, timeout)
        try:
            return json.loads(line)
        except (json.JSONDecodeError, ValueError):
            return None


# ---------------------------------------------------------------------------
#  MIDI File Player (background thread)
# ---------------------------------------------------------------------------
class MidiPlayer:
    """Plays a MIDI file through the device's MIDI port in a background thread."""

    def __init__(self):
        self.thread = None
        self.stop_event = threading.Event()
        self.playing = False
        self.filename = ""
        self.progress = 0.0  # 0.0 to 1.0

    @staticmethod
    def list_midi_ports():
        """Return (list_of_port_names, best_match_or_None)."""
        try:
            import mido
            ports = mido.get_output_names()
        except ImportError:
            return [], None
        except Exception:
            return [], None

        if not ports:
            return [], None

        # Priority matching: most-specific first
        for name in ports:
            if "TC-Interrupter" in name:
                return ports, name
        # USB MIDI devices on Windows often appear as
        # "USB MIDI Interface 0" or the product string
        for name in ports:
            nl = name.lower()
            if "interrupter" in nl or "tc-" in nl or "stm" in nl:
                return ports, name
        # Fall back to any port with MIDI in the name (but not
        # built-in synths like "Microsoft GS Wavetable Synth")
        for name in ports:
            nl = name.lower()
            if "usb" in nl and "midi" in nl:
                return ports, name
        # If only one port exists, that's probably it
        if len(ports) == 1:
            return ports, ports[0]
        return ports, None

    @staticmethod
    def find_midi_port():
        _, best = MidiPlayer.list_midi_ports()
        return best

    def play(self, filepath, midi_port_name, on_finish=None):
        if self.playing:
            return
        self.stop_event.clear()
        self.filename = os.path.basename(filepath)
        self.progress = 0.0
        self.playing = True
        self.thread = threading.Thread(
            target=self._play_worker,
            args=(filepath, midi_port_name, on_finish),
            daemon=True
        )
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3.0)
        self.playing = False
        self.progress = 0.0

    def _play_worker(self, filepath, midi_port_name, on_finish):
        try:
            import mido
            mid = mido.MidiFile(filepath)
            total_time = mid.length or 1.0
            elapsed = 0.0

            with mido.open_output(midi_port_name) as port:
                for msg in mid.play():
                    if self.stop_event.is_set():
                        # Send All Notes Off on all channels
                        for ch in range(16):
                            port.send(mido.Message("control_change", channel=ch,
                                                   control=123, value=0))
                        break
                    if not msg.is_meta:
                        port.send(msg)
                    elapsed += msg.time if hasattr(msg, 'time') else 0
                    self.progress = min(elapsed / total_time, 1.0)

                # File finished naturally — send All Notes Off
                if not self.stop_event.is_set():
                    for ch in range(16):
                        port.send(mido.Message("control_change", channel=ch,
                                               control=123, value=0))
        except Exception as e:
            print(f"MIDI Player error: {e}")
        finally:
            self.playing = False
            self.progress = 1.0
            if on_finish:
                on_finish()


# ---------------------------------------------------------------------------
#  Main GUI Application
# ---------------------------------------------------------------------------
class TCInterrupterGUI:
    def __init__(self, root, initial_port=None):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("900x720")
        self.root.minsize(800, 600)

        self.dev = DeviceLink()
        self.player = MidiPlayer()
        self.polling = False
        self._coil_firing = False   # True while a test tone is active
        self._update_pending = None # Scheduled after-id for live updates

        self._build_ui()

        if initial_port:
            self.port_var.set(initial_port)
            self.root.after(100, self._connect)

    # -----------------------------------------------------------------------
    #  UI Construction
    # -----------------------------------------------------------------------
    def _build_ui(self):
        # -- Top: Connection Bar --
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, padx=8, pady=(8, 4))

        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var,
                                       width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 4))
        self._refresh_ports()

        ttk.Button(conn_frame, text="Refresh", command=self._refresh_ports,
                   width=7).pack(side=tk.LEFT, padx=2)
        self.connect_btn = ttk.Button(conn_frame, text="Connect",
                                      command=self._toggle_connect, width=10)
        self.connect_btn.pack(side=tk.LEFT, padx=2)

        self.conn_status = ttk.Label(conn_frame, text="Disconnected",
                                     foreground="red")
        self.conn_status.pack(side=tk.LEFT, padx=10)

        self.fw_label = ttk.Label(conn_frame, text="")
        self.fw_label.pack(side=tk.RIGHT, padx=5)

        # -- Notebook (Tabs) --
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        self._build_pulse_tab(nb)
        self._build_midi_tab(nb)
        self._build_routing_tab(nb)
        self._build_limits_tab(nb)
        self._build_console_tab(nb)

        # -- Bottom: Status Bar --
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, padx=8, pady=(0, 8))

        self.status_label = ttk.Label(status_frame, text="Ready")
        self.status_label.pack(side=tk.LEFT)

        self.estop_label = ttk.Label(status_frame, text="")
        self.estop_label.pack(side=tk.RIGHT, padx=10)

        self.sched_label = ttk.Label(status_frame, text="")
        self.sched_label.pack(side=tk.RIGHT, padx=10)

    # -- Pulse Tab --
    def _build_pulse_tab(self, nb):
        tab = ttk.Frame(nb, padding=10)
        nb.add(tab, text="  Static Pulses  ")

        # Coil selector
        sel_frame = ttk.LabelFrame(tab, text="Coil Selection", padding=8)
        sel_frame.pack(fill=tk.X, pady=(0, 8))

        self.coil_var = tk.IntVar(value=0)
        for c in range(NUM_COILS):
            pin_label = ["PE9", "PD12", "PE5", "PF6", "PF7", "PF8"][c]
            timer_label = ["TIM1", "TIM4", "TIM9", "TIM10", "TIM11", "TIM13"][c]
            ttk.Radiobutton(
                sel_frame, text=f"Coil {c+1} ({timer_label} / {pin_label})",
                variable=self.coil_var, value=c
            ).pack(side=tk.LEFT, padx=8)

        # Parameters
        param_frame = ttk.LabelFrame(tab, text="Pulse Parameters", padding=8)
        param_frame.pack(fill=tk.X, pady=(0, 8))

        # Frequency
        ttk.Label(param_frame, text="Frequency (Hz):").grid(row=0, column=0,
                                                             sticky=tk.W, padx=4, pady=4)
        self.freq_var = tk.IntVar(value=100)
        self.freq_scale = ttk.Scale(param_frame, from_=1, to=2000,
                                     variable=self.freq_var, orient=tk.HORIZONTAL,
                                     length=400)
        self.freq_scale.grid(row=0, column=1, padx=4, pady=4, sticky=tk.EW)
        self.freq_entry = ttk.Entry(param_frame, textvariable=self.freq_var, width=8)
        self.freq_entry.grid(row=0, column=2, padx=4, pady=4)
        ttk.Label(param_frame, text="Hz").grid(row=0, column=3, padx=2)

        # On-time
        ttk.Label(param_frame, text="On-Time (µs):").grid(row=1, column=0,
                                                           sticky=tk.W, padx=4, pady=4)
        self.ontime_var = tk.IntVar(value=50)
        self.ontime_scale = ttk.Scale(param_frame, from_=1, to=500,
                                       variable=self.ontime_var, orient=tk.HORIZONTAL,
                                       length=400)
        self.ontime_scale.grid(row=1, column=1, padx=4, pady=4, sticky=tk.EW)
        self.ontime_entry = ttk.Entry(param_frame, textvariable=self.ontime_var, width=8)
        self.ontime_entry.grid(row=1, column=2, padx=4, pady=4)
        ttk.Label(param_frame, text="µs").grid(row=1, column=3, padx=2)

        param_frame.columnconfigure(1, weight=1)

        # Bind slider/entry changes to live-update when firing
        self.freq_var.trace_add("write", self._on_param_changed)
        self.ontime_var.trace_add("write", self._on_param_changed)

        # Fire / Stop buttons
        btn_frame = ttk.Frame(tab)
        btn_frame.pack(fill=tk.X, pady=8)

        self.fire_btn = ttk.Button(btn_frame, text="FIRE", command=self._fire,
                                    style="Accent.TButton")
        self.fire_btn.pack(side=tk.LEFT, padx=8, ipadx=20, ipady=8)

        self.stop_btn = ttk.Button(btn_frame, text="STOP Selected Coil",
                                    command=self._stop_coil)
        self.stop_btn.pack(side=tk.LEFT, padx=8, ipadx=10, ipady=8)

        self.stopall_btn = ttk.Button(btn_frame, text="STOP ALL",
                                       command=self._stopall)
        self.stopall_btn.pack(side=tk.LEFT, padx=8, ipadx=10, ipady=8)

        # Live coil status
        status_frame = ttk.LabelFrame(tab, text="Coil Status (live)", padding=8)
        status_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 4))

        cols = ("Coil", "Timer", "Pin", "Enabled", "Voices", "Pulses", "Active")
        self.coil_tree = ttk.Treeview(status_frame, columns=cols, show="headings",
                                       height=6)
        for c in cols:
            self.coil_tree.heading(c, text=c)
            self.coil_tree.column(c, width=90, anchor=tk.CENTER)
        self.coil_tree.pack(fill=tk.BOTH, expand=True)

        timers = ["TIM1", "TIM4", "TIM9", "TIM10", "TIM11", "TIM13"]
        pins = ["PE9", "PD12", "PE5", "PF6", "PF7", "PF8"]
        for i in range(NUM_COILS):
            self.coil_tree.insert("", tk.END, iid=str(i),
                                  values=(f"Coil {i+1}", timers[i], pins[i],
                                          "-", "-", "-", "-"))

    # -- MIDI Tab --
    def _build_midi_tab(self, nb):
        tab = ttk.Frame(nb, padding=10)
        nb.add(tab, text="  MIDI Player  ")

        # File selection
        file_frame = ttk.LabelFrame(tab, text="MIDI File", padding=8)
        file_frame.pack(fill=tk.X, pady=(0, 8))

        self.midi_file_var = tk.StringVar(value="(no file loaded)")
        ttk.Label(file_frame, textvariable=self.midi_file_var,
                  width=60).pack(side=tk.LEFT, padx=4)
        ttk.Button(file_frame, text="Browse...",
                   command=self._browse_midi).pack(side=tk.RIGHT, padx=4)

        self.midi_filepath = None

        # MIDI Port
        port_frame = ttk.LabelFrame(tab, text="MIDI Output Port", padding=8)
        port_frame.pack(fill=tk.X, pady=(0, 8))

        self.midi_port_var = tk.StringVar(value="(auto-detect)")
        self.midi_port_combo = ttk.Combobox(port_frame, textvariable=self.midi_port_var,
                                             width=40)
        self.midi_port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(port_frame, text="Refresh",
                   command=self._refresh_midi_ports).pack(side=tk.LEFT, padx=4)
        self._refresh_midi_ports()

        # Playback controls
        ctrl_frame = ttk.Frame(tab)
        ctrl_frame.pack(fill=tk.X, pady=8)

        self.play_btn = ttk.Button(ctrl_frame, text="Play",
                                    command=self._play_midi)
        self.play_btn.pack(side=tk.LEFT, padx=8, ipadx=20, ipady=8)

        self.stop_midi_btn = ttk.Button(ctrl_frame, text="Stop Playback",
                                         command=self._stop_midi)
        self.stop_midi_btn.pack(side=tk.LEFT, padx=8, ipadx=10, ipady=8)

        # Progress
        prog_frame = ttk.Frame(tab)
        prog_frame.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(prog_frame, text="Progress:").pack(side=tk.LEFT, padx=4)
        self.midi_progress = ttk.Progressbar(prog_frame, mode="determinate",
                                              length=500)
        self.midi_progress.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.midi_status_label = ttk.Label(prog_frame, text="Idle")
        self.midi_status_label.pack(side=tk.LEFT, padx=4)

        # Info
        info = ttk.Label(tab, text=(
            "Note: MIDI playback sends notes through the device's USB MIDI interface.\n"
            "Ensure channel routing is configured (see Routing tab).\n"
            "By default, MIDI channels 0-5 map to coils 1-6.\n"
            "For single-coil testing, use ROUTEALL to send all channels to coil 1."
        ), justify=tk.LEFT, foreground="gray")
        info.pack(fill=tk.X, padx=8, pady=16)

    # -- Routing Tab --
    def _build_routing_tab(self, nb):
        tab = ttk.Frame(nb, padding=10)
        nb.add(tab, text="  Routing  ")

        ttk.Label(tab, text="MIDI Channel → Coil Routing",
                  font=("", 11, "bold")).pack(anchor=tk.W, pady=(0, 8))

        # Quick actions
        quick_frame = ttk.Frame(tab)
        quick_frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Button(quick_frame, text="Reset to Defaults",
                   command=self._route_reset).pack(side=tk.LEFT, padx=4)
        ttk.Button(quick_frame, text="Route ALL → Coil 1",
                   command=lambda: self._route_all(0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(quick_frame, text="Refresh Table",
                   command=self._route_refresh).pack(side=tk.LEFT, padx=4)

        # Routing table
        rt_frame = ttk.Frame(tab)
        rt_frame.pack(fill=tk.BOTH, expand=True)

        self.route_combos = []
        coil_options = [f"Coil {c+1}" for c in range(NUM_COILS)] + ["Unmapped"]

        for ch in range(16):
            row = ch // 4
            col = (ch % 4) * 2
            ttk.Label(rt_frame, text=f"CH {ch}:").grid(
                row=row, column=col, padx=(8, 2), pady=4, sticky=tk.E)
            var = tk.StringVar(value=f"Coil {ch+1}" if ch < NUM_COILS else "Unmapped")
            combo = ttk.Combobox(rt_frame, textvariable=var, values=coil_options,
                                  width=10, state="readonly")
            combo.grid(row=row, column=col + 1, padx=(0, 12), pady=4, sticky=tk.W)
            self.route_combos.append((ch, var, combo))

        ttk.Button(tab, text="Apply Routing",
                   command=self._route_apply).pack(pady=10)

    # -- Limits Tab --
    def _build_limits_tab(self, nb):
        tab = ttk.Frame(nb, padding=10)
        nb.add(tab, text="  Safety Limits  ")

        ttk.Label(tab, text="Per-Coil Safety Parameters",
                  font=("", 11, "bold")).pack(anchor=tk.W, pady=(0, 8))

        ttk.Label(tab, text=(
            "Absolute caps: Max On-Time = 500µs, Max Duty = 5.0%, Min Off-Time = 20µs"
        ), foreground="gray").pack(anchor=tk.W, pady=(0, 8))

        lim_frame = ttk.Frame(tab)
        lim_frame.pack(fill=tk.X)

        headers = ["Coil", "Max On-Time (µs)", "Max Duty (‰)", "Min Off-Time (µs)", ""]
        for i, h in enumerate(headers):
            ttk.Label(lim_frame, text=h, font=("", 9, "bold")).grid(
                row=0, column=i, padx=6, pady=4)

        self.limit_entries = []
        for c in range(NUM_COILS):
            ttk.Label(lim_frame, text=f"Coil {c+1}").grid(
                row=c + 1, column=0, padx=6, pady=2)

            ot_var = tk.IntVar(value=50)
            ot_entry = ttk.Entry(lim_frame, textvariable=ot_var, width=8)
            ot_entry.grid(row=c + 1, column=1, padx=6, pady=2)

            duty_var = tk.IntVar(value=50)
            duty_entry = ttk.Entry(lim_frame, textvariable=duty_var, width=8)
            duty_entry.grid(row=c + 1, column=2, padx=6, pady=2)

            off_var = tk.IntVar(value=10)
            off_entry = ttk.Entry(lim_frame, textvariable=off_var, width=8)
            off_entry.grid(row=c + 1, column=3, padx=6, pady=2)

            ttk.Button(lim_frame, text="Set",
                       command=lambda c=c: self._set_limits(c)).grid(
                row=c + 1, column=4, padx=6, pady=2)

            self.limit_entries.append((ot_var, duty_var, off_var))

        btn_frame = ttk.Frame(tab)
        btn_frame.pack(fill=tk.X, pady=10)
        ttk.Button(btn_frame, text="Set All",
                   command=self._set_all_limits).pack(side=tk.LEFT, padx=8)
        ttk.Button(btn_frame, text="Read All from Device",
                   command=self._read_all_limits).pack(side=tk.LEFT, padx=8)

    # -- Console Tab --
    def _build_console_tab(self, nb):
        tab = ttk.Frame(nb, padding=10)
        nb.add(tab, text="  Console  ")

        ttk.Label(tab, text="Raw CDC Command Console",
                  font=("", 11, "bold")).pack(anchor=tk.W, pady=(0, 4))

        self.console_log = scrolledtext.ScrolledText(tab, height=20, width=80,
                                                      font=("Consolas", 10),
                                                      state=tk.DISABLED)
        self.console_log.pack(fill=tk.BOTH, expand=True, pady=(0, 8))

        cmd_frame = ttk.Frame(tab)
        cmd_frame.pack(fill=tk.X)

        self.cmd_var = tk.StringVar()
        self.cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var,
                                    font=("Consolas", 10))
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.cmd_entry.bind("<Return>", lambda e: self._send_console_cmd())

        ttk.Button(cmd_frame, text="Send",
                   command=self._send_console_cmd).pack(side=tk.LEFT)

    # -----------------------------------------------------------------------
    #  Connection
    # -----------------------------------------------------------------------
    def _refresh_ports(self):
        ports = DeviceLink.list_ports()
        auto = DeviceLink.find_port()
        self.port_combo["values"] = ports
        if auto and auto in ports:
            self.port_var.set(auto)
        elif ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.dev.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "No port selected")
            return
        try:
            self.dev.connect(port)
            resp = self.dev.send("PING")
            if resp != "PONG":
                self.dev.disconnect()
                messagebox.showerror("Error", f"Device not responding (got: '{resp}')")
                return

            ver = self.dev.send("VERSION")
            self.fw_label.config(text=ver)
            self.conn_status.config(text=f"Connected ({port})", foreground="green")
            self.connect_btn.config(text="Disconnect")

            self._log(f"Connected to {port}")
            self._log(f"Firmware: {ver}")

            self.polling = True
            self._poll_status()
            self._route_refresh()
            self._read_all_limits()

        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def _disconnect(self):
        self.polling = False
        self._coil_firing = False
        self.dev.disconnect()
        self.conn_status.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.fw_label.config(text="")
        self._log("Disconnected")

    # -----------------------------------------------------------------------
    #  Status Polling
    # -----------------------------------------------------------------------
    def _poll_status(self):
        if not self.polling or not self.dev.connected:
            return

        # Poll COILS? for tree view
        data = self.dev.send_json("COILS?")
        if data and "coils" in data:
            timers = ["TIM1", "TIM4", "TIM9", "TIM10", "TIM11", "TIM13"]
            pins = ["PE9", "PD12", "PE5", "PF6", "PF7", "PF8"]
            for i, c in enumerate(data["coils"]):
                self.coil_tree.item(str(i), values=(
                    f"Coil {i+1}", timers[i], pins[i],
                    "Yes" if c.get("en", 1) else "No",
                    c.get("voices", "-"),
                    c.get("pulses", "-"),
                    "FIRING" if c.get("active", 0) else "idle"
                ))

        # E-Stop
        estop_resp = self.dev.send("ESTOP?")
        if "ESTOP=1" in estop_resp:
            self.estop_label.config(text="E-STOP ACTIVE", foreground="red",
                                    font=("", 10, "bold"))
        elif "ESTOP=0" in estop_resp:
            self.estop_label.config(text="E-Stop: OK", foreground="green")

        # Scheduler
        sched = self.dev.send_json("SCHED?")
        if sched:
            running = "Running" if sched.get("running") else "Stopped"
            ticks = sched.get("ticks", "?")
            self.sched_label.config(text=f"Scheduler: {running} | Ticks: {ticks}")

        # MIDI progress
        if self.player.playing:
            self.midi_progress["value"] = self.player.progress * 100
            self.midi_status_label.config(text=f"Playing: {self.player.filename}")
        else:
            if self.midi_progress["value"] >= 99:
                self.midi_status_label.config(text="Done")

        self.root.after(POLL_INTERVAL_MS, self._poll_status)

    # -----------------------------------------------------------------------
    #  Pulse Tab Actions
    # -----------------------------------------------------------------------
    def _fire(self):
        if not self.dev.connected:
            messagebox.showwarning("Not Connected", "Connect to the device first.")
            return
        coil = self.coil_var.get()
        try:
            freq = int(self.freq_var.get())
            ontime = int(self.ontime_var.get())
        except (ValueError, tk.TclError):
            return
        if freq <= 0 or ontime <= 0:
            messagebox.showwarning("Invalid", "Frequency and on-time must be > 0")
            return
        # Stop existing tone first so FIRE overwrites cleanly
        if self._coil_firing:
            self.dev.send(f"STOP {coil}")
        cmd = f"FIRE {coil} {freq} {ontime}"
        resp = self.dev.send(cmd)
        self._log(f"> {cmd}")
        self._log(f"< {resp}")
        self._coil_firing = True
        self.status_label.config(text=f"Fired Coil {coil+1}: {freq} Hz, {ontime} µs")

    def _stop_coil(self):
        if not self.dev.connected:
            return
        coil = self.coil_var.get()
        cmd = f"STOP {coil}"
        resp = self.dev.send(cmd)
        self._log(f"> {cmd}")
        self._log(f"< {resp}")
        self._coil_firing = False
        self.status_label.config(text=f"Stopped Coil {coil+1}")

    def _stopall(self):
        if not self.dev.connected:
            return
        resp = self.dev.send("STOPALL")
        self._log("> STOPALL")
        self._log(f"< {resp}")
        self._coil_firing = False
        self.status_label.config(text="All coils stopped")

    def _on_param_changed(self, *_args):
        """Called when freq or ontime slider/entry changes.

        If a coil is actively firing, schedule a re-FIRE after a short
        debounce delay so the new parameters take effect immediately
        without spamming the device on every pixel of slider movement.
        """
        if not self._coil_firing or not self.dev.connected:
            return
        # Cancel any previously scheduled update
        if self._update_pending is not None:
            self.root.after_cancel(self._update_pending)
        # Debounce: wait 100 ms after the last change before sending
        self._update_pending = self.root.after(100, self._live_update)

    def _live_update(self):
        """Re-send FIRE with current slider values (debounced)."""
        self._update_pending = None
        if not self._coil_firing or not self.dev.connected:
            return
        coil = self.coil_var.get()
        try:
            freq = int(self.freq_var.get())
            ontime = int(self.ontime_var.get())
        except (ValueError, tk.TclError):
            return
        if freq <= 0 or ontime <= 0:
            return
        # STOP + FIRE to cleanly replace the active tone
        self.dev.send(f"STOP {coil}")
        cmd = f"FIRE {coil} {freq} {ontime}"
        resp = self.dev.send(cmd)
        self.status_label.config(text=f"Live: Coil {coil+1}: {freq} Hz, {ontime} µs")

    # -----------------------------------------------------------------------
    #  MIDI Tab Actions
    # -----------------------------------------------------------------------
    def _browse_midi(self):
        fp = filedialog.askopenfilename(
            title="Select MIDI File",
            filetypes=[("MIDI files", "*.mid *.midi"), ("All files", "*.*")]
        )
        if fp:
            self.midi_filepath = fp
            self.midi_file_var.set(os.path.basename(fp))
            self._log(f"Loaded MIDI: {fp}")

    def _refresh_midi_ports(self):
        try:
            import mido
            ports, best = MidiPlayer.list_midi_ports()
            if not ports:
                # Show diagnostic: try to figure out why
                diag = "(no MIDI ports found"
                try:
                    import rtmidi
                    api = rtmidi.RtMidi.get_compiled_api()
                    diag += f" — rtmidi APIs: {api}"
                except Exception:
                    pass
                diag += ")"
                self.midi_port_combo["values"] = [diag]
                self.midi_port_var.set(diag)
                self._log(f"MIDI refresh: {diag}")
                self._log("Tip: Ensure the device is connected and recognized ")
                self._log("     by Windows as a MIDI device (check Device Manager ")
                self._log("     under 'Sound, video and game controllers').")
                return

            # Allow the user to also type a custom name
            self.midi_port_combo["state"] = "normal"
            self.midi_port_combo["values"] = ports
            if best:
                self.midi_port_var.set(best)
            else:
                self.midi_port_var.set(ports[0])

            port_list = ', '.join(ports)
            self._log(f"MIDI ports found: {port_list}")
            if best:
                self._log(f"Auto-selected: {best}")
        except ImportError:
            self.midi_port_combo["values"] = ["(mido not installed — pip install mido python-rtmidi)"]
            self.midi_port_var.set("(mido not installed)")

    def _play_midi(self):
        if not self.midi_filepath:
            messagebox.showwarning("No File", "Select a MIDI file first.")
            return
        if self.player.playing:
            messagebox.showinfo("Playing", "Stop current playback first.")
            return

        midi_port = self.midi_port_var.get()
        if not midi_port or "not installed" in midi_port:
            messagebox.showerror("Error", "No MIDI port available. Install mido + python-rtmidi.")
            return

        self.midi_progress["value"] = 0
        self.midi_status_label.config(text="Starting...")
        self._log(f"Playing MIDI: {self.midi_filepath} → {midi_port}")

        def on_finish():
            self.root.after(0, lambda: self.midi_status_label.config(text="Done"))
            self.root.after(0, lambda: self._log("MIDI playback finished"))

        self.player.play(self.midi_filepath, midi_port, on_finish=on_finish)

    def _stop_midi(self):
        if self.player.playing:
            self.player.stop()
            # Also tell the device to stop all tones
            if self.dev.connected:
                self.dev.send("STOPALL")
            self.midi_status_label.config(text="Stopped")
            self._log("MIDI playback stopped")
        self.midi_progress["value"] = 0

    # -----------------------------------------------------------------------
    #  Routing Tab Actions
    # -----------------------------------------------------------------------
    def _route_refresh(self):
        if not self.dev.connected:
            return
        data = self.dev.send_json("ROUTE?")
        if data and "routes" in data:
            for entry in data["routes"]:
                ch = entry["ch"]
                coil = entry["coil"]
                if ch < len(self.route_combos):
                    _, var, _ = self.route_combos[ch]
                    if coil < NUM_COILS:
                        var.set(f"Coil {coil+1}")
                    else:
                        var.set("Unmapped")

    def _route_apply(self):
        if not self.dev.connected:
            messagebox.showwarning("Not Connected", "Connect first.")
            return
        for ch, var, _ in self.route_combos:
            val = var.get()
            if val == "Unmapped":
                coil = 255
            else:
                coil = int(val.split()[1]) - 1  # "Coil 1" → 0
            self.dev.send(f"ROUTE {ch} {coil}")
        self._log("Routing applied")
        self.status_label.config(text="Routing updated")

    def _route_reset(self):
        if not self.dev.connected:
            return
        resp = self.dev.send("ROUTERESET")
        self._log(f"> ROUTERESET\n< {resp}")
        self._route_refresh()
        self.status_label.config(text="Routing reset to defaults")

    def _route_all(self, coil):
        if not self.dev.connected:
            return
        resp = self.dev.send(f"ROUTEALL {coil}")
        self._log(f"> ROUTEALL {coil}\n< {resp}")
        self._route_refresh()
        self.status_label.config(text=f"All channels → Coil {coil+1}")

    # -----------------------------------------------------------------------
    #  Limits Tab Actions
    # -----------------------------------------------------------------------
    def _set_limits(self, coil):
        if not self.dev.connected:
            return
        ot, duty, off = self.limit_entries[coil]
        cmd = f"LIMITS {coil} {ot.get()} {duty.get()} {off.get()}"
        resp = self.dev.send(cmd)
        self._log(f"> {cmd}\n< {resp}")

    def _set_all_limits(self):
        if not self.dev.connected:
            messagebox.showwarning("Not Connected", "Connect first.")
            return
        for c in range(NUM_COILS):
            self._set_limits(c)
        self.status_label.config(text="All limits updated")

    def _read_all_limits(self):
        if not self.dev.connected:
            return
        for c in range(NUM_COILS):
            resp = self.dev.send(f"LIMITS? {c}")
            # Parse: "LIMITS coil=N max_ontime=N duty_permil=N min_offtime=N"
            if "max_ontime=" in resp:
                try:
                    parts = resp.split()
                    vals = {}
                    for p in parts:
                        if "=" in p:
                            k, v = p.split("=")
                            vals[k] = int(v)
                    ot_var, duty_var, off_var = self.limit_entries[c]
                    if "max_ontime" in vals:
                        ot_var.set(vals["max_ontime"])
                    if "duty_permil" in vals:
                        duty_var.set(vals["duty_permil"])
                    if "min_offtime" in vals:
                        off_var.set(vals["min_offtime"])
                except (ValueError, KeyError):
                    pass

    # -----------------------------------------------------------------------
    #  Console Tab Actions
    # -----------------------------------------------------------------------
    def _send_console_cmd(self):
        if not self.dev.connected:
            messagebox.showwarning("Not Connected", "Connect first.")
            return
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        self._log(f"> {cmd}")
        resp = self.dev.send(cmd)
        self._log(f"< {resp}")
        self.cmd_var.set("")

    def _log(self, text):
        if not hasattr(self, "console_log"):
            return
        self.console_log.config(state=tk.NORMAL)
        self.console_log.insert(tk.END, text + "\n")
        self.console_log.see(tk.END)
        self.console_log.config(state=tk.DISABLED)


# ---------------------------------------------------------------------------
#  Entry Point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=APP_TITLE)
    parser.add_argument("--port", help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    root = tk.Tk()

    # Try to set a modern theme
    try:
        style = ttk.Style()
        available = style.theme_names()
        for pref in ("vista", "clam", "alt", "default"):
            if pref in available:
                style.theme_use(pref)
                break
    except Exception:
        pass

    port = args.port or DeviceLink.find_port()
    app = TCInterrupterGUI(root, initial_port=port)

    # Handle window close
    def on_close():
        app.polling = False
        if app.player.playing:
            app.player.stop()
        if app.dev.connected:
            try:
                app.dev.send("STOPALL")
            except Exception:
                pass
            app.dev.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
