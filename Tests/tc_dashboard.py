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
QCW_NUM_CHANNELS = 3
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
        self.root.geometry("1000x800")
        self.root.minsize(900, 650)
        
        # Attempt to load sv_ttk for dark mode
        try:
            import sv_ttk
            try:
                sv_ttk.set_theme("dark")
            except Exception:
                sv_ttk.use_dark_theme()
            self.root.configure(bg="#1c1c1c")
        except ImportError:
            # Fallback custom styling if sv_ttk not available
            self.root.configure(bg="#2b2b2b")
            style = ttk.Style(self.root)
            if "clam" in style.theme_names():
                style.theme_use("clam")
            style.configure(".", background="#2b2b2b", foreground="#ffffff")
            style.configure("TFrame", background="#2b2b2b")
            style.configure("TLabelframe", background="#2b2b2b")
            style.configure("TLabelframe.Label", background="#2b2b2b", foreground="#ffffff")

        self.dev = DeviceLink()
        self.player = MidiPlayer()
        self.polling = False
        self._update_pending = None 
        
        self.current_tab_index = 0
        self._build_ui()

        if initial_port:
            self.port_var.set(initial_port)
            self.root.after(100, self._connect)

        self.polling = True
        self._poll_status()

    # -----------------------------------------------------------------------
    #  UI Construction
    # -----------------------------------------------------------------------
    def _build_ui(self):
        # -- Global Header --
        header_frame = ttk.Frame(self.root, padding=10)
        header_frame.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(header_frame, text="TC-Interrupter Dashboard", font=("Segoe UI", 16, "bold")).pack(side=tk.LEFT, padx=(0,20))

        ttk.Label(header_frame, text="Port:").pack(side=tk.LEFT, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(header_frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 4))
        self._refresh_ports()

        ttk.Button(header_frame, text="Refresh", command=self._refresh_ports, width=7).pack(side=tk.LEFT, padx=2)
        self.connect_btn = ttk.Button(header_frame, text="Connect", command=self._toggle_connect, width=10)
        self.connect_btn.pack(side=tk.LEFT, padx=2)
        
        self.conn_status = ttk.Label(header_frame, text="Disconnected", foreground="#ff4444")
        self.conn_status.pack(side=tk.LEFT, padx=10)

        # Giant Estop / Stop All button
        self.estop_btn = tk.Button(header_frame, text="STOP ALL / PAUSE", font=("Segoe UI", 11, "bold"), 
                                   bg="#d32f2f", fg="white", activebackground="#ff6659", activeforeground="white",
                                   command=self._stop_all, width=18, height=1)
        self.estop_btn.pack(side=tk.RIGHT, padx=5)

        # -- Main Notebook --
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab_change)

        self._build_pulse_studio()
        self._build_midi_studio()
        self._build_qcw_studio()
        self._build_settings_tab()

        # -- Footer Status Bar --
        footer = ttk.Frame(self.root, padding=5)
        footer.pack(side=tk.BOTTOM, fill=tk.X)
        self.fw_label = ttk.Label(footer, text="FW: Unknown")
        self.fw_label.pack(side=tk.LEFT)
        self.sys_status = ttk.Label(footer, text="SYS: OK")
        self.sys_status.pack(side=tk.RIGHT)

    def _build_pulse_studio(self):
        tab = ttk.Frame(self.nb, padding=10)
        self.nb.add(tab, text="  Pulse Studio  ")

        self.pulse_state = []
        for i in range(NUM_COILS):
            card = ttk.LabelFrame(tab, text=f"Coil {i+1}", padding=10)
            
            # Map Coil 1+2 to Column 0 (top/bottom), 3+4 to Column 1, 5+6 to Column 2
            r = i % 2
            c = i // 2
            card.grid(row=r, column=c, padx=10, pady=10, sticky="nsew")
            tab.grid_columnconfigure(c, weight=1)
            
            # On/Off Var
            en_var = tk.IntVar(value=0)
            ttk.Checkbutton(card, text="Enable Pulse Mode", variable=en_var, style="Switch.TCheckbutton" if "Switch.TCheckbutton" in ttk.Style().element_names() else "",
                            command=lambda c=i: self._on_pulse_enable(c)).pack(anchor=tk.W, pady=(0,8))

            # Freq
            f_frame = ttk.Frame(card)
            f_frame.pack(fill=tk.X, pady=2)
            ttk.Label(f_frame, text="Freq (Hz):").pack(side=tk.LEFT)
            fq_var = tk.IntVar(value=100)
            ttk.Entry(f_frame, textvariable=fq_var, width=8).pack(side=tk.RIGHT)
            ttk.Scale(f_frame, from_=1, to=2000, variable=fq_var, orient=tk.HORIZONTAL,
                      command=lambda v, var=fq_var: var.set(round(float(v)))).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
            
            # Ontime
            t_frame = ttk.Frame(card)
            t_frame.pack(fill=tk.X, pady=2)
            ttk.Label(t_frame, text="On (us):").pack(side=tk.LEFT)
            ot_var = tk.IntVar(value=50)
            ttk.Entry(t_frame, textvariable=ot_var, width=8).pack(side=tk.RIGHT)
            ttk.Scale(t_frame, from_=1, to=500, variable=ot_var, orient=tk.HORIZONTAL,
                      command=lambda v, var=ot_var: var.set(round(float(v)))).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
            
            self.pulse_state.append({"en": en_var, "fq": fq_var, "ot": ot_var})
            
            # Local Fire button
            ttk.Button(card, text="FIRE ONCE", command=lambda c=i: self._fire_pulse(c)).pack(fill=tk.X, pady=(10,0))
            
            # Auto-update when sliders move (if enabled)
            fq_var.trace_add("write", lambda *args, c=i: self._live_update_pulse(c))
            ot_var.trace_add("write", lambda *args, c=i: self._live_update_pulse(c))

    def _build_midi_studio(self):
        tab = ttk.Frame(self.nb, padding=10)
        self.nb.add(tab, text="  MIDI Studio  ")

        # Top control
        ctrl_frame = ttk.Frame(tab)
        ctrl_frame.pack(fill=tk.X, pady=(0, 15))
        
        self.midi_global_en = tk.IntVar(value=0)
        ttk.Checkbutton(ctrl_frame, text="Enable Master MIDI Playback (Locks all coils)", variable=self.midi_global_en, 
                        command=self._on_midi_global_enable).pack(side=tk.LEFT)
        
        # Grid of Coil assignments
        grid_frame = ttk.LabelFrame(tab, text="1-to-1 MIDI Channel Routing", padding=10)
        grid_frame.pack(fill=tk.BOTH, expand=True)

        for i in range(NUM_COILS):
            row = ttk.Frame(grid_frame)
            row.pack(fill=tk.X, pady=5)
            ttk.Label(row, text=f"• Coil {i+1} : Listening to MIDI Channel {i+1}").pack(side=tk.LEFT, padx=10)

        # Player
        player_frame = ttk.LabelFrame(tab, text="Playback Controls", padding=10)
        player_frame.pack(fill=tk.X, pady=(15, 0))

        top_p = ttk.Frame(player_frame)
        top_p.pack(fill=tk.X)
        self.midi_lbl = ttk.Label(top_p, text="No file selected")
        self.midi_lbl.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(top_p, text="Browse...", command=self._browse_midi).pack(side=tk.RIGHT, padx=5)
        
        bot_p = ttk.Frame(player_frame)
        bot_p.pack(fill=tk.X, pady=8)
        self.play_btn = ttk.Button(bot_p, text="▶ PLAY", command=self._play_midi, state=tk.DISABLED)
        self.play_btn.pack(side=tk.LEFT, padx=5)
        self.stop_btn = ttk.Button(bot_p, text="⏹ STOP", command=self._stop_midi, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.midi_progress = ttk.Progressbar(bot_p, orient=tk.HORIZONTAL, mode='determinate')
        self.midi_progress.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)

    def _build_qcw_studio(self):
        tab = ttk.Frame(self.nb, padding=10)
        self.nb.add(tab, text="  QCW Studio  ")
        
        self.qcw_vars = []
        for i in range(QCW_NUM_CHANNELS):
            ch = i + 1
            card = ttk.LabelFrame(tab, text=f"QCW Pair {ch} (Coils {2*i+1} & {2*i+2})", padding=10)
            card.grid(row=0, column=i, padx=10, pady=10, sticky="nsew")
            tab.grid_rowconfigure(0, weight=1)
            tab.grid_columnconfigure(i, weight=1)

            top = ttk.Frame(card)
            top.pack(fill=tk.X, pady=2)
            en_var = tk.IntVar(value=0)
            ttk.Checkbutton(top, text="Engage QCW Mode", variable=en_var, command=lambda c=ch, v=en_var: self._on_qcw_enable(c, v)).pack(side=tk.LEFT)

            # Envelope sliders
            env_frame = ttk.Frame(card)
            env_frame.pack(fill=tk.X, pady=5)
            
            vars_dict = {}
            labels = [("Tmin1 (%)", 10), ("Tmax (%)", 50), ("Tmin2 (%)", 0), 
                      ("Tramp1 (ms)", 15), ("Tramp2 (ms)", 10), ("Thold (ms)", 0)]
            
            for j, (lbl, default_val) in enumerate(labels):
                col = 0
                row_idx = j
                subf = ttk.Frame(env_frame)
                subf.grid(row=row_idx, column=col, padx=4, pady=2, sticky="ew")
                env_frame.grid_columnconfigure(col, weight=1)
                
                ttk.Label(subf, text=lbl).pack(side=tk.LEFT)
                v = tk.IntVar(value=default_val)
                scale_to = 100 if "%" in lbl else 300
                ttk.Entry(subf, textvariable=v, width=8).pack(side=tk.RIGHT)
                ttk.Scale(subf, from_=0, to=scale_to, variable=v, orient=tk.HORIZONTAL,
                          command=lambda val, var=v, idx=i: [var.set(round(float(val))), self._qcw_draw_envelope(idx)]).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=5)
                vars_dict[lbl.split()[0]] = v
                
                v.trace_add("write", lambda *args, idx=i: self._qcw_draw_envelope(idx))

            # Canvas Graph
            canvas = tk.Canvas(card, height=180, bg="#1e1e2e", highlightthickness=0)
            canvas.pack(fill=tk.BOTH, expand=True, pady=10)
            
            self.qcw_vars.append({"en": en_var, "params": vars_dict, "canvas": canvas})
            
            # Initial drawing
            self.root.after(200, lambda i=i: self._qcw_draw_envelope(i))
            
            # Fire
            btnf = ttk.Frame(card)
            btnf.pack(fill=tk.X, pady=10)
            
            f_btn = ttk.Button(btnf, text="SYNC CONFIG & FIRE", command=lambda c=ch, d=vars_dict: self._fire_qcw(c, d))
            f_btn.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, ipady=10)

    def _build_settings_tab(self):
        tab = ttk.Frame(self.nb, padding=10)
        self.nb.add(tab, text="  Settings / CLI  ")

        # Serial Console
        cons_frame = ttk.LabelFrame(tab, text="Raw CDC Console", padding=10)
        cons_frame.pack(fill=tk.BOTH, expand=True)

        self.console = scrolledtext.ScrolledText(cons_frame, wrap=tk.WORD, height=12, bg="#1e1e1e", fg="#00ff00", font=("Consolas", 10))
        self.console.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        self.console.bind("<Key>", lambda e: "break")

        entry_f = ttk.Frame(cons_frame)
        entry_f.pack(fill=tk.X)
        self.cmd_entry = ttk.Entry(entry_f)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cmd_entry.bind("<Return>", lambda e: self._send_cli())
        ttk.Button(entry_f, text="Send", command=self._send_cli).pack(side=tk.RIGHT, padx=5)

    # -----------------------------------------------------------------------
    #  Tab Lifecycle & State Management
    # -----------------------------------------------------------------------
    def _on_tab_change(self, event):
        idx = self.nb.index(self.nb.select())
        if self.current_tab_index == idx: return
        self.current_tab_index = idx
        
        if not self.dev.connected: return
        
        # Cleanly sever previous states by turning everything effectively OFF
        self.dev.send("STOPALL")
        for i in range(1, NUM_COILS+1):
            self.dev.send(f"DISABLE {i}")
            self.dev.send(f"MODE {i} 0")
            qcw_ch = ((i-1)//2) + 1
            self.dev.send(f"QCW_MODE {qcw_ch} 0")
            
        # Reset local GUI vars
        for state in self.pulse_state: state["en"].set(0)
        self.midi_global_en.set(0)
        for state in self.qcw_vars: state["en"].set(0)

    # -----------------------------------------------------------------------
    #  Pulse Handlers
    # -----------------------------------------------------------------------
    def _on_pulse_enable(self, coil_idx):
        if not self.dev.connected: return
        ch = coil_idx + 1
        en = self.pulse_state[coil_idx]["en"].get()
        if en:
            self.dev.send(f"ENABLE {ch}")
            self.dev.send(f"MODE {ch} 1")
            self.dev.send(f"STOP {ch}")
        else:
            self.dev.send(f"DISABLE {ch}")
            self.dev.send(f"MODE {ch} 0")

    def _live_update_pulse(self, coil_idx):
        pass

    def _fire_pulse(self, coil_idx):
        if not self.dev.connected: return
        ch = coil_idx + 1
        if not self.pulse_state[coil_idx]["en"].get():
            self.pulse_state[coil_idx]["en"].set(1)
            self._on_pulse_enable(coil_idx)
            
        fq = self.pulse_state[coil_idx]["fq"].get()
        ot = self.pulse_state[coil_idx]["ot"].get()
        self.dev.send(f"FIRE {ch} {fq} {ot}")

    # -----------------------------------------------------------------------
    #  MIDI Handlers
    # -----------------------------------------------------------------------
    def _on_midi_global_enable(self):
        if not self.dev.connected: return
        en = self.midi_global_en.get()
        if en:
            self.dev.send("ROUTERESET")
            for i in range(1, NUM_COILS+1):
                self.dev.send(f"ENABLE {i}")
                self.dev.send(f"MODE {i} 2")
        else:
            self.dev.send("STOPALL")
            for i in range(1, NUM_COILS+1):
                self.dev.send(f"DISABLE {i}")
                self.dev.send(f"MODE {i} 0")

    def _browse_midi(self):
        fp = filedialog.askopenfilename(filetypes=[("MIDI files", "*.mid")])
        if fp:
            self.player.filename = fp
            self.midi_lbl.config(text=os.path.basename(fp))
            self.play_btn.config(state=tk.NORMAL)

    def _play_midi(self):
        if not self.player.filename: return
        if not self.midi_global_en.get():
            self.midi_global_en.set(1)
            self._on_midi_global_enable()

        port_name = MidiPlayer.find_midi_port()
        if not port_name:
            messagebox.showerror("MIDI Error", "Could not identify the device's USB MIDI port.")
            return

        self.player.play(self.player.filename, port_name, on_finish=self._on_midi_finish)
        self.play_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

    def _stop_midi(self):
        self.player.stop()
        self.dev.send("STOPALL")
        self.play_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.midi_progress["value"] = 0

    def _on_midi_finish(self):
        self.play_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.midi_progress["value"] = 0

    # -----------------------------------------------------------------------
    #  QCW Handlers
    # -----------------------------------------------------------------------
    def _qcw_draw_envelope(self, idx):
        """Draw the trapezoidal QCW envelope on the preview canvas for pair idx."""
        if idx >= len(self.qcw_vars): return
        q_data = self.qcw_vars[idx]
        c = q_data["canvas"]
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        pad_x = 35
        pad_top = 20
        pad_bot = 35
        plot_w = w - 2 * pad_x
        plot_h = h - pad_top - pad_bot

        try:
            params = q_data["params"]
            tmin1  = int(params["Tmin1"].get())
            tmax_v = int(params["Tmax"].get())
            tmin2  = int(params["Tmin2"].get())
            tramp1 = max(int(params["Tramp1"].get()), 0)
            tramp2 = max(int(params["Tramp2"].get()), 0)
            thold  = max(int(params["Thold"].get()), 0)
        except (ValueError, tk.TclError):
            return

        total_ms = tramp1 + thold + tramp2
        if total_ms <= 0:
            total_ms = 1

        def x_of(ms):
            return pad_x + int(ms / total_ms * plot_w)

        def y_of(duty):
            return pad_top + plot_h - int(duty / 100 * plot_h)

        # Background grid lines (horizontal percentages)
        for pct in (0, 25, 50, 75, 100):
            y = y_of(pct)
            c.create_line(pad_x, y, w - pad_x, y, fill="#313244", dash=(2, 4))
            c.create_text(pad_x - 6, y, text=f"{pct}%",
                          fill="#6c7086", anchor=tk.E, font=("Segoe UI", 8))

        # Vertical guide lines for phases
        c.create_line(x_of(tramp1), y_of(0), x_of(tramp1), y_of(100), fill="#313244", dash=(2, 4))
        if thold > 0:
            c.create_line(x_of(tramp1+thold), y_of(0), x_of(tramp1+thold), y_of(100), fill="#313244", dash=(2, 4))

        # Envelope points
        pts = [
            (0,      tmin1),
            (tramp1, tmax_v),
            (tramp1 + thold, tmax_v),
            (total_ms, tmin2),
        ]

        coords = []
        for ms, duty in pts:
            coords.extend([x_of(ms), y_of(duty)])

        # Fill under curve (solid dark emerald)
        fill_coords = list(coords) + [x_of(total_ms), y_of(0), x_of(0), y_of(0)]
        c.create_polygon(fill_coords, fill="#1c3d31", outline="")

        # Envelope thick bright line
        c.create_line(coords, fill="#10b981", width=3)
        
        # Phase labels (centered in their respective blocks, drawn at bottom padding)
        lbl_y = h - 12
        mid_ramp1 = tramp1 / 2
        c.create_text(x_of(mid_ramp1), lbl_y, text=f"Ramp Up\n{tramp1}ms",
                      fill="#89b4fa", font=("Segoe UI", 8, "bold"), justify=tk.CENTER)
        if thold > 0:
            mid_hold = tramp1 + thold / 2
            c.create_text(x_of(mid_hold), lbl_y, text=f"Hold\n{thold}ms",
                          fill="#f9e2af", font=("Segoe UI", 8, "bold"), justify=tk.CENTER)
        mid_ramp2 = tramp1 + thold + tramp2 / 2
        c.create_text(x_of(mid_ramp2), lbl_y, text=f"Ramp Dn\n{tramp2}ms",
                      fill="#f38ba8", font=("Segoe UI", 8, "bold"), justify=tk.CENTER)

        # Dot markers
        for ms, duty in pts:
            cx, cy = x_of(ms), y_of(duty)
            c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                          fill="#1e1e2e", outline="#10b981", width=2)

    def _on_qcw_enable(self, qcw_ch, en_var):
        if not self.dev.connected: return
        en = en_var.get()
        c1 = (qcw_ch - 1) * 2 + 1
        c2 = c1 + 1
        if en:
            self.dev.send(f"MODE {c1} 3")
            self.dev.send(f"MODE {c2} 3")
            self.dev.send(f"QCW_MODE {qcw_ch} 1")
        else:
            self.dev.send(f"QCW_MODE {qcw_ch} 0")
            self.dev.send(f"MODE {c1} 0")
            self.dev.send(f"MODE {c2} 0")

    def _fire_qcw(self, qcw_ch, params):
        if not self.dev.connected: return
        en_var = self.qcw_vars[qcw_ch-1]["en"]
        if not en_var.get():
            en_var.set(1)
            self._on_qcw_enable(qcw_ch, en_var)

        t1 = params["Tmin1"].get() * 10
        tm = params["Tmax"].get() * 10
        t2 = params["Tmin2"].get() * 10
        r1 = params["Tramp1"].get()
        r2 = params["Tramp2"].get()
        h  = params["Thold"].get()
        
        self.dev.send(f"QCW_CONFIG {qcw_ch} {t1} {tm} {t2} {r1} {r2} {h}")
        self.dev.send(f"QCW_FIRE {qcw_ch}")

    # -----------------------------------------------------------------------
    #  Core Application Logic
    # -----------------------------------------------------------------------
    def _refresh_ports(self):
        ports = DeviceLink.list_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            for p in ports:
                if "TC-Interrupter" in p or "STM" in p or "USB" in p:
                    self.port_var.set(p)
                    break
            if not self.port_var.get():
                self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.dev.connected:
            self.dev.disconnect()
            self.connect_btn.config(text="Connect")
            self.conn_status.config(text="Disconnected", foreground="#ff4444")
            self.fw_label.config(text="FW: Unknown")
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port: return
        try:
            self.dev.connect(port)
            line = self.dev.send("VERSION")
            if "TC" in line:
                self.connect_btn.config(text="Disconnect")
                self.conn_status.config(text="Connected", foreground="#00ff00")
                self.fw_label.config(text=f"FW: {line.strip()}")
            else:
                raise IOError("Invalid VERSION response")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect to {port}\n{e}")
            self.dev.disconnect()

    def _stop_all(self):
        if self.player.playing:
            self._stop_midi()
        if self.dev.connected:
            self.dev.send("STOPALL")
            # Also abort QCW just in case
            for i in range(1, 4):
                self.dev.send(f"QCW_ABORT {i}")
            # Ensure safe visual state
            self.dev.send("ESTOP?")

    def _send_cli(self):
        cmd = self.cmd_entry.get().strip()
        if not cmd or not self.dev.connected: return
        self.cmd_entry.delete(0, tk.END)
        self.console.insert(tk.END, f"> {cmd}\n", "out")
        resp = self.dev.send(cmd)
        if resp:
            self.console.insert(tk.END, f"{resp}\n")
        self.console.see(tk.END)

    def _poll_status(self):
        if not self.polling: return
        if self.dev.connected:
            st = self.dev.send_json("STATUS")
            if st:
                es = st.get("estop", 0)
                if es:
                    self.sys_status.config(text="SYS: *E-STOP*", foreground="red")
                else:
                    self.sys_status.config(text="SYS: OK", foreground="#00ff00")
                    
        if self.player.playing:
            self.midi_progress["value"] = self.player.progress * 100

        self._update_pending = self.root.after(POLL_INTERVAL_MS, self._poll_status)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="COM port to use")
    args = parser.parse_args()

    root = tk.Tk()
    port = args.port or DeviceLink.find_port()
    app = TCInterrupterGUI(root, initial_port=port)

    def on_close():
        app.polling = False
        if app.player.playing: app.player.stop()
        if app.dev.connected:
            try: app.dev.send("STOPALL") 
            except: pass
            app.dev.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
