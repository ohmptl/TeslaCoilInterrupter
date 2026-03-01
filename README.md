# Project Coil / Tesla Coil MIDI Interrupter v2.0

A robust, lab-grade platform that transforms standard MIDI music from a modern DAW (e.g., Ableton) into precisely-timed interrupter signals for up to **6 DRSSTC Tesla coils** via isolated fiber optic outputs. Designed for repeated academic demos, coil characterization, and live musical performances in a high-voltage research lab.

## Key Features
- **USB Composite Device**: Native MIDI 1.0 (16 channels for DAWs) + CDC Virtual COM Port (control, status, telemetry) via USB-C.
- **Microsecond Precision Polyphony**: Powered by an STM32F407ZGT6 running at 168 MHz with a custom Hybrid Scheduler utilizing One-Pulse Mode (OPM) hardware timers.
- **6-Coil Array Support**: Six independent fiber-optic gate signals allowing true polyphonic output.
- **Hardware-Level Safety**: Strict hardware-enforced pulse-width safety limits (via OPM timers) prevent IGBT destruction even if the MCU stalls. Includes external E-Stop and Hardware Watchdog (IWDG).
- **Companion PC Software Suite**: Features a .NET 8 WinUI GUI dashboard for real-time control and an offline MIDI processor for compiling safe, optimized MIDI profiles.

---

## System Architecture

### 1. Hardware Overview
* **Microcontroller**: STM32F407ZGT6 built on an STM32-E407 Development Board.
* **USB Interface**: USB-C bus-powered (320 mA max peak), enumerating as a Composite Device.
* **Outputs**: 6x HFBR-1521Z/2521Z Fiber Optic Tx drivers, physically isolated from the communication ports to minimize EMI cross-talk.
* **Display**: 0.96" OLED SSD1306 (I2C) for status and fault monitoring.
* **Safety Components**: Latching front-panel Emergency Stop that directly cuts 5V power to the fiber drivers, accompanied by an independent GPIO interrupt line to halt all TIM instances in firmware.

### 2. Firmware (STM32CubeMX + HAL)
The core pulse-generation engine uses a **Hybrid Scheduler + One-Pulse Mode (OPM)** architecture:
* **The Master Timebase (`TIM7`)**: Runs at a 1 µs tick rate, acting as the software sequencer. It mathematically mixes all active MIDI notes per coil based on the channel routing set by the PC.
* **Per-Coil Output Timers (`TIM1`, `TIM4`, `TIM9`, `TIM10`, `TIM11`, `TIM13`)**: Configured strictly in **One-Pulse Mode (OPM)**. When a coil pulse is required, the sequencer loads the calculated pulse width into the timer's registers and arms it. The hardware timer fires exactly *one* precision pulse and shuts itself down. This guarantees the maximum actual on-time is rigidly enforced by silicon.
* **Dual-Layer Priority Protocol**: The CDC Virtual COM Port (handling setup, telemetry, and manual limits) is strictly prioritized over the MIDI stream to ensure commands are never blocked during heavy musical floods.

### 3. PC Software Suite (.NET 8 WinUI)
* **Offline MIDI Processor**: An application component that loads raw `.mid` files, applies user-defined coil capabilities (JSON profiles dictating max duty, max on-time, frequency ranges), and exports an optimized, hardware-safe MIDI file.
* **Live GUI & Dashboard**: Connects over the CDC endpoint to provide dynamic routing (e.g., "Channel 1 -> Coil 4"), real-time OLED status mirroring, fault log downloads, and variable parameter sliders (BPS/duty test sweeps) for Standalone Test Mode.

---

## Safety & EMI Hardening
Since the physical hardware operates directly adjacent to ultra-high-voltage arcs (1–2m safe distance limit), safety is absolute:
1. **Absolute Firmware Clamping**: All PC software-configured limits (max on-time, max duty, min off-time) are aggressively clamped at the firmware level *before* any timer registers are updated. 
2. **EMI Immunity**: 4-layer PCB design (signal/GND/power/GND) with physical separation of high-voltage output pins (Ports D, E, F) from communication pins (Ports A, B, C).
3. **Redundant Failsafes**: Watchdog timers provide a ~4-second hardware fail-safe. 
4. **Diagnostic Firehose**: Given the unreliability of USB in extreme EMI conditions, a continuous stream of un-killable telemetry logs is pushed out via `USART1`.

---

## Roadmap & Milestones
- [ ] **Milestone 1**: High-Priority USB Composite & Diagnostics Layer (CDC priorities, parser routing).
- [ ] **Milestone 2**: Single to Multi-Coil Engine Core (Implementation of the `TIM7` Hybrid OPM Scheduler).
- [ ] **Milestone 3**: Dynamic Routing, Standalone Test Mode, and Extreme MIDI Stress Testing.
- [ ] **Milestone 4**: The PC Software Suite (.NET 8 WinUI) (Full offline profiling and live dashboard).

## Credits
This project's architecture and baseline parameters are heavily inspired by the benchmark achievements of the [Syntherrupter](https://github.com/MMMZZZZ/Syntherrupter) platform in the realm of interrupter timers and continuous MIDI parsing.
