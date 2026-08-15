#!/usr/bin/env python3
"""Desktop control panel for the RP2040 22-servo board.

Run this on the Raspberry Pi Desktop, from the repository root:
    python3 servo_gui.py
"""

import tkinter as tk
from tkinter import ttk

from smbus2 import SMBus

import rpi_master as servo_hat


REFRESH_MS = 1000


class ServoHatApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RP2040 Servo Hat")
        self.root.geometry("1120x760")
        self.bus = SMBus(servo_hat.I2C_BUS)
        self.auto_refresh = tk.BooleanVar(value=True)
        self.all_pulse = tk.IntVar(value=1500)
        self.voltage_text = tk.StringVar(value="Bus voltage: -- V")
        self.status_text = tk.StringVar(value="Connecting…")
        self.servo_values = []

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh_telemetry()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        header = ttk.Frame(main)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(header, text="RP2040 Servo Hat", font=("TkDefaultFont", 18, "bold")).pack(side="left")
        ttk.Label(header, textvariable=self.voltage_text, font=("TkDefaultFont", 14, "bold")).pack(side="right")

        tools = ttk.LabelFrame(main, text="Global controls", padding=8)
        tools.pack(fill="x", pady=(0, 10))
        ttk.Label(tools, text="All servos (µs):").pack(side="left")
        ttk.Spinbox(tools, from_=1000, to=2000, increment=10, width=7, textvariable=self.all_pulse).pack(side="left", padx=(6, 6))
        ttk.Button(tools, text="Set all", command=self.set_all).pack(side="left")
        ttk.Button(tools, text="Safe: all 1000 µs", command=self.safe_position).pack(side="left", padx=(8, 0))
        ttk.Button(tools, text="Refresh now", command=self.refresh_telemetry).pack(side="right")
        ttk.Checkbutton(tools, text="Auto-refresh", variable=self.auto_refresh).pack(side="right", padx=10)

        body = ttk.PanedWindow(main, orient="horizontal")
        body.pack(fill="both", expand=True)

        servo_box = ttk.LabelFrame(body, text="Servo controls", padding=6)
        telemetry_box = ttk.LabelFrame(body, text="Live current readings", padding=6)
        body.add(servo_box, weight=3)
        body.add(telemetry_box, weight=2)
        self._build_servo_controls(servo_box)
        self._build_telemetry(telemetry_box)

        ttk.Label(main, textvariable=self.status_text, anchor="w").pack(fill="x", pady=(8, 0))

    def _build_servo_controls(self, parent):
        canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)
        scroll_frame.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for text, column in (("Servo", 0), ("GPIO", 1), ("Pulse width", 2), ("Action", 3)):
            ttk.Label(scroll_frame, text=text, font=("TkDefaultFont", 9, "bold")).grid(row=0, column=column, sticky="w", padx=4, pady=(0, 4))

        for servo, gpio in enumerate(servo_hat.SERVO_GPIO):
            value = tk.IntVar(value=1000)
            self.servo_values.append(value)
            row = servo + 1
            ttk.Label(scroll_frame, text=f"S{servo:02d}").grid(row=row, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(scroll_frame, text=f"GP{gpio}").grid(row=row, column=1, sticky="w", padx=4, pady=2)
            scale = ttk.Scale(scroll_frame, from_=1000, to=2000, variable=value, length=190)
            scale.grid(row=row, column=2, sticky="ew", padx=4, pady=2)
            scale.bind("<ButtonRelease-1>", lambda _event, index=servo: self.set_servo(index))
            ttk.Spinbox(scroll_frame, from_=1000, to=2000, increment=10, width=6, textvariable=value).grid(row=row, column=3, sticky="w", padx=(4, 2))
            ttk.Button(scroll_frame, text="Set", width=5, command=lambda index=servo: self.set_servo(index)).grid(row=row, column=4, sticky="w", padx=(0, 4))

        scroll_frame.columnconfigure(2, weight=1)

    def _build_telemetry(self, parent):
        columns = ("servo", "gpio", "raw", "amps")
        self.current_tree = ttk.Treeview(parent, columns=columns, show="headings", height=16)
        for name, text, width in (
            ("servo", "Servo", 62),
            ("gpio", "GPIO", 62),
            ("raw", "Raw ADC", 82),
            ("amps", "Current (A)", 100),
        ):
            self.current_tree.heading(name, text=text)
            self.current_tree.column(name, width=width, anchor="center")
        self.current_tree.pack(fill="both", expand=True)
        ttk.Label(parent, text="No current ADC: S00, S05, S10, S15, S20, S21", wraplength=360).pack(anchor="w", pady=(8, 0))

    def set_servo(self, servo):
        try:
            pulse = int(self.servo_values[servo].get())
            servo_hat.set_servo(self.bus, servo, pulse)
            self.status_text.set(f"Set S{servo:02d} to {servo_hat.clamp_pulse(pulse)} µs")
        except (OSError, RuntimeError, ValueError, tk.TclError) as error:
            self.status_text.set(f"Servo command failed: {error}")

    def set_all(self):
        try:
            pulse = int(self.all_pulse.get())
            servo_hat.set_all_servos(self.bus, pulse)
            pulse = servo_hat.clamp_pulse(pulse)
            for value in self.servo_values:
                value.set(pulse)
            self.status_text.set(f"Set all servos to {pulse} µs")
        except (OSError, RuntimeError, ValueError, tk.TclError) as error:
            self.status_text.set(f"All-servo command failed: {error}")

    def safe_position(self):
        try:
            servo_hat.safe_position(self.bus)
            for value in self.servo_values:
                value.set(1000)
            self.status_text.set("Safe position sent: all servos at 1000 µs")
        except (OSError, RuntimeError) as error:
            self.status_text.set(f"Safe-position command failed: {error}")

    def refresh_telemetry(self):
        try:
            bus_voltage, divider_voltage, _raw = servo_hat.read_mcp3425_bus_voltage(self.bus)
            sequence, raw_values = servo_hat.read_servo_adc(self.bus)
            self.voltage_text.set(f"Bus voltage: {bus_voltage:.3f} V")
            for item in self.current_tree.get_children():
                self.current_tree.delete(item)
            for servo, raw in zip(servo_hat.ADC_SERVO_MAP, raw_values):
                gpio = servo_hat.SERVO_GPIO[servo]
                amps = servo_hat.current_from_raw(raw)
                self.current_tree.insert("", "end", values=(f"S{servo:02d}", f"GP{gpio}", raw, f"{amps:.3f}"))
            self.status_text.set(f"Sample {sequence}: MCP3425 input {divider_voltage:.5f} V")
        except (OSError, RuntimeError, TimeoutError) as error:
            self.status_text.set(f"Telemetry read failed: {error}")
        finally:
            if self.auto_refresh.get():
                self.root.after(REFRESH_MS, self.refresh_telemetry)

    def close(self):
        self.bus.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    ServoHatApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
