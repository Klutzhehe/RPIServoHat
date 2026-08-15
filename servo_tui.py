#!/usr/bin/env python3
"""Interactive Terminal User Interface (TUI) for RP2040 22-Servo Hat.

Provides high-speed, real-time servo control and live telemetry:
  - Adjust individual or all servo angles in real time
  - Live target angle & pulse width readout
  - Visual bargraph representation for each servo
  - Live current sensing for all 16 routed channels
  - Live MCP3425 bus voltage display
  - High refresh rate with persistent I2C bus

Controls:
  Up / Down (or k/j)     : Select servo (S00..S21)
  Left / Right (or h/l)  : Change angle by ±1°
  [ / ] (or PgUp/PgDn)   : Change angle by ±10°
  Home / End             : Jump to 0° / 180°
  0 .. 9                 : Set angle preset (0°, 20°, 40°, ..., 180°)
  a / A                  : Set ALL servos to selected angle
  s / S                  : Reset all servos to Safe/Starting position
  q / ESC                : Quit
"""

import curses
import sys
import time
from pathlib import Path

try:
    from smbus2 import SMBus
except ImportError:
    print("Error: smbus2 is required. Install with: pip install smbus2", file=sys.stderr)
    sys.exit(1)

import rpi_master as servo_hat


def make_bar(angle, width=12):
    """Generate ASCII bargraph slider e.g. [----|.....]"""
    ratio = max(0.0, min(1.0, angle / 180.0))
    pos = int(ratio * (width - 1))
    bar = ["."] * width
    for i in range(pos):
        bar[i] = "="
    bar[pos] = "|"
    return "[" + "".join(bar) + "]"


def run_tui(stdscr):
    # Curses setup
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(True)  # Non-blocking getch
    curses.use_default_colors()

    # Color pairs if available
    has_color = curses.has_colors()
    if has_color:
        curses.start_color()
        curses.init_pair(1, curses.COLOR_CYAN, -1)     # Header / Title
        curses.init_pair(2, curses.COLOR_GREEN, -1)    # Normal / Safe
        curses.init_pair(3, curses.COLOR_YELLOW, -1)   # Selected / Active
        curses.init_pair(4, curses.COLOR_RED, -1)      # Current / Warning
        curses.init_pair(5, curses.COLOR_MAGENTA, -1)  # Voltage

    selected = 0
    angles = [90.0] * servo_hat.SERVO_COUNT
    currents = [0.0] * servo_hat.SERVO_COUNT
    raw_adcs = [0] * servo_hat.SERVO_COUNT
    bus_voltage = 0.0
    status_msg = "Ready. Use Arrow keys or [ / ] to control servos."

    last_adc_read = 0.0
    last_mcp_read = 0.0
    last_target_read = 0.0
    frame_count = 0
    fps = 0.0
    fps_timer = time.monotonic()

    with SMBus(servo_hat.I2C_BUS) as bus:
        # Initial read of targets
        try:
            initial_targets = servo_hat.read_servo_targets(bus)
            for i, p in enumerate(initial_targets):
                angles[i] = servo_hat.pulse_to_angle(p)
        except Exception:
            pass

        while True:
            now = time.monotonic()
            frame_count += 1
            if now - fps_timer >= 1.0:
                fps = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now

            # 1. Periodic Telemetry Read (every 50 ms)
            if now - last_adc_read >= 0.05:
                last_adc_read = now
                try:
                    seq, raw_vals = servo_hat.read_servo_adc(bus)
                    for servo_idx, raw in zip(servo_hat.ADC_SERVO_MAP, raw_vals):
                        raw_adcs[servo_idx] = raw
                        currents[servo_idx] = servo_hat.current_from_raw(raw)
                except Exception:
                    pass

            # 2. Periodic MCP3425 Bus Voltage Read (every 250 ms)
            if now - last_mcp_read >= 0.25:
                last_mcp_read = now
                try:
                    v_bus, _, _ = servo_hat.read_mcp3425_bus_voltage(bus)
                    bus_voltage = v_bus
                except Exception:
                    pass

            # 3. Handle Keyboard Input
            key = stdscr.getch()
            if key in (ord('q'), ord('Q'), 27):  # 'q' or ESC
                break

            elif key in (curses.KEY_UP, ord('k'), ord('K')):
                selected = (selected - 1) % servo_hat.SERVO_COUNT

            elif key in (curses.KEY_DOWN, ord('j'), ord('J')):
                selected = (selected + 1) % servo_hat.SERVO_COUNT

            elif key in (curses.KEY_LEFT, ord('h'), ord('H')):
                angles[selected] = max(0.0, angles[selected] - 1.0)
                try:
                    servo_hat.set_servo_angle(bus, selected, angles[selected])
                    status_msg = f"Set S{selected:02d} to {angles[selected]:.1f}°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (curses.KEY_RIGHT, ord('l'), ord('L')):
                angles[selected] = min(180.0, angles[selected] + 1.0)
                try:
                    servo_hat.set_servo_angle(bus, selected, angles[selected])
                    status_msg = f"Set S{selected:02d} to {angles[selected]:.1f}°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (ord('['), curses.KEY_PPAGE):  # -10 deg
                angles[selected] = max(0.0, angles[selected] - 10.0)
                try:
                    servo_hat.set_servo_angle(bus, selected, angles[selected])
                    status_msg = f"Set S{selected:02d} to {angles[selected]:.1f}°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (ord(']'), curses.KEY_NPAGE):  # +10 deg
                angles[selected] = min(180.0, angles[selected] + 10.0)
                try:
                    servo_hat.set_servo_angle(bus, selected, angles[selected])
                    status_msg = f"Set S{selected:02d} to {angles[selected]:.1f}°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (curses.KEY_HOME,):
                angles[selected] = 0.0
                try:
                    servo_hat.set_servo_angle(bus, selected, angles[selected])
                    status_msg = f"Set S{selected:02d} to 0.0°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (curses.KEY_END,):
                angles[selected] = 180.0
                try:
                    servo_hat.set_servo_angle(bus, selected, angles[selected])
                    status_msg = f"Set S{selected:02d} to 180.0°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif ord('0') <= key <= ord('9'):
                preset_angle = (key - ord('0')) * 20.0
                angles[selected] = preset_angle
                try:
                    servo_hat.set_servo_angle(bus, selected, preset_angle)
                    status_msg = f"Set S{selected:02d} to preset {preset_angle:.0f}°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (ord('a'), ord('A')):
                target_ang = angles[selected]
                for i in range(servo_hat.SERVO_COUNT):
                    angles[i] = target_ang
                try:
                    servo_hat.set_all_servo_angles(bus, target_ang)
                    status_msg = f"Set ALL servos to {target_ang:.1f}°"
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            elif key in (ord('s'), ord('S')):
                try:
                    servo_hat.safe_position(bus)
                    status_msg = "Safe / Starting position sent to all servos."
                    # Sync angles from targets report
                    targets = servo_hat.read_servo_targets(bus)
                    for i, p in enumerate(targets):
                        angles[i] = servo_hat.pulse_to_angle(p)
                except Exception as e:
                    status_msg = f"I2C Error: {e}"

            # 4. Render UI
            stdscr.erase()
            h, w = stdscr.getmaxyx()

            # Title & Header
            title = " RP2040 22-SERVO HAT CONTROLLER "
            stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD | (curses.color_pair(1) if has_color else 0))

            volts_str = f"Bus: {bus_voltage:5.2f} V | Rate: {fps:4.1f} Hz | SCL/SDA: GP21/GP20 (0x2A)"
            stdscr.addstr(1, max(0, (w - len(volts_str)) // 2), volts_str, curses.color_pair(5) if has_color else 0)

            # Table Header
            hdr = f"{'Sel':<4}{'Servo':<7}{'GPIO':<6}{'Angle':<8}{'Pulse':<9}{'Slider':<16}{'Current':<10}"
            stdscr.addstr(3, 2, hdr, curses.A_UNDERLINE | curses.A_BOLD)

            # 22 Servos (Display in 2 columns if terminal width allows, otherwise 1 column)
            two_cols = w >= 90
            rows_per_col = 11 if two_cols else 22

            for i in range(servo_hat.SERVO_COUNT):
                col_offset = 48 if (two_cols and i >= 11) else 2
                row_idx = 4 + (i % 11 if two_cols else i)

                if row_idx >= h - 4:
                    break

                is_sel = (i == selected)
                marker = " ▶ " if is_sel else "   "
                gpio = servo_hat.SERVO_GPIO[i]
                ang = angles[i]
                pulse = servo_hat.angle_to_pulse(ang)
                bar = make_bar(ang, width=12)

                if i in servo_hat.ADC_SERVO_MAP:
                    amps_str = f"{currents[i]:5.3f} A"
                else:
                    amps_str = "  N/A   "

                line = f"{marker}{f'S{i:02d}':<6}{f'GP{gpio:02d}':<6}{f'{ang:5.1f}°':<8}{f'{pulse:4d} µs':<9}{bar:<15}{amps_str:<9}"

                attr = curses.A_BOLD if is_sel else 0
                if has_color:
                    if is_sel:
                        attr |= curses.color_pair(3) | curses.A_REVERSE
                    else:
                        attr |= curses.color_pair(2)

                try:
                    stdscr.addstr(row_idx, col_offset, line[:w - col_offset - 1], attr)
                except curses.error:
                    pass

            # Bottom Status & Controls
            stat_row = min(h - 3, 27)
            try:
                stdscr.addstr(stat_row, 2, f"Status: {status_msg}"[:w - 4], curses.A_BOLD)
                help_line = "Keys: [↑/↓] Select | [←/→] ±1° | [ [ / ] ] ±10° | [0-9] Preset | [A] Set All | [S] Safe | [Q] Quit"
                stdscr.addstr(stat_row + 1, 2, help_line[:w - 4], curses.color_pair(1) if has_color else curses.A_DIM)
            except curses.error:
                pass

            stdscr.refresh()
            time.sleep(0.015)  # ~60 FPS loop


def main():
    try:
        curses.wrapper(run_tui)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
