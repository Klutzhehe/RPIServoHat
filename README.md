# RP2040 22-Servo HAT + Raspberry Pi Master

A high-performance Python application for controlling 22 PWM servos with live 16-channel current sensing and bus voltage monitoring over I2C.

Servos are commanded by **Angle ($0^\circ \dots 180^\circ$)**, matching Arduino's standard `Servo.write(angle)` API:
* **$0^\circ$** $\iff 1000\,\mu\text{s}$
* **$90^\circ$** $\iff 1500\,\mu\text{s}$ (Neutral / Default Starting Position)
* **$180^\circ$** $\iff 2000\,\mu\text{s}$

Servo mapping: `S0..S19 = GP0..GP19`, `S20 = GP22`, `S21 = GP23`. (`GP20` and `GP21` are dedicated I2C pins).

---

## I2C Wiring & Pinout

Connect all three devices to the Raspberry Pi's **3.3 V I2C-1** bus:

| Signal | Raspberry Pi | RP2040 Board | MCP3425A0T | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **SDA** | GPIO2 (Pin 3) | GP20 (Pin 26) | SDA | Shared I2C Data line |
| **SCL** | GPIO3 (Pin 5) | GP21 (Pin 27) | SCL | Shared I2C Clock line |
| **GND** | Any GND pin | GND | VSS & VIN- | Common ground |
| **3.3 V** | 3V3 (Pin 1) | 3V3 | VDD | Logic power supply |

> [!IMPORTANT]
> Use I2C pull-up resistors to **3.3 V only**. Do not allow 5 V onto Raspberry Pi or RP2040 I2C pins.
> * RP2040 I2C Slave Address: `0x2A`
> * MCP3425 ADC Address: `0x68`

---

## 1. RP2040 Firmware Setup

1. Connect your Raspberry Pi Pico via USB.
2. In Thonny (or your MicroPython IDE), copy [`rp2040/main.py`](rp2040/main.py) to the root of the Pico filesystem as `main.py`.
3. Soft-reset (or power cycle) the Pico.

### Customizing Starting Angles
In [`rp2040/main.py`](rp2040/main.py), you can configure the boot/safe starting angle for all servos:
```python
# Default is 90 degrees for all 22 servos:
SERVO_START_ANGLES = (90,) * SERVO_COUNT

# Or customize per servo (e.g. S0=0°, S1=90°, S2=180°, ...):
# SERVO_START_ANGLES = (0, 90, 180, ...)
```

---

## 2. Raspberry Pi Setup

1. Enable I2C via `raspi-config` (`Interfacing Options` -> `I2C` -> `Enable`).
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Test bus detection:
   ```bash
   sudo i2cdetect -y 1
   ```
   *(Addresses `0x2A` and `0x68` should appear)*.

---

## 3. Interactive Terminal User Interface (TUI)

A real-time, double-buffered curses application designed for fast control over SSH or local terminal without GUI dependencies:

```bash
python3 servo_tui.py
```

### Keyboard Controls:
| Key | Action |
| :--- | :--- |
| `↑` / `↓` (or `k` / `j`) | Select servo (`S00` .. `S21`) |
| `←` / `→` (or `h` / `l`) | Adjust selected servo by $\pm 1^\circ$ |
| `[` / `]` (or `PgUp` / `PgDn`) | Adjust selected servo by $\pm 10^\circ$ |
| `Home` / `End` | Jump to $0^\circ$ / $180^\circ$ limits |
| `0` .. `9` | Instant preset angles ($0^\circ, 20^\circ, 40^\circ, \dots, 180^\circ$) |
| `a` / `A` | Set **ALL servos** to currently selected angle |
| `s` / `S` | Return all servos to **Safe / Starting angles** |
| `q` / `ESC` | Quit |

---

## 4. Command-Line Usage (`rpi_master.py`)

```bash
# Set individual servo angle (0° - 180°):
python3 rpi_master.py set 0 90          # S0 to 90° (neutral)
python3 rpi_master.py set 0 0           # S0 to 0°
python3 rpi_master.py set 0 180         # S0 to 180°

# Set ALL servos to an angle:
python3 rpi_master.py all 90            # All servos to 90°
python3 rpi_master.py all 45            # All servos to 45°

# Return all servos to safe starting angles:
python3 rpi_master.py safe

# Read active target positions of all 22 servos:
python3 rpi_master.py targets

# Read bus voltage, 16-channel currents, and targets:
python3 rpi_master.py read

# Continuously monitor telemetry:
python3 rpi_master.py monitor --interval 0.5
```

---

## 5. Python API

```python
from smbus2 import SMBus
import rpi_master as servo_hat

with SMBus(servo_hat.I2C_BUS) as bus:
    # Set servo S0 to 90 degrees
    servo_hat.set_servo(bus, 0, 90.0)

    # Set all servos to 45 degrees
    servo_hat.set_all_servos(bus, 45.0)

    # Read active target pulse widths (1000..2000 us)
    targets = servo_hat.read_servo_targets(bus)
    for servo_id, pulse in enumerate(targets):
        angle = servo_hat.pulse_to_angle(pulse)
        print(f"S{servo_id:02d}: {angle:.1f}° ({pulse} µs)")

    # Read bus voltage and live currents
    v_bus, v_div, raw_mcp = servo_hat.read_mcp3425_bus_voltage(bus)
    seq, raw_adcs = servo_hat.read_servo_adc(bus)
```

---

## 6. Desktop GUI (`servo_gui.py`)

For desktop environments (HDMI monitor, VNC, or X11):

```bash
sudo apt install -y python3-tk
python3 servo_gui.py
```

Features individual $0^\circ \dots 180^\circ$ sliders, numeric inputs, **Set all**, **Safe: all 90°**, and live current telemetry table.

---

## 7. Current-Sense Channel Mapping

The RP2040 scans 16 current-sense channels via 4 ADCs and a 74HC4052 analog multiplexer:

| Channel Index | Servo Pin | RP2040 GPIO | Current Shunt |
| :---: | :---: | :---: | :---: |
| 0 .. 3 | S04, S01, S03, S02 | GP04, GP01, GP03, GP02 | Routed |
| 4 .. 7 | S09, S06, S08, S07 | GP09, GP06, GP08, GP07 | Routed |
| 8 .. 11 | S14, S11, S13, S12 | GP14, GP11, GP13, GP12 | Routed |
| 12 .. 15 | S19, S16, S18, S17 | GP19, GP16, GP18, GP17 | Routed |

*(Servos `S00`, `S05`, `S10`, `S15`, `S20`, and `S21` are PWM outputs without routed current-sense shunts)*.

---

## 8. I2C Wire Protocol Specification

| Command | Bytes Written (Master -> Slave) | Slave Response / Action |
| :--- | :--- | :--- |
| **Set Servo** | `0x01 [servo 0..21] [pulse_hi] [pulse_lo]` | Sets PWM pulse width ($1000 \dots 2000\,\mu\text{s}$). |
| **Set All Servos** | `0x02 [pulse_hi] [pulse_lo]` | Sets all 22 PWM outputs. |
| **Safe Position** | `0x03` | Resets all servos to configured starting angles. |
| **Read Current ADCs** | Write `0x10`, then read 36 bytes | Header: `0xA5 0x01 [seq] 0x10`, followed by 16 $\times$ `uint16` big-endian raw ADC values. |
| **Read Target Angles** | Write `0x11`, then read 48 bytes | Header: `0xB5 0x01 0x16 0x00`, followed by 22 $\times$ `uint16` big-endian target pulse widths. |
