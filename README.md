# RP2040 22-servo board + Raspberry Pi master

This project is entirely Python: standard MicroPython on the RP2040 and Python
on the Raspberry Pi. It uses the installed
[`ifurusato/rp2040-i2c-slave`](https://github.com/ifurusato/rp2040-i2c-slave)
driver to make the RP2040 an I2C slave. Servo outputs are `S0..S19 = GP0..GP19`,
`S20 = GP22`, and `S21 = GP23`. GP20 and GP21 are reserved for I2C and are
never configured as PWM outputs.

## I2C wiring

Connect all three devices on one 3.3 V I2C bus:

| Signal | Raspberry Pi | RP2040 board | MCP3425A0T |
| --- | --- | --- | --- |
| SDA | GPIO2 / physical pin 3 | GP20 | SDA |
| SCL | GPIO3 / physical pin 5 | GP21 | SCL |
| Ground | any GND | GND | VSS and VIN- |
| 3.3 V | 3V3 | 3V3 logic supply | VDD |

Use one suitable pair of I2C pull-ups to **3.3 V** only. Do not allow a 5 V
pull-up onto either Raspberry Pi or RP2040 I2C pin. The addresses are RP2040
`0x2A` and MCP3425A0T `0x68`.

The 39 kOhm / 1 kOhm divider has a ratio of 40. With the MCP3425 at gain x1,
its differential input limit is +/-2.048 V, so the nominal positive bus
measurement limit is 81.92 V. Keep both MCP3425 input pins within their
absolute voltage limits as well.

## Install the RP2040 application

First install the contents of the library's `upy/` directory on the RP2040 as
you already did. The `RPI-RP2` drive shown while BOOTSEL is held is only for
copying a MicroPython `.uf2` firmware file; Python files must be copied through
Thonny's **MicroPython device** filesystem after normal boot. Then replace the
RP2040 root-level `main.py` with this
repository's [`rp2040/main.py`](rp2040/main.py). Do **not** remove the library
files `rp2040_slave.py`, `RP2040_I2C_Registers.py`, or its `core/` directory:
the application imports them.

The Raspberry Pi must be the only I2C master. It reads the MCP3425 directly and
sends commands to the RP2040. Use the default 100 kHz I2C rate; the slave is a
MicroPython polling implementation.

## Run on the Raspberry Pi

On Raspberry Pi OS, enable I2C in `raspi-config`, then install the Python
dependency and run, for example:

```sh
python3 -m pip install -r requirements.txt
```

If Raspberry Pi OS reports that its system Python is externally managed, create
a virtual environment instead:

```sh
sudo apt install -y python3-venv
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

Then run:

```sh
python3 rpi_master.py set 20 1500
python3 rpi_master.py read
python3 rpi_master.py monitor --interval 0.5
```

`read` and `monitor` show MCP3425 bus voltage plus the 16 current-sense routes.
They are S1-S4, S6-S9, S11-S14, and S16-S19; the hardware mapping does not
provide ADC current routes for S0, S5, S10, S15, S20, or S21.

## Terminal User Interface (TUI)

For fast, interactive terminal-based control over SSH or local terminal without GUI dependencies:

```sh
python3 servo_tui.py
```

* **Keyboard controls**:
  * `↑` / `↓` (or `k` / `j`): Select servo (`S00` .. `S21`)
  * `←` / `→` (or `h` / `l`): Adjust angle by $\pm 1^\circ$
  * `[` / `]` (or `PgUp` / `PgDn`): Adjust angle by $\pm 10^\circ$
  * `0` .. `9`: Instant preset angles ($0^\circ, 20^\circ, \dots, 180^\circ$)
  * `a` / `A`: Set all servos to selected angle
  * `s` / `S`: Safe position (return all servos to starting angles)
  * `q` / `ESC`: Quit

## Command-Line Usage

```sh
python3 rpi_master.py set 0 1500           # Set S0 to 1500 us
python3 rpi_master.py set-angle 0 90       # Set S0 to 90 degrees
python3 rpi_master.py all-angles 90        # Set all servos to 90 degrees
python3 rpi_master.py targets              # Read active target angles of all 22 servos
python3 rpi_master.py read                 # Read bus voltage, currents, and target angles
python3 rpi_master.py monitor --interval 0.5
```

## Desktop GUI

The control panel needs a Raspberry Pi desktop session (local monitor, VNC, or
remote desktop), plus Tkinter:

```sh
sudo apt install -y python3-tk
python3 servo_gui.py
```

## Examples

See [`examples/`](examples/README.md) for a telemetry display, an editable pose,
and a conservative one-servo sweep. Always begin with one disconnected or
unloaded servo and confirm its safe travel before using a wider range.

## I2C protocol

The Raspberry Pi script implements the wire protocol, but it is documented here
for other software:

| Command | Bytes written | Result |
| --- | --- | --- |
| Set one servo | `01 SS HH LL` | `SS` is S0-S21; `HH LL` is pulse width in us (1000-2000 us). |
| Set all servos | `02 HH LL` | Sets every servo to 1000-2000 us. |
| Safe position | `03` | Sets every servo to its configured starting angle. |
| Read current ADCs | Write `10`, then read 36 bytes | `A5 01 sequence 10` followed by 16 big-endian raw ADC values. |
| Read target angles | Write `11`, then read 48 bytes | `B5 01 16 00` followed by 22 big-endian uint16 target pulse us values. |

