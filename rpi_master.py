#!/usr/bin/env python3
"""Raspberry Pi I2C master for the RP2040 MicroPython servo board and MCP3425A0T.

Both peripherals share the Raspberry Pi's I2C-1 bus:
  Raspberry Pi SDA/SCL <-> RP2040 GP20/GP21 <-> MCP3425 SDA/SCL

Examples:
  python3 rpi_master.py set 20 1500
  python3 rpi_master.py all 1500
  python3 rpi_master.py read
  python3 rpi_master.py monitor --interval 0.5
"""

import argparse
import sys
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError as error:
    raise SystemExit("Install the dependency first: python3 -m pip install smbus2") from error

I2C_BUS = 1
RP2040_ADDRESS = 0x2A
MCP3425_ADDRESS = 0x68  # MCP3425A0T; A1/A2/A3 parts use 0x69/0x6A/0x6B.

SERVO_COUNT = 22
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000

# Physical GP pin controlled by each S number.
SERVO_GPIO = tuple(range(20)) + (22, 23)

# Report order from the RP2040 firmware.  These are the 16 current-sense routes.
ADC_SERVO_MAP = (4, 1, 3, 2, 9, 6, 8, 7, 14, 11, 13, 12, 19, 16, 18, 17)

ADC_VREF = 3.3
SHUNT_OHMS = 0.020
INA_GAIN = 50.0

CMD_SET_SERVO = 0x01
CMD_SET_ALL = 0x02
CMD_SAFE_POSITION = 0x03
CMD_READ_ADC_REPORT = 0x10

REPORT_MAGIC = 0xA5
REPORT_SIZE = 36

# MCP3425: start one-shot conversion, 16-bit / 15 SPS / PGA x1.
MCP3425_CONFIG_16BIT_ONE_SHOT = 0x88
MCP3425_LSB_VOLTS = 2.048 / 32768.0
DIVIDER_HIGH_OHMS = 39000.0
DIVIDER_LOW_OHMS = 1000.0
DIVIDER_RATIO = (DIVIDER_HIGH_OHMS + DIVIDER_LOW_OHMS) / DIVIDER_LOW_OHMS


def write(bus, address, data):
    bus.i2c_rdwr(i2c_msg.write(address, data))


def read(bus, address, count):
    message = i2c_msg.read(address, count)
    bus.i2c_rdwr(message)
    return bytes(message)


def clamp_pulse(pulse_us):
    return max(SERVO_MIN_US, min(SERVO_MAX_US, pulse_us))


def set_servo(bus, servo, pulse_us):
    if not 0 <= servo < SERVO_COUNT:
        raise ValueError("servo must be 0 through 21")
    pulse_us = clamp_pulse(pulse_us)
    write(bus, RP2040_ADDRESS, (CMD_SET_SERVO, servo, pulse_us >> 8, pulse_us & 0xFF))


def set_all_servos(bus, pulse_us):
    pulse_us = clamp_pulse(pulse_us)
    write(bus, RP2040_ADDRESS, (CMD_SET_ALL, pulse_us >> 8, pulse_us & 0xFF))


def safe_position(bus):
    write(bus, RP2040_ADDRESS, (CMD_SAFE_POSITION,))


def read_servo_adc(bus, retries=3):
    # The command is completed by a STOP; the next transaction reads its report.
    for attempt in range(retries):
        try:
            write(bus, RP2040_ADDRESS, (CMD_READ_ADC_REPORT,))
            time.sleep(0.010)  # Allow RP2040 slave time to stage the reply into FIFO
            report = read(bus, RP2040_ADDRESS, REPORT_SIZE)
            if len(report) != REPORT_SIZE or report[0] != REPORT_MAGIC or report[1] != 1 or report[3] != 16:
                raise RuntimeError("invalid RP2040 ADC report: " + report.hex(" "))

            raw_values = tuple((report[offset] << 8) | report[offset + 1] for offset in range(4, REPORT_SIZE, 2))
            return report[2], raw_values
        except (OSError, RuntimeError) as e:
            if attempt == retries - 1:
                raise
            time.sleep(0.01)


def current_from_raw(raw):
    voltage = raw * ADC_VREF / 65535.0
    return voltage / (SHUNT_OHMS * INA_GAIN)


def read_mcp3425_bus_voltage(bus):
    # Request a fresh one-shot 16-bit conversion then wait for RDY=0.
    write(bus, MCP3425_ADDRESS, (MCP3425_CONFIG_16BIT_ONE_SHOT,))
    deadline = time.monotonic() + 0.25
    # 16-bit 15 SPS conversion takes ~67 ms; sleep before polling to reduce bus congestion
    time.sleep(0.05)
    while True:
        response = read(bus, MCP3425_ADDRESS, 3)
        if len(response) != 3:
            raise RuntimeError("short MCP3425 response")
        raw = int.from_bytes(response[:2], byteorder="big", signed=True)
        if not response[2] & 0x80:  # RDY bit clears when conversion is complete.
            sensor_volts = raw * MCP3425_LSB_VOLTS
            return sensor_volts * DIVIDER_RATIO, sensor_volts, raw
        if time.monotonic() >= deadline:
            raise TimeoutError("MCP3425 16-bit conversion timed out")
        time.sleep(0.005)


def print_status(bus):
    bus_volts, divider_volts, mcp_raw = read_mcp3425_bus_voltage(bus)
    sequence, raw_values = read_servo_adc(bus)
    print(f"Bus voltage: {bus_volts:.3f} V  (MCP3425 Vin+: {divider_volts:.5f} V, raw {mcp_raw})")
    print(f"Servo current sample {sequence}:")
    for servo, raw in zip(ADC_SERVO_MAP, raw_values):
        print(f"  S{servo:02d} / GP{SERVO_GPIO[servo]:02d}: raw={raw:5d}  current={current_from_raw(raw):.3f} A")
    print("  No routed current ADC: S00, S05, S10, S15, S20, S21")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    set_parser = subparsers.add_parser("set")
    set_parser.add_argument("servo", type=int)
    set_parser.add_argument("pulse_us", type=int)
    all_parser = subparsers.add_parser("all")
    all_parser.add_argument("pulse_us", type=int)
    subparsers.add_parser("safe")
    subparsers.add_parser("read")
    monitor_parser = subparsers.add_parser("monitor")
    monitor_parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    with SMBus(I2C_BUS) as bus:
        if args.command == "set":
            set_servo(bus, args.servo, args.pulse_us)
        elif args.command == "all":
            set_all_servos(bus, args.pulse_us)
        elif args.command == "safe":
            safe_position(bus)
        elif args.command == "read":
            print_status(bus)
        elif args.command == "monitor":
            while True:
                print_status(bus)
                time.sleep(max(0.05, args.interval))


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
