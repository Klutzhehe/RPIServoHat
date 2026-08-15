#!/usr/bin/env python3
"""Gently sweep one explicitly selected servo for wiring and range testing."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smbus2 import SMBus
import rpi_master as servo_hat


parser = argparse.ArgumentParser()
parser.add_argument("servo", type=int, help="servo number, 0 through 21")
parser.add_argument("--minimum", type=int, default=1400)
parser.add_argument("--maximum", type=int, default=1600)
parser.add_argument("--step", type=int, default=20)
parser.add_argument("--delay", type=float, default=0.08)
args = parser.parse_args()

if not 0 <= args.servo < servo_hat.SERVO_COUNT:
    raise SystemExit("servo must be 0 through 21")
if args.minimum >= args.maximum or args.step <= 0:
    raise SystemExit("minimum must be smaller than maximum and step must be positive")

with SMBus(servo_hat.I2C_BUS) as bus:
    try:
        while True:
            for pulse_us in range(args.minimum, args.maximum + 1, args.step):
                servo_hat.set_servo(bus, args.servo, pulse_us)
                time.sleep(args.delay)
            for pulse_us in range(args.maximum, args.minimum - 1, -args.step):
                servo_hat.set_servo(bus, args.servo, pulse_us)
                time.sleep(args.delay)
    except KeyboardInterrupt:
        servo_hat.set_servo(bus, args.servo, 1500)
        print("Stopped; servo returned to 1500 µs.")
