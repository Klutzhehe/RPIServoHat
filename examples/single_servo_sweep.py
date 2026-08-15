#!/usr/bin/env python3
"""Gently sweep one explicitly selected servo in degrees (0° - 180°) for range testing."""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smbus2 import SMBus
import rpi_master as servo_hat


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("servo", type=int, help="Servo number, 0 through 21")
parser.add_argument("--min-angle", type=float, default=70.0, help="Minimum sweep angle in degrees (default 70°)")
parser.add_argument("--max-angle", type=float, default=110.0, help="Maximum sweep angle in degrees (default 110°)")
parser.add_argument("--step", type=float, default=2.0, help="Step angle in degrees (default 2°)")
parser.add_argument("--delay", type=float, default=0.04, help="Delay between steps in seconds (default 0.04s)")
args = parser.parse_args()

if not 0 <= args.servo < servo_hat.SERVO_COUNT:
    raise SystemExit("Servo must be 0 through 21")
if args.min_angle >= args.max_angle or args.step <= 0:
    raise SystemExit("min-angle must be smaller than max-angle and step must be positive")

with SMBus(servo_hat.I2C_BUS) as bus:
    try:
        print(f"Sweeping S{args.servo:02d} between {args.min_angle}° and {args.max_angle}° (Ctrl+C to stop)...")
        while True:
            cur = args.min_angle
            while cur <= args.max_angle:
                servo_hat.set_servo(bus, args.servo, cur)
                time.sleep(args.delay)
                cur += args.step

            cur = args.max_angle
            while cur >= args.min_angle:
                servo_hat.set_servo(bus, args.servo, cur)
                time.sleep(args.delay)
                cur -= args.step
    except KeyboardInterrupt:
        servo_hat.set_servo(bus, args.servo, 90.0)
        print("Stopped; servo returned to 90° (neutral).")
