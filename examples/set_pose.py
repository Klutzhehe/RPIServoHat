#!/usr/bin/env python3
"""Set a named servo pose. Edit POSE_US to match your mechanism."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smbus2 import SMBus
import rpi_master as servo_hat


# Example only: adjust these values only after checking each servo's safe range.
POSE_US = {
    0: 1500,
    1: 1500,
    2: 1500,
    3: 1500,
}

with SMBus(servo_hat.I2C_BUS) as bus:
    for servo, pulse_us in POSE_US.items():
        servo_hat.set_servo(bus, servo, pulse_us)
        print(f"S{servo:02d} -> {pulse_us} µs")
