#!/usr/bin/env python3
"""Set a named servo pose in degrees (0° - 180°). Edit POSE_ANGLES to match your mechanism."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smbus2 import SMBus
import rpi_master as servo_hat


# Example angles (0° to 180°):
POSE_ANGLES = {
    0: 90.0,
    1: 90.0,
    2: 45.0,
    3: 135.0,
}

with SMBus(servo_hat.I2C_BUS) as bus:
    for servo, angle in POSE_ANGLES.items():
        servo_hat.set_servo(bus, servo, angle)
        print(f"S{servo:02d} -> {angle:5.1f}° ({servo_hat.angle_to_pulse(angle)} µs)")
