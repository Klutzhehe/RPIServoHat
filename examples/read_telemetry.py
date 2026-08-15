#!/usr/bin/env python3
"""Print MCP3425 voltage and the 16 available servo-current readings."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from smbus2 import SMBus
import rpi_master as servo_hat


with SMBus(servo_hat.I2C_BUS) as bus:
    try:
        while True:
            servo_hat.print_status(bus)
            print()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped.")
