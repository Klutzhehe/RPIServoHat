"""Standalone MicroPython servo test for RP2040.

Run this script directly in Thonny on the Pico to test physical servo movement
without needing the Raspberry Pi or I2C bus.

Sweeps S0 (GP0), S1 (GP1), S2 (GP2), S3 (GP3), and S4 (GP4).
"""

from machine import Pin, PWM
import time

# Test servo channels (GPIO numbers)
TEST_PINS = [0, 1, 2, 3, 4]
PULSE_MIN_US = 1000  # 0 degrees
PULSE_MID_US = 1500  # 90 degrees (neutral)
PULSE_MAX_US = 2000  # 180 degrees

print("=== RP2040 Direct Servo Hardware Test ===")
print("Initializing PWM at 50 Hz on GP pins:", TEST_PINS)

pwms = {}
for gp in TEST_PINS:
    try:
        pwm = PWM(Pin(gp))
        pwm.freq(50)
        # Set to 1500 us neutral (1,500,000 ns)
        pwm.duty_ns(PULSE_MID_US * 1000)
        pwms[gp] = pwm
        print(f"  GP{gp:02d} initialized OK (neutral 1500 us)")
    except Exception as e:
        print(f"  GP{gp:02d} failed: {e}")

print("\nStarting continuous sweep on all test pins (Ctrl+C to stop)...")
print("Check that your 5V/6V servo power supply is turned ON and GND is connected.\n")

try:
    while True:
        # Move to 0 degrees (1000 us)
        print("--> Moving to 0° (1000 µs)...")
        for gp, pwm in pwms.items():
            pwm.duty_ns(PULSE_MIN_US * 1000)
        time.sleep(1.0)

        # Move to 90 degrees (1500 us)
        print("--> Moving to 90° (1500 µs neutral)...")
        for gp, pwm in pwms.items():
            pwm.duty_ns(PULSE_MID_US * 1000)
        time.sleep(1.0)

        # Move to 180 degrees (2000 us)
        print("--> Moving to 180° (2000 µs)...")
        for gp, pwm in pwms.items():
            pwm.duty_ns(PULSE_MAX_US * 1000)
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nTest stopped. Resetting to neutral 1500 us.")
    for gp, pwm in pwms.items():
        pwm.duty_ns(PULSE_MID_US * 1000)
