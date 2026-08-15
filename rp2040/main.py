"""22-servo RP2040 MicroPython I2C slave application.

This replaces the demo main.py from ifurusato/rp2040-i2c-slave.  Keep that
project's rp2040_slave.py, RP2040_I2C_Registers.py, and core/ folder on the
RP2040; this file uses its low-level I2C slave driver directly.

I2C: I2C0, GP20=SDA, GP21=SCL, slave address 0x2A, 100 kHz maximum.
Servo outputs: S0..S19=GP0..GP19, S20=GP22, S21=GP23.
"""

from machine import ADC, PWM, Pin
from time import ticks_add, ticks_diff, ticks_us

from rp2040_slave import RP2040_Slave


# ---------------------------------------------------------------------------
# Hardware configuration
# ---------------------------------------------------------------------------
I2C_BUS_ID = 0
I2C_ADDRESS = 0x2A
I2C_SDA_PIN = 20
I2C_SCL_PIN = 21

SERVO_GPIOS = tuple(range(20)) + (22, 23)
SERVO_COUNT = len(SERVO_GPIOS)
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_START_US = 1000       # Safe position used at boot and by CMD_SAFE.
SERVO_PERIOD_US = 20000     # 50 Hz

MUX_A0 = Pin(24, Pin.OUT)
MUX_A1 = Pin(25, Pin.OUT)
ADC_PINS = (29, 28, 27, 26)  # Same ordering as the original debug program.
ADCS = tuple(ADC(pin) for pin in ADC_PINS)
ADC_SAMPLES = 8
ADC_SAMPLE_GAP_US = 100
MUX_SETTLE_US = 10000

# The report uses this order. S0, S5, S10, S15, S20, and S21 have no routed
# current-sense ADC in the supplied board mapping.
ADC_SERVO_MAP = (4, 1, 3, 2, 9, 6, 8, 7, 14, 11, 13, 12, 19, 16, 18, 17)


# ---------------------------------------------------------------------------
# Binary protocol (Raspberry Pi master -> RP2040 slave)
# ---------------------------------------------------------------------------
CMD_SET_SERVO = 0x01        # [0x01, servo 0..21, pulse_us_hi, pulse_us_lo]
CMD_SET_ALL = 0x02          # [0x02, pulse_us_hi, pulse_us_lo]
CMD_SAFE = 0x03             # [0x03] -> 1000 us on every servo
CMD_READ_ADC_REPORT = 0x10  # [0x10], then master reads 36 bytes

REPORT_MAGIC = 0xA5
REPORT_VERSION = 0x01
REPORT_SIZE = 36             # magic, version, sequence, count, 16 x uint16


def clamp_pulse(pulse_us):
    if pulse_us < SERVO_MIN_US:
        return SERVO_MIN_US
    if pulse_us > SERVO_MAX_US:
        return SERVO_MAX_US
    return pulse_us


def pulse_to_duty(pulse_us):
    return (pulse_us * 65535) // SERVO_PERIOD_US


# ---------------------------------------------------------------------------
# Servo setup
# ---------------------------------------------------------------------------
SERVOS = []
for gpio in SERVO_GPIOS:
    pwm = PWM(Pin(gpio))
    pwm.freq(50)
    pwm.duty_u16(pulse_to_duty(SERVO_START_US))
    SERVOS.append(pwm)


def set_servo(servo_number, pulse_us):
    if not 0 <= servo_number < SERVO_COUNT:
        return False
    SERVOS[servo_number].duty_u16(pulse_to_duty(clamp_pulse(pulse_us)))
    return True


def set_all_servos(pulse_us):
    duty = pulse_to_duty(clamp_pulse(pulse_us))
    for pwm in SERVOS:
        pwm.duty_u16(duty)


# ---------------------------------------------------------------------------
# Incremental current-sense scan
#
# Do not scan all 16 channels in one blocking operation. The RP2040 is an I2C
# slave, so it must return quickly to service the Pi. One ADC sample is taken
# per main-loop pass, while the mux gets its full 10 ms settle time.
# ---------------------------------------------------------------------------
adc_raw = [0] * 16
adc_sequence = 0
scan_mux = 0
scan_adc = 0
sample_sum = 0
sample_count = 0
next_adc_sample_at = 0


def select_mux(mux_index):
    MUX_A0.value(mux_index & 1)
    MUX_A1.value((mux_index >> 1) & 1)


def start_scan():
    global next_adc_sample_at
    select_mux(scan_mux)
    next_adc_sample_at = ticks_add(ticks_us(), MUX_SETTLE_US)


def adc_scan_tick():
    global scan_mux, scan_adc, sample_sum, sample_count
    global next_adc_sample_at, adc_sequence

    now = ticks_us()
    if ticks_diff(now, next_adc_sample_at) < 0:
        return

    sample_sum += ADCS[scan_adc].read_u16()
    sample_count += 1
    next_adc_sample_at = ticks_add(now, ADC_SAMPLE_GAP_US)

    if sample_count < ADC_SAMPLES:
        return

    # Store ADC-major, MUX-minor: exactly the mapping used by the original code.
    adc_raw[scan_adc * 4 + scan_mux] = sample_sum // ADC_SAMPLES
    sample_sum = 0
    sample_count = 0
    scan_adc += 1

    if scan_adc < 4:
        return

    scan_adc = 0
    scan_mux += 1
    if scan_mux == 4:
        scan_mux = 0
        adc_sequence = (adc_sequence + 1) & 0xFF
    select_mux(scan_mux)
    next_adc_sample_at = ticks_add(ticks_us(), MUX_SETTLE_US)


start_scan()


# ---------------------------------------------------------------------------
# I2C slave transport. RP2040_Slave is supplied by the library you installed.
# ---------------------------------------------------------------------------
slave = RP2040_Slave(
    i2c_id=I2C_BUS_ID,
    sda=I2C_SDA_PIN,
    scl=I2C_SCL_PIN,
    i2c_address=I2C_ADDRESS,
)

rx_buffer = bytearray()
rx_overflow = False
reply = bytearray((0xEE,))  # 0xEE means invalid command.
reply_index = 0


def set_reply(data):
    global reply, reply_index
    reply = bytearray(data)
    reply_index = 0


def build_adc_report():
    report = bytearray(REPORT_SIZE)
    report[0] = REPORT_MAGIC
    report[1] = REPORT_VERSION
    report[2] = adc_sequence
    report[3] = len(adc_raw)
    for index, raw in enumerate(adc_raw):
        offset = 4 + index * 2
        report[offset] = raw >> 8
        report[offset + 1] = raw & 0xFF
    set_reply(report)


def process_command(packet):
    if len(packet) == 4 and packet[0] == CMD_SET_SERVO:
        pulse_us = (packet[2] << 8) | packet[3]
        set_reply((0x00 if set_servo(packet[1], pulse_us) else 0xEE,))
    elif len(packet) == 3 and packet[0] == CMD_SET_ALL:
        set_all_servos((packet[1] << 8) | packet[2])
        set_reply((0x00,))
    elif len(packet) == 1 and packet[0] == CMD_SAFE:
        set_all_servos(SERVO_START_US)
        set_reply((0x00,))
    elif len(packet) == 1 and packet[0] == CMD_READ_ADC_REPORT:
        build_adc_report()
    else:
        set_reply((0xEE,))


def i2c_tick():
    """Service at most the currently pending I2C event; never block here."""
    global rx_buffer, rx_overflow, reply_index

    state = slave.handle_event()
    if state == slave.I2CStateMachine.I2C_START:
        # A new master read begins at byte zero of the prepared response.
        reply_index = 0
    elif state == slave.I2CStateMachine.I2C_RECEIVE:
        while slave.Available():
            received = slave.Read_Data_Received()
            if len(rx_buffer) < 4:
                rx_buffer.append(received)
            else:
                rx_overflow = True
    elif state == slave.I2CStateMachine.I2C_REQUEST:
        if reply_index < len(reply):
            slave.Slave_Write_Data(reply[reply_index])
            reply_index += 1
        else:
            slave.Slave_Write_Data(0x00)
    elif state == slave.I2CStateMachine.I2C_FINISH:
        if rx_buffer:
            if rx_overflow:
                set_reply((0xEE,))
            else:
                process_command(rx_buffer)
        rx_buffer = bytearray()
        rx_overflow = False


print("RP2040 servo slave ready: I2C0 GP20/GP21, address 0x2A")
while True:
    i2c_tick()
    adc_scan_tick()
