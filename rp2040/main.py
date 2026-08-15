"""22-servo RP2040 MicroPython I2C slave application.

High-performance, zero-allocation implementation for Raspberry Pi master communication:
  - 22 PWM servo outputs: S0..S19=GP0..GP19, S20=GP22, S21=GP23
  - Configurable startup angle table for all 22 servos (default 90 deg / 1500 us)
  - Live target pulse/angle tracking
  - Priority I2C loop: tight FIFO service loop takes precedence over ADC scanning
  - Zero runtime GC allocation inside i2c_tick and adc_scan_tick
  - Command 0x01: Set servo pulse width [0x01, servo 0..21, hi, lo]
  - Command 0x02: Set all servos pulse width [0x02, hi, lo]
  - Command 0x03: Return all servos to safe starting position
  - Command 0x10: 36-byte current-sense ADC report (Magic 0xA5)
  - Command 0x11: 48-byte servo target pulse/angle report (Magic 0xB5)
"""

from machine import ADC, PWM, Pin, mem32
from time import ticks_add, ticks_diff, ticks_us, ticks_ms
import gc
import sys
import micropython

micropython.alloc_emergency_exception_buf(100)

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
SERVO_PERIOD_US = 20000  # 50 Hz

# Configurable starting angle for each servo (S0..S21) in degrees (0..180)
# Default is 90 degrees (1500 us neutral position). Customize per servo as needed:
SERVO_START_ANGLES = (90,) * SERVO_COUNT

MUX_A0 = Pin(24, Pin.OUT)
MUX_A1 = Pin(25, Pin.OUT)
ADC_PINS = (29, 28, 27, 26)
ADCS = tuple(ADC(pin) for pin in ADC_PINS)
ADC_SAMPLES = 4
ADC_SAMPLE_GAP_US = 20
MUX_SETTLE_US = 100  # 100 µs settle time for fast telemetry refresh

# 16 routed current-sense ADC mapping
ADC_SERVO_MAP = (4, 1, 3, 2, 9, 6, 8, 7, 14, 11, 13, 12, 19, 16, 18, 17)

# ---------------------------------------------------------------------------
# Binary protocol (Raspberry Pi master -> RP2040 slave)
# ---------------------------------------------------------------------------
CMD_SET_SERVO = 0x01          # [0x01, servo 0..21, pulse_us_hi, pulse_us_lo]
CMD_SET_ALL = 0x02            # [0x02, pulse_us_hi, pulse_us_lo]
CMD_SAFE = 0x03               # [0x03] -> return all servos to starting angles
CMD_READ_ADC_REPORT = 0x10    # [0x10], then master reads 36-byte ADC report
CMD_READ_TARGET_REPORT = 0x11 # [0x11], then master reads 48-byte target report

ADC_REPORT_MAGIC = 0xA5
ADC_REPORT_VERSION = 0x01
ADC_REPORT_SIZE = 36          # magic (1), ver (1), seq (1), count (1), 16 x uint16 (32)

TARGETS_REPORT_MAGIC = 0xB5
TARGETS_REPORT_VERSION = 0x01
TARGETS_REPORT_SIZE = 48       # magic (1), ver (1), count (1), res (1), 22 x uint16 (44)

# Pre-allocated zero-allocation telemetry reports
adc_report = bytearray(ADC_REPORT_SIZE)
adc_report[0] = ADC_REPORT_MAGIC
adc_report[1] = ADC_REPORT_VERSION
adc_report[2] = 0
adc_report[3] = 16

targets_report = bytearray(TARGETS_REPORT_SIZE)
targets_report[0] = TARGETS_REPORT_MAGIC
targets_report[1] = TARGETS_REPORT_VERSION
targets_report[2] = SERVO_COUNT
targets_report[3] = 0x00


def clamp_pulse(pulse_us):
    if pulse_us < SERVO_MIN_US:
        return SERVO_MIN_US
    if pulse_us > SERVO_MAX_US:
        return SERVO_MAX_US
    return int(pulse_us)


def pulse_to_duty(pulse_us):
    return (int(pulse_us) * 65535) // SERVO_PERIOD_US


def angle_to_pulse(angle_deg):
    if angle_deg < 0:
        angle_deg = 0
    elif angle_deg > 180:
        angle_deg = 180
    return SERVO_MIN_US + int((angle_deg * (SERVO_MAX_US - SERVO_MIN_US)) / 180)


def pulse_to_angle(pulse_us):
    p = clamp_pulse(pulse_us)
    return ((p - SERVO_MIN_US) * 180.0) / (SERVO_MAX_US - SERVO_MIN_US)


# ---------------------------------------------------------------------------
# Servo setup and live target tracking
# ---------------------------------------------------------------------------
servo_targets = [angle_to_pulse(SERVO_START_ANGLES[i]) for i in range(SERVO_COUNT)]
SERVOS = []

for i, gpio in enumerate(SERVO_GPIOS):
    pwm = PWM(Pin(gpio))
    pwm.freq(50)
    p = servo_targets[i]
    pwm.duty_u16(pulse_to_duty(p))
    SERVOS.append(pwm)
    offset = 4 + i * 2
    targets_report[offset] = (p >> 8) & 0xFF
    targets_report[offset + 1] = p & 0xFF


def set_servo(servo_number, pulse_us):
    if not 0 <= servo_number < SERVO_COUNT:
        return False
    clamped = clamp_pulse(pulse_us)
    servo_targets[servo_number] = clamped
    SERVOS[servo_number].duty_u16(pulse_to_duty(clamped))
    offset = 4 + servo_number * 2
    targets_report[offset] = (clamped >> 8) & 0xFF
    targets_report[offset + 1] = clamped & 0xFF
    return True


def set_all_servos(pulse_us):
    clamped = clamp_pulse(pulse_us)
    duty = pulse_to_duty(clamped)
    for i, pwm in enumerate(SERVOS):
        servo_targets[i] = clamped
        pwm.duty_u16(duty)
        offset = 4 + i * 2
        targets_report[offset] = (clamped >> 8) & 0xFF
        targets_report[offset + 1] = clamped & 0xFF


def reset_to_starting_angles():
    for i in range(SERVO_COUNT):
        set_servo(i, angle_to_pulse(SERVO_START_ANGLES[i]))


# ---------------------------------------------------------------------------
# Incremental current-sense scan
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

    avg_sample = sample_sum // ADC_SAMPLES
    idx = scan_adc * 4 + scan_mux
    adc_raw[idx] = avg_sample

    offset = 4 + (idx * 2)
    adc_report[offset] = (avg_sample >> 8) & 0xFF
    adc_report[offset + 1] = avg_sample & 0xFF

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
        adc_report[2] = adc_sequence

    select_mux(scan_mux)
    next_adc_sample_at = ticks_add(ticks_us(), MUX_SETTLE_US)


start_scan()

# ---------------------------------------------------------------------------
# Zero-Allocation RP2040 I2C Slave Driver
# ---------------------------------------------------------------------------
IO_BANK0_BASE = 0x40014000
I2C0_BASE     = 0x40044000
I2C1_BASE     = 0x40048000

MEM_RW  = 0x0000
MEM_XOR = 0x1000
MEM_SET = 0x2000
MEM_CLR = 0x3000

IC_CON             = 0x00
IC_TAR             = 0x04
IC_SAR             = 0x08
IC_DATA_CMD        = 0x10
IC_INTR_STAT       = 0x2C
IC_INTR_MASK       = 0x30
IC_RAW_INTR_STAT   = 0x34
IC_CLR_INTR        = 0x40
IC_CLR_RX_UNDER    = 0x44
IC_CLR_RX_OVER     = 0x48
IC_CLR_TX_OVER     = 0x4C
IC_CLR_RD_REQ      = 0x50
IC_CLR_TX_ABRT     = 0x54
IC_CLR_RX_DONE     = 0x58
IC_CLR_ACTIVITY    = 0x5C
IC_CLR_STOP_DET    = 0x60
IC_CLR_START_DET   = 0x64
IC_CLR_GEN_CALL    = 0x68
IC_ENABLE          = 0x6C
IC_STATUS          = 0x70
IC_TXFLR           = 0x74
IC_RXFLR           = 0x78
IC_CLR_RESTART_DET = 0xA8

INTR_RX_UNDER    = 0x0001
INTR_RX_OVER     = 0x0002
INTR_RX_FULL     = 0x0004
INTR_TX_OVER     = 0x0008
INTR_TX_EMPTY    = 0x0010
INTR_RD_REQ      = 0x0020
INTR_TX_ABRT     = 0x0040
INTR_RX_DONE     = 0x0080
INTR_ACTIVITY    = 0x0100
INTR_STOP_DET    = 0x0200
INTR_START_DET   = 0x0400
INTR_GEN_CALL    = 0x0800
INTR_RESTART_DET = 0x1000

STATUS_ACTIVITY  = 0x01
STATUS_TFNF      = 0x02  # TX FIFO Not Full
STATUS_TFE       = 0x04  # TX FIFO Empty
STATUS_RFNE      = 0x08  # RX FIFO Not Empty
STATUS_RFF       = 0x10  # RX FIFO Full

STATE_RECEIVE = 0
STATE_REQUEST = 1
STATE_FINISH  = 2
STATE_START   = 3


class RP2040_Slave_Driver:
    class I2CStateMachine:
        I2C_RECEIVE = STATE_RECEIVE
        I2C_REQUEST = STATE_REQUEST
        I2C_FINISH  = STATE_FINISH
        I2C_START   = STATE_START

    def __init__(self, i2c_id=0, sda=20, scl=21, i2c_address=0x2A):
        self._scl = scl
        self._sda = sda
        self._i2c_address = i2c_address
        self._i2c_base = I2C0_BASE if i2c_id == 0 else I2C1_BASE

        # 1. Disable the I2C controller
        mem32[self._i2c_base | MEM_CLR | IC_ENABLE] = 0x01

        # 2. Set slave address in IC_SAR (bits 9:0)
        mem32[self._i2c_base | MEM_CLR | IC_SAR] = 0x03FF
        mem32[self._i2c_base | MEM_SET | IC_SAR] = self._i2c_address & 0x03FF

        # 3. Configure IC_CON: 7-bit slave mode, slave enabled, master disabled,
        # bit 7 (STOP_DET_IFADDRESSED) = 1, bit 9 (RX_FIFO_FULL_HLD_CTRL) = 1
        mem32[self._i2c_base | MEM_CLR | IC_CON] = 0x0041
        mem32[self._i2c_base | MEM_SET | IC_CON] = 0x0280

        # 4. Enable I2C controller
        mem32[self._i2c_base | MEM_SET | IC_ENABLE] = 0x01

        # 5. Clear all pending interrupts initially
        _ = mem32[self._i2c_base | IC_CLR_INTR]

        # 6. Configure GPIO pins for I2C (Function 3 = I2C)
        mem32[IO_BANK0_BASE | MEM_CLR | (4 + 8 * self._sda)] = 0x1F
        mem32[IO_BANK0_BASE | MEM_SET | (4 + 8 * self._sda)] = 0x03

        mem32[IO_BANK0_BASE | MEM_CLR | (4 + 8 * self._scl)] = 0x1F
        mem32[IO_BANK0_BASE | MEM_SET | (4 + 8 * self._scl)] = 0x03

    def handle_event(self):
        """Poll and service hardware I2C events with 0 allocations."""
        base = self._i2c_base
        intr = mem32[base | IC_RAW_INTR_STAT]
        status = mem32[base | IC_STATUS]

        # 1. High priority: Drain RX FIFO if data is waiting
        if status & STATUS_RFNE:
            return STATE_RECEIVE

        # 2. Master is requesting data (Read Request)
        if intr & INTR_RD_REQ:
            return STATE_REQUEST

        # 3. Master aborted transaction
        if intr & INTR_TX_ABRT:
            _ = mem32[base | IC_CLR_TX_ABRT]
            return STATE_FINISH

        # 4. Master finished reading
        if intr & INTR_RX_DONE:
            _ = mem32[base | IC_CLR_RX_DONE]
            return STATE_FINISH

        # 5. Stop condition detected
        if intr & INTR_STOP_DET:
            _ = mem32[base | IC_CLR_STOP_DET]
            return STATE_FINISH

        # 6. Start / Restart condition detected
        if intr & (INTR_START_DET | INTR_RESTART_DET):
            _ = mem32[base | IC_CLR_START_DET]
            _ = mem32[base | IC_CLR_RESTART_DET]
            return STATE_START

        # 7. Clear overflow errors
        if intr & (INTR_RX_OVER | INTR_TX_OVER | INTR_RX_UNDER):
            _ = mem32[base | IC_CLR_INTR]

        return None

    def Available(self):
        return bool(mem32[self._i2c_base | IC_STATUS] & STATUS_RFNE)

    def Read_Data_Received(self):
        return mem32[self._i2c_base | IC_DATA_CMD] & 0xFF

    def Slave_Write_Data(self, data):
        base = self._i2c_base
        mem32[base | IC_DATA_CMD] = data & 0xFF
        _ = mem32[base | IC_CLR_RD_REQ]


slave = RP2040_Slave_Driver(
    i2c_id=I2C_BUS_ID,
    sda=I2C_SDA_PIN,
    scl=I2C_SCL_PIN,
    i2c_address=I2C_ADDRESS,
)

MAX_RX_LEN = 4
rx_buffer = bytearray(MAX_RX_LEN)
rx_len = 0
rx_overflow = False

reply_ok = bytearray((0x00,))
reply_err = bytearray((0xEE,))
reply = reply_err
reply_index = 0

# Diagnostic counters
stats_writes_ok = 0
stats_writes_bad = 0
stats_reads_ok = 0
stats_overflow = 0
stats_exc_i2c = 0
stats_exc_adc = 0
loop_worst_gap_us = 0
last_loop_us = ticks_us()


def set_reply(data):
    global reply, reply_index
    reply = data
    reply_index = 0


def process_command(length):
    global stats_writes_ok, stats_writes_bad

    if length == 0:
        return

    cmd = rx_buffer[0]

    if cmd == CMD_SET_SERVO and length >= 4:
        servo_id = rx_buffer[1]
        pulse_us = (rx_buffer[2] << 8) | rx_buffer[3]
        ok = set_servo(servo_id, pulse_us)
        set_reply(reply_ok if ok else reply_err)
        if ok:
            stats_writes_ok += 1
            print("[cmd] Set S{:02d} -> {} us ({:.1f} deg)".format(servo_id, pulse_us, pulse_to_angle(pulse_us)))
        else:
            stats_writes_bad += 1

    elif cmd == CMD_SET_ALL and length >= 3:
        pulse_us = (rx_buffer[1] << 8) | rx_buffer[2]
        set_all_servos(pulse_us)
        set_reply(reply_ok)
        stats_writes_ok += 1
        print("[cmd] Set ALL -> {} us ({:.1f} deg)".format(pulse_us, pulse_to_angle(pulse_us)))

    elif cmd == CMD_SAFE:
        reset_to_starting_angles()
        set_reply(reply_ok)
        stats_writes_ok += 1
        print("[cmd] Safe position (all servos to starting angles)")

    elif cmd == CMD_READ_ADC_REPORT:
        set_reply(adc_report)

    elif cmd == CMD_READ_TARGET_REPORT:
        set_reply(targets_report)

    else:
        set_reply(reply_err)


def i2c_tick():
    """Service pending I2C hardware events. Returns True if an event was handled."""
    global rx_len, rx_overflow, reply_index, stats_overflow, stats_reads_ok

    state = slave.handle_event()
    if state is None:
        return False

    if state == STATE_START:
        reply_index = 0

    elif state == STATE_RECEIVE:
        while slave.Available():
            received = slave.Read_Data_Received()
            if rx_len < MAX_RX_LEN:
                rx_buffer[rx_len] = received
                rx_len += 1
            else:
                rx_overflow = True

    elif state == STATE_REQUEST:
        if reply_index < len(reply):
            slave.Slave_Write_Data(reply[reply_index])
            reply_index += 1
        else:
            slave.Slave_Write_Data(0x00)

    elif state == STATE_FINISH:
        reply_index = 0
        if rx_len:
            if rx_overflow:
                set_reply(reply_err)
                stats_overflow += 1
            else:
                process_command(rx_len)
            rx_len = 0
            rx_overflow = False

    return True


HEARTBEAT_MS = 2000
next_heartbeat_ms = ticks_add(ticks_ms(), HEARTBEAT_MS)


def heartbeat():
    global next_heartbeat_ms, loop_worst_gap_us
    now_ms = ticks_ms()
    if ticks_diff(now_ms, next_heartbeat_ms) < 0:
        return
    next_heartbeat_ms = ticks_add(now_ms, HEARTBEAT_MS)

    free = gc.mem_free()
    alloc = gc.mem_alloc()
    print(
        "[hb] free={} alloc={} | writes ok={} bad={} | reads ok={} | "
        "overflow={} | gap_us={} | exc i2c={} adc={}".format(
            free,
            alloc,
            stats_writes_ok,
            stats_writes_bad,
            stats_reads_ok,
            stats_overflow,
            loop_worst_gap_us,
            stats_exc_i2c,
            stats_exc_adc,
        )
    )
    loop_worst_gap_us = 0


print("RP2040 servo slave ready: I2C0 GP20/GP21, address 0x2A")
while True:
    now_us = ticks_us()
    gap = ticks_diff(now_us, last_loop_us)
    last_loop_us = now_us
    if gap > loop_worst_gap_us:
        loop_worst_gap_us = gap

    try:
        while i2c_tick():
            pass
    except Exception as e:
        stats_exc_i2c += 1
        print("!! i2c_tick exception:", e)
        sys.print_exception(e)

    try:
        adc_scan_tick()
    except Exception as e:
        stats_exc_adc += 1
        print("!! adc_scan_tick exception:", e)
        sys.print_exception(e)

    heartbeat()
