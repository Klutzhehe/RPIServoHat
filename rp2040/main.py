"""22-servo RP2040 MicroPython I2C slave application — DEBUG BUILD.

Same protocol/behavior as main.py, but instrumented to chase down the
read-path failures (I/O error right after reset, timeouts after ~1-2 min):

  - rx_buffer is pre-allocated once (no per-transaction bytearray alloc,
    which was the leading suspect for GC pauses landing mid-transaction)
  - automatic GC is disabled; collection only runs at a chosen bus-idle
    point (right after a write transaction finishes)
  - i2c_tick() / adc_scan_tick() are each wrapped so an exception gets
    printed with a full traceback instead of silently killing the whole
    slave loop (which would explain a later hard timeout)
  - lightweight counters/timers track: write/read counts, RX overflow
    and invalid-packet counts, GC pause duration, worst-case main-loop
    iteration gap, and worst-case duration of a full read transaction
  - NOTHING above prints from inside a live transaction. All output is
    either a rare exception print, or one heartbeat line every
    HEARTBEAT_MS from the idle main loop. Printing over USB serial is
    itself slow enough to make clock-stretching worse, so the hot path
    (I2C_RECEIVE / I2C_REQUEST) never touches print().

How to use:
  1. Copy this over rp2040/main.py (keep the original as a backup).
  2. Connect Thonny (or any serial terminal) to the Pico's REPL so you
     can watch the console live.
  3. From the Pi, run your normal `read` / `monitor` test through a
     failure, exactly as before.
  4. Watch the heartbeat lines. Things to look for:
       - free/alloc trending down then jumping back up = GC activity;
         correlate the jump timing against when the Pi-side call fails.
       - gc worst_us growing large = a collection that took a long time
         (long collections are more likely to land inside a request).
       - loop worst_gap_us spiking = something (not necessarily GC) is
         blocking the main loop for a while.
       - read_txn worst_us growing large or a read transaction that
         never reaches I2C_FINISH (reads ok stops incrementing while
         writes ok keeps climbing) = the RP2040 is stalling specifically
         mid-read, which is the read-path clock-stretch theory.
       - any "!! i2c_tick exception" or "!! adc_scan_tick exception"
         line = the firmware actually crashed/recovered there; if the
         loop stops producing heartbeats entirely after one of these,
         the slave loop died for good, which is the "firmware silently
         dying" theory instead.
"""

from machine import ADC, PWM, Pin, mem32
from time import ticks_add, ticks_diff, ticks_us, ticks_ms
import gc
import sys

# ---------------------------------------------------------------------------
# Hardware configuration (unchanged from main.py)
# ---------------------------------------------------------------------------
I2C_BUS_ID = 0
I2C_ADDRESS = 0x2A
I2C_SDA_PIN = 20
I2C_SCL_PIN = 21

SERVO_GPIOS = tuple(range(20)) + (22, 23)
SERVO_COUNT = len(SERVO_GPIOS)
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_START_US = 1000  # Safe position used at boot and by CMD_SAFE.
SERVO_PERIOD_US = 20000  # 50 Hz

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
# Binary protocol (unchanged)
# ---------------------------------------------------------------------------
CMD_SET_SERVO = 0x01  # [0x01, servo 0..21, pulse_us_hi, pulse_us_lo]
CMD_SET_ALL = 0x02    # [0x02, pulse_us_hi, pulse_us_lo]
CMD_SAFE = 0x03        # [0x03] -> 1000 us on every servo
CMD_READ_ADC_REPORT = 0x10  # [0x10], then master reads 36 bytes

REPORT_MAGIC = 0xA5
REPORT_VERSION = 0x01
REPORT_SIZE = 36  # magic, version, sequence, count, 16 x uint16

I2C_TX_FIFO_BATCH = 16  # Fill up to the RP2040 hardware I2C FIFO capacity

# Pre-allocated report buffer to avoid runtime GC allocations
adc_report = bytearray(REPORT_SIZE)
adc_report[0] = REPORT_MAGIC
adc_report[1] = REPORT_VERSION
adc_report[2] = 0
adc_report[3] = 16


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
# ---------------------------------------------------------------------------
adc_raw = [0] * 16
adc_sequence = 0
scan_mux = 0
scan_adc = 0
sample_sum = 0
sample_count = 0
next_adc_sample_at = 0
ADC_SAMPLES = 4
ADC_SAMPLE_GAP_US = 20
MUX_SETTLE_US = 100  # 100 µs settle time for high-speed current telemetry


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
# Debug instrumentation — no printing inside a live transaction, see notes
# at the top of the file.
# ---------------------------------------------------------------------------
HEARTBEAT_MS = 2000

import micropython

micropython.alloc_emergency_exception_buf(100)
gc.enable()  # Ensure MicroPython automatic GC is active

stats_writes_ok = 0
stats_writes_bad = 0
stats_reads_ok = 0
stats_overflow = 0
stats_invalid_packet = 0
stats_gc_collections = 0
stats_gc_last_freed = 0
stats_gc_worst_us = 0
stats_i2c_exceptions = 0
stats_adc_exceptions = 0

loop_last_tick_us = ticks_us()
loop_worst_gap_us = 0

read_txn_start_us = 0
read_txn_worst_us = 0

last_heartbeat_ms = ticks_ms()


def heartbeat():
    global loop_worst_gap_us, read_txn_worst_us
    free = gc.mem_free()
    alloc = gc.mem_alloc()
    print(
        "[hb] free={} alloc={} | writes ok={} bad={} | reads ok={} | "
        "overflow={} invalid={} | gc n={} last_freed={} worst_us={} | "
        "loop worst_gap_us={} | read_txn worst_us={} | exc i2c={} adc={}".format(
            free, alloc,
            stats_writes_ok, stats_writes_bad,
            stats_reads_ok,
            stats_overflow, stats_invalid_packet,
            stats_gc_collections, stats_gc_last_freed, stats_gc_worst_us,
            loop_worst_gap_us,
            read_txn_worst_us,
            stats_i2c_exceptions, stats_adc_exceptions,
        )
    )
    loop_worst_gap_us = 0
    read_txn_worst_us = 0


def safe_collect():
    """Run GC at a point we know the bus is idle (right after a write
    transaction finishes), and time how long it took."""
    global stats_gc_collections, stats_gc_last_freed, stats_gc_worst_us
    before = gc.mem_free()
    t0 = ticks_us()
    gc.collect()
    dt = ticks_diff(ticks_us(), t0)
    after = gc.mem_free()
    stats_gc_collections += 1
    stats_gc_last_freed = after - before
    if dt > stats_gc_worst_us:
        stats_gc_worst_us = dt


# ---------------------------------------------------------------------------
# Self-contained Zero-Allocation RP2040 I2C Slave Driver
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

        # 3. Configure IC_CON: 7-bit slave mode, slave enabled, master disabled, clock stretching enabled
        mem32[self._i2c_base | MEM_CLR | IC_CON] = 0x0041
        mem32[self._i2c_base | MEM_SET | IC_CON] = 0x0200

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
        intr = mem32[base | IC_INTR_STAT]
        status = mem32[base | IC_STATUS]

        # 1. PRIORITY: If RX FIFO has data, service it FIRST before STOP
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

        # 6. Start condition detected
        if intr & INTR_START_DET:
            _ = mem32[base | IC_CLR_START_DET]
            return STATE_START

        # 7. Restart condition detected
        if intr & INTR_RESTART_DET:
            _ = mem32[base | IC_CLR_RESTART_DET]
            return STATE_START

        # 8. Clear overflow errors
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
rx_buffer = bytearray(MAX_RX_LEN)  # pre-allocated once, no per-txn alloc
rx_len = 0
rx_overflow = False

reply_ok = bytearray((0x00,))
reply_err = bytearray((0xEE,))
reply = reply_err
reply_index = 0


def set_reply(data):
    global reply, reply_index
    reply = data
    reply_index = 0


def process_command(length):
    global stats_writes_ok, stats_writes_bad, stats_invalid_packet

    if length == 0:
        return

    cmd = rx_buffer[0]
    print("[cmd] rx_len={} cmd=0x{:02X}".format(length, cmd))

    if cmd == CMD_SET_SERVO and length >= 4:
        pulse_us = (rx_buffer[2] << 8) | rx_buffer[3]
        ok = set_servo(rx_buffer[1], pulse_us)
        set_reply(reply_ok if ok else reply_err)
        if ok:
            stats_writes_ok += 1
        else:
            stats_writes_bad += 1

    elif cmd == CMD_SET_ALL and length >= 3:
        set_all_servos((rx_buffer[1] << 8) | rx_buffer[2])
        set_reply(reply_ok)
        stats_writes_ok += 1

    elif cmd == CMD_SAFE:
        set_all_servos(SERVO_START_US)
        set_reply(reply_ok)
        stats_writes_ok += 1

    elif cmd == CMD_READ_ADC_REPORT:
        print("[cmd] staged ADC report (len={}) magic=0x{:02X}".format(len(adc_report), adc_report[0]))
        set_reply(adc_report)

    else:
        set_reply(reply_err)
        stats_invalid_packet += 1


def i2c_tick():
    global rx_len, rx_overflow, reply_index
    global stats_overflow, stats_reads_ok
    global read_txn_start_us, read_txn_worst_us

    state = slave.handle_event()

    if state == STATE_START:
        reply_index = 0
        if reply is adc_report:
            read_txn_start_us = ticks_us()

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
        if reply is adc_report and read_txn_start_us:
            dt = ticks_diff(ticks_us(), read_txn_start_us)
            if dt > read_txn_worst_us:
                read_txn_worst_us = dt
            stats_reads_ok += 1
            read_txn_start_us = 0

        if rx_len:
            if rx_overflow:
                set_reply(reply_err)
                stats_overflow += 1
            else:
                process_command(rx_len)
            rx_len = 0
            rx_overflow = False
            safe_collect()





alloc_i2c_total = 0
alloc_adc_total = 0
alloc_other_total = 0


def heartbeat():
    global loop_worst_gap_us, read_txn_worst_us
    global alloc_i2c_total, alloc_adc_total, alloc_other_total
    free = gc.mem_free()
    alloc = gc.mem_alloc()
    print(
        "[hb] free={} alloc={} | alloc_bytes: i2c={} adc={} other={} | writes ok={} | reads ok={} | gap_us={}".format(
            free, alloc,
            alloc_i2c_total, alloc_adc_total, alloc_other_total,
            stats_writes_ok, stats_reads_ok,
            loop_worst_gap_us,
        )
    )
    alloc_i2c_total = 0
    alloc_adc_total = 0
    alloc_other_total = 0
    loop_worst_gap_us = 0
    read_txn_worst_us = 0


try:
    print("RP2040 servo slave ready (debug build): I2C0 GP20/GP21, address 0x2A")

    while True:
        now = ticks_us()
        gap = ticks_diff(now, loop_last_tick_us)
        if gap > loop_worst_gap_us:
            loop_worst_gap_us = gap
        loop_last_tick_us = now

        _a0 = gc.mem_alloc()
        try:
            i2c_tick()
        except Exception as e:
            stats_i2c_exceptions += 1
            print("!! i2c_tick exception:")
            sys.print_exception(e)
        _a1 = gc.mem_alloc()
        if _a1 > _a0:
            alloc_i2c_total += (_a1 - _a0)

        try:
            adc_scan_tick()
        except Exception as e:
            stats_adc_exceptions += 1
            print("!! adc_scan_tick exception:")
            sys.print_exception(e)
        _a2 = gc.mem_alloc()
        if _a2 > _a1:
            alloc_adc_total += (_a2 - _a1)

        if ticks_diff(ticks_ms(), last_heartbeat_ms) >= HEARTBEAT_MS:
            heartbeat()
            last_heartbeat_ms = ticks_ms()

except KeyboardInterrupt:
    print("RP2040 slave stopped.")
