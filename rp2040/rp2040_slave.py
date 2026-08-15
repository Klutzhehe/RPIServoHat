#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""High-performance, zero-allocation RP2040 I2C Slave driver for MicroPython.

Fixes the original library issues:
  1. Priority order bug: RFNE (RX FIFO Not Empty) is now serviced BEFORE
     STOP_DET so incoming bytes are never trapped in the RX FIFO.
  2. Zero heap allocations: All register offsets and bitmasks are compile-time
     integer constants instead of runtime dict/list-comprehensions.
     Eliminates all garbage collection pauses (0 KB/s memory allocation).
  3. Complete error clearance: Clears RX_OVER, TX_ABRT, and TX_OVER so the
     RP2040 hardware I2C controller never hangs or refuses ACKs after a transaction.
"""

from machine import mem32

# ---------------------------------------------------------------------------
# RP2040 Hardware Register Constants (from RP2040 Datasheet)
# ---------------------------------------------------------------------------
IO_BANK0_BASE = 0x40014000
I2C0_BASE     = 0x40044000
I2C1_BASE     = 0x40048000

# Atomic Register Access Aliases
MEM_RW  = 0x0000
MEM_XOR = 0x1000
MEM_SET = 0x2000
MEM_CLR = 0x3000

# Register Offsets
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

# IC_INTR_STAT / IC_RAW_INTR_STAT bitmasks
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

# IC_STATUS bitmasks
STATUS_ACTIVITY  = 0x01
STATUS_TFNF      = 0x02  # TX FIFO Not Full
STATUS_TFE       = 0x04  # TX FIFO Empty
STATUS_RFNE      = 0x08  # RX FIFO Not Empty
STATUS_RFF       = 0x10  # RX FIFO Full


class RP2040_Slave:
    class I2CStateMachine:
        I2C_RECEIVE = 0
        I2C_REQUEST = 1
        I2C_FINISH  = 2
        I2C_START   = 3

    def __init__(self, i2c_id=0, sda=20, scl=21, i2c_address=0x2A):
        self._scl = scl
        self._sda = sda
        self._i2c_address = i2c_address
        self._i2c_base = I2C0_BASE if i2c_id == 0 else I2C1_BASE

        # 1. Disable the I2C controller to configure registers
        mem32[self._i2c_base | MEM_CLR | IC_ENABLE] = 0x01

        # 2. Set slave address in IC_SAR (bits 9:0)
        mem32[self._i2c_base | MEM_CLR | IC_SAR] = 0x03FF
        mem32[self._i2c_base | MEM_SET | IC_SAR] = self._i2c_address & 0x03FF

        # 3. Configure IC_CON: 7-bit slave mode, slave enabled, master disabled, clock stretching enabled
        # Bit 0 (MASTER_MODE) = 0, Bit 6 (IC_SLAVE_DISABLE) = 0, Bit 9 (RX_FIFO_FULL_HLD_CTRL) = 1
        mem32[self._i2c_base | MEM_CLR | IC_CON] = 0x0041  # clear MASTER_MODE & IC_SLAVE_DISABLE
        mem32[self._i2c_base | MEM_SET | IC_CON] = 0x0200  # enable RX_FIFO_FULL_HLD_CTRL (clock stretch)

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
        """Poll and service hardware I2C events with zero allocations."""
        base = self._i2c_base
        intr = mem32[base | IC_INTR_STAT]
        status = mem32[base | IC_STATUS]

        # 1. PRIORITY: If RX FIFO has data, service it FIRST before handling STOP
        if status & STATUS_RFNE:
            return self.I2CStateMachine.I2C_RECEIVE

        # 2. Master is requesting data (Read Request)
        if intr & INTR_RD_REQ:
            return self.I2CStateMachine.I2C_REQUEST

        # 3. Master aborted transaction
        if intr & INTR_TX_ABRT:
            _ = mem32[base | IC_CLR_TX_ABRT]
            return self.I2CStateMachine.I2C_FINISH

        # 4. Master finished reading (NACK received after slave transmit)
        if intr & INTR_RX_DONE:
            _ = mem32[base | IC_CLR_RX_DONE]
            return self.I2CStateMachine.I2C_FINISH

        # 5. Stop condition detected
        if intr & INTR_STOP_DET:
            _ = mem32[base | IC_CLR_STOP_DET]
            return self.I2CStateMachine.I2C_FINISH

        # 6. Start condition detected
        if intr & INTR_START_DET:
            _ = mem32[base | IC_CLR_START_DET]
            return self.I2CStateMachine.I2C_START

        # 7. Restart condition detected
        if intr & INTR_RESTART_DET:
            _ = mem32[base | IC_CLR_RESTART_DET]
            return self.I2CStateMachine.I2C_START

        # 8. Clear any overflow/underflow errors so the hardware never hangs
        if intr & (INTR_RX_OVER | INTR_TX_OVER | INTR_RX_UNDER):
            _ = mem32[base | IC_CLR_INTR]

        return None

    def Available(self):
        """Return True if RX FIFO has received data."""
        return bool(mem32[self._i2c_base | IC_STATUS] & STATUS_RFNE)

    def Read_Data_Received(self):
        """Read one byte from RX FIFO."""
        return mem32[self._i2c_base | IC_DATA_CMD] & 0xFF

    def Slave_Write_Data(self, data):
        """Write one byte to TX FIFO and acknowledge read request."""
        base = self._i2c_base
        mem32[base | IC_DATA_CMD] = data & 0xFF
        _ = mem32[base | IC_CLR_RD_REQ]
