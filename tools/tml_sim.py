#!/usr/bin/env python3
"""
tml_sim.py — Technosoft TML Drive Simulator

Simulates a Technosoft TML intelligent drive on a serial port so
the EPICS IOC (technosoft-asyn) can run without real hardware.

Usage:
    # Terminal 1: create a virtual serial pair
    socat -d -d pty,raw,echo=0,link=/var/tmp/ttyV0 \
                pty,raw,echo=0,link=/var/tmp/ttyV1

    # Terminal 2: start this simulator on one end
    python3 tml_sim.py /var/tmp/ttyV1

    # Terminal 3: start the IOC (it connects to /var/tmp/ttyV0)
    cd iocBoot/iocTml && ../../bin/linux-x86_64/technosoft ./st.cmd

Protocol reference:
    Frame: [LEN] [ADDR_HI][ADDR_LO] [OP_HI][OP_LO] [DATA...] [CHKSUM]
    LEN   = 2(addr) + 2(opcode) + 2*nDataWords
    CHKSUM = (sum of all bytes from LEN to last data) & 0xFF
    After each valid frame: drive sends ACK byte 0x4F
    For GiveMeData requests: drive sends ACK + response frame

    Opcodes (verified from TML Manual P091.055.MCII_.TML_.UM_.0806.pdf):
        GiveMeData16 = 0xB004   GiveMeData32 = 0xB005
        TakeData16   = 0xB404   TakeData32   = 0xB405
        WriteData16  = 0x9004   WriteData32  = 0x9005
        MCR_CONFIG   = 0x5909   SAP          = 0x8400
    GiveMeData data: [sender_axis_id, dm_address]
    TakeData data:   [sender_axis_id, dm_address_echo, value...]

Author: Andrea Michelotti — INFN-LNF (2026-02)
"""

import sys
import os
import struct
import time
import select
import signal
import termios
import tty

# ----------------------------------------------------------------
# TML protocol constants (must match tmlSerial.h)
# ----------------------------------------------------------------
ACK_BYTE  = 0x4F   # 'O'
SYNC_BYTE = 0x0D

# Opcodes (verified against TML Manual P091.055.MCII_.TML_.UM_.0806.pdf)
OP_GIVE_ME_DATA_16 = 0xB004
OP_GIVE_ME_DATA_32 = 0xB005
OP_TAKE_DATA_16    = 0xB404
OP_TAKE_DATA_32    = 0xB405

# Online WriteData protocol (from libtmlcomm.so SendData)
OP_WRITE_DATA_16 = 0x9004
OP_WRITE_DATA_32 = 0x9005

# Simple TML instructions (no data words)
OP_ENDINIT  = 0x0020
OP_AXISON   = 0x0102
OP_AXISOFF  = 0x0002
OP_STOP     = 0x01C4  # STOP3: decelerated
OP_STOP_IMM = 0x0104  # STOP0: voltage=0
OP_FAULTR   = 0x0402  # RESET (no dedicated fault-clear in TML ISA)
OP_UPD      = 0x0108  # Update on event
OP_UPD_IMM  = 0x0008  # Update immediate
OP_RESET    = 0x0402  # RESET

# MCR Configuration instruction (opcode + 2 data words: mask, value)
OP_MCR_CONFIG = 0x5909

# MCR CPA/CPR/MODE mask and value constants
MCR_CPA_MASK      = 0xFFFF;  MCR_CPA_VAL      = 0x2000
MCR_CPR_MASK      = 0xDFFF;  MCR_CPR_VAL      = 0x0000
MCR_MODE_PP3_MASK = 0xBFC1;  MCR_MODE_PP3_VAL = 0x8701
MCR_MODE_SP1_MASK = 0xBBC1;  MCR_MODE_SP1_VAL = 0x8301

# SAP: Set Actual Position (opcode + 2 data words: LOWORD, HIWORD)
OP_SAP = 0x8400

# STA: Set Target = Actual (opcode + 1 data word: APOS address)
OP_STA = 0x2CB2

# Well-known DM addresses (16-bit address space)
DM_APOS       = 0x0228
DM_TPOS       = 0x022A
DM_CPOS       = 0x022E
DM_CSPD       = 0x0230
DM_CACC       = 0x0232
DM_MCR        = 0x0908
DM_MSR        = 0x0909
DM_ISR        = 0x090A
DM_SRL        = 0x090E
DM_SRH        = 0x090F
DM_MER        = 0x0910
DM_SRL_MASK   = 0x0912
DM_SRH_MASK   = 0x0913
DM_MER_MASK   = 0x0914
DM_MASTERID   = 0x0916

# SRL bits
SRL_BIT_MOTION_COMPLETE = (1 << 10)
SRL_BIT_AXIS_ON         = (1 << 15)

# SRH bits
SRH_BIT_DRIVE_ON  = (1 << 15)

# ----------------------------------------------------------------
# Data memory model (32768 16-bit words)
# ----------------------------------------------------------------
class DriveSim:
    """Simulates one TML drive axis."""

    def __init__(self, axis_id=15):
        self.axis_id = axis_id
        self.dm = {}  # sparse: addr → 16-bit value

        # Initialise key registers
        self._set16(DM_MCR, 0x0000)
        self._set16(DM_MSR, 0x0000)
        self._set16(DM_ISR, 0x0000)
        self._set16(DM_SRL, SRL_BIT_MOTION_COMPLETE)  # idle, motion complete
        self._set16(DM_SRH, 0x0000)
        self._set16(DM_MER, 0x0000)
        self._set16(DM_SRL_MASK, 0x0000)
        self._set16(DM_SRH_MASK, 0x0000)
        self._set16(DM_MER_MASK, 0x0000)
        self._set16(DM_MASTERID, 0x00FF)
        self._set32(DM_APOS, 0)
        self._set32(DM_TPOS, 0)
        self._set32(DM_CPOS, 0)
        self._set32(DM_CSPD, 0)
        self._set32(DM_CACC, 0)

        self.powered = False
        self.moving = False
        self.target_pos = 0     # target in encoder counts
        self.actual_pos = 0     # actual in encoder counts
        self.speed = 0.0        # counts/poll-cycle simulation step
        self.move_direction = 1
        self.move_start_time = 0
        self.mode = 'idle'      # idle, position, velocity

    def _set16(self, addr, val):
        self.dm[addr] = val & 0xFFFF

    def _get16(self, addr):
        return self.dm.get(addr, 0) & 0xFFFF

    def _set32(self, addr, val):
        """Store 32-bit value in two consecutive 16-bit addresses (lo, hi)."""
        uval = val & 0xFFFFFFFF
        self.dm[addr]     = uval & 0xFFFF
        self.dm[addr + 1] = (uval >> 16) & 0xFFFF

    def _get32(self, addr):
        lo = self.dm.get(addr, 0) & 0xFFFF
        hi = self.dm.get(addr + 1, 0) & 0xFFFF
        return (hi << 16) | lo

    def update_status(self):
        """Recalculate SRL/SRH from internal state."""
        srl = self._get16(DM_SRL) & ~(SRL_BIT_MOTION_COMPLETE | SRL_BIT_AXIS_ON)
        srh = self._get16(DM_SRH) & ~SRH_BIT_DRIVE_ON

        if self.powered:
            srl |= SRL_BIT_AXIS_ON
            srh |= SRH_BIT_DRIVE_ON

        if not self.moving:
            srl |= SRL_BIT_MOTION_COMPLETE

        self._set16(DM_SRL, srl)
        self._set16(DM_SRH, srh)

        # Update APOS and TPOS in DM
        self._set32(DM_APOS, int(self.actual_pos) & 0xFFFFFFFF)
        self._set32(DM_TPOS, int(self.actual_pos) & 0xFFFFFFFF)

    def simulate_motion_step(self):
        """Called periodically to advance simulated position."""
        if not self.moving:
            return

        if self.mode == 'position':
            # Move towards target
            diff = self.target_pos - self.actual_pos
            if abs(diff) <= abs(self.speed):
                self.actual_pos = self.target_pos
                self.moving = False
                self.mode = 'idle'
            else:
                step = self.speed if diff > 0 else -self.speed
                self.actual_pos += step

        elif self.mode == 'velocity':
            # Continuous velocity move
            self.actual_pos += self.speed * self.move_direction

        self.update_status()

    def process_command(self, opcode, data_words):
        """Process a single TML command. Returns True if a response is needed."""
        dbg(f"  CMD opcode=0x{opcode:04X} data={[f'0x{w:04X}' for w in data_words]}")

        # --- WriteData16 (0x9004): data[0]=addr, data[1]=value ---
        if opcode == OP_WRITE_DATA_16:
            if len(data_words) >= 2:
                addr = data_words[0]
                val = data_words[1]
                self._set16(addr, val)
                dbg(f"  WriteData16 [0x{addr:04X}] = 0x{val:04X}")
            return False

        # --- WriteData32 (0x9005): data[0]=addr, data[1]=lo, data[2]=hi ---
        if opcode == OP_WRITE_DATA_32:
            if len(data_words) >= 3:
                addr = data_words[0]
                val32 = (data_words[2] << 16) | data_words[1]
                self._set32(addr, val32)
                dbg(f"  WriteData32 [0x{addr:04X}] = 0x{val32:08X} ({self._signed32(val32)})")

                if addr == DM_CSPD:
                    self.speed = max(1, abs(self._signed32(val32)) >> 14)
                    dbg(f"  -> speed step = {self.speed}")
            return False

        # --- MCR Configuration (0x5909): data[0]=mask, data[1]=value ---
        if opcode == OP_MCR_CONFIG:
            if len(data_words) >= 2:
                mask = data_words[0]
                val = data_words[1]
                dbg(f"  MCR CONFIG mask=0x{mask:04X} val=0x{val:04X}")

                # Decode which MCR command this is
                if mask == MCR_CPA_MASK and val == MCR_CPA_VAL:
                    pos32 = self._get32(DM_CPOS)
                    self.target_pos = self._signed32(pos32)
                    dbg(f"  -> CPA target={self.target_pos}")
                elif mask == MCR_CPR_MASK and val == MCR_CPR_VAL:
                    rel32 = self._get32(DM_CPOS)
                    rel = self._signed32(rel32)
                    self.target_pos = int(self.actual_pos) + rel
                    dbg(f"  -> CPR rel={rel} target={self.target_pos}")
                elif val == MCR_MODE_PP3_VAL:
                    self.mode = 'position'
                    dbg("  -> MODE PP3 (position profile)")
                elif val == MCR_MODE_SP1_VAL:
                    self.mode = 'velocity'
                    cspd = self._signed32(self._get32(DM_CSPD))
                    self.move_direction = 1 if cspd >= 0 else -1
                    dbg(f"  -> MODE SP1 (speed profile) dir={self.move_direction}")
                else:
                    dbg(f"  -> Unknown MCR config (mask=0x{mask:04X} val=0x{val:04X})")
            return False

        # --- SAP (0x8400): data[0]=LOWORD, data[1]=HIWORD ---
        if opcode == OP_SAP:
            if len(data_words) >= 2:
                pos32 = (data_words[1] << 16) | data_words[0]
                spos = self._signed32(pos32)
                self.actual_pos = spos
                self._set32(DM_APOS, pos32)
                dbg(f"  SAP position={spos}")
            return False

        # --- STA (0x2CB2): Set Target = Actual ---
        if opcode == OP_STA:
            self.target_pos = int(self.actual_pos)
            self._set32(DM_TPOS, self._get32(DM_APOS))
            dbg(f"  STA target={self.target_pos}")
            return False

        # --- Simple commands ---
        if opcode == OP_ENDINIT:
            dbg("  ENDINIT")
            return False
        if opcode == OP_AXISON:
            self.powered = True
            self.update_status()
            dbg("  AXIS ON")
            return False
        if opcode == OP_AXISOFF:
            self.powered = False
            self.moving = False
            self.mode = 'idle'
            self.update_status()
            dbg("  AXIS OFF")
            return False
        if opcode == OP_STOP or opcode == OP_STOP_IMM:
            self.moving = False
            self.mode = 'idle'
            self.update_status()
            dbg("  STOP")
            return False
        if opcode == OP_FAULTR:
            self._set16(DM_MER, 0)
            dbg("  FAULT RESET")
            return False
        if opcode == OP_UPD_IMM:
            # Trigger motion start
            if self.mode in ('position', 'velocity') and self.powered:
                self.moving = True
                self.move_start_time = time.time()
                if self.speed < 1:
                    self.speed = 1
                dbg(f"  UPD! → motion started (mode={self.mode} "
                    f"target={self.target_pos} speed={self.speed})")
            self.update_status()
            return False
        if opcode == OP_UPD:
            dbg("  UPD (on event)")
            return False
        if opcode == OP_RESET:
            dbg("  RESET")
            self.__init__(self.axis_id)
            return False

        dbg(f"  Unknown opcode 0x{opcode:04X} — ACK only")
        return False

    @staticmethod
    def _signed32(val):
        """Convert unsigned 32-bit to signed."""
        if val >= 0x80000000:
            return val - 0x100000000
        return val


# ----------------------------------------------------------------
# Protocol parser
# ----------------------------------------------------------------
VERBOSE = True

def dbg(msg):
    if VERBOSE:
        print(f"[TML-SIM] {msg}", flush=True)


def checksum(data):
    return sum(data) & 0xFF


def build_response_frame(addr, opcode, data_words):
    """Build a TML serial frame."""
    n_data = len(data_words)
    payload_len = 4 + 2 * n_data
    buf = bytearray()
    buf.append(payload_len)
    buf.append((addr >> 8) & 0xFF)
    buf.append(addr & 0xFF)
    buf.append((opcode >> 8) & 0xFF)
    buf.append(opcode & 0xFF)
    for w in data_words:
        buf.append((w >> 8) & 0xFF)
        buf.append(w & 0xFF)
    buf.append(checksum(buf))
    return bytes(buf)


def parse_frame(buf):
    """
    Try to parse a TML frame from buffer.
    Returns (addr, opcode, data_words, consumed_bytes) or None.
    """
    if len(buf) < 6:
        return None

    payload_len = buf[0]
    total_len = 1 + payload_len + 1  # len_byte + payload + checksum

    if len(buf) < total_len:
        return None

    # Verify checksum
    expected = checksum(buf[:total_len - 1])
    got = buf[total_len - 1]
    if expected != got:
        dbg(f"  Checksum FAIL: expected 0x{expected:02X} got 0x{got:02X}")
        return None

    addr   = (buf[1] << 8) | buf[2]
    opcode = (buf[3] << 8) | buf[4]
    n_data = (payload_len - 4) // 2
    data_words = []
    for i in range(n_data):
        off = 5 + 2 * i
        data_words.append((buf[off] << 8) | buf[off + 1])

    return (addr, opcode, data_words, total_len)


def run_simulator(port_path, axis_id=15):
    """Main simulator loop."""
    dbg(f"Opening {port_path} ...")

    fd = os.open(port_path, os.O_RDWR | os.O_NOCTTY)

    # Configure raw serial mode (8N2)
    try:
        attrs = termios.tcgetattr(fd)
        # Input flags: raw
        attrs[0] = 0
        # Output flags: raw
        attrs[1] = 0
        # Control flags: 8N2, CREAD, CLOCAL
        attrs[2] = termios.CS8 | termios.CSTOPB | termios.CREAD | termios.CLOCAL
        # Local flags: raw
        attrs[3] = 0
        # VMIN/VTIME
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except termios.error as e:
        dbg(f"termios setup warning: {e} (may be OK for pty)")

    drive = DriveSim(axis_id)
    buf = bytearray()
    last_motion_tick = time.time()

    dbg(f"TML Drive Simulator ready — axis_id={axis_id}")
    dbg("Waiting for commands...")

    running = True
    def handle_signal(sig, frame):
        nonlocal running
        running = False
        dbg("Shutting down...")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    while running:
        # Simulate motion every ~50ms
        now = time.time()
        if now - last_motion_tick >= 0.05:
            drive.simulate_motion_step()
            last_motion_tick = now

        # Wait for data with short timeout
        try:
            rlist, _, _ = select.select([fd], [], [], 0.02)
        except (OSError, ValueError):
            break

        if rlist:
            try:
                chunk = os.read(fd, 256)
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)

        # Check for SYNC byte (re-synchronisation request)
        while len(buf) > 0 and buf[0] == SYNC_BYTE:
            dbg("SYNC request → responding with SYNC")
            os.write(fd, bytes([SYNC_BYTE]))
            buf.pop(0)

        # Try to parse a frame
        while len(buf) >= 6:
            result = parse_frame(buf)
            if result is None:
                # Not enough data or bad checksum — skip first byte
                if len(buf) >= 6:
                    # Could be partial frame, need more data
                    break
                buf.pop(0)
                break

            addr, opcode, data_words, consumed = result
            buf = buf[consumed:]

            axis_from_addr = (addr >> 4) & 0xFF
            dbg(f"RX frame: addr=0x{addr:04X} (axis={axis_from_addr}) "
                f"opcode=0x{opcode:04X} nData={len(data_words)}")

            # Send ACK
            os.write(fd, bytes([ACK_BYTE]))
            dbg("  → ACK sent")

            # --- GiveMeData16 ---
            # Request data layout: data[0]=sender(host), data[1]=DM address
            if opcode == OP_GIVE_ME_DATA_16:
                if len(data_words) >= 2:
                    sender_id = data_words[0]   # host axis ID
                    read_addr = data_words[1]   # DM address
                    val = drive._get16(read_addr)
                    # Response: TakeData16 with [sender(drive), addr_echo, value]
                    resp = build_response_frame(
                        sender_id, OP_TAKE_DATA_16,
                        [drive.axis_id << 4, read_addr, val])
                    os.write(fd, resp)
                    dbg(f"  → TakeData16 [0x{read_addr:04X}] = 0x{val:04X}")

            # --- GiveMeData32 ---
            elif opcode == OP_GIVE_ME_DATA_32:
                if len(data_words) >= 2:
                    sender_id = data_words[0]   # host axis ID
                    read_addr = data_words[1]   # DM address
                    val32 = drive._get32(read_addr)
                    lo = val32 & 0xFFFF
                    hi = (val32 >> 16) & 0xFFFF
                    # Response: TakeData32 with [sender(drive), addr_echo, lo, hi]
                    resp = build_response_frame(
                        sender_id, OP_TAKE_DATA_32,
                        [drive.axis_id << 4, read_addr, lo, hi])
                    os.write(fd, resp)
                    dbg(f"  → TakeData32 [0x{read_addr:04X}] = 0x{val32:08X} "
                        f"({DriveSim._signed32(val32)})")

            # --- All other commands ---
            else:
                drive.process_command(opcode, data_words)

    os.close(fd)
    dbg("Simulator stopped.")


# ----------------------------------------------------------------
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial-port> [axis-id]")
        print(f"  e.g. {sys.argv[0]} /var/tmp/ttyV1 15")
        sys.exit(1)

    port = sys.argv[1]
    aid  = int(sys.argv[2]) if len(sys.argv) > 2 else 15
    VERBOSE = '--quiet' not in sys.argv

    run_simulator(port, aid)
