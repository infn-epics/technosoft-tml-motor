#!/usr/bin/env python3
"""
Moxa NPort  ←→  TML Drive Diagnostic Tool.

Tests raw TML serial communication through a Moxa NPort (transparent
TCP-to-serial bridge).  Unlike xport_diag.py, this does NOT send
XPORT-specific probes (sync 0x0D echo) that would corrupt the RS-485 bus.

Usage:
    python3 moxa_diag.py <MOXA_IP> [PORT] [AXIS_ID] [HOST_ID]

    PORT     TCP port (default: 4001 = serial port 1)
    AXIS_ID  TML axis ID of one drive to probe (default: 10)
    HOST_ID  Host address, must differ from all axis IDs (default: 1)

Examples:
    python3 moxa_diag.py 192.168.190.55
    python3 moxa_diag.py 192.168.190.55 4001 10 1
    python3 moxa_diag.py 192.168.190.55 4001 3  1
"""

import sys
import socket
import struct
import time

DEFAULT_PORT = 4001
DEFAULT_AXIS = 10
DEFAULT_HOST = 1
TIMEOUT = 3.0


# ── Frame helpers ────────────────────────────────────────────────────

def tml_checksum(buf: bytes) -> int:
    return sum(buf) & 0xFF


def hex_str(data: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in data)


def build_tml_frame(axis_id: int, opcode: int, data_words=None) -> bytes:
    """Build a TML serial frame:  [LEN] [ADDR_HI LO] [OP_HI LO] [DATA...] [CHKSUM]"""
    addr = axis_id << 4
    if data_words:
        fmt = '>HH' + 'H' * len(data_words)
        payload = struct.pack(fmt, addr, opcode, *data_words)
    else:
        payload = struct.pack('>HH', addr, opcode)
    length = len(payload)
    frame = bytes([length]) + payload
    frame += bytes([tml_checksum(frame)])
    return frame


def build_endinit(axis_id: int) -> bytes:
    """ENDINIT (0x0020) — validate drive initialisation."""
    return build_tml_frame(axis_id, 0x0020)


def build_axison(axis_id: int) -> bytes:
    """AXISON (power stage ON)."""
    return build_tml_frame(axis_id, 0x0921)


def build_axisoff(axis_id: int) -> bytes:
    """AXISOFF (power stage OFF)."""
    return build_tml_frame(axis_id, 0x0922)


def build_give_me_data_16(axis_id: int, host_id: int, dm_addr: int) -> bytes:
    """GiveMeData 16-bit read from DM address.
    Opcode = 0xB001 (TML_OP_GIVE_ME_DATA)
    Data[0] = return address = (hostId << 4) | 0x0001
    Data[1] = DM address to read
    """
    sender = (host_id << 4) | 0x0001
    return build_tml_frame(axis_id, 0xB001, [sender, dm_addr])


def build_give_me_data_32(axis_id: int, host_id: int, dm_addr: int) -> bytes:
    """GiveMeData 32-bit (long) read."""
    sender = (host_id << 4) | 0x0001
    return build_tml_frame(axis_id, 0xB002, [sender, dm_addr])


# ── Known DM addresses (standard TML) ───────────────────────────────

DM_SRL  = 0x020E   # Status register low
DM_SRH  = 0x020F   # Status register high
DM_MER  = 0x0210   # Motion error register
DM_APOS = 0x0228   # Actual position (32-bit)


# ── Test functions ────────────────────────────────────────────────────

def make_socket(host: str, port: int, settle_ms: int = 300) -> socket.socket:
    """Connect to Moxa, wait for serial bridge to settle, drain stale bytes."""
    sock = socket.create_connection((host, port), timeout=TIMEOUT)
    sock.settimeout(TIMEOUT)
    time.sleep(settle_ms / 1000.0)
    # Drain any stale bytes from previous sessions
    sock.setblocking(False)
    try:
        stale = sock.recv(256)
        if stale:
            print(f"    Drained {len(stale)} stale bytes: {hex_str(stale)}")
    except BlockingIOError:
        pass
    sock.setblocking(True)
    sock.settimeout(TIMEOUT)
    return sock


def send_and_recv(sock: socket.socket, frame: bytes, label: str,
                  timeout: float = 2.0) -> bytes:
    """Send a frame and try to receive a response."""
    print(f"    TX ({label}): {hex_str(frame)}")
    sock.sendall(frame)
    sock.settimeout(timeout)
    try:
        data = sock.recv(256)
        print(f"    RX: {hex_str(data)}  ({len(data)} bytes)")
        return data
    except socket.timeout:
        print(f"    RX: <no response in {timeout:.1f}s>")
        return b''


def test_tcp(host: str, port: int) -> bool:
    """Step 1: Test raw TCP connectivity to Moxa."""
    print(f"\n{'='*60}")
    print(f"[1] TCP CONNECT to {host}:{port}")
    print(f"{'='*60}")
    try:
        t0 = time.time()
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        dt = (time.time() - t0) * 1000
        print(f"    OK — connected in {dt:.0f} ms (fd={sock.fileno()})")
        # Check for unsolicited data
        sock.settimeout(0.5)
        try:
            data = sock.recv(256)
            print(f"    Unsolicited data: {hex_str(data)}")
            print(f"    WARNING: Moxa sent data on connect — check 'TCP Server Mode'")
        except socket.timeout:
            print(f"    No unsolicited data — good (transparent mode)")
        sock.close()
        return True
    except Exception as e:
        print(f"    FAILED: {e}")
        print(f"    → Check: Moxa powered on? IP correct? TCP port {port} enabled?")
        return False


def test_endinit(host: str, port: int, axis_id: int) -> bool:
    """Step 2: Send ENDINIT and look for ACK (0x4F = 'O')."""
    print(f"\n{'='*60}")
    print(f"[2] TML ENDINIT to axis {axis_id}  (addr=0x{axis_id<<4:04X})")
    print(f"{'='*60}")
    try:
        sock = make_socket(host, port)
        frame = build_endinit(axis_id)
        data = send_and_recv(sock, frame, "ENDINIT")
        sock.close()

        if not data:
            print(f"\n    DIAGNOSIS: Drive did not respond at all.")
            print(f"    Possible causes:")
            print(f"      1. Moxa serial settings WRONG (need: 9600, 8N2, no flow ctrl)")
            print(f"      2. RS-485 wiring issue (A/B swapped, termination)")
            print(f"      3. Drive not powered or wrong axis ID (expected {axis_id})")
            print(f"      4. Moxa 'Operation Mode' must be 'TCP Server'")
            return False

        if 0x4F in data:
            print(f"\n    ✓ ACK received (0x4F) — drive is alive and responding!")
            return True
        else:
            print(f"\n    Got {len(data)} bytes but no ACK (0x4F).")
            print(f"    Could be: wrong baud rate, parity, or axis ID.")
            return False

    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_read_status(host: str, port: int, axis_id: int, host_id: int) -> bool:
    """Step 3: GiveMeData read SRL (status register low)."""
    print(f"\n{'='*60}")
    print(f"[3] READ STATUS (SRL) from axis {axis_id}  (hostId={host_id})")
    print(f"{'='*60}")
    try:
        sock = make_socket(host, port)
        frame = build_give_me_data_16(axis_id, host_id, DM_SRL)
        data = send_and_recv(sock, frame, "GiveMeData SRL")
        sock.close()

        if not data:
            print(f"    No response — drive not reachable or serial config wrong.")
            return False

        # Should get ACK (0x4F) then a reply frame
        if 0x4F in data:
            print(f"    ✓ ACK received")
            # Try to parse reply frame (after ACK byte)
            ack_idx = data.index(0x4F)
            reply = data[ack_idx + 1:]
            if len(reply) >= 6:
                reply_len = reply[0]
                if len(reply) >= 1 + reply_len + 1:
                    reply_addr = (reply[1] << 8) | reply[2]
                    reply_op = (reply[3] << 8) | reply[4]
                    if len(reply) >= 7:
                        status = (reply[5] << 8) | reply[6]
                        print(f"    Reply: addr=0x{reply_addr:04X} op=0x{reply_op:04X} SRL=0x{status:04X}")
                        print(f"    ✓ Drive communication fully working!")
                        return True
            print(f"    ACK received but reply frame incomplete — try longer timeout")
        else:
            print(f"    Got bytes but no ACK — framing issue")
        return False

    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_read_position(host: str, port: int, axis_id: int, host_id: int) -> bool:
    """Step 4: GiveMeData read APOS (32-bit actual position)."""
    print(f"\n{'='*60}")
    print(f"[4] READ POSITION (APOS) from axis {axis_id}  (hostId={host_id})")
    print(f"{'='*60}")
    try:
        sock = make_socket(host, port)
        frame = build_give_me_data_32(axis_id, host_id, DM_APOS)
        data = send_and_recv(sock, frame, "GiveMeData APOS")
        sock.close()

        if not data:
            print(f"    No response.")
            return False

        if 0x4F in data:
            print(f"    ✓ ACK received")
            ack_idx = data.index(0x4F)
            reply = data[ack_idx + 1:]
            if len(reply) >= 8:
                lo = (reply[5] << 8) | reply[6]
                hi = (reply[7] << 8) | reply[8] if len(reply) >= 9 else 0
                pos = (hi << 16) | lo
                if pos >= 0x80000000:
                    pos -= 0x100000000
                print(f"    APOS = {pos} counts")
                return True
            print(f"    ACK but incomplete reply")
        return False

    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_all_axes(host: str, port: int, host_id: int) -> None:
    """Step 5: Quick scan — send ENDINIT to every axis ID 1-15 and report who ACKs."""
    print(f"\n{'='*60}")
    print(f"[5] BUS SCAN — probing axis IDs 1-15  (skipping hostId={host_id})")
    print(f"{'='*60}")

    found = []
    for axis_id in range(1, 16):
        if axis_id == host_id:
            print(f"    Axis {axis_id:2d}: SKIP (= hostId)")
            continue
        try:
            sock = make_socket(host, port, settle_ms=100)
            frame = build_endinit(axis_id)
            sock.sendall(frame)
            sock.settimeout(0.5)
            try:
                data = sock.recv(64)
                if 0x4F in data:
                    print(f"    Axis {axis_id:2d}: ✓ ACK")
                    found.append(axis_id)
                else:
                    print(f"    Axis {axis_id:2d}: ? got {hex_str(data)}")
            except socket.timeout:
                print(f"    Axis {axis_id:2d}: — no response")
            sock.close()
        except Exception as e:
            print(f"    Axis {axis_id:2d}: ERROR {e}")

    print(f"\n    Found {len(found)} responding axes: {found if found else 'NONE'}")
    if not found:
        print(f"\n    !! No drives responded. Check Moxa serial settings:")
        print(f"       Baud=9600  Data=8  Parity=None  Stop=2  Flow=None")
        print(f"       Interface=RS-485 2-wire  Mode=TCP Server")


# ── Main ─────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(__doc__)
        sys.exit(0)

    host    = sys.argv[1]
    port    = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    axis_id = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_AXIS
    host_id = int(sys.argv[4]) if len(sys.argv) > 4 else DEFAULT_HOST

    print(f"Moxa NPort TML Diagnostic")
    print(f"  Target:  {host}:{port}")
    print(f"  Axis ID: {axis_id}   Host ID: {host_id}")
    print(f"  Expected serial: 9600, 8N2, no flow ctrl")

    if host_id == axis_id:
        print(f"\n  WARNING: hostId ({host_id}) == axisId ({axis_id}) — "
              f"they MUST differ for RS-485!")

    # Run tests in sequence — stop early if TCP fails
    if not test_tcp(host, port):
        print(f"\nABORT: Cannot connect to Moxa. Fix network first.")
        sys.exit(1)

    if not test_endinit(host, port, axis_id):
        # ENDINIT failed — do bus scan to see if ANY drive responds
        test_all_axes(host, port, host_id)
        sys.exit(1)

    # ENDINIT worked — try reading data
    test_read_status(host, port, axis_id, host_id)
    test_read_position(host, port, axis_id, host_id)

    print(f"\n{'='*60}")
    print(f"DONE — all tests completed")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
