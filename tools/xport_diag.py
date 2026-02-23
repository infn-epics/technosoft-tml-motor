#!/usr/bin/env python3
"""
XPORT Diagnostic Tool for Technosoft TML drives.

Probes a Digi XPORT ethernet-to-serial bridge and the connected TML drive
to diagnose communication problems.

Usage:
    python3 xport_diag.py <XPORT_IP> [DATA_PORT] [CONFIG_PORT]

Example:
    python3 xport_diag.py 192.168.190.55 4001
"""

import sys
import socket
import struct
import time

DEFAULT_DATA_PORT = 4001
DEFAULT_WEB_PORT = 80
DEFAULT_CONFIG_PORT = 9999  # Digi XPORT telnet config port
TIMEOUT = 2.0


def tml_checksum(buf: bytes) -> int:
    """TML checksum: low byte of sum of all preceding bytes."""
    return sum(buf) & 0xFF


def build_endinit(axis_id: int) -> bytes:
    """Build a TML ENDINIT frame for the given axis."""
    addr = axis_id << 4
    opcode = 0x0020  # ENDINIT
    payload = struct.pack('>HH', addr, opcode)
    length = len(payload)
    frame = bytes([length]) + payload
    frame += bytes([tml_checksum(frame)])
    return frame


def build_give_me_data_apos(axis_id: int, host_id: int) -> bytes:
    """Build a GiveMeData request for APOS (actual position)."""
    addr = axis_id << 4
    opcode = 0xB002  # GiveMeData LONG
    sender = (host_id << 4) | 0x0001
    apos_addr = 0x0228  # standard APOS DM address
    payload = struct.pack('>HHHH', addr, opcode, sender, apos_addr)
    length = len(payload)
    frame = bytes([length]) + payload
    frame += bytes([tml_checksum(frame)])
    return frame


def hex_str(data: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in data)


def test_tcp_connect(host: str, port: int) -> bool:
    """Test raw TCP connectivity."""
    print(f"\n[1] TCP CONNECT to {host}:{port}")
    try:
        t0 = time.time()
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        dt = (time.time() - t0) * 1000
        print(f"    OK — connected in {dt:.0f} ms")
        sock.close()
        return True
    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_unsolicited_data(host: str, port: int) -> bytes:
    """Check if XPORT sends anything on connect (banner, etc.)."""
    print(f"\n[2] CHECK FOR UNSOLICITED DATA after connect")
    try:
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        sock.settimeout(1.0)
        try:
            data = sock.recv(256)
            print(f"    Got {len(data)} bytes: {hex_str(data)}")
            sock.close()
            return data
        except socket.timeout:
            print(f"    No data received (1s) — XPORT is in transparent mode (expected)")
            sock.close()
            return b''
    except Exception as e:
        print(f"    FAILED: {e}")
        return b''


def test_sync_echo(host: str, port: int) -> bool:
    """Send TML sync byte (0x0D) and check for echo."""
    print(f"\n[3] TML SYNC BYTE (0x0D) echo test")
    try:
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        sock.settimeout(1.0)

        # Wait for XPORT to stabilize
        time.sleep(0.2)
        # Drain stale
        try:
            sock.recv(256)
        except socket.timeout:
            pass

        sock.sendall(b'\x0D')
        print(f"    TX: 0D")
        try:
            data = sock.recv(64)
            print(f"    RX: {hex_str(data)}")
            if b'\x0D' in data:
                print(f"    RESULT: sync echo detected — drive is connected and responding")
                sock.close()
                return True
            else:
                print(f"    RESULT: got non-sync response — investigate")
                sock.close()
                return True
        except socket.timeout:
            print(f"    RESULT: no echo — drive may not be connected or serial settings mismatch")
            sock.close()
            return False
    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_endinit(host: str, port: int, axis_id: int) -> bool:
    """Send ENDINIT and look for ACK (0x4F)."""
    frame = build_endinit(axis_id)
    print(f"\n[4] TML ENDINIT to axis {axis_id} (addr=0x{axis_id << 4:04X})")
    print(f"    TX: {hex_str(frame)}")
    try:
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        sock.settimeout(2.0)

        time.sleep(0.3)  # XPORT settle
        # Drain stale
        try:
            sock.recv(256)
        except socket.timeout:
            pass

        sock.sendall(frame)
        try:
            data = sock.recv(64)
            print(f"    RX: {hex_str(data)}")
            if 0x4F in data:
                print(f"    RESULT: ACK (0x4F) received — drive is alive!")
                sock.close()
                return True
            else:
                print(f"    RESULT: got response but no ACK (0x4F) — possible protocol issue")
                sock.close()
                return False
        except socket.timeout:
            print(f"    RESULT: no response to ENDINIT (2s timeout)")
            sock.close()
            return False
    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_give_me_data(host: str, port: int, axis_id: int, host_id: int) -> bool:
    """Send GiveMeData APOS and look for any response."""
    frame = build_give_me_data_apos(axis_id, host_id)
    print(f"\n[5] TML GiveMeData APOS (axis={axis_id}, host={host_id})")
    print(f"    TX: {hex_str(frame)}")
    try:
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        sock.settimeout(2.0)

        time.sleep(0.3)
        try:
            sock.recv(256)
        except socket.timeout:
            pass

        sock.sendall(frame)
        try:
            data = sock.recv(64)
            print(f"    RX: {hex_str(data)}")
            print(f"    RESULT: got {len(data)} bytes — drive responded!")
            sock.close()
            return True
        except socket.timeout:
            print(f"    RESULT: no response (2s timeout)")
            sock.close()
            return False
    except Exception as e:
        print(f"    FAILED: {e}")
        return False


def test_raw_bytes(host: str, port: int) -> None:
    """Send various byte patterns and see if anything echoes."""
    print(f"\n[6] RAW BYTE LOOPBACK TEST (AA 55 FF 00)")
    try:
        sock = socket.create_connection((host, port), timeout=TIMEOUT)
        sock.settimeout(1.0)
        time.sleep(0.3)
        try:
            sock.recv(256)
        except socket.timeout:
            pass

        test_bytes = b'\xAA\x55\xFF\x00'
        sock.sendall(test_bytes)
        print(f"    TX: {hex_str(test_bytes)}")
        try:
            data = sock.recv(64)
            print(f"    RX: {hex_str(data)}")
            if data == test_bytes:
                print(f"    RESULT: exact loopback — XPORT may be in loopback/echo mode!")
            else:
                print(f"    RESULT: got different data — possible drive response or corruption")
        except socket.timeout:
            print(f"    RESULT: no echo — bytes are being sent to serial port (good)")
        sock.close()
    except Exception as e:
        print(f"    FAILED: {e}")


def test_web_interface(host: str, port: int = 80) -> None:
    """Try to fetch XPORT web config (Digi devices usually have port 80)."""
    print(f"\n[7] XPORT WEB INTERFACE check (http://{host}:{port}/)")
    try:
        sock = socket.create_connection((host, port), timeout=2.0)
        sock.settimeout(2.0)
        sock.sendall(b"GET / HTTP/1.0\r\nHost: " + host.encode() + b"\r\n\r\n")
        resp = b''
        while True:
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                resp += chunk
            except socket.timeout:
                break
        sock.close()
        if resp:
            text = resp.decode('latin-1', errors='replace')
            print(f"    OK — got {len(resp)} bytes response")
            # Look for serial config info
            for keyword in ['baud', 'serial', 'parity', 'stop', 'flow', 'port']:
                lines = [l.strip() for l in text.split('\n') if keyword.lower() in l.lower()]
                for l in lines[:3]:
                    print(f"    >> {l[:120]}")
            if not any(kw in text.lower() for kw in ['baud', 'serial']):
                print(f"    (No obvious serial config found in root page)")
                print(f"    Try: http://{host}/serial or http://{host}/setup in a browser")
        else:
            print(f"    Empty response")
    except Exception as e:
        print(f"    No web interface: {e}")


def test_config_port(host: str, port: int = 9999) -> None:
    """Try the Digi XPORT telnet configuration port."""
    print(f"\n[8] XPORT CONFIG PORT ({host}:{port})")
    try:
        sock = socket.create_connection((host, port), timeout=2.0)
        sock.settimeout(2.0)
        try:
            data = sock.recv(1024)
            text = data.decode('latin-1', errors='replace')
            print(f"    Got {len(data)} bytes: {text[:200]}")
        except socket.timeout:
            print(f"    Connected but no banner (may need to send CR)")
            sock.sendall(b'\r\n')
            try:
                data = sock.recv(1024)
                text = data.decode('latin-1', errors='replace')
                print(f"    After CR: {text[:200]}")
            except socket.timeout:
                print(f"    Still no response")
        sock.close()
    except Exception as e:
        print(f"    Not available: {e}")


def test_alternate_ports(host: str) -> None:
    """Check common XPORT data ports."""
    print(f"\n[9] ALTERNATE PORT SCAN")
    common_ports = [2101, 4001, 10001, 3001, 8001]
    for p in common_ports:
        try:
            sock = socket.create_connection((host, p), timeout=0.5)
            print(f"    Port {p}: OPEN")
            sock.close()
        except Exception:
            print(f"    Port {p}: closed/filtered")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    host = sys.argv[1]
    data_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_DATA_PORT
    axis_id = int(sys.argv[3]) if len(sys.argv) > 3 else 15
    host_id = int(sys.argv[4]) if len(sys.argv) > 4 else axis_id

    print(f"=" * 60)
    print(f"XPORT Diagnostic Tool — Technosoft TML")
    print(f"  XPORT: {host}:{data_port}")
    print(f"  Axis ID: {axis_id}  Host ID: {host_id}")
    print(f"=" * 60)

    if not test_tcp_connect(host, data_port):
        print("\n*** TCP connection failed — check network/IP/port ***")
        sys.exit(1)

    test_unsolicited_data(host, data_port)
    test_sync_echo(host, data_port)
    test_endinit(host, data_port, axis_id)
    test_give_me_data(host, data_port, axis_id, host_id)
    test_raw_bytes(host, data_port)
    test_web_interface(host)
    test_config_port(host)
    test_alternate_ports(host)

    print(f"\n{'=' * 60}")
    print("DIAGNOSIS CHECKLIST:")
    print("  1. If TCP connects but NO response to ANY test:")
    print("     → Check XPORT serial settings: baud=9600, 8 data, No parity, 2 stop, No flow ctrl")
    print("     → TML drives use 8N2 (2 stop bits) — XPORT default is often 8N1")
    print("     → Access XPORT config: http://<IP>/ or telnet <IP> 9999")
    print("  2. If loopback test echoes bytes:")
    print("     → XPORT may be in echo/loopback mode, not connected to serial")
    print("  3. If web interface found:")
    print("     → Check serial port settings match drive (typically 9600,8,N,2)")
    print("  4. If sync byte (0x0D) echoes but ENDINIT doesn't get ACK:")
    print("     → Address mismatch — try different axis IDs (1, 255, etc.)")
    print("  5. If alternate ports are open:")
    print("     → You may be connecting to wrong port (config vs data)")
    print(f"{'=' * 60}")


if __name__ == '__main__':
    main()
