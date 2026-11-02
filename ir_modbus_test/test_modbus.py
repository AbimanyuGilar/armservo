#!/usr/bin/env python3
"""
Tes cepat Modbus TCP ke ESP8266 (tanpa library tambahan / stdlib saja).

Pakai:
    python3 test_modbus.py 192.168.1.17            # baca sekali
    python3 test_modbus.py 192.168.1.17 --watch    # pantau terus
    python3 test_modbus.py 192.168.1.17 --coil0 on # nyalakan conveyor
    python3 test_modbus.py 192.168.1.17 --coil0 off

Alamat yang dicek:
    coil 2  -> status IR   (%IX0.0 di OpenPLC, FC01 offset 2)
    coil 0  -> perintah conveyor (%QX0.0)
    hreg 0  -> mirror IR (%QW0)
"""

import argparse
import socket
import struct
import sys
import time

UNIT_ID = 1


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("koneksi ditutup ESP")
        buf += chunk
    return buf


def _request(host, port, pdu, timeout=2.0):
    tid = 0x1234
    mbap = struct.pack(">HHHB", tid, 0, len(pdu) + 1, UNIT_ID)
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(mbap + pdu)
        header = _recv_exact(s, 7)
        _tid, _proto, length, _unit = struct.unpack(">HHHB", header)
        body = _recv_exact(s, length - 1)
    if body[0] & 0x80:
        raise RuntimeError(
            f"Modbus exception FC{body[0] & 0x7F}, code={body[1]} "
            "(alamat/register tidak ada di ESP)"
        )
    return body


def read_coils(host, port, addr, count):
    body = _request(host, port, struct.pack(">BHH", 0x01, addr, count))
    bits = []
    for byte in body[2:2 + body[1]]:
        for i in range(8):
            bits.append((byte >> i) & 1)
    return bits[:count]


def read_hregs(host, port, addr, count):
    body = _request(host, port, struct.pack(">BHH", 0x03, addr, count))
    return list(struct.unpack(">" + "H" * count, body[2:2 + 2 * count]))


def write_coil(host, port, addr, value):
    val = 0xFF00 if value else 0x0000
    _request(host, port, struct.pack(">BHH", 0x05, addr, val))


def one_shot(host, port):
    coils = read_coils(host, port, 0, 8)
    print(f"coil 2 (IR / %IX0.0)   = {coils[2]}   {'ADA BENDA' if coils[2] else 'kosong'}")
    print(f"coil 3 (ARM_BUSY)      = {coils[3]}")
    print(f"coil 0 (CONVEYOR cmd)  = {coils[0]}")
    try:
        hregs = read_hregs(host, port, 0, 2)
        print(f"hreg 0                 = {hregs[0]}")
        print(f"hreg 1                 = {hregs[1]}")
    except Exception as e:
        print(f"hreg 0/1               = n/a ({e})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", help="IP ESP8266")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--coil0", choices=["on", "off"])
    args = ap.parse_args()

    try:
        if args.coil0 is not None:
            write_coil(args.host, args.port, 0, args.coil0 == "on")
            print(f"coil 0 -> {args.coil0}")
            return

        if not args.watch:
            one_shot(args.host, args.port)
            return

        print(f"Memantau {args.host}:{args.port} (Ctrl+C untuk stop)")
        while True:
            try:
                c = read_coils(args.host, args.port, 2, 1)[0]
                h = read_hregs(args.host, args.port, 0, 1)[0]
                print(f"{time.strftime('%H:%M:%S')}  IR={c}  mirror={h}")
            except Exception as e:
                print(f"{time.strftime('%H:%M:%S')}  ERROR: {e}")
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"GAGAL: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
