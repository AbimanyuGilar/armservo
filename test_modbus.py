#!/usr/bin/env python3
"""
Tes cepat Modbus TCP ke ESP8266 (tanpa library tambahan / stdlib saja).

Pakai:
    python3 test_modbus.py <IP_ESP>            # baca status sekali
    python3 test_modbus.py <IP_ESP> --watch    # pantau terus (live)
    python3 test_modbus.py <IP_ESP> --conv on  # uji coba nyalakan conveyor
    python3 test_modbus.py <IP_ESP> --conv off # matikan conveyor

Alamat yang dicek sesuai pemetaan OpenPLC:
    coil 6  -> status IR   (%IX0.0 di OpenPLC, FC01 offset 6)
    coil 4  -> perintah conveyor (%QX0.0 di OpenPLC, FC05 offset 4)
    hreg 0..3 -> Base, Shoulder, Elbow, Gripper (%QW0..%QW3)
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
    print("=" * 50)
    print(f" Status Modbus ESP8266 ({host}:{port})")
    print("=" * 50)
    print(f" Coil 6 (IR / %IX0.0)    = {coils[6]} -> {'ADA BENDA (1)' if coils[6] else 'KOSONG (0)'}")
    print(f" Coil 4 (CONVEYOR cmd)   = {coils[4]} -> {'RUN (1)' if coils[4] else 'STOP (0)'}")
    print(f" Coil 2 (Backup IR)      = {coils[2]}")
    print("-" * 50)
    try:
        hregs = read_hregs(host, port, 0, 5)
        print(f" %QW0 (Base)             = {hregs[0]} deg")
        print(f" %QW1 (Shoulder)         = {hregs[1]} deg")
        print(f" %QW2 (Elbow)            = {hregs[2]} deg")
        print(f" %QW3 (Gripper)          = {hregs[3]} deg")
        print(f" %QW4 (IR Mirror)        = {hregs[4]}")
    except Exception as e:
        print(f" Holding registers error : {e}")
    print("=" * 50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", help="IP ESP8266")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--watch", action="store_true", help="Pantau terus status IR secara real-time")
    ap.add_argument("--conv", choices=["on", "off"], help="Kirim perintah conveyor on/off")
    args = ap.parse_args()

    try:
        if args.conv is not None:
            write_coil(args.host, args.port, 4, args.conv == "on")
            print(f"Coil 4 (Conveyor) -> {args.conv.upper()}")
            return

        if not args.watch:
            one_shot(args.host, args.port)
            return

        print(f"Memantau {args.host}:{args.port} (Tekan Ctrl+C untuk berhenti)...")
        while True:
            try:
                coils = read_coils(args.host, args.port, 0, 8)
                ir_state = coils[6]
                conv_state = coils[4]
                status_str = "ADA BENDA (1)" if ir_state else "KOSONG (0)"
                conv_str = "RUN" if conv_state else "STOP"
                print(f"[{time.strftime('%H:%M:%S')}] IR (Coil 6): {status_str} | Conv (Coil 4): {conv_str}")
            except Exception as e:
                print(f"[{time.strftime('%H:%M:%S')}] Gagal membaca: {e}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nSelesai.")
    except Exception as e:
        print(f"GAGAL: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
