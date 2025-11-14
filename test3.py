#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import base64
import time

import smbus2

CONFIG = {
    "i2c_bus": 1,
    "i2c_addr": 0x42,

    "ntrip_host": "213.209.192.165",
    "ntrip_port": 2101,
    "mount": "NEXTER",
    "user": "nexter",
    "password": "nexter25",
}

def ublox_i2c_write(bus, data: bytes):
    if not data:
        return
    try:
        offset = 0
        length = len(data)
        while offset < length:
            chunk = data[offset:offset + 32]
            bus.write_i2c_block_data(CONFIG["i2c_addr"], 0xFF, list(chunk))
            offset += len(chunk)
    except OSError as e:
        print(f"[I2C] errore scrittura a F9P: {e}")

def ntrip_loop(bus):
    total_bytes = 0
    last_print = time.time()

    while True:
        try:
            creds = base64.b64encode(
                f"{CONFIG['user']}:{CONFIG['password']}".encode()
            ).decode()

            req = (
                f"GET /{CONFIG['mount']} HTTP/1.0\r\n"
                f"User-Agent: NTRIP test-python\r\n"
                f"Authorization: Basic {creds}\r\n\r\n"
            )

            print(f"[NTRIP] Connessione a {CONFIG['ntrip_host']}:{CONFIG['ntrip_port']}...")
            with socket.create_connection(
                (CONFIG["ntrip_host"], CONFIG["ntrip_port"]), 10
            ) as s:
                s.sendall(req.encode())
                resp = s.recv(1024)
                if b"ICY 200 OK" not in resp:
                    print("[NTRIP] risposta non valida:")
                    print(resp.decode(errors="ignore"))
                    raise ConnectionError("risposta non valida dal caster")

                print("[NTRIP] connesso, inizio ricezione RTCM…")

                while True:
                    data = s.recv(1024)
                    if not data:
                        raise ConnectionError("stream chiuso dal server")

                    total_bytes += len(data)

                    # debug: mostra solo pochi byte all'inizio
                    # (scommenta se vuoi vedere la D3 ...)
                    # print(data[:16].hex(" "))

                    ublox_i2c_write(bus, data)

                    now = time.time()
                    if now - last_print >= 5:
                        print(f"[NTRIP] Totale inviato al F9P: {total_bytes} byte")
                        last_print = now

        except Exception as e:
            print(f"[NTRIP] errore: {e}. Riprovo tra 5s...")
            time.sleep(5)

def main():
    print(f"[I2C] Apro bus {CONFIG['i2c_bus']} addr 0x{CONFIG['i2c_addr']:02X}")
    bus = smbus2.SMBus(CONFIG["i2c_bus"])

    try:
        ntrip_loop(bus)
    except KeyboardInterrupt:
        print("\n[MAIN] Interrotto dall'utente.")
    finally:
        try:
            bus.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
