#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import smbus2

I2C_BUS = 1
I2C_ADDR = 0x42

# UBX-MON-VER poll:
#  B5 62 0A 04 00 00 0E 34
UBX_MON_VER_POLL = bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34])

def ublox_i2c_read_available(bus):
    try:
        lsb = bus.read_byte_data(I2C_ADDR, 0xFD)
        msb = bus.read_byte_data(I2C_ADDR, 0xFE)
        return (msb << 8) | lsb
    except OSError as e:
        print(f"[I2C] errore lettura count: {e}")
        return 0

def ublox_i2c_read_block(bus, max_bytes=512):
    available = ublox_i2c_read_available(bus)
    if available == 0:
        return b""

    to_read = min(available, max_bytes)
    data = bytearray()

    try:
        while to_read > 0:
            chunk = min(32, to_read)
            block = bus.read_i2c_block_data(I2C_ADDR, 0xFF, chunk)
            data.extend(block)
            to_read -= chunk
    except OSError as e:
        print(f"[I2C] errore lettura blocco: {e}")
    return bytes(data)

def ublox_i2c_write(bus, data: bytes):
    if not data:
        return
    try:
        offset = 0
        length = len(data)
        while offset < length:
            chunk = data[offset:offset + 32]
            bus.write_i2c_block_data(I2C_ADDR, 0xFF, list(chunk))
            offset += len(chunk)
    except OSError as e:
        print(f"[I2C] errore scrittura: {e}")

def main():
    print(f"[I2C] Apro bus {I2C_BUS}, addr 0x{I2C_ADDR:02X}")
    bus = smbus2.SMBus(I2C_BUS)

    try:
        while True:
            print("[TEST] Invio UBX-MON-VER poll via I2C…")
            ublox_i2c_write(bus, UBX_MON_VER_POLL)

            # attendo che il F9P risponda
            time.sleep(0.1)

            resp = ublox_i2c_read_block(bus, max_bytes=512)
            if resp:
                print(f"[TEST] Riceviti {len(resp)} byte dalla scheda:")
                print(resp.hex(" "))
            else:
                print("[TEST] Nessuna risposta (nessun dato nel buffer I2C)")

            print("---------------------------------------------------")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[MAIN] Interrotto dall'utente.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
