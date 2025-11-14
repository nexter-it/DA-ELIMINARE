#!/usr/bin/env python3
"""
Script per leggere dati dal modulo u-blox ZED-F9P via I2C
Indirizzo I2C: 0x42
"""

import smbus2
import time
import sys

# Configurazione I2C
I2C_BUS = 1  # Raspberry Pi usa bus I2C 1
I2C_ADDR = 0x42  # Indirizzo del ZED-F9P

# Registri u-blox I2C
REG_DATA_STREAM = 0xFF  # Registro per leggere stream di dati
REG_BYTES_AVAILABLE_HIGH = 0xFD  # Byte disponibili (high byte)
REG_BYTES_AVAILABLE_LOW = 0xFE   # Byte disponibili (low byte)

def read_bytes_available(bus):
    """Legge il numero di byte disponibili nel buffer"""
    try:
        high = bus.read_byte_data(I2C_ADDR, REG_BYTES_AVAILABLE_HIGH)
        low = bus.read_byte_data(I2C_ADDR, REG_BYTES_AVAILABLE_LOW)
        return (high << 8) | low
    except Exception as e:
        return 0

def read_ubx_data(bus, num_bytes):
    """Legge dati dal modulo"""
    try:
        data = bus.read_i2c_block_data(I2C_ADDR, REG_DATA_STREAM, num_bytes)
        return bytes(data)
    except Exception as e:
        print(f"Errore lettura: {e}")
        return b''

def parse_nmea(data):
    """Estrae e stampa frasi NMEA"""
    try:
        text = data.decode('ascii', errors='ignore')
        lines = text.split('\n')
        for line in lines:
            if line.startswith('$'):
                print(f"NMEA: {line.strip()}")
    except:
        pass

def main():
    print("=== Lettore I2C ZED-F9P ===")
    print(f"Bus I2C: {I2C_BUS}")
    print(f"Indirizzo: 0x{I2C_ADDR:02X}")
    print("Premi Ctrl+C per terminare\n")
    
    try:
        # Inizializza bus I2C
        bus = smbus2.SMBus(I2C_BUS)
        
        # Test connessione
        try:
            bus.read_byte(I2C_ADDR)
            print("✓ Modulo ZED-F9P rilevato!\n")
        except:
            print("✗ ERRORE: Modulo non trovato all'indirizzo 0x42")
            print("Verifica:")
            print("  1. Collegamenti I2C (SDA, SCL, GND, 5V)")
            print("  2. I2C abilitato su u-center")
            print("  3. sudo i2cdetect -y 1")
            sys.exit(1)
        
        buffer = b''
        
        while True:
            # Leggi quanti byte sono disponibili
            available = read_bytes_available(bus)
            
            if available > 0:
                # Limita la lettura a 32 byte alla volta
                read_size = min(available, 32)
                data = read_ubx_data(bus, read_size)
                
                if data:
                    buffer += data
                    
                    # Stampa dati raw in hex
                    print(f"[{len(data)} bytes] HEX: {data.hex()}")
                    
                    # Prova a decodificare NMEA
                    parse_nmea(data)
                    
                    # Stampa dati ASCII (se leggibili)
                    try:
                        ascii_data = data.decode('ascii', errors='ignore').strip()
                        if ascii_data and any(c.isprintable() for c in ascii_data):
                            print(f"ASCII: {ascii_data}")
                    except:
                        pass
                    
                    print("-" * 60)
            
            time.sleep(0.1)  # Polling ogni 100ms
            
    except KeyboardInterrupt:
        print("\n\nInterrotto dall'utente")
    except Exception as e:
        print(f"\nErrore: {e}")
    finally:
        try:
            bus.close()
        except:
            pass

if __name__ == "__main__":
    main()
