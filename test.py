#!/usr/bin/env python3
"""
Lettore GPS ZED-F9P via I2C con parsing NMEA migliorato
"""

import smbus2
import time
import re
from datetime import datetime

I2C_ADDR = 0x42
bus = smbus2.SMBus(1)

def parse_gngga(sentence):
    """Parse GNGGA sentence per info GPS"""
    parts = sentence.split(',')
    if len(parts) < 10:
        return None
    
    try:
        fix_quality = int(parts[6]) if parts[6] else 0
        num_sats = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 99.99
        altitude = float(parts[9]) if parts[9] else 0.0
        
        # Converti coordinate se presenti
        lat = parts[2]
        lat_dir = parts[3]
        lon = parts[4]
        lon_dir = parts[5]
        
        return {
            'fix': fix_quality,
            'sats': num_sats,
            'hdop': hdop,
            'alt': altitude,
            'lat': f"{lat} {lat_dir}" if lat else "N/A",
            'lon': f"{lon} {lon_dir}" if lon else "N/A"
        }
    except:
        return None

def parse_gnrmc(sentence):
    """Parse GNRMC per velocità"""
    parts = sentence.split(',')
    if len(parts) < 8:
        return None
    
    try:
        status = parts[2]  # A = valid, V = invalid
        speed_knots = float(parts[7]) if parts[7] else 0.0
        speed_kmh = speed_knots * 1.852
        
        return {
            'valid': status == 'A',
            'speed_kmh': speed_kmh
        }
    except:
        return None

def parse_gngsa(sentence):
    """Parse GNGSA per DOP"""
    parts = sentence.split(',')
    if len(parts) < 17:
        return None
    
    try:
        fix_type = int(parts[2]) if parts[2] else 1
        pdop = float(parts[15]) if parts[15] else 99.99
        hdop = float(parts[16]) if parts[16] else 99.99
        vdop = float(parts[17].split('*')[0]) if parts[17] else 99.99
        
        return {
            'fix_type': fix_type,  # 1=no fix, 2=2D, 3=3D
            'pdop': pdop,
            'hdop': hdop,
            'vdop': vdop
        }
    except:
        return None

print("=" * 70)
print("       ZED-F9P GPS Monitor - I2C Interface")
print("=" * 70)
print(f"Indirizzo I2C: 0x{I2C_ADDR:02X}")
print("Premi Ctrl+C per terminare")
print("=" * 70)

# Test connessione
try:
    bus.read_byte(I2C_ADDR)
    print("✓ Modulo GPS connesso\n")
except:
    print("✗ ERRORE: Modulo non trovato!")
    print("Verifica con: sudo i2cdetect -y 1")
    exit(1)

buffer = ""
last_status_print = 0
gps_data = {
    'fix': 0,
    'sats': 0,
    'hdop': 99.99,
    'lat': 'N/A',
    'lon': 'N/A',
    'alt': 0.0,
    'speed': 0.0
}

try:
    while True:
        try:
            # Leggi byte disponibili
            high = bus.read_byte_data(I2C_ADDR, 0xFD)
            low = bus.read_byte_data(I2C_ADDR, 0xFE)
            available = (high << 8) | low
            
            if available > 0:
                # Leggi dati
                read_size = min(available, 32)
                data = bus.read_i2c_block_data(I2C_ADDR, 0xFF, read_size)
                buffer += bytes(data).decode('ascii', errors='ignore')
                
                # Processa frasi NMEA complete
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line.startswith('$'):
                        # Stampa frase raw
                        print(f"[RAW] {line}")
                        
                        # Parse specifiche frasi
                        if line.startswith('$GNGGA'):
                            result = parse_gngga(line)
                            if result:
                                gps_data.update(result)
                                
                        elif line.startswith('$GNRMC'):
                            result = parse_gnrmc(line)
                            if result:
                                gps_data['speed'] = result['speed_kmh']
                                
                        elif line.startswith('$GNGSA'):
                            result = parse_gngsa(line)
                            if result:
                                print(f"      └─> Fix Type: {result['fix_type']} (1=No, 2=2D, 3=3D) | PDOP: {result['pdop']:.2f}")
            
            # Stampa status ogni 2 secondi
            current_time = time.time()
            if current_time - last_status_print >= 2:
                last_status_print = current_time
                
                print("\n" + "─" * 70)
                print(f"📡 STATUS @ {datetime.now().strftime('%H:%M:%S')}")
                print("─" * 70)
                
                fix_status = ["❌ NO FIX", "⚠️  NO FIX", "🟡 2D FIX", "🟢 3D FIX"]
                fix_idx = min(gps_data['fix'], 3)
                
                print(f"Fix:       {fix_status[fix_idx]}")
                print(f"Satelliti: {gps_data['sats']:2d}")
                print(f"HDOP:      {gps_data['hdop']:6.2f} (< 2.0 = Ottimo, < 5.0 = Buono)")
                print(f"Latitudine:  {gps_data['lat']}")
                print(f"Longitudine: {gps_data['lon']}")
                print(f"Altitudine:  {gps_data['alt']:.1f} m")
                print(f"Velocità:    {gps_data['speed']:.1f} km/h")
                
                if gps_data['fix'] == 0:
                    print("\n💡 SUGGERIMENTI:")
                    print("   • Assicurati che l'antenna sia collegata")
                    print("   • Posiziona il GPS all'aperto o vicino a una finestra")
                    print("   • Il primo fix può richiedere 1-5 minuti (cold start)")
                
                print("─" * 70 + "\n")
                        
        except Exception as e:
            pass
            
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("\n\n✓ Chiusura del programma...")
    bus.close()
except Exception as e:
    print(f"\n❌ Errore: {e}")
    bus.close()
