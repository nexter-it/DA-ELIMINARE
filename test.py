#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Invia una singola riga compatta con i dati RTK/GNSS
su tutte le destinazioni UDP configurate e ne stampa
una al secondo sul terminale.

Formato pacchetto:
    MAC/±DD.dddddd7/±DDD.dddddd7/ss/q/vv.v/YYMMDDhhmmss/tms

Lettura dati da ZED-F9P via I2C (indirizzo 0x42).
NTRIP: riceve RTCM da caster e li inoltra al F9P via I2C.
"""

import socket
import threading
import time
import base64
import datetime
import subprocess
import re

import pynmea2
import smbus2

# ────────────────────────── CONFIGURAZIONE ──────────────────────────
CONFIG = {
    # I2C
    "i2c_bus": 1,
    "i2c_addr": 0x42,

    # Destinazioni UDP
    # "destinations": [("193.70.113.55", 3131)],
    "destinations": [("193.70.113.55", 8888)],

    # Parametri NTRIP (opzionale):
    "ntrip_host": "213.209.192.165",
    "ntrip_port": 2101,
    "mount": "NEXTER",
    "user": "nexter",
    "password": "nexter25",

    # GPIO da abilitare prima di avviare il modulo (se serve alimentare qualcosa)
    "gpio_enable_pin": 26,     # BCM 26 (pin fisico 37)
    "gpio_active_high": True,  # True = porta alta, False = bassa
    "gpio_warmup_sec": 2       # attesa prima di avviare GPS/NTRIP
}

# ID dispositivo (puoi rimettere la logica per il MAC se vuoi)
MAC_ADDR = "1"
print(f"[INFO] ID DEVICE: {MAC_ADDR}")

# ───────────── variabili globali condivise fra i thread ─────────────
running          = True
udp_socks        = []

# I2C bus e lock
i2c_bus = None
i2c_lock = threading.Lock()

# Dati GPS più recenti
gps_data = {
    'speed_kmh': 0.0,
    'timestamp': datetime.datetime.utcnow().strftime("%y%m%d%H%M%S"),
    'tms': 0,
    'latitude': None,
    'longitude': None,
    'satellites': 0,
    'quality': 0,
    'last_valid_time': None
}
gps_lock = threading.Lock()

last_print_ts = 0.0    # per limitare la stampa a 1 Hz
# --------------------------------------------------------------------

# ─────────────────────────── FUNZIONI UTILI ─────────────────────────
def init_udp():
    """Inizializza i socket UDP indicati in CONFIG."""
    global udp_socks
    for s, *_ in udp_socks:
        s.close()
    udp_socks.clear()
    for host, port in CONFIG["destinations"]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socks.append((sock, host, port))
        print(f"[UDP] destinazione {host}:{port}")


def send_udp(msg: str):
    """Invia msg a tutte le destinazioni UDP configurate."""
    data = msg.encode()
    for sock, host, port in udp_socks:
        try:
            sock.sendto(data, (host, port))
        except OSError as e:
            print(f"[!] UDP error {host}:{port} – {e}")


def update_timestamp_from_msg(msg):
    """Aggiorna timestamp YYMMDDhhmmss e millisecondi (tms=0..999) da NMEA (UTC)."""
    current_dt = None
    tms = None

    # RMC: data + time (UTC)
    if hasattr(msg, 'datestamp') and hasattr(msg, 'timestamp') and msg.datestamp and msg.timestamp:
        try:
            current_dt = datetime.datetime.combine(msg.datestamp, msg.timestamp)
            tms = getattr(msg.timestamp, 'microsecond', 0) // 1000
        except Exception:
            current_dt = None

    # GGA: solo time (UTC) -> combiniamo con la data UTC corrente
    if not current_dt and hasattr(msg, 'timestamp') and msg.timestamp:
        try:
            utc_today = datetime.datetime.utcnow().date()
            current_dt = datetime.datetime.combine(utc_today, msg.timestamp)
            tms = getattr(msg.timestamp, 'microsecond', 0) // 1000
        except Exception:
            current_dt = None

    # Fallback: usa l'orologio di sistema (UTC)
    if not current_dt:
        now = datetime.datetime.utcnow()
        with gps_lock:
            gps_data['timestamp'] = now.strftime("%y%m%d%H%M%S")
            gps_data['tms'] = int(now.microsecond / 1000)
            gps_data['last_valid_time'] = now
        return False

    # Aggiorna stato condiviso
    with gps_lock:
        gps_data['timestamp'] = current_dt.strftime("%y%m%d%H%M%S")
        gps_data['tms'] = int(tms) if tms is not None else int((datetime.datetime.utcnow().microsecond) / 1000)
        gps_data['last_valid_time'] = current_dt
    return True
# --------------------------------------------------------------------

# ──────────────────────── I2C / UBLOX HELPERS ───────────────────────
def ublox_i2c_read_available():
    """
    Legge quanti byte sono disponibili nel buffer I2C del modulo.
    u-blox: contatore 16 bit nei registri 0xFD (LSB) e 0xFE (MSB).
    """
    global i2c_bus
    try:
        with i2c_lock:
            lsb = i2c_bus.read_byte_data(CONFIG["i2c_addr"], 0xFD)
            msb = i2c_bus.read_byte_data(CONFIG["i2c_addr"], 0xFE)
        return (msb << 8) | lsb
    except OSError as e:
        print(f"[I2C] errore lettura count: {e}")
        return 0


def ublox_i2c_read_block(max_bytes=512):
    """
    Legge fino a max_bytes dal buffer del modulo.
    Restituisce bytes.
    """
    global i2c_bus
    available = ublox_i2c_read_available()
    if available == 0:
        return b""

    to_read = min(available, max_bytes)
    data = bytearray()

    try:
        while to_read > 0:
            chunk = min(32, to_read)  # leggere in blocchi piccoli è più stabile
            with i2c_lock:
                block = i2c_bus.read_i2c_block_data(CONFIG["i2c_addr"], 0xFF, chunk)
            data.extend(block)
            to_read -= chunk
    except OSError as e:
        print(f"[I2C] errore lettura blocco: {e}")
    return bytes(data)


def ublox_i2c_write(data: bytes):
    """
    Scrive dati (es. RTCM) nel buffer del modulo via I2C.
    u-blox: si scrive sul registro 0xFF.
    """
    global i2c_bus
    if not data:
        return

    try:
        offset = 0
        length = len(data)
        while offset < length:
            chunk = data[offset:offset + 32]  # blocchi piccoli
            with i2c_lock:
                i2c_bus.write_i2c_block_data(CONFIG["i2c_addr"], 0xFF, list(chunk))
            offset += len(chunk)
    except OSError as e:
        print(f"[I2C] errore scrittura dati a F9P: {e}")
# --------------------------------------------------------------------

# ───────────────────────── THREAD - GPS ─────────────────────────────
def gps_worker():
    """Legge il F9P via I2C, estrae NMEA RMC/GGA/VTG e invia pacchetti."""
    global last_print_ts

    print(f"[GPS] I2C bus {CONFIG['i2c_bus']} addr 0x{CONFIG['i2c_addr']:02X}")

    buffer = bytearray()

    while running:
        # 1) leggi eventuali dati dal modulo
        chunk = ublox_i2c_read_block(max_bytes=512)
        if chunk:
            buffer.extend(chunk)
        else:
            # niente dati: piccolo sleep per non saturare la CPU
            time.sleep(0.01)

        # 2) processa le linee NMEA complete nel buffer
        while True:
            nl_index = buffer.find(b'\n')
            if nl_index == -1:
                break  # nessuna riga completa

            line_bytes = buffer[:nl_index + 1]
            del buffer[:nl_index + 1]

            try:
                line = line_bytes.decode("ascii", errors="replace").strip()
            except UnicodeDecodeError:
                continue

            if not line.startswith('$'):
                continue

            # prova a parsare la sentenza NMEA
            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue

            # ------------------- VTG -------------------
            if isinstance(msg, pynmea2.VTG):
                with gps_lock:
                    try:
                        if msg.spd_over_grnd_kmph is not None:
                            gps_data['speed_kmh'] = float(msg.spd_over_grnd_kmph)
                        elif msg.spd_over_grnd_kts is not None:
                            speed_knots = float(msg.spd_over_grnd_kts)
                            gps_data['speed_kmh'] = speed_knots * 1.852
                        else:
                            gps_data['speed_kmh'] = 0.0
                    except (ValueError, TypeError):
                        gps_data['speed_kmh'] = 0.0

            # ------------------- RMC -------------------
            elif isinstance(msg, pynmea2.RMC):
                with gps_lock:
                    try:
                        if msg.spd_over_grnd is not None:
                            gps_data['speed_kmh'] = float(msg.spd_over_grnd) * 1.852
                        else:
                            gps_data['speed_kmh'] = 0.0
                    except (ValueError, TypeError):
                        gps_data['speed_kmh'] = 0.0

                    if msg.status == 'A' and msg.latitude and msg.longitude:
                        gps_data['latitude'] = msg.latitude
                        gps_data['longitude'] = msg.longitude

                update_timestamp_from_msg(msg)

            # ------------------- GGA -------------------
            elif isinstance(msg, pynmea2.GGA):
                should_send = False

                with gps_lock:
                    try:
                        gps_data['satellites'] = int(msg.num_sats) if msg.num_sats else 0
                    except (ValueError, TypeError):
                        gps_data['satellites'] = 0

                    try:
                        gps_data['quality'] = int(msg.gps_qual) if msg.gps_qual else 0
                    except (ValueError, TypeError):
                        gps_data['quality'] = 0

                    if msg.gps_qual and int(msg.gps_qual) > 0 and msg.latitude and msg.longitude:
                        gps_data['latitude'] = msg.latitude
                        gps_data['longitude'] = msg.longitude
                        should_send = True

                    if should_send and gps_data['latitude'] is not None and gps_data['longitude'] is not None:
                        compact = (
                            f"{MAC_ADDR}/"
                            f"{gps_data['latitude']:+09.7f}/"
                            f"{gps_data['longitude']:+010.7f}/"
                            f"{gps_data['satellites']:02d}/"
                            f"{gps_data['quality']}/"
                            f"{gps_data['speed_kmh']:.1f}/"
                            f"{gps_data['timestamp']}/"
                            f"{int(gps_data['tms']) % 1000:03d}\n"
                        )

                update_timestamp_from_msg(msg)

                if should_send:
                    send_udp(compact)

                    now = time.time()
                    if now - last_print_ts >= 1.0:
                        print(compact.strip())
                        last_print_ts = now

# --------------------------------------------------------------------

# ─────────────────────── THREAD - NTRIP (opz.) ──────────────────────
def ntrip_worker():
    """Riceve correzioni RTCM dal caster NTRIP e le inoltra al F9P via I2C."""
    while running:
        try:
            creds = base64.b64encode(f"{CONFIG['user']}:{CONFIG['password']}".encode()).decode()
            req = (f"GET /{CONFIG['mount']} HTTP/1.0\r\n"
                   f"User-Agent: NTRIP python\r\n"
                   f"Authorization: Basic {creds}\r\n\r\n")

            with socket.create_connection((CONFIG["ntrip_host"], CONFIG["ntrip_port"]), 10) as s:
                s.sendall(req.encode())
                resp = s.recv(1024)
                if b"ICY 200 OK" not in resp:
                    print("[NTRIP] risposta non valida")
                    raise ConnectionError("risposta non valida dal caster")
                print("[NTRIP] connesso")

                while running:
                    data = s.recv(1024)
                    if not data:
                        raise ConnectionError("stream chiuso")
                    # inoltra le correzioni RTCM al F9P via I2C
                    ublox_i2c_write(data)

        except Exception as e:
            print(f"[NTRIP] {e}; riconnessione in 5 s")
            time.sleep(5)
# --------------------------------------------------------------------

# ────────────── GPIO: abilitazione prima di GPS/NTRIP ───────────────
def enable_gpio_before_start():
    """
    Imposta il GPIO indicato in CONFIG come uscita e lo porta
    al livello attivo (alto di default), poi attende warmup.
    """
    pin = CONFIG["gpio_enable_pin"]
    active_high = CONFIG["gpio_active_high"]
    warmup = CONFIG["gpio_warmup_sec"]

    try:
        import RPi.GPIO as GPIO
    except ImportError:
        print("[GPIO] Modulo RPi.GPIO non disponibile. Proseguo senza pilotare il pin.")
        return

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH if active_high else GPIO.LOW)
    GPIO.output(pin, GPIO.HIGH if active_high else GPIO.LOW)
    print(f"[GPIO] Pin BCM {pin} impostato a {'HIGH' if active_high else 'LOW'}. Attesa {warmup}s…")
    time.sleep(warmup)
    # Non facciamo cleanup qui per mantenere il livello durante l'esecuzione.
# --------------------------------------------------------------------

# ───────────────────────────── MAIN ────────────────────────────────
if __name__ == "__main__":
    # 1) Abilita GPIO e attende (se serve)
    enable_gpio_before_start()

    # 2) Inizializza I2C
    try:
        i2c_bus = smbus2.SMBus(CONFIG["i2c_bus"])
        print(f"[I2C] Aperto bus {CONFIG['i2c_bus']} addr 0x{CONFIG['i2c_addr']:02X}")
    except Exception as e:
        print(f"[I2C] ERRORE apertura bus: {e}")
        exit(1)

    # 3) Inizializza UDP e avvia i thread GPS/NTRIP
    init_udp()

    t_gps   = threading.Thread(target=gps_worker,   daemon=True)
    t_ntrip = threading.Thread(target=ntrip_worker, daemon=True)
    t_gps.start()
    t_ntrip.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        running = False
        print("\n[MAIN] interrompo…")
        for s, *_ in udp_socks:
            s.close()
        try:
            if i2c_bus is not None:
                i2c_bus.close()
        except Exception:
            pass
        # porta il pin a livello inattivo e rilascia le risorse GPIO (best-effort)
        try:
            import RPi.GPIO as GPIO
            GPIO.output(CONFIG["gpio_enable_pin"],
                        GPIO.LOW if CONFIG["gpio_active_high"] else GPIO.HIGH)
            GPIO.cleanup()
        except Exception:
            pass
