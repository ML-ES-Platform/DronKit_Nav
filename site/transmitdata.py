#!/usr/bin/env python3
import asyncio
import json
import websockets
import time
import random
import math
from datetime import datetime

# Importă bibliotecile pentru hardware real
try:
    import Adafruit_DHT # Sau altă bibliotecă, ex: import dht
    DHT_SENSOR_TYPE = Adafruit_DHT.DHT11 # Sau DHT22
    DHT_PIN = 4 # GPIO Pin-ul la care este conectat senzorul DHT
except ImportError:
    print("WARN: Biblioteca Adafruit_DHT nu este instalată. Datele de temperatură/umiditate nu vor fi reale.")
    Adafruit_DHT = None # Setează la None dacă nu e instalată

try:
    from pymavlink import mavutil
except ImportError:
    print("EROARE CRITICĂ: Biblioteca pymavlink nu este instalată. Rulați 'pip install pymavlink pyserial'")
    exit(1)

# --- Configurare Hardware ---
# Setează la False pentru a folosi hardware real
SIMULATION_MODE = False

# String de conexiune MAVLink (Exemple)
# Pentru conexiune serială (Linux): '/dev/ttyACM0' sau '/dev/ttyUSB0' (verifică portul corect)
# Pentru conexiune serială (Windows): 'COM3' (verifică portul corect)
# Pentru SITL (UDP): 'udp:localhost:14550'
# Pentru conexiune TCP: 'tcp:IP_DRONA:5760'
MAVLINK_CONNECTION_STRING = '/dev/serial0' # <--- MODIFICĂ AICI CU CONEXIUNEA TA REALĂ
MAVLINK_BAUDRATE = 921600 # Baud rate pentru conexiuni seriale (adesea 57600 sau 115200)

# Definește portul WebSocket
WEBSOCKET_PORT = 8765

class TelemetryData:
    def __init__(self):
        # Inițializează cu valori default
        self.telemetry = {
            "flightMode": "Unknown",
            "altitude": 0,         # Relative altitude (metri)
            "verticalSpeed": 0,  # m/s
            "groundSpeed": 0,    # m/s
            "heading": 0,        # Grade (0-359)
            "batteryVoltage": 0.0, # Volți
            "batteryPercentage": -1, # Procentaj (-1 dacă nu e disponibil)
            "gpsSignal": "No Fix", # Starea GPS (No Fix, 2D, 3D, DGPS etc.)
            "gpsNumSatellites": 0, # Număr sateliți
            "distance": 0,       # Distanța față de Home (metri)
            "flightTime": "00:00:00", # Timpul de zbor (de la armare)
            "roll": 0,           # Grade
            "pitch": 0,          # Grade
            "yaw": 0,            # Grade
            "latitude": 0.0,     # Grade decimale
            "longitude": 0.0,    # Grade decimale
            "temperature": None,   # Grade Celsius (de la senzor extern)
            "humidity": None,    # Procentaj (de la senzor extern)
            "pressure": None,    # Presiune (dacă e disponibilă de la MAVLink)
            "wind": "N/A",       # Vânt (poate fi estimat sau de la MAVLink dacă e disponibil)
            "isConnected": False # Starea conexiunii MAVLink
        }
        self.home_position = None # (lat, lon)
        self.arm_time = None

        # Waypoints (pot fi încărcate ulterior)
        self.waypoints = []
        self.mission_status = "idle" # idle, running, paused, completed, aborted
        self.current_waypoint = None # ID-ul waypoint-ului curent (sau index MAVLink)

        # Conexiunea MAVLink
        self.mav_connection = None
        if not SIMULATION_MODE:
            self._connect_mavlink()

        # Generare flight path (doar pentru simulare sau vizualizare plan)
        # Într-un scenariu real, path-ul este urmat de dronă
        self.flight_path = []
        self.path_index = 0 # Folosit doar în simulare

        # Timpul de start al zborului (pentru simulare)
        self.flight_start_time = None # Folosit doar în simulare

    def _connect_mavlink(self):
        """Inițiază conexiunea MAVLink"""
        try:
            print(f"Încercare de conectare MAVLink la: {MAVLINK_CONNECTION_STRING}")
            # Pentru conexiuni seriale, adaugă baudrate
            if MAVLINK_CONNECTION_STRING.startswith('/dev/') or MAVLINK_CONNECTION_STRING.startswith('COM'):
                self.mav_connection = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING, baud=MAVLINK_BAUDRATE)
            else:
                self.mav_connection = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING)

            # Așteaptă primul mesaj HEARTBEAT pentru a confirma conexiunea
            print("Așteptare mesaj HEARTBEAT de la dronă...")
            self.mav_connection.wait_heartbeat(timeout=10) # Așteaptă max 10 secunde
            print("HEARTBEAT primit, conexiune MAVLink stabilită!")
            self.telemetry["isConnected"] = True
            # Încearcă să obții poziția Home
            self._request_home_position()

        except Exception as e:
            print(f"EROARE la conectarea MAVLink: {e}")
            self.mav_connection = None
            self.telemetry["isConnected"] = False

    def _request_home_position(self):
         """Cere poziția Home de la vehicul"""
         if self.mav_connection:
             try:
                 # Cere mesajul HOME_POSITION
                 self.mav_connection.mav.command_long_send(
                     self.mav_connection.target_system, self.mav_connection.target_component,
                     mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                     0, 0, 0, 0, 0, 0, 0, 0)
                 print("Cerere pentru poziția Home trimisă.")
             except Exception as e:
                 print(f"Eroare la cererea poziției Home: {e}")

    def generate_flight_path(self):
        """Generează un path vizual bazat pe waypoints (folosit mai mult în simulare)"""
        path = []
        if not self.waypoints:
            return path

        start_lat = self.home_position[0] if self.home_position else self.telemetry["latitude"]
        start_lon = self.home_position[1] if self.home_position else self.telemetry["longitude"]
        path.append((start_lat, start_lon))

        current_lat, current_lon = start_lat, start_lon
        for wp in self.waypoints:
            # Calculează puncte intermediare dacă este necesar pentru vizualizare
            # Într-un scenariu real, drona navighează direct
            # Aici adăugăm doar waypoint-ul final al segmentului
            path.append((wp["latitude"], wp["longitude"]))
            current_lat, current_lon = wp["latitude"], wp["longitude"]

        return path

    def update_telemetry(self):
        """Actualizează datele de telemetrie (real sau simulat)"""
        if SIMULATION_MODE:
            self._update_simulated_telemetry()
        else:
            self._update_real_telemetry()

    def _update_simulated_telemetry(self):
        """Actualizează cu valori simulate (codul tău original, ușor adaptat)"""
        # ... (păstrează logica ta de simulare aici dacă vrei să comuți între moduri)
        # ... Asigură-te că folosești self.telemetry pentru actualizări ...

        # Exemplu simplu de actualizare simulare (dacă vrei să o păstrezi)
        if self.mission_status == 'running':
             self.telemetry["altitude"] += (random.random() - 0.5) * 1
             self.telemetry["groundSpeed"] = 5 + (random.random() - 0.5)
             self.telemetry["heading"] = (self.telemetry["heading"] + 1) % 360
             self.telemetry["roll"] = (random.random() - 0.5) * 5
             self.telemetry["pitch"] = (random.random() - 0.5) * 3
             # Simulează descărcarea bateriei
             self.telemetry["batteryPercentage"] = max(0, self.telemetry.get("batteryPercentage", 100) - 0.02)
             self.telemetry["batteryVoltage"] = 10.5 + (self.telemetry["batteryPercentage"] / 100) * 2.0

             # Actualizează timpul de zbor simulat
             if self.flight_start_time:
                 elapsed_seconds = int(time.time() - self.flight_start_time)
                 hours, remainder = divmod(elapsed_seconds, 3600)
                 minutes, seconds = divmod(remainder, 60)
                 self.telemetry["flightTime"] = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
        else:
             self.telemetry["groundSpeed"] = 0
             self.telemetry["verticalSpeed"] = 0

        # Calculează distanța simulată (dacă home e setat)
        if self.home_position:
             self._calculate_distance(self.home_position[0], self.home_position[1])
        self.telemetry["isConnected"] = True # În simulare, mereu conectat


    def _update_real_telemetry(self):
        """Actualizează telemetria citind datele de la hardware"""
        if not self.mav_connection:
            self.telemetry["isConnected"] = False
            # Încearcă reconectarea periodic?
            # self._connect_mavlink()
            return

        # Citeste datele de la senzorul DHT (dacă e disponibil și configurat)
        if Adafruit_DHT:
            try:
                humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR_TYPE, DHT_PIN)
                if humidity is not None and temperature is not None:
                    self.telemetry["humidity"] = round(humidity, 1)
                    self.telemetry["temperature"] = round(temperature, 1)
                else:
                    # Păstrează valorile vechi sau setează la None dacă citirea eșuează
                    pass
            except Exception as e:
                # print(f"Avertisment: Nu s-au putut citi datele DHT: {e}")
                # Păstrează valorile vechi sau setează la None
                pass


        # Procesează mesajele MAVLink primite
        while True: # Procesează toate mesajele disponibile
            msg = self.mav_connection.recv_match(blocking=False) # Non-blocant
            if not msg:
                break # Nu mai sunt mesaje de procesat acum

            msg_type = msg.get_type()
            # print(f"Received MAVLink message: {msg_type}") # Debug

            # Actualizează starea conexiunii la primirea oricărui mesaj valid
            self.telemetry["isConnected"] = True

            if msg_type == 'BAD_DATA':
                # Ignoră datele corupte
                continue

            if msg_type == 'HEARTBEAT':
                # Decodează modul de zbor
                mode_id = msg.custom_mode
                flight_mode = self.mav_connection.mode_mapping().get(mode_id, f"Unknown ({mode_id})")
                self.telemetry["flightMode"] = flight_mode
                # Verifică starea de armare
                is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if is_armed and self.arm_time is None:
                    self.arm_time = time.time()
                elif not is_armed:
                    self.arm_time = None # Resetează timpul la dezarmare
                    self.telemetry["flightTime"] = "00:00:00"

            elif msg_type == 'SYS_STATUS':
                self.telemetry["batteryVoltage"] = msg.voltage_battery / 1000.0 # mV -> V
                # Procentajul poate fi -1 dacă nu e suportat/configurat corect pe dronă
                self.telemetry["batteryPercentage"] = msg.battery_remaining

            elif msg_type == 'GPS_RAW_INT':
                self.telemetry["gpsNumSatellites"] = msg.satellites_visible
                # Mapează fix_type la o stare lizibilă
                fix = msg.fix_type
                if fix >= 3:
                    self.telemetry["gpsSignal"] = "3D Fix"
                elif fix == 2:
                    self.telemetry["gpsSignal"] = "2D Fix"
                else:
                    self.telemetry["gpsSignal"] = "No Fix"

            elif msg_type == 'GLOBAL_POSITION_INT':
                self.telemetry["latitude"] = msg.lat / 1e7  # Latitudine în grade
                self.telemetry["longitude"] = msg.lon / 1e7 # Longitudine în grade
                self.telemetry["altitude"] = msg.relative_alt / 1000.0 # Altitudine relativă la Home (mm -> m)
                self.telemetry["heading"] = msg.hdg / 100.0 # Heading (0-35999 cdeg -> 0-359.99 deg)
                # Calculează distanța față de Home dacă avem poziția Home
                if self.home_position:
                    self._calculate_distance(self.home_position[0], self.home_position[1])

            elif msg_type == 'VFR_HUD':
                self.telemetry["groundSpeed"] = msg.groundspeed # m/s
                self.telemetry["verticalSpeed"] = msg.climb # m/s
                # Uneori VFR_HUD are altitudine, dar GLOBAL_POSITION_INT.relative_alt e preferată
                # self.telemetry["altitude"] = msg.alt # Altitudine AMSL sau relativă depinde de configurare
                # Heading poate fi și aici, dar GLOBAL_POSITION_INT.hdg e adesea mai precis
                # self.telemetry["heading"] = msg.heading # Grade

            elif msg_type == 'ATTITUDE':
                # Convertește radiani în grade
                self.telemetry["roll"] = math.degrees(msg.roll)
                self.telemetry["pitch"] = math.degrees(msg.pitch)
                self.telemetry["yaw"] = (math.degrees(msg.yaw) + 360) % 360 # Asigură 0-360

            elif msg_type == 'HOME_POSITION':
                # Am primit poziția Home
                if msg.latitude != 0 or msg.longitude != 0: # Verifică dacă e validă
                     self.home_position = (msg.latitude / 1e7, msg.longitude / 1e7)
                     print(f"Poziția Home setată la: {self.home_position}")
                else:
                     print("Poziția Home primită dar pare invalidă (0,0).")


            elif msg_type == 'MISSION_CURRENT':
                # Ne spune la ce waypoint este drona în modul AUTO
                # msg.seq conține indexul waypoint-ului curent (începe de la 0)
                # Trebuie să mapăm indexul la ID-ul nostru dacă folosim ID-uri custom
                if self.waypoints and msg.seq < len(self.waypoints):
                     # Asumând că ID-urile sunt 1, 2, 3... și se aliniază cu secvența MAVLink
                     self.current_waypoint = self.waypoints[msg.seq].get("id", msg.seq + 1)
                else:
                    self.current_waypoint = msg.seq # Folosește indexul MAVLink dacă nu avem mapare

            elif msg_type == 'STATUSTEXT':
                # Afișează mesajele text de la dronă (utile pentru debug)
                print(f"Drone Status: {msg.text}")

        # Actualizează timpul de zbor dacă drona este armată
        if self.arm_time:
            elapsed_seconds = int(time.time() - self.arm_time)
            hours, remainder = divmod(elapsed_seconds, 3600)
            minutes, seconds = divmod(remainder, 60)
            self.telemetry["flightTime"] = f"{hours:02d}:{minutes:02d}:{seconds:02d}"


    def _calculate_distance(self, home_lat, home_lon):
        """Calculează distanța Haversine între poziția curentă și Home"""
        R = 6371e3 # Raza Pământului în metri
        lat1_rad = math.radians(home_lat)
        lon1_rad = math.radians(home_lon)
        lat2_rad = math.radians(self.telemetry["latitude"])
        lon2_rad = math.radians(self.telemetry["longitude"])

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        self.telemetry["distance"] = round(distance, 1) # Rotunjit la o zecimală


    def handle_command(self, command):
        """Gestionează comenzile primite și le trimite către dronă (dacă e cazul)"""
        command_type = command.get("type", "")
        payload = command.get("payload", {})

        if SIMULATION_MODE:
            # Gestionează comenzile în modul simulare (codul tău existent)
            print(f"SIMULARE: Procesare comandă {command_type}")
            if command_type == "START_MISSION":
                if self.mission_status != "running":
                    self.mission_status = "running"
                    self.path_index = 0 # Reset path index for simulation
                    self.current_waypoint = self.waypoints[0]["id"] if self.waypoints else None
                    self.telemetry["flightMode"] = "Auto Mission (Sim)"
                    self.flight_start_time = time.time() # Start simulation timer
            elif command_type == "PAUSE_MISSION":
                 if self.mission_status == "running":
                     self.mission_status = "paused"
                     self.telemetry["flightMode"] = "Loiter (Sim)"
            elif command_type == "RESUME_MISSION":
                 if self.mission_status == "paused":
                     self.mission_status = "running"
                     self.telemetry["flightMode"] = "Auto Mission (Sim)"
            elif command_type == "ABORT_MISSION":
                 self.mission_status = "aborted"
                 self.current_waypoint = None
                 self.telemetry["flightMode"] = "Loiter (Sim)"
                 # Oprește mișcarea în simulare
                 self.telemetry["groundSpeed"] = 0
                 self.telemetry["verticalSpeed"] = 0
            elif command_type == "RETURN_TO_HOME":
                 self.mission_status = "idle" # Sau "returning"
                 self.current_waypoint = None
                 self.telemetry["flightMode"] = "RTL (Sim)"
                 # Simulează întoarcerea la home
                 self.telemetry["latitude"] = self.home_position[0] if self.home_position else 47.6257531
                 self.telemetry["longitude"] = self.home_position[1] if self.home_position else -122.3125569
                 self.telemetry["distance"] = 0
                 self.telemetry["altitude"] = 0 # Aterizează
            elif command_type == "UPLOAD_MISSION":
                 if "waypoints" in payload:
                     self.waypoints = payload["waypoints"]
                     self.flight_path = self.generate_flight_path() # Generează path nou pentru simulare
                     print(f"SIMULARE: Misiune actualizată cu {len(self.waypoints)} waypoints.")
                     self.mission_status = "idle" # Gata de start
            return # Ieși din funcție dacă e simulare

        # --- Gestiune Comenzi Hardware Real ---
        if not self.mav_connection or not self.telemetry["isConnected"]:
            print("Eroare: Nu există conexiune MAVLink pentru a trimite comanda.")
            return

        print(f"HARDWARE: Procesare comandă {command_type}")
        try:
            target_system = self.mav_connection.target_system
            target_component = self.mav_connection.target_component # De obicei 1 (Autopilot)

            if command_type == "START_MISSION":
                # 1. Schimbă modul în AUTO
                # 2. Trimite comanda de start misiune
                if self._set_flight_mode("AUTO"):
                    time.sleep(0.5) # Așteaptă puțin ca modul să se schimbe
                    self.mav_connection.mav.command_long_send(
                        target_system, target_component,
                        mavutil.mavlink.MAV_CMD_MISSION_START,
                        0, # Confirmation
                        0, # param1: first_item
                        0, # param2: last_item (0 for all)
                        0, 0, 0, 0, 0 # params 3-7 unused
                    )
                    print("Comandă MAV_CMD_MISSION_START trimisă.")
                    self.mission_status = "running" # Actualizează starea locală
                else:
                     print("Eroare: Nu s-a putut seta modul AUTO.")


            elif command_type == "PAUSE_MISSION":
                # Schimbă modul în LOITER (sau BRAKE/HOLD depinde de firmware/vehicul)
                 if self._set_flight_mode("LOITER"): # Sau "BRAKE" / "HOLD"
                     print("Mod LOITER setat pentru pauză.")
                     self.mission_status = "paused"
                 else:
                     print("Eroare: Nu s-a putut seta modul LOITER pentru pauză.")


            elif command_type == "RESUME_MISSION":
                # Schimbă modul înapoi în AUTO
                 if self._set_flight_mode("AUTO"):
                     print("Mod AUTO setat pentru reluare.")
                     self.mission_status = "running"
                 else:
                     print("Eroare: Nu s-a putut seta modul AUTO pentru reluare.")

            elif command_type == "ABORT_MISSION":
                 # O opțiune este să treci în LOITER sau RTL
                 if self._set_flight_mode("LOITER"): # Sau RTL
                     print("Misiune abandonată. Trecut în modul LOITER.")
                     self.mission_status = "aborted"
                 else:
                     print("Eroare: Nu s-a putut abandona misiunea (setare mod eșuată).")

            elif command_type == "RETURN_TO_HOME":
                # Schimbă modul în RTL
                 if self._set_flight_mode("RTL"):
                     print("Mod RTL (Return to Launch) setat.")
                     self.mission_status = "returning" # O stare intermediară
                 else:
                     print("Eroare: Nu s-a putut seta modul RTL.")


            elif command_type == "UPLOAD_MISSION":
                # Aceasta este partea cea mai complexă
                if "waypoints" in payload:
                    print(f"HARDWARE: Încercare încărcare {len(payload['waypoints'])} waypoints...")
                    success = self._upload_mavlink_mission(payload["waypoints"])
                    if success:
                        print("Misiune încărcată cu succes pe dronă.")
                        self.waypoints = payload["waypoints"] # Actualizează waypoints locali
                        self.mission_status = "idle" # Gata de start
                    else:
                        print("EROARE la încărcarea misiunii pe dronă.")
                else:
                    print("Eroare: Lipsesc datele 'waypoints' din payload pentru UPLOAD_MISSION.")

            elif command_type == "ARM_DISARM":
                # Comandă de armare/dezarmare
                should_arm = payload.get("arm", False) # Așteaptă un boolean 'arm' în payload
                self.mav_connection.mav.command_long_send(
                    target_system, target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, # Confirmation
                    1.0 if should_arm else 0.0, # param1: 1 to arm, 0 to disarm
                    0, # param2: force (nu forța de obicei)
                    0, 0, 0, 0, 0
                )
                action = "Armare" if should_arm else "Dezarmare"
                print(f"Comandă {action} trimisă.")

            elif command_type == "TAKEOFF":
                 # Comandă de decolare (dacă e în mod ghidat/stabilizat)
                 altitude_m = payload.get("altitude", 5.0) # Altitudine default 5m
                 if self._set_flight_mode("GUIDED"): # Decolarea necesită adesea mod GUIDED
                     time.sleep(0.5)
                     self.mav_connection.mav.command_long_send(
                         target_system, target_component,
                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                         0, # Confirmation
                         0, # param1: Minimum pitch (set to 0 for fixed-wing takeoff)
                         0, # param2: unused
                         0, # param3: unused
                         0, # param4: Yaw angle (NaN for unchanged)
                         0, # param5: Latitude (use 0 for current)
                         0, # param6: Longitude (use 0 for current)
                         altitude_m # param7: Altitude (meters)
                     )
                     print(f"Comandă TAKEOFF la {altitude_m}m trimisă.")
                 else:
                     print("Eroare: Nu s-a putut seta modul GUIDED pentru decolare.")

            # --- Adaugă aici alte comenzi MAVLink după necesități ---
            # Exemple: SET_SERVO, DO_DIGICAM_CONTROL, etc.

        except Exception as e:
            print(f"EROARE la trimiterea comenzii MAVLink '{command_type}': {e}")

    def _set_flight_mode(self, mode_name):
        """Setează modul de zbor al dronei folosind MAVLink"""
        if not self.mav_connection: return False

        mode_id = self.mav_connection.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            print(f"Eroare: Modul de zbor '{mode_name}' nu este recunoscut.")
            return False

        try:
            self.mav_connection.mav.set_mode_send(
                self.mav_connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            print(f"Cerere de setare mod '{mode_name}' (ID: {mode_id}) trimisă.")

            # Așteaptă confirmarea (opțional, dar recomandat)
            # Poți aștepta un mesaj ACK sau verifica HEARTBEAT-ul ulterior
            # Aici doar trimitem comanda și presupunem că va funcționa (simplificat)
            return True

        except Exception as e:
            print(f"Eroare la setarea modului '{mode_name}': {e}")
            return False

    def _upload_mavlink_mission(self, waypoints_data):
        """Încarcă o listă de waypoints pe dronă folosind protocolul MAVLink Mission"""
        if not self.mav_connection: return False

        try:
            target_system = self.mav_connection.target_system
            target_component = self.mav_connection.target_component

            # 1. Trimite MISSION_COUNT
            num_waypoints = len(waypoints_data)
            print(f"Trimitere MISSION_COUNT: {num_waypoints}")
            self.mav_connection.mav.mission_count_send(target_system, target_component, num_waypoints)

            # 2. Așteaptă cererile MISSION_REQUEST și trimite fiecare MISSION_ITEM
            for i in range(num_waypoints):
                # Așteaptă cererea pentru waypoint-ul 'i'
                print(f"Așteptare MISSION_REQUEST pentru waypoint {i}...")
                msg = self.mav_connection.recv_match(type='MISSION_REQUEST', condition=f'MISSION_REQUEST.seq=={i}', blocking=True, timeout=5)
                if not msg:
                    print(f"Eroare: Timeout la așteptarea MISSION_REQUEST pentru waypoint {i}")
                    # Poți încerca să retrimiți MISSION_COUNT sau să anulezi
                    self.mav_connection.mav.mission_ack_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_ERROR)
                    return False

                wp_data = waypoints_data[i]
                print(f"Trimitere MISSION_ITEM pentru waypoint {i} (ID: {wp_data.get('id', i)})")

                # Construiește mesajul MISSION_ITEM (sau MISSION_ITEM_INT)
                # Aici folosim MISSION_ITEM (lat/lon ca float). MISSION_ITEM_INT e preferat pentru precizie.
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT # Altitudine relativă la Home
                command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT # Comanda standard pentru waypoint
                current = 1 if i == 0 else 0 # Primul waypoint este 'current' pentru unele GCS/firmware-uri? Verifică documentația.
                autocontinue = 1 # Mergi automat la următorul waypoint

                # Parametri specifici MAV_CMD_NAV_WAYPOINT
                param1 = wp_data.get("actions", {}).get("wait_time", 0) # Timp de așteptare la waypoint (secunde)
                param2 = wp_data.get("acceptance_radius", 5) # Raza de acceptare (metri) - ajustează!
                param3 = 0 # Pass through waypoint (0) or orbit (radius)
                param4 = float('nan') # Yaw angle (NaN to keep current heading)
                latitude = wp_data["latitude"]
                longitude = wp_data["longitude"]
                altitude = wp_data["altitude"] # Asigură-te că e în metri

                self.mav_connection.mav.mission_item_send(
                    target_system, target_component,
                    i,          # seq
                    frame,      # frame
                    command,    # command
                    current,    # current
                    autocontinue, # autocontinue
                    param1, param2, param3, param4, # params 1-4
                    latitude, longitude, altitude  # x, y, z (lat, lon, alt)
                )

                # TODO: Adaugă logica pentru 'actions' (takePhoto, recordVideo)
                # Asta ar implica trimiterea unor comenzi DO_* după MAV_CMD_NAV_WAYPOINT
                # sau folosirea unor comenzi MAV_CMD specifice în loc de NAV_WAYPOINT dacă acțiunea
                # trebuie executată exact la atingerea waypoint-ului.

            # 3. Așteaptă confirmarea finală MISSION_ACK
            print("Așteptare MISSION_ACK final...")
            msg = self.mav_connection.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if not msg:
                print("Eroare: Timeout la așteptarea MISSION_ACK final.")
                return False

            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("MISSION_ACK primit: Misiune acceptată de dronă.")
                return True
            else:
                # Afișează codul erorii
                error_name = mavutil.mavlink.enums['MAV_MISSION_RESULT'][msg.type].name
                print(f"EROARE: Misiunea a fost respinsă de dronă cu eroarea: {error_name} ({msg.type})")
                return False

        except Exception as e:
            print(f"EROARE excepție în timpul încărcării misiunii: {e}")
            # Trimite un NACK dacă este posibil
            try:
                 self.mav_connection.mav.mission_ack_send(target_system, target_component, mavutil.mavlink.MAV_MISSION_ERROR)
            except:
                 pass # Ignoră erorile la trimiterea NACK-ului
            return False

# --- Cod WebSocket Server (în mare parte neschimbat) ---

async def telemetry_server(websocket, path):
    """Gestionează conexiunile și mesajele WebSocket"""
    print(f"Client conectat: {websocket.remote_address}")
    sender_task = None
    try:
        # Creează instanța TelemetryData pentru acest client
        # Într-o aplicație reală, ai putea avea o singură instanță TelemetryData
        # dacă serverul deservește o singură dronă și mai mulți clienți văd aceleași date.
        # Aici, fiecare client primește propria instanță (util pentru testare).
        telemetry = TelemetryData()

        # Trimite datele inițiale (stare și waypoint-uri dacă există)
        await send_telemetry(websocket, telemetry)
        await send_mission_status(websocket, telemetry)
        await send_waypoints(websocket, telemetry) # Trimite waypoint-urile curente (dacă sunt)

        # Pornește task-ul care trimite actualizări periodice
        sender_task = asyncio.create_task(telemetry_sender(websocket, telemetry))

        # Bucla de recepție comenzi
        async for message in websocket:
            try:
                command = json.loads(message)
                print(f"Comandă primită: {command}")

                # Gestionează comanda
                telemetry.handle_command(command)

                # Trimite imediat statusul actualizat după procesarea comenzii
                await send_mission_status(websocket, telemetry)
                if command.get("type") == "UPLOAD_MISSION":
                    # Trimite lista actualizată de waypoints dacă misiunea a fost încărcată
                     await send_waypoints(websocket, telemetry)
                if telemetry.current_waypoint is not None:
                    await send_current_waypoint(websocket, telemetry)

            except json.JSONDecodeError:
                print(f"Eroare decodare JSON: {message}")
            except Exception as e:
                print(f"Eroare la procesarea mesajului client: {e}")

    except websockets.exceptions.ConnectionClosedOK:
        print(f"Client deconectat normal: {websocket.remote_address}")
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"Client deconectat cu eroare: {websocket.remote_address} - {e}")
    except Exception as e:
         print(f"Eroare neașteptată în handler-ul clientului: {e}")
    finally:
        # Oprește task-ul de trimitere când clientul se deconectează
        if sender_task:
            sender_task.cancel()
            try:
                await sender_task # Așteaptă finalizarea anulării
            except asyncio.CancelledError:
                pass # Ne așteptăm la asta
        print(f"Curățare resurse pentru clientul deconectat: {websocket.remote_address}")
        # Închide conexiunea MAVLink dacă aceasta este specifică clientului
        # (Nu este cazul în modelul actual unde TelemetryData este per client)
        # if 'telemetry' in locals() and telemetry.mav_connection:
        #     telemetry.mav_connection.close()


async def telemetry_sender(websocket, telemetry):
    """Task asincron care trimite periodic datele de telemetrie"""
    update_interval = 0.5 # Secunde (2 Hz)
    last_status_sent = 0
    last_waypoint_sent = 0
    status_interval = 2 # Trimite statusul mai rar (la fiecare 2 secunde)
    waypoint_interval = 2 # Trimite waypoint curent mai rar

    try:
        while True:
            # Actualizează datele de telemetrie (citire hardware sau simulare)
            telemetry.update_telemetry()

            # Trimite pachetul principal de telemetrie
            await send_telemetry(websocket, telemetry)

            current_time = time.time()
            # Trimite statusul misiunii periodic
            if current_time - last_status_sent > status_interval:
                 await send_mission_status(websocket, telemetry)
                 last_status_sent = current_time

            # Trimite waypoint-ul curent periodic (dacă există)
            if telemetry.current_waypoint is not None and current_time - last_waypoint_sent > waypoint_interval:
                 await send_current_waypoint(websocket, telemetry)
                 last_waypoint_sent = current_time

            # Așteaptă înainte de următoarea actualizare
            await asyncio.sleep(update_interval)
    except asyncio.CancelledError:
        print("Task-ul telemetry_sender anulat.")
    except websockets.exceptions.ConnectionClosed:
        print("Task-ul telemetry_sender oprit: Conexiune închisă.")
    except Exception as e:
        print(f"Eroare în task-ul telemetry_sender: {e}")


async def send_message(websocket, message_type, payload):
    """Funcție helper pentru trimiterea mesajelor JSON"""
    try:
        message = {
            "type": message_type,
            "payload": payload
        }
        await websocket.send(json.dumps(message))
    except websockets.exceptions.ConnectionClosed:
        # Ignoră eroarea dacă conexiunea s-a închis între timp
        pass
    except Exception as e:
        print(f"Eroare la trimiterea mesajului {message_type}: {e}")

async def send_telemetry(websocket, telemetry):
    """Trimite datele principale de telemetrie"""
    await send_message(websocket, "TELEMETRY_UPDATE", telemetry.telemetry)

async def send_mission_status(websocket, telemetry):
    """Trimite statusul curent al misiunii"""
    await send_message(websocket, "MISSION_STATUS_UPDATE", {"status": telemetry.mission_status})

async def send_current_waypoint(websocket, telemetry):
    """Trimite ID-ul waypoint-ului curent"""
    await send_message(websocket, "CURRENT_WAYPOINT_UPDATE", {"waypointId": telemetry.current_waypoint})

async def send_waypoints(websocket, telemetry):
    """Trimite lista completă de waypoints (util după upload/conectare)"""
    await send_message(websocket, "WAYPOINTS_LIST_UPDATE", {"waypoints": telemetry.waypoints})

# Funcția principală
async def main():
    """Pornește serverul WebSocket"""
    print(f"Pornire server WebSocket pe portul {WEBSOCKET_PORT}")
    # Poți adăuga aici logica pentru a avea o singură instanță TelemetryData
    # partajată între toți clienții, dacă este necesar.
    # telemetry_instance = TelemetryData()
    # handler = functools.partial(telemetry_server, telemetry_instance=telemetry_instance)
    # async with websockets.serve(handler, "0.0.0.0", WEBSOCKET_PORT):
    #     await asyncio.Future() # Rulează la infinit

    # Modelul curent: fiecare client are propria instanță TelemetryData
    async with websockets.serve(telemetry_server, "0.0.0.0", WEBSOCKET_PORT):
        print("Serverul rulează. Apăsați Ctrl+C pentru a opri.")
        await asyncio.Future() # Rulează la infinit

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer oprit de utilizator.")
    except Exception as e:
        print(f"Eroare la pornirea serverului: {e}")