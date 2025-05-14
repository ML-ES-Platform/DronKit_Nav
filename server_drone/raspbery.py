# mavlink_dht_server.py
import time
import threading
from flask import Flask, jsonify

# --- Importuri pentru MAVLink ---
try:
    from pymavlink import mavutil
    import serial
except ImportError as e:
    print(f"Eroare importare module MAVLink: {e}")
    print("Rulati 'pip install pyserial pymavlink Flask' pentru a instala modulele necesare")
    exit(1)

# --- Importuri pentru DHT11 ---
try:
    import Adafruit_DHT
except ImportError as e:
    print(f"Eroare importare Adafruit_DHT: {e}")
    print("Rulati 'sudo pip3 install Adafruit_DHT' si asigurati-va ca dependintele sunt satisfacute.")
    print("Consultati documentatia Adafruit pentru DHT.")
    # Nu iesim, poate utilizatorul vrea sa ruleze doar partea MAVLink
    Adafruit_DHT = None # Setam la None pentru a putea verifica ulterior

# --- Configurare MAVLink ---
CONNECTION_STRING = '/dev/serial0' # Sau '/dev/serial0', etc.
BAUD_RATE = 921600

# --- Configurare DHT11 ---
DHT_SENSOR_TYPE = Adafruit_DHT.DHT11 if Adafruit_DHT else None
DHT_GPIO_PIN = 4
DHT_READ_INTERVAL = 5 # Secunde intre citirile DHT

# --- Stocare globala pentru date ---
latest_data = {
    "altitude": None,
    "latitude": None,
    "longitude": None,
    "battery_voltage": None,
    "groundspeed": None,
    "temperature": None, # Nou
    "humidity": None,    # Nou
    "mavlink_connected": False,
    "mavlink_status_message": "Asteptare conexiune MAVLink...",
    "dht_status_message": "Asteptare citire DHT..." if Adafruit_DHT else "Modul Adafruit_DHT neincarcat."
}
data_lock = threading.Lock()

# --- Functia care citeste datele MAVLink intr-un thread separat ---
def mavlink_reader_thread():
    global latest_data
    mav = None

    while True:
        try:
            print(f"MAVLink: Incerc conectarea la {CONNECTION_STRING}...")
            with data_lock:
                latest_data["mavlink_status_message"] = f"Incerc conectarea la {CONNECTION_STRING}..."
                latest_data["mavlink_connected"] = False

            if "serial" in CONNECTION_STRING:
                mav = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
            else:
                mav = mavutil.mavlink_connection(CONNECTION_STRING)

            print("MAVLink: Astept heartbeat...")
            with data_lock:
                latest_data["mavlink_status_message"] = "Astept heartbeat..."
            mav.wait_heartbeat(timeout=10)
            print("MAVLink: Heartbeat primit!")
            with data_lock:
                latest_data["mavlink_connected"] = True
                latest_data["mavlink_status_message"] = "Conectat la MAVLink. Solicit date..."

            message_types_to_request = [
                (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT"),
                (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, "SYS_STATUS"),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, "VFR_HUD")
            ]

            for msg_id, msg_name in message_types_to_request:
                print(f"MAVLink: Solicit date pentru {msg_name} la 1Hz...")
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, 1000000, 0, 0, 0, 0, 0
                )
                ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
                # print(f"MAVLink: ACK pentru {msg_name}: {ack}")


            print("MAVLink: Incep receptia datelor...")
            while True:
                msg = mav.recv_match(
                    type=['GLOBAL_POSITION_INT', 'SYS_STATUS', 'VFR_HUD'],
                    blocking=True, timeout=2
                )
                if msg:
                    msg_type = msg.get_type()
                    with data_lock:
                        latest_data["mavlink_connected"] = True
                        latest_data["mavlink_status_message"] = "Date MAVLink receptionate."
                        if msg_type == 'GLOBAL_POSITION_INT':
                            latest_data["altitude"] = msg.alt / 1000.0
                            latest_data["latitude"] = msg.lat / 1.0e7
                            latest_data["longitude"] = msg.lon / 1.0e7
                        elif msg_type == 'SYS_STATUS':
                            latest_data["battery_voltage"] = msg.voltage_battery / 1000.0
                        elif msg_type == 'VFR_HUD':
                            latest_data["groundspeed"] = msg.groundspeed
                else:
                    print("MAVLink: Timeout la receptia mesajelor. Verific conexiunea...")
                    with data_lock:
                         latest_data["mavlink_status_message"] = "Timeout la receptia mesajelor MAVLink."
                    if not mav.hb_connected:
                         print("MAVLink: Heartbeat pierdut. Reincerc conectarea...")
                         with data_lock:
                            latest_data["mavlink_connected"] = False
                            latest_data["mavlink_status_message"] = "Heartbeat pierdut. Reincerc conectarea..."
                         break
                time.sleep(0.05)
        except serial.SerialException as e:
            print(f"MAVLink: Eroare de conexiune seriala: {e}")
            with data_lock:
                latest_data["mavlink_status_message"] = f"Eroare seriala MAVLink: {e}"
                latest_data["mavlink_connected"] = False
            time.sleep(5)
        except mavutil.mavlink.MAVError as e:
            print(f"MAVLink: Eroare MAVLink: {e}")
            with data_lock:
                latest_data["mavlink_status_message"] = f"Eroare MAVLink: {e}"
                latest_data["mavlink_connected"] = False
            time.sleep(5)
        except Exception as e:
            print(f"MAVLink: Eroare neasteptata in thread-ul MAVLink: {e}")
            with data_lock:
                latest_data["mavlink_status_message"] = f"Eroare MAVLink: {e}"
                latest_data["mavlink_connected"] = False
            time.sleep(5)
        finally:
            if mav:
                mav.close()
                print("MAVLink: Conexiune MAVLink inchisa.")

# --- Functia care citeste datele de la DHT11 intr-un thread separat ---
def dht_reader_thread():
    global latest_data
    if not Adafruit_DHT or not DHT_SENSOR_TYPE:
        print("DHT: Modulul Adafruit_DHT nu este incarcat corect sau senzorul nu este specificat. Thread-ul DHT nu porneste.")
        with data_lock:
            latest_data["dht_status_message"] = "Modul Adafruit_DHT neconfigurat/neincarcat."
        return

    print(f"DHT: Pornesc citirea de la senzorul DHT pe pinul GPIO{DHT_GPIO_PIN} la fiecare {DHT_READ_INTERVAL}s.")
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR_TYPE, DHT_GPIO_PIN)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        with data_lock:
            if humidity is not None and temperature is not None:
                latest_data["temperature"] = round(temperature, 1)
                latest_data["humidity"] = round(humidity, 1)
                latest_data["dht_status_message"] = f"Date DHT citite cu succes la {timestamp}."
                # print(f"DHT: Temp={temperature:.1f}°C, Hum={humidity:.1f}%") # Optional: decomenteaza pentru logging
            else:
                latest_data["dht_status_message"] = f"Eroare la citirea DHT la {timestamp}. Verificati conexiunile."
                # Pastram valorile vechi pentru temp/hum in caz de eroare temporara
                print(f"DHT: Eroare la citirea senzorului la {timestamp}.")
        time.sleep(DHT_READ_INTERVAL)

# --- Aplicatia Flask ---
app = Flask(__name__)

@app.route('/data')
def get_data():
    with data_lock:
        return jsonify(dict(latest_data))

if __name__ == '__main__':
    # Pornim thread-ul MAVLink
    mav_thread = threading.Thread(target=mavlink_reader_thread, daemon=True)
    mav_thread.start()

    # Pornim thread-ul DHT
    if Adafruit_DHT and DHT_SENSOR_TYPE: # Pornim doar daca modulul e incarcat
        dht_thread = threading.Thread(target=dht_reader_thread, daemon=True)
        dht_thread.start()
    else:
        print("Skipping DHT thread due to import/config issues.")


    print("Serverul Flask ruleaza pe http://<IP-ul_Pi-ului>:5000/data")
    app.run(host='0.0.0.0', port=5000)