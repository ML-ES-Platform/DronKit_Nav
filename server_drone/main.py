# mavlink_dht_controller.py
import time
import threading
import logging
from flask import Flask, jsonify, request, render_template_string, send_from_directory
import serial # Already in your try-except
from pymavlink import mavutil
import math
import collections

# --- Setup Logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s')

# --- Importuri pentru DHT11 ---
try:
    import Adafruit_DHT
    DHT_SENSOR_TYPE_DHT11 = Adafruit_DHT.DHT11
    DHT_SENSOR_TYPE_DHT22 = Adafruit_DHT.DHT22
except ImportError:
    logging.warning("Adafruit_DHT module not found. DHT sensor functionality will be disabled.")
    Adafruit_DHT = None
    DHT_SENSOR_TYPE_DHT11 = None
    DHT_SENSOR_TYPE_DHT22 = None

# --- Configuration ---
class Config:
    # MAVLink Configuration
    MAVLINK_CONNECTION_STRING = '/dev/serial0'  # For RPi serial port
    # MAVLINK_CONNECTION_STRING = 'udp:127.0.0.1:14550' # For SITL/UDP
    MAVLINK_BAUD_RATE = 921600 # Common for telemetry radios
    MAVLINK_SOURCE_SYSTEM = 255 # Our GCS system ID
    MAVLINK_SOURCE_COMPONENT = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
    MAVLINK_HEARTBEAT_TIMEOUT = 15 # seconds
    MAVLINK_RECONNECT_DELAY = 5 # seconds
    MAVLINK_COMMAND_RETRY_TIMEOUT = 3 # seconds
    MAVLINK_TARGET_SYSTEM = 1 # Default target system ID
    MAVLINK_TARGET_COMPONENT = 1 # Default target component ID (autopilot)

    # DHT Sensor Configuration
    DHT_ENABLED = Adafruit_DHT is not None
    DHT_SENSOR_TYPE = DHT_SENSOR_TYPE_DHT11 # or DHT_SENSOR_TYPE_DHT22
    DHT_GPIO_PIN = 4
    DHT_READ_INTERVAL = 5  # Seconds

    # Flask Configuration
    FLASK_HOST = '0.0.0.0'
    FLASK_PORT = 5000

    # Flight Configuration
    DEFAULT_TAKEOFF_ALTITUDE = 5.0 # meters
    DEFAULT_GOTO_SPEED = 5.0 # m/s (if settable, otherwise uses drone's default)

# --- Global Data Store ---
latest_data = {
    "mavlink_status": {
        "connected": False,
        "message": "Awaiting MAVLink connection...",
        "last_heartbeat_time": None,
        "target_system_id": None,
        "target_component_id": None,
        "autopilot_type": None,
        "firmware_version": None,
    },
    "dht_status": {
        "message": "DHT sensor reading pending..." if Config.DHT_ENABLED else "DHT module not loaded.",
        "last_read_time": None,
    },
    "drone_state": {
        "armed": False,
        "mode": "UNKNOWN",
        "system_status": None, # From HEARTBEAT
        "is_landed": True, # Approximation
        "active_mission_item": 0,
    },
    "telemetry": {
        "latitude": None,
        "longitude": None,
        "altitude_amsl": None, # Above Mean Sea Level
        "altitude_relative": None, # Above Home/Takeoff
        "vx": None, # Ground speed X
        "vy": None, # Ground speed Y
        "vz": None, # Ground speed Z (vertical)
        "groundspeed": None,
        "heading": None, # Compass heading
        "roll": None, # degrees
        "pitch": None, # degrees
        "yaw": None, # degrees (often same as heading)
        "gps_fix_type": None,
        "satellites_visible": None,
        "eph": None, # GPS HDOP
        "epv": None, # GPS VDOP
        "battery_voltage": None,
        "battery_current": None,
        "battery_remaining_percent": None,
        "throttle_percentage": None, # VFR_HUD
        "climb_rate": None, # VFR_HUD
        "temperature_internal": None, # SCALED_PRESSURE or HIGHRES_IMU
        "pressure_absolute": None, # SCALED_PRESSURE
        "temperature_dht": None,
        "humidity_dht": None,
    },
    "autopilot_messages": [], # Stores STATUSTEXT messages
    "last_command_ack": {
        "command_id": None,
        "result": None,
        "result_text": None,
        "timestamp": None,
    }
}
data_lock = threading.Lock()
mav_connection = None
command_queue = collections.deque(maxlen=10) # Queue for MAVLink commands

# --- MAVLink Helper Functions ---
def is_mavlink_connected():
    global mav_connection
    return mav_connection is not None and latest_data["mavlink_status"]["connected"]

def update_mavlink_status(connected, message, target_system_id=None, target_component_id=None):
    with data_lock:
        latest_data["mavlink_status"]["connected"] = connected
        latest_data["mavlink_status"]["message"] = message
        if connected:
            latest_data["mavlink_status"]["last_heartbeat_time"] = time.time()
            if target_system_id is not None:
                latest_data["mavlink_status"]["target_system_id"] = target_system_id
                Config.MAVLINK_TARGET_SYSTEM = target_system_id # Update config with actual target
            if target_component_id is not None:
                latest_data["mavlink_status"]["target_component_id"] = target_component_id
                Config.MAVLINK_TARGET_COMPONENT = target_component_id # Update config
        else:
            latest_data["drone_state"]["armed"] = False # If disconnected, assume disarmed for safety
            latest_data["drone_state"]["mode"] = "UNKNOWN"

def send_mavlink_command_long(command_id, params, confirmation=0, blocking=True, timeout=Config.MAVLINK_COMMAND_RETRY_TIMEOUT):
    global mav_connection
    if not is_mavlink_connected():
        logging.error(f"MAVLink not connected. Cannot send command {mavutil.mavlink.enums['MAV_CMD'][command_id].name if command_id in mavutil.mavlink.enums['MAV_CMD'] else command_id}.")
        return False, "MAVLink not connected"

    p1, p2, p3, p4, p5, p6, p7 = params
    try:
        mav_connection.mav.command_long_send(
            Config.MAVLINK_TARGET_SYSTEM, Config.MAVLINK_TARGET_COMPONENT,
            command_id, confirmation,
            p1, p2, p3, p4, p5, p6, p7
        )
        cmd_name = mavutil.mavlink.enums['MAV_CMD'][command_id].name if command_id in mavutil.mavlink.enums['MAV_CMD'] else str(command_id)
        logging.info(f"Sent MAV_CMD: {cmd_name} with params: {params}")

        if blocking:
            ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
            if ack_msg:
                ack_result_text = mavutil.mavlink.enums['MAV_RESULT'][ack_msg.result].name if ack_msg.result in mavutil.mavlink.enums['MAV_RESULT'] else str(ack_msg.result)
                logging.info(f"ACK for {cmd_name}: {ack_result_text} (Cmd: {ack_msg.command}, Result: {ack_msg.result})")
                with data_lock:
                    latest_data["last_command_ack"] = {
                        "command_id": ack_msg.command,
                        "result": ack_msg.result,
                        "result_text": ack_result_text,
                        "timestamp": time.time()
                    }
                return ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED, ack_result_text
            else:
                logging.warning(f"No ACK received for {cmd_name} within timeout.")
                with data_lock:
                    latest_data["last_command_ack"] = {
                        "command_id": command_id,
                        "result": -1, # Custom: timeout
                        "result_text": "Timeout waiting for ACK",
                        "timestamp": time.time()
                    }
                return False, "Timeout waiting for ACK"
        return True, "Command sent (non-blocking)"
    except Exception as e:
        logging.error(f"Error sending MAVLink command {command_id}: {e}")
        return False, str(e)

def set_message_interval(message_id, frequency_hz):
    global mav_connection
    if not is_mavlink_connected():
        logging.error(f"MAVLink not connected. Cannot set interval for message ID {message_id}.")
        return

    interval_us = int(1_000_000 / frequency_hz) if frequency_hz > 0 else 0 # 0 to disable, -1 to restore default
    logging.info(f"Requesting MAVLink message ID {message_id} at {frequency_hz} Hz (interval {interval_us} us)...")
    # Use command_long_send for safety, though direct message_interval_send works too
    send_mavlink_command_long(
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        [message_id, interval_us, 0, 0, 0, 0, 0],
        blocking=True, # Wait for ACK
        timeout=1
    )

# --- MAVLink Communication Thread ---
def mavlink_communication_thread_func():
    global mav_connection, latest_data, command_queue

    while True:
        try:
            logging.info(f"MAVLink: Attempting connection to {Config.MAVLINK_CONNECTION_STRING} at {Config.MAVLINK_BAUD_RATE} baud...")
            update_mavlink_status(False, f"Attempting connection to {Config.MAVLINK_CONNECTION_STRING}...")

            if "serial" in Config.MAVLINK_CONNECTION_STRING.lower():
                mav_connection = mavutil.mavlink_connection(
                    Config.MAVLINK_CONNECTION_STRING,
                    baud=Config.MAVLINK_BAUD_RATE,
                    source_system=Config.MAVLINK_SOURCE_SYSTEM,
                    source_component=Config.MAVLINK_SOURCE_COMPONENT
                )
            else: # UDP or TCP
                mav_connection = mavutil.mavlink_connection(
                    Config.MAVLINK_CONNECTION_STRING,
                    source_system=Config.MAVLINK_SOURCE_SYSTEM,
                    source_component=Config.MAVLINK_SOURCE_COMPONENT
                )

            logging.info("MAVLink: Waiting for first heartbeat...")
            update_mavlink_status(False, "Waiting for first heartbeat...")
            hb = mav_connection.wait_heartbeat(timeout=Config.MAVLINK_HEARTBEAT_TIMEOUT)
            if hb is None:
                logging.error(f"MAVLink: No heartbeat received within {Config.MAVLINK_HEARTBEAT_TIMEOUT}s. Retrying...")
                update_mavlink_status(False, f"No heartbeat received. Reconnecting in {Config.MAVLINK_RECONNECT_DELAY}s...")
                if mav_connection: mav_connection.close()
                mav_connection = None
                time.sleep(Config.MAVLINK_RECONNECT_DELAY)
                continue

            logging.info(f"MAVLink: Heartbeat received! System ID: {mav_connection.target_system}, Component ID: {mav_connection.target_component}")
            update_mavlink_status(True, "MAVLink Connected. Initializing data streams...",
                                  mav_connection.target_system, mav_connection.target_component)

            with data_lock:
                latest_data["mavlink_status"]["autopilot_type"] = mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name if hb.autopilot in mavutil.mavlink.enums['MAV_AUTOPILOT'] else 'UNKNOWN_AUTOPILOT'
                latest_data["drone_state"]["mode"] = mavutil.mavlink.enums['MAV_MODE_FLAG'][hb.base_mode].name if hb.base_mode in mavutil.mavlink.enums['MAV_MODE_FLAG'] else 'UNKNOWN_MODE'
                latest_data["drone_state"]["system_status"] = mavutil.mavlink.enums['MAV_STATE'][hb.system_status].name if hb.system_status in mavutil.mavlink.enums['MAV_STATE'] else 'UNKNOWN_STATE'


            # Request data streams or individual message intervals
            # ArduPilot GCS stream rates: https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html
            # For Pixhawk/ArduPilot, setting stream rates is often preferred
            # mav_connection.mav.request_data_stream_send(mav_connection.target_system, mav_connection.target_component,
            #                                          mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 1) # Stop all streams
            # time.sleep(0.1)
            # mav_connection.mav.request_data_stream_send(mav_connection.target_system, mav_connection.target_component,
            #                                          mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1) # SYS_STATUS, POWER_STATUS, MEMINFO, etc. @ 2Hz
            # mav_connection.mav.request_data_stream_send(mav_connection.target_system, mav_connection.target_component,
            #                                          mavutil.mavlink.MAV_DATA_STREAM_POSITION, 3, 1) # GLOBAL_POSITION_INT @ 3Hz
            # mav_connection.mav.request_data_stream_send(mav_connection.target_system, mav_connection.target_component,
            #                                          mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 2, 1) # ATTITUDE, AHRS, HWSTATUS @ 2Hz
            # mav_connection.mav.request_data_stream_send(mav_connection.target_system, mav_connection.target_component,
            #                                          mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 2, 1) # VFR_HUD @ 2Hz

            # Alternatively, use MAV_CMD_SET_MESSAGE_INTERVAL for finer control
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1) # Already comes, but good practice
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5) # 5 Hz
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 2) # 2 Hz
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5) # 5 Hz
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1)
            # set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 0.5) # Optional
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 1) # Optional
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 1) # For landed state
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 0.2) # Once or low rate
            set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 1)

            logging.info("MAVLink: Data streams requested. Receiving data...")
            update_mavlink_status(True, "Receiving MAVLink data.")

            last_heartbeat_check_time = time.time()

            while True:
                # Check for lost heartbeat
                if time.time() - last_heartbeat_check_time > Config.MAVLINK_HEARTBEAT_TIMEOUT:
                    hb = mav_connection.wait_heartbeat(timeout=1) # Quick check
                    if hb is None:
                        logging.warning(f"MAVLink: Heartbeat lost. Reconnecting...")
                        update_mavlink_status(False, f"Heartbeat lost. Reconnecting in {Config.MAVLINK_RECONNECT_DELAY}s...")
                        break # Break inner loop to reconnect
                    else: # Heartbeat is back
                        logging.info("MAVLink: Heartbeat re-established.")
                        update_mavlink_status(True, "MAVLink Connected. Receiving data.", mav_connection.target_system, mav_connection.target_component)
                        last_heartbeat_check_time = time.time() # Reset check time


                # Process incoming MAVLink messages
                # Use a short timeout to allow checking command_queue periodically
                msg = mav_connection.recv_match(blocking=False, timeout=0.05) # Non-blocking with timeout
                if msg:
                    msg_type = msg.get_type()
                    # logging.debug(f"Received MAVLink message: {msg_type}") # Very verbose

                    with data_lock:
                        if not latest_data["mavlink_status"]["connected"]: # If connection was marked false by timeout
                           update_mavlink_status(True, "MAVLink Connected. Receiving data.", mav_connection.target_system, mav_connection.target_component)
                        latest_data["mavlink_status"]["last_heartbeat_time"] = time.time()
                        last_heartbeat_check_time = time.time() # Reset check time on any message

                        if msg_type == 'BAD_DATA':
                            logging.warning(f"MAVLink: Bad data received: {msg}")
                        elif msg_type == 'HEARTBEAT':
                            latest_data["drone_state"]["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                            current_mode_name = "UNKNOWN"
                            # Ardupilot custom modes are in custom_mode field
                            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                                try:
                                    # This requires flightmode list from the specific ArduPilot firmware
                                    # For simplicity, we'll try to get it from pymavlink's standard modes if possible
                                    # or just use the number.
                                    # For ArduCopter:
                                    # flight_mode_map = {mode.value: mode.name for mode in mavutil.mavlink.enums['COPTER_MODE']}
                                    # current_mode_name = flight_mode_map.get(msg.custom_mode, f"CUSTOM_MODE_{msg.custom_mode}")
                                    # For now, let's use a simpler approach or rely on base_mode if custom_mode is complex
                                    # A more robust way is to fetch MAV_MODE_MAP from AUTOPILOT_VERSION
                                    pass # Fallback to base_mode or just custom_mode number for now
                                except Exception as e:
                                    logging.debug(f"Could not map custom_mode {msg.custom_mode}: {e}")

                            # If custom mode wasn't easily mapped, try base_mode for some common states
                            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                                latest_data["drone_state"]["mode"] = f"CUSTOM({msg.custom_mode})" # Placeholder
                            elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED:
                                latest_data["drone_state"]["mode"] = "AUTO"
                            elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
                                latest_data["drone_state"]["mode"] = "GUIDED"
                            elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED:
                                latest_data["drone_state"]["mode"] = "STABILIZE"
                            else:
                                latest_data["drone_state"]["mode"] = "UNKNOWN_BASE_MODE"

                            latest_data["drone_state"]["system_status"] = mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name if msg.system_status in mavutil.mavlink.enums['MAV_STATE'] else str(msg.system_status)
                            latest_data["mavlink_status"]["firmware_version"] = f"{msg.mavlink_version}.{msg.type}.{msg.autopilot}" # Simplified

                        elif msg_type == 'GLOBAL_POSITION_INT':
                            latest_data["telemetry"]["latitude"] = msg.lat / 1.0e7
                            latest_data["telemetry"]["longitude"] = msg.lon / 1.0e7
                            latest_data["telemetry"]["altitude_amsl"] = msg.alt / 1000.0
                            latest_data["telemetry"]["altitude_relative"] = msg.relative_alt / 1000.0
                            latest_data["telemetry"]["vx"] = msg.vx / 100.0 # cm/s to m/s
                            latest_data["telemetry"]["vy"] = msg.vy / 100.0 # cm/s to m/s
                            latest_data["telemetry"]["vz"] = msg.vz / 100.0 # cm/s to m/s
                            latest_data["telemetry"]["heading"] = msg.hdg / 100.0 # cdeg to deg

                        elif msg_type == 'SYS_STATUS':
                            latest_data["telemetry"]["battery_voltage"] = msg.voltage_battery / 1000.0
                            latest_data["telemetry"]["battery_current"] = msg.current_battery / 100.0 if msg.current_battery != -1 else None # Amps (-1 if not supported)
                            latest_data["telemetry"]["battery_remaining_percent"] = msg.battery_remaining if msg.battery_remaining != -1 else None # Percent (-1 if not supported)

                        elif msg_type == 'VFR_HUD':
                            latest_data["telemetry"]["groundspeed"] = msg.groundspeed # m/s
                            latest_data["telemetry"]["throttle_percentage"] = msg.throttle # percent
                            # alt is ASL, taken from GLOBAL_POSITION_INT.relative_alt for AGL
                            latest_data["telemetry"]["climb_rate"] = msg.climb # m/s
                            # heading is also in GLOBAL_POSITION_INT.hdg

                        elif msg_type == 'ATTITUDE':
                            latest_data["telemetry"]["roll"] = math.degrees(msg.roll)
                            latest_data["telemetry"]["pitch"] = math.degrees(msg.pitch)
                            latest_data["telemetry"]["yaw"] = math.degrees(msg.yaw) # Often relative to startup, heading is better for navigation

                        elif msg_type == 'GPS_RAW_INT':
                            latest_data["telemetry"]["gps_fix_type"] = msg.fix_type
                            latest_data["telemetry"]["satellites_visible"] = msg.satellites_visible
                            latest_data["telemetry"]["eph"] = msg.eph / 100.0 if msg.eph != 65535 else None # cm to m
                            latest_data["telemetry"]["epv"] = msg.epv / 100.0 if msg.epv != 65535 else None # cm to m

                        elif msg_type == 'SCALED_PRESSURE': # often temperature of barometer
                            latest_data["telemetry"]["pressure_absolute"] = msg.press_abs # hPa
                            latest_data["telemetry"]["temperature_internal"] = msg.temperature / 100.0 # cdegC

                        elif msg_type == 'SCALED_PRESSURE2': # backup barometer or external
                            pass # Can add if multiple baros

                        elif msg_type == 'HIGHRES_IMU': # Can also get temperature from IMUs
                             if latest_data["telemetry"]["temperature_internal"] is None and hasattr(msg, 'temperature') and msg.temperature != 0: # Some IMUs might report 0 if no temp sensor
                                latest_data["telemetry"]["temperature_internal"] = msg.temperature # degC

                        elif msg_type == 'STATUSTEXT':
                            message_text = msg.text.rstrip('\0')
                            logging.info(f"AUTOPILOT MSG ({msg.severity}): {message_text}")
                            latest_data["autopilot_messages"].append(f"[{time.strftime('%H:%M:%S')}|S{msg.severity}] {message_text}")
                            if len(latest_data["autopilot_messages"]) > 20: # Keep last 20 messages
                                latest_data["autopilot_messages"].pop(0)

                        elif msg_type == 'EXTENDED_SYS_STATE':
                            if msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                                latest_data["drone_state"]["is_landed"] = True
                            elif msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR or \
                                 msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF or \
                                 msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
                                latest_data["drone_state"]["is_landed"] = False
                            # else: MAV_LANDED_STATE_UNDEFINED

                        elif msg_type == 'HOME_POSITION':
                            # Useful for RTL verification or displaying home
                            pass # Can store home_lat, home_lon, home_alt

                        elif msg_type == 'MISSION_CURRENT':
                            latest_data["drone_state"]["active_mission_item"] = msg.seq

                        elif msg_type == 'COMMAND_ACK': # Also handled by send_mavlink_command_long but good to log here too
                            ack_result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name if msg.result in mavutil.mavlink.enums['MAV_RESULT'] else str(msg.result)
                            cmd_name = mavutil.mavlink.enums['MAV_CMD'][msg.command].name if msg.command in mavutil.mavlink.enums['MAV_CMD'] else str(msg.command)
                            logging.info(f"Received async COMMAND_ACK for {cmd_name}: {ack_result_text}")
                            # Update global ACK status if it's newer or for a different command
                            with data_lock:
                                if latest_data["last_command_ack"]["timestamp"] is None or time.time() > latest_data["last_command_ack"]["timestamp"] or latest_data["last_command_ack"]["command_id"] != msg.command :
                                    latest_data["last_command_ack"] = {
                                        "command_id": msg.command,
                                        "result": msg.result,
                                        "result_text": ack_result_text,
                                        "timestamp": time.time()
                                    }
                # Process command queue
                if command_queue:
                    cmd_task = command_queue.popleft()
                    cmd_func = cmd_task['function']
                    cmd_args = cmd_task['args']
                    cmd_kwargs = cmd_task['kwargs']
                    logging.info(f"Executing command from queue: {cmd_func.__name__} with args {cmd_args} and kwargs {cmd_kwargs}")
                    try:
                        # These functions (e.g., internal_arm_drone) will call send_mavlink_command_long
                        cmd_func(*cmd_args, **cmd_kwargs)
                    except Exception as e:
                        logging.error(f"Error executing command {cmd_func.__name__} from queue: {e}")

                time.sleep(0.01) # Brief sleep to yield CPU

        except serial.SerialException as e:
            logging.error(f"MAVLink: Serial connection error: {e}")
            update_mavlink_status(False, f"Serial error: {e}. Reconnecting in {Config.MAVLINK_RECONNECT_DELAY}s...")
        except mavutil.mavlink.MAVError as e:
            logging.error(f"MAVLink: MAVLink protocol error: {e}")
            update_mavlink_status(False, f"MAVLink error: {e}. Reconnecting in {Config.MAVLINK_RECONNECT_DELAY}s...")
        except ConnectionResetError as e:
            logging.error(f"MAVLink: Connection reset error: {e}")
            update_mavlink_status(False, f"Connection reset. Reconnecting in {Config.MAVLINK_RECONNECT_DELAY}s...")
        except Exception as e:
            logging.error(f"MAVLink: Unexpected error in MAVLink thread: {e}", exc_info=True)
            update_mavlink_status(False, f"Unexpected MAVLink error. Reconnecting in {Config.MAVLINK_RECONNECT_DELAY}s...")
        finally:
            if mav_connection:
                try:
                    mav_connection.close()
                except Exception as e_close:
                    logging.error(f"Error closing MAVLink connection: {e_close}")
            mav_connection = None
            update_mavlink_status(False, f"MAVLink disconnected. Retrying in {Config.MAVLINK_RECONNECT_DELAY}s...")
            time.sleep(Config.MAVLINK_RECONNECT_DELAY)


# --- Drone Command Functions (to be queued) ---
# These are internal functions called by the MAVLink thread from the queue
def _internal_set_mode(mode_name_or_id):
    global mav_connection
    if not is_mavlink_connected():
        return False, "Not connected"

    mode_id = None
    if isinstance(mode_name_or_id, str):
        # ArduPilot specific mode setting (more reliable for custom modes)
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
        # For ArduCopter, mode_name_or_id could be 'STABILIZE', 'ALT_HOLD', 'LOITER', 'GUIDED', 'AUTO', 'RTL', 'LAND'
        # These need to be mapped to custom mode numbers for ArduPilot.
        # Pymavlink has some standard mappings for common modes.
        # mavutil.mavlink.MAV_MODE_GUIDED_ARMED or MAV_MODE_GUIDED_DISARMED etc.
        # MAV_CMD_DO_SET_MODE allows setting base_mode and custom_mode.

        # Try to find a standard mode ID first
        try:
            if hasattr(mavutil.mavlink, f'MAV_MODE_{mode_name_or_id.upper()}'):
                 mode_id = getattr(mavutil.mavlink, f'MAV_MODE_{mode_name_or_id.upper()}')
                 # This sets the base_mode directly, might not be enough for ArduPilot custom modes
                 return send_mavlink_command_long(
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    [mode_id, 0, 0, 0, 0, 0, 0] # param1 = MAV_MODE, param2 = custom_mode
                )

            # ArduPilot specific mode string to custom_mode ID mapping (example for Copter)
            # This mapping should ideally be fetched from the vehicle or be comprehensive
            # For simplicity, using a small map here.
            mode_mapping_copter = {
                "STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4,
                "LOITER": 5, "RTL": 6, "LAND": 9, "DRIFT": 11, "SPORT": 13,
                "FLIP": 14, "AUTOTUNE": 15, "POSHOLD": 16, "BRAKE": 17,
                "THROW": 18, "AVOID_ADSB": 19, "GUIDED_NOGPS": 20,
                "SMART_RTL": 21, "FLOWHOLD": 22, "FOLLOW": 23, "ZIGZAG": 24,
                "SYSTEMID": 25, "AUTOROTATE": 26, "AUTO_RTL": 27
            }
            # Plane/Rover have different mappings. Assuming Copter for now.
            # A robust solution queries capabilities or uses more generic MAVLink modes.
            custom_mode_id = mode_mapping_copter.get(mode_name_or_id.upper())
            if custom_mode_id is not None:
                logging.info(f"Setting ArduPilot custom mode: {mode_name_or_id} ({custom_mode_id})")
                # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
                base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if latest_data["drone_state"]["armed"]: # Preserve armed state
                    base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

                return send_mavlink_command_long(
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    [base_mode, custom_mode_id, 0, 0, 0, 0, 0]
                )
            else:
                logging.error(f"Mode '{mode_name_or_id}' not recognized for direct custom mode setting.")
                return False, f"Mode '{mode_name_or_id}' not recognized"

        except Exception as e:
            logging.error(f"Error setting mode {mode_name_or_id}: {e}")
            return False, str(e)

    elif isinstance(mode_name_or_id, int): # Assume it's a MAV_MODE enum value
        mode_id = mode_name_or_id
        return send_mavlink_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [mode_id, 0, 0, 0, 0, 0, 0] # param1 = MAV_MODE, param2 = custom_mode (0 for none)
        )
    else:
        return False, "Invalid mode type"


def _internal_arm_disarm(arm: bool):
    if not is_mavlink_connected(): return False, "Not connected"
    action = 1.0 if arm else 0.0
    return send_mavlink_command_long(
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        [action, 0, 0, 0, 0, 0, 0] # param1: 1 to arm, 0 to disarm
    )

def _internal_takeoff(altitude_m: float):
    if not is_mavlink_connected(): return False, "Not connected"
    # Check if in GUIDED mode or a mode that allows takeoff command
    current_mode = latest_data["drone_state"]["mode"]
    if "GUIDED" not in current_mode.upper() and "AUTO" not in current_mode.upper() : # AUTO might be in mission
        logging.warning(f"Takeoff command sent but drone not in GUIDED or AUTO mode (current: {current_mode}). Attempting to set GUIDED.")
        success, msg = _internal_set_mode("GUIDED") # Ensure GUIDED for takeoff command
        if not success:
            return False, f"Failed to set GUIDED mode for takeoff: {msg}"
        time.sleep(0.5) # Give time for mode change to reflect

    if not latest_data["drone_state"]["armed"]:
        logging.warning("Drone not armed. Attempting to arm before takeoff.")
        success, msg = _internal_arm_disarm(True)
        if not success:
            return False, f"Failed to arm for takeoff: {msg}"
        # Wait for armed state to be confirmed via telemetry or a short delay
        time.sleep(2) # Give time for arming
        if not latest_data["drone_state"]["armed"]: # Check again
             return False, "Arming confirmation not received after takeoff attempt."


    logging.info(f"Sending MAV_CMD_NAV_TAKEOFF to altitude {altitude_m}m.")
    # Pitch, Yaw are typically 0 for takeoff command unless specific heading is desired.
    # For ArduPilot, a zero latitude, longitude, and yaw means current location and heading.
    return send_mavlink_command_long(
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        [0, 0, 0, 0, # Param 1-4: Pitch (ignored), Empty, Empty, Yaw (0 for current)
         0, 0, # Param 5-6: Latitude, Longitude (0 for current)
         altitude_m] # Param 7: Altitude
    )

def _internal_goto_location(lat: float, lon: float, alt_amsl: float, speed_ms=None):
    if not is_mavlink_connected(): return False, "Not connected"

    current_mode = latest_data["drone_state"]["mode"]
    if "GUIDED" not in current_mode.upper():
        logging.warning(f"GoTo command sent but drone not in GUIDED mode (current: {current_mode}). Setting GUIDED.")
        success, msg = _internal_set_mode("GUIDED")
        if not success:
            return False, f"Failed to set GUIDED mode for GoTo: {msg}"
        time.sleep(0.5) # Allow mode change

    if not latest_data["drone_state"]["armed"]:
        return False, "Drone is not armed. Cannot go to location."

    logging.info(f"Sending SET_POSITION_TARGET_GLOBAL_INT to Lat: {lat}, Lon: {lon}, Alt(AMSL): {alt_amsl}m")

    # Using SET_POSITION_TARGET_GLOBAL_INT for guided flight.
    # Type_mask can ignore certain fields (e.g., velocity, acceleration).
    # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT uses altitude relative to home.
    # MAV_FRAME_GLOBAL_INT or MAV_FRAME_GLOBAL uses AMSL altitude.
    # We'll use MAV_FRAME_GLOBAL_INT for AMSL altitude.
    # Mask to use only position: IGNORE_VX, IGNORE_VY, IGNORE_VZ, IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ, IGNORE_YAW, IGNORE_YAW_RATE
    type_mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

    mav_connection.mav.set_position_target_global_int_send(
        0, # time_boot_ms (not used)
        Config.MAVLINK_TARGET_SYSTEM, Config.MAVLINK_TARGET_COMPONENT,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # Coordinate frame (AMSL alt)
        type_mask,
        int(lat * 1e7), int(lon * 1e7), float(alt_amsl),
        0, 0, 0, # vx, vy, vz (ignored by type_mask)
        0, 0, 0, # afx, afy, afz (ignored by type_mask)
        0, 0      # yaw, yaw_rate (ignored by type_mask)
    )
    # SET_POSITION_TARGET_GLOBAL_INT does not have a direct ACK in the same way as COMMAND_LONG.
    # Success is typically monitored by observing drone movement or MISSION_ITEM_REACHED (if part of a mission).
    # For simple GUIDED mode, we assume it's sent.
    # We could also use MAV_CMD_NAV_WAYPOINT for a single goto, which does have an ACK.
    # Let's try MAV_CMD_NAV_WAYPOINT for better feedback.
    # Hold time 0, Acceptance Radius (use system default, or e.g. 2m), PassThru 0 (stop at WP)
    # Param4 Yaw (NaN for don't change, 0 for facing next WP or use MAV_CMD_CONDITION_YAW before)
    # For simplicity, using 0 for yaw (face waypoint or use current logic of drone).
    return send_mavlink_command_long(
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        [0, # Hold time
         2, # Acceptance radius (meters), 0 for system default
         0, # Pass through: 0 to stop at WP
         float('nan'), # Desired yaw angle (NaN for don't change)
         lat, lon, alt_amsl]
    )

def _internal_return_to_launch():
    return _internal_set_mode("RTL") # ArduPilot specific mode string

def _internal_land():
    # For ArduPilot, setting mode to LAND is preferred.
    # MAV_CMD_NAV_LAND can also be used, typically with current lat/lon.
    success, msg = _internal_set_mode("LAND")
    if success:
        return success, msg
    else: # Fallback or if mode set fails, try NAV_LAND command
        logging.info("Setting LAND mode failed or not supported, trying MAV_CMD_NAV_LAND at current location.")
        current_lat = latest_data["telemetry"]["latitude"]
        current_lon = latest_data["telemetry"]["longitude"]
        if current_lat is not None and current_lon is not None:
            return send_mavlink_command_long(
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                [0,0,0,0, # Abort alt, Precision land mode, Empty, Yaw
                current_lat, current_lon, 0] # Land at current lat/lon, alt is usually ignored or ground alt.
            )
        else:
            return False, "Cannot execute NAV_LAND: Current position unknown."


# --- DHT Sensor Reader Thread ---
def dht_reader_thread_func():
    global latest_data
    if not Config.DHT_ENABLED or not Config.DHT_SENSOR_TYPE:
        logging.info("DHT: Module Adafruit_DHT not loaded or sensor type not specified. DHT thread will not start.")
        with data_lock:
            latest_data["dht_status"]["message"] = "DHT module/sensor not configured."
        return

    logging.info(f"DHT: Starting sensor reading on GPIO{Config.DHT_GPIO_PIN} every {Config.DHT_READ_INTERVAL}s.")
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(Config.DHT_SENSOR_TYPE, Config.DHT_GPIO_PIN)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        with data_lock:
            latest_data["dht_status"]["last_read_time"] = time.time()
            if humidity is not None and temperature is not None:
                latest_data["telemetry"]["temperature_dht"] = round(temperature, 1)
                latest_data["telemetry"]["humidity_dht"] = round(humidity, 1)
                latest_data["dht_status"]["message"] = f"DHT data read successfully at {timestamp}."
                # logging.debug(f"DHT: Temp={temperature:.1f}°C, Hum={humidity:.1f}%")
            else:
                latest_data["dht_status"]["message"] = f"Failed to read from DHT sensor at {timestamp}. Check connections."
                # Keep old values on read failure
                logging.warning(f"DHT: Failed to read sensor at {timestamp}.")
        time.sleep(Config.DHT_READ_INTERVAL)

# --- Flask Application ---
app = Flask(__name__)

@app.route('/')
def index():
    # More comprehensive HTML UI
    html_content = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Drone Control & Telemetry</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #f4f4f4; color: #333; }
            .container { width: 90%; max-width: 1200px; margin: 20px auto; background: #fff; padding: 20px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
            header { background-color: #333; color: #fff; padding: 10px 0; text-align: center; }
            nav button, .command-form button { background-color: #5cb85c; color: white; padding: 10px 15px; margin: 5px; border: none; cursor: pointer; border-radius: 5px; }
            nav button:hover, .command-form button:hover { background-color: #4cae4c; }
            .command-form input[type="number"], .command-form input[type="text"] { padding: 8px; margin: 5px; border: 1px solid #ddd; border-radius: 4px; }
            .telemetry-grid, .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-top: 20px; }
            .grid-item { background: #eee; padding: 15px; border-radius: 5px; }
            .grid-item h3 { margin-top: 0; }
            #messages { height: 150px; overflow-y: scroll; border: 1px solid #ccc; padding: 10px; background: #f9f9f9; margin-top: 10px; }
            .error { color: red; font-weight: bold; }
            .success { color: green; font-weight: bold; }
            .status-dot { height: 10px; width: 10px; border-radius: 50%; display: inline-block; margin-right: 5px; }
            .connected { background-color: green; }
            .disconnected { background-color: red; }
            .armed { color: orange; font-weight: bold; }
            .disarmed { color: green; }
            table { width: 100%; border-collapse: collapse; }
            th, td { text-align: left; padding: 8px; border-bottom: 1px solid #ddd; }
        </style>
    </head>
    <body>
        <header><h1>Drone Control Interface</h1></header>
        <div class="container">
            <h2>Connection Status</h2>
            <div class="status-grid">
                <div class="grid-item">
                    <h3>MAVLink: <span id="mavlink-conn-dot" class="status-dot disconnected"></span><span id="mavlink-status"></span></h3>
                    <p>Target: <span id="mavlink-target">N/A</span></p>
                    <p>Autopilot: <span id="autopilot-type">N/A</span></p>
                </div>
                 <div class="grid-item">
                    <h3>DHT Sensor: <span id="dht-status"></span></h3>
                    <p>Temp: <span id="dht-temp">N/A</span> °C, Humidity: <span id="dht-hum">N/A</span> %</p>
                </div>
            </div>

            <h2>Drone State</h2>
            <div class="status-grid">
                <div class="grid-item">Mode: <strong id="drone-mode">N/A</strong></div>
                <div class="grid-item">Armed: <strong id="drone-armed" class="disarmed">DISARMED</strong></div>
                <div class="grid-item">Landed State: <strong id="drone-landed">N/A</strong></div>
                <div class="grid-item">Battery: <span id="battery-voltage">N/A</span>V, <span id="battery-percent">N/A</span>%</div>
            </div>

            <h2>Commands</h2>
            <nav>
                <button onclick="sendCommand('arm')">Arm</button>
                <button onclick="sendCommand('disarm')">Disarm</button>
                <button onclick="sendCommand('takeoff')">Takeoff (<input type="number" id="takeoff-alt" value="5" style="width:50px;">m)</button>
                <button onclick="sendCommand('rtl')">Return to Launch</button>
                <button onclick="sendCommand('land')">Land</button>
            </nav>
            <div class="command-form" style="margin-top:10px;">
                <h3>Go to Location (Lat, Lon, Alt AMSL)</h3>
                <input type="number" step="any" id="goto-lat" placeholder="Latitude">
                <input type="number" step="any" id="goto-lon" placeholder="Longitude">
                <input type="number" step="any" id="goto-alt" placeholder="Altitude (AMSL)">
                <button onclick="sendCommand('goto')">Go</button>
            </div>
             <div class="command-form" style="margin-top:10px;">
                <h3>Set Mode</h3>
                <input type="text" id="set-mode-value" placeholder="e.g., GUIDED, LOITER">
                <button onclick="sendCommand('set_mode')">Set Mode</button>
            </div>
            <p>Last Command Status: <span id="command-feedback"></span></p>

            <h2>Telemetry</h2>
            <div class="telemetry-grid">
                <div class="grid-item">
                    <h3>Position</h3>
                    <p>Lat: <span id="lat">N/A</span>, Lon: <span id="lon">N/A</span></p>
                    <p>Alt (AMSL): <span id="alt-amsl">N/A</span> m</p>
                    <p>Alt (Rel): <span id="alt-rel">N/A</span> m</p>
                    <p>Heading: <span id="heading">N/A</span> °</p>
                </div>
                <div class="grid-item">
                    <h3>Speed & Movement</h3>
                    <p>Groundspeed: <span id="gnd-speed">N/A</span> m/s</p>
                    <p>Climb Rate: <span id="climb-rate">N/A</span> m/s</p>
                    <p>Velocity (X,Y,Z): <span id="vx">N/A</span>, <span id="vy">N/A</span>, <span id="vz">N/A</span> m/s</p>
                </div>
                <div class="grid-item">
                    <h3>Attitude</h3>
                    <p>Roll: <span id="roll">N/A</span> °</p>
                    <p>Pitch: <span id="pitch">N/A</span> °</p>
                    <p>Yaw: <span id="yaw">N/A</span> °</p>
                </div>
                <div class="grid-item">
                    <h3>GPS</h3>
                    <p>Fix Type: <span id="gps-fix">N/A</span></p>
                    <p>Satellites: <span id="gps-sats">N/A</span></p>
                    <p>HDOP (eph): <span id="gps-eph">N/A</span> m</p>
                    <p>VDOP (epv): <span id="gps-epv">N/A</span> m</p>
                </div>
                 <div class="grid-item">
                    <h3>System</h3>
                    <p>Throttle: <span id="throttle">N/A</span> %</p>
                    <p>Internal Temp: <span id="temp-internal">N/A</span> °C</p>
                    <p>Abs Pressure: <span id="pressure-abs">N/A</span> hPa</p>
                 </div>
            </div>

            <h2>Autopilot Messages</h2>
            <div id="messages"></div>
        </div>

        <script>
            function updateTelemetry() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        // Mavlink Status
                        const mavStatusEl = document.getElementById('mavlink-status');
                        const mavDotEl = document.getElementById('mavlink-conn-dot');
                        mavStatusEl.textContent = data.mavlink_status.message;
                        if (data.mavlink_status.connected) {
                            mavDotEl.classList.remove('disconnected');
                            mavDotEl.classList.add('connected');
                        } else {
                            mavDotEl.classList.remove('connected');
                            mavDotEl.classList.add('disconnected');
                        }
                        document.getElementById('mavlink-target').textContent = data.mavlink_status.target_system_id ? `Sys: ${data.mavlink_status.target_system_id}, Comp: ${data.mavlink_status.target_component_id}` : 'N/A';
                        document.getElementById('autopilot-type').textContent = data.mavlink_status.autopilot_type || 'N/A';

                        // DHT Status
                        document.getElementById('dht-status').textContent = data.dht_status.message;
                        document.getElementById('dht-temp').textContent = data.telemetry.temperature_dht !== null ? data.telemetry.temperature_dht.toFixed(1) : 'N/A';
                        document.getElementById('dht-hum').textContent = data.telemetry.humidity_dht !== null ? data.telemetry.humidity_dht.toFixed(1) : 'N/A';

                        // Drone State
                        document.getElementById('drone-mode').textContent = data.drone_state.mode || 'N/A';
                        const armedEl = document.getElementById('drone-armed');
                        armedEl.textContent = data.drone_state.armed ? 'ARMED' : 'DISARMED';
                        if(data.drone_state.armed) { armedEl.className = 'armed'; } else { armedEl.className = 'disarmed'; }
                        document.getElementById('drone-landed').textContent = data.drone_state.is_landed ? 'ON GROUND' : 'IN AIR';
                        document.getElementById('battery-voltage').textContent = data.telemetry.battery_voltage !== null ? data.telemetry.battery_voltage.toFixed(2) : 'N/A';
                        document.getElementById('battery-percent').textContent = data.telemetry.battery_remaining_percent !== null ? data.telemetry.battery_remaining_percent : 'N/A';


                        // Position
                        document.getElementById('lat').textContent = data.telemetry.latitude !== null ? data.telemetry.latitude.toFixed(7) : 'N/A';
                        document.getElementById('lon').textContent = data.telemetry.longitude !== null ? data.telemetry.longitude.toFixed(7) : 'N/A';
                        document.getElementById('alt-amsl').textContent = data.telemetry.altitude_amsl !== null ? data.telemetry.altitude_amsl.toFixed(2) : 'N/A';
                        document.getElementById('alt-rel').textContent = data.telemetry.altitude_relative !== null ? data.telemetry.altitude_relative.toFixed(2) : 'N/A';
                        document.getElementById('heading').textContent = data.telemetry.heading !== null ? data.telemetry.heading.toFixed(1) : 'N/A';

                        // Speed & Movement
                        document.getElementById('gnd-speed').textContent = data.telemetry.groundspeed !== null ? data.telemetry.groundspeed.toFixed(2) : 'N/A';
                        document.getElementById('climb-rate').textContent = data.telemetry.climb_rate !== null ? data.telemetry.climb_rate.toFixed(2) : 'N/A';
                        document.getElementById('vx').textContent = data.telemetry.vx !== null ? data.telemetry.vx.toFixed(2) : 'N/A';
                        document.getElementById('vy').textContent = data.telemetry.vy !== null ? data.telemetry.vy.toFixed(2) : 'N/A';
                        document.getElementById('vz').textContent = data.telemetry.vz !== null ? data.telemetry.vz.toFixed(2) : 'N/A';

                        // Attitude
                        document.getElementById('roll').textContent = data.telemetry.roll !== null ? data.telemetry.roll.toFixed(1) : 'N/A';
                        document.getElementById('pitch').textContent = data.telemetry.pitch !== null ? data.telemetry.pitch.toFixed(1) : 'N/A';
                        document.getElementById('yaw').textContent = data.telemetry.yaw !== null ? data.telemetry.yaw.toFixed(1) : 'N/A';

                        // GPS
                        const fixTypes = ['NO_GPS', 'NO_FIX', '2D_FIX', '3D_FIX', 'DGPS', 'RTK_FLOAT', 'RTK_FIXED'];
                        document.getElementById('gps-fix').textContent = data.telemetry.gps_fix_type !== null ? (fixTypes[data.telemetry.gps_fix_type] || data.telemetry.gps_fix_type) : 'N/A';
                        document.getElementById('gps-sats').textContent = data.telemetry.satellites_visible !== null ? data.telemetry.satellites_visible : 'N/A';
                        document.getElementById('gps-eph').textContent = data.telemetry.eph !== null ? data.telemetry.eph.toFixed(2) : 'N/A';
                        document.getElementById('gps-epv').textContent = data.telemetry.epv !== null ? data.telemetry.epv.toFixed(2) : 'N/A';

                        // System
                        document.getElementById('throttle').textContent = data.telemetry.throttle_percentage !== null ? data.telemetry.throttle_percentage : 'N/A';
                        document.getElementById('temp-internal').textContent = data.telemetry.temperature_internal !== null ? data.telemetry.temperature_internal.toFixed(1) : 'N/A';
                        document.getElementById('pressure-abs').textContent = data.telemetry.pressure_absolute !== null ? data.telemetry.pressure_absolute.toFixed(2) : 'N/A';


                        // Autopilot Messages
                        const messagesDiv = document.getElementById('messages');
                        // Only update if content has changed to avoid scroll jump
                        const newMessagesHtml = data.autopilot_messages.map(m => `<p>${m}</p>`).join('');
                        if (messagesDiv.innerHTML !== newMessagesHtml) {
                             const isScrolledToBottom = messagesDiv.scrollHeight - messagesDiv.clientHeight <= messagesDiv.scrollTop + 1;
                             messagesDiv.innerHTML = newMessagesHtml;
                             if(isScrolledToBottom) messagesDiv.scrollTop = messagesDiv.scrollHeight;
                        }


                        // Command Feedback (from last_command_ack)
                        const ack = data.last_command_ack;
                        const feedbackEl = document.getElementById('command-feedback');
                        if (ack && ack.timestamp) {
                            const timeAgo = Math.round((new Date().getTime()/1000 - ack.timestamp));
                            feedbackEl.textContent = `Cmd ${ack.command_id}: ${ack.result_text} (${timeAgo}s ago)`;
                            if (ack.result === 0) { // MAV_RESULT_ACCEPTED
                                feedbackEl.className = 'success';
                            } else if (ack.result === -1) { // Custom: Timeout
                                 feedbackEl.className = 'error';
                            }
                            else {
                                feedbackEl.className = 'error';
                            }
                        }


                    })
                    .catch(err => console.error("Error fetching telemetry:", err));
            }

            function sendCommand(command) {
                let payload = { command: command };
                if (command === 'takeoff') {
                    payload.altitude = parseFloat(document.getElementById('takeoff-alt').value);
                } else if (command === 'goto') {
                    payload.latitude = parseFloat(document.getElementById('goto-lat').value);
                    payload.longitude = parseFloat(document.getElementById('goto-lon').value);
                    payload.altitude = parseFloat(document.getElementById('goto-alt').value);
                     if (isNaN(payload.latitude) || isNaN(payload.longitude) || isNaN(payload.altitude)) {
                        alert("Invalid Lat/Lon/Alt for GoTo command.");
                        return;
                    }
                } else if (command === 'set_mode') {
                    payload.mode = document.getElementById('set-mode-value').value;
                    if (!payload.mode) {
                        alert("Mode cannot be empty.");
                        return;
                    }
                }


                fetch('/command', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                })
                .then(response => response.json())
                .then(data => {
                    console.log("Command response:", data);
                    const feedbackEl = document.getElementById('command-feedback');
                    feedbackEl.textContent = data.message;
                    if (data.status === 'success' || data.status === 'queued') {
                        feedbackEl.className = 'success';
                    } else {
                        feedbackEl.className = 'error';
                    }
                })
                .catch(err => {
                    console.error("Error sending command:", err);
                    document.getElementById('command-feedback').textContent = 'Error: ' + err;
                    document.getElementById('command-feedback').className = 'error';
                });
            }

            setInterval(updateTelemetry, 1000); // Update telemetry every second
            window.onload = updateTelemetry; // Initial load
        </script>
    </body>
    </html>
    """
    return render_template_string(html_content)

@app.route('/data')
def get_data_endpoint():
    with data_lock:
        # Create a deep copy to avoid issues if data is modified during serialization
        # For this simple dict, a shallow dict(latest_data) might be okay,
        # but deepcopy is safer for nested structures if they were complex objects.
        # However, current latest_data is all basic types or lists of basic types.
        return jsonify(dict(latest_data))


@app.route('/command', methods=['POST'])
def handle_command():
    global command_queue
    if not request.is_json:
        return jsonify({"status": "error", "message": "Invalid request: Not JSON"}), 400

    data = request.get_json()
    command_type = data.get('command')
    logging.info(f"Received command request: {command_type} with data: {data}")

    if not is_mavlink_connected() and command_type not in ['connect']: # Allow connect command if added
        return jsonify({"status": "error", "message": "MAVLink not connected. Command rejected."}), 503

    response_status = "queued"
    response_message = "Command queued for execution."

    cmd_task = None

    if command_type == 'arm':
        cmd_task = {'function': _internal_arm_disarm, 'args': (True,), 'kwargs': {}}
    elif command_type == 'disarm':
        cmd_task = {'function': _internal_arm_disarm, 'args': (False,), 'kwargs': {}}
    elif command_type == 'takeoff':
        alt = data.get('altitude', Config.DEFAULT_TAKEOFF_ALTITUDE)
        try:
            alt = float(alt)
            if alt <= 0: raise ValueError("Altitude must be positive.")
            cmd_task = {'function': _internal_takeoff, 'args': (alt,), 'kwargs': {}}
        except ValueError as e:
            response_status = "error"
            response_message = f"Invalid altitude: {e}"
    elif command_type == 'goto':
        try:
            lat = float(data.get('latitude'))
            lon = float(data.get('longitude'))
            alt_amsl = float(data.get('altitude')) # Assuming AMSL from UI
            cmd_task = {'function': _internal_goto_location, 'args': (lat, lon, alt_amsl), 'kwargs': {}}
        except (TypeError, ValueError) as e: # Handles missing keys or non-float values
            response_status = "error"
            response_message = f"Invalid parameters for goto: {e}"
    elif command_type == 'rtl':
        cmd_task = {'function': _internal_return_to_launch, 'args': (), 'kwargs': {}}
    elif command_type == 'land':
        cmd_task = {'function': _internal_land, 'args': (), 'kwargs': {}}
    elif command_type == 'set_mode':
        mode = data.get('mode')
        if mode and isinstance(mode, str) and len(mode) > 0:
            cmd_task = {'function': _internal_set_mode, 'args': (mode.upper(),), 'kwargs': {}}
        else:
            response_status = "error"
            response_message = "Invalid mode specified."
    else:
        response_status = "error"
        response_message = f"Unknown command: {command_type}"

    if cmd_task:
        if len(command_queue) < command_queue.maxlen:
            command_queue.append(cmd_task)
            logging.info(f"Command {command_type} added to queue.")
        else:
            response_status = "error"
            response_message = "Command queue is full. Please wait and try again."
            logging.warning("Command queue full. Command rejected.")
    elif response_status != "error": # If no task but also no error set yet
        response_status = "error"
        response_message = "Command recognized but no action defined or error in parameters."


    return jsonify({"status": response_status, "message": response_message})


if __name__ == '__main__':
    # Start MAVLink communication thread
    mavlink_thread = threading.Thread(target=mavlink_communication_thread_func, name="MAVLinkThread", daemon=True)
    mavlink_thread.start()

    # Start DHT sensor reader thread if enabled
    if Config.DHT_ENABLED:
        dht_thread = threading.Thread(target=dht_reader_thread_func, name="DHTThread", daemon=True)
        dht_thread.start()
    else:
        logging.info("DHT sensor is disabled by configuration or missing module.")
        with data_lock: # Ensure initial status is correct
            latest_data["dht_status"]["message"] = "DHT sensor disabled."


    logging.info(f"Flask server starting on http://{Config.FLASK_HOST}:{Config.FLASK_PORT}")
    app.run(host=Config.FLASK_HOST, port=Config.FLASK_PORT, debug=False) # debug=False for 'production-like'