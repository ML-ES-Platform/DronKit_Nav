import time
try:
    from pymavlink import mavutil
    import serial
except ImportError as e:
    print(f"Eroare importare module: {e}")
    print("Rulati 'pip install pyserial pymavlink' pentru a instala modulele necesare")
    exit(1)

# Connection string (asigurati-va ca il adaptati)
connection_string = '/dev/serial0'  # Serial
#connection_string = 'udp:127.0.0.1:14550'  # UDP

try:
    print(f"Incerc conectarea la {connection_string}...")
    # Initializam conexiunea
    mav = mavutil.mavlink_connection(connection_string, baud=921600)

    # Asteptam un heartbeat
    print("Astept heartbeat...")
    mav.wait_heartbeat()
    print("Heartbeat primit!")

    # Setam intervalele pentru toate mesajele dorite (in microsecunde)
    message_types = [
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT"),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, "SYS_STATUS"),  # Changed from SYSTEM_STATUS to SYS_STATUS
        (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, "VFR_HUD")
    ]

    for msg_id, msg_name in message_types:
        print(f"Solicit date pentru {msg_name}...")
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            1000000,  # 1 Hz (in microsecunde)
            0, 0, 0, 0, 0
        )
        # Asteptam confirmarea comenzii
        mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    
    print("Incep receptia datelor...")
    while True:
        # Asteptam si procesam mesajele primite
        msg = mav.recv_match(type=['GLOBAL_POSITION_INT', 'SYS_STATUS', 'VFR_HUD'], blocking=True, timeout=1)
        if msg is not None:
            msg_type = msg.get_type()
            print(f"\nMesaj primit: {msg_type}")
            
            if msg_type == 'GLOBAL_POSITION_INT':
                print(f"Altitudine: {msg.alt / 1000.0}m")
                print(f"Latitudine: {msg.lat / 1.0e7}")
                print(f"Longitudine: {msg.lon / 1.0e7}")
            elif msg_type == 'SYS_STATUS':  # Changed from SYSTEM_STATUS to SYS_STATUS
                print(f"Baterie: {msg.voltage_battery / 1000.0}V")
            elif msg_type == 'VFR_HUD':
                print(f"Viteza la sol: {msg.groundspeed}m/s")

        time.sleep(1)

except serial.SerialException as e:
    print(f"Eroare de conexiune seriala: {e}")
    print("Verificati ca:")
    print("1. Aveti permisiuni de acces la portul serial")
    print("2. Portul serial specificat este corect")
    print("3. Niciun alt program nu foloseste portul serial")
    exit(1)
except Exception as e:
    print(f"Eroare: {e}")

finally:
    if 'mav' in locals():
        mav.close()