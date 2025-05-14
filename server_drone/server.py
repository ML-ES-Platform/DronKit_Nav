# streamlit_dht_dashboard_stabilized.py
import streamlit as st
import requests
import time
import pandas as pd
from streamlit_folium import st_folium
import folium
import os
import csv
from datetime import datetime

# --- Configurari Globale ---
DATA_URL_TEMPLATE = "http://{}:5000/data"
REFRESH_INTERVAL = 1.5  # SECUNDE - Am marit putin pentru stabilitate vizuala
LOG_FILENAME = "mavlink_dht_log.csv"

# --- Valori Default Globale ---
DEFAULT_VALUE_STR = "N/A"
DEFAULT_MAV_STATUS = "Așteptare date..."
DEFAULT_DHT_STATUS = "Așteptare date..."
DEFAULT_LOCATION = {"latitude": 47.0105, "longitude": 28.8638}

# --- Functii Utilitare ---
def fetch_data(url):
    try:
        response = requests.get(url, timeout=3.0)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.ConnectionError:
        return {"error": "ConnectionError", "message": "Nu s-a putut conecta la Raspberry Pi."}
    except requests.exceptions.Timeout:
        return {"error": "Timeout", "message": "Timeout la conectarea către Raspberry Pi."}
    except requests.exceptions.HTTPError as e:
        return {"error": "HTTPError", "message": f"Eroare HTTP: {e.response.status_code}"}
    except requests.exceptions.RequestException as e:
        return {"error": "RequestException", "message": f"Eroare la request: {str(e)}"}
    except ValueError:
        return {"error": "ValueError", "message": "Raspuns invalid de la server (nu este JSON valid)."}

def append_to_log_file(raw_pi_data):
    log_entry = {
        'client_timestamp': datetime.now().isoformat(),
        'altitude': raw_pi_data.get('altitude'),
        'latitude': raw_pi_data.get('latitude'),
        'longitude': raw_pi_data.get('longitude'),
        'battery_voltage': raw_pi_data.get('battery_voltage'),
        'groundspeed': raw_pi_data.get('groundspeed'),
        'temperature': raw_pi_data.get('temperature'),
        'humidity': raw_pi_data.get('humidity'),
        'mavlink_status_message': raw_pi_data.get('mavlink_status_message'),
        'dht_status_message': raw_pi_data.get('dht_status_message')
    }
    fieldnames = list(log_entry.keys())
    file_exists = os.path.isfile(LOG_FILENAME)
    try:
        with open(LOG_FILENAME, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists or f.tell() == 0:
                writer.writeheader()
            writer.writerow(log_entry)
    except IOError as e:
        st.toast(f"Eroare la scrierea in log: {e}", icon="⚠️")

# --- Initializare Session State ---
default_states = {
    'pi_ip': "",
    'running': False,
    'logging_active': False,
    'log_buffer': [],
    'last_meaningful_mav_message': DEFAULT_MAV_STATUS,
    'last_meaningful_dht_message': DEFAULT_DHT_STATUS,
    'last_known_location': DEFAULT_LOCATION.copy(),
    'last_data_package': {},
    # Pentru actualizari conditionate ale UI (evitarea flicker-ului)
    'last_displayed_mav_status_str': None,
    'last_displayed_dht_status_str': None,
    'last_displayed_error_str': None,
    'map_last_rendered_tuple_loc_running_error': (None, None, None, None) # (lat, lon, running_state, error_present)
}
for key, value in default_states.items():
    if key not in st.session_state:
        st.session_state[key] = value

# --- Interfata Streamlit ---
st.set_page_config(page_title="Drone Dashboard Pro", layout="wide", initial_sidebar_state="collapsed")
st.title("🚁 Drone Operations Dashboard Pro")

col_btn_start_stop, col_status_msg_area = st.columns([1, 3])
with col_btn_start_stop:
    if st.button("🚀 Start/Stop Auto-Refresh", 
                  type="primary" if not st.session_state.running else "secondary", 
                  use_container_width=True, 
                  disabled=not st.session_state.pi_ip):
        st.session_state.running = not st.session_state.running
        if st.session_state.running:
             st.session_state.log_buffer = []
        # Fortam actualizarea starii hartii la schimbarea starii de 'running'
        st.session_state.map_last_rendered_tuple_loc_running_error = (None, None, None, None) 
        st.rerun()

with col_status_msg_area:
    if st.session_state.running and st.session_state.pi_ip:
        st.success(f"Auto-refresh pornit! Se actualizează la {REFRESH_INTERVAL}s. IP: {st.session_state.pi_ip}")
    elif st.session_state.pi_ip:
        st.info(f"Auto-refresh oprit. IP configurat: {st.session_state.pi_ip}. Apăsați Start.")
    else:
        st.warning("Configurați adresa IP a Raspberry Pi în tab-ul '⚙️ Settings', apoi apăsați Start.")

status_mav_placeholder = st.empty()
status_dht_placeholder = st.empty()
error_placeholder = st.empty()

tab_data, tab_map, tab_logging, tab_settings = st.tabs([
    "📊 Real-Time Data", "🗺️ Live Map", "💾 Data Logging", "⚙️ Settings"
])

with tab_settings:
    st.header("⚙️ Configurații")
    st.markdown("Introduceți adresa IP a dispozitivului Raspberry Pi.")
    new_pi_ip = st.text_input("Adresa IP Raspberry Pi:", value=st.session_state.pi_ip, placeholder="Ex: 192.168.1.10", label_visibility="collapsed")
    if new_pi_ip != st.session_state.pi_ip:
        st.session_state.pi_ip = new_pi_ip
        st.session_state.running = False
        for key_to_reset in ['last_meaningful_mav_message', 'last_meaningful_dht_message', 
                             'last_known_location', 'last_data_package', 'log_buffer',
                             'last_displayed_mav_status_str', 'last_displayed_dht_status_str',
                             'last_displayed_error_str', 'map_last_rendered_tuple_loc_running_error']:
            if 'location' in key_to_reset:
                st.session_state[key_to_reset] = DEFAULT_LOCATION.copy()
            elif 'message' in key_to_reset: #mav or dht status
                 st.session_state[key_to_reset] = DEFAULT_MAV_STATUS if 'mav' in key_to_reset else DEFAULT_DHT_STATUS
            elif key_to_reset in ['log_buffer', 'last_data_package']:
                 st.session_state[key_to_reset] = [] if 'log_buffer' in key_to_reset else {}
            else: # For display strings or map tuple
                 st.session_state[key_to_reset] = None if 'displayed' in key_to_reset else (None,None,None,None)


        error_placeholder.empty() 
        st.rerun()
    st.markdown("---")
    st.markdown(f"**Interval de refresh date:** `{REFRESH_INTERVAL}` secundă/e.")
    st.markdown(f"**Fișier de log persistent:** `{LOG_FILENAME}`.")

with tab_data:
    st.header("📊 Real-Time Data")
    st.subheader("🛰️ Date MAVLink")
    mav_col1, mav_col2, mav_col3 = st.columns(3)
    alt_placeholder = mav_col1.empty()
    lat_placeholder = mav_col2.empty()
    lon_placeholder = mav_col3.empty()
    mav_col4, mav_col5 = st.columns(2)
    volt_placeholder = mav_col4.empty()
    gnd_speed_placeholder = mav_col5.empty()
    st.subheader("🌡️ Date Senzor DHT")
    dht_col1, dht_col2 = st.columns(2)
    temp_placeholder = dht_col1.empty()
    hum_placeholder = dht_col2.empty()

with tab_map:
    st.header("🗺️ Live Map")
    map_placeholder = st.empty()

with tab_logging:
    st.header("💾 Data Logging")
    col_log_btn, col_log_status = st.columns(2)
    with col_log_btn:
        if st.button("🎬 Start/Stop Logging Date", use_container_width=True):
            st.session_state.logging_active = not st.session_state.logging_active
            st.rerun()
    with col_log_status:
        if st.session_state.logging_active:
            st.success(f"Logging activ. Date în `{LOG_FILENAME}` și buffer sesiune.")
        else:
            st.info("Logging inactiv.")
    st.markdown("---")
    st.subheader("Date Înregistrate în Sesiunea Curentă de Monitorizare")
    log_buffer_df_placeholder = st.empty()
    if st.session_state.log_buffer:
        df_buffer = pd.DataFrame(st.session_state.log_buffer)
        if 'client_timestamp' in df_buffer.columns:
             try: df_buffer['client_timestamp'] = pd.to_datetime(df_buffer['client_timestamp']).dt.strftime('%Y-%m-%d %H:%M:%S')
             except: pass
        log_buffer_df_placeholder.dataframe(df_buffer.tail(), use_container_width=True, hide_index=True)
        csv_export = df_buffer.to_csv(index=False).encode('utf-8')
        st.download_button(label="📥 Descarcă Log Sesiune (CSV)", data=csv_export, file_name=f"drone_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", mime="text/csv", use_container_width=True)
    else:
        log_buffer_df_placeholder.info("Nicio dată în buffer pentru această sesiune.")

def update_all_elements():
    data_from_pi = st.session_state.last_data_package
    final_mav_status_for_display = st.session_state.last_meaningful_mav_message
    final_dht_status_for_display = st.session_state.last_meaningful_dht_message
    current_error_message_str = None # Mesajul de eroare curent pentru afisare

    if st.session_state.running and st.session_state.pi_ip:
        data_url = DATA_URL_TEMPLATE.format(st.session_state.pi_ip)
        data_from_pi = fetch_data(data_url)
        st.session_state.last_data_package = data_from_pi

        if "error" in data_from_pi:
            current_error_message_str = f"**Eroare de comunicare:** {data_from_pi.get('message', 'Eroare necunoscuta')}"
            final_mav_status_for_display = "Eroare comunicare cu Pi"
            final_dht_status_for_display = "Eroare comunicare cu Pi"
            st.session_state.last_meaningful_mav_message = DEFAULT_MAV_STATUS
            st.session_state.last_meaningful_dht_message = DEFAULT_DHT_STATUS
        else: # No error in data_from_pi
            # current_error_message_str ramane None, deci error_placeholder va fi golit
            pi_mav_message = data_from_pi.get('mavlink_status_message')
            if pi_mav_message and pi_mav_message != DEFAULT_MAV_STATUS:
                st.session_state.last_meaningful_mav_message = pi_mav_message
            final_mav_status_for_display = st.session_state.last_meaningful_mav_message
            
            pi_dht_message = data_from_pi.get('dht_status_message')
            if pi_dht_message and pi_dht_message != DEFAULT_DHT_STATUS:
                st.session_state.last_meaningful_dht_message = pi_dht_message
            final_dht_status_for_display = st.session_state.last_meaningful_dht_message

            lat = data_from_pi.get('latitude')
            lon = data_from_pi.get('longitude')
            if isinstance(lat, float) and isinstance(lon, float):
                st.session_state.last_known_location = {'latitude': lat, 'longitude': lon}

            if st.session_state.logging_active:
                append_to_log_file(data_from_pi)
                session_log_entry = {'client_timestamp': datetime.now().isoformat(), **{k: data_from_pi.get(k) for k in ['altitude','latitude','longitude','battery_voltage','groundspeed','temperature','humidity','mavlink_status_message','dht_status_message']}}
                st.session_state.log_buffer.append(session_log_entry)

    # --- Actualizare Conditionata Placeholders de Text ---
    # Error placeholder
    if st.session_state.last_displayed_error_str != current_error_message_str:
        if current_error_message_str:
            error_placeholder.error(current_error_message_str)
        else:
            error_placeholder.empty()
        st.session_state.last_displayed_error_str = current_error_message_str

    # MAV status
    new_mav_status_str = f"🛰️ Status MAVLink Pi: {final_mav_status_for_display}"
    if st.session_state.last_displayed_mav_status_str != new_mav_status_str:
        status_mav_placeholder.info(new_mav_status_str)
        st.session_state.last_displayed_mav_status_str = new_mav_status_str
    
    # DHT status
    new_dht_status_str = f"🌡️ Status DHT Pi: {final_dht_status_for_display}"
    if st.session_state.last_displayed_dht_status_str != new_dht_status_str:
        status_dht_placeholder.info(new_dht_status_str)
        st.session_state.last_displayed_dht_status_str = new_dht_status_str

    # --- Actualizare Metrici (se actualizeaza mereu, Streamlit e eficient pentru text simplu) ---
    current_data_for_metrics = st.session_state.last_data_package if not ("error" in st.session_state.last_data_package) else {}
    alt_placeholder.metric("Altitudine (AGL)", f"{current_data_for_metrics.get('altitude', DEFAULT_VALUE_STR):.2f} m" if isinstance(current_data_for_metrics.get('altitude'), float) else DEFAULT_VALUE_STR)
    lat_placeholder.metric("Latitudine", f"{current_data_for_metrics.get('latitude', DEFAULT_VALUE_STR):.6f}°" if isinstance(current_data_for_metrics.get('latitude'), float) else DEFAULT_VALUE_STR)
    lon_placeholder.metric("Longitudine", f"{current_data_for_metrics.get('longitude', DEFAULT_VALUE_STR):.6f}°" if isinstance(current_data_for_metrics.get('longitude'), float) else DEFAULT_VALUE_STR)
    volt_placeholder.metric("Tensiune Baterie", f"{current_data_for_metrics.get('battery_voltage', DEFAULT_VALUE_STR):.2f} V" if isinstance(current_data_for_metrics.get('battery_voltage'), float) else DEFAULT_VALUE_STR)
    gnd_speed_placeholder.metric("Viteză la Sol", f"{current_data_for_metrics.get('groundspeed', DEFAULT_VALUE_STR):.2f} m/s" if isinstance(current_data_for_metrics.get('groundspeed'), float) else DEFAULT_VALUE_STR)
    temp_placeholder.metric("Temperatură", f"{current_data_for_metrics.get('temperature', DEFAULT_VALUE_STR):.1f} °C" if isinstance(current_data_for_metrics.get('temperature'), float) else DEFAULT_VALUE_STR)
    hum_placeholder.metric("Umiditate", f"{current_data_for_metrics.get('humidity', DEFAULT_VALUE_STR):.1f} %" if isinstance(current_data_for_metrics.get('humidity'), float) else DEFAULT_VALUE_STR)

    # --- Actualizare Conditionata Harta ---
    loc_lat = st.session_state.last_known_location.get('latitude')
    loc_lon = st.session_state.last_known_location.get('longitude')
    is_error_present_for_map = "error" in st.session_state.last_data_package
    
    # Starea curenta care determina daca harta trebuie redesenata
    current_map_state_tuple = (loc_lat, loc_lon, st.session_state.running, is_error_present_for_map)

    if st.session_state.map_last_rendered_tuple_loc_running_error != current_map_state_tuple:
        with map_placeholder.container():
            map_key_suffix = "active" if st.session_state.running else "static"
            has_valid_pi_data_for_map = 'latitude' in current_data_for_metrics and 'longitude' in current_data_for_metrics

            if st.session_state.running and has_valid_pi_data_for_map and isinstance(loc_lat, float) and isinstance(loc_lon, float):
                map_view = folium.Map(location=[loc_lat, loc_lon], zoom_start=16, tiles="CartoDB positron")
                folium.Marker([loc_lat, loc_lon], popup=f"Drona\nLat: {loc_lat:.5f}\nLon: {loc_lon:.5f}\nAlt: {current_data_for_metrics.get('altitude', 'N/A')}m", tooltip="Locație Dronă Activă").add_to(map_view)
                st_folium(map_view, width=725, height=500, key=f"drone_map_{map_key_suffix}", returned_objects=[])
            elif not st.session_state.running and st.session_state.pi_ip:
                st.info(f"Harta este inactivă (Auto-Refresh Oprit). Ultima locație: Lat {loc_lat:.4f}, Lon {loc_lon:.4f}")
                map_view = folium.Map(location=[loc_lat, loc_lon], zoom_start=10, tiles="CartoDB Voyager")
                folium.Marker([loc_lat, loc_lon], popup="Ultima locație / Default", tooltip="Monitorizare Inactivă").add_to(map_view)
                st_folium(map_view, width=725, height=450, key=f"drone_map_{map_key_suffix}", returned_objects=[])
            elif st.session_state.running and not has_valid_pi_data_for_map and not is_error_present_for_map:
                 st.info("Așteptare date GPS valide de la dronă...")
            elif not st.session_state.pi_ip :
                st.info("Configurați IP-ul și porniți monitorizarea pentru a vedea harta.")
            elif is_error_present_for_map :
                 st.warning("Eroare comunicare. Harta poate afișa ultima locație validă.")
                 map_view = folium.Map(location=[loc_lat, loc_lon], zoom_start=10, tiles="CartoDB Voyager") # Afisam ultima locatie cunoscuta
                 folium.Marker([loc_lat, loc_lon], popup="Ultima locație înainte de eroare", tooltip="Eroare Comunicare").add_to(map_view)
                 st_folium(map_view, width=725, height=450, key=f"drone_map_error", returned_objects=[])
            else: # Fallback pentru stări neacoperite explicit
                st.info("Harta nu poate fi afișată momentan. Verificați configurația și statusul conexiunii.")
        st.session_state.map_last_rendered_tuple_loc_running_error = current_map_state_tuple


# --- Logica principala de executie ---
if st.session_state.running and st.session_state.pi_ip:
    update_all_elements()
    time.sleep(REFRESH_INTERVAL)
    st.rerun()
else:
    update_all_elements()
    if not st.session_state.pi_ip:
        # Curatam doar daca IP-ul nu mai e valid, altfel pastram ultimul status "lipicios"
        if st.session_state.last_displayed_mav_status_str is not None : status_mav_placeholder.empty(); st.session_state.last_displayed_mav_status_str = None
        if st.session_state.last_displayed_dht_status_str is not None : status_dht_placeholder.empty(); st.session_state.last_displayed_dht_status_str = None
        if st.session_state.last_displayed_error_str is not None : error_placeholder.empty(); st.session_state.last_displayed_error_str = None