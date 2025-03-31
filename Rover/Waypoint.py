#!/usr/bin/env python3
from pymavlink import mavutil
import time

baud_rate = 115200
port = 'COM5'

master = mavutil.mavlink_connection(port, baud=baud_rate)
master.wait_heartbeat()

def request_message(message_id):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        message_id, 0, 0, 0, 0, 0, 0
    )

def get_firmware_version():
    print("Requesting firmware version...")
    request_message(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
    msg = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5)
    if msg:
        firmware_version = msg.flight_sw_version
        major = (firmware_version >> 24) & 0xFF
        minor = (firmware_version >> 16) & 0xFF
        patch = (firmware_version >> 8) & 0xFF
        print(f"Firmware Version: {major}.{minor}.{patch}")
    else:
        print("Failed to retrieve firmware version.")

def get_rc_channels():
    msg = master.recv_match(type="RC_CHANNELS_RAW", blocking=True, timeout=5)
    if msg:
        print(f"RC Input - CH1: {msg.chan1_raw}, CH2: {msg.chan2_raw}, CH3: {msg.chan3_raw}, CH4: {msg.chan4_raw}")
    else:
        print("RC input request failed.")

def get_system_status():
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if msg:
        mode = msg.base_mode
        armed_status = "Armed" if mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "Disarmed"
        print(f"System Status: {armed_status}")
    else:
        print("System status request failed.")

def is_armed():
    msg = master.recv_match(type="HEARTBEAT", blocking=True)
    return msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

def arm_vehicle():
    print("Arming rover...")
    master.arducopter_arm()
    time.sleep(1)
    while not is_armed():
        print("Waiting for arming...")
        time.sleep(1)
    print("Rover armed!")

def disarm_vehicle():
    print("Disarming rover...")
    master.arducopter_disarm()
    time.sleep(1)
    while is_armed():
        print("Waiting for disarm...")
        time.sleep(1)
    print("Rover disarmed.")

def set_mode(mode_name):
    if mode_name not in master.mode_mapping():
        print(f"Mode {mode_name} not found")
        return
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

def send_waypoint(seq, lat, lon):
    master.mav.mission_item_send(
        master.target_system,
        master.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1, 0, 0, 0, 0,
        lat, lon, 0
    )

def upload_mission():
    waypoints = [(33.47379904133127, -88.79123624264602), (33.47385012053299, -88.7914942079917)]
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    for i, (lat, lon) in enumerate(waypoints):
        send_waypoint(i, lat, lon)

def set_home_location():
    print("Fetching current location to set as home...")
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, condition='fix_type>=3')
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            0,
            0, 0, 0,
            lat, lon, 0  # Altitude set to 0 since it's not needed
        )
        print(f"Home location set to current coordinates: Latitude = {lat}, Longitude = {lon}")
    else:
        print("Unable to fetch a valid GPS fix.")

set_home_location()

get_firmware_version()
get_rc_channels()
get_system_status()

upload_mission()

arm_vehicle()

set_mode("AUTO")

master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_MISSION_START,
    0, 0, 0, 0, 0, 0, 0, 0
)

print("Mission Started!")

time.sleep(300)  # Wait for 10 minutes

set_mode("RTL")
print("Returning to home waypoint")

time.sleep(300)  # Wait for 5 minutes

disarm_vehicle()
