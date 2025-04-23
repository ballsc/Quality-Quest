#!/usr/bin/env python3
import time
import threading
import serial
import csv
import os
from datetime import datetime

import numpy as np
import pyzed.sl as sl
from pymavlink import mavutil

import rover_h as r  

# === Configuration ===
VEHICLE_PORT       = '/dev/ttyACM0'   # MAVLink port
#WATER_SENSOR_PORT  = '/dev/ttyACM0'   # Serial port for water sensor
BAUD_RATE          = 115200

# ZED camera ROI & timing
MIN_U, MAX_U = 200, 450
MIN_V, MAX_V = 530, 750
TURN_DELAY    = 1  # seconds to turn ~90°

# === Vehicle & Sensor Setup ===
master = mavutil.mavlink_connection(VEHICLE_PORT, baud=BAUD_RATE)
master.wait_heartbeat()
print(f"[MAV] Connected to system {master.target_system}, component {master.target_component}")

#water_ser = serial.Serial(WATER_SENSOR_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # allow serial to warm up

# === MAVLink Utility Functions (from Waypoint.py) :contentReference[oaicite:10]{index=10}&#8203;:contentReference[oaicite:11]{index=11} ===
def set_mode(mode_name):
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        print(f"[MODE] {mode_name} not available")
        return
    mode_id = mapping[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"[MODE] switching to {mode_name}")
    # wait until mode is actually set
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb and mavutil.mode_string_v10(hb) == mode_name:
            print(f"[MODE] now in {mode_name}")
            break

def get_current_wp():
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    msg = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
    return msg.seq if msg else None

def upload_mission(waypoints):
    """
    waypoints: list of (lat, lon)
    """
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    for i,(lat,lon) in enumerate(waypoints):
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0,0,0,0,
            lat, lon, 0
        )
    print(f"[MISSION] Uploaded {len(waypoints)} waypoints")

"""
def run_sensor_test(duration=60, post_delay=120):
    try:
        ser = serial.Serial(WATER_SENSOR_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        fname = f"sensor_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with open(fname,'w',newline='') as f:
            w = csv.writer(f)
            w.writerow(['Timestamp','pH','TDS'])
            ser.write(b'START\n')
            start = time.time()
            while time.time() - start < duration:
                line = ser.readline().decode().strip()
                if line:
                    parts = line.split(',')
                    if len(parts)==2:
                        now = datetime.now().isoformat()
                        print(f"[SENSOR] {now} pH={parts[0]} TDS={parts[1]}")
                        w.writerow([now,*parts])
            ser.write(b'STOP\n')
        print(f"[SENSOR] logged to {fname}, waiting {post_delay}s to empty")
        time.sleep(post_delay)
    except Exception as e:
        print(f"[SENSOR] error: {e}")
    finally:
        ser.close()
"""
def wait_for_waypoint_reached_and_loiter(seq):
    print(f"[MISSION] waiting for WP {seq}")
    while True:
        msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True, timeout=60)
        if msg and msg.seq == seq:
            print(f"[MISSION] reached WP {seq}")
            set_mode("LOITER")
            # Run external script and check exit code
            exit_code = os.system("sudo python3 /git/Quality-Quest/Rover/Data/Serial.py")
            if exit_code == 0:
                print("[MISSION] external script succeeded, switching back to AUTO")
                set_mode("AUTO")
            else:
                print(f"[MISSION] external script failed with exit code {exit_code}, remaining in LOITER")
            break

def set_home_location():
    print("[HOME] waiting for a valid GPS fix…")
    # loop until we actually see fix_type >= 3
    while True:
        msg = master.recv_match(
            type='GPS_RAW_INT',
            blocking=True,
            timeout=1
        )
        if msg and msg.fix_type >= 3:
            break

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1e3     # convert mm → m

    print(f"[HOME] GPS fix acquired at {lat:.7f}, {lon:.7f}, {alt:.1f} m")
    # set home to *exactly* here
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,      # confirmation
        1, 0,0,0,  # param1=1 → “use current location”
        0, 0, 0   # (ignored)
    )

    # check ACK
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("[HOME] home position successfully set")
        else:
            print(f"[HOME] set-home rejected: result={ack.result}")
    else:
        print("[HOME] no ACK received for set-home")


is_in_water = False
def monitor_water_sensor():
    global is_in_water
    print("[WATER] sensor monitor started")
    while True:
        try:
            line = water_ser.readline().decode().strip()
            if line=="WATER" and not is_in_water:
                is_in_water = True
                print("[WATER] entering water mode")
                # override channels for water (ch1=1500,ch3=1600,ch4=1600)
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1500,0,1500,1600,0,1600,0,0,0,0
                )
            elif line=="LAND" and is_in_water:
                is_in_water = False
                print("[WATER] returning to land mode")
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    1600,0,1600,1500,0,1500,0,0,0,0
                )
        except Exception as e:
            print(f"[WATER] error: {e}")


def zed_setup():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.AUTO
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.ULTRA
    return zed, init, sl.Mat(), sl.RuntimeParameters(), sl.Mat()

def avoid_obstacle():
    print("[AVOID] obstacle detected, taking over")
    set_mode("MANUAL")
    # smooth 90° right
    r.send_rc_command(master, 1850,1500)  
    time.sleep(TURN_DELAY)
    # forward-right
    r.send_rc_command(master,1500,1500)
    time.sleep(2)
    # smooth 90° left
    r.send_rc_command(master,1500,1500)
    time.sleep(.3)
    r.send_rc_command(master,1150,1500)
    time.sleep(TURN_DELAY)
    # forward
    r.send_rc_command(master,1500,1500)
    # resume AUTO at current WP
    wp = get_current_wp()
    if wp is not None:
        print(f"[AVOID] resuming AUTO at WP {wp}")
        master.mav.mission_set_current_send(master.target_system, master.target_component, wp)
    set_mode("AUTO")

def monitor_obstacle_thread():
    try:
        zed, init, img, params, cloud = zed_setup()
        if zed.open(init) != sl.ERROR_CODE.SUCCESS:
            print("[AVOID] ZED open error"); return
        print("[AVOID] ZED camera ready")
        set_mode("AUTO")
        obstructed = 0
        while True:
            if zed.grab(params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_measure(cloud, sl.MEASURE.XYZRGBA)
                pc = cloud.get_data()[MIN_U:MAX_U, MIN_V:MAX_V,:3]
                dists = np.sqrt((pc**2).sum(axis=2))
                closest = np.nanmin(dists)
                if closest < 500:
                    obstructed = obstructed + 1
                else:
                    obstructed = max(obstructed-1, 0)
                if obstructed > 15:
                    avoid_obstacle()
                    obstructed = 0
    except Exception as e:
        print(f"[AVOID] error: {e}")
    finally:
        zed.close()

# === Main Mission Flow ===
if __name__ == "__main__":
    try:
    # 1) Pre‐flight
        r.arm_vehicle(master)
        set_home_location()
        r.disarm(master)
    # Optional debug info
    # get_firmware_version(); get_rc_channels(); get_system_status()
    # 2) Start background monitors
    #    threading.Thread(target=monitor_water_sensor, daemon=True).start()
    #    threading.Thread(target=monitor_obstacle_thread, daemon=True).start()

    # 3) Upload & launch mission
        waypoints = [
            (33.47401138270369, -88.79087958741471),
            (33.473931657832495, -88.79028720462831)
        ]
        upload_mission(waypoints)
        r.arm_vehicle(master)
        set_mode("AUTO")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,0,0,0,0,0,0,0
        )
        print("[MISSION] started")

        # 4) At each waypoint loiter & log sensors
        for seq in range(len(waypoints)):
            wait_for_waypoint_reached_and_loiter(seq)

        # 5) Return & disarm
        set_mode("RTL")
        print("[MISSION] returning home, waiting 5min")
        time.sleep(300)
        r.disarm(master)
        print("[MISSION] complete. Goodbye.")

    except KeyboardInterrupt:
        # zed.close()
        r.disarm(master)
