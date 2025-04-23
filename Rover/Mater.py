#!/usr/bin/env python3

import math
import time
import os
from pymavlink import mavutil

# === Mission Item Class ===
class mission_item:
    def __init__(self, seq, current, x, y, z):
        self.seq = seq
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.0
        self.param3 = 20.0
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

# === Drone Control Functions ===
def arm(the_connection):
    print("-- Arming")
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    ack(the_connection, "COMMAND_ACK")

def disarm(the_connection):
    print("-- Disarming")
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack(the_connection, "COMMAND_ACK")
    while True:
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED == 0:
            print("-- Rover disarmed.")
            break
        print("-- Waiting for disarm...")
        time.sleep(1)

def set_mode(the_connection, mode_name):
    if mode_name not in the_connection.mode_mapping():
        print("Mode {} not found".format(mode_name))
        return
    mode_id = the_connection.mode_mapping()[mode_name]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print("-- Mode set to {}".format(mode_name))

def upload_mission(the_connection, mission_items):
    print("-- Sending Mission")
    the_connection.mav.mission_count_send(
        the_connection.target_system, the_connection.target_component,
        len(mission_items), 0
    )
    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("-- Creating waypoint {}".format(waypoint.seq))
        the_connection.mav.mission_item_send(
            the_connection.target_system, the_connection.target_component,
            waypoint.seq, waypoint.frame, waypoint.command, waypoint.current,
            waypoint.auto, waypoint.param1, waypoint.param2, waypoint.param3,
            waypoint.param4, waypoint.param5, waypoint.param6, waypoint.param7,
            waypoint.mission_type
        )

    ack(the_connection, "MISSION_ACK")


def set_return(the_connection):
    print("-- Set Return To Launch")
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack(the_connection, "COMMAND_ACK")

def start_mission(the_connection):
    print("-- Mission Start")
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack(the_connection, "COMMAND_ACK")

def ack(the_connection, keyword):
    print("-- Message Read = {}".format(str(
        the_connection.recv_match(type=keyword, blocking=True)
    )))

# === Main ===
if __name__ == "__main__":
    print("-- Program Started")
    the_connection = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)

    try:
        while the_connection.target_system == 0:
            print("-- Checking Heartbeat")
            the_connection.wait_heartbeat()
            print("-- Heartbeat from system {} component {}".format(
                the_connection.target_system, the_connection.target_component
            ))

        mission_waypoints = []
        mission_waypoints.append(mission_item(0, 0, 33.47402027300395, -88.79094159193298, 92))
        mission_waypoints.append(mission_item(0, 0, 33.47398111977065, -88.79058619923738, 92))

        upload_mission(the_connection, mission_waypoints)
        arm(the_connection)
        set_mode(the_connection, "AUTO")
        start_mission(the_connection)

        for mission_item in mission_waypoints:
            print("-- Waiting to reach waypoint {}".format(mission_item.seq))
            the_connection.recv_match(
                type="MISSION_ITEM_REACHED",
                condition="MISSION_ITEM_REACHED.seq == {}".format(mission_item.seq),
                blocking=True
            )
            print("-- Reached waypoint {}".format(mission_item.seq))

            print("-- Executing external script...")
            exit_code = os.system("python3 collect_data.py")
            while exit_code != 0:
                print("-- Script failed with exit code {}. Retrying...".format(exit_code))
                exit_code = os.system("sudo python3 /git/Quality-Quest/Data/Serial.py")

            print("-- Script at waypoint {} finished with code {}".format(mission_item.seq, exit_code))

        set_return(the_connection)
        disarm(the_connection)

    except KeyboardInterrupt:
        print("\n-- Keyboard interrupt received! Attempting to disarm...")
        disarm(the_connection)
        print("-- Disarmed and exiting safely.")
