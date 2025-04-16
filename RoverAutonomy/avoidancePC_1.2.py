from pymavlink import mavutil
import cv2
import pyzed.sl as sl
import numpy as np
import time
import rover_h as r

MAX_U, MIN_U, MAX_V, MIN_V = 450, 200, 750, 530
TURN_DELAY = 1  # seconds
RC_NEUTRAL = 1500
PORT, BAUDRATE = "/dev/ttyACM0", 115200

master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)
master.wait_heartbeat()

# Mode switching utility
def set_mode(mode_str):
    mode_id = master.mode_mapping()[mode_str]
    master.set_mode(mode_id)
    while True:
        ack_msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if ack_msg and mavutil.mode_string_v10(ack_msg) == mode_str:
            print(f"Now in {mode_str} mode")
            break

# Get current waypoint index
def get_current_wp():
    master.mav.mission_request_list_send(master.target_system, master.target_component)
    while True:
        msg = master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=3)
        if msg:
            return msg.seq

# ZED setup
def zedSetup():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.camera_fps = 15  
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    return zed, init_params, sl.Mat(), sl.RuntimeParameters(), sl.Mat()

# Avoid obstacle sequence
def avoidObstacle():
    print("Avoiding obstacle")
    set_mode("GUIDED")

    # Turn right
    r.send_rc_command(master, RC_NEUTRAL + 200, RC_NEUTRAL)
    time.sleep(TURN_DELAY)

    # Move straight
    r.send_rc_command(master, RC_NEUTRAL, RC_NEUTRAL + 200)
    time.sleep(1)

    # Turn left
    r.send_rc_command(master, RC_NEUTRAL - 200, RC_NEUTRAL)
    time.sleep(TURN_DELAY)

    # Reset to neutral
    r.send_rc_command(master, RC_NEUTRAL, RC_NEUTRAL)

# Main function
def main():
    try:
        r.arm_vehicle(master)

        zed, init_params, image, runtime_parameters, zed_pointcloud = zedSetup()
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera open error.")
            return

        set_mode("AUTO")
        obstructed = unable = 0

        while True:
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_measure(zed_pointcloud, sl.MEASURE.XYZRGBA)
                np_pointcloud = zed_pointcloud.get_data()
                np_pointcloud = np_pointcloud[MIN_U:MAX_U, MIN_V:MAX_V, :3]

                np_distance = np.sqrt(np.sum(np_pointcloud ** 2, axis=2))
                closePoint = np.nanmin(np_distance)

                pathObstructed = closePoint < 2000

                unable = unable + 1 if closePoint < 300 else max(unable - 1, 0)

                if unable > 20:
                    print("Path completely obstructed")
                    r.disarm(master)
                    break

                obstructed = obstructed + 1 if pathObstructed else max(obstructed - 1, 0)

                if obstructed > 15:
                    current_wp = get_current_wp()
                    print(f"Obstacle detected at waypoint {current_wp}, starting avoidance.")
                    avoidObstacle()

                    # Resume AUTO at waypoint after avoidance
                    master.mav.mission_set_current_send(master.target_system, master.target_component, current_wp)
                    set_mode("AUTO")
                    print(f"Resumed AUTO mode at waypoint {current_wp}")

                    obstructed = -15

            if cv2.waitKey(20) == 27:
                break

    except KeyboardInterrupt:
        pass
    finally:
        zed.close()
        r.disarm(master)

if __name__ == "__main__":
    main()
