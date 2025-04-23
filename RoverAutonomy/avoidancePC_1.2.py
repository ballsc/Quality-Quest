# import cv2
import pyzed.sl as sl
import math
import numpy as np
import sys
import time
import rover_h as r

from pymavlink import mavutil

MAX_U, MIN_U, MAX_V, MIN_V = 350, 100, 750, 530
TURN_DELAY = 1  # seconds
RC_NEUTRAL = 1500
PORT, BAUDRATE = "/dev/ttyACM0", 115200

 # Connect to the CubePilot Rover
print("Connecting to CubePilot Rover...")
master = mavutil.mavlink_connection(PORT, baud=BAUDRATE)

# Wait for heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

def set_mode(mode):
  mode_id = master.mode_mapping()[mode]
  master.set_mode(mode_id)
  while True:
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if msg and mavutil.mode_string_v10(msg) == mode:
      print(f"Now in {mode} mode")
      break

def get_current_wp():
  master.mav.mission_request_list_send(master.target_system, master.target_component)
  while True:
    msg = master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=3)
    if msg:
      return msg.seq

def zedSetup():

  zed = sl.Camera()

  filepath = "./zed_recordings/recording3s.svo2" # Path to the .svo file to be playbacked
  input_type = sl.InputType()
  input_type.set_from_svo_file(filepath)  #Set init parameter to run from the .svo 
  # init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False) # uncomment for .svo2 
  init_params = sl.InitParameters() # uncomment for actual camera
  init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 or HD1200 video mode, depending on camera type.
  init_params.camera_fps = 15  
  init_params.depth_mode = sl.DEPTH_MODE.ULTRA

  return zed, init_params, sl.Mat(), sl.RuntimeParameters(), sl.Mat()

def avoidObstacle():
  print("")
  print("Avoiding obstacle")
  print("")

  set_mode("MANUAL")

  # turn right
  r.send_rc_command(master, RC_NEUTRAL + 350, RC_NEUTRAL)
  time.sleep(TURN_DELAY) # test how long to turn 90 deg, set TURN_DELAY to this

  # continue straight
  r.send_rc_command(master, RC_NEUTRAL, RC_NEUTRAL)
  time.sleep(.3)
  r.send_rc_command(master, RC_NEUTRAL, RC_NEUTRAL+200)
  time.sleep(2) #seconds

  # turn left
  r.send_rc_command(master, RC_NEUTRAL, RC_NEUTRAL)
  time.sleep(.3)
  r.send_rc_command(master, RC_NEUTRAL-350, RC_NEUTRAL)
  time.sleep(TURN_DELAY) # test how long to turn 90 deg, set TURN_DELAY to this

  r.send_rc_command(master, RC_NEUTRAL, RC_NEUTRAL)

  return 1

def main():
  try:

    r.arm_vehicle(master)

    # out = cv2.VideoWriter('demonstration.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (1280, 720))

    obstructed = unable = 0

    zed, init_params, image, runtime_parameters, zed_pointcloud = zedSetup()
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
      print("Camera open: " + repr(err) + ". Exit program.")
      r.disarm()
      return
    
    set_mode("AUTO")

    while True and not unable:
      #print(STATUS)
      ## TODO: make send status to flight controller

      if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        #cvimg = image.get_data()
        #img = cv2.cvtColor(cvimg, cv2.COLOR_RGBA2RGB)

        zed.retrieve_measure(zed_pointcloud, sl.MEASURE.XYZRGBA)
        np_pointcloud = zed_pointcloud.get_data()
        np_pointcloud = np_pointcloud[MIN_U:MAX_U, MIN_V:MAX_V, :3] # convert pointcloud to interest dimensions

        np_distance = np.sqrt(np.sum(np_pointcloud ** 2, axis = 2)) # create array of distances

        closePoint = np.nanmin(np_distance)

        pathObstructed = closePoint < 2000

        print("Distance to object: ", closePoint)

        unable = unable + 1 if closePoint < 500 else max(unable-1, 0)
        if unable > 20:
          print("Path obstructed")
          r.disarm(master)
          break

        #cv2.rectangle(img, (MIN_V, MIN_U), (MAX_V, MAX_U), (0, 0, 0), 2)

        obstructed = obstructed + 1 if pathObstructed else max(obstructed-1, -15)

        #cv2.putText(img, str(closePoint)[0:4] + " mm", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

        if obstructed > 15:
          current_wp = get_current_wp()
          avoidObstacle()

          master.mav.mission_set_current_send(master.target_system, master.target_component, current_wp)
          set_mode("AUTO")
          print(f"Resumed AUTO mode")

          obstructed = -15

      #cv2.imshow("Detection and Path", img)
      # out.write(img)

      #if cv2.waitKey(20) == 27:
      #  break
    

  except KeyboardInterrupt:
    zed.close()
    r.disarm(master)
 
if __name__ == "__main__":
  main()
