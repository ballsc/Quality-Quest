from ultralytics import YOLO
import cv2
import pyzed.sl as sl
import math
import numpy as np
import sys
import math

MAX_U = 450 ## between 0 and 719
MIN_U = 160
MAX_V = 750 ## between 0 and 1279
MIN_V = 530

STATUS = 0
 
## OVERALL TODO:
## Calibrate Magnetometer
## Make heading function using magnetometer for use in avoidObstacle
## Finish avoidObstacle with dronekit and pymavlink functions

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
  print("avoiding obstacle")
  ## TODO: FINISH, Check with COOPER if code is syntax with his

  # # Switch to GUIDED mode
  # master.mav.set_mode_send(
  #     master.target_system,
  #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
  #     15  # GUIDED mode for ArduRover typically mode ID 15
  # )

  # # Confirm mode change
  # ack = False
  # while not ack:
  #     ack_msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
  #     if ack_msg and mavutil.mode_string_v10(ack_msg) == "GUIDED":
  #         ack = True
  # print("Now in GUIDED mode")

  # master.mav.rc_channels_override_send(
  #   master.target_system,         # target_system (usually 1)
  #   master.target_component,      # target_component (usually 1)
  #   1100,            # Channel 1: Steering (Left) * 1100 left, 1500 typical, 1900 right
  #   1500,                 # Channel 2: Throttle 1100 low, 1500 typical, 1900 high
  #   0, 0, 0, 0, 0, 0              # Channels 3-8 unused (set 0 to ignore)
  # )

  # delay(TURN_DELAY) # test how long to turn 90 deg, set TURN_DELAY to this

  # master.mav.rc_channels_override_send(
  #   master.target_system,         # target_system (usually 1)
  #   master.target_component,      # target_component (usually 1)
  #   1900,            # Channel 1: Steering (Left) * 1100 left, 1500 typical, 1900 right
  #   1500,                 # Channel 2: Throttle 1100 low, 1500 typical, 1900 high
  #   0, 0, 0, 0, 0, 0              # Channels 3-8 unused (set 0 to ignore)
  # )

  # delay(TURN_DELAY) # test how long to turn 90 deg, set TURN_DELAY to this

  # master.mav.rc_channels_override_send(
  #   master.target_system,         # target_system (usually 1)
  #   master.target_component,      # target_component (usually 1)
  #   1500,            # Channel 1: Steering (Left) * 1100 left, 1500 typical, 1900 right
  #   1500,                 # Channel 2: Throttle 1100 low, 1500 typical, 1900 high
  #   0, 0, 0, 0, 0, 0              # Channels 3-8 unused (set 0 to ignore)
  # )

  # Switch back to AUTO mode
  # master.mav.set_mode_send(
  #     master.target_system,
  #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
  #     10  # AUTO mode for ArduRover typically mode ID 10
  # )

  return 1

def unavaliable():
  while(1):
    STATUS = 0

def currHeading():
  # N NE E SE S SW W NW
  heading = "N"
  ## TODO: FINISH with magenetometer

  return heading

def main():

  # out = cv2.VideoWriter('demonstration.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (1280, 720))

  width, height = 1280, 720
  center_x, center_y = (width//2, height//2) # center of the frame
  obstructed = 0

  zed, init_params, image, runtime_parameters, zed_pointcloud = zedSetup()
  err = zed.open(init_params)
  if err != sl.ERROR_CODE.SUCCESS:
    print("Camera open: " + repr(err) + ". Exit program.")
    unavaliable()

  STATUS = 1

  while True:
    print(STATUS)
    ## TODO: make send status to flight controller

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
      zed.retrieve_image(image, sl.VIEW.LEFT)
      cvimg = image.get_data()
      img = cv2.cvtColor(cvimg, cv2.COLOR_RGBA2RGB)

      zed.retrieve_measure(zed_pointcloud, sl.MEASURE.XYZRGBA)
      np_pointcloud = zed_pointcloud.get_data()
      np_pointcloud = np_pointcloud[MIN_U:MAX_U, MIN_V:MAX_V, :3] # convert pointcloud to interest dimensions

      np_distance = np.sqrt(np.sum(np_pointcloud ** 2, axis = 2)) # create array of distances

      closePoint = np.nanmin(np_distance)

      pathObstructed = closePoint < 2000

      cv2.rectangle(img, (MIN_V, MIN_U), (MAX_V, MAX_U), (0, 0, 0), 2)

      if pathObstructed:
        color = (0, 0, 255)
        obstructed += 1
      else:
        color = (255, 0, 0)
        obstructed -= 1

      cv2.putText(img, str(closePoint)[0:4] + " mm", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

      if obstructed > 30:
        avoidObstacle()
        obstructed = -15

    cv2.imshow("Detection and Path", img)
    # out.write(img)

    if cv2.waitKey(20) == 27:
      break
    
  zed.close()
 
if __name__ == "__main__":
  main()