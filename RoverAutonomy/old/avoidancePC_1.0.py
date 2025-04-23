from ultralytics import YOLO
import cv2
import pyzed.sl as sl
import math
import numpy as np
import sys
import math

MAX_U, MIN_U, MAX_V, MIN_V = 350, 100, 750, 530
 
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
  ## TODO: FINISH
  return 1

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
    exit()


  while True:

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