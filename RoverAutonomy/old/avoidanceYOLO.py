from ultralytics import YOLO
import cv2
import pyzed.sl as sl
import math
import numpy as np
import sys
import math
 
## OVERALL TODO:
## make global path with either magnetometer or accelerommeter readings

## TODO:
## Add temporary cv2 variable to store old frame and plot over current frame as "remembered" path if box is dropped

def main():
 
  model = YOLO("best.pt")
 
  zed = sl.Camera()

  # out = cv2.VideoWriter('demonstration.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (1280, 720))
  
  filepath = "./zed_recordings/recording3s.svo2" # Path to the .svo file to be playbacked
  input_type = sl.InputType()
  input_type.set_from_svo_file(filepath)  #Set init parameter to run from the .svo 
  # init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=False) # uncomment for .svo2 
  init_params = sl.InitParameters() # uncomment for actual camera
  init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 or HD1200 video mode, depending on camera type.
  init_params.camera_fps = 15  
  init_params.depth_mode = sl.DEPTH_MODE.ULTRA

  width, height = 1280, 720
  center_x, center_y = (width//2, height//2) # center of the frame
 
  point_cloud = sl.Mat()
 
  err = zed.open(init_params)
  if err != sl.ERROR_CODE.SUCCESS:
      print("Camera Open: " + repr(err) + ". Exit program.")
      exit()
 
  image = sl.Mat()
  runtime_parameters = sl.RuntimeParameters()
  while True:
    detection = False
    # initialize distance with big number
    distance = 99999

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
      # get image data as an opencv np array
      zed.retrieve_image(image, sl.VIEW.LEFT)
      cvimg = image.get_data()
      img = cv2.cvtColor(cvimg, cv2.COLOR_RGBA2RGB)

      # detect obstacles in frame with yolo
      # use image size = 1280 and confidence threshold = 60%
      results = model.predict(img, conf = .5, save = False)
 
      # determine if detected objects are in vehicle path
      for box in results[0].boxes:
       
        detection  = True
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        bcenter_x, bcenter_y = (x1 + x2) // 2, (y1 + y2) // 2
 
        if (abs(center_x - bcenter_x) < width * .15) and (abs(center_y - bcenter_y) < height * .15):
          # find distance from rover to obstacle
          zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
          # pointcloud_np = point_cloud.get_data() # point cloud as a np array
 
          ### determine np array size
          ### TODO: make depth take closest point <max(nparry.x_values)> instead of
          ### simply using the bounding box  center point of the depth map
          err, value = point_cloud.get_value(bcenter_x, bcenter_y-50) ## Currently using the depth value at a point at the center bottom of the bounding box
 
          if math.isfinite(value[2]):
            distance = math.sqrt(value[0]*value[0] + value[1]*value[1] + value[2]*value[2])
          else:
            continue

    isObstacle = (distance < 4000)

    if distance != 99999:
      cv2.putText(img, str(distance)[0:4] + " mm", (bcenter_x, bcenter_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv2.LINE_AA)

    ## plot path diverging left if within 4 meters, else plot straight line
    if isObstacle: # using 4000 mm, might have to compare to meters
      # annotate image with red box to show intersection with vehicle path
      cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

      theY = y2 + 20

      if min(bcenter_x, center_x) == bcenter_x: #go right around obstacle
        theX = max(x2+100, center_x)
        cv2.line(img, (center_x, height-1), (center_x, theY), (0, 165, 255), 5) 
        cv2.line(img, (center_x, theY), (theX, theY), (0, 165, 255), 5)
        cv2.line(img, (theX, theY), (theX, center_y+125), (0, 165, 255), 5)
      else: # go left around obstacle
        theX = min(x1-100, center_x)
        cv2.line(img, (center_x, height-1), (center_x, theY), (0, 165, 255), 5) 
        cv2.line(img, (center_x, theY), (x1-50, theY), (0, 165, 255), 5)
        cv2.line(img, (x1-50, theY), (x1-50, center_y+125), (0, 165, 255), 5)
    else:
      # annotate image with green for not obstacle
      if detection:
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

      cv2.line(img, (center_x, height-1), (center_x, center_y+125), (0, 165, 255), 5)


    cv2.imshow("Detection and Path", img)
    # out.write(img)

    if cv2.waitKey(20) == 27:
      break
    
  zed.close()
 
if __name__ == "__main__":
  main()