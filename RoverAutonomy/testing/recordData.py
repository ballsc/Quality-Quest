########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import argparse 
import os 
import cv2


cam = sl.Camera()

image = sl.Mat()

# Handler to deal with CTRL+C properly
def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main():
    
    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.ULTRA # Set configuration parameters for the ZED
    init.camera_fps =  30  # Set fps at 30

    status = cam.open(init) 
    if status != sl.ERROR_CODE.SUCCESS: 
        print("Camera Open", status, "Exit program.")
        exit(1)
        
    recording_param = sl.RecordingParameters('./recording3s.svo', sl.SVO_COMPRESSION_MODE.H264) # Enable recording with the filename specified in argument
    err = cam.enable_recording(recording_param)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Recording ZED : ", err)
        exit(1)

    runtime = sl.RuntimeParameters()
    print("SVO is Recording, use Ctrl-C to stop.") # Start recording SVO, stop with Ctrl-C command
    frames_recorded = 0

    while frames_recorded < 1000:
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS : # Check that a new image is successfully acquired
            frames_recorded += 1
            print("Frame count: " + str(frames_recorded), end="\r")
            cam.retrieve_image(image, sl.VIEW.LEFT)
            cvimg = image.get_data()
            img = cv2.cvtColor(cvimg, cv2.COLOR_RGBA2RGB)
            data = sl.SVOData()
            data.key = "TEST"
            data.set_string_content("Hello, SVO World >> " + str(cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).data_ns))
            data.timestamp_ns = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE)
            print('INGEST', cam.ingest_data_into_svo(data))
            cv2.imshow("Detection and Path", img)

            if cv2.waitKey(20) == 27:
                break
    
if __name__ == "__main__":
    main()