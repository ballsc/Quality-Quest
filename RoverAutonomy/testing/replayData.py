import sys
import pyzed.sl as sl
import cv2
import argparse 
import os 

def main():
    filepath = "./zed_recordings/recording.svo2" # Path to the .svo file to be playbacked
    input_type = sl.InputType()
    input_type.set_from_svo_file(filepath)  #Set init parameter to run from the .svo 
    init = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
    init.depth_mode = sl.DEPTH_MODE.ULTRA #other depth mode is performance
    cam = sl.Camera()
    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS: #Ensure the camera opened succesfully 
        print("Camera Open", status, "Exit program.")
        exit(1)

    print('External SVOData channels:', cam.get_svo_data_keys())

    # Set a maximum resolution, for visualisation confort 
    resolution = cam.get_camera_information().camera_configuration.resolution
    print(resolution.width)
    print(resolution.height)
    low_resolution = sl.Resolution(min(720,resolution.width) * 2, min(404,resolution.height))
    svo_image = sl.Mat(min(720,resolution.width) * 2,min(404,resolution.height), sl.MAT_TYPE.U8_C4, sl.MEM.CPU)
    
    runtime = sl.RuntimeParameters()
    
    mat = sl.Mat()

    key = ' '
    print(" Press 'q' to exit...")

    svo_frame_rate = cam.get_init_parameters().camera_fps
    nb_frames = cam.get_svo_number_of_frames()
    print("[Info] SVO contains " ,nb_frames," frames")
    

    key = ''
    last_timestamp_ns = sl.Timestamp()
    while key != 113:  # for 'q' key
        err = cam.grab(runtime)

        data_map = {}
        # print("Reading between ", str(last_timestamp_ns.data_ns), " and ", str(cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).data_ns))
        ing = cam.retrieve_svo_data("TEST", data_map, last_timestamp_ns, cam.get_timestamp(sl.TIME_REFERENCE.IMAGE))
        for d in data_map:  
            s = data_map[d].get_content_as_string()
            print("Retrieved:", s)

        last_timestamp_ns = cam.get_timestamp(sl.TIME_REFERENCE.IMAGE)

        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            break

    cv2.destroyAllWindows()
    cam.close()


if __name__ == "__main__":
    main()