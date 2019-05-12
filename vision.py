import pyrealsense2 as rs
import numpy as np
import cv2

#Depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) #enable_stream(source, width, height, format, fps)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) #Intel resources say 1280 & 720 is best for the depth calculations, then you want to downsize it later)
pipeline.start(config)

threshold_low = (0, 0, 0)
threshold_high = (180, 255, 255)

debug = True

#In a dodgy try/catch so on exit (i.e. Ctrl-C) will still run the pipeline.close()
try:
    while True:
        #Wait for a coherent pair of depth & color frames
        #According to internet (implement later), should make sure takes picture from left camera
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue #(Yikes - at some point should probably time how long it takes to get frames, move it into it's own thread if it's IO heavy)

        #Convert images to numpy arrays and resize them
        depth_image = cv2.resize(np.asanyarray(depth_frame.get_data()), (640, 360), interpolation=cv2.INTER_NEAREST)
        color_image = cv2.resize(np.asanyarray(color_frame.get_data()), (640, 360), interpolation=cv2.INTER_NEAREST)
        
        #Process image:
        #Convert from RGB to HSV & threshold
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        threshold_image = cv2.inRange(hsv_image, threshold_values.get_low(), threshold_values.get_high())

        #Detect edges with canny
        canny_image = cv2.Canny(threshold_image, 100, 200)
        
        if debug:
            cv2.imshow("Raw input", color_image)
            cv2.imshow("Threshold", threshold_image)
            cv2.imshow("Canny image", canny_image)
            cv2.waitKey(1)
finally:
    pipeline.stop()
