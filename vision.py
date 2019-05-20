import pyrealsense2 as rs
import numpy as np
import cv2

#Depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) #enable_stream(source, width, height, format, fps)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) #Intel resources say 1280 & 720 is best for the depth calculations, then you want to downsize it later)
pipeline.start(config)

# Initial Lower HSV threshold values(To be tuned later)
lh = 0
ls = 0
lv = 0

# Initial Upper HSV threshold values (to be tunes later)
uh = 180
us = 255
uv = 255


threshold_low = (lh, ld, lv)
threshold_high = (uh, us, uv)

debug = True
if debug:
    import matplotlib.pyplot as plt

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

        #Polynomial stuff
        non_zero_pixels = np.nonzero(threshold_image)
        x_values = non_zero_pixels[0]
        y_values = non_zero_pixels[1]
        coefficients = np.polyfit(y_values, x_values, 2) #Gives the coefficients in order highest-power to lowest power (i.e. coefficients[0] = a for ax^2 + bx + c)
        #Is rotated (i.e. domain is y-value of image, codomain x-value of image) as works better (think related to being more function-y) (also more in-line with how will be used - e.g. x-value for bottom of image gives where the line is now, for half way down the image gives where line is in say half a second)
        
        #Detect edges with canny
        #canny_image = cv2.Canny(threshold_image, 100, 200)

        if debug:
            # create trackbars for Upper  and Lower HSV
            cv2.createTrackbar('UpperH',window_name,0,255,nothing)
            cv2.setTrackbarPos('UpperH',window_name, uh)

            cv2.createTrackbar('LowerH',window_name,0,255,nothing)
            cv2.setTrackbarPos('LowerH',window_name, lh)

            cv2.createTrackbar('UpperS',window_name,0,255,nothing)
            cv2.setTrackbarPos('UpperS',window_name, us)

            cv2.createTrackbar('LowerS',window_name,0,255,nothing)
            cv2.setTrackbarPos('LowerS',window_name, ls)

            cv2.createTrackbar('UpperV',window_name,0,255,nothing)
            cv2.setTrackbarPos('UpperV',window_name, uv)

            cv2.createTrackbar('LowerV',window_name,0,255,nothing)
            cv2.setTrackbarPos('LowerV',window_name, lv)

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.imshow("Raw input", color_image)
            cv2.imshow("Canny image", canny_image)
            plt.imshow(threshold_image)
            #Plot a bunch of points to graph
            x_values = list(range(1, len(threshold_image)))
            y_values = []
            for x in x_values:
                temp_sum = 0
                for coefficient_index in range(len(coefficients)):
                    temp_sum += list(coefficients)[coefficient_index]*(x**(len(coefficients)-i-1))
                y_values.append(temp_sum)
            plt.plot(y_values, x_values)
            plt.show()
            cv2.waitKey(1)
finally:
    pipeline.stop()
