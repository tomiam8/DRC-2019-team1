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

#Empty coefficients list for first run
coefficients = []

class Threshold_manager:
    def __init__(self, lh, ls, lv, uh, us, uv):
        self.threshold_low = (lh,ls,lv)
        self.threshold_high = (uh,us,uv)

    def get_low(self):
        return self.threshold_low

    def get_high(self):
        return self.threshold_high

class Threshold_manager_debug:
    def __init__(self):
        self.lh = 22
        self.uh = 68
        self.ls = 53
        self.us = 107
        self.lv = 114
        self.uv = 254

    def on_low_H_thresh_trackbar(self, val):
        self.lh = val
    def on_high_H_thresh_trackbar(self, val):
        self.uh = val
    def on_low_S_thresh_trackbar(self, val):
        self.ls = val
    def on_high_S_thresh_trackbar(self, val):
        self.us = val
    def on_low_V_thresh_trackbar(self, val):
        self.lv = val
    def on_high_V_thresh_trackbar(self, val):
        self.uv = val

    def get_low(self):
        return (self.lh, self.ls, self.lv)
    def get_high(self):
        return (self.uh, self.us, self.uv)

threshold_values = Threshold_manager(lh, ls, lv, uh, us, uv)
debug = True
if debug:
    import matplotlib.pyplot as plt
    threshold_values = Threshold_manager_debug()
    cv2.namedWindow("Threshold")

    cv2.createTrackbar("low hue", "Threshold", 0, 180, threshold_values.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold", 0, 180, threshold_values.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold", 0, 255, threshold_values.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold", 0, 255, threshold_values.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold", 0, 255, threshold_values.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold", 0, 255, threshold_values.on_high_V_thresh_trackbar)


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

        if debug:
            # create trackbars for Upper  and Lower HSV

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.imshow("Threshold", threshold_image)
            cv2.imshow("Raw input", color_image)
            fraction = int(len(color_image)/10)
            cv2.imshow("masked_image", color_image[fraction*5:fraction*6])
            for i in range(10):
                print(i)
                edges = cv2.Canny(color_image[fraction*i, fraction*(i+1)], 100, 200)
                (_, contours, _) = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                canny_output = cv2.drawContours(color_image[fraction*i:fraction*(i+1)], contours, -1, (255,255,0),10)
                cv2.imshow("{} section".format(i), canny_output)
            cv2.waitKey(1)
finally:
    cv2.imwrite("camera.jpg", color_image)
    pipeline.stop()
