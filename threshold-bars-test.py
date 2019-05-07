import pyrealsense2 as rs
import numpy as np
import cv2
import random

#HSV thresholding manager
class Threshold_value_manager:
    def __init__(self, window):
        self.window = window
        self.hue_low = 0 
        self.hue_high = 180 
        self.sat_low =0 
        self.sat_high =255 
        self.val_low =0 
        self.val_high =255 
    def on_low_H_thresh_trackbar(self, val):
        self.hue_low = val
        cv2.setTrackbarPos("low_H", "Threshold", self.hue_low)
    def on_high_H_thresh_trackbar(self, val):
        self.hue_high = val
        cv2.setTrackbarPos("high_H", "Threshold", self.hue_high)
    def on_low_S_thresh_trackbar(self, val):
        self.sat_low = val
    def on_high_S_thresh_trackbar(self, val):
        self.sat_high = val
    def on_low_V_thresh_trackbar(self, val):
        self.val_low = val
    def on_high_V_thresh_trackbar(self, val):
        self.val_high = val
    def get_low(self):
        return np.array([self.hue_low, self.sat_low, self.val_low])
    def get_high(self):
        return np.array([self.hue_high, self.sat_high, self.val_high])



"""Persistent vars"""
#Depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) #enable_stream(source, width, height, format, fps)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) #Intel resources say 1280 & 720 is best for the depth calculations, then you want to downsize it later)
pipeline.start(config)

cv2.namedWindow("Raw input")
cv2.namedWindow("Threshold")

threshold_values = Threshold_value_manager("Threshold")

#Trackbars
cv2.createTrackbar("low_H", "Threshold", 0, 180, threshold_values.on_low_H_thresh_trackbar)
cv2.createTrackbar("high_H", "Threshold", 0, 180, threshold_values.on_high_H_thresh_trackbar)
cv2.createTrackbar("low_S", "Threshold", 0, 255, threshold_values.on_low_S_thresh_trackbar)
cv2.createTrackbar("high_S", "Threshold", 0, 255, threshold_values.on_high_S_thresh_trackbar)
cv2.createTrackbar("low_V", "Threshold", 0, 255, threshold_values.on_low_V_thresh_trackbar)
cv2.createTrackbar("high_V", "Threshold", 0, 255, threshold_values.on_high_V_thresh_trackbar)

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
        #Convert from RGB to HSV (improves thresholding)
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        threshold_image = cv2.inRange(hsv_image, threshold_values.get_low(), threshold_values.get_high())
        #Remove noise by eroding, then dilating twice (to remove holes) - see https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
        #kernel = np.ones((5,5),np.uint8)
        #morph_image = cv2.erode(threshold_image, kernel, iterations=1)
        #morph_image = cv2.dilate(morph_image,kernel,iterations=1)
        #morph_image = cv2.dilate(morph_image,kernel,iterations=1)

        line_image = np.copy(color_image)
        lines = cv2.HoughLinesP(threshold_image, 1, np.pi/180, 100, 100, 10)
        #print("\n\n\nXXXXXXXXXXXXXXXXXXXXXXXXXX\n",lines,"\n\n\nXXXXXXXXXXXXXXXXXXXXXXXXXX\n")
        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(line_image, (l[0], l[1]), (l[2], l[3]), (random.randint(0,255),random.randint(0,255),random.randint(0,255)), 3, cv2.LINE_AA)

        #Detect edges with canny
        #canny_image = cv2.Canny(threshold_image, 100, 200)
        #canny_morph_image = cv2.Canny(morph_image, 100, 200)
        cv2.imshow("Raw input", color_image)
        cv2.imshow("Threshold", threshold_image)
        #cv2.imshow("Morph", morph_image)
        #cv2.imshow("Canny image", canny_image)
        cv2.imshow("Line image", line_image)
        #cv2.imshow("Morph canny image", canny_morph_image)
        cv2.waitKey(1)
finally:
    pipeline.stop()
