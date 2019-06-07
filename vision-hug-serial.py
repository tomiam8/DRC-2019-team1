import numpy as np
import cv2
import serial
import time, math, sys, threading
import pyrealsense2 as rs

from cam import setupstream
from cam import getframes

################ Setup constants ################
#Thresholds: Yellow
thresh_yellow_low = (18,20,158)
thresh_yellow_high = (40,158,254)

# debug mode
debug = 0

bin_nums = 10
fraction = int(720/bin_nums)
midpoints = []
width = 320
height = 180
center_fixed = width
angle_constant = 0.01
speed_constant = 100

class Threshold_manager:
    def __init__(self, low, high):
        self.threshold_low = low
        self.threshold_high = high

    def get_low(self):
        return self.threshold_low

    def get_high(self):
        return self.threshold_high

class Threshold_manager_debug:
    def __init__(self, low_default=(0,0,0), high_default=(180,255,255)):
        self.lh = low_default[0]
        self.uh = high_default[0]
        self.ls = low_default[1]
        self.us = high_default[1]
        self.lv = low_default[2]
        self.uv = high_default[2]

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

class Arduino:
    def __init__(self):
        self.connection = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.speed = 0
        self.angle = 0

    def update_speed(self, speed):
        self.speed = speed

    def update_angle(self, angle):
        self.angle = angle

    def get_speed(self):
        return self.speed

    def get_angle(self):
        return self.angle

    def run(self):
        while True:
            s.write(b"D000")
            time.sleep(0.04)
            s.write(f"M{self.speed:03d}".encode())
            time.sleep(0.04)
            s.write(f"S{self.angle:03d}".encode())
            time.sleep(0.04)

random_colours = [(255,0,0),(255,255,0),(0,255,),(0,255,255),(0,0,255),(255,0,255)]

threshold_yellow = Threshold_manager(thresh_yellow_low, thresh_yellow_high)

#Setup debug stuff (i.e run headerless if not debug)
if len(sys.argv) > 1:
    debug = True
else:
    debug = False
if debug:
    threshold_yellow = Threshold_manager_debug(thresh_yellow_low, thresh_yellow_high)

    cv2.namedWindow("Threshold yellow")

    cv2.createTrackbar("low hue", "Threshold yellow", threshold_yellow.lh, 180, threshold_yellow.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold yellow", threshold_yellow.uh, 180, threshold_yellow.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold yellow", threshold_yellow.ls, 255, threshold_yellow.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold yellow", threshold_yellow.us, 255, threshold_yellow.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold yellow", threshold_yellow.lv, 255, threshold_yellow.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold yellow", threshold_yellow.uv, 255, threshold_yellow.on_high_V_thresh_trackbar)

# select the region of interest for the detected edges
def roi(image, polygons):
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked = cv2.bitwise_and(image, mask)
    return masked


# display the lines on the screen
def display_line(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (10, 100, 255), 3)
            cv2.circle(line_image, (340, 360), 5, [150, 10, 25], 10)
    return line_image

# processing image for detecting edge using canny edge detection and blur the image using gaussian blur
def proceesed_img(original_image):
    proceesed_img = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)
    proceesed_img = cv2.GaussianBlur(proceesed_img, (5, 5), 0)
    proceesed_img = cv2.Canny(proceesed_img, threshold1=20, threshold2=12)
    # these polygon repressent the data point within with the pixel data are selected for lane detection
    polygons = np.array([[10, 380], [10, 600], [800, 600], [800, 300], [100, 300]])
    proceesed_img = roi(proceesed_img, [polygons])
    return proceesed_img

def filter_image(color_frame, thresh_yellow_low, thresh_yellow_high, debug):
    #Image processing (resize, hsv, blur, add border, threshold, blur, Canny, contour finding)

    hsv_img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
    blur_img = cv2.GaussianBlur(hsv_img, (5,5), 0)
    threshold_yellow_img = cv2.inRange(blur_img, thresh_yellow_low, thresh_yellow_high)
    #Debug display stuff
    if debug:
        cv2.imshow("Threshold yellow", threshold_yellow_img)
        cv2.waitKey(1)

    #Return stuff
    return threshold_yellow_img

def main():
    time.sleep(3)

    LIVE = True
    file = "xyz.bag"
    pipe, config, profile = setupstream(LIVE, file)
    x_1 = 0

    arduino = Arduino()
    arduino_thread = threading.Thread(target=arduino.run)
    arduino_thread.start()

    while (True):

        raw_color_frame, depth_frame, frameset = getframes(pipe)
        depth_sensor = profile.get_device().first_depth_sensor()
        color_frame = cv2.resize(raw_color_frame, (width, height), interpolation=cv2.INTER_NEAREST)
        thresh = filter_image(color_frame,threshold_yellow.get_low(), threshold_yellow.get_high(), debug)
        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, 100, np.array([]), minLineLength=height/8, maxLineGap=height/12)

        if lines is not None:
            r_avg = np.average(lines[0], axis=0)
            r = r_avg.tolist()
            # with the finded slope and intercept, this is used to find the value of point x on both left and right line
            # the center point is denoted by finding center distance between two lines

            c1, d1, c2, d2 = r
            r_slope = (d2 - d1) / (c2 - c1)
            r_intercept = d1 - (r_slope * c1)
            y = height
            r_x = (y - r_intercept) / r_slope

            offset = r_x - center_fixed
            print("OFF BY {}".format(offset))
            
            arduino.update_angle(angle_constant*(90 + offset))
            arduino.update_speed(speed_constant)
        else:
            pass
            print('slow')

        if r_x != 0:
            line_image = display_line(color_frame, lines)
            combo_image = cv2.addWeighted(color_frame, 0.8, line_image, 1.2, 2)
            cv2.imshow('Lines', combo_image)

        if cv2.waitKey(25) & 0xff == ord('q'):
            cv2.destroyAllWindows()
            break
    return


if __name__ == '__main__':
    main()
