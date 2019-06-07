import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import math
import pyrealsense2 as rs
import sys

from cam import setupstream
from cam import getframes

################ Setup constants ################
#Thresholds: Yellow
<<<<<<< HEAD
thresh_yellow_low = (17,73,56)
thresh_yellow_high = (94,163,255)

#Thresholds: blue
thresh_blue_low = (17,128,56)
thresh_blue_high = (170, 249, 204)

# debug mode
debug = 1

bin_nums = 10
fraction = int(720/bin_nums)
midpoints = []

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

random_colours = [(255,0,0),(255,255,0),(0,255,),(0,255,255),(0,0,255),(255,0,255)]

threshold_blue = Threshold_manager(thresh_blue_low, thresh_blue_high)
threshold_yellow = Threshold_manager(thresh_yellow_low, thresh_yellow_high)

#Setup debug stuff (i.e run headerless if not debug)
if len(sys.argv) > 1:
    debug = True
else:
    debug = False
if debug:
    threshold_blue = Threshold_manager_debug(thresh_blue_low, thresh_blue_high)
    threshold_yellow = Threshold_manager_debug(thresh_yellow_low, thresh_yellow_high)

    cv2.namedWindow("Threshold blue")
    cv2.namedWindow("Threshold yellow")

    cv2.createTrackbar("low hue", "Threshold blue", threshold_blue.lh, 180, threshold_blue.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold blue", threshold_blue.uh, 180, threshold_blue.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold blue", threshold_blue.ls, 255, threshold_blue.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold blue", threshold_blue.us, 255, threshold_blue.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold blue", threshold_blue.lv, 255, threshold_blue.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold blue", threshold_blue.uv, 255, threshold_blue.on_high_V_thresh_trackbar)


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
def display_line(image, lines,x_1,x_2,y,l_x,line_center):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (10, 100, 255), 3)
            cv2.line(line_image, (x_1, y), (x_2, y), (0, 255, 0), 3)
            cv2.line(line_image, (int(l_x + line_center), y + 25), (int(l_x + line_center), y - 25), (100, 25, 50), 5)
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

# this funtions sends the input to the game which is running on left side of screen

#In the original code, the guy would activate keyboard inputs virtually to control a car on the Forza computer game. Maybe we can use this bit to send the
# commands to the Arduino to turn the robot
def straight():
    pass


def little_left():
    pass


def full_left():
    pass


def little_right():
    pass


def full_right():
    pass


def slow():
    pass

# last_time  = time.time()

def filter_image(color_frame, thresh_yellow_low, thresh_yellow_high, thresh_blue_low, thresh_blue_high, debug):
    #Image processing (resize, hsv, blur, add border, threshold, blur, Canny, contour finding)

    hsv_img = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)
    blur_img = cv2.GaussianBlur(hsv_img, (5,5), 0)
    bordered_img = cv2.copyMakeBorder(blur_img, 3, 3, 3, 3, cv2.BORDER_CONSTANT, (0,0,0))

    threshold_yellow_img = cv2.inRange(bordered_img, thresh_yellow_low, thresh_yellow_high)
    edges_yellow = cv2.Canny(threshold_yellow_img, 75, 150) #TODO: Find proper values
    (_, contours_yellow,_) = cv2.findContours(edges_yellow, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    (_, contours_yellow, _) = cv2.findContours(edges_yellow, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    threshold_blue_img = cv2.inRange(bordered_img, thresh_blue_low, thresh_blue_high)
    edges_blue = cv2.Canny(threshold_blue_img, 75, 150)
    (_, contours_blue, _) = cv2.findContours(edges_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)


    #Debug display stuff
    if debug:
        cv2.imshow("Threshold yellow", threshold_yellow_img)
        cv2.imshow("Threshold blue", threshold_blue_img)

        drawn = np.copy(cv2.cvtColor(bordered_img, cv2.COLOR_HSV2BGR))
        drawn = cv2.drawContours(drawn, filtered_contours_yellow, -1, (255,0,0), 2)
        drawn = cv2.drawContours(drawn, filtered_contours_blue, -1, (0, 0, 255), 2)

        for line in range(len(midpoints_yellow)-1):
            drawn = cv2.line(drawn, midpoints_yellow[line], midpoints_yellow[line+1], (255,255,0), 2)
        for line in range(len(midpoints_blue)-1):
            drawn = cv2.line(drawn, midpoints_blue[line], midpoints_blue[line+1], (255,255,0), 2)
        for line in range(len(midpoints_final)-1):
            drawn = cv2.line(drawn, midpoints_final[line], midpoints_final[lline+1], (0, 255, 0), 2)

        cv2.imshow("Lined", drawn)
        cv2.waitKey(1)

    #Return stuff
    return threshold_blue_img


def main():
    time.sleep(3)

    LIVE = True
    file = "xyz.bag"
    pipe, config, profile = setupstream(LIVE, file)
    x_1 = 0


    while (True):

        color_frame, depth_frame, frameset = getframes(pipe)
        depth_sensor = profile.get_device().first_depth_sensor()
        #cv2.imshow("color",color_frame)

        aligned_color = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)
        #resize_img = cv2.resize(np.asanyarray(aligned_color), (320, 180), interpolation=cv2.INTER_NEAREST)
        process_image = proceesed_img(aligned_color)
        gray = cv2.cvtColor(aligned_color, cv2.COLOR_BGR2GRAY)
        cv2.imshow("grey",process_image)
        thresh = filter_image(color_frame,thresh_yellow_low, thresh_yellow_high, thresh_blue_low, thresh_blue_high, debug)
        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, 100, np.array([]), minLineLength=50, maxLineGap=150)

        #print(len(lines))
        left_coordinate = []
        right_coordinate = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (x2 - x1) / (y2 - y1)
                if slope < 0:
                    left_coordinate.append([x1, y1, x2, y2])
                elif slope > 0:
                    right_coordinate.append([x1, y1, x2, y2])
            l_avg = np.average(left_coordinate, axis=0)
            r_avg = np.average(right_coordinate, axis=0)
            l = l_avg.tolist()
            r = r_avg.tolist()
            try:
                # with the finded slope and intercept, this is used to find the value of point x on both left and right line
                # the center point is denoted by finding center distance between two lines

                c1, d1, c2, d2 = r
                a1, b1, a2, b2 = l
                l_slope = (b2 - b1) / (a2 - a1)
                r_slope = (d2 - d1) / (c2 - c1)
                l_intercept = b1 - (l_slope * a1)
                r_intercept = d1 - (r_slope * c1)
                y = 360
                l_x = (y - l_intercept) / l_slope
                r_x = (y - r_intercept) / r_slope
                distance = math.sqrt((r_x - l_x) ** 2 + (y - y) ** 2)
                # line_center repressent the center point on the line
                line_center = distance / 2

                center_pt = [(l_x + line_center)]
                f_r = [(l_x + (line_center * 0.25))]
                f_l = [(l_x + (line_center * 1.75))]
                # create a center point which is fixed
                center_fixed = [340]
                x_1 = int(l_x)
                x_2 = int(r_x)
                '''The logic behind this code is simple,
                the center_fixed should be in the center_line.
                means the cars is in center of the lane, if its get away from center,
                then the left and right functions are used accordingly,then if
                the center fixed is too far from the center_pt the car takes complete left or right accordingly!'''
                if center_pt == center_fixed:
                    straight()
                    print('forward')
                elif center_pt > center_fixed and center_fixed > f_r:
                    little_right()
                    print('right')
                elif center_pt < center_fixed and center_fixed < f_l:
                    little_left()
                    print('left')
                elif center_fixed < f_r:
                    full_right()
                    print('full_ right')
                elif center_fixed > f_l:
                    full_left()
                    print('full_left')
                else:
                    slow()
                    print('okay')


            except:
                pass
                slow()
                print('slow')

        if x_1 != 0:
            line_image = display_line(aligned_color, lines,x_1,x_2,y,l_x,line_center)
            combo_image = cv2.addWeighted(aligned_color, 0.8, line_image, 1.2, 2)
            cv2.imshow('my_driver_bot', cv2.cvtColor(combo_image, cv2.COLOR_BGR2RGB))

        if cv2.waitKey(25) & 0xff == ord('q'):
            cv2.destroyAllWindows()
            break

    #plt.imshow(screen)
    #plt.show()
    return


if __name__ == '__main__':
    # do main stuff
    main()
