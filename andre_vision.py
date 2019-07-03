import cv2
import numpy as np
import logging
import math
import datetime
import threading
import _thread
import time
import serial
import traceback #TODO remove

import pyrealsense2 as rs
import sys

from cam import setupstream
from cam import getframes


_SHOW_IMAGE = True

#CONSTANTS
#Thresholds: Yellow
thresh_yellow_low = (22,78,116)
thresh_yellow_high = (38,192,242)

#Thresholds: blue
thresh_blue_low = (98,46,147)
thresh_blue_high = (135, 246, 239)

#Thresholds: Purple (obstacle)
thresh_purple_low = (141, 82, 34)
thresh_purple_high = (162, 137, 108)

#Thresholds: Green
thresh_green_low = (40, 39, 135)
thresh_green_high = (72, 99, 198)

#Image size
width = 320
height = 180

#Zoning (constants to split into top/middle/bottom sections)
top_mask = height*1/10
section_half_height = (height - top_mask)*1/10
section_overlap = section_half_height * 1/4
border_top = top_mask
top_goal = top_mask + section_half_height
border_middle_top = top_mask + section_half_height*2
middle_goal = top_mask + section_half_height*3
border_middle_bottom = top_mask + section_half_height*4
bottom_goal = top_mask + section_half_height*5
border_bottom = top_mask + section_half_height*6

#Globals rather than costants but shhh
should_stop = False
green_line_detection_history = [False for x in range(10)]

def stopper(arduino):
    global should_stop #Sorry.... :(
    while True:
        leInput = input("Press enter to {}:".format("stop" if should_stop else "start"))
        should_stop = not should_stop
        arduino.mode("STOP" if should_stop else "START")

class FakeArduino:
    def __init__(self):
        self.speed = 90
        self.send_speed = True
        self.angle = 90
        self.send_angle = True

    def update_speed(self, speed):
        if speed > 180:
            speed = 180
        elif speed < 0:
            speed = 0
        self.speed = int(speed)
        self.send_speed = True

    def update_angle(self, angle):
        if angle > 180:
            angle = 180
        elif angle < 0:
            angle = 0
        angle = 180 - angle
        self.angle = int(angle)

    def get_speed(self):
        return self.speed

    def get_angle(self):
        return self.angle

    def mode(self, okay_i_guess_ill_take_your_arguments):
        pass

    def run(self):
        while True:
            time.sleep(0.04)
            if self.send_speed:
                #print(f"M{self.speed:03d}")
                self.send_speed = False
                time.sleep(0.04)
            #print(f"S{self.angle:03d}")
            time.sleep(0.04)

class Arduino:
    def __init__(self):
        self.connection = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.speed = 90
        self.send_speed = True
        self.angle = 90
        self.send_angle = True
        self.should_run = True

    def mode(self, value):
        if value=="STOP":
            self.should_run = False
        elif value=="START":
            self.should_run = True

    def update_speed(self, speed):
        if speed > 180:
            speed = 180
        elif speed < 0:
            speed = 0
        self.speed = int(speed)
        self.send_speed = True

    def update_angle(self, angle):
        if angle > 180:
            angle = 180
        elif angle < 0:
            angle = 0
        angle = 180 - angle
        self.angle = int(angle)

    def get_speed(self):
        return self.speed

    def get_angle(self):
        return self.angle

    def run(self):
        while True:
            #self.connection.write("D000".encode())
            #time.sleep(0.01)
            if self.should_run and self.send_speed:
                self.connection.write(f"M{self.speed:03d}".encode())
                self.send_speed = False
                #time.sleep(0.005)
            elif not self.should_run:
                self.connection.write("M090".encode())
                #time.sleep(0.005)
            self.connection.write(f"S{self.angle:03d}".encode())

"""class Stopper:
    def __init__(self, arduino):
        self.arduino = arduino

    def run(self):
        while True:
            self.arduino.update_speed(90)
            time.sleep(4)
            input("Press enter to start again:")
            self.arduino.update_speed(80)
"""

class Camera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,
                                  30)  # enable_stream(source, width, height, format, fps)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8,
                                  30)  # Intel resources say 1280 & 720 is best for the depth calculations, then you want to downsize it later)
        self.pipeline.start(self.config)
        self.color_out = None
        self.depth_out = None
        self.run = True

    def take_pics(self):
        while self.run:
            # Currently, just runs the color as we're not (yet) using depth
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            # depth_frame = frames.get_depth_frame()
            """if not color_frame or not depth_frame:
                pass
            else:
                self.color_out = color_frame
                self.depth_out = depth_frame"""

            # Temp color-only code:
            if color_frame is not None:
                self.color_out = color_frame.get_data()
                self.timestamp = time.time()

    def stop(self):
        self.run = False
        self.pipeline.stop()

    def get_color_frame(self):
        return self.color_out

    def get_depth_frame(self):
        return self.depth_out

    def get_timestamp(self):
        return self.timestamp

class HandCodedLaneFollower(object):

    def __init__(self, car=None):
        logging.info('Creating a HandCodedLaneFollower...')
        self.car = car
        self.curr_steering_angle = 90
        self.arduino = FakeArduino()
        self.arduino_thread = threading.Thread(target=self.arduino.run)
        self.arduino_thread.start()

        #self.stopper = Stopper(self.arduino)
        #self.stopper_thread = threading.Thread(target=self.stopper.run)
        #self.stopper_thread.start()

    def follow_lane(self, frame):

        # Main entry point of the lane follower
        #show_image("orig", frame)
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_NEAREST)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) #TODO remove
        crop_polygon = np.array([[
            (0, top_mask),
            (width, top_mask),
            (width, height),
            (0, height)
        ]], np.int32)

        color_image = np.copy(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        show_image("original", color_image)
        yellow_points, blue_points, lane_lines_image, change_in_x_lines, yellow_gradients, blue_gradients = detect_lane(frame, color_image) #points = (bottom, middle, top) x values

        #green_line = detect_green(frame, average_gradient)
        #green_line_detection_history.pop(0)
        #green_line_detection_history.append(green_line)
        #if sum([1 for x in green_line_detection_history if x is True])/len(green_line_detection_history) > 0.2:
        #    should_stop = True
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mid_points = [None, None, None]
        change_in_x_yellow_lines = change_in_x_lines[:3]
        change_in_x_blue_lines = change_in_x_lines[3:]
        yellow_goal_points = (310, 300, 290) #TODO find empirically
        blue_goal_points = (10, 20, 30)

        # Check for obstacles and return the center point
        obs_x, obs_y = detect_obstacle(frame)

        # cacluate where to steer if an object exists
        if obs_x != 0 and obs_y != 0:
            blue_points, yellow_points = avoid_obstacle(frame, obs_x, obs_y, yellow_points, blue_points)

        found_midpoints = 0
        for counter in range(3):
            if yellow_points[counter] is not None and blue_points[counter] is not None:
                mid_points[counter] = (yellow_points[counter] + blue_points[counter])/2
                found_midpoints += 1

        if found_midpoints == 0: #Just go off estimates
            for counter in range(3):
                if yellow_points[counter] is not None and yellow_gradients[counter] is not None:
                    mid_points[counter] = yellow_points[counter] - 3 * width/4 - 1/(yellow_gradients[counter]**4) #- yellow_goal_points[counter]
                elif blue_points[counter] is not None and blue_gradients[counter] is not None:
                    mid_points[counter] = 3*width/4 + blue_points[counter] + 1/(blue_gradients[counter]**4)#- blue_goal_points[counter]

        elif found_midpoints == 1: #Use offset from the single midpoint
            #Find which one has the midpoint
            midpoint = None
            for point in range(3):
                if mid_points[point] is not None:
                    midpoint = point

            for counter in range(3):
                if mid_points[counter] is None:
                    if yellow_points[counter] is not None and change_in_x_yellow_lines[counter] is not None:
                        #mid_points[counter] = mid_points[midpoint] + (yellow_goal_points[midpoint] - yellow_goal_points[counter]) - (yellow_points[midpoint] - yellow_points[counter])
                        if counter > midpoint:
                            mid_points[counter] = mid_points[midpoint] + 0.5 * change_in_x_yellow_lines[counter]
                        elif counter < midpoint:
                            mid_points[counter] = mid_points[midpoint] - 0.5 * change_in_x_yellow_lines[counter]
                    elif blue_points[counter] is not None and change_in_x_blue_lines[counter] is not None:
                        #mid_points[counter] = mid_points[midpoint] + (blue_goal_points[midpoint] - blue_goal_points[counter]) - (blue_points[midpoint] - blue_points[counter])
                        if counter > midpoint:
                            mid_points[counter] = mid_points[midpoint] + 0.5 * change_in_x_blue_lines[counter]
                        elif counter < midpoint:
                            mid_points[counter] = mid_points[midpoint] - 0.5 * change_in_x_blue_lines[counter]

        elif found_midpoints == 2:
            first_midpoint = None
            second_midpoint = None
            for point in range(3):
                if mid_points[point] is not None:
                    if first_midpoint is None:
                        first_midpoint = point
                    else:
                        second_midpoint = point

            #Find the expected gradient of the line
            try:
                m = (second_midpoint-first_midpoint)/(mid_points[second_midpoint] - mid_points[first_midpoint])
            except ZeroDivisionError:
                m = 1024
            b = first_midpoint - m * mid_points[first_midpoint]
            for counter in range(3):
                if mid_points[counter] is None:
                    if yellow_points[counter] is not None:
                        mid_points[counter] = (1/m)*(counter - b)

        #Draw mid points
        mid_points_draw = ((mid_points[0], bottom_goal), (mid_points[1], middle_goal), (mid_points[2], top_goal))
        mid_points_draw = [point for point in mid_points_draw if point[0] is not None]
        cv2.circle(lane_lines_image, (obs_x, obs_y), 5, (255, 255, 255), -1)
        cv2.putText(lane_lines_image, "centroid", (obs_x - 25, obs_y - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        lane_lines_image = display_points(lane_lines_image, mid_points_draw, point_color=(127,0,255))
        show_image("Mid points", lane_lines_image)

        #Do steering stuff
        #final_frame = self.steer(frame, lane_lines)
        points = [x for x in ((mid_points[0], bottom_goal), (mid_points[1], middle_goal), (mid_points[2], top_goal))]
        try:
            angle = calculate_angle(points, angle)
        except NameError:
            angle = calculate_angle(points)

        if angle:
            speed = calculate_speed(points, angle)
            self.arduino.update_angle(angle)
            if speed:
                self.arduino.update_speed(speed)
        else:
            self.arduino.update_speed(75)


def calculate_angle(midpoints, old_steer=90):
    # midpoints is a list of midpoints :)
    # returns the value that should be sent to arduino
    kp = 4 #KPP
    kd = -0
    #ko = 0.4
    old_steer = old_steer - 90 if old_steer > 90 else 90 - old_steer

    midpoint_list = [x for x in midpoints if x[0] != None]

    # if no midpoints found don't turn?? reconsider this later
    if len(midpoint_list) == 0:
        return 180

    # one midpoint == only proportional
    if len(midpoint_list) == 1:
        try:
            x, y = midpoint_list[0]
            # theta = x from centre to midpoint / y from midpoint to bottom
            # theta = (x - width/2) / (height - y)
            # theta = math.degrees(math.atan(theta))

            l = x - width/2
            delta_l = x - width/2
            delta_h = height - y

            # -20 acts as a stabiliser
            return 90 + kp * (l) + kd * (delta_l / delta_h) #- ko * old_steer
        except:
            return None

    # more than one midpoint == proportional and differential
    if len(midpoint_list) == 2:
        try:
            x1, y1 = midpoint_list[0]
            x2, y2 = midpoint_list[1]

            #theta1 = (x1 - width/2) / (height - y1)
            #theta1 = math.degrees(math.atan(theta1))
            #theta2 = (x2 - width/2) / (height - y2)
            #theta2 = math.degrees(math.atan(theta2))
            #theta_average = theta1 + theta2
            #theta_average /= 2

            l = x1 - width/2
            delta_l = x2 - x1
            delta_h = y1 - y2

            return 90 + kp * (l) + kd * (delta_l / delta_h) #- ko * old_steer
        except:
            return None

    if len(midpoint_list) == 3:
        try:
            x1, y1 = midpoint_list[0]
            x2, y2 = midpoint_list[1]
            x3, y3 = midpoint_list[2]

            # theta1 = (x1 - width/2) / (height - y1)
            # theta1 = math.degrees(math.atan(theta1))
            # theta2 = (x2 - width/2) / (height - y2)
            # theta2 = math.degrees(math.atan(theta2))
            # theta3 = (x3 - width/2) / (height - y3)
            # theta3 = math.degrees(math.atan(theta3))

            # theta_average = theta1 + theta2 + theta3
            # theta_average /= 3

            l = x1 - width/2
            delta1 = (x2 - x1) / (y1 - y2)
            delta2 = (x3 - x2) / (y2 - y3)
            delta_average = (delta1 + delta2) / 2

            # return kp * (theta1 - 20) + kd * (change_in_theta1 - 20 / height) + kd * (change_in_theta2 - 20 / height)
            return 90 + kp * (l) + kd * (delta_average) #- ko * old_steer
        except:
            return None

    # if something wrong tho don't steer
    return 90

def calculate_speed(midpoints, steer):
    midpoint_list = [x for x in midpoints if x[0] != None]
    # if no midpoints found don't move?? reconsider this later
    if len(midpoint_list) == 0:
        return 75

    steer *= 0.05
    steer = 180 if steer > 180 else steer
    steer = 0 if steer < 0 else steer
    steer = steer - 90 if steer > 90 else 90 - steer
    steer = 90 - steer
    min_speed = 84
    max_speed_proportional = 75
    kp = (min_speed - max_speed_proportional) / 90
    max_speed_integration = 70
    ki = 0

    proportional_speed = min_speed + kp * steer
    # one midpooint == only proportional
    if len(midpoint_list) == 1:

        return proportional_speed

    # more than one midpoint == proportional and integration

    if len(midpoint_list) == 2:
        try:
            x1, y1 = midpoint_list[0]
            x2, y2 = midpoint_list[1]

            change_in_y = y1 - y2

            ki = max_speed_integration / (max_speed_proportional * change_in_y)

            return ki * change_in_y * proportional_speed
        except:
            return None

    if len(midpoint_list) == 3:
        try:
            x1, y1 = midpoint_list[0]
            x2, y2 = midpoint_list[1]
            x3, y3 = midpoint_list[2]

            change_in_y_1 = y1 - y2
            change_in_y_2 = y3 - y2
            change_in_y_average = (change_in_y_1 + change_in_y_2) / 2

            # ki_1 = max_speed_integration / (max_speed_proportional * change_in_y1)
            # ki_2 = max_speed_integration / (max_speed_proportional * change_in_y2)

            ki = max_speed_integration / (max_speed_proportional * change_in_y_average)

            # return ki_1 * change_in_y1 * proportional_speed + ki_2 * change_in_y2 * proportional_speed
            return ki * change_in_y_average * proportional_speed
        except:
            return None

    # if something wrong tho don't move
    return 90

############################
# Frame processing steps
############################
def detect_lane(frame, color_image):
    logging.debug('detecting lane lines...')

    yellow_edges = detect_edges(frame,thresh_yellow_low,thresh_yellow_high)
    blue_edges = detect_edges(frame,thresh_blue_low,thresh_blue_high)

    #show_image('yellow edges', yellow_edges)
    #show_image('blue edges', blue_edges)

    #Crop out top of image
    crop_polygon = np.array([[
        (0, top_mask),
        (width, top_mask),
        (width, height),
        (0, height)
    ]], np.int32)

    yellow_cropped = region_of_interest(yellow_edges, crop_polygon)
    blue_cropped = region_of_interest(blue_edges, crop_polygon)

    show_image('yellow edges', yellow_cropped)
    show_image('blue edges', blue_cropped)

    yellow_line_segments = detect_line_segments(yellow_cropped)
    blue_line_segments = detect_line_segments(blue_cropped)

    if yellow_line_segments is None:
        yellow_line_segments = []
    #else: #TODO REMOVE
    #    yellow_line_segments = [yellow_line_segments[0]] #TODO REMOVE
    if blue_line_segments is None:
        blue_line_segments = []
    #else: #TODO REMOVE
    #    blue_line_segments = [blue_line_segments[0]] #TODO REMOVE

    line_segment_image_yellow = display_lines(frame, yellow_line_segments)
    show_image("yellow line segments", line_segment_image_yellow)
    line_segment_image_blue = display_lines(frame, blue_line_segments)
    show_image("blue line segments", line_segment_image_blue)

    #Split lines into three segments:
    yellow_bottom, yellow_mid, yellow_top = split_lines(yellow_line_segments, height)
    blue_bottom, blue_mid, blue_top = split_lines(blue_line_segments, height)

    lane_lines_image = display_lines(np.copy(color_image), (((0, border_top, width, border_top),), ((0, top_goal, width, top_goal),), ((0, border_middle_top, width, border_middle_top),), ((0, middle_goal, width, middle_goal),), ((0, border_middle_bottom, width, border_middle_bottom),), ((0, bottom_goal, width, bottom_goal),), ((0, border_bottom, width, border_bottom),)), line_color=(255,255,255), line_width=1)

    blue_top_image = display_lines(frame, blue_top)
    show_image("blue line top segments", blue_top_image)
    blue_mid_image = display_lines(frame, blue_mid)
    show_image("blue mid segments", blue_mid_image)
    blue_bottom_image = display_lines(frame, blue_bottom)
    show_image("blue bottom segments", blue_bottom_image)

    yellow_top_image = display_lines(frame, yellow_top)
    show_image("yellow line top segments", yellow_top_image)
    yellow_mid_image = display_lines(frame, yellow_mid)
    show_image("yellow mid segments", yellow_mid_image)
    yellow_bottom_image = display_lines(frame, yellow_bottom)
    show_image("yellow bottom segments", yellow_bottom_image)

    yellow_bottom_line = section_average_slope_intercept(yellow_bottom, bottom_goal) #returns (gradient, intercept)
    yellow_mid_line = section_average_slope_intercept(yellow_mid, middle_goal)
    yellow_top_line = section_average_slope_intercept(yellow_top, top_goal)

    blue_bottom_line = section_average_slope_intercept(blue_bottom, bottom_goal) #returns (gradient, intercept)
    blue_mid_line = section_average_slope_intercept(blue_mid, middle_goal)
    blue_top_line = section_average_slope_intercept(blue_top, top_goal)

    change_in_x_lines = []
    gradients = []

    """yellow_bottom_line = average_slope_intercept(frame, yellow_bottom) #returns (gradient, intercept)
    yellow_mid_line = average_slope_intercept(frame, yellow_mid)
    yellow_top_line = average_slope_intercept(frame, yellow_top)

    blue_bottom_line = average_slope_intercept(frame, blue_bottom) #returns (gradient, intercept)
    blue_mid_line = average_slope_intercept(frame, blue_mid)
    blue_top_line = average_slope_intercept(frame, blue_top)"""

    #Subbing in for x (y=mx+b style)

    if yellow_bottom_line is not None:
        yellow_bottom_point = (1/yellow_bottom_line[0])*(bottom_goal - yellow_bottom_line[1])
        change_in_x = yellow_bottom_point - (1/yellow_bottom_line[0])*(border_bottom - yellow_bottom_line[1])
        change_in_x_lines.append(2 * change_in_x)
        gradients.append(yellow_bottom_line[0])
    else:
        yellow_bottom_point = None
        change_in_x_lines.append(None)

    if yellow_mid_line is not None:
        yellow_mid_point = (1/yellow_mid_line[0])*(middle_goal - yellow_mid_line[1])
        change_in_x = yellow_mid_point - (1/yellow_mid_line[0])*(border_middle_bottom - yellow_mid_line[1])
        change_in_x_lines.append(2 * change_in_x)
        gradients.append(yellow_mid_line[0])
    else:
        yellow_mid_point = None
        change_in_x_lines.append(None)

    if yellow_top_line is not None:
        yellow_top_point = (1/yellow_top_line[0])*(top_goal - yellow_top_line[1])
        change_in_x = yellow_top_point - (1/yellow_top_line[0])*(border_middle_top - yellow_top_line[1])
        change_in_x_lines.append(2 * change_in_x)
        gradients.append(yellow_top_line[0])
    else:
        yellow_top_point = None
        change_in_x_lines.append(None)


    if blue_bottom_line is not None:
        blue_bottom_point = (1/blue_bottom_line[0])*(bottom_goal - blue_bottom_line[1])
        change_in_x = blue_bottom_point - (1/blue_bottom_line[0])*(border_bottom - blue_bottom_line[1])
        change_in_x_lines.append(2 * change_in_x)
        gradients.append(blue_bottom_line[0])
    else:
        blue_bottom_point = None #TODO replace 0 with None, do properly
        change_in_x_lines.append(None)

    if blue_mid_line is not None:
        blue_mid_point = (1/blue_mid_line[0])*(middle_goal - blue_mid_line[1])
        change_in_x = blue_mid_point - (1/blue_mid_line[0])*(border_middle_bottom - blue_mid_line[1])
        change_in_x_lines.append(2 * change_in_x)
        gradients.append(blue_mid_line[0])
    else:
        blue_mid_point = None
        change_in_x_lines.append(None)

    if blue_top_line is not None:
        blue_top_point = (1/blue_top_line[0])*(top_goal - blue_top_line[1])
        change_in_x = blue_top_point - (1/blue_top_line[0])*(border_middle_top - blue_top_line[1])
        change_in_x_lines.append(2 * change_in_x)
        gradients.append(blue_top_line[0])
    else:
        blue_top_point = None
        change_in_x_lines.append(None)

    yellow_points = (yellow_bottom_point, yellow_mid_point, yellow_top_point)
    blue_points = (blue_bottom_point, blue_mid_point, blue_top_point)

    for point in yellow_points:
        if point is not None:
            if (-width > point):
                point = -width
            elif point < width*2:
                point = 2*width
    for point in blue_points:
        if point is not None:
            if -width > point:
                point = -width
            elif point < width*2:
                point = 2*width

    #Display stuff
    if yellow_bottom_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(border_bottom, yellow_bottom_line[0], yellow_bottom_line[1]), border_bottom, calculate_x_from_y(border_middle_bottom, yellow_bottom_line[0], yellow_bottom_line[1]), border_middle_bottom),),), line_color=(204,102,0))
    if yellow_mid_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(border_middle_bottom, yellow_mid_line[0], yellow_mid_line[1]), border_middle_bottom, calculate_x_from_y(border_middle_top, yellow_mid_line[0], yellow_mid_line[1]), border_middle_top),),), line_color=(255,153,51))
    if yellow_top_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(border_middle_top, yellow_top_line[0], yellow_top_line[1]), border_middle_top, calculate_x_from_y(border_top, yellow_top_line[0], yellow_top_line[1]), border_top),),), line_color=(255,204,153))

    if blue_bottom_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(border_bottom, blue_bottom_line[0], blue_bottom_line[1]), border_bottom, calculate_x_from_y(border_middle_bottom, blue_bottom_line[0], blue_bottom_line[1]), border_middle_bottom),),), line_color=(0,153,153))
    if blue_mid_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(border_middle_bottom, blue_mid_line[0], blue_mid_line[1]), border_middle_bottom, calculate_x_from_y(border_middle_top, blue_mid_line[0], blue_mid_line[1]), border_middle_top),),), line_color=(0,255,255))
    if blue_top_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(border_middle_top, blue_top_line[0], blue_top_line[1]), border_middle_top, calculate_x_from_y(border_top, blue_top_line[0], blue_top_line[1]), border_top),),), line_color=(153,255,255))

    line_points = ((yellow_bottom_point, bottom_goal), (yellow_mid_point, middle_goal), (yellow_top_point, top_goal), (blue_bottom_point, bottom_goal), (blue_mid_point, middle_goal), (blue_top_point, top_goal))
    line_points = [point for point in line_points if point[0] is not None]
    lane_lines_image = display_points(lane_lines_image, line_points)

    show_image("lane lines", lane_lines_image)

    return (yellow_bottom_point, yellow_mid_point, yellow_top_point), (blue_bottom_point, blue_mid_point, blue_top_point), lane_lines_image, change_in_x_lines, (yellow_bottom_line[0] if yellow_bottom_line is not None else None, yellow_mid_line[0] if yellow_mid_line is not None else None, yellow_top_line[0] if yellow_top_line is not None else None), (blue_bottom_line[0] if blue_bottom_line is not None else None, blue_mid_line[0] if blue_mid_line is not None else None, blue_top_line[0] if blue_top_line is not None else None)

def detect_green(frame, lane_line_gradient):
    green_mask = np.array([[
        (0, border_middle_bottom),
        (width, border_middle_top),
        (width, height),
        (0, height)
    ]], np.int32)

    green_edges = detect_edges(frame, thresh_green_low, thresh_green_high)
    frame = region_of_interest(frame, green_mask)
    show_image("green_edges", green_edges)
    if lane_line_gradient is not None:
        lane_line_angle = abs(math.atan(lane_line_gradient))
        green_lines = detect_line_segments(green_edges)
        green_line_image = display_lines(frame, green_lines)
        show_image("Green lines", green_line_image)
        if (green_lines is not None):
            green_line = section_average_slope_intercept(green_lines, bottom_goal, give_horizontals=True)
            if green_line is not None:
                green_line_angle = abs(math.atan(green_line[0]))
                if (0.5 < lane_line_angle - green_line_angle < 0.9 and -0.1 < green_line_angle < 0.1): #Is roughly perpeundicular to the lane lines and horizontal
                    """#Check is roughly inside the lane lines: (If can't see the lane lines, that it at least passes through the middle of the image)
                     #Inside lane lines:
                    non_none_blue = [point for point in blue_points if point is not None]
                    non_none_yellow = [point for point in yellow_points if point is not None]
                     fail = False
                    if len(non_none_blue) > 0:
                        if min()"""
                    return True
    return False

def calculate_x_from_y(y, gradient, intercept):
    return (1/gradient)*(y-intercept)

def split_lines(lines, height):
    bottom = []
    middle = []
    top = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        #Make y2 always bottom (higher value) than y1
        if y2 > y1:
            temp = y1
            y1 = y2
            y2 = temp
            temp = x1
            x1 = x2
            x2 = temp

        #Bottom
        if ((border_middle_bottom < y2 and border_bottom > y1) or #Both ends inside bottom zone
          (bottom_goal < y1 < border_bottom + section_overlap) or #Lower end far inside bottom zone
          (border_middle_bottom < y1 < border_bottom and border_middle_bottom - section_overlap < y2)): #end in bottom zone, top not far off
            bottom.append(line)

        #Middle
        if ((border_middle_top > y2 and border_middle_bottom < y1) or #Both ends in middle zone
          (middle_goal < y1 < border_middle_bottom) or #Bottom end far in middle zone
          (border_middle_top < y2 < middle_goal) or #Top end far in middle zone
          (border_middle_top < y1 < border_middle_bottom and border_middle_top - section_overlap < y2) or #Bottom end in middle zone, and top is very close to middle zone
          (border_middle_top < y2 < border_middle_bottom * 5/6 and y1 < border_middle_top + section_overlap)): #Top end in middle zone, and bottom is very close to middle zone
            middle.append(line)

        #Top
        if ((border_top < y1 < border_middle_top) or #Both ends inside top zone
          (border_top < y2 < top_goal) or #Top end far inside top zone
          (border_top < y2 < border_middle_top and border_middle_top + section_overlap < y1)): #Top inside zone, bottom near zone
            top.append(line)

    top_needs_more = True if len(top) <= 1 else False
    middle_needs_more = True if len(middle) <= 1 else False
    bottom_needs_more = True if len(bottom) <= 1 else False

    if (top_needs_more or middle_needs_more or bottom_needs_more):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            #Make y2 always bottom (higher value) than y1
            if y2 > y1:
                temp = y1
                y1 = y2
                y2 = temp
                temp = x1
                x1 = x2
                x2 = temp

            #Bottom
            if (bottom_needs_more) and (border_middle_bottom < y2):
                bottom.append(line)

            #Middle
            if (middle_needs_more) and ((border_middle_top < y1 < border_middle_bottom) or (border_middle_top < y2 < border_middle_bottom)):
                middle.append(line)

            #Top
            if (top_needs_more) and (border_middle_top > y1):
                top.append(line)

    return bottom, middle, top

def detect_edges(frame,low_thresh,high_thresh):
    # filter for  lane lines
    # show_image("hsv", hsv)
    mask = cv2.inRange(frame, low_thresh, high_thresh)
    #show_image("filtered colour", mask)

    # detect edges
    #edges = cv2.Canny(mask, 200, 400)

    return mask

def region_of_interest(canny,polygon):
    mask = np.zeros_like(canny)

    cv2.fillPoly(mask, polygon, 255)
    #show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=20,
                                    maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments

def section_average_slope_intercept(line_segments, goal_height, give_horizontals=False):
    """
    Same logic as average_slope_intercept but without differentiating between left and right lines
    (as we can split left and right lines by colour)

    Rather than averaging gradients and intercepts, to solve issue of vertical lines average the
    angle and intercept with x-axis (for horizontal lines, include angle but forget intercept because they're not common
    and idk what to do.)
    Potential improvement if lines aren't averaging well - average angle and distance from origin to closest point on the line.
    """
    angles = []
    distances = []
    num_lines = len(line_segments)
    if num_lines == 0:
        return None
    for line in line_segments:
            x1, y1, x2, y2 = line[0]
            if (x2==x1):
                angles.append(math.pi/2)
                distances.append(x1)
            elif (y2 == y1):
                angles.append(0)
                distances.append(y1)
            else:
                angles.append(math.atan((y2-y1)/(x2-x1)))
                #Distance calculation - find general form values (set a to 1).
                b = -(x2-x1)/(y2-y1)
                c = -y1*b - x1
                distances.append(((b*goal_height + c))/math.sqrt(1+b**2))

    #Average
    #Find number of data points to include in average:
    data_point_num = num_lines - round(num_lines*0.5)
    if (num_lines%2==0 and data_point_num%2==1) or (num_lines%2==1 and data_point_num%2==0):
        data_point_num += 1
    start = int((num_lines - data_point_num)/2)
    end = num_lines - start

    angles.sort()
    average_angle = sum(angles[start:end])/data_point_num
    distances.sort()
    average_distance = sum(distances[start:end])/data_point_num

    #Check 2/3rds of angles are within pi/8 radians
    fail_num = 0
    for angle in angles:
        if angle - average_angle > math.pi/8 or angle - average_angle < -math.pi/8:
            fail_num += 1
    if fail_num > num_lines*2/3:
        return None
    fail_num = 0
    for distance in distances:
        if abs(distance - average_distance) > 25:
            fail_num += 1
    if fail_num > num_lines*2/3:
        return None

    #average_distance = distances[int(num_lines/2)]
    if average_angle == math.pi/2: #Average still vertical:
        average_gradient = 1024 #idk seems like a large enough number?
        average_intercept = -average_distance*math.sqrt(average_gradient**2 + 1) + goal_height
    elif average_angle == 0 or abs(average_angle) == math.pi:
        if not give_horizontals:
            return None #Probably should do better something for horizontal lines than just giving up
        else:
            average_gradient = 0
            return (average_gradient, average_distance*math.sqrt(average_gradient**2 + 1))
    else:
        average_gradient = math.tan(average_angle)
        average_intercept = average_distance*math.sqrt(average_gradient**2 + 1)* (-1 if average_gradient < 0 else 1) + goal_height
    return average_gradient, average_intercept

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    fit = []
    #right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary  # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            poly = np.polyfit((x1, x2), (y1, y2), 1)
            slope = poly[0]
            intercept = poly[1]
            fit.append((slope, intercept))

    fit_average = np.average(fit, axis=0)
    if len(fit) > 0:
        return fit_average
    else:
        return None

# Using blob detect and taking centroid of the blob as a region to avoid
def detect_obstacle(frame):

    # Threshold image, to filter out all but purple
    gray_image = detect_edges(frame, thresh_purple_low, thresh_purple_high)

    # Filter out bad readings
    median = cv2.medianBlur(gray_image,15)
    cv2.imshow("filter", median)
    cv2.waitKey(1)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(median,127,255,0)

    # calculate moments of binary image
    M = cv2.moments(thresh)

    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0,0

    # put text and highlight the center
    if cX != 0 and cY != 0:
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # display the image
    #cv2.imshow("Image", frame)
    #cv2.waitKey(1)

    return cX, cY

def avoid_obstacle(frame, obs_x, obs_y, yellow_points, blue_points):

    width = frame.shape[1]
    print(width)
    display = 0

    if display == 1:
        print("yellow", yellow_points)
        print("blue", blue_points)
        print("obs_x", obs_x)

    # Determine where the obstacle is on the screen
    if obs_x < width/2:

        # set this x coordinate as the new right line
        blue_points = (obs_x,obs_x , obs_x)

    elif obs_x > width/2:

        # set this x coordinate as the new left lines
        yellow_points = (obs_x,obs_x, obs_x)

    if display == 1:
        print("new yellow", yellow_points)
        print("new blue", blue_points)

    return blue_points, yellow_points

def filter_contours(contours):
    final_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if obstacle_area_min < area < obstacle_area_max:
            perimeter = cv2.arcLength(contour, True)
            if obstacle_perimeter_min < perimeter < obstacle_perimeter_max:
                rect = cv2.minAreaRect(contour)
                width = min(rect[1])
                height = max(rect[1])
                ratio = width/height


def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return 90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = 90 + angle_to_mid_deg # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines,
                             max_angle_deviation_two_lines=20, max_angle_deviation_one_lane=20):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2:
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else:
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle


    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle


############################
# Utility Functions
############################
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            try:
                x1 = int(line[0][0])
                y1 = int(line[0][1])
                x2 = int(line[0][2])
                y2 = int(line[0][3])
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
            except (OverflowError, ValueError) as e:
                print(e)
    line_image = cv2.addWeighted(frame, 1, line_image, 0.8, 1)
    return line_image

def display_points(frame, points, point_color=(138,43,226), point_radius=6):
    point_image = np.zeros_like(frame)
    if points is not None:
        for point in points:
            try:
                cv2.circle(point_image, (int(point[0]), int(point[1])), point_radius, point_color, thickness=point_radius)
            except (ValueError, OverflowError) as e:
                print(e)
    point_image = cv2.addWeighted(frame, 1, point_image, 0.8, 1)
    return point_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 3)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    try:
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]
    except OverflowError:
        return [[-width, y1, -width, y1]]

class File_Inputter:
    def __init__(self):
        self.frame_counter = 0
        self.frame_goal = 1
        self.prev_increment = 50

    def next_frame_counter(self):
        while True:
            leInput = input("At frame {}: ".format(self.frame_goal))
            if leInput == "":
                self.frame_goal += self.prev_increment
            else:
                try:
                    self.frame_goal += int(leInput)
                    self.prev_increment = int(leInput)
                except ValueError:
                    pass

#cap=Camera()
#video_file='test5'
frame_input = File_Inputter()
_thread.start_new_thread(frame_input.next_frame_counter, tuple())
lane_follower = HandCodedLaneFollower()
#_thread.start_new_thread(stopper, (lane_follower.arduino,)) #TODO turn stopper back on
print("Running...")

def main():
    time.sleep(3)
    #counter_serial = 0

    #REMEMBER TO REMOVE RGB2BGR COLOUR FLIP (FOR BAG FILE)
    LIVE = False
    file = "4th.bag"
    pipe, config, profile = setupstream(LIVE, file)

    #while cap.isOpened():
    while (True):

            if frame_input.frame_counter < frame_input.frame_goal:
                frame, depth_frame, frameset = getframes(pipe)
                frame_input.frame_counter += 1
            #counter_serial += 1
            #print(counter_serial)

            #frame, depth_frame, frameset = getframes(pipe)
            combo_image = lane_follower.follow_lane(frame)
            time.sleep(0.04)

            if cv2.waitKey(25) & 0xff == ord('q'):
                cv2.destroyAllWindows()
                break
    return


if __name__ == '__main__':
    # do main stuff
    main()
