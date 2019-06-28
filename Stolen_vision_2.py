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
#Threshold: yellow
thresh_yellow_low = (20,19,161)
thresh_yellow_high = (47,255,255)

#Thresholds: blue
thresh_blue_low = (96,30,147)
thresh_blue_high = (145, 246, 239)

class FakeArduino:
    def __init__(self):
        self.speed = 80
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
        self.speed = 80
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

    def run(self):
        while True:
            self.connection.write(b"D000")
            time.sleep(0.04)
            if self.send_speed:
                self.connection.write(f"M{self.speed:03d}".encode())
                self.send_speed = False
                time.sleep(0.04)
            self.connection.write(f"S{self.angle:03d}".encode())
            time.sleep(0.04)

class Stopper:
    def __init__(self, arduino):
        self.arduino = arduino

    def run(self):
        while True:
            self.arduino.update_speed(90)
            time.sleep(4)
            input("Press enter to start again:")
            self.arduino.update_speed(80)

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

        lane_lines, frame = detect_lane(frame)
        #final_frame = self.steer(frame, lane_lines)
        final_frame = frame

        return final_frame

    def steer(self, frame, lane_lines):
        logging.debug('steering...')
        if len(lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
            return frame

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle,
                                                            len(lane_lines))

        self.arduino.update_angle(self.curr_steering_angle)
        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)

        return curr_heading_image


############################
# Frame processing steps
############################
def detect_lane(frame):
    logging.debug('detecting lane lines...')

    frame = cv2.resize(frame, (320, 180), interpolation=cv2.INTER_NEAREST)
    yellow_edges = detect_edges(frame,thresh_yellow_low,thresh_yellow_high)
    blue_edges = detect_edges(frame,thresh_blue_low,thresh_blue_high)

    #show_image('yellow edges', yellow_edges)
    #show_image('blue edges', blue_edges)

    height,width=yellow_edges.shape

    #Crop out top of image
    crop_polygon = np.array([[
        (0, height / 3),
        (width, height / 3),
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
    if blue_line_segments is None:
        blue_line_segments = []

    line_segment_image_yellow = display_lines(frame, yellow_line_segments)
    show_image("yellow line segments", line_segment_image_yellow)
    line_segment_image_blue = display_lines(frame, blue_line_segments)
    show_image("blue line segments", line_segment_image_blue)

    #Split lines into three segments:
    yellow_bottom, yellow_mid, yellow_top = split_lines(yellow_line_segments, height)
    blue_bottom, blue_mid, blue_top = split_lines(blue_line_segments, height)

    yellow_bottom_line = section_average_slope_intercept(yellow_bottom) #returns (gradient, intercept)
    yellow_mid_line = section_average_slope_intercept(yellow_mid)
    yellow_top_line = section_average_slope_intercept(yellow_top)

    blue_bottom_line = section_average_slope_intercept(blue_bottom) #returns (gradient, intercept)
    blue_mid_line = section_average_slope_intercept(blue_mid)
    blue_top_line = section_average_slope_intercept(blue_top)

    #Subbing in for x (y=mx+b style)
    if yellow_bottom_line is not None:
        yellow_bottom_point = (1/yellow_bottom_line[0])*(height - yellow_bottom_line[1])
    else:
        yellow_bottom_point = None #TODO replace 0 with None, do properly

    if yellow_mid_line is not None:
        yellow_mid_point = (1/yellow_mid_line[0])*(height*2/3 - yellow_mid_line[1])
    else:
        yellow_mid_point = None

    if yellow_top_line is not None:
        yellow_top_point = (1/yellow_top_line[0])*(height*0.5 - yellow_top_line[1])
    else:
        yellow_top_point = None
    
    
    if blue_bottom_line is not None:
        blue_bottom_point = (1/blue_bottom_line[0])*(height - blue_bottom_line[1])
    else:
        blue_bottom_point = None #TODO replace 0 with None, do properly

    if blue_mid_line is not None:
        blue_mid_point = (1/blue_mid_line[0])*(height*2/3 - blue_mid_line[1])
    else:
        blue_mid_point = None

    if blue_top_line is not None:
        blue_top_point = (1/blue_top_line[0])*(height*0.5 - blue_top_line[1])
    else:
        blue_top_point = None


    if yellow_bottom_point is not None and blue_bottom_point is not None:
        average_bottom_point = (yellow_bottom_point + blue_bottom_point) / 2
    else:
        average_bottom_point = None
    if yellow_mid_point is not None and blue_mid_point is not None:
        average_mid_point = (yellow_mid_point + blue_mid_point) / 2
    else:
        average_mid_point = None
    if yellow_top_point is not None and blue_top_point is not None:
        average_top_point = (yellow_top_point + blue_top_point) / 2
    else:
        average_top_point = None


    #Display stuff
    lane_lines_image = np.copy(frame)
    if yellow_bottom_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(height, yellow_bottom_line[0], yellow_bottom_line[1]), height, calculate_x_from_y(height*2/3, yellow_bottom_line[0], yellow_bottom_line[1]), height*2/3),),), line_color=(204,102,0))
    if yellow_mid_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(height*2/3, yellow_mid_line[0], yellow_mid_line[1]), height*2/3, calculate_x_from_y(height*0.5, yellow_mid_line[0], yellow_mid_line[1]), height*0.5),),), line_color=(255,153,51))
    if yellow_top_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(height*0.5, yellow_top_line[0], yellow_top_line[1]), height*0.5, calculate_x_from_y(height*1/3, yellow_top_line[0], yellow_top_line[1]), height*1/3),),), line_color=(255,204,153))

    if blue_bottom_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(height, blue_bottom_line[0], blue_bottom_line[1]), height, calculate_x_from_y(height*2/3, blue_bottom_line[0], blue_bottom_line[1]), height*2/3),),), line_color=(0,153,153))
    if blue_mid_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(height*2/3, blue_mid_line[0], blue_mid_line[1]), height*2/3, calculate_x_from_y(height*0.5, blue_mid_line[0], blue_mid_line[1]), height*0.5),),), line_color=(0,255,255))
    if blue_top_line is not None:
        lane_lines_image = display_lines(lane_lines_image, (((calculate_x_from_y(height*0.5, blue_top_line[0], blue_top_line[1]), height*0.5, calculate_x_from_y(height*1/3, blue_top_line[0], blue_top_line[1]), height*1/3),),), line_color=(153,255,255))

    line_points = ((yellow_bottom_point, height*5/6), (yellow_mid_point, height*2/3), (yellow_top_point, height*0.5), (blue_bottom_point, height*5/6), (blue_mid_point, height*2/3), (blue_top_point, height*0.5))
    line_points = [point for point in line_points if point[0] is not None]
    lane_lines_image = display_points(lane_lines_image, line_points)

    mid_points = ((average_bottom_point, height*5/6), (average_mid_point, height*2/3), (average_top_point, height*0.5))
    mid_points = [point for point in mid_points if point[0] is not None]
    lane_lines_image = display_points(lane_lines_image, mid_points, point_color=(127,0,255))
    show_image("lane lines", lane_lines_image)

    return (average_bottom_point, average_mid_point, average_top_point), lane_lines_image

def calculate_x_from_y(y, gradient, intercept):
    return (1/gradient)*(y-intercept)

def split_lines(lines, height):
    bottom = []
    middle = []
    top = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        #Make y2 always bottom (higher value) than y1
        if y2 < y1:
            temp = y1
            y1 = y2
            y2 = temp

        if y1 > height * 2/3:
            bottom.append(line)
        if y1 > height * 1/2 and y2 < height * 2/3:
            middle.append(line)
        if y2 < height * 1/2:
            top.append(line)

    return bottom, middle, top

        

def detect_edges(frame,low_thresh,high_thresh):
    # filter for  lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #show_image("hsv", hsv)
    mask = cv2.inRange(hsv, low_thresh, high_thresh)
    #show_image("filtered colour", mask)

    # detect edges
    #edges = cv2.Canny(mask, 200, 400)

    return mask



def region_of_interest(canny,polygon):
    height, width = canny.shape
    mask = np.zeros_like(canny)

    cv2.fillPoly(mask, polygon, 255)
    show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=15,
                                    maxLineGap=8)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments

def section_average_slope_intercept(line_segments):
    """
    Same logic as average_slope_intercept but without differentiating between left and right lines
    (as we can split left and right lines by colour)
    """
    slopes = []
    intercepts = []
    num_lines = len(line_segments)
    if num_lines == 0:
        return None
    for line in line_segments:
            x1, y1, x2, y2 = line[0]
            gradient = (y2-y1)/(x2-x1)
            slopes.append(gradient)
            intercepts.append(y1-gradient*x1)
    return (sum(slopes)/num_lines, sum(intercepts)/num_lines)


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
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary  # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines


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

cap=cv2.VideoCapture("test5.mp4")
video_file='test5'
frame_input = File_Inputter()
_thread.start_new_thread(frame_input.next_frame_counter, tuple())
lane_follower = HandCodedLaneFollower()
print("Running...")

def main():
    time.sleep(3)

    LIVE = True
    file = "xyz.bag"
    #pipe, config, profile = setupstream(LIVE, file)

    while cap.isOpened():
    #while (True):

            if frame_input.frame_counter < frame_input.frame_goal:
                _, frame = cap.read()
                frame_input.frame_counter += 1
            
            #frame, depth_frame, frameset = getframes(pipe)
            combo_image = lane_follower.follow_lane(frame)
            cv2.imshow("Road with Lane line", combo_image)

            if cv2.waitKey(25) & 0xff == ord('q'):
                cv2.destroyAllWindows()
                break
    return


if __name__ == '__main__':
    # do main stuff
    main()
