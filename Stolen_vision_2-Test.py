import cv2
import numpy as np
import logging
import math
import datetime
import threading
import _thread
import time
import serial

import pyrealsense2 as rs
import sys

from cam import setupstream
from cam import getframes


_SHOW_IMAGE = True

#CONSTANTS
#Threshold: yellow
thresh_yellow_low = (19,67,23)
thresh_yellow_high = (94,206,255)

#Thresholds: blue
thresh_blue_low = (31,44,88)
thresh_blue_high = (146, 249, 188)

class Arduino:
    def __init__(self):
        self.connection = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.speed = 100
        self.angle = 90

    def update_speed(self, speed):
        if speed > 180:
            speed = 180
        elif speed < 0:
            speed = 0
        self.speed = int(speed)

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
            #self.connection.write(f"M{self.speed:03d}".encode())
            self.connection.write("M100".encode())
            #print(f"SENT SPEED: M{self.speed:03d} for speed {self.speed}")
            time.sleep(0.04)
            self.connection.write(f"S{self.angle:03d}".encode())
            print(f"SENT ANGLE: S{self.angle:03d} for angle {self.angle}")
            time.sleep(0.04)

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
        #self.arduino = Arduino()
        #self.arduino_thread = threading.Thread(target=self.arduino.run)
        #self.arduino_thread.start()

    def follow_lane(self, frame):
        # Main entry point of the lane follower
        #show_image("orig", frame)

        lane_lines, frame = detect_lane(frame)
        final_frame = self.steer(frame, lane_lines)

        return final_frame

    def steer(self, frame, lane_lines):
        logging.debug('steering...')
        if len(lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
            return frame

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle,
                                                            len(lane_lines))

        #self.arduino.update_angle(self.curr_steering_angle)
        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)

        return curr_heading_image


############################
# Frame processing steps
############################
def detect_lane(frame):
    logging.debug('detecting lane lines...')

    yellow_edges = detect_edges(frame,thresh_yellow_low,thresh_yellow_high)
    blue_edges = detect_edges(frame,thresh_blue_low,thresh_blue_high)

    #show_image('yellow edges', yellow_edges)
    #show_image('blue edges', blue_edges)

    height,width=yellow_edges.shape

    # only focus bottom right half of the screen
    yellow_polygon= np.array([[
        (width/2, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (width/2, height)
    ]], np.int32)

    #only focus on the bottom left helf of the screen
    blue_polygon= np.array([[
        (0, 100),
        (width, 100),
        (width, height),
        (0, height)
    ]], np.int32)

    yellow_cropped_edges = region_of_interest(yellow_edges,blue_polygon)
    blue_cropped_edges = region_of_interest(blue_edges,blue_polygon)

    show_image('yellow edges cropped', yellow_cropped_edges)
    show_image('blue edges cropped', blue_cropped_edges)

    yellow_line_segments = detect_line_segments(yellow_cropped_edges)
    blue_line_segments = detect_line_segments(blue_cropped_edges)

    #line_segment_image = display_lines(frame, yellow_line_segments)
    #show_image("yellow line segments", line_segment_image)

    yellow_lane_lines = average_slope_intercept(frame, yellow_line_segments)
    blue_lane_lines = average_slope_intercept(frame, blue_line_segments)

    if len(yellow_lane_lines)>0 and len(blue_lane_lines)>0:
        lane_lines=[blue_lane_lines[0],yellow_lane_lines[0]]
    elif len(yellow_lane_lines)>0:
        lane_lines=[yellow_lane_lines[0]]
    elif len(blue_lane_lines)>0:
        lane_lines=[blue_lane_lines[0]]
    else:
        lane_lines=[]


    lane_lines_image = display_lines(frame, lane_lines)
    #show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image


def detect_edges(frame,low_thresh,high_thresh):
    # filter for  lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #show_image("hsv", hsv)
    mask = cv2.inRange(hsv, low_thresh, high_thresh)
    #show_image("filtered colour", mask)

    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges



def region_of_interest(canny,polygon):
    height, width = canny.shape
    mask = np.zeros_like(canny)

    cv2.fillPoly(mask, polygon, 255)
    #show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 20  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=6,
                                    maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments


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
            if slope<0:
                #if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                #if x1 > right_region_boundary and x2 > right_region_boundary:
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
        logging.info('No lane lines detected, move forward')
        return 90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, y1, x2, y2 = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

        # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height-100)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = 90 + angle_to_mid_deg # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines,
                             max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
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
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
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


cap=cv2.VideoCapture("test5.mp4")
video_file='test5'
lane_follower = HandCodedLaneFollower()

# skip first second of video.
for i in range(3):
    _, frame = cap.read()

try:
    while cap.isOpened():
        _, frame = cap.read()
        combo_image = lane_follower.follow_lane(frame)
        cv2.imshow("Road with Lane line", combo_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()