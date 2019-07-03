import pyrealsense2 as rs
import numpy as np
import cv2, sys, time, _thread
from cam import setupstream
from cam import getframes

################ Setup constants ################
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
thresh_green_low = (40, 23, 135)
thresh_green_high = (69, 99, 198)

class Threshold_manager:
    def __init__(self, low, high):
        self.threshold_low = low
        self.threshold_high = high

    def get_low(self):
        return self.threshold_low

    def get_high(self):
        return self.threshold_high


class Threshold_manager_debug:
    def __init__(self, low_default=(0, 0, 0), high_default=(180, 255, 255)):
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


random_colours = [(255, 0, 0), (255, 255, 0), (0, 255,), (0, 255, 255), (0, 0, 255), (255, 0, 255)]

threshold_blue = Threshold_manager(thresh_blue_low, thresh_blue_high)
threshold_yellow = Threshold_manager(thresh_yellow_low, thresh_yellow_high)
threshold_purple = Threshold_manager(thresh_purple_low, thresh_purple_high)
threshold_green = Threshold_manager(thresh_green_low, thresh_green_high)

# Setup debug stuff (i.e run headerless if not debug)
if len(sys.argv) > 1:
    debug = True
else:
    debug = False
debug=True

if debug:
    threshold_purple = Threshold_manager_debug(thresh_purple_low, thresh_purple_high)
    threshold_green = Threshold_manager_debug(thresh_green_low, thresh_green_high)
    threshold_blue = Threshold_manager_debug(thresh_blue_low, thresh_blue_high)
    threshold_yellow = Threshold_manager_debug(thresh_yellow_low, thresh_yellow_high)

    cv2.namedWindow("Threshold purple")
    cv2.namedWindow("Threshold green")
    cv2.namedWindow("Threshold blue")
    cv2.namedWindow("Threshold yellow")

    cv2.createTrackbar("low hue", "Threshold blue", threshold_blue.lh, 180, threshold_blue.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold blue", threshold_blue.uh, 180, threshold_blue.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold blue", threshold_blue.ls, 255, threshold_blue.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold blue", threshold_blue.us, 255, threshold_blue.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold blue", threshold_blue.lv, 255, threshold_blue.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold blue", threshold_blue.uv, 255, threshold_blue.on_high_V_thresh_trackbar)

    cv2.createTrackbar("low hue", "Threshold yellow", threshold_yellow.lh, 180,
                       threshold_yellow.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold yellow", threshold_yellow.uh, 180,
                       threshold_yellow.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold yellow", threshold_yellow.ls, 255,
                       threshold_yellow.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold yellow", threshold_yellow.us, 255,
                       threshold_yellow.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold yellow", threshold_yellow.lv, 255,
                       threshold_yellow.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold yellow", threshold_yellow.uv, 255,
                       threshold_yellow.on_high_V_thresh_trackbar)

    cv2.createTrackbar("low hue", "Threshold purple", threshold_purple.lh, 180, threshold_purple.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold purple", threshold_purple.uh, 180, threshold_purple.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold purple", threshold_purple.ls, 255, threshold_purple.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold purple", threshold_purple.us, 255, threshold_purple.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold purple", threshold_purple.lv, 255, threshold_purple.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold purple", threshold_purple.uv, 255, threshold_purple.on_high_V_thresh_trackbar)

    cv2.createTrackbar("low hue", "Threshold green", threshold_green.lh, 180, threshold_green.on_low_H_thresh_trackbar)
    cv2.createTrackbar("high  hue", "Threshold green", threshold_green.uh, 180, threshold_green.on_high_H_thresh_trackbar)
    cv2.createTrackbar("low sat", "Threshold green", threshold_green.ls, 255, threshold_green.on_low_S_thresh_trackbar)
    cv2.createTrackbar("high  sat", "Threshold green", threshold_green.us, 255, threshold_green.on_high_S_thresh_trackbar)
    cv2.createTrackbar("low val", "Threshold green", threshold_green.lv, 255, threshold_green.on_low_V_thresh_trackbar)
    cv2.createTrackbar("high  val", "Threshold green", threshold_green.uv, 255, threshold_green.on_high_V_thresh_trackbar)


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

def process_image(color_frame, thresh_yellow_low, thresh_yellow_high, thresh_blue_low, thresh_blue_high, thresh_purple_low, thresh_purple_high, thresh_green_low, thresh_green_high, debug):
    # Image processing (resize, hsv, blur, add border, threshold, blur, Canny, contour finding)
    resize_img = cv2.resize(np.asanyarray(color_frame), (320, 180), interpolation=cv2.INTER_NEAREST)
    resize_img = cv2.cvtColor(resize_img, cv2.COLOR_RGB2BGR) #TODO remove
    hsv_img = cv2.cvtColor(resize_img, cv2.COLOR_BGR2HSV)
    blur_img = cv2.GaussianBlur(hsv_img, (5, 5), 0)
    bordered_img = cv2.copyMakeBorder(blur_img, 3, 3, 3, 3, cv2.BORDER_CONSTANT, (0, 0, 0))

    threshold_yellow_img = cv2.inRange(bordered_img, thresh_yellow_low, thresh_yellow_high)
    threshold_blue_img = cv2.inRange(bordered_img, thresh_blue_low, thresh_blue_high)
    threshold_green_img = cv2.inRange(bordered_img, thresh_green_low, thresh_green_high)
    threshold_purple_img = cv2.inRange(bordered_img, thresh_purple_low, thresh_purple_high)

    # Debug display stuff
    if debug:
        cv2.imshow("Threshold yellow", threshold_yellow_img)
        cv2.imshow("Threshold blue", threshold_blue_img)
        cv2.imshow("Threshold green", threshold_green_img)
        cv2.imshow("Threshold purple", threshold_purple_img)
        cv2.imshow("Colour", resize_img)
        cv2.waitKey(1)

    # Return stuff
    return None

class File_Inputter:
    def __init__(self):
        self.frame_counter = 0
        self.frame_goal = 1
        self.prev_increment = 50
    
    def next_frame_counter(self):
        while True:
            leInput = input("Frames to view: ")
            if leInput == "":
                self.frame_goal += self.prev_increment
            else:
                try:
                    self.frame_goal += int(leInput)
                    self.prev_increment = int(leInput)
                except ValueError:
                    pass


frame_input = File_Inputter()
_thread.start_new_thread(frame_input.next_frame_counter, tuple())
LIVE = False
file = "2nd.bag"
pipe, config, profile = setupstream(LIVE, file)
try:
    while True:
        if frame_input.frame_counter < frame_input.frame_goal:
            frame, depth_frame, frameset = getframes(pipe)
            frame_input.frame_counter += 1
        process_image(frame, threshold_yellow.get_low(), threshold_yellow.get_high(),
                            threshold_blue.get_low(), threshold_blue.get_high(), threshold_purple.get_low(), threshold_purple.get_high(), threshold_green.get_low(), threshold_green.get_high(), debug)    
except Exception as e:
    print(e)
finally:
    print("\nYellow low: {}\nYellow high: {}\nBlue low: {}\nBlue high:{}".format(threshold_yellow.get_low(), threshold_yellow.get_high(), threshold_blue.get_low(), threshold_blue.get_high()))
