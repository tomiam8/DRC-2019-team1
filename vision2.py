import pyrealsense2 as rs
import numpy as np
import cv2, sys, time, _thread

################ Setup constants ################
#Thresholds: Yellow
thresh_yellow_low = (19,67,23)
thresh_yellow_high = (94,206,255)

#Thresholds: blue
thresh_blue_low = (31,44,88)
thresh_blue_high = (146, 249, 188)

bin_nums = 20
fraction = int(720 / bin_nums)
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

# Setup debug stuff (i.e run headerless if not debug)
if len(sys.argv) > 1:
    debug = True
else:
    debug = False
debug=True

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


def process_image(color_frame, thresh_yellow_low, thresh_yellow_high, thresh_blue_low, thresh_blue_high, debug):
    # Image processing (resize, hsv, blur, add border, threshold, blur, Canny, contour finding)
    resize_img = cv2.resize(np.asanyarray(color_frame), (320, 180), interpolation=cv2.INTER_NEAREST)
    hsv_img = cv2.cvtColor(resize_img, cv2.COLOR_BGR2HSV)
    blur_img = cv2.GaussianBlur(hsv_img, (5, 5), 0)
    bordered_img = cv2.copyMakeBorder(blur_img, 3, 3, 3, 3, cv2.BORDER_CONSTANT, (0, 0, 0))

    threshold_yellow_img = cv2.inRange(bordered_img, thresh_yellow_low, thresh_yellow_high)
    dilated_yellow_img = cv2.dilate(threshold_yellow_img, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)), 1)
    edges_yellow = cv2.Canny(dilated_yellow_img, 75, 125)  # TODO: Find proper values
    (_, contours_yellow, _) = cv2.findContours(edges_yellow, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    threshold_blue_img = cv2.inRange(bordered_img, thresh_blue_low, thresh_blue_high)
    dilated_blue_img = cv2.dilate(threshold_blue_img, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), 1)
    edges_blue = cv2.Canny(dilated_blue_img, 75, 125)
    (_, contours_blue, _) = cv2.findContours(edges_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    # Contour filtering
    filtered_contours_yellow = filter_contours(contours_yellow, np.copy(cv2.cvtColor(bordered_img, cv2.COLOR_HSV2BGR))) #TODO remove sending the contour for debug
    filtered_contours_blue = filter_contours(contours_blue, np.copy(cv2.cvtColor(bordered_img, cv2.COLOR_HSV2BGR)))

    # Midpoint stuff
    midpoints_yellow = bin_contours(filtered_contours_yellow)
    midpoints_blue = bin_contours(filtered_contours_blue)

    # Midpoints of midpoints
    bin_midpoints = [[] for x in range(bin_nums)]
    for midpoint in midpoints_yellow:
        if midpoint != []:
            bin_midpoints[int(midpoint[1] / fraction)].append((midpoint[0], midpoint[1]))
    for midpoint in midpoints_blue:
        if midpoint != []:
            bin_midpoints[int(midpoint[1] / fraction)].append((midpoint[0], midpoint[1]))

    midpoints_final = []
    for midpoint in bin_midpoints:
        if len(midpoint) == 2:
            midpoints_final.append((int(sum([point[0] for point in midpoint]) / len(midpoint)),
                                    int(sum([point[1] for point in midpoint]) / len(midpoint))))

    # Debug display stuff
    if debug:
        cv2.imshow("Threshold yellow", threshold_yellow_img)
        cv2.imshow("Threshold blue", threshold_blue_img)
        cv2.imshow("Morph", dilated_blue_img)
        cv2.imshow("Morph yellow", dilated_yellow_img)
        cv2.imshow("Canny yellow", edges_yellow)
        cv2.imshow("Canny", edges_blue)

        drawn = np.copy(cv2.cvtColor(bordered_img, cv2.COLOR_HSV2BGR))
        pre_filtering = np.copy(drawn)
        pre_filtering = cv2.drawContours(pre_filtering, contours_yellow, -1, (255, 0, 0), 2)
        pre_filtering = cv2.drawContours(pre_filtering, contours_blue, -1, (0, 0, 255), 2)
        
        """for contour in contours_yellow:
            cv2.imshow("ewfd", cv2.drawContours(drawn, (contour,), -1, (0, 255, 0), 2))
            print(cv2.arcLength(contour, True))
            cv2.waitKey(0)
            pass
        """

        drawn = cv2.drawContours(drawn, filtered_contours_yellow, -1, (255, 0, 0), 2)
        drawn = cv2.drawContours(drawn, filtered_contours_blue, -1, (0, 0, 255), 2)

        filtered_midpoints_yellow = [x for x in midpoints_yellow if x != []]
        filtered_midpoints_blue = [x for x in midpoints_blue if x != []]
        filtered_midpoints_final = [x for x in midpoints_final if x != []]
        for line in range(len(filtered_midpoints_yellow) - 1):
            drawn = cv2.line(drawn, filtered_midpoints_yellow[line], filtered_midpoints_yellow[line + 1], (255, 255, 0),
                             2)
        for line in range(len(filtered_midpoints_blue) - 1):
            drawn = cv2.line(drawn, filtered_midpoints_blue[line], filtered_midpoints_blue[line + 1], (255, 255, 0), 2)
        for line in range(len(filtered_midpoints_final)-1):
            drawn = cv2.line(drawn, filtered_midpoints_final[line], filtered_midpoints_final[line+1], (0, 255, 0), 2)

        cv2.imshow("Lined", drawn)
        cv2.imshow("Pre filtering", pre_filtering)
        cv2.waitKey(1)

    # Return stuff
    return None


def filter_contours(contours, image): #TODO remove image (is for debug)
    final_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 15 and area < 6000:
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 100 and perimeter < 600:
                rect = cv2.minAreaRect(contour)
                width = min(rect[1])
                height = max(rect[1])
                ratio = width/height
                #print(ratio)
                #print(rect)
                #cv2.imshow("WEfd", cv2.drawContours(np.copy(image), (contour, np.int0(cv2.boxPoints(rect))), -1, (0, 255,0), 2))
                #cv2.waitKey(0)
                if 0.05 < ratio < 0.2: #Aspect ratio
                    if -65 < rect[2] < -40: #Angle
                        #cv2.imshow("WEfd", cv2.drawContours(np.copy(image), (contour, np.int0(cv2.boxPoints(rect))), -1, (0, 255,0), 2))
                        #cv2.waitKey(0)
                        final_contours.append(contour)

    return final_contours


def bin_contours(contours):
    if contours != []:
        bins = [[] for x in range(bin_nums)]
        bin_midpoints = [[] for x in range(bin_nums)]

        for point in contours[0]:
            bins[int(point[0][1] / fraction)].append(point[0])

        for contour in bins:
            contour = cv2.convexHull(np.array(contour, dtype=np.int32).reshape((-1, 1, 2)), False)

            moments = cv2.moments(contour)
            if moments["m00"] != 0:
                bin_midpoints.append((int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])))
        return bin_midpoints
    else:
        return []

camera = Camera()
_thread.start_new_thread(camera.take_pics, tuple())
try:
    while camera.get_color_frame() is None:
        time.sleep(0.1)  # Wait for camera to start
    while True:
        process_image(camera.get_color_frame(), threshold_yellow.get_low(), threshold_yellow.get_high(),
                      threshold_blue.get_low(), threshold_blue.get_high(), debug)
except Exception as e:
    camera.stop()
    raise e
