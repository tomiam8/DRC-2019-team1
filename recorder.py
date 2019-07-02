import cv2, time, os
import numpy as np
import pyrealsense2 as rs
import cam
pipe, _, _ =cam.setupstream(True, None)

counter = 0
path = "Recording " + str(time.ctime())
os.mkdir(path)
while True:
    time.sleep(0.5)
    color_image, _, _ = cam.getframes(pipe)
    cv2.imshow("Image", color_image)
    cv2.imwrite(path + "/" + str(counter) + ".jpg", color_image)
    cv2.waitKey(10)
    counter += 1
