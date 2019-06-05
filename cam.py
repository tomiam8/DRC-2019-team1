#! /usr/bin/python3
import cv2 # state of the art computer vision algorithms library
import numpy as np # fundamental package for scientific computing
import matplotlib.pyplot as plt # 2D plotting library producing publication quality figures
import pyrealsense2 as rs # Intel RealSense cross-platform open-source API


#--------------------Sets up a pipeline to the realsense or the bagfile--------
# Inputs:
# live - True streams live data from RealSense
#      - False streams from a bagfile
# file - string, location of a bagfile
def setupstream(live, file):

    print("Loading Stream")

    if live == True:
        pipe = rs.pipeline()
        config = rs.config()                #1280, 720
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #enable_stream(source, width, height, format, fps)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) #Intel resources say 1280 & 720 is best for the depth calculations, then you want to downsize it later)
        profile = pipe.start(config)

        print("Stream loaded")
        return pipe, config, profile

    else:
        pipe = rs.pipeline()
        config = rs.config()
        config.enable_device_from_file(file)
        profile = pipe.start(config)

        print("Stream loaded")
        return pipe, config, profile

# Read frames from the realsense data pipeline
def getframes(pipe):

    pipe.wait_for_frames()
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    if not depth_frame or not color_frame:
        return None, None
    else:
        return color_image, depth_image, frameset
