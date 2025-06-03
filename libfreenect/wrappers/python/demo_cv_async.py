#!/usr/bin/env python3
import freenect
import cv2
import frame_convert

# Create two named windows for depth and RGB
cv2.namedWindow('Depth')
cv2.namedWindow('RGB')
keep_running = True

def display_depth(dev, data, timestamp):
    global keep_running
    # frame_convert.pretty_depth_cv(data) returns a NumPy array
    depth_img = frame_convert.pretty_depth_cv(data)
    cv2.imshow('Depth', depth_img)
    if cv2.waitKey(10) == 27:  # ESC pressed?
        keep_running = False

def display_rgb(dev, data, timestamp):
    global keep_running
    # frame_convert.video_cv(data) returns a NumPy array in RGB order
    rgb_img = frame_convert.video_cv(data)
    # cv2.imshow expects BGR, so convert RGB → BGR
    cv2.imshow('RGB', cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))
    if cv2.waitKey(10) == 27:  # ESC pressed?
        keep_running = False

def body(*args):
    if not keep_running:
        raise freenect.Kill

print('Press ESC in either window to stop')
freenect.runloop(depth=display_depth,
                 video=display_rgb,
                 body=body)
