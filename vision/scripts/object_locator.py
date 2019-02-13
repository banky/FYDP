# Copyright Beach Cleaning Automated
# 
# This file is used to locate objects of interest in in view of 
# a connected kinect and determine how far they are


import numpy as np
import cv2
import frame_convert2
import freenect

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RESOLUTION = 10 # Number of pixels per degree Kinect uses
FRAME_MAX_DEPTH = 2047

class Point():
    """ Point describes locations of objects of interest in video """
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Feature():
    """ Describes the location of interesting objects in our vicinity
    (both garbage and obstacles) Positions are described using angular
    notation (distance and angle from kinect reference frame) """

    def __init__ (self, dist, theta, is_garbage):
        self.dist = dist
        self.theta = theta
        self.is_garbage = is_garbage

def depth_2_feet(depth):
    """ Returns the depth measured by the Kinect in meters """
    return 4945422 + (2.258474 - 4945422)/(1 + (depth/5031.15)**8.170714)

def depth_2_metres(depth):
    """ Converts a value in feet to metres """
    return 0.3048 * depth_2_feet(depth)

def get_depth():
    """ Get's the current depth data from Kinect """
    return freenect.sync_get_depth()[0]


def get_video():
    """ Get's the current video data from Kinect """
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

def find_rects(data, is_garbage):

    points = []

    data = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
    x, y, w, h = cv2.boundingRect(data)
    cv2.rectangle(data, (x, y), (x+w, y+h), (0, 255, 0), 2)
    point = Point(x + w/2, y + h/2)

    if (w > 5 and h > 5):
        points.append(point)

    if is_garbage:
        cv2.imshow('Garbage Frame', data)
    else:
        cv2.imshow('Obstacle Frame', data)

    return points

def find_circles(data, is_garbage):
    """ Pass in filtered data and whether or not the filter is for
    garbage or obstacles """

    points = []

    # convert to greyscale colourspace
    frame = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)

    # (optional: smooth image to reduce detection errors, must be an odd number)
    frame = cv2.GaussianBlur(frame, (9, 9), 0)

    circles = cv2.HoughCircles(
        frame, cv2.HOUGH_GRADIENT, 1, 200, param1=50, param2=15, minRadius=1, maxRadius=50)
    # this function will need some tuning and trial & error, see the opencv documentation

    if circles is not None:
        circles = np.uint16(np.around(circles))

        # loop through all the circles detected
        for i in range(len(circles[0])):
            point = Point(circles[0][i][0], circles[0][i][1])
            points.append(point)

            # draw the outer circle. Red circle on garbage, green on other objects
            cv2.circle(data, (circles[0][i][0], circles[0][i][1]),
                       circles[0][i][2], (0, 255 - 255*is_garbage, 255*is_garbage), 2)
            # draw the centre of the circle
            cv2.circle(data, (circles[0][i][0], circles[0][i][1]), 2, (0, 0, 255), 3)
            # see documentation for draw functions

    # show frame in a separate window
    if is_garbage:
        cv2.imshow('Garbage Frame', data)
    else:
        cv2.imshow('Obstacle Frame', data)

    return points

def filter_image(data, lower_limit, upper_limit):
    """ Filters input image by lower and upper limit HSV values """

    frame = cv2.cvtColor(data, cv2.COLOR_BGR2HSV)

    # Cast to type that the opencv can process consistently
    lower = np.array(lower_limit, dtype="uint8")
    upper = np.array(upper_limit, dtype="uint8")

    # find boolean mask for filtering
    mask = cv2.inRange(frame, lower, upper)

    # bitwise-and original frame using the mask above to find the filtered frame.
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Need two steps here bc we can't go direct from hsv to gray
    frame = (frame > 0).astype(int) # select for all values that are not black
    frame = np.multiply(frame, 255) # Scale all values that are not black up to white
    frame = np.array(frame, dtype=np.uint8) # Need uint8 type

    # Filter image with morphological operators
    er_kernel = np.ones((3, 3), np.uint8)
    frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, er_kernel, iterations=1)
    
    # frame = cv2.erode(frame, er_kernel, iterations=1)

    dil_kernel = np.ones((3, 3), np.uint8)
    frame = cv2.dilate(frame, dil_kernel, iterations=10)

    # convert filtered frame back to BGR colourspace (opencv likes BGR rather than RGB)
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

def find_obstacles(video, depth):
    """ Get the feature location of obstacles in frame """

    obstacles = []

    # Setup lower threshold masks for filtering (H,S,V)
    # Use higher than 180 hue to wrap around to 0
    lower_red = np.array([175, 250, 20])
    upper_red = np.array([185, 255, 200])
    obstacle_frame = filter_image(video, lower_red, upper_red)
    obstacle_locs = find_rects(obstacle_frame, False)

    if obstacle_locs is None:
        return obstacles

    for obstacle_loc in obstacle_locs:
        theta = obstacle_loc.x - FRAME_WIDTH / 2
        theta = theta / FRAME_RESOLUTION
        dist = depth[obstacle_loc.y][obstacle_loc.x]
        dist = depth_2_metres(dist)
        feature = Feature(dist, theta, False)

        if dist < FRAME_MAX_DEPTH:  # Only add feature if it is in an acceptable distance away
            obstacles.append(feature)

    return obstacles

def find_garbage(video, depth):
    """ Get the feature location of garbage in the frame """

    garbages = []

    lower_green = np.array([30, 250, 20])
    upper_green = np.array([40, 255, 200])
    garbage_frame = filter_image(video, lower_green, upper_green)
    # garbage_locs = find_circles(garbage_frame, True)
    garbage_locs = find_rects(garbage_frame, True)

    if garbage_locs is None:
        return garbages

    for garbage_loc in garbage_locs:
        theta = garbage_loc.x - FRAME_WIDTH / 2
        theta = theta / FRAME_RESOLUTION
        dist = depth[garbage_loc.y][garbage_loc.x]
        dist = depth_2_metres(dist)
        feature = Feature(dist, theta, True)

        if dist < FRAME_MAX_DEPTH:  # Only add feature if it is in an acceptable distance away
            garbages.append(feature)

    return garbages

def init():
    ctx = freenect.init()
    dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
    freenect.set_tilt_degs(dev, 0)
    freenect.close_device(dev)
