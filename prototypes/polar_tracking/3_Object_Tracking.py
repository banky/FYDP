# Prerequisites:

# make sure your camera is recognised and enabled on your system, common USB webcams are usually ok but picam may need some setup.
# goto start>Preferences>Rapberry Pi Configuration>Interfaces (tab) --> Enable camera
# picam may also need further drivers installed, like the picamera library

# Also helpful
# https://www.raspberrypi.org/forums/viewtopic.php?t=176697
# sudo modprobe bcm2835-v4l2

# This code runs on the opencv framework for python, use the bash prompt to download and install.
# I used the SimpleCV tutorial as it was easier to do in one go.

# https://github.com/sightmachine/SimpleCV/blob/develop/doc/HOWTO-Install%20on%20RaspberryPi.rst
# if this fails use "--no-cache-dir" to stop pip error

# https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
# helpful tutorial

import numpy as np
import cv2
# import cv2.cv
import frame_convert2
import freenect

class Point():
    """ Point describes locations of objects of interest in video """
    def __init__(self, x, y):
        self.x = x
        self.y = y


def get_depth():
    """ Get's the current depth data from Kinect """
    return frame_convert2.pretty_depth_cv(freenect.sync_get_depth()[0])


def get_video():
    """ Get's the current video data from Kinect """
    return frame_convert2.video_cv(freenect.sync_get_video()[0])


def get_circle_pos(data):
    """ Get's the x,y location of circles in the current frame """

    frame = frame_convert2.video_cv(data)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # setup lower threshold masks for filtering (H,S,V)
    lower_red = np.array([165, 220, 50])
    # setup upper threshold masks for filtering (H,S,V)
    upper_red = np.array([185, 255, 200])
    # use higher than 180 hue to wrap around to 0

    # cast to type that the opencv can process consistently
    lower = np.array(lower_red, dtype="uint8")
    upper = np.array(upper_red, dtype="uint8")

    _mask = cv2.inRange(frame, lower, upper)  # find boolean mask for filtering
    # bitwise-and original frame using the mask above to find the filtered frame.
    frame = cv2.bitwise_and(frame, frame, mask=_mask)

    # convert filtered frame back to BGR colourspace (opencv likes BGR rather than RGB)
    frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)

    # CIRCLE DETECTION
    # convert to greyscale colourspace
    cimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # (optional: smooth image to reduce detection errors, must be an odd number)
    cimg = cv2.GaussianBlur(cimg, (9, 9), 0)

    circles = cv2.HoughCircles(
        cimg, cv2.HOUGH_GRADIENT, 1, 200, param1=40, param2=15, minRadius=1, maxRadius=50)
    # this function will need some tuning and trial & error, see the opencv documentation

    # if(circles != None): # if there are circle detected in the frame

    point = Point(0, 0)

    if circles is not None:
        circles = np.uint16(np.around(circles))

        # loop through all the circles detected
        # for i in range(len(circles[0])):
        i = 0
        point.x = circles[0][i][0]
        point.y = circles[0][i][1]

        # draw the outer circle
        cv2.circle(frame, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 255, 0), 2)
        # draw the centre of the circle
        cv2.circle(frame, (circles[0][i][0], circles[0][i][1]), 2, (0, 0, 255), 3)
        # see documentation for draw functions
    else:
        point.x = -1
        point.y = -1

    # show frame in a separate window
    cv2.imshow('RGB', frame)
    return point


def main():
    """ Main Function For Program """

    cv2.namedWindow('RGB')

    print('Press ESC in window to stop')

    while True:
        depth = get_depth()
        video = get_video()

        # cv2.imshow("RGB", video)
        if cv2.waitKey(10) == 27:
            break
        circle_loc = get_circle_pos(video)
        print("x: ", circle_loc.x, "y: ", circle_loc.y)


main()
