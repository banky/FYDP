# This program finds the HSV values from the centre pixel from a camera feed.
# uses a red-dot reticle to find the object to extract colour components from.

import freenect
import numpy as np
import cv2
import frame_convert2
import time

keep_running = True
cv2.namedWindow('RGB')


def depth(dev, data, timestamp):
    # global keep_running
    # cv2.imshow('Depth', frame_convert2.pretty_depth_cv(data))
    # if cv2.waitKey(10) == 27:
    #     keep_running = False
    pass


def rgb(dev, data, timestamp):
    global keep_running

    frame = frame_convert2.video_cv(data)
    _target = frame.copy()

    # find the horizontal centre pixel
    _center1 = np.uint16(np.round(frame.shape[1]/2))
    # find the vertical centre pixel
    _center2 = np.uint16(np.round(frame.shape[0]/2))

    cv2.circle(_target, (_center1, _center2), 2, (0, 0, 255), 3)
    # apply a red dot to the centre of the frame so the user can target the object to find HSV components from.

    cv2.imshow('RGB', _target)
    # show targeting frame in a window

    # Convert to HSV colourspace
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # find the Hue, Saturation, Value components of the frame
    CH_H, CH_S, CH_V = cv2.split(frame)

    if cv2.waitKey(10) == 27:
        # find Hue value of middle pixel
        pxl_H = CH_H[int(frame.shape[0]/2), int(frame.shape[1]/2)]
        # find Saturation value of middle pixel
        pxl_S = CH_S[int(frame.shape[0]/2), int(frame.shape[1]/2)]
        # find intensity value of middle pixel
        pxl_V = CH_V[int(frame.shape[0]/2), int(frame.shape[1]/2)]

        print("Hue", pxl_H)
        print("Saturation", pxl_S)
        print("Intensity", pxl_V)

        keep_running = False


def body(*args):
    if not keep_running:
        raise freenect.Kill


print('Press ESC in window to stop')
freenect.runloop(depth=depth,
                 video=rgb,
                 body=body)

