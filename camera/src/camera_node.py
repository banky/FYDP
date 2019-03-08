#!/usr/bin/env python-snakes

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import os
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import freenect
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from scripts import frame_convert2

PUBLISH_RATE = 60 # Publish Rate in Hz

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")

def get_video():
    """ Get's the current video data from Kinect """
    return frame_convert2.video_cv(freenect.sync_get_video()[0])

def get_depth():
    """ Get's the current depth data from Kinect """
    return freenect.sync_get_depth()[0]

def main():
    rospy.init_node("camera")
    rospy.loginfo("Starting Camera Node")
    rospy.on_shutdown(shutdown_hook)

    # Set initial camera angle
    ctx = freenect.init()
    dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
    freenect.set_tilt_degs(dev, 10)
    freenect.close_device(dev)

    r = rospy.Rate(PUBLISH_RATE)
    cv_bridge = CvBridge()

    video_pub = rospy.Publisher('camera/video', Image, queue_size=10)
    depth_pub = rospy.Publisher('camera/depth', Image, queue_size=10)

    

    while not rospy.is_shutdown():

        video = get_video()
        depth = get_depth()

        video_msg = cv_bridge.cv2_to_imgmsg(video, encoding="passthrough")
        depth_msg = cv_bridge.cv2_to_imgmsg(depth, encoding="passthrough")

        video_pub.publish(video_msg)
        depth_pub.publish(depth_msg)

        r.sleep()

if __name__ == "__main__":
    main()
