#!/usr/bin/env python-snakes

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import os
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

import freenect
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from scripts import frame_convert2

PUBLISH_RATE = 30 # Publish Rate in Hz

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down camera node...")

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
    freenect.set_tilt_degs(dev, 20)
    freenect.close_device(dev)

    r = rospy.Rate(PUBLISH_RATE)
    cv_bridge = CvBridge()

    video_pub = rospy.Publisher('/camera/video', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth', Image, queue_size=10)

    # Uses a different encoding for april tags
    camera_image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)

    # Kinect manually calibrated using this http://people.csail.mit.edu/peterkty/teaching/Lab4Handout_Fall_2016.pdf
    camera_info = CameraInfo()
    hd = Header()
    camera_info.height = 480
    camera_info.width = 640
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = [0.16966890693679473, -0.32564392755677646, 0.0014273722857157428, -0.0007780067287402459, 0.0]
    camera_info.K = [522.7790149706918, 0.0, 317.5941836796907, 0.0, 523.5195539902463, 255.18973237498545, 0.0, 0.0, 1.0]
    camera_info.P = [532.5130615234375, 0.0, 317.01325625466416, 0.0, 0.0, 535.176025390625, 255.7121671461573, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    while not rospy.is_shutdown():
        video = get_video()
        depth = get_depth()

        video_msg = cv_bridge.cv2_to_imgmsg(video, encoding="passthrough")
        depth_msg = cv_bridge.cv2_to_imgmsg(depth, encoding="passthrough")
        cam_img_msg = cv_bridge.cv2_to_imgmsg(video, encoding="bgr8")

        camera_info.header = video_msg.header

        video_pub.publish(video_msg)
        depth_pub.publish(depth_msg)
        camera_image_pub.publish(cam_img_msg)
        camera_info_pub.publish(camera_info)

        r.sleep()

if __name__ == "__main__":
    main()
