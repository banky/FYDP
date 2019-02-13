#!/usr/bin/env python

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import sys
import os.path
import rospy
import cv2
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# from ..scripts import object_locator
import scripts.object_locator as object_locator
import scripts.frame_convert2 as frame_convert2
from vision.msg import Position

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")

if __name__ == "__main__":
    # init ros
    rospy.init_node("object_locator")
    rospy.loginfo("Starting Vision Node")
    rospy.on_shutdown(shutdown_hook)

    cv2.namedWindow('Garbage Frame')
    cv2.namedWindow('Obstacle Frame')
    # cv2.namedWindow('Depth')

    object_locator.init()

    print('Press ESC in window to stop')

    PUBLISH_RATE = 10 # Publish Rate in Hz

    garbage_pub = rospy.Publisher('vision/garbage', Position, queue_size=10)
    obstacle_pub = rospy.Publisher('vision/obstacles', Position, queue_size=10)

    r = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():

        depth = object_locator.get_depth()
        video = object_locator.get_video()
        depth_frame = frame_convert2.pretty_depth_cv(depth)
        # cv2.imshow('Depth', depth_frame)
        # cv2.imshow('RGB', video)

        # cv2.imshow("RGB", video)
        if cv2.waitKey(10) == 27: # Wait until ESC is pressed
            break
        
        garbages = object_locator.find_garbage(video, depth)
        for garbage in garbages:
            garbage_pub.publish(garbage.dist, garbage.theta)

        obstacles = object_locator.find_obstacles(video, depth)
        for obstacle in obstacles:
            obstacle_pub.publish(obstacle.dist, obstacle.theta)

        r.sleep()
