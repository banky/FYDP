#!/usr/bin/env python-snakes

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import sys
import os.path
import rospy
import cv2
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
print(sys.version)

# from ..scripts import object_locator
from scripts import object_locator
import scripts.frame_convert2 as frame_convert2
from vision.msg import Position
from . import object_locator

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")
    # yolo.close_session()

if __name__ == "__main__":
    # init ros
    rospy.init_node("object_locator")
    rospy.loginfo("Starting Vision Node")
    rospy.on_shutdown(shutdown_hook)

    yolo = object_locator.YOLO()

    # cv2.namedWindow('Garbage Frame')
    # cv2.namedWindow('Obstacle Frame')
    # cv2.namedWindow('Depth')


    # print('Press ESC in window to stop')

    PUBLISH_RATE = 10 # Publish Rate in Hz

    garbage_pub = rospy.Publisher('vision/garbage', Position, queue_size=10)
    obstacle_pub = rospy.Publisher('vision/obstacles', Position, queue_size=10)

    r = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():

        # depth = object_locator.get_depth()
        # video = object_locator.get_video()
        # depth_frame = frame_convert2.pretty_depth_cv(depth)
        # cv2.imshow('Depth', depth_frame)
        # cv2.imshow('RGB', video)

        # cv2.imshow("RGB", video)
        if cv2.waitKey(10) == 27: # Wait until ESC is pressed
            break
        
        out_boxes, out_scores, out_classes = object_locator.update(yolo)

        garbages = object_locator.find_garbage(yolo, out_boxes, out_scores, out_classes)
        for garbage in garbages:
            garbage_pub.publish(garbage.dist, garbage.theta)

        obstacles = object_locator.find_obstacles(yolo, out_boxes, out_scores, out_classes)
        for obstacle in obstacles:
            obstacle_pub.publish(obstacle.dist, obstacle.theta)

        # r.sleep()
