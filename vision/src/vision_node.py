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
# from scripts import object_locator
from vision.msg import Position
from scripts import object_locator

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")
    # yolo.close_session()

bridge = CvBridge()
video_output = None
depth_output = None

def video_callback(data):
    global video_output
    video_output = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # print(video_output.shape)
        
def depth_callback(data):
    global depth_output
    depth_output = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

def main():
    # init ros
    rospy.init_node("vision")
    rospy.loginfo("Starting Vision Node")
    rospy.on_shutdown(shutdown_hook)

    yolo = object_locator.YOLO()

    PUBLISH_RATE = 10 # Publish Rate in Hz

    garbage_pub = rospy.Publisher('vision/garbage', Position, queue_size=10)
    obstacle_pub = rospy.Publisher('vision/obstacles', Position, queue_size=10)

    # rospy.Subscriber('/camera/video', Image, video_callback, queue_size=1)
    rospy.Subscriber('/tag_detections_image', Image, video_callback, queue_size=1)
    # rospy.Subscriber('/camera/image_rect_color', Image, video_callback, queue_size=1)
    rospy.Subscriber('/camera/depth', Image, depth_callback, queue_size=1)

    r = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():

        if video_output is None or depth_output is None:
            continue
        
        # cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)

        cv2.imshow("Video", video_output) 
        # cv2.imshow("Depth", depth_output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()

        # out_boxes, out_scores, out_classes = object_locator.update(yolo, video_output)

        # garbages = object_locator.find_garbage(yolo, out_boxes, out_scores, out_classes, depth_output)
        # for garbage in garbages:
        #     garbage_pub.publish(garbage.dist, garbage.theta)

        # obstacles = object_locator.find_obstacles(yolo, out_boxes, out_scores, out_classes, depth_output)
        # for obstacle in obstacles:
        #     obstacle_pub.publish(obstacle.dist, obstacle.theta)

        r.sleep()

if __name__ == "__main__":
    main()