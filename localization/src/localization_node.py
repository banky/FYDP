#!/usr/bin/env python-snakes

import os
import sys
from math import sqrt
import rospy
import yaml

import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped

from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray

BASE_DIR = os.path.join(os.path.dirname(__file__), '..')
sys.path.append(BASE_DIR)

YAML_FILE = os.path.join(BASE_DIR, 'config/tags_on_map.yaml')
MAX_WORLD_SIZE = 10 # Assume the world is at most 10 meters long
PUBLISH_RATE = 10 # Publish Rate in Hz

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down localization node...")

def distance(pose):
    x = pose.pose.pose.position.x   # lol these ros definitions
    y = pose.pose.pose.position.y
    z = pose.pose.pose.position.z

    dist = sqrt(x*x + y*y + z*z)
    return dist

curr_pose = PoseStamped()

def tag_callback(data, listener):
    """ Callback when we receive new tag information """

    tags = data
    global curr_pose
    
    # No detected tags
    if len(tags.detections) == 0:
        return
    
    detections = tags.detections
    min_dist = MAX_WORLD_SIZE
    min_detection = AprilTagDetection()

    # Use minimum distance to calculate position.
    # The closer we are, the more accurate the estimate
    for detection in detections:
        dist = distance(detection.pose)
        if dist < min_dist:
            min_dist = dist
            min_detection = detection

    tag_name = 'tag_' + str(min_detection.id[0])

    # Block if our transforms don't exist yet
    listener.waitForTransform(tag_name, "camera", rospy.Time(0), rospy.Duration(3))
    listener.waitForTransform("map", tag_name, rospy.Time(0), rospy.Duration(3))

    # In the camera frame, camera is at (0,0,0)
    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "camera"
    camera_pose.pose.position.x = 0
    camera_pose.pose.position.y = 0
    camera_pose.pose.position.z = 0

    p_in_tag = listener.transformPose(tag_name, camera_pose)
    p_in_map = listener.transformPose("map", p_in_tag)

    curr_pose.pose.position = p_in_map.pose.position
    curr_pose.header.frame_id = "map"
    curr_pose.header.stamp = rospy.Time.now()

def orientation_callback(data):
    """ Callback when we receive new orientation information """

    global curr_pose

    qx = data.x
    qy = data.y
    qz = data.z
    qw = data.w
    orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    curr_pose.pose.orientation = orientation
    curr_pose.header.frame_id = "map"
    curr_pose.header.stamp = rospy.Time.now()


def main():
    rospy.init_node("localization")
    rospy.loginfo("Starting Localization Node")
    rospy.on_shutdown(shutdown_hook)

    with open(YAML_FILE, 'r') as f_in:
        yaml_data = (yaml.load(f_in))
        tag_transforms = yaml_data['tags']

    r = rospy.Rate(PUBLISH_RATE)
    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback, callback_args=listener, queue_size=1)
    rospy.Subscriber('/imu/orientation', Quaternion, orientation_callback, queue_size=1)

    pose_publisher = rospy.Publisher('/localization/pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():

        # Broadcast the transforms describing tag positions on map.
        # Ideally this should be done in another node but we have two days till symposium
        for tag_transform in tag_transforms:
            stamped_t = TransformStamped()
            frame_id = "map"
            child_frame_id = tag_transform['name']
            x = tag_transform['x']
            y = tag_transform['y']
            z = tag_transform['z']
            q = quaternion_from_euler(tag_transform['roll'], tag_transform['pitch'], tag_transform['yaw'])
            qx = q[0]
            qy = q[1]
            qz = q[2]
            qw = q[3]

            broadcaster.sendTransform((x, y, z),
                                      (qx, qy, qz, qw),
                                      rospy.Time.now(),
                                      child_frame_id,
                                      frame_id)

        pose_publisher.publish(curr_pose)

        r.sleep()

if __name__ == "__main__":
    main()