#!/usr/bin/env python-snakes

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import os
import sys
import numpy as np

import rospy
from vision.msg import Position
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

# Map dimensions in meters
MAP_HEIGHT = 9
MAP_WIDTH = 3
MAP_UNIT = 0.1 # Size of occupancy grid cells
OBSTACLE_SIZE = 0.5 # Size of all obstacles in meters

PUBLISH_RATE = 10 # Hz

# All cells start as uninitialized at first
sandbox = np.zeros((int(MAP_WIDTH / MAP_UNIT), 
                   int(MAP_HEIGHT / MAP_UNIT)))

curr_pose = PoseStamped()

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down camera node...")

def pose_callback(pose_stamped):
    """ Callback when we receive a new pose estimate """

    global curr_pose
    curr_pose = pose_stamped

def get_xy_of_object(ob):
    position = curr_pose.pose.position
    euler = euler_from_quaternion([curr_pose.pose.orientation.x,
                                   curr_pose.pose.orientation.y,
                                   curr_pose.pose.orientation.z,
                                   curr_pose.pose.orientation.w],
                                  axes='sxyz')

    robot_range = math.sqrt(position.x**2 + position.y**2)
    # robot_roll = euler[0]
    # robot_pitch = euler[1]
    robot_yaw = euler[2]

    obst_range = ob.range
    obst_bearing = ob.bearing

    # cosine law http://hyperphysics.phy-astr.gsu.edu/hbase/lcos.html
    C = math.radians(180 - obst_bearing)
    world_range = math.sqrt(robot_range**2 + obst_range**2
                            - 2 * robot_range * obst_range
                            * math.cos(C))
    world_theta = robot_yaw + obst_bearing

    world_x = world_range * math.cos(world_theta)
    world_y = world_range * math.sin(world_theta)

    return (world_x, world_y)

def garbage_callback(garbage):
    """ Callback when we identify some garbage """
    x, y = get_xy_of_object(garbage)

def obstacle_callback(obstacle):
    """ callback when we identify some obstacles """

    obstacle_unit = OBSTACLE_SIZE / MAP_UNIT
    x, y = get_xy_of_object(obstacle)

    # Set obstacle cell and some around to be "occupied"
    sandbox[x - int(obstacle_unit/2) : x + int(obstacle_unit/2),
            y - int(obstacle_unit/2) : y + int(obstacle_unit/2)] = 1

def main():
    rospy.init_node("path_planning")
    rospy.loginfo("Starting Path Planning Node")
    rospy.on_shutdown(shutdown_hook)

    r = rospy.Rate(PUBLISH_RATE)

    rospy.Subscriber('/vision/garbage', Position, garbage_callback)
    rospy.Subscriber('/vision/obstacles', Position, obstacle_callback)
    rospy.Subscriber('/localization/pose', PoseStamped, pose_callback)

    map_dims = sandbox.shape
    num_milestones = 10
    np.random.seed(69)

    # Random x/y positions of milestones
    milestones = (np.random.rand(num_milestones, 2)
                  * np.array(map_dims)).astype(int)

    # Remove milestones that collide with obstacles


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass