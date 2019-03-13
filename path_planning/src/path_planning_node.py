#!/usr/bin/env python-snakes

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

import os
import sys
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Point
from vision.msg import Position
from controls.msg import TwoPointVector
from tf.transformations import euler_from_quaternion
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from scripts.nearest_neighbor import get_all_k_nearest_neighbours, remove_invalid_neighbours, generate_distance_graph
from scripts.dijkstra import meta_dijkstra

# Map dimensions in meters
MAP_HEIGHT = 9
MAP_WIDTH = 2.7
MAP_UNIT = 0.1 # Size of occupancy grid cells
OBSTACLE_SIZE = 0.5 # Size of all obstacles in meters
MIN_GARBAGE_SEPARATION = 1 # Minimum distance between garbage pieces (m)
STOP_DIST = 0.3 # Distance from us to milestone to switch to next

PUBLISH_RATE = 2 # Hz

# All cells start as uninitialized at first
sandbox = np.zeros((int(MAP_WIDTH / MAP_UNIT),
                   int(MAP_HEIGHT / MAP_UNIT)))

curr_pose = PoseStamped()

# List of garbage positions is initially empty. x/y positions
garbages = np.zeros([0, 2])

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down camera node...")

def dist(x1, y1, x2, y2):
    """ Returns euclidian distance between points """

    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def pose_callback(pose_stamped):
    """ Callback when we receive a new pose estimate """

    global curr_pose
    curr_pose = pose_stamped

def meters_to_idx(x_pos, y_pos):
    """ Converts position from meters to index """

    x_idx = min(int(MAP_WIDTH / MAP_UNIT) - 1, max(0, int((x_pos + MAP_WIDTH / 2) / MAP_UNIT)))
    y_idx = min(int(MAP_HEIGHT / MAP_UNIT) - 1, max(0, int(y_pos / MAP_UNIT)))

    return (x_idx, y_idx)

def get_xy_of_object(ob):
    """ Gets the x/y map indices of an object (garbage or obstacles) """

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
                            - (2 * robot_range * obst_range
                            * math.cos(C)))
    world_theta = robot_yaw + obst_bearing

    world_x = world_range * math.cos(world_theta)
    world_y = world_range * math.sin(world_theta)

    return meters_to_idx(world_x, world_y)

def garbage_callback(garbage):
    """ Callback when we identify some garbage """
    global garbages

    x_idx, y_idx = get_xy_of_object(garbage)
    min_separation = MIN_GARBAGE_SEPARATION / MAP_UNIT

    # Check if we already know about this garbage
    for curr_garbage in garbages: # Loop through rows
        if dist(curr_garbage[0], curr_garbage[1], x_idx, y_idx) < min_separation:
            return

    np.concatenate(garbages, [[x_idx, y_idx]], axis=0)

def obstacle_callback(obstacle):
    """ Callback when we identify some obstacle """

    obstacle_unit = OBSTACLE_SIZE / MAP_UNIT
    x_idx, y_idx = get_xy_of_object(obstacle)

    x_idx_min = max(0, x_idx - int(obstacle_unit/2))
    x_idx_max = min(int(MAP_WIDTH / MAP_UNIT) - 1, x_idx + int(obstacle_unit/2))
    y_idx_min = max(0, y_idx - int(obstacle_unit/2))
    y_idx_max = min(int(MAP_HEIGHT / MAP_UNIT) - 1, y_idx + int(obstacle_unit/2))

    # Set obstacle cell and some around to be "occupied"
    sandbox[x_idx_min : x_idx_max, y_idx_min : y_idx_max] = 1

def main():
    rospy.init_node("path_planning")
    rospy.loginfo("Starting Path Planning Node")
    rospy.on_shutdown(shutdown_hook)

    r = rospy.Rate(PUBLISH_RATE)

    rospy.Subscriber('/vision/garbage', Position, garbage_callback, queue_size=10)
    rospy.Subscriber('/vision/obstacles', Position, obstacle_callback, queue_size=10)
    rospy.Subscriber('/localization/pose', PoseStamped, pose_callback, queue_size=1)

    path_pub = rospy.Publisher('/path_planning/trajectory', TwoPointVector, queue_size=10)

    map_dims = sandbox.shape
    num_milestones = 10
    np.random.seed(69)

    # Get the map position of the end position
    end_pos_x = 0
    end_pos_y = MAP_HEIGHT
    end_pos_idx = np.array(meters_to_idx(end_pos_x, end_pos_y))

    # Number of neighbours for each milestone
    # More gives better path but more computations
    K = 3

    # Starting index for list of milestones. Start at the
    # end of randomized milestones
    curr_milestone_idx = num_milestones

    # Trajectory (Passed to controller)
    traj = TwoPointVector()
    traj_start = Point()
    traj_end = Point()

    while not rospy.is_shutdown():
        # Random x/y positions of milestones
        milestones = (np.random.rand(num_milestones, 2)
                      * np.array(map_dims)).astype(int)

        # Remove milestones that collide with obstacles
        milestones = milestones[sandbox[milestones[:, 0], sandbox[:, 1]] == 0]

        # Add current position to list of milestones so we don't try to jump between milestones
        curr_pos_idx_x, curr_pos_idx_y = meters_to_idx(curr_pose.pose.position.x, curr_pose.pose.position.y)
        milestones = np.concatenate((milestones, [[curr_pos_idx_x, curr_pos_idx_y]]), axis=0)

        # Add locations of garbage to list of milestones to hit
        milestones = np.concatenate(milestones, garbages, axis=0)

        # Add end position to milestones
        milestones = np.concatenate(milestones, end_pos_idx, axis=0)

        # Get k nearest neighbors to each milestone
        neighbours = get_all_k_nearest_neighbours(milestones, K)

        # Remove neighbors with edges that have collisions
        neighbours = remove_invalid_neighbours(milestones, neighbours, sandbox)

        # Generate distance graph for eventual path planning
        graph = generate_distance_graph(milestones, neighbours)

        waypoints_for_dijkstra = list(range(num_milestones, np.size(milestones, 0) - 1))

        path_indices, _ = meta_dijkstra(graph, waypoints_for_dijkstra)

        # Check if we should switch to next step
        next_milestone = milestones[path_indices[curr_milestone_idx + 1], :]
        if (dist(curr_pos_idx_x, curr_pos_idx_y,
                 next_milestone[0], next_milestone[1]) <
                 STOP_DIST / MAP_UNIT):

            # Check if we are at the end of the road
            if np.array_equal(next_milestone, end_pos_idx):
                # When controller sees same points, it stops
                traj_start.x = end_pos_x
                traj_start.y = end_pos_y
                traj_end.x = end_pos_x
                traj_end.y = end_pos_y

                traj.start_point = traj_start
                traj.end_point = traj_end

                path_pub.publish(traj)
                rospy.loginfo('Path Planning Completed')

            else:
                curr_milestone_idx = curr_milestone_idx + 1

        curr_milestone = milestones[path_indices[curr_milestone_idx], :]
        next_milestone = milestones[path_indices[curr_milestone_idx + 1], :]

        # TODO: change to real world coordinates for controls
        traj_start.x = curr_milestone[0]
        traj_start.y = curr_milestone[1]
        traj_end.x = next_milestone[0]
        traj_end.y = next_milestone[1]

        traj.start_point = traj_start
        traj.end_point = traj_end

        path_pub.publish(traj)

        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass