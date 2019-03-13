#!/usr/bin/env python-snakes

#   Copyright Beach Cleaning Automated
#
#   Author: Johvonna Murray-Bradshaw

"""
    This module is the driving controller used to control the 
    powertrain and allow the ABCD to follow a predefined straight line.
"""

import numpy as np
import math as m
import rospy
import tf
from controls.msg import TwoPointVector
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3, Point
from powertrain.msg import PowertrainParams

STOP_VELOCITY = 0   #Velocity setting to stop robot
GO_VELOCITY = 1     #Constant robot velocity in m/s when moving
K_SMOOTH = 1.5      #Turn smoothing constant
K_STEER = 1         #Steering constant
DELTA_MAX = 25 * m.pi / 180

traj_start = np.array([0,0])   #Initialize global trajectory start point to origin
traj_end = np.array([0,1000])  #Initialize global trajectory end point to far away on y-axis


def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")

# Return the cross track error, given the start and end point 
# of trajectory line and robot's current position
def get_crosstrack_error(traj_start, traj_end, curr_position):
    traj_vect = traj_end - traj_start
    start_to_x = curr_position - traj_start

    print("traj_vect ", traj_vect)
    print("start_to_x ", start_to_x)

    # Compute the distance to the line as a vector, using the projection
    projection = traj_start + ( (traj_vect.dot(start_to_x)) / (np.linalg.norm(traj_vect)**2)) * traj_vect
    distance = curr_position - projection

    # Conver traj_vect and start_to_x line from 1x2 to 1x3 vectors
    traj_vect_3D = np.array([traj_vect[0],traj_vect[1],0])
    start_to_x_3D = np.array([start_to_x[0],start_to_x[1],0])
    
    cross_product = np.cross(traj_vect_3D, start_to_x_3D)

    #Determine if robot is above or below the trajectory line
    pos_neg = 1
    if (cross_product[2] < 0):
        pos_neg = -1

    crosstrack_error = np.linalg.norm(distance) * pos_neg
    return(crosstrack_error)


# Bind any angle in radians between [-pi to pi]
def angleWrap(angle):
    while (angle < -m.pi or angle > m.pi):
        if (angle < -m.pi):
            angle = angle + 2 * m.pi
        else:
            angle = angle - 2 * m.pi
    return (angle)


def pose_callback(data, powertrain_cmds_pub):
    curr_position = np.array([data.pose.position.x , data.pose.position.y])
    print("current position:", curr_position)

    #Convert orientation from quaternions to euler angles
    e_orientation = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    print("euler_orientation", e_orientation)

    crosstrack_error = get_crosstrack_error(traj_start, traj_end, curr_position)
    print("crosstrackerror", crosstrack_error)

    # Determine trajectory angle
    traj_angle = np.arctan2(traj_end[1] - traj_start[1], traj_end[0] - traj_start[0])
    
    # Calculate steering angle (delta)
    print("orientation ", e_orientation[2])
    wrapped = angleWrap(e_orientation[2] - traj_angle)
    print("wrapped", wrapped)
    arctan = np.arctan2(crosstrack_error, K_SMOOTH * GO_VELOCITY)
    print("arctan :" , arctan)

    delta = K_STEER * (angleWrap(e_orientation[2] - traj_angle) + np.arctan2(crosstrack_error, K_SMOOTH * GO_VELOCITY))

    if (delta > DELTA_MAX or delta < -DELTA_MAX):
        if (delta > DELTA_MAX):
            delta = DELTA_MAX
        else:
            delta = -DELTA_MAX
    print("delta: " , delta)

    # Publish powertrain commands
    pp_msg = PowertrainParams()
    pp_msg.speed = GO_VELOCITY
    pp_msg.delta = delta

    powertrain_cmds_pub.publish(pp_msg)

    
def trajectory_callback(data, powertrain_cmds_pub):
    global traj_start
    global traj_end
    
    # Update the start and end points of the trajectory vector
    traj_start = np.array([data.start_point.x, data.start_point.y])
    traj_end = np.array([data.end_point.x, data.end_point.y])

    #STOP Condition: 
    #   When start and end point of trajectory vector are equal
    if (traj_start[0] == traj_end[0] and traj_start[1] == traj_end[1]): #TO-DO: ADD TOLERANCE??????????
        pp_msg = PowertrainParams()
        pp_msg.speed = STOP_VELOCITY
        pp_msg.delta = 0

        #Publish stop condition to powertrain command topic
        powertrain_cmds_pub.publish(pp_msg)


def main():
    rospy.init_node("controls")
    rospy.loginfo("Starting Controls Node")
    rospy.on_shutdown(shutdown_hook)

    #Publish commands for power train to this topic
    powertrain_cmds_pub = rospy.Publisher('powertrain/cmd', PowertrainParams, queue_size=10)
    
    #Subscribe to Pose node for position and orientation data
    rospy.Subscriber('localization/pose', PoseStamped, pose_callback, callback_args=powertrain_cmds_pub)

    #Subscribe to trajectory node for path to follow
    rospy.Subscriber('path_planning/trajectory', TwoPointVector, trajectory_callback, callback_args=powertrain_cmds_pub)

    rospy.spin()


if __name__ == "__main__":
    main()
