#!/usr/bin/env python

#   Copyright Beach Cleaning Automated
#
#   Author: Johvonna Murray-Bradshaw

"""
    This module is the driving controller used to control the 
    powertrain and allow the ABCD to follow a predefined straight line.
"""

import numpy
import rospy
import tf
from controls.msg import TwoPointVector
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3, Point

VELOCITY = 1 # Robot velocity in m/s
start_point = 0 #Initialize global start_point variable
end_point = 0 #Initialize global end_point variable

#Call back function for everytime a new pose is read
def pose_callback(data, args):
    curr_position = data.position
    
    #Convert orientation from quaternions to euler angles
    q_orientation = data.orientation
    e_orientation = tf.transformations.euler_from_quaternion(q_orientation, 'sxyz')
    
    #
    traj_to_x_angle = arctan2(end_point.x - start_point.x , end_point.y - start_point.y)
    
    #steer_angle = 

    
    

#Vector
def trajectory_callback(data, args):
    start_point = data.start_point
    end_point = data.end_point

'''
traj_angle = atan2(end_point(2) - start_point(2), end_point(1) - start_point(1));
[crosstrack_error, next_point] = distanceToLineSegment(start_point,end_point,mu(1:2)');

% Calculate steering angle
curr_delta = angleWrap((mu(3) - traj_angle))+ atan2(crosstrack_error,K_smoothing*velocity);
delta = max(-delta_max,min(delta_max, K_steer*curr_delta));
w = velocity;'''


#Subscribe to Pose node for position and orientation data
rospy.Subscriber('localization/pose', PoseStamped, pose_callback)

#Subscribe to ... node for path to follow (line)
rospy.Subscriber('path_planning/trajectory', TwoPointVector, trajectory_callback)

#Publish commands for power train to this topic
rospy.Publisher('powertrain/cmd', PowertrainParams, queue_size=10)
