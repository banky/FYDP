#!/usr/bin/env python

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo and Johvonna Murray-Bradshaw

"""
    This module is used to transfer serial data between the NVIDIA Jetson and Arduino
"""

import serial
import rospy
import numpy
import tf
from vision.msg import Position
from geometry_msgs.msg import Quaternion

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")

# TODO: Move this stuff to path planning and controls. This hack is temporary
def garbage_callback(data, args):
    pass

def obstacle_callback(data, args):
    dist = data.range
    bearing = data.bearing
    ser_local = args[0]

    # If we are this close to an obstacle
    if dist < 1.5:
        ser_local.write('x 97')
    else:
        ser_local.write('x 100')

if __name__ == "__main__":
    rospy.init_node("powertrain")
    rospy.loginfo("Starting Powertrain Node")
    rospy.on_shutdown(shutdown_hook)

    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/ttyACM2'
    ser.open()

    rospy.Subscriber('vision/garbage', Position, garbage_callback, callback_args=ser)
    rospy.Subscriber('vision/obstacles', Position, obstacle_callback, callback_args=ser)




    '''Reading IMU orientation data from Arduino and sending to Jetson'''
    orientation_pub = rospy.Publisher('imu/orientation', Quaternion, queue_size=100)
     
    #Disgard first 10 readings on startup
    for i in range(10):
        ser.readline()
    
    #Continuously read orientation data and publish to imu topic
    while (True):
        #Convert reading to a matrix of Euler angles
        line = ser.readline().split(',')
        e_orientation = [float(euler_angle) for euler_angle in line]

        #Convert Euler angles to quaternions
        q_orientation = tf.transformations.quaternion_from_euler(e_orientation[0], e_orientation[1], e_orientation[2], 'sxyz')

        #Make a quaternion message
        q_msg = Quaternion(x = q_orientation[0], y = q_orientation[1], z = q_orientation[2], w = q_orientation[3])

        #Publish to topic
        orientation_pub.publish(q_msg)
