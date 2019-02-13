#!/usr/bin/env python

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

"""
    This module is used to send serial commands from NVIDIA Jetson to Arduino
"""

import serial
import rospy
from vision.msg import Position

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
    ser.port = '/dev/ttyUSB0'

    rospy.Subscriber('vision/garbage', Position, garbage_callback, callback_args=ser)
    rospy.Subscriber('vision/obstacles', Position, obstacle_callback, callback_args=ser)

    rospy.spin()