#!/usr/bin/env python

#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

"""
    This module is used to send serial commands from NVIDIA Jetson to Arduino
"""

import time
import serial
import rospy
from vision.msg import Position

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")
    ser.close()

# TODO: Move this stuff to path planning and controls. This hack is temporary
def garbage_callback(data, args):
    pass

def obstacle_callback(data, args):
    dist = data.range
    bearing = data.bearing
    ser_local = args

    # If we are this close to an obstacle
    if dist < 1.5:
        ser_local.write('x 97')
    else:
        ser_local.write('x 99')

if __name__ == "__main__":
    
    rospy.init_node("powertrain")
    rospy.loginfo("Starting Powertrain Node")
    rospy.on_shutdown(shutdown_hook)

    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/ttyACM0'
    ser.open()

    time.sleep(2)

    # Run calibration
    ser.write('c')
    while ser.in_waiting:
        print("Serial Data: ", ser.readline())

    time.sleep(10) # Wait for calibration to be completed

    rospy.Subscriber('vision/garbage', Position, garbage_callback, callback_args=ser)
    rospy.Subscriber('vision/obstacles', Position, obstacle_callback, callback_args=ser)

    start_time = time.time()
    while (True):
        if ser.in_waiting:    # If there is data in the buffer
            print("Serial Data: ", ser.readline())

#        if (time.time() - start_time < 5):
#            ser.write('x 99')
#            time.sleep(1)
#        else:
#            ser.write('x 97')
#            time.sleep(1)
    # rospy.spin()

