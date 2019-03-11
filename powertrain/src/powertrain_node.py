#!/usr/bin/env python-snakes

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
from powertrain.msg import PowertrainParams
from geometry_msgs.msg import Quaternion
from math import pi

PUBLISH_RATE = 10 # Publish Rate in Hz
MAX_LEFT = -pi/4
MAX_RIGHT = pi/4
MAX_BRUSH = 100
MIN_BRUSH = -100
MAX_CONV = 100
MIN_CONV = -100

def shutdown_hook():
    """ Called when ROS is exit """

    rospy.loginfo("Shutting down...")

curr_powertrain = PowertrainParams()

def update_powertrain(data):
    """ Updates the powertrain state """

    global curr_powertrain

    speed = data.speed
    delta = data.delta
    brush_speed = data.brush_speed
    conveyor_speed = data.conveyor_speed

    # Convert speed from m/s to powertrain value
    if speed == 0:  # Two cases for now, stopped and 1m/s
        speed = 97
    elif speed == 1:
        speed = 100

    # Convert angle from radians to powertrain value
    if delta < MAX_LEFT:
        delta = MAX_LEFT
    elif delta > MAX_RIGHT:
        delta = MAX_RIGHT
    delta = int(((delta - MAX_LEFT) / (MAX_RIGHT - MAX_LEFT)) * 255) # Convert to range 0-255

    # Convert brush_speed from percentage to powertrain value
    if brush_speed < MIN_BRUSH:
        brush_speed = MIN_BRUSH
    elif brush_speed > MAX_BRUSH:
        brush_speed = MAX_BRUSH
    brush_speed = int(((brush_speed - MIN_BRUSH) / (MAX_BRUSH - MIN_BRUSH)) * 255) # Convert to range 0-255

    # Convert conveyor from percentage to powertrain value
    if conveyor_speed < MIN_CONV:
        conveyor_speed = MIN_CONV
    elif conveyor_speed > MAX_CONV:
        conveyor_speed = MAX_CONV
    conveyor_speed = int(((conveyor_speed - MIN_BRUSH) / (MAX_BRUSH - MIN_BRUSH)) * 255) # Convert to range 0-255

    curr_powertrain.speed = speed
    curr_powertrain.delta = delta
    curr_powertrain.brush_speed = brush_speed
    curr_powertrain.conveyor_speed = conveyor_speed

def main():
    rospy.init_node("powertrain")
    rospy.loginfo("Starting Powertrain Node")
    rospy.on_shutdown(shutdown_hook)

    global curr_powertrain

    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/ttyACM0'
    ser.open()

    rospy.Subscriber('powertrain/cmd', PowertrainParams, update_powertrain)

    # Reading IMU orientation data from Arduino and sending to Jetson
    orientation_pub = rospy.Publisher('imu/orientation', Quaternion, queue_size=10)
    r = rospy.Rate(PUBLISH_RATE)

    #Disgard first 10 readings on startup
    for i in range(10):
        ser.readline()
    
    #Continuously read orientation data and publish to imu topic
    while (not rospy.is_shutdown()):
        #Convert reading to a matrix of Euler angles
        line = ser.readline().split(',')
        e_orientation = [float(euler_angle) for euler_angle in line]

        #Convert Euler angles to quaternions
        q_orientation = tf.transformations.quaternion_from_euler(e_orientation[0], e_orientation[1], e_orientation[2], 'sxyz')

        #Make a quaternion message
        q_msg = Quaternion(x = q_orientation[0], y = q_orientation[1], z = q_orientation[2], w = q_orientation[3])

        #Publish to topic
        orientation_pub.publish(q_msg)

        # Broadcast latest powertrain data to arduino
        # ASCII character table: http://www.asciitable.com/
        ser.write('x ' + str(curr_powertrain.speed))
        ser.write('y ' + str(curr_powertrain.delta))
        ser.write('z ' + str(curr_powertrain.brush_speed))
        ser.write('{ ' + str(curr_powertrain.conveyor_speed))

        r.sleep()

if __name__ == "__main__":
    main()

