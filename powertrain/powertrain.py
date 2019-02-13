#   Copyright Beach Cleaning Automated
#
#   Author: Bankole Adebajo

"""
    This module is used to send serial commands from NVIDIA Jetson to Arduino
"""

import serial

def init():
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/ttyUSB0'

def main():
    
    

    pass

main()