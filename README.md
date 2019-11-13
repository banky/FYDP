# FYDP (Final Year Design Project)

This repository contains all the associated software that was used to operate our autonomous beach cleaning robot. I'd link to our project website, but we stopped paying for the domain :) 

## Setup Instructions
1. Install ROS from http://wiki.ros.org/ROS/Installation
2. Install opencv tools
```bash
sudo apt-get install libopencv-dev python-opencv
```
3. Install libusb for USB communication
```bash
sudo apt-get install libusb-1.0-0-dev
```
4. Install GLUT libraries (for kinect)
```bash
sudo apt-get install libxmu-dev libxi-dev
```
5. Install some more libraries 
```bash
sudo apt-get install cython python-dev python-numpy
```
6. Install libfreenect
```bash
sudo apt-get install freenect
sudo python setup.py install # From https://github.com/OpenKinect/libfreenect/tree/master/wrappers/python
```
7. Machine learning models are on Google Drive under Machine Learning Models/

8. Install TKinter
```bash
sudo apt-get install python3-tk
```

