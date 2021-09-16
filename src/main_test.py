#! /usr/bin/env python

import rospy, rospkg, time, sys, os
import numpy as np
import cv2, random, math

from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray

from lim_image_processing_module import *
#from yolo_module import *
from lim_drive_module import *

global imageProcessModule
#global yoloModule
global driveModule

def init():
    global imageProcessModule
    #global yoloModule
    global driveModule

    imageProcessModule = ImageProcessingModule()
    #yoloModule = YoloModule()
    driveModule = DriveModule()
    
    rospy.init_node("test")
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

def img_callback(data):
    global imageProcessModule
    #global yoloModule
    global driveModule
    
    imageProcessModule.set_image(data)

def test1():
    global imageProcessModule
    #global yoloModule
    global driveModule
    
    cte, fail_count, stopline = imageProcessModule.get_cte()
    angle, speed = driveModule.Hough_drive(cte, fail_count, stopline)
    xycar_drive(speed, angle)
    
def xycar_drive(angle, speed): 
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    xycar_msg = xycar_motor()
    
    xycar_msg.angle = angle
    xycar_msg.speed = speed

    pub.publish(xycar_msg)

if __name__ == '__main__':
    global imageProcessModule
    #global yoloModule
    global driveModule
    
    init()
    
    rate = rospy.Rate(10)
    while not imageProcessModule.get_image_size() == (640*480*3):
        print("image size is not 640*480*3")
        continue
    
    while not rospy.is_shutdown():
        test1()
        rate.sleep()
