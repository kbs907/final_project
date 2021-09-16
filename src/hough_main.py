#!/usr/bin/env python

import hough_drive
import rospy
from sensor_msgs.msg import Image

lidardata = None
imudata = None
ardata = None
ultrasounddata = None
camimage = None
rospy.init_node("hough_main")
mode = 1

def img_callback(data):
	global camimage, mode
	camimage = data
	if mode == 1:
		hough_drive.start(camimage)

rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

rospy.spin()




