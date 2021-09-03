#!/usr/bin/env python

import cv2, math, random, time
import numpy as np
import rospy
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
import hough_drive_s

bridge = CvBridge()
Offset = 330
Gap = 30
image = np.empty(shape=[0])
Width = 640
found_line = False
msg = xycar_motor()
pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

def img_callback(data):
	global image, Offset, Gap, Width, found_line, msg, pub
	
	image = bridge.imgmsg_to_cv2(data, "bgr8")
	rect = np.array([[200,300], [0,400], [640,400], [500,300]], dtype="float32")
	dst = np.array([[0,0],[0,100],[640,100],[640,0]], dtype="float32")
	M = cv2.getPerspectiveTransform(rect, dst)
	warped = cv2.warpPerspective(image, M, (640, 100))
	gray = cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)
	_, black = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
	cv2.imshow("warped", warped)
	cv2.imshow("black", black)
	search_roi = black[0:50, 640-70:640]
	num_whites = (cv2.countNonZero(search_roi))
	print(num_whites)
	cv2.imshow("search", search_roi)
	cv2.waitKey(1)

	if 3265 < num_whites < 3600:
	    msg.angle = 0
	    msg.speed = 0
	    for i in range(10):
	        pub.publish(msg)
	        rospy.sleep(0.1)
	    msg.speed = 20
	    for i in range(5):
	        pub.publish(msg)
	        rospy.sleep(0.1)
	else:
	    hough_drive_s.img_callback(data)
	    hough_drive_s.start()



	cv2.waitKey(1)

	

