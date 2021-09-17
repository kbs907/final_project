#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("camera")

bridge = CvBridge()

def callback(data):
	global bridge
	polygon = np.array([[100,340], [200,300], [480,300], [540,340]])

	imgdata = bridge.imgmsg_to_cv2(data, "bgr8")
	gray = cv2.cvtColor(imgdata, cv2.COLOR_BGR2GRAY)

	ROI = np.zeros_like(gray)
	cv2.fillConvexPoly(ROI, polygon, 1)
	ROI = ROI.astype(np.uint8)
	img = cv2.bitwise_and(gray, gray, mask=ROI)
	cv2.imshow('roi', img)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		return

rospy.Subscriber("/usb_cam/image_raw", Image, callback)
rospy.spin()

