#!/usr/bin/env python

import cv2, math, random, time
import numpy as np
import rospy
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
import hough_drive_park

bridge = CvBridge()
Offset = 330
Gap = 30
image = np.empty(shape=[0])
Width, Height = 640, 480
found_line = False
msg = xycar_motor()
pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
#cap = cv2.VideoCapture("/home/nvidia/xycar_ws/src/minsang_test/src/track.mkv")

mtx = np.array([[ 364.14123,    0.     ,  325.19317],
                [   0.     ,  365.9626 ,  216.14575],
                [   0.     ,    0.     ,    1.     ]])
dist = np.array([-0.292620, 0.068675, 0.006335, -0.002769, 0.000000])

cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height)) 
right = 99
rpos = 590

def to_calibrated(img):
    global mtx, dist, cal_mtx
    tf_image = cv2.undistort(img, mtx, dist, None, cal_mtx)
    return tf_image
    
def ultra_callback(data):
    global right
    right = data[4]
    print(right)

def img_callback(data):
    global image, Offset, Gap, Width, found_line, msg, pub, right, rpos

    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = to_calibrated(image)
	#retval, image = cap.read()
    cv2.imshow('img', image)
    rect = np.array([[200,300], [0,400], [640,400], [440,300]], dtype="float32")
    dst = np.array([[0,0],[0,100],[640,100],[640,0]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (640, 100))
    gray = cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)
    _, black = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    cv2.imshow("warped", warped)
    cv2.imshow("black", black)
    search_roi = black[0:10, 640-100:]
    num_whites = (cv2.countNonZero(search_roi))
    cv2.imshow("search", search_roi)
    cv2.waitKey(1)
    print(num_whites)
    if num_whites < 450:
        msg.angle = 0
        msg.speed = 0
        for i in range(15):
            pub.publish(msg)
            rospy.sleep(0.1)
        if right > 70:
            msg.angle = 0
            msg.speed = 25
            for i in range(25):
                pub.publish(msg)
                rospy.sleep(0.1)
            msg.angle = -40
            for i in range(10):
                pub.publish(msg)
                rospy.sleep(0.1)
            msg.angle = 0
            msg.speed = -50
            for i in range(20):
                pub.publish(msg)
                rospy.sleep(0.1)
            msg.angle = -35
            for i in range(10):
                pub.publish(msg)
                rospy.sleep(0.1)
            msg.speed = 0
            msg.angle = 0
            while True:
                pub.publish(msg)
    else:
        hough_drive_park.img_callback(image)
        hough_drive_park.start()
    cv2.waitKey(1)

	


