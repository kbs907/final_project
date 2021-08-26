#! /usr/bin/env python
# -*- coding:utf-8 -*-

## 필요한 모듈 import ##
import cv2
import rospy
from xycar_msgs.msg import xycar_motor

def init():
    ## 변수, 발행자, 구독자 선언
    motor = xycar_motor()
	stopline_detect =
	

def sensor_check():
    ## 센서 on될동안 대기

## 구독자 callback 함수들 ##
def callback(msg) :
   

#################################



while not rospy.is_shutdown():
	if mode = '1' :
		if 신호등 전 :
			motor.angle, motor.speed = traffic_stop()
		elif 끼어들어야 하면 :
			motor.angle, motor.speed = cut_in()
			
		else :
			hough_driving()
			if stopline_detect == True :
				stopline_detect == 
			if 교차로 AR태그 인식 :
				mode = '2'
	
	elif mode = '2' :
		if 정지선 전 :
			angle, speed = stop_line()
			

        cv2.waitKey(1)
        #t0 = time.time()
        self.rate.sleep()
        #print('time:', time.time()-t0)
