#! /usr/bin/env python
# -*- coding:utf-8 -*-

## 필요한 모듈 import ##
import cv2
import rospy
from xycar_msgs.msg import xycar_motor

def init():
    ## 변수, 발행자, 구독자 선언
    motor = xycar_motor()
	find_stopline = False	# 정지선 찾기 on/off
	find_traffic = True		# 신호 찾기 on/off
	mode = '1'
	drive = driving()
	
	

def sensor_check():
    ## 센서 on될동안 대기

## 구독자 callback 함수들 ########
def callback(msg) :
   

#################################



while not rospy.is_shutdown():
	if mode = '1' :	#시작~교차로 전
		if 신호 찾아야 할 때 :
			if 파란불 아니면 :
				motor.angle, motor.speed = 0, 0
			else :
				find_traffic = False
		elif 끼어들어야 하면 :
			motor.angle, motor.speed, flag = cut_in()
           
			
		else :	# 일반 주행
			angle, speed, flag = .hough_driving(mode = 1)
			if 교차로 AR태그 인식 :
				mode = '2'
				target = ''
				find_stopline = True	# 교차로 진입 전이므로 정지선 찾기 on
				find_traffic = True		# 신호 찾기 on
	
	elif mode = '2' :	# 교차로
		if find_stopline :	# 정지선 찾아야 할 때
			if stop_line() :
				angle, speed = 0, 0
				find_stopline = False
			else :
				angle, speed = hough_driving()
		elif 신호 찾아야 할 때 :
			if 파란불 아니면 : 
				angle, speed = traffic_stop()
			else :
				angle, speed = intersec_driving()
				find_traffic = False
		elif :

	
	elif mode = '3'	# 교차로이후 ~ 언덕 전
		angle, speed = curve_driving()
		if 언덕 만나면 :
			mode = '4'

	elif mode = '4' # 언덕~ 로터리전
		angle, speed = slope_driving()
		if 정지선 :
			angle, speed = 0, 0
			mode = '5'

	elif mode = '5' # 로터리
		angle, speed = rotary_driving()
		if 로터리 나오면 : # 주차 ar태그 pose로 판단
			mode = '6'

	elif mode = '6' # 장애물 회피
		angle, speed = avoid_driving()
		if 회피주행 끝나면 : # ex) 주차 ar태그 pose로 판단
			mode = '7'

	else : 
		if 정지선 찾아야 하면 :
			if 정지선 :
				angle, speed = 0, 0
				find_stopline = False
				find_traffic = True
			else :
				angle, speed = hough_driving()
		elif 신호 찾아야하면 :
			if 파란불 :
				find_traffic = False
		else :
			angle, speed = parallel_driving()
