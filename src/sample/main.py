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
	do_T_parking = True
	do_yolo_stop = True
	mode = '1'
	drive = driving()
	
	

def sensor_check():
    ## 센서 on될동안 대기

## 구독자 callback 함수들 ########
def callback(msg) :
   

#################################



while not rospy.is_shutdown():
	
	motor.angle, motor.speed = hough(), 40

	if mode = '1' :	#시작~교차로 전
		if find_traffic :
			if 파란불 아니면 :
				motor.angle, motor.speed = 0, 0
			else :
				find_traffic = False
		elif 끼어들어야 하면 and 측면, 후방에 차량 있으면 :
			motor.angle, motor.speed = 0, 0
           
			
		else :	# 일반 주행
			if 교차로 AR태그 인식 :
				mode = '2'
				find_stopline = True	# 교차로 진입 전이므로 정지선 찾기 on
				find_traffic = True		# 신호 찾기 on
	
	elif mode = '2' :	# 교차로
		if find_stopline and 정지선 보일때:	# 정지선 찾아야 할 때
			angle, speed = 0, 0
			pub.publish(motor)
			find_stopline = False
			time.sleep(3)	# 3초간 정차
		elif 신호 찾아야 할 때 :
			if 파란불 아니면 : 
				angle, speed = 0, 0
			else :
				find_traffic = False
		elif t 주차 해야하면 and ar태그가 특정 위치에 있으면 :
			t_parking()	# t주차 알고리즘
			do_T_parking = False
		else :
			if 사람 정차 안했을 때 and 사람 앞일때 :
				yolo_stop()
				yolo_person = True
			elif 사람 정차 했고 and 타겟 앞일때 :
				yolo_stop()
				do_yolo_stop = False
			
			if 교차로 나오면 :
				mode = '3'
			
	
	elif mode = '3'	# 교차로이후 ~ 언덕 전
		if 언덕 보이면 :
			slope_driving()	#언덕 주행
			mode = '4'

	elif mode = '4' # 언덕이후~ 로터리전
		if 정지선 :
			angle, speed = 0, 0
			pub.publish(msg)
			time.sleep(3)
			mode = '5'

	elif mode = '5' # 로터리
		if 장애물 있으면 :
			angle, speed = 0, 0
		if 로터리 나오면 : # 주차 ar태그 pose로 판단
			mode = '6'

	elif mode = '6' # 장애물 회피
		angle, speed = avoid_driving()
		if 회피주행 끝나면 : # ex) 주차 ar태그 pose로 판단
			mode = '7'
			find_stopline = Ture

	else : 
		if 정지선 찾아야 하면 and 정지선 보일 때 :
			angle, speed = 0, 0
			find_stopline = False
			find_traffic = True
		elif 신호 찾아야하면 :
			if 파란불 :
				find_traffic = False
		else :
			angle, speed = parallel_driving()
	
	pub.publish(motor)
