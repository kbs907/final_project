#! /usr/bin/env python
# -*- coding:utf-8 -*-

### 필요한 모듈 import ###
import rospy, rospkg, time, sys, os
import numpy as np
import cv2, random, math

from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray

from image_processing_module import *
#from yolo_module import *
from drive_module import *
from ultra_module import *

### module 사용을 위한 전역 변수 ###
global imageProcessModule
#global yoloModule
global driveModule
global ultraModule

### main 전역 변수 ###
global find_stopline	# 정지선 찾기 on/off
global find_traffic		# 신호 찾기 on/off
global do_T_parking
global do_yolo_stop
global mode
global cut_in

def init():
    ## 변수, 발행자, 구독자 선언
    global imageProcessModule
    #global yoloModule
    global driveModule
    
    global find_stopline	# 정지선 찾기 on/off
    global find_traffic		# 신호 찾기 on/off
    global do_T_parking
    global do_yolo_stop
    global mode
    global cut_in
    
    imageProcessModule = ImageProcessingModule()
    #yoloModule = YoloModule()
    driveModule = DriveModule()
    ultraModule = UltraModule()

    find_stopline = False	# 정지선 찾기 on/off
    find_traffic = True		# 신호 찾기 on/off
    do_T_parking = True
    do_yolo_stop = True
    mode = '1'
    cut_in = True

    rospy.init_node("main")
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)


def sensor_check():
    ## 센서 on될동안 대기
    pass

### xycar motor 토픽 publish 함수 ###    
def xycar_drive(angle, speed): 
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    xycar_msg = xycar_motor()
    
    xycar_msg.angle = angle
    xycar_msg.speed = speed

    pub.publish(xycar_msg)
    
### 구독자 callback 함수들 ###
def img_callback(data):
    global imageProcessModule
    imageProcessModule.set_image(data)

def ultra_callback(data) :
    global ultraModule
    ultraModule.set_data()

while not rospy.is_shutdown():
    global imageProcessModule
    #global yoloModule
    global driveModule
    global ultraModule

    global find_stopline
    global find_traffic
    global do_T_parking
    global do_yolo_stop
    global mode
    global cut_in

    cte, fail_count, stopline = imageProcessModule.get_cte()
    angle, speed = driveModule.Hough_drive(cte, fail_count, stopline)  # 기본 주행
    
    if mode == '1' :	#시작~교차로 전
        if find_traffic :
            if get_traffic_light('first') :
                angle, speed = 0, 0
                
    else :
        find_traffic = False
    elif driveModule.cut_in(imageProcessModule.get_road_width(), ultraModule.get_data()) and cut_in :
        drive(steer,0)
        sleep(40)
        cut_in = False

        else :	# 일반 주행
            if 교차로 AR태그 인식 :
                mode = '2'
                find_stopline = True	# 교차로 진입 전이므로 정지선 찾기 on
                find_traffic = True		# 신호 찾기 on
    
    elif mode == '2' :	# 교차로
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
            if 교차로 진입 :
                find_stopline = False
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
    
    pub.publish(angle, speed)
