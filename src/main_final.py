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
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

from image_processing_module import *
from yolo_module import *
from drive_module import *
from ultra_module import *
from ar_module import *

### module 사용을 위한 전역 변수 ###
global imageProcessModule
global yoloModule
global driveModule
global ultraModule
global arModule

### main 전역 변수 ###
global find_stopline	# 정지선 찾기 on/off
global find_traffic		# 신호 찾기 on/off
global do_T_parking
global do_yolo_stop
global mode
global cut_in
global class_name

def init():
    ## 변수, 발행자, 구독자 선언
    global imageProcessModule
    global yoloModule
    global driveModule
    global ultraModule
    global arModule

    global find_stopline	# 정지선 찾기 on/off
    global find_traffic		# 신호 찾기 on/off
    global do_T_parking
    global do_yolo_stop
    global mode
    global cut_in
    global yolo_person
    global class_name  

    imageProcessModule = ImageProcessingModule()
    yoloModule = YoloModule()
    driveModule = DriveModule()
    ultraModule = UltraModule()
    arModule = ArModule()

    find_stopline = False	# 정지선 찾기 on/off
    find_traffic = True		# 신호 찾기 on/off
    do_T_parking = True
    do_yolo_stop = True
    mode = '1'
    cut_in = True
    yolo_person = True
    class_name = 'person'

    rospy.init_node('main')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)


def sensor_check():
    global imageProcessModule
    global ultraModule

    while True :
        if imageProcessModule.get_image_size() != (640*480*3) :
            continue
        if not ultraModule.get_data() :
            continue
        break

### 구독자 callback 함수들 ###
def img_callback(data):
    global imageProcessModule
    imageProcessModule.set_image(data)

def ultra_callback(data) :
    global ultraModule
    ultraModule.set_data(data)

def ar_callback(data):
    global arModule
    if len(data.markers) :
        arModule.set_arData(data)

if __name__ == '__main__':
    init()
    sensor_check()

    print('==============')
    print('=   mode 1   =')
    print('==============')

    while not rospy.is_shutdown():
        global imageProcessModule
        global yoloModule
        global driveModule
        global ultraModule

        global find_stopline
        global find_traffic
        global do_T_parking
        global do_yolo_stop
        global mode
        global cut_in
        global class_name

        cte, fail_count = imageProcessModule.get_cte()
        angle, speed = driveModule.Hough_drive(cte, fail_count)  # 기본 주행
        
        print ('imageProcessModule.get_corner_count', imageProcessModule.get_corner_count())
        if mode == '1' :	#시작~교차로 전
            if find_traffic :
                if not imageProcessModule.get_traffic_light('first') :
                    angle, speed = 0, 0
                else :
                    find_traffic = False
            elif cut_in and driveModule.cut_in(imageProcessModule.get_road_width(), ultraModule.get_data()) :
                    angle, speed = angle, 0
                    driveModule.drive(angle,speed)
                    driveModule.stop_nsec(2)
                    cut_in = False

            else :	# 일반 주행
                if imageProcessModule.get_corner_count() == 3 :
                    mode = '2'
                    find_stopline = True	# 교차로 진입 전이므로 정지선 찾기 on
                    find_traffic = True		# 신호 찾기 on

                    print('==============')
                    print('=   mode 2   =')
                    print('==============')
        
        elif mode == '2' :	# 교차로
            yolo_size = yoloModule.get_size(class_name)
            if find_stopline and imageProcessModule.detect_stopline() :	# 정지선 찾아야 할 때
                driveModule.stop_nsec(3) # 3초 정차
                find_stopline = False
                    
            elif find_traffic :
                if not imageProcessModule.get_traffic_light('second') : 
                    angle, speed = driveModule.stop()
                else :
                    find_traffic = False
                    find_stopline = True
            elif do_T_parking and arModule.is_ar() :
                driveModule.T_parking(arModule.get_yaw())	# t주차 알고리즘
                do_T_parking = False
            else :
                if do_yolo_stop and yolo_size != None :
                    do_yolo_stop, class_name = driveModule.yolo_drive(angle, class_name, yolo_size)
                if not do_yolo_stop : # 교차로 진입
                    find_stopline = True
                    mode = '3'
        '''
        elif mode = '3'	# 교차로이후 ~ 언덕 전
            if 언덕 보이면 :
                slope_driving()	#언덕 주행
                mode = '4'

        elif mode = '4' # 언덕이후~ 로터리전
            if 정지선 :
                angle, speed = imageProcessModule.stop()
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
        '''
        
        driveModule.drive(angle, speed)
