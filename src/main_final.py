#! /usr/bin/env python
# -*- coding:utf-8 -*-

### 필요한 모듈 import ###
import rospy, rospkg, time, sys, os
import numpy as np
import cv2, random, math

from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray, String
from darknet_ros_msgs.msg import BoundingBoxes
from visualization_msgs.msg import Marker, MarkerArray
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

from image_processing_module import *
from yolo_module import *
from drive_module import *
from ultra_module import *
from ar_module import *
from lidar_module import *

### module 사용을 위한 전역 변수 ###
global imageProcessModule
global yoloModule
global driveModule
global ultraModule
global arModule

### main 전역 변수 ###
global find_stopline	# 정지선 찾기 on/off
global find_traffic		# 신호 찾기 on/off
global find_ar
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
    global lidarModule

    global find_stopline	# 정지선 찾기 on/off
    global find_traffic		# 신호 찾기 on/off
    global find_ar
    global do_T_parking
    global do_yolo_stop
    global mode
    global cut_in, cut_in2
    global yolo_person
    global class_name  
    global drive_count

    imageProcessModule = ImageProcessingModule()
    yoloModule = YoloModule()
    driveModule = DriveModule()
    ultraModule = UltraModule()
    arModule = ArModule()
    lidarModule = LidarModule()
    find_stopline = True#False	# 정지선 찾기 on/off
    find_traffic = True#False #True		# 신호 찾기 on/off
    find_ar = True
    do_T_parking = False
    do_yolo_stop = False
    mode = '1'
    cut_in = True
    cut_in2 = True
    yolo_person = True
    class_name = 'person'
    drive_count = 0
    rospy.init_node('main')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
    rospy.Subscriber('scan', LaserScan, lidar_callback)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, yolo_callback)
    rospy.Subscriber("mission", String, mission_callback)

def sensor_check():
    global imageProcessModule
    global ultraModule

    while True :
        if imageProcessModule.get_image_size() != (640*480*3) :
            continue
        if not ultraModule.get_data() :
            continue
        if not lidarModule.get_data() :
            continue
        #if not yoloModule.get_data() :
        #    continue
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
    arModule.set_arData(data)

def lidar_callback(data):
    global lidarModule
    lidarModule.set_lidarData(data)

def yolo_callback(data):
    global yoloModule
    yoloModule.set_boxdata(data)

def mission_callback(data):
    global find_stopline
    global find_traffic
    global find_ar
    global do_T_parking
    global do_yolo_stop
    global mode
    global cut_in, cut_in2
    global drive_count
    
    mission = data.data
    
    if mission == "1":
        print('==============')
        print('=   mode 1   =')
        print('==============')

        mode = '1'
        find_traffic = False
        cut_in = True
        cut_in2 = True
    elif mission == "2":
        print('==============')
        print('=   mode 2   =')
        print('==============')

        mode = '2'
        find_stopline = True
        find_traffic = True
    elif mission == "3":
        print('==============')
        print('=   mode 3   =')
        print('==============')
        
        mode = '2'
        find_stopline = False
        find_traffic = False
        find_ar = False
        do_T_parking = False
        do_yolo_stop = True

    elif mission == "4":
        print('==============')
        print('=   mode 3   =')
        print('==============')
        find_stopline = True
        mode = '3'
        
    elif mission == "5":
        print('==============')
        print('=   mode 4   =')
        print('==============')

        mode = '4'
        find_stopline = True

    elif mission == "6":
        print('==============')
        print('=   mode 7   =')
        print('==============')

        mode = '7'
        find_stopline = True

    elif mission == "7":
        print('==============')
        print('=   mode 7   =')
        print('==============')

        mode = '7'
        find_stopline = False
        find_traffic = False
        drive_count = 80



if __name__ == '__main__':
    rospy.sleep(10)
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
        global find_ar
        global do_T_parking
        global do_yolo_stop
        global mode
        global cut_in, cut_in2
        global class_name
        global drive_count

        #print("MODE: " + mode)

        cte, fail_count = imageProcessModule.get_cte()
        #print("failcount = ", fail_count)
        angle, speed = driveModule.Hough_drive(cte, fail_count)  # 기본 주행
        
        #print ('imageProcessModule.get_corner_count', imageProcessModule.get_corner_count())
        if mode == '1' :	#시작~교차로 전
            speed = 20
            if find_traffic :
                if not imageProcessModule.get_traffic_light('first') :
                    speed = 0
                else :
                    find_traffic = False
            elif cut_in :
                speed, cut_in = driveModule.cut_in(imageProcessModule.get_road_width()) 
            elif cut_in2 :
                speed, cut_in2 = lidarModule.cut_in_check()
            else :	# 일반 주행
                speed = 20
                if lidarModule.forward_obstacle() :
                    speed = 0
                if imageProcessModule.get_corner_count() == 1 :
                    mode = '2'
                    find_stopline = True	# 교차로 진입 전이므로 정지선 찾기 on
                    find_traffic = True		# 신호 찾기 on

                    print('==============')
                    print('=   mode 2   =')
                    print('==============')
        
        elif mode == '2' :	# 교차로
            #print('ardata : ', arModule.get_ardata())
            #find_stopline, find_traffic = False, False
            speed = 15
            if find_stopline :
                if imageProcessModule.detect_stopline() :	# 정지선 찾아야 할 때
                    #print("stopline")
                    driveModule.stop_nsec(1) # 1초 정차
                    find_stopline = False
                    
            elif find_traffic :
                if not imageProcessModule.get_traffic_light('second') : 
                    angle, speed = driveModule.stop()
                else :
                    find_traffic = False
            
            elif find_ar :
                if arModule.is_ar():
                    find_ar = False
                    do_T_parking = True
            
            elif do_T_parking:
                print("start T parking")
                driveModule.start_T_parking()
                print("do T Parking")
                driveModule.T_parking(arModule.get_distance(), arModule.get_arctan())

                '''
                if not arModule.finish_T_parking():
                    while arModule.finish_T_parking():
                        print('again T parking')
                        driveModule.again_T_parking(arModule.get_distance(), arModule.get_arctan())
                '''

                driveModule.end_T_parking(arModule.get_arctan())
                print("end T parking")
                do_T_parking = False
                do_yolo_stop = True
                intersec = imageProcessModule.get_corner_count()

            else :
                yolo_size = yoloModule.get_size(class_name)
                #print('yolo_size: ', yolo_size)
                #print("classname", class_name)
                if do_yolo_stop and yolo_size != None :
                    do_yolo_stop, class_name = driveModule.yolo_drive(angle, class_name, yolo_size)
                if not do_yolo_stop and imageProcessModule.get_corner_count() - intersec > 2 : # 교차로 진입
                    print('==============')
                    print('=   mode 3   =')
                    print('==============')
                    find_stopline = True
                    mode = '3'
        
        elif mode == '3' :	# 교차로이후 ~ 언덕 전
            speed = 18
            if imageProcessModule.detect_slope() :
                driveModule.slope_drive(angle)	#언덕 주행
                
                print('==============')
                print('=   mode 4   =')
                print('==============')
                mode = '4'

        elif mode == '4' : # 언덕이후~ 로터리전
            if find_stopline :
                speed = 15
                if imageProcessModule.detect_stopline() :	# 정지선 찾아야 할 때
                    driveModule.stop_nsec(1) # 1초 정차
                    find_stopline = False
                    print('stopline!')
            else :
                if lidarModule.can_rotary_in() :  #로터리 진입 가능하면
                    print('==============')
                    print('=   mode 5   =')
                    print('==============')
                    mode = '5'
                else :
                    speed = 0
       
        elif mode == '5' : # 로터리
            if lidarModule.forward_obstacle() :
                speed = 0

            if lidarModule.end_rotary() : # 주차 ar태그 pose로 판단 또는 차선으로
                print('==============')
                print('=   mode 6   =')
                print('==============')
                mode = '6'
        
        elif mode == '6' : # 장애물 회피
            speed = 10
            lpos, rpos = imageProcessModule.get_lane_pos()
            '''
            if yolo_first_car:
                car_pose = yoloModule.car_pose()
            else:
                car_pose = "None"

            angle = yoloModule.car_avoid(lpos, rpos, car_pose)
            '''
            angle = yoloModule.car_avoid(lpos, rpos)

            if 0.1 < arModule.get_distance() < 0.5 : # ex) 주차 ar태그 pose로 판단
                print('==============')
                print('=   mode 7   =')
                print('==============')
                mode = '7'
                find_stopline = True

        else : 
            if find_stopline :
                if imageProcessModule.detect_stopline_2() :
                    print("stopline")
                    driveModule.stop_nsec(1)
                    find_stopline = False
                    find_traffic = True
                    speed=0
            elif find_traffic :
                print('find traffic')
                speed = 0
                if imageProcessModule.get_traffic_light('first') :
                    find_traffic = False

            elif not find_traffic:
                if drive_count < 90 :
                    #print(drive_count)
                    drive_count += 1
                
                elif imageProcessModule.detect_parkinglot() :
                    print('detect parkinglot')
                    speed = 10
                    driveModule.stop_nsec(1)
                    for _ in range(5):
                        driveModule.drive(0, 15)
                        rospy.sleep(0.1)
                    if not ultraModule.right_obstacle() :
                        driveModule.parallel_parking()
                        driveModule.stop_nsec(1)
                        print('##### finish! #####')
                        break

        driveModule.drive(angle, speed)
