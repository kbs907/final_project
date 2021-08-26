#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time, cv2
import numpy as np
import yolo_steering_class as yolo
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


image = np.empty(shape=[0])
bridge = CvBridge()
xycar_msg = xycar_motor()
Width = 640
Height = 480

mode = 0
arData = {"DX":0.0}


# calibration 위한 변수 및 함수
calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))


# 카메라로부터 Subscribe한 Image형 토픽을 전처리시킬 수 있는 이미지로 변환시킴과 동시에 calibration 작업 수행 
def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    image = calibrate_image(image)


# 카메라로부터 AR코드가 인식되면 AR코드의 id값과 DX값을 저장
def reverse_callback(data):
    global ar_id, arData

    for i in data.markers:
        ar_id = i.id
        arData["DX"] = i.pose.pose.position.x


# 조향각과 속도를 xycar_motor로 Publish
def drive(Angle, Speed):
    global xycar_msg, motor_pub
    xycar_msg.angle = Angle
    xycar_msg.speed = Speed
    motor_pub.publish(xycar_msg)
    

# Publisher, Subscriber 선언
rospy.init_node('reverse_drive')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, reverse_callback)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )


# 이미지 전처리
def process_image(frame):
    global Width, Height

#    cv2.imshow('original', frame)
    
    H, S, V = cv2.split(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
    _, V = cv2.threshold(V, 148, 255, cv2.THRESH_BINARY_INV)
#    cv2.imshow('v', V)

ar_id = 1

while not rospy.is_shutdown():

    ############################## AR 태크 위치 보여주는 GUI창 생성 #############################
    img = np.zeros((100, 500, 3))
    img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    # print(" x : " + str(arData["DX"]))	
    point = int(arData["DX"] + 250)
    if point > 475:
        point = 475
    elif point < 25 : 
        point = 25	
    img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
    cv2.putText(img, "DX:"+str(arData["DX"]), (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    cv2.imshow('AR Tag Position', img)
    cv2.waitKey(1)
    ##########################################################################################


    ####################################### 정지선 검출 #######################################
    while not image.size == (640*480*3):
        continue

    process_image(image)
    ##########################################################################################

    angle = 0
    speed = 0
    object1 = 'pottedplant'
    object2 = 'bicycle'

    
    if ar_id == 0:
        mode = 1
    
    if ar_id == 7 and mode == 1:
        mode = 2
        
    if ar_id == 5:
    	  yolo.init_node()
    	  yolo.result(object1)
    elif ar_id == 4:
        yolo.init_node()
        yolo.result(object2)

    
    if mode == 1:
        print("mode 1")
        t_end = time.time() + 1.3
        while time.time() < t_end:
            drive(-30, -30)
        t_end = time.time() + 1.0
        while time.time() < t_end:
            drive(30, 10)
            
    time.sleep(5)


    if mode == 2:
        drive(0, 0)
    

    drive(angle, speed)
    print(ar_id, mode, angle, speed)

cv2.destroyAllWindows()

