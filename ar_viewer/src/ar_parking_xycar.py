#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math, time
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

roll, pitch, yaw = 0, 0, 0

def callback(msg):
    global arData

    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        

rospy.init_node('ar_parking')

rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)

motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size =1 )

xycar_msg = xycar_motor()


def back_drive(angle, cnt): #자동차를 후진시키는 함수, 속도를 -10으로 세팅하여 모터제어 토픽을 발행한다.
    global xycar_msg, motor_pub
    
    for cnt in range(cnt): #cnt회수만큼 토픽을 발행한다.
        xycar_msg.angle = angle
        xycar_msg.speed = -10
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
        
    for cnt in range(cnt): #cnt회수의 1/3만큼 토픽을 발행한다. 회전 방향은 반대로 한다.
        xycar_msg.angle = -angle
        xycar_msg.speed = -10
        motor_pub.publish(xycar_msg)
        time.sleep(0.1)
        

while not rospy.is_shutdown():

    (roll,pitch,yaw)=euler_from_quaternion((arData["AX"],arData["AY"],arData["AZ"], arData["AW"]))
	
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    pitch = round(pitch, 1)
    yaw = math.degrees(yaw)
    #img = np.zeros((100, 500, 3))
    #print(roll, pitch, yaw)
    
    #img = cv2.line(img,(25,65),(475,65),(0,0,255),2)
    #img = cv2.line(img,(25,40),(25,90),(0,0,255),3)
    #img = cv2.line(img,(250,40),(250,90),(0,0,255),3)
    #img = cv2.line(img,(475,40),(475,90),(0,0,255),3)

    point = int(arData["DX"]) + 250

    if point > 475:
        point = 475

    elif point < 25 : 
        point = 25	

    #img = cv2.circle(img,(point,65),15,(0,255,0),-1)  
  
    #distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DZ"],2))
    
    #2.putText(img, str(int(distance))+" pixel", (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))
    #cv2.putText(img, (350,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255))

    #dx_dz_pitch = "DX:"+str(int(arData["DX"]))+" DZ:"+str(int(arData["DZ"])) +" pitch:"+ str(round(pitch,1)) 
    #dx_dz_yaw = "DX:"+str(arData["DX"]*100)+" DZ:"+str(arData["DZ"]*100) +" Yaw:"+ str(round(yaw,1)) 
    #cv2.putText(img, dx_dz_pitch, (20,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255))

    #cv2.imshow('AR Tag Position', img)
    #cv2.waitKey(1)

    angle = 0
    speed = 10

    #arData["DX"] *= 100
    #arData["DZ"] *= 100
    print(arData["DX"], arData["DZ"], pitch)
    if arData["DX"] >= 0: #AR태그가 오른쪽에 있을 경우
        if (pitch < 0):   #Yaw값이 음수이면 크게 우회전
            angle = 50
        elif (pitch >= 0 and pitch < 30): #roll값이 0~30 사이인 경우 (약간 비스듬히 보이는 경우)
            if(arData["DX"] < 0.03): #DX값이 0~2 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] < 0.05): #DX 3~4이면 작게 우회전
                angle = 20
            else:
                angle = 50  #DX값이 5 이상이면 크게 우회전
                
        elif (pitch >= 10): #roll값이 10보다 큰 경우 (많이 비스듬히 보이는 경우)
            if(arData["DX"] < 0.03): #DX값이 0~2 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] < 0.05): #DX 3~4이면 작게 우회전, 하지만 roll값이 0~10 사이일 때보단 적게
                angle = 10
            else:
                angle = 25  #DX값이 5 이상이면 크게 우회전, 하지만 roll값이 0~10 사이일 때보단 적게
    elif arData["DX"] < 0: #AR태그가 오른쪽에 있을 경우
        if (pitch > 0):   #roll값이 양수이면 크게 좌회전
            angle = -50
        elif (pitch <= 0 and pitch > -30): #roll값이 -30~0 사이인 경우 (약간 비스듬히 보이는 경우)
            if(arData["DX"] > -0.03): #DX값이 -2~0 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] > -0.05): #DX -4~-3이면 작게 좌회전
                angle = -20
            else:
                angle = -50  #DX값이 -5 이하이면 크게 좌회전
                
        elif (pitch <= -30): #roll값이 -30보다 작은 경우 (많이 비스듬히 보이는 경우)
            if(arData["DX"] > -0.03): #DX값이 -2~0 이면 직진 (angle = 0) (차의 중앙과 적게 떨어져 있을 경우)
                angle = 0
            elif(arData["DX"] > -0.05): #DX -4~-3이면 작게 좌회전, 위 보단 작은 각도로
                angle = -10
            else:
                angle = -25  #DX값이 -5 이하이면 크게 좌회전

    if(arData["DZ"] > 1.5): #DZ값이 150 이상이면 멀리 있는 것이므로 속도를 빠르게 -> 30
        speed = 20
    elif(arData["DZ"] > 1): #DZ값이 100~150 사이면 속도를 조금 낮추기 -> 20
        speed = 15
    elif(arData["DZ"] > 0.3): #DZ값이 70~100 사이면 속도를 더 낮추기 -> 10
        speed = 10
        
        if (pitch > 20 or abs(arData["DX"] > 0.05)): #roll값이 10 이상이거나 DX값이 100 이상이면 후진(시계방향으로 후진)
            back_drive(-50, 45)
        elif (pitch < -20 or abs(arData["DX"] > 0.05)): #roll값이 -10 이하이거나 DX값이 100 이상이면 후진(반시계방향으로 후진)
            back_drive(50, 45)
    else:
        speed = 0
    
    xycar_msg.angle = angle
    xycar_msg.speed = speed
    motor_pub.publish(xycar_msg)
    #time.sleep(0.05)

cv2.destroyAllWindows()



