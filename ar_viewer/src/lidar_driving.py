#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy, math
import time

from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

speed = 10

# 조향각과 속도를 xycar_motor로 Publish
def drive(Angle, Speed):
    global motor_pub, xycar_msg
    xycar_msg.angle = Angle
    xycar_msg.speed = Speed
    motor_pub.publish(xycar_msg)


def callback(data):
    lidar_data = data.ranges
    increment = 0.714285708755853066

    r = lidar_data[375] #[int(270 / increment = 378)]
    fr = lidar_data[431] #[int(315 / increment = 441)]
    fm = lidar_data[-5] #[int(0 / increment = 0)]
    fl = lidar_data[58] #[int(45 / increment = 63)]
    l = lidar_data[125] #[int(90 / increment = 126)]
    
    print("r: "+str(r)+" fr: "+str(fr)+" fm: "+str(fm)+" fl: "+str(fl)+" l: "+str(l))

    '''
    if fl < 1.125 and fr > 0.5625:
        drive(50, 30)
    elif fl > 0.5625:
        drive(-50, 30)
    else:
        drive(0, 30)
    '''
    if fm < 0.3 and (l > 3.0 or r > 3.0):
        if fr > fl:
            drive(50, speed)
        elif fl > fr:
            drive(-50, speed)
            
            
    if r != 0.0 and l != 0.0:
        if r > l:
            drive(50, speed)
        elif l > r:
            drive(-50, speed)
    elif r == 0.0 or r > 0.6:
        drive(50, speed)
    elif l == 0.0 or l > 0.6:
        drive(-50, speed)
    else:
        drive(0, speed)


time.sleep(3)

# Publisher, Subscriber 선언
rospy.init_node('lidar_drive')
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
xycar_msg = xycar_motor()

rospy.spin()



