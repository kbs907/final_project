#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor

steering_pub = None
box_data = []
motor_msg = xycar_motor()
person = None

def init_node():
    global steering_pub
    #rospy.init_node('human_follow')
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    time.sleep(15)
    rate = rospy.Rate(10)
    
def exit_node():
    print('finished')
    
def drive(angle, speed):
    global steering_pub
    motor_msg.header.stamp = rospy.Time.now()
    motor_msg.angle = angle
    motor_msg.speed = speed
    steering_pub.publish(motor_msg)
    
def callback(msg):
    global box_data
    box_data = msg
    print(box_data)
    
def result(object):
    while box_data is None:
        rate.sleep()
        
    while not rospy.is_shutdown():
        nobody = True
        boxes = box_data
        #print(boxes)
        for i in range(len(boxes.bounding_boxes)):
            if boxes.bounding_boxes[i].Class == object:
                nobody = False
                center = (boxes.bounding_boxes[i].xmax + boxes.bounding_boxes[i].xmin)/2
                angle = int(50.0*((center - 320.0)/320.0))
                drive(angle, 10)
        if nobody:
            drive(50, 10)
    rospy.on_shutdown(exit_node)
    
   
    
