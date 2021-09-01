#! /usr/bin/env python

import cv2
import yolo_module
import rospy,time
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor


box_data = BoundingBoxes()
steering_pub = None
class_name = ''
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
Yolo_m = None
def yolo_callback(msg):
  #print(msg)
  Yolo_m.set_boxdata(msg)
'''  
def ar_callback(msg):
	global ar_flag, arData, class_name, yolo_flag

	for i in msg.markers:
		print('prevId', prevId, 'i.id', i.id, 'ar_flag', ar_flag)

		if i.id==??:
			ar_flag=1
      class_name='cat'
    elif i.id == ?? :
      ar_flag=1
      class_name = 'dog'
    elif i.id == ?? :
      ar_flag = 1
      class_name = 'cow' 

		
		elif prevId==?? and i.id==??:
			yolo_flag=True
########	
		  arData["DX"] = i.pose.pose.position.x+0.1 if i.pose.pose.position.x>0 else i.pose.pose.position.x-0.1
  	arData["DY"] = i.pose.pose.position.z
  	#self.arData["DZ"] = i.pose.pose.position.z

  	arData["AX"] = i.pose.pose.orientation.x
  	arData["AY"] = i.pose.pose.orientation.y
  	arData["AZ"] = i.pose.pose.orientation.z
  	arData["AW"] = i.pose.pose.orientation.w
##########
'''	

  
def init_node_1():
  global steering_pub
  rospy.init_node("yolo_mission")
  #rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
  rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, yolo_callback)
  #rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
  steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
  
def drive(angle, speed):
  global steering_pub
  motor_msg.header.stamp = rospy.Time.now()
  motor_msg.angle = angle
  motor_msg.speed = speed
  #print(motor_msg)
  steering_pub.publish(motor_msg)
  
if __name__ == '__main__':
    #YoloModule.init()
    global class_name, Yolo_m
    class_name = 'dog'  #change ar.id
    init_node_1()
    Yolo_m = yolo_module.YoloModule()
    rate = rospy.Rate(10)
    while box_data is None:
      rate.sleep()
    while not rospy.is_shutdown():
      #person_pos = Yolo_m.get_mission("person")
      
      mission_pos =  Yolo_m.get_mission(class_name) 
      
    