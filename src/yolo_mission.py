#! /usr/bin/env python

import cv2
import yolo_module
import hough
import rospy,time
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_msgs.msg import xycar_motor
import  math, random, time
import numpy as np
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

######## 추가 변수 ##########
Yolo_m = None
class_name = 'person'
yolo_flag = True
yolo_count = 15
yolo_stop_size = 110
#############################

bridge = CvBridge()
fail_count = 0
box_data = BoundingBoxes()
motor_msg = xycar_motor()
steering_pub = None
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
image = np.empty(shape=[0])

############# 추가 ###############
def yolo_callback(msg):
  #print(msg)
  #global box_data
  #box_data = 
  Yolo_m.set_boxdata(msg)
  #Yolo_m.set_boxdata(msg)
###################################


############### 추가 함수 ##################
def yolo_mission(cte):
  global class_name, yolo_count, yolo_stop_size
  if  yolo_size < yolo_stop_size:
      print("YOLO")
      drive(cte*0.4, 15)
  elif yolo_size >= yolo_stop_size:
      print("!))")
      for _ in range(yolo_count):
          #print(c_s, yolo_size, "dd")
          drive(cte*0.4, 12)
          rate.sleep()
      for _ in range(50):
          drive(cte*0.4,0)
          rate.sleep()
      print("Stop")
      if class_name == 'person':
          class_name = 'cat'
          yolo_count += 3
          ## cat count = 18
          
      else:
          yolo_flag = False

##########################

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")
  
def init_node_1():
  global steering_pub
  rospy.init_node("yolo_mission")
  rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, yolo_callback)
  #rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback)
  rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
  steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
  
def drive(angle, speed):
  global steering_pub, motor_msg
  motor_msg.angle = angle
  motor_msg.speed = speed
  #print(motor_msg)
  steering_pub.publish(motor_msg)
  
if __name__ == '__main__':
    #YoloModule.init()
    global class_name, Yolo_m, image, fail_count
    Yolo_m = yolo_module.YoloModule()
    init_node_1()
    #rate = rospy.Rate(10)
    rate = rospy.Rate(20)
    while box_data is None:
        rate.sleep()
    while not image.size == (640*480*3):
        continue
    
    while not rospy.is_shutdown():
        #global image
        cal_image = hough.to_calibrated(image)
        #cv2.imshow("image",image)
        pose, hough_img = hough.process_image(cal_image)
        center = (pose[0] + pose[1])/2
        cte = center - 320
        #print(cte*0.4)

        ################## 메인 추가 ##################
        yolo_size = Yolo_m.get_mission(class_name)
        print(class_name, yolo_size)
        if yolo_flag and yolo_size != None :
            yolo_mission(cte)
        ##############################################

        elif fail_count >2 :
            #if dir_order[dir_count] == 'right' :
            drive(50,20) #drive(50, 20)
            #else :
            #   drive(-40,15)
        else :          
            drive(cte*0.4, 15) #drive(cte*0.4,15)
        cv2.imshow("hough", hough_img)
        rate.sleep()	
        cv2.waitKey(1)



