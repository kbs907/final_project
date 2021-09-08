#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from xycar_msgs.msg import xycar_motor
#from std_msg.msg import Int32MultiArray

class DriveModule:
    prev_cte = 0
    p_gain, i_gain, d_gain = 0, 0, 0
    
    def __init__(self):
        #rospy.init_node("drive")
        self.p_gain, self.d_gain = 0.3, 0.7
        self.msg = xycar_motor()
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        pass

    def T_parking(self, angle):
        global pub
        for i in range(10):
	        self.msg.angle = 0
	        self.msg.speed = 0
	        self.pub.publish(self.msg)
	        rospy.sleep(0.1)
        for i in range(30):
	        self.msg.angle = angle*-150
	        self.msg.speed = -25
	        self.pub.publish(self.msg)
	        rospy.sleep(0.1)
        for i in range(20):
	        self.msg.angle = angle*75
	        self.msg.speed = -25
	        self.pub.publish(self.msg)
	        rospy.sleep(0.1)
        self.msg.angle = 0
        self.msg.speed = 0
        self.pub.publish(self.msg)
            
    def yolo_drive(self, angle, class_name, yolo_size):
        yolo_stop_size = 110

        if class_name == 'person' :
	    yolo_count = 15
        else :
            yolo_count = 18

        if  yolo_size < yolo_stop_size:
            print("YOLO")
            self.drive(angle, 15)
        elif yolo_size >= yolo_stop_size:
            for _ in range(yolo_count):
                #print(c_s, yolo_size, "dd")
                self.drive(angle, 12)
                rospy.sleep(0.05)
            for _ in range(50):
                self.drive(angle,0)
                rospy.sleep(0.05)
            print("Stop")
            if class_name == 'person':
                class_name = 'cat'
                do_yolo_stop = True
            else:
                do_yolo_stop = False

        return do_yolo_stop, class_name

    def cut_in(self, road_width, ultra_msg):
        if road_width < 300 and (ultra_msg[0] < 30 or ultra_msg[-1] < 80):
            return True
        return False
    
    def stop(self) :
        return (0, 0)
    
    def stop_nsec(self, stop_time) :
        start_time = time.time()
        self.msg.angle = 0
        self.msg.speed = 0
        self.pub.publish(self.msg)
        while (time.time() <= start_time + stop_time):
            pass
    
    def drive(self, angle, speed) :
        self.msg.angle = angle
        self.msg.speed = speed
        self.pub.publish(self.msg)
    
    def Hough_drive(self, cte, fail_count):
        self.prev_cte = cte
        d_term = cte - self.prev_cte
        
        steer = self.p_gain * cte + self.d_gain * d_term
        
        if fail_count > 2:
            return (50, 30)
	'''
        else:
            if stopline:
            		print('stopline!')
               
            		for _ in range(60):
              			return (steer, 0)
        '''
        return (steer, 40)
