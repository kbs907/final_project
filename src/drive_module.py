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

    def drive(self, angle, speed) :
        self.msg.angle = angle
        self.msg.speed = speed

        self.pub.publish(self.msg)
       
    def stop(self) :
        return (0, 0)
    
    def stop_nsec(self, stop_time) :
        start_time = time.time()
        self.drive(0,0)
        
        while (time.time() <= start_time + stop_time):
            pass
    
    def slope_drive(self, angle) :
        for _ in range(17): #20
            self.drive(0, 40)
            rospy.sleep(0.1)
        for _ in range(6):
            self.drive(angle, -40) #-angle
            rospy.sleep(0.1)
            
    def start_T_parking(self):
        for i in range(15): # motor change
            self.drive(-5, 10)
            rospy.sleep(0.1)
            
        for i in range(30): #17
            self.drive(50, 10) #50,10
            rospy.sleep(0.1)

        for i in range(10):
            self.drive(0, 0)
            rospy.sleep(0.1)

    def T_parking(self, dist, degree):
        for i in range(30):
            self.drive(-(degree*2), -35)
            rospy.sleep(0.1)
        for i in range(30):
            self.drive(degree/2, -35)
            rospy.sleep(0.1)

    def again_T_parking(self, dist, degree):
        num = 0
        new_degree = degree
        
        while abs(new_degree) >= 3:
            if new_degree >= 0:
                new_degree -= 1
            else:
                new_degree += 1
            self.drive(new_degree, 10)
            rospy.sleep(0.1)
            num += 1

        for i in range(num):
            self.drive(-(degree/2), -30)
            rospy.sleep(0.1)

    def end_T_parking(self, degree):
        self.stop_nsec(3)
        for i in range(20):
            self.drive(degree * 2, 10)
            rospy.sleep(0.1)
        for i in range(30):
            self.drive(-50, 10)
            rospy.sleep(0.1)
        for i in range(20):
            self.drive(20, -35)
            rospy.sleep(0.1)
        for i in range(5):
            self.drive(-50, 10)
            rospy.sleep(0.1)
        for i in range(5):
            self.drive(20, -35)
            rospy.sleep(0.1)

    def parallel_parking(self) :
        for _ in range(17): #22
            self.drive(0,15)
            rospy.sleep(0.1)
        for _ in range(10):
            self.drive(-40,15)
            rospy.sleep(0.1)
        for _ in range(20): #22
            self.drive(0,-50)
            rospy.sleep(0.1)
        for _ in range(10):
            self.drive(-30,-40)
            rospy.sleep(0.1)

    def yolo_drive(self, angle, class_name, yolo_size):
        yolo_stop_size = 120 #110
        do_yolo_stop = True

        if class_name == 'person' :
	        yolo_count = 20 #18 #15
        else :
            yolo_count = 20 #21 #18

        if  yolo_size < yolo_stop_size:
            self.drive(angle, 15)
        elif yolo_size >= yolo_stop_size:
            for _ in range(yolo_count):
                self.drive(angle, 12)
                rospy.sleep(0.05)

            for _ in range(100):
                self.drive(angle,0)
                rospy.sleep(0.05)

            if class_name == 'person':
                class_name = 'cat'
                do_yolo_stop = True
            else:
                do_yolo_stop = False

        return do_yolo_stop, class_name

    def cut_in(self, road_width):
        if road_width < 250 :
                return 0, False
        else :
            return 16, True

    
    def Hough_drive(self, cte, fail_count):
        d_term = cte - self.prev_cte
        self.prev_cte = cte
        
        
        steer = self.p_gain * cte + self.d_gain * d_term
        try:
            if fail_count > 2:
                return (50, 15)
        except:
            pass

        return (steer, 15)
