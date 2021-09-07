#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
#from std_msg.msg import Int32MultiArray

class DriveModule:
    prev_cte = 0
    p_gain, i_gain, d_gain = 0, 0, 0
    
    def __init__(self):
        self.p_gain, self.d_gain = 0.3, 0.7
        pass

    def T_parking(self):
        pass

    def cut_in(self, road_width, ultra_msg):
        if road_width < 300 and (self.ultra_msg[0] < 30 or self.ultra_msg[-1] < 80):
            return True
        return False
        
    def Hough_drive(self, cte, fail_count, stopline):
        self.prev_cte = cte
        d_term = cte - self.prev_cte
        
        steer = self.p_gain * cte + self.d_gain * d_term
        
        if fail_count > 2:
            return (50, 30)
        else:
            if stopline:
            		print('stopline!')
               
            		for _ in range(60):
              			return (steer, 0)
                    
            return (steer, 40)
