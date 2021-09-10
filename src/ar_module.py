#! /usr/bin/env python

import rospy, math
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class ArModule():
    arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}

    def __init__(self):
        self.arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        self.id = 0
        self.yaw = 0

    def set_arData(self, data):
	for i in data.markers :
            self.id = i.id
            self.arData["DX"] = i.pose.pose.position.x
            self.arData["DY"] = i.pose.pose.position.y
            self.arData["DZ"] = i.pose.pose.position.z

            self.arData["AX"] = i.pose.pose.orientation.x
            self.arData["AY"] = i.pose.pose.orientation.y
            self.arData["AZ"] = i.pose.pose.orientation.z
            self.arData["AW"] = i.pose.pose.orientation.w
            _, _, self.yaw = euler_from_quaternion([self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]])

    def get_yaw(self):
        return self.yaw

    def get_id(self):
        return self.id
    
    def get_ardata(self):
        return self.arData

    def get_distance(self):
        return math.sqrt(pow(self.arData["DX"], 2) + pow(self.arData["DZ"], 2))

    def is_ar(self):
       if self.get_id() == 0 and self.get_distance() < 0.2 :
           return True
       return False
       
