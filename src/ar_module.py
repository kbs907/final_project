#! /usr/bin/env python

import rospy, math
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class ArModule():
    arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
    i = 0

    def __init__(self):
        self.arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        self.i = 0
        self.roll, self.pitch, self.yaw = 0, 0, 0

    def set_arData(self, data):
        self.i = data.markers
        print(self.i)

        self.arData["DX"] = self.i.pose.pose.position.x
        self.arData["DY"] = self.i.pose.pose.position.y
        self.arData["DZ"] = self.i.pose.pose.position.z

        self.arData["AX"] = self.i.pose.pose.orientation.x
        self.arData["AY"] = self.i.pose.pose.orientation.y
        self.arData["AZ"] = self.i.pose.pose.orientation.z
        self.arData["AW"] = self.i.pose.pose.orientation.w
        self.roll, self.pitch, self.yaw = euler_from_quaternion([self.arData["AX"], self.arData["AY"], self.arData["AZ"], self.arData["AW"]])

    def get_yaw(self):
        return self.yaw

    def get_id(self):
        return self.i.id
    
    def get_distance(self):
        return math.sqrt(pow(self.arData["DX"], 2) + pow(self.arData["DZ"], 2))

    def is_ar(self):
       if self.get_id() == 0 and self.get_distance() < 0.6 :
           return True
       return False
       
