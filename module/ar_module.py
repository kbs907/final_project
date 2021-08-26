import rospy, math
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

class ArModule():
    def __init__(self):
        arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
        
    def set_arData(i):
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
            
    def get_id():
        return i.id
    
    def get_distance():
        return math.sqrt(pow(arData["DX"], 2) + pow(arData["DZ"], 2))
       
