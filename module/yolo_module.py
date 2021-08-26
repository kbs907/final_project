import rospy
import cv2
from darknet_ros_msgs.msg import BoundingBoxes

class YoloModule:
    def __init__(self, boxdata):
        self.boxdata = boxdata
        pass

    def get_car_center():
        if boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "car"
                    return (i.xmax + i.xmin)/2
                    
    def get_car_width():
        if boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "car"
                    return (i.xmax - i.xmin)
                
    def get_person_x_position():
        if boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "person"
                    return [i.xmin, i.xmax]
                    
    def get_mission():
        if boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "cat"
                    return ["cat", i.xmin, i.xmax]
                if i.Class == "dog"
                    return ["dog", i.xmin, i.xmax]
                if i.Class == "cow"
                    return ["cow", i.xmin, i.xmax]
                    
    def get_traffic_light_roi_position():
        if boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "traffic light"
                    return [i.xmin, i.ymin, i.xmax, i.ymin]
                    
