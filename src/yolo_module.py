#! /usr/bin/env python

import rospy
import cv2
from darknet_ros_msgs.msg import BoundingBoxes

class YoloModule:
    boxdata = None
    yolo_stop_size = 0

    def __init__(self):
        self.boxdata = None
        self.yolo_stop_size = 110

    def set_boxdata(self, boxdata):
        self.boxdata = boxdata

    def car_avoid(self, lpos, rpos):
        for i in self.boxdata.bounding_boxes :
            if i.Class == "car" :
                if (i.xmin+i.xmax)/2 < 320 :
                    yolo_lpos = i.xmax
                    yolo_rpos = 0
                else :
                    yolo_lpos = 0
                    yolo_rpos = i.xmin
                if yolo_lpos != 0:
                    lpos = yolo_lpos+55
                    rpos += 55
                elif yolo_rpos != 0:
                    rpos = yolo_rpos-55
                    lpos -= 55

        center = (lpos+rpos)/2
        cte = center - 320
        steer = (cte*0.4)

        return steer

    def get_car_center(self):
        if self.boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "car":
                    return (i.xmax + i.xmin)/2
                    
    def get_car_width(self):
        if self.boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "car":
                    return (i.xmax - i.xmin)
    '''         
    def get_person_x_position(self):
        if self.boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "person":
                    return [i.xmin, i.xmax]
                    
    def get_mission(self):
        if self.boxdata is not None:
            for i in boxdata.bounding_boxes:
                if i.Class == "cat":
                    return ["cat", i.xmin, i.xmax]
                if i.Class == "dog":
                    return ["dog", i.xmin, i.xmax]
                if i.Class == "cow":
                    return ["cow", i.xmin, i.xmax]
    '''
    def get_size(self, class_name):
        if self.boxdata is not None:
            for i in self.boxdata.bounding_boxes:
                print(i)
                if i.Class == class_name :
                    #print(class_name, i.xmin, i.xmax)
                    yolo_size = i.xmax - i.xmin
                    #return [class_name, i.xmin, i.xmax]
                    return yolo_size 
                else:
                    return None
