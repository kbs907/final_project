#! /usr/bin/env python

import rospy
import cv2
from darknet_ros_msgs.msg import BoundingBoxes

class YoloModule:
    def __init__(self):
        self.boxdata = None

    def set_boxdata(self, boxdata):
        self.boxdata = boxdata

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
    def get_mission(self, class_name):
      if self.boxdata is not None:
          for i in self.boxdata.bounding_boxes:
              if i.Class == class_name :
                  #print(class_name, i.xmin, i.xmax)
                  yolo_size = i.xmax - i.xmin
                  #return [class_name, i.xmin, i.xmax]
                  return yolo_size 
              else:
                  return None
     #def yolo_drive(self, class_name):
        