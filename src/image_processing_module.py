#! /usr/bin/env python

import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError

class ImageProcessingModule():
    image = np.empty(shape=[0])
    bridge = CvBridge()
    width = 640
    Height = 320

    def __init__(self):
        pass
        
    def set_image(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def get_traffic_light(self):
        print("get_traffic_light()")
        roi = [0, 200, self.width/2 - 200, self.width/2 + 200]
        roi_img = self.image[roi[0]:roi[1], roi[2]:roi[3]]

        blur_img = cv2.medianBlur(roi_img, 5)
        hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)
        
        circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=25, minRadius=0, maxRadius=30)
        circles = np.uint16(np.around(circles))
        
        for i in circles[0, :]:
            circle_check_img = v[i[1]-10: i[1]+10, i[0]-10: i[0]+10]
            print(circle_check_img.mean())
          
#        if max(circle_v) <= 254:
#            pass  
#        elif np.argmax(circle_v) == 0:
#            pass
#        elif np.argmax(circle_v) == 1:
#            pass
#        elif np.argmax(circle_v) == 2:
#            pass

        cv2.imshow('roi_img', roi_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        