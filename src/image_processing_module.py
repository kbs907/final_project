#! /usr/bin/env python
# -*- coding: utf-8 -*-

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
        circles_x, circles_y = [], []
        roi = [0, 200, self.width/2 - 200, self.width/2 + 200]
        roi_img = self.image[roi[0]:roi[1], roi[2]:roi[3]]

        blur_img = cv2.medianBlur(roi_img, 5)
        hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        # 1st traffic light
#        circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 8, param1=30, param2=18, minRadius=5, maxRadius=15)
        # 2nd traffic light
        circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 15, param1=30, param2=18, minRadius=8, maxRadius=20)
        
        # HoughCircles에서 circles가 검출될 때
        if circles is not None:
            # circles의 차원이 3 이상(1개 검출되면 2차원)이고 검출된 circles가 3 이상일 때
            if circles.ndim == 3 and circles.shape[1] >= 3:
                circles = np.uint16(np.around(circles))
                flag = True
                
                for i in circles[0, :, 0:2]:
                    circles_x.append(i[0])
                    circles_y.append(i[1])
                
                # circles의 x값의 max와 min의 차이가 90 이하이고, y 값의 max와 min의 차이가 5 이하 일때
                # 1st_traffic
#                if ((max(circles_x) - min(circles_x)) <= 90) and ((max(circles_y) - min(circles_y)) <= 10):
                # circles의 x값의 max와 min의 차이가 70 이상 150 이하이고, y 값의 max와 min의 차이가 5 이하 일때
                # 2st_traffic
                if ((max(circles_x) - min(circles_x)) <= 150) and ((max(circles_x) - min(circles_x)) >= 70) and ((max(circles_y) - min(circles_y)) <= 10):
                    # circle의 max 값인 인덱스가 초록불
                    circles_max_x = np.argmax(circles_x)
                    green_point = circles[0, circles_max_x, :]
                    print('green_point_v_mean: ', v[green_point[1]-10: green_point[1]+10, green_point[0]-10: green_point[0]+10].mean())
                    cv2.circle(roi_img,(green_point[0], green_point[1]), green_point[2],(0,255,0),2)
oyAllWindows()
                    
                    if v[green_point[1]-10: green_point[1]+10, green_point[0]-10: green_point[0]+10].mean() >= 200:
                        print('circles', circles)                    
                        return True
                        
        return False
