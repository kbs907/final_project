#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, math, random, time
import numpy as np

from cv_bridge import CvBridge

class ImageProcessingModule:
    ### 변수 선언 ###
    # image reading setup
    bridge = CvBridge()
    image = np.empty(shape=[0])
    cal_image = np.empty(shape=[0])
    width, height = 0, 0

    # image preprocessing params
    blur_size = 0
    canny_low = 0
    canny_high = 0

    # lane parameters
    low_slope_threshold = 0
    high_slope_threshold = 0

    # detected lane properties initialize
    lx1, lx2, rx1, rx2 = 0, 0, 0, 0
    lpos, rpos, l_avg, r_avg, road_width, top_l, bottom_l = 0, 0, 0, 0, 0, 0, 0
    detect_line = False
    l_detect = False
    r_detect = False
    l_fail_count = 0
    r_fail_count = 0

    # direction
    dir_count = 0
    dir_order = ['right', 'right', 'right', 'left']
    fail_count = 0
    corner_count = 0
    intersec_count = 0

    ## optimization params ##
    ##					   ##
    # ROI setting
    Offset = 0
    Gap = 0
        
    # lane detecting range
    l_existable_range = 0
    r_existable_range = 0
    detecting_gap = 0

    def __init__(self):
        # image reading setup
        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])
        self.cal_image = np.empty(shape=[0])
        self.blur_gray = np.empty(shape=[0])
        self.width, self.height = 640, 480

        # image preprocessing params
        self.blur_size = 5
        self.canny_low = 50
        self.canny_high = 150
        self.mtx = np.array([[ 364.14123,    0.     ,  325.19317],
                    [   0.     ,  365.9626 ,  216.14575],
                    [   0.     ,    0.     ,    1.     ]])
        self.dist = np.array([-0.292620, 0.068675, 0.006335, -0.002769, 0.000000])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width, self.height), 1, (self.width, self.height))
        
        # lane parameters
        self.low_slope_threshold = 0
        self.high_slope_threshold = 10

        # detected lane properties initialize
        self.lx1 = 0
        self.lx2 = 0
        self.rx1 = 0
        self.rx2 = 0
        self.lpos = 0
        self.rpos = 0
        self.l_avg = 0
        self.r_avg = 0
        self.road_width = 0
        self.top_l = 0
        self.bottom_l =0
        self.detect_line = False
        self.l_detect = False
        self.r_detect = False
        self.l_fail_count = 0
        self.r_fail_count = 0

        # direction
        self.dir_count = -1
        self.dir_order = ['right', 'right', 'right', 'left']
        self.fail_count = 0
        self.corner_count = 0
        self.intersec_count = 0
        ## optimization params ##
        ##					   ##
        # ROI setting
        self.Offset = 340
        self.Gap = 40
            
        # lane detecting range
        self.l_existable_range = self.width/2 - 30
        self.r_existable_range = self.width/2 + 30
        self.detecting_gap = 25
        
    def set_image(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
    def get_image_size(self):
        return self.image.size
    
    def get_corner_count(self):
        return self.corner_count

    def get_road_width(self):
        return self.road_width
        
    def to_calibrated(self, img):
        tf_image = cv2.undistort(img, self.mtx, self.dist, None, self.cal_mtx)
        return tf_image

    def get_traffic_light(self, traffic_mode):
        circles_x, circles_y = [], []
        roi = [0, 200, self.width/2 - 200, self.width/2 + 200]
        roi_img = self.image[roi[0]:roi[1], roi[2]:roi[3]]

        blur_img = cv2.medianBlur(roi_img, 5)
        hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        if traffic_mode == 'first':
            circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 15, param1=30, param2=18, minRadius=8, maxRadius=20)
        elif traffic_mode == 'second':
            circles = cv2.HoughCircles(v, cv2.HOUGH_GRADIENT, 1, 8, param1=30, param2=18, minRadius=5, maxRadius=15)
                        
        # HoughCircles에서 circles가 검출될 때
        if circles is not None:
            # circles의 차원이 3 이상(1개 검출되면 2차원)이고 검출된 circles가 3 이상일 때
            if circles.ndim == 3 and circles.shape[1] >= 3:
                circles = np.uint16(np.around(circles))
                flag = True
                
                for i in circles[0, :, 0:2]:
                    circles_x.append(i[0])
                    circles_y.append(i[1])
                
                
                if traffic_mode == 'first': 
                    if ((max(circles_x) - min(circles_x)) <= 150) and ((max(circles_x) - min(circles_x)) >= 70) and ((max(circles_y) - min(circles_y)) <= 10):
                        circles_max_x = np.argmax(circles_x)
                        green_point = circles[0, circles_max_x, :]
                        cv2.circle(roi_img,(green_point[0], green_point[1]), green_point[2],(0,255,0),2)
                        if v[green_point[1]-10: green_point[1]+10, green_point[0]-10: green_point[0]+10].mean() >= 200:               
                            return True
                            
                            
                elif traffic_mode == 'second':
                    if ((max(circles_x) - min(circles_x)) <= 90) and ((max(circles_y) - min(circles_y)) <= 10):
                        circles_max_x = np.argmax(circles_x)
                        green_point = circles[0, circles_max_x, :]
                        cv2.circle(roi_img,(green_point[0], green_point[1]), green_point[2],(0,255,0),2)
                        if v[green_point[1]-10: green_point[1]+10, green_point[0]-10: green_point[0]+10].mean() >= 200:               
                            return True
                        
        return False
        
    def detect_slope(self):
        _, baw = cv2.threshold(self.blur_gray[self.Offset : self.Offset+self.Gap, 0 : self.width], 100, 255, cv2.THRESH_BINARY)
        if cv2.countNonZero(baw) < 7000 :
            return True
        return False

    def detect_stopline(self):
        d_lpos = max(self.lpos, 0)
        d_rpos = min(self.rpos, 640)
        x_len = d_rpos - d_lpos - 20
        if d_lpos!= 0 and d_rpos != 0 and x_len > 0:
            stopline_roi = self.cal_image[360:390, d_lpos + 10 :d_rpos - 10]
            stopline_image = self.stopline_image_processing(stopline_roi)
            #cv2.imshow("bin", stopline_image)
            cNZ = cv2.countNonZero(stopline_image)
            #print(cNZ, x_len * self.Gap * 0.25,  x_len * self.Gap * 0.3,  x_len * self.Gap * 0.4)
            #print('cnz : ', cNZ, x_len * self.Gap * 0.2)
            if cNZ > x_len * self.Gap * 0.2 :
                print("stopline")
                return True
                
        return False

    def stopline_image_processing(self, stopline_image):
        blur = cv2.GaussianBlur(stopline_image, (5, 5), 0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        _, lane = cv2.threshold(gray, 125, 255, cv2.THRESH_BINARY_INV)
        #cv2.imshow("L", L)
        return lane
    
    def draw_lines(self, img, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1+self.Offset), (x2, y2+self.Offset), color, 2)
        return img

    def draw_rectangle(self, img, offset=0):
        center = (self.lpos + self.rpos) / 2

        cv2.rectangle(img, (self.lpos - 5, 15 + offset),
                            (self.lpos + 5, 25 + offset),
                            (0, 255, 0), 2)
        cv2.rectangle(img, (self.rpos - 5, 15 + offset),
                            (self.rpos + 5, 25 + offset),
                            (0, 255, 0), 2)
        cv2.rectangle(img, (center-5, 15 + offset),
                            (center+5, 25 + offset),
                            (0, 255, 0), 2)    
        cv2.rectangle(img, (315, 15 + offset),
                            (325, 25 + offset),
                            (0, 0, 255), 2)
        return img


    def divide_left_right(self, lines):
        # calculate slope & filtering with threshold
        slopes = []
        new_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                slope = 0
            else:
                slope = float(y2-y1) / float(x2-x1)
            
            if abs(slope) > self.low_slope_threshold and abs(slope) < self.high_slope_threshold:
                slopes.append(slope)
                new_lines.append(line[0])
                
        # divide lines left to right
        left_lines = []
        right_lines = []

        for j in range(len(slopes)):
            Line = new_lines[j]
            slope = slopes[j]

            x1, y1, x2, y2 = Line
            x_m = (x1 + x2)/2
            
            if (slope < 0) and (x2 < self.l_existable_range):
                if self.l_detect :
                    if (self.l_avg - self.detecting_gap < x_m) and (x_m < self.l_avg + self.detecting_gap):
                        left_lines.append([Line.tolist()])
                else :
                    left_lines.append([Line.tolist()])
            elif (slope > 0) and (x1 > self.r_existable_range):
                if self.r_detect :
                    if (self.r_avg - self.detecting_gap < x_m) and (x_m < self.r_avg + self.detecting_gap):
                        right_lines.append([Line.tolist()])
                else :
                    right_lines.append([Line.tolist()])
                       
        return left_lines, right_lines


    # get average m, b of lines
    def get_line_params(self, lines):
        # sum of x, y, m
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0

        size = len(lines)
        if size == 0:
            return 0, 0, 0

        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = float(x_sum) / float(size * 2)
        y_avg = float(y_sum) / float(size * 2)

        m = m_sum / size
        b = y_avg - m * x_avg

        return m, b, x_avg

    # get lpos, rpos
    def get_line_pos(self, lines, left=False, right=False):
        m, b, x_avg = self.get_line_params(lines)
        
        if m == 0 and b == 0:
            if left :
                return self.lx1, self.lx2, self.lpos, self.l_avg, False
            if right :
                return self.rx1, self.rx2, self.rpos, self.r_avg, False
        else:
            y = self.Gap / 2
            pos = (y - b) / m

            b += self.Offset
            x1 = (self.height - b) / float(m)
            x2 = ((self.height/2) - b) / float(m)
        return x1, x2, int(pos), x_avg, True

    # show image and return lpos, rpos
    def get_cte(self):
        self.cal_image = self.to_calibrated(self.image)
        gray = cv2.cvtColor(self.cal_image, cv2.COLOR_BGR2GRAY)
        self.blur_gray = cv2.GaussianBlur(gray,(self.blur_size, self.blur_size), 0)
        edge_img = cv2.Canny(np.uint8(blur_gray), self.canny_low, self.canny_high)
        #cv2.imshow("canny", edge_img)

        # HoughLinesP : cv2.HoughLinesP(image, rho, theta, threshold, minLineLength, maxLineGap)
        roi = edge_img[self.Offset : self.Offset+self.Gap, 0 : self.width]
        all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10) # 10, 10 , 50

        # divide left, right lines
        if all_lines is None:
            return 0, self.cal_image
        
        left_lines, right_lines = self.divide_left_right(all_lines)

        # get center of lines
        self.lx1, self.lx2, self.lpos, self.l_avg, self.l_detect = self.get_line_pos(left_lines, left=True)
        self.rx1, self.rx2, self.rpos, self.r_avg, self.r_detect = self.get_line_pos(right_lines, right=True)

        self.road_width = self.rpos - self.lpos
        #print('road_width : ' , self.road_width)        
        self.top_l = self.rx1-self.lx2
        self.bottom_l = self.rx2-self.lx1
        if not self.l_detect and not self.r_detect :
            self.fail_count += 1
            if self.fail_count == 3 :
                print('##########fail detecting line! :', self.fail_count, '##########')
                self.dir_count = (self.dir_count + 1) % 4 
                self.intersec_count += 1
            self.detect_line = False
        else :
            if not self.l_detect :
                self.lx1, self.lx2 = self.rx2 - self.bottom_l, self.rx1 - self.top_l
                self.lpos = self.rpos-400
                self.l_fail_count += 1
                self.r_fail_count = 0
                print('l_fail_count : ', self.l_fail_count)
                if self.l_fail_count == 20 :
                    self.corner_count += 1
            elif not self.r_detect :
                self.rx1, self.rx2 = self.lx2 + self.top_l , self.lx1 + self.bottom_l
                self.rpos = self.lpos+400
                self.r_fail_count += 1
                self.l_fail_count = 0
                print('r_fail_count : ', self.r_fail_count)
                if self.r_fail_count == 20 :
                    self.corner_count += 1
            else :   
                self.lx1, self.lx2 = self.rx2 - self.bottom_l, self.rx1 - self.top_l
                self.lpos = self.rpos-370
                self.l_fail_count = 0
                self.r_fail_count = 0
            self.detect_line = True
            self.fail_count = 0
        '''    
        draw_image = self.cal_image.copy()
        
        draw_image = cv2.line(draw_image, (int(self.lx1), self.height), (int(self.lx2), (self.height/2)), (255, 0,0), 3)
        draw_image = cv2.line(draw_image, (int(self.rx1), self.height), (int(self.rx2), (self.height/2)), (255, 0,0), 3)

        # draw lines
        draw_image = self.draw_lines(draw_image, left_lines)
        draw_image = self.draw_lines(draw_image, right_lines)
                                        
        # draw rectangle
        draw_image = self.draw_rectangle(draw_image, offset=self.Offset)
        cv2.imshow("draw_image", draw_image)
        cv2.waitKey(1)
             
        '''     
        center = (self.lpos + self.rpos)/2
        cte = center - self.width/2
        
        return cte, self.fail_count
