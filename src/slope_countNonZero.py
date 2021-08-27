#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
from xycar_msgs.msg import xycar_motor
import numpy as np
import cv2, random, math, copy

num =0

##################
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import sys
import os
import signal



image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
#Offset = 340
Offset = 350
Gap = 40

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

frame = np.empty(shape=[0])
Width = 640
Height = 480
#Offset = 340
Offset = 350
Gap = 40

# draw lines

def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img



# draw rectangle

def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)

    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)

    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 255), 2)

    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    
    cv2.line(img, (lpos, 15+offset), (rpos, 15+offset), (125,125,125), 2)

    return img



# left lines, right lines

def divide_left_right(lines):
    global Width
    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line
        if (slope < 0) and (x2 < Width/2 - 90):

            left_lines.append([Line.tolist()])

        elif (slope > 0) and (x1 > Width/2 + 90):

            right_lines.append([Line.tolist()])

    return left_lines, right_lines



# get average m, b of lines

def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2

        y_sum += y1 + y2

        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos

def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)

        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)



    return img, int(pos)



def process_image(frame):
    global Width
    global Offset, Gap
    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    
    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    #cv2.imshow('calibration', frame)

    return lpos, rpos



#############

def detect_slope(cal_image, low_threshold_value):
    global num, Offset, Gap, Width
    #x_len = 400
    offset_y = 30
    speed = 15
    lpos, rpos = process_image(cal_image)
    #x_len = rpos - lpos
    #print(x_len)
    #rpos = 440
    #lpos = 200
    #x_len = rpos- lpos
    ######frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
    center = (lpos + rpos) / 2
    #y_start = 15
    ### 나중 조절
    #slope_roi = cal_image[Offset-200 : Offset+Gap, lpos+20:rpos-20]
    slope_roi = cal_image[230 : 410, center-50:center+50]
    #stopline_roi, _, _ = set_roi(cal_image, x_len, 300, y_w)
    cv2.imshow("slope_roi",slope_roi)
#set_roi(cal_image, 250, 350, 10)
  
    #cv2.imshow("roi", stopline_roi)
    image = image_processing(slope_roi, low_threshold_value)
    cv2.imshow("HLS", image)
    #검은색 x부분
    print(cv2.countNonZero(image), 80 * (410-230) * 0.9)
    if cv2.countNonZero(image) > 80 * (410-230) * 0.9 :
        #image = cv2.rectangle(image,(lpos,300),(rpos,300+offset_y),(0,255,0),2)
        #cv2.imshow("check",image)
        print("slope")
        #print(num)
        speed = 0
        num +=1
    return speed
'''

def set_roi(frame, x_len, start_y, offset_y):

    _, width, _ = frame.shape

    start_x = int(width/2 - (x_len/2))

    end_x = int(width - start_x)

    return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

'''

def image_processing(image, low_threshold_value):
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    #_, _, B = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2LAB))
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    #_, L, _ = cv2.split(cv2.cvtColor(image, cv2.COLOR_BGR2HLS))
    #_, lane = cv2.threshold(L, low_threshold_value, 255, cv2.THRESH_BINARY)
    _, lane = cv2.threshold(L, 200, 255, cv2.THRESH_BINARY_INV)
    #cv2.imshow("L", lane)
    return lane



def calibrate_image(frame, mtx, dist, cal_mtx, cal_roi):
    height, width, _ = frame.shape
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (width, height))



#cap = cv2.VideoCapture("stopline_test.mkv")

#cap = cv2.VideoCapture("track.mkv")

#cap = cv2.VideoCapture("traffic2.mkv")

Width, Height = 640, 480

mtx = np.array([[ 364.14123,    0.     ,  325.19317],
            [0.     ,  365.9626 ,  216.14575],
            [0.     ,    0.     ,    1.     ]])

#np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])

dist = np.array([-0.292620, 0.068675, 0.006335, -0.002769, 0.000000])

#np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])

cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))
rospy.init_node("stopline")
pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
motor = xycar_motor()
#while cap.isOpened():
image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
rospy.sleep(2)

while True:
    while not image.size == (640*480*3):
        continue

    cal_image = calibrate_image(image, mtx, dist, cal_mtx, cal_roi)

    #detect_stopline(cal_image, 125)

    speed = detect_slope(cal_image, 125)
    motor.speed = 0
    motor.angle = 0 #기본 유지

    pub.publish(motor)
    cv2.imshow("simple detect", cal_image)
    #if speed == 0:
    #    rospy.sleep(5)
    #    break
  
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#rospy.spin()

      
    #_, frame = cap.read()

