#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from tf.transformations import *
import matplotlib.pyplot as plt

class PID():

    def __init__(self, kp, ki, kd):

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):

        self.d_error = cte - self.p_error
        self.p_error = cte
        if abs(self.i_error + cte) < 2000:
            self.i_error += cte

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error



image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 300 #300
Gap = 100
timestamp = 0
x_axis = []
y_axis = []
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll = 0
pitch = 0
yaw = 0
frame_id = -1
distance = 99
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    
def ar_callback(data):
    global arData, roll, pitch, yaw, frame_id, distance
    #print(data)
    for i in data.markers:
        frame_id = i.id
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
        distance = math.sqrt(pow(arData["DX"],2) + pow(arData["DZ"],2))

        roll, pitch, yaw = euler_from_quaternion([arData["AX"], arData["AY"], arData["AZ"], arData["AW"]])

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = 15
    if Angle > 25:
        for i in range(4000):
            pub.publish(msg)
    else:
        pub.publish(msg)

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
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
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

    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    polygon = np.array([[30,400], [100,300], [540,300], [610,400]])
    ROI = np.zeros_like(gray)
    cv2.fillConvexPoly(ROI, polygon, 1)
    
    roi = cv2.bitwise_and(gray, gray, mask=ROI)
    cv2.imshow('roi', roi)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(roi,(kernel_size, kernel_size), 0)

    roi = cv2.bitwise_not(roi)

    cv2.bitwise_not(roi, roi, mask=ROI)

    ret, thresh = cv2.threshold(roi,150,255,cv2.THRESH_BINARY_INV)

    all_lines = cv2.HoughLinesP(thresh,1,np.pi/180,30,maxLineGap=1)
    framecopy = frame.copy()

    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    avgx1 = 0
    avgx2 = 0
    avgy1 = 0
    avgy2 = 0
    linesum = 0

    for line in left_lines:
    	x1, y1, x2, y2 = line[0]
        linesum += 1
        avgx1 += x1
        avgx2 += x2
        avgy1 += y1
        avgy2 += y2
    
    if linesum != 0:
        cv2.line(framecopy, (avgx1/linesum, avgy1/linesum), (avgx2/linesum, avgy2/linesum), (255, 0, 0), 3)
        avgleftpos = (avgx2+avgx1)/(2*linesum)
    else:
        avgleftpos = 0
    cv2.circle(framecopy, (avgleftpos, 350), 10, (0, 255, 0), 2)

    avgx1 = 0
    avgx2 = 0
    avgy1 = 0
    avgy2 = 0
    linesum = 0

    for line in right_lines:
    	x1, y1, x2, y2 = line[0]
        linesum += 1
        avgx1 += x1
        avgx2 += x2
        avgy1 += y1
        avgy2 += y2
    
    if linesum != 0:
        cv2.line(framecopy, (avgx1/linesum, avgy1/linesum), (avgx2/linesum, avgy2/linesum), (255, 0, 0), 3)
        avgrightpos = (avgx2+avgx1)/(2*linesum)
    else:
        avgrightpos = 640
    cv2.circle(framecopy, (avgrightpos, 390), 10, (0, 255, 0), 2)

    cv2.imshow('frame', framecopy)
    cv2.imshow('thresh', thresh)

    return avgleftpos, avgrightpos
    
def start():
    global pub
    global image
    global cap
    global Width, Height
    global x_axis, y_axis
    global arData, distance, frame_id

    pid = PID(0.05, 0.0, 0.25)
    if frame_id == 0:
        if abs(arData["DX"]-arData["DZ"]) < 0.5:
            for i in range(20):
                msg = xycar_motor()
                msg.angle = 25
                msg.speed = 20
                pub.publish(msg)
                rospy.sleep(0.1)
            return False
	
    if not image.size == (640*480*3):
        #print('t')
        return True

    lpos, rpos = process_image(calibrate_image(image))

    center = (lpos + rpos) / 2
    error = -(Width/2 - center - 15)

    angle = pid.pid_control(error)

    if lpos == 0.0 or rpos > 630.0:
        drive(error-15, 20)
    else:
        drive(angle, 20)
        

    return True





