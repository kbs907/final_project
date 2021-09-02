#!/usr/bin/env python

import cv2, math, random, time
import numpy as np
import rospy
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

# mode 3, speed 40, offset 350 : p= 0.3, d= 0.7

bridge = CvBridge()
Offset = 350 #330
Gap = 40
detect_line = False
image = np.empty(shape=[0])
lx1, lx2, rx1, rx2, lpos, rpos, l_avg, r_avg, top_l, bottom_l = 0,0,0,0,0,0,0,0,0,0
dir_count = -1
dir_order = ['right','right','right','left']
fail_count = 0

def drive(Angle, Speed): 
    global pub
    
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def to_calibrated(img):
    tf_image = cv2.undistort(img, mtx, dist, None, cal_mtx)
    return tf_image

def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

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
    global Width, lpos, rpos

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
        x_m = (x1 + x2)/2
	if (slope < 0) and (x2 < Width/2 - 30):
            if detect_line :
                if (l_avg - 30 < x_m) and (x_m < l_avg + 30):
            	  left_lines.append([Line.tolist()])
            else :
                left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 30):
            if detect_line :
                if (r_avg - 30 < x_m) and (x_m < r_avg + 30):
            	  right_lines.append([Line.tolist()])
            else :
                right_lines.append([Line.tolist()])
	
	'''
        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])
	'''
    return left_lines, right_lines


# get average m, b of lines
def get_line_params(lines):
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
def get_line_pos(lines, left=False, right=False):
    global Width, Height
    global Offset, Gap
    global lx1, lx2, rx1, rx2, lpos, rpos
    m, b, x_avg = get_line_params(lines)
    
    
    if m == 0 and b == 0:
        if left :
		return lx1, lx2, lpos, l_avg, False
	if right :
		return rx1, rx2, rpos, r_avg, False
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)
    return x1, x2, int(pos), x_avg, True

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global lx1, lx2, rx1, rx2, lpos, rpos, l_avg, r_avg, top_l, bottom_l, detect_line
    global dir_count, dir_order, fail_count
    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 50
    high_threshold = 150
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    cv2.imshow("canny", edge_img)

    # HoughLinesP (cv2.HoughLinesP(image, rho, theta, threshold, minLineLength, maxLineGap)
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    cv2.imshow('roi', roi)
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10) # 10, 10 , 50

    # divide left, right lines
    if all_lines is None:
        return (0, 640), frame

    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    lx1, lx2, lpos, l_avg, l_detect = get_line_pos(left_lines, left=True)
    rx1, rx2, rpos, r_avg, r_detect = get_line_pos(right_lines, right=True)
    
    top_l = rx1-lx2
    bottom_l = rx2-lx1
    #print('top, bottop  : ', rx1-lx2, rx2-lx1)
    if not l_detect and not r_detect :
	fail_count += 1
	if fail_count == 3 :
	    print('##########fail detecting line! :', fail_count, '##########')
	    dir_count = (dir_count + 1) % 4 
	detect_line = False
    else :
        if not l_detect :
            lx1, lx2 = rx2 - bottom_l, rx1 - top_l
	    lpos = rpos-440
        elif not r_detect :
	    rx1, rx2 = lx2 + top_l , lx1 + bottom_l
	    rpos = lpos+440
        else :   
            lx1, lx2 = rx2 - bottom_l, rx1 - top_l
	    lpos = rpos-400
            detect_line = True
	fail_count = 0
    
    #print('road width : ', rpos - lpos)
    
    frame = cv2.line(frame, (int(lx1), Height), (int(lx2), (Height/2)), (255, 0,0), 3)
    frame = cv2.line(frame, (int(rx1), Height), (int(rx2), (Height/2)), (255, 0,0), 3)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    return (lpos, rpos), frame

cap = cv2.VideoCapture("track.mkv")

Width, Height = 640, 480
mtx = np.array([[ 364.14123,    0.     ,  325.19317],
                [   0.     ,  365.9626 ,  216.14575],
                [   0.     ,    0.     ,    1.     ]])
dist = np.array([-0.292620, 0.068675, 0.006335, -0.002769, 0.000000])

cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height)) 

rospy.init_node('hough_drive')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
rate = rospy.Rate(20)

while not image.size == (640*480*3):
        continue

p_gain = 0.3#0.25
d_gain = 0.7#1.7
prev_cte = 0
while not rospy.is_shutdown():
    global image
    cal_image = to_calibrated(image)
    pose, hough = process_image(cal_image)
    center = (pose[0] + pose[1])/2
    cte = center - 320
    d_term = cte - prev_cte
    prev_cte = cte
    steer = p_gain * cte + d_gain * d_term
    #print(cte*0.4)
    if fail_count >2 :
        drive(50, 30)
    else :
        drive(steer,40)
    cv2.imshow("hough", hough)
    rate.sleep()
    cv2.waitKey(1)
