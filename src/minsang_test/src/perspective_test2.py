#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
rospy.init_node("t")

def callback(data):
    bgr = bridge.imgmsg_to_cv2(data, "bgr8")
    gray = cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 50
    high_threshold = 150
    canny = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)
    cv2.imshow('canny', canny)
    lines = cv2.HoughLinesP(canny, rho=1, theta=np.pi/180, threshold=100, minLineLength=150, maxLineGap=100)
    print(len(lines))
    for i in range(len(lines)):
        for x1,y1,x2,y2 in lines[i]:
            cv2.line(bgr,(x1,y1),(x2,y2),(0,0,255),2)
    cv2.imshow('hough', bgr)
    _, contours, _ = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(bgr, contours, -1, (0,0,0), 5, 4)
    cv2.imshow('bgr', bgr)
    rows,cols = bgr.shape[0:2]

    pts1 = np.float32([[490,20], [296,150], [860,150], [630,20]]) # red, green, blue, cyan
    #300 105 1120 490 630 20 296 860 150
    pts2 = np.float32([[500,0], [500,1200], [800,1200], [800,0]]) # 


    M = cv2.getPerspectiveTransform(pts1, pts2)
    print M

    dst = cv2.warpPerspective(bgr, M, (int(cols),rows*3))
    perspective = cv2.resize(dst, dsize=(640, 480), interpolation=cv2.INTER_LINEAR)
    cv2.imshow('perspective', perspective)
    canny = cv2.Canny(np.uint8(perspective), low_threshold, high_threshold)
    canny_blur = cv2.GaussianBlur(canny,(kernel_size, kernel_size), 0)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    canny_close = cv2.morphologyEx(canny_blur, cv2.MORPH_CLOSE, k)
    _, contours, _ = cv2.findContours(canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.imshow('canny_blur', canny_blur)
    cv2.imshow('contour_canny', canny)
    cv2.imshow('canny_close', canny_close)
    cv2.drawContours(perspective, contours, -1, (0,255,0), 3, 4)
    for cont in contours:
        length = cv2.arcLength(cont, True)
        area = cv2.contourArea(cont)
        
        #if not ((area > 3000) and (length > 500)):
        #if not ((2500< area < 3500) and (300 <length < 500)):
         #   continue
        #print('length : ', length, 'area : ', area)
        approx = cv2.approxPolyDP(cont, length*0.03, True)
        if len(approx) != 4:
            continue
        cv2.drawContours(perspective, cont, -1, (255,0,0), 3, 4)
        rect = cv2.minAreaRect(cont)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(perspective,[box],0,(0,0,255),2)
        ''''
        (x, y, w, h) = cv2.boundingRect(cont)
        center = (x + int(w/2), y + int(h/2))
        _, width, _ = perspective.shape
        print('center pos : ', center[0])
        #if 250 <= center[0] <= (width - 250):
        cv2.rectangle(perspective, (x, y), (x + w, y + h), (0, 255, 0), 2)
    '''
    cv2.imshow('perspective_contours', perspective)
    #cv2.imshow('origin', perspective)


    print(dst.shape)
    print(perspective.shape)

    while 1 :
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    plt.subplot(121),plt.imshow(img),plt.title('image')
    plt.subplot(122),plt.imshow(dst),plt.title('Perspective')
    plt.show()

rospy.Subscriber("/usb_cam/image_raw", Image, callback)
rospy.spin()

