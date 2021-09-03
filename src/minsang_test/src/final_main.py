#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image, Imu
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from visualization_msgs.msg import Marker, MarkerArray
import hough_drive_t
import hough_drive_s
import parallel_parking

rospy.init_node('final_main')
laserdata = None
mode = -1
imagedata = None
ardata = None
runHoughT = False
runHoughS = True
runParallel = False
roll, pitch, yaw = 0, 0, 0

time.sleep(5)

def laser_callback(data):
    global runHoughS

def img_callback(data):
    global mode, runHoughT, runHoughS, runParallel
    if runHoughT:
        hough_drive_t.img_callback(data)
    elif runHoughS:
        hough_drive_s.img_callback(data)
    elif runParallel:
        parallel_parking.img_callback(data)

def ar_callback(data):
    global runHoughT
    if runHoughT:
        hough_drive_t.ar_callback(data)

def imu_callback(data):
    global runHoughS
    if runHoughS:
        hough_drive_s.imu_callback(data)
    
#rospy.Subscriber('imu', Imu, imu_callback)
#rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_callback)
#rospy.Subscriber('/scan', LaserScan, laser_callback)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    if runHoughT:
        runHoughT = hough_drive_t.start()
    elif runHoughS:
        runHoughS = hough_drive_s.start()
    rate.sleep()

rospy.spin()








