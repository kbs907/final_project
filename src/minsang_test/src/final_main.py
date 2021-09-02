#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from visualization_msgs.msg import Marker, MarkerArray
import hough_drive

rospy.init_node('final_main')
laserdata = None
mode = -1
imagedata = None
ardata = None
runHoughT = True

time.sleep(5)

def lasercallback(data):
    global laserdata, mode
    laserdata = data

def img_callback(data):
    global mode
    hough_drive.img_callback(data)

def ar_callback(data):
    global mode
    hough_drive.ar_callback(data)
    
rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_callback)
rospy.Subscriber('/scan', LaserScan, lasercallback)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

while not rospy.is_shutdown():
    if runHoughT:
        runHoughT = hough_drive.start()
    rospy.sleep(0.1)

rospy.spin()








