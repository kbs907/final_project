#! /usr/bin/env python


import rospy
from std_msgs.msg import String

rospy.init_node('mission')
pub = rospy.Publisher('mission', String, queue_size = 1)

while not rospy.is_shutdown():
    key = raw_input("************** Input Mission ***************\n 1: cut-in, 2: mode2, 3: YOLO, 4: T-parking, 5: mode3, 6: mode4, 7: avoid car, 8: find stopline, 9:p-parking :, ")
    if 0 < int(key) < 10 :
        pub.publish(key)
        print("mission change", key)
    else :
        print("mission not change")
