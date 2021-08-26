import ...

lidardata = None
imudata = None
ardata = None
ultrasounddata = None
camimage = None

mode = 1

motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
rospy.Subscriber("lidar")
rospy.Subscriber("imu")
rospy.Subscriber("ar_pose_marker", AlvarMarker)

if mode == 0:
	if checkStopLine(camimage):
		mode = 2
	else:
		houghDrive(camimage)

elif mode == 1:
	if checkTrafficLight(camimage):
		mode = changeLanes(ultrasounddata, lidardata)

elif mode == 2:
	if checkTrafficLight(camimage):

elif mode == 3:








