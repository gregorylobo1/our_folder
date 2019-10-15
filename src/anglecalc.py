#!/usr/bin/env python
import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

global euler_angle


def anglec(data):

	global euler_angle

	z=data.pose.pose.orientation.z
	w=data.pose.pose.orientation.w


	euler_angle=math.asin(2*w*z)
	euler_angle=euler_angle/math.pi*180

	if(abs(z)>abs(w)):
		if(z>0):
			euler_angle=180-euler_angle
		elif(z<=0):
			euler_angle=-180-euler_angle


def receivea():

	global euler_angle


	rospy.init_node('anglec',anonymous=False)

	pose_subs=rospy.Subscriber('/RosAria/pose', Odometry, anglec, queue_size=10)

	rospy.sleep(1)

	check=0
	print("program start...")

	while not (rospy.is_shutdown()):

		print(euler_angle)
		#check=check+1
		rospy.sleep(2)



if __name__ == '__main__':
	try:
		receivea()
	except rospy.ROSInterruptException: pass
