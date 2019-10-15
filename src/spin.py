#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

global speed
global xpos
global ypos
global theta

def update(data):

	global xpos
	global ypos
	global theta

	xpos=(data.pose.pose.position.x)
	ypos=(data.pose.pose.position.y)
	z=(data.pose.pose.orientation.z)
	w=(data.pose.pose.orientation.w)
	theta=math.asin(2*w*z)
	theta=theta/math.pi*180

	if(abs(z)>abs(w)):
		if(z>0):
			theta=180-theta
		elif(z<=0):
			theta=-180-theta
	if (theta<0):
		theta=360+theta




def move():

	global speed
	global xpos
	global ypos
	global theta

	linear_speed = 0.3
	angular_speed=0



	#starts new node
	rospy.init_node('followcmds', anonymous=True)
	velocity_publisher=rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()

	pose_subs=rospy.Subscriber('/RosAria/pose', Odometry, update, queue_size=10)

	rospy.sleep(1)

	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.z=0
	vel_msg.angular.y=0
	vel_msg.angular.x=0

	print(math.atan2(0,-1)/math.pi*180)
	while not rospy.is_shutdown():

		angular_speed=angular_speed+0.00001
		vel_msg.angular.z=angular_speed

		velocity_publisher.publish(vel_msg)
		print(angular_speed)



if __name__ == '__main__':
	try:
		#Testing our function
		move()
	except rospy.ROSInterruptException: pass
