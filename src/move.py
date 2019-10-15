#!/usr/bin/env python
#import libraries for rospy and math
import rospy
import math

#import rosmsg data to be used
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#define global variabls
global speed
global xpos
global ypos
global theta

def update(data):

	global xpos
	global ypos
	global theta

	#find pose values
	xpos=(data.pose.pose.position.x)
	ypos=(data.pose.pose.position.y)
	z=(data.pose.pose.orientation.z)
	w=(data.pose.pose.orientation.w)
	theta=math.asin(2*w*z)/math.pi*180

	#conversion of angle
	if(abs(z)>abs(w)):
		if(z>0):
			theta=180-theta
		elif(z<=0):
			theta=-180-theta
	if (theta<0):
		theta=360+theta

#Main function for moving
def move():

	global speed
	global xpos
	global ypos
	global theta

	#defining speeds
	angular_speed=1

	speed=0
	xpos=0
	ypos=0
	theta=0

	#starts new node
	rospy.init_node('followcmds', anonymous=True)

	#velocity publisher for pioneer
	velocity_publisher=rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()

	#pose subsciber calls update function
	pose_subs=rospy.Subscriber('/RosAria/pose', Odometry, update, queue_size=10)

	#get input
	rospy.sleep(1)
	print("Program start....")
	xgoal=input("Enter X loc: ")
	xgoal=(xpos+xgoal)
	print(xgoal)
	ygoal=input("Enter Y loc: ")
	ygoal=(ypos+ygoal)

	#calculate goal angle relative to the robot's pose
	thetag=math.atan2((ygoal-ypos),(xgoal-xpos))/math.pi*180
	print(thetag)
	if(thetag>0):
		vel_msg.angular.z=angular_speed
	elif(thetag<0):
		vel_msg.angular.z=-angular_speed
	if (thetag<0):
		thetag=360+thetag
	thetag=thetag+theta
	if (thetag>=360):
		thetag=thetag-360

	angleflag=1
	distflag=0

	#set velocity values
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x=0
	vel_msg.angular.y=0


	while not rospy.is_shutdown():

		#move to correct angle
		while(angleflag) and not rospy.is_shutdown():
			velocity_publisher.publish(vel_msg)
			print("goal")
			print(thetag)
			print("current")
			print(theta)
			print(thetag-theta)

			#Reduce speed when close
			if (abs(thetag-theta)<10):
				if(thetag>theta):
					vel_msg.angular.z=0.1
				elif(thetag<theta):
					vel_msg.angular.z=-0.1
			#stop within 2 degrees
			if (abs(thetag-theta)<2):
				vel_msg.angular.z=0
				angleflag=0

		#set linear velocity
		vel_msg.linear.x=0.1

		#move to location within 20cm
		while(distflag) and not rospy.is_shutdown():
			velocity_publisher.publish(vel_msg)
			print("goal")
			print(xgoal)
			print("current")
			print(xpos)
			print(xgoal-xpos)
			if (abs(xgoal-xpos)<0.2):
				vel_msg.linear.x=0
				distflag=0




if __name__ == '__main__':
	try:
		#Testing our function
		move()
	except rospy.ROSInterruptException: pass
