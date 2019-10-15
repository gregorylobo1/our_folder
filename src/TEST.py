#!/usr/bin/env python
#import libraries for rospy and math
import rospy
import math
import numpy as np


import random
import ctypes
import struct

#import rosmsg data to be used
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

global t1
global t2
global xdat
global ydat
global zdat
global person

xdat=1
ydat=1
zdat=1


def Kmean(data, K, iterations):

	means=[]
	ccarray=np.empty(K)

	data=np.array(data)

	for mean in range(K):
		r=random.randint(1,255)
		g=random.randint(1,255)
		b=random.randint(1,255)
		means.append([r,g,b])

	means=np.array(means)
	length=(data).shape[0]

	for iter in range(iterations):

		alloc=[]

		for i in range(length):

			minmean=0
			minval=3000000

			for j in range(K):
				dist=data[i,:]-means[j,:]
				dist=np.dot(dist,dist)
				dist=math.sqrt(dist)

				if dist < minval:
					minval=dist
					minmean=j+1

			alloc.append(minmean)

		alloc=np.array(alloc)

		for m in range(K):
			rm=0
			gm=0
			bm=0
			cc=0
			for c in range(alloc.shape[0]):
				if alloc[c]==(m+1):
					rm+=data[c,0]
					gm+=data[c,1]
					bm+=data[c,2]
					cc+=1

			if(cc>0):
				means[m,:]=[rm/cc, bm/cc, gm/cc]
				ccarray[m]=cc





	print(means)
	print(ccarray)

def means_track(data):
	#pub1=rospy.Publisher("mycolors", Vector3, queue_size=1)

	kmean_in=[]
	r=0
	g=0
	b=0
	ic=1

	for i in range(len(data)):
		#data[i]=int(data[i])
		#r=((data[i])>>16)
		#g=((data[i])>>8)
		#b=((data[i]))

        # cast float32 to int so that bitwise operations are possible
		s = struct.pack('>f' ,data[i])
		i = struct.unpack('>l',s)[0]
		# you can get back the float value by the inverse operations
		pack = ctypes.c_uint32(i).value
		r = (pack & 0x00FF0000)>> 16
		g = (pack & 0x0000FF00)>> 8
		b = (pack & 0x000000FF)
		kmean_in.append([r, g, b])


	#print(kmean_in)
	return(kmean_in)

def valset(data):
	global xdat
	global ydat

	xdat=data.y
	ydat=data.x

def publishdata(xyz):

	pub=rospy.Publisher("mypoints", PointCloud2, queue_size=1)

	#pts=[]

	#for i in range(len(x)):
	#	pt=[float(x[i]), float(y[i]), float(z[i])]
	#	pts.append(pt)
		#print(pt)

	fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)]
          #PointField('rgb', 16, PointField.FLOAT32,1)]

	header = Header()

	header.frame_id = "map"

	mypc2 = pc2.create_cloud(header, fields, xyz)

	mypc2.header.stamp = rospy.Time.now()
	pub.publish(mypc2)
	#print "sending..."
	#print(mypc2 )



def update(data):

	xyz=[]
	full=[]
	colours=[]

	count=0
	adj=-math.pi/3

	for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
		#print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
		#Here p[0] is X p[1] is Y and P[2] is Z for each point
		count=count+1

		if(count%5==0):
			x=(p[0])
			y=(p[1]*math.cos(adj)-p[2]*math.sin(adj))
			z=(p[2]*math.cos(adj)+p[1]*math.sin(adj))
			#rgb=(p[3])

			if(x<xdat+0.3) and (x>xdat-0.3) and (y<ydat+0.3) and (y>ydat-0.3) and zdat==1:
				npoint=[x,y,z]
				xyz.append(npoint)
				#colours.append(rgb)

	publishdata(xyz)

	#if np.array(xyz).shape[0]>100:
	#	coldata=means_track(colours)
	#	Kmean(coldata,4,10)



def find():


	#starts new node
	rospy.init_node('kinect_pos', anonymous=True)

	t1=rospy.Time.now()
	t2=t1
	#pose subsciber calls update function
	pose_subs=rospy.Subscriber('/camera/depth_registered/points', PointCloud2, update, queue_size=1)
	leg_subs=rospy.Subscriber('/vector', Vector3, valset, queue_size=1)



	while not rospy.is_shutdown():
		#print("waiting...")
		#rospy.sleep(1)
		rospy.spin()


if __name__ == '__main__':
	try:
		#Testing our function
		find()
	except rospy.ROSInterruptException: pass
