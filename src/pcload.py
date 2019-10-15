#!/usr/bin/env python
#import libraries for rospy and math
import rospy
import math
import numpy as np

import random
import ctypes
import struct

from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

flag=0


def kmeans(data,K,iterations):


    means=[]
    ccarray=np.empty(K)

    length=len(data)
    data=np.array(data)

    for mean in range(K):
        r=random.randint(1,255)
        g=random.randint(1,255)
        b=random.randint(1,255)
        means.append([r,g,b])

    means=np.array(means)




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

            #print(i)



            alloc.append(minmean)

        alloc=np.array(alloc)

        #print("here")

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

    rs=0
    bs=0
    gs=0

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
        #kmean_in.append([r, g, b])

        rs=rs+r
        gs=gs+g
        bs=bs+b

    kmean_in=[rs/len(data), gs/len(data), bs/len(data)]

	#print(kmean_in)
    return(kmean_in)

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

            #if(x<xdat+0.3) and (x>xdat-0.3) and (y<ydat+0.3) and (y>ydat-0.3) and zdat==1:
            npoint=[x,y,z]
            xyz.append(npoint)
                #colours.append(rgb)

def loadim(data):

    global flag

    #info=data.data
    #print("height: ", data.height)
    #print("Width: ", data.width)
    #print("Step: ", data.step)

    #length=len(list(data.data))
    #print("shape is: ", length)
    final_image=np.empty([data.height, data.width, 3])

    for r in range(data.height):
        for c in range(data.width):
            for rgbval in range(3):
                final_image[r,c,rgbval]=ord(data.data[r*data.width+c+rgbval*data.width*data.height])

    r=np.array(final_image[:,:,0])
    r=r.flatten()
    g=np.array(final_image[:,:,1])
    g=g.flatten()
    b=np.array(final_image[:,:,2])
    b=b.flatten()

    cdata=[]
    count=0
    for dat in range(len(r)):
        if count%100==0:
            p=[r[dat], g[dat], b[dat]]
            cdata.append(p)
        count+=1

    #print(r[2])
    kmeans(cdata,2,10)


def load():

    rospy.init_node('testpc', anonymous=True)

    pose_subs=rospy.Subscriber('/camera/depth_registered/points', PointCloud2, update, queue_size=1)
    camera_sub=rospy.Subscriber('/camera/rgb/image_color/', Image, loadim, queue_size=1)

    while not rospy.is_shutdown():
		#print(pose_subs)
        rospy.spin()

if __name__ == '__main__':
	try:
		#Testing our function
		load()
	except rospy.ROSInterruptException: pass
