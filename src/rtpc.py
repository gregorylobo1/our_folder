#!/usr/bin/env python
import rospy
import math
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
import math

from struct import unpack

import random
import ctypes
import struct

from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

global flag

flag=2
key=0

def HSVconv(vec):

    #print(vec)
    vec=np.ndarray.tolist(vec)
    #print(vec)

    #vec=float(vec)

    minval=float(min(vec))
    mincol=vec.index(minval)
    maxval=float(max(vec))
    maxcol=vec.index(maxval)

    C=maxval-minval


    H=60*(maxcol*2+(vec[(maxcol+1)%3]-vec[(maxcol+2)%3])/C)
    #print(H)

    S=C/maxval
    V=maxval/255

    if H<0:
      H=H+360

    #print(vec[(maxcol+1)%3])

    return [H, S, V]






class Server:
    def __init__(self):
        self.person = {}
        self.colours = None
        self.loc= [0.5,0,0]
        self.camera=None
        self.sight=1

    def RGB_callback(self, msg):
        # "Store" message received.
        global flag

        if key is not 0:
            return
        #self.orientation = msg

        #print('RGB')

        count=0
        data=msg
        im= np.empty([data.height,data.width,3])
        global flag

        for r in range(data.height):
            for c in range(data.width):
                for i in range(3):
                    im[r,c,i]=ord(data.data[c*3+r*3*640+(2-i)])
        #print('done')
        self.camera=im
        if flag==1:
            flag=2
            print(im.shape)
            print('first digit: ')
            print(ord(data.data[0+639*3]))
            print(ord(data.data[1+639*3]))
            print(ord(data.data[2+639*3]))
            print('second digit: ')
            print(im[479,300,0])
            print(im[479,300,1])
            print(im[470,300,2])
            plt.imshow(im)
            plt.show()

    def depth_callback(self, msg):

        data=msg
        print('depth')
        key=1

        if self.sight==0:
            print('blind')
            return

        global flag
        f=580
        w=data.width
        h=data.height
        adj=math.pi/6
        #princ=[floor(h/2),floor(w/2)]
        #im=np.zeros([h,w])
        im=[]
        raw=data.data
        pers={}
        #count=0
        c=0
        r=0

        for i in range(h*w):
            #count=count+1
            if i%1==0:
                p1=raw[i*4]
                p2=raw[i*4+1]
                p3=raw[i*4+2]
                p4=raw[i*4+3]

                point=[p1,p2,p3,p4]

                # aa=str(bytearray(list1))  # edit: this conversion wasn't needed
                aa= bytearray(point)
                bb=struct.unpack('<f', aa)[0]
                if math.isnan(bb):
                    bb=10

                #if (bb<self.loc[0]+0.2) or (bb>self.loc[0]-0.2):


                x=320-c
                y=240-r


                if(bb<2):
                    xp=bb
                    yp=bb/f*x
                    zp=bb/f*y

                    xpa=math.cos(adj)*xp-math.sin(adj)*zp
                    zpa=math.sin(adj)*xp+math.cos(adj)*zp

                    if (xpa<self.loc[0]+0.3) and (xpa>self.loc[0]-0.3) and (yp>self.loc[1]-0.3) and (yp<self.loc[1]+0.3):
                        #im.append([xpa,yp,zpa])
                        pers[tuple([r,c])]=1

            c=c+1
            if(c==640):
                c=0
                r=r+1
        key=0
        self.person=pers
        self.findperson()
        if flag==0:
            im=np.array(im)
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(im[:,0],im[:,1],im[:,2])
            plt.ylabel='x'
            plt.xlabel='y'
            plt.zlabel='bb'
            plt.show()
            flag=2

    def legset(self,msg):
        self.loc=[msg.x, msg.y, msg.z]
        #print('legs: ' + str(self.loc))
        #print(msg)
        theta=math.atan2(msg.y,msg.x)/math.pi*180
        if theta > 30 or msg.z>3:
            self.sight = 0
        else:
            self.sight=1

    def findperson(self):
        print('DOINGIT')
        global flag

        person=self.person
        unproc_image=self.camera
        check=0
        validcols=[]

        if len(person)<2:
            return

        for r in range(480):
            for c in range(640):
                if tuple([r,c]) in person:
                    if check%100==0:
                        validcols.append(list(unproc_image[r,c,:]))
                    check+=1
                else:
                    unproc_image[r,c,:]=0

        if flag==3:
            flag=3
            plt.imshow(unproc_image)
            plt.show()
            #print(validcols)
            plt.close()


        self.kmeans(validcols,2,3)


        #print(str(sum(validcols)/len(validcols)))

    def kmeans(self,data,K,iterations):
        print('means')
        data=np.array(data)
        print(data.shape[0])
        datasize=data.shape[0]
        datanew=[]

        if len(data)<100:
            return

        for i in range(datasize):

            greycheck=abs(sum(data[i,:]-data[i,0]))
            if abs(greycheck) < 70:
                continue

            #print(greycheck)
            datanew.append(data[i,:])

        if len(datanew)<20:
            return

        datanew=np.array(datanew)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(datanew[:,0],datanew[:,1],datanew[:,2])

        means=[]
        ccarray=np.empty(K)

        data=np.array(datanew)

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
                    means[m,:]=[rm/cc, gm/cc, bm/cc]
                    ccarray[m]=cc




        #print(means)
        for m in range(K):
            print(HSVconv(means[m,:]))
        print(ccarray)



if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()
    rospy.Subscriber('/vector', Vector3, server.legset, queue_size=1)
    rospy.Subscriber('/camera/rgb/image_color/', Image, server.RGB_callback, queue_size=1)
    rospy.Subscriber('/camera/depth/image/', Image, server.depth_callback, queue_size=1)
    #print(HSVconv([33, 151, 200]))
    rospy.spin()
