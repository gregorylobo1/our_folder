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
    if(C==0):
        return[0,0,0]


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
        self.colours = []
        self.loc= [0.5,0,0]
        self.camera=[]
        self.sight=1
        #self.rawdepth=None

    def locatepers(self,msg):
        #print('looking for')

        guesspub=rospy.Publisher('/kinect_guess', Vector3, queue_size=1)

        mostcolour=0
        mostcolpos=0
        adj=math.pi/6
        newguess=[]

        if len(self.colours)<1:
            self.sight=1
            return
        for max in range(len(self.colours)):
            if(self.colours[max][3]>mostcolour):
                mostcolour=self.colours[max][3]
                mostcolpos=max

        target=self.colours[mostcolpos]
        #print(target)
        if len(self.camera)<2:
            #self.sight=1
            return

        for vl in range(48):

            consec=0

            for hl in range(160):
                pixel=self.camera[(479-vl*10),(hl*4),:]
                pixel=HSVconv(pixel)

                hdiff=abs(pixel[0]-target[0])
                sdiff=abs(pixel[1]-target[1])

                if hdiff<10 and sdiff<0.2:
                    #print('FOUND')
                    #print()
                    consec+=1

                    p1=msg.data[(479-vl*10)*640+(hl*4)*4+0]
                    p2=msg.data[(479-vl*10)*640+(hl*4)*4+1]
                    p3=msg.data[(479-vl*10)*640+(hl*4)*4+2]
                    p4=msg.data[(479-vl*10)*640+(hl*4)*4+3]

                    point=[p1,p2,p3,p4]

                    # aa=str(bytearray(list1))  # edit: this conversion wasn't needed
                    aa= bytearray(point)
                    bb=struct.unpack('<f', aa)[0]
                    if math.isnan(bb):
                        continue

                    #if (bb<self.loc[0]+0.2) or (bb>self.loc[0]-0.2):


                    x=320-(hl*4)
                    y=240-(479-vl*10)

                    xp=bb
                    yp=bb/580*x
                    zp=bb/580*y

                    xpa=math.cos(adj)*xp-math.sin(adj)*zp
                    if xpa>2.5:
                        continue
                    zpa=math.sin(adj)*xp+math.cos(adj)*zp

                    #if consec>4:
                    newguess.append([xpa/2,yp/2,zpa/2])
                else:
                    consec=0

                #if consec==10:
                #    break

        newguess=np.array([newguess])
        finalguess=np.mean(newguess[0,:,:], axis=0)
        print(finalguess)
        #print(newguess.shape)
        pos=Vector3(finalguess[0],finalguess[1],finalguess[2])
        if(newguess.shape[1]>50):
            guesspub.publish(pos)
            self.sight=1
        self.camera=[]
        #self.sight=1
        #print(pos)
        #rospy.sleep(0.5)


    def RGB_callback(self, msg):
        # "Store" message received.
        global flag

        #if key is not 0:
        #    return
        #self.orientation = msg

        print('RGB')

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
        #print('depth')


        if self.sight==0:
            #print('blind')
            #self.rawdepth=msg
            self.locatepers(msg)
            return
        if self.sight==2:
            return

        key=1
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

        #if self.sight==0:
        #    return

        self.loc=[msg.x, msg.y, msg.z]
        #print('legs: ' + str(self.loc))
        #print(msg)
        theta=math.atan2(msg.y,msg.x)/math.pi*180
        if msg.z>=3 or msg.z==-1:
            #if self.sight==1:
            print('going blind')
            self.sight=0
        if abs(theta) > 30:
            if self.sight==1:
                print('out of frame')
                self.sight = 2
        if msg.z<3 and abs(theta)<30:
            if self.sight==2:
                self.sight=1


        #else:
            #self.sight=1

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
        #fig = plt.figure()
        #ax = fig.add_subplot(111, projection='3d')
        #ax.scatter(datanew[:,0],datanew[:,1],datanew[:,2])

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
        print(ccarray)

        for m in range(K):
            print('check for: ')
            print(HSVconv(means[m,:]))

            if ccarray[m]/float(len(datanew))> 0.2 and ccarray[m]<float(len(datanew)):
                self.storemeans(HSVconv(means[m,:]))


    def storemeans(self,msg):
        print('storing')
        saved=0

        newm=np.zeros(4)
        newm[:3]=np.array(msg)
        newm[3]=1
        #newm=np.concatenate(msg,np.array([1]),axis=1)

        if len(self.colours)<1:
            print('new')
            self.colours.append(newm)
            return

        for m in range(len(self.colours)):
            mcol=self.colours[m]
            hdiff=abs(newm[0]-mcol[0])
            sdiff=abs(newm[1]-mcol[1])

            if hdiff<10 and sdiff<0.18:
                print('adjust')
                mcol[:3]=mcol[:3]*mcol[3]
                mcol+=newm
                mcol[:3]=mcol[:3]/mcol[3]
                self.colours[m]=mcol
                saved=1

        if(len(self.colours)>5):
            mostcolour=0
            mostcolpos=0
            for max in range(len(self.colours)):
                if(self.colours[max][3]>mostcolour):
                    mostcolour=self.colours[max][3]
                    mostcolpos=max

            newlist=[]
            newlist.append(self.colours[mostcolpos])
            self.colours=newlist
            print('purge')

        if saved==0:
            print('add')
            self.colours.append(newm)

        #print('final')
        #print(self.colours)

if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()
    rospy.Subscriber('/vector', Vector3, server.legset, queue_size=1)
    rospy.Subscriber('/camera/rgb/image_color/', Image, server.RGB_callback, queue_size=1, buff_size=6553600)
    rospy.Subscriber('/camera/depth/image/', Image, server.depth_callback, queue_size=1, buff_size=6553600)
    #print(HSVconv([33, 151, 200]))
    rospy.spin()
