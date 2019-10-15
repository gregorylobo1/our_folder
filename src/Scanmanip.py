#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

def manipulater(data):

    amin=data.angle_min
    amax=data.angle_max
    ainc=data.angle_increment

    pnum=(amax-amin)/ainc

    ranges=data.ranges

    print(ranges[100])
    return(1)

def Read():

    rospy.init_node('laser1reader', anonymous=False)

    scan_sub=rospy.Subscriber('/scan', LaserScan, manipulater)

    rospy.sleep(1)
    p



if __name__ == '__main__':
    try:
        #Testing our function
        Read()
    except rospy.ROSInterruptException: pass
