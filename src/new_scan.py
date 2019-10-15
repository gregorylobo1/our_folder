#!/usr/bin/env python
'''
last editted on 16/09/2019
by Phillip Luu
'''
import rospy
from sensor_msgs.msg import LaserScan
import math

rospy.init_node('scan_read')

ns_pub = rospy.Publisher("new_scan", LaserScan, queue_size = 1)

def read_cb(data):
    # reset feature variables
    angle = []
    rmin = 0.030
    rmax = 3

    # new data that is shortened
    newdata = data

    # ensure laser variables are up to date
    ang_min = data.angle_min
    ang_max = data.angle_max
    a_inc = data.angle_increment

    new_scan = list(data.ranges)
    # reject invalid data and initialise new data to be processed
    for d, data_l in enumerate(data.ranges):
        angle.append(ang_min + d*a_inc)
        if data_l < rmin or data_l > rmax or math.isnan(data_l) or d < 3:
            new_scan[d] = float('nan')

    print(len(angle))
    print('angle = ' + str(angle))
    newdata.ranges = list(new_scan)
    ns_pub.publish(newdata)

def listener():
    rospy.Subscriber("/scan", LaserScan, read_cb, queue_size = 1)

    #spin forever
    rospy.spin()

listener()
