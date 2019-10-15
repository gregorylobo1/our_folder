#!/usr/bin/env python
# attempts to subscribe and print data from the pointcloud data

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

global ang_min
global ang_max
global a_inc
radius = 0.55                # swing diameter of 52 cm, with tolerancing of +3cm
rmin = 0.060
rmax = 4

rospy.init_node('gap_find')

ns_pub = rospy.Publisher("vector", Vector3, queue_size = 1)

def read_cb(data):
    global ang_min
    global ang_max
    global a_inc
    global radius
    global rmin
    global rmax

    # reset feature variables
    angle = []
    far = 0
    far_a = 0
    vector = Vector3()

    # ensure laser variables are up to date
    ang_min = data.angle_min
    ang_max = data.angle_max
    a_inc = data.angle_increment

    # initialise scan data to be mutable object
    scan_d = list(data.ranges)

    # reject invalid data and initialise new data to be published
    for c, data in enumerate(data.ranges):
        angle.append(ang_min + c*a_inc)
        if data < rmin or data > rmax or math.isnan(data):
            scan_d[c] = float('nan')

    new_scan = list(scan_d)
    for i, scan in enumerate(scan_d):
        if math.isnan(scan):
            continue
        theta = math.atan(0.5*radius/scan)
        devi = int(math.floor(theta/a_inc))
        for x in range(-devi,devi):
            # check if valid value in array
            if (i+x) < 0 or (i+x) > (len(scan_d) - 1):
                continue
            r_dash = scan/math.cos(x*a_inc)
            if (scan_d[i+x] > r_dash and r_dash < new_scan[i+x]) or math.isnan(scan_d[i+x]):
                new_scan[i+x] = r_dash

        if math.isnan(new_scan[i]):
            far = 4
            far_a = angle[i]
        elif new_scan[i] > far and new_scan[i] > 0.5*radius:
            far = r_dash
            far_a = angle[i]
        elif new_scan[i] < 0.5*radius:
            far = 4
            far_a = 3.1416

    vector.x = r_dash * math.cos(far_a)
    vector.y = r_dash * math.sin(far_a)
    vector.z = 1

    #print(vector)
    ns_pub.publish(vector)

def listener():
    rospy.Subscriber("/scan", LaserScan, read_cb, queue_size = 1)

    #spin forever
    rospy.spin()

listener()
