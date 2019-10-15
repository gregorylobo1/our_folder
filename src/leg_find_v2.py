#!/usr/bin/env python
# attempts to subscribe and print data from the pointcloud data

#leg finding script - version 2
#last editted on 02/08/2019
#by Phillip Luu

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math

global ang_min
global a_inc
global prev_x
global prev_y
global prev_z
radius = 0.55                # swing diameter of 52 cm, with tolerancing of +3cm
rmin = 0.040
rmax = 2.5
prev_x = 0
prev_y = 0
prev_z = 0

rospy.init_node('leg_find')

leg_pub = rospy.Publisher("vector", Vector3, queue_size = 1)

def polar_dist(r1, th1, r2, th2):
    #calculate polar distance between points
    global ang_min
    global a_inc

    dist = math.sqrt(r1*r1 + r2*r2 - 2*r1*r2*math.cos(th1 - th2))
    return dist

def read_cb(data):
    global ang_min
    global a_inc
    global radius
    global rmin
    global rmax
    global prev_x
    global prev_y
    global prev_z

    # define constants
    leg_size_min = 0.05
    leg_size_max = 0.15
    depth_min = 0.01
    depth_max = 0.07
    connect_dist = 0.05

    # reset feature variables
    angle = []
    far = 0
    vector = Vector3()
    temp_i = 0
    temp_f = 0
    clear_flag = 0
    leg = []
    leg_c = 0

    # ensure laser variables are up to date
    ang_min = data.angle_min
    ang_max = data.angle_max
    a_inc = data.angle_increment

    # initialise scan data to be mutable object
    new_scan = list(data.ranges)
    print('New Leg')

    # reject invalid data and initialise new data to be processed
    for d, data_l in enumerate(data.ranges):
        angle.append(ang_min + d*a_inc)
        if data_l < rmin or data_l > rmax or math.isnan(data_l):
            new_scan[d] = float('nan')

    # attempt to algorithm
    for c, scan in enumerate(new_scan):
        if c == (len(new_scan)-1):
            break
        if abs(new_scan[c] - new_scan[c+1]) < connect_dist:
            if temp_i == 0:
                temp_i = c
            continue
        elif temp_i != 0:
            temp_f = c
            clear_flag = 1
        if temp_f - temp_i > 5:
            print('Entry {:f} and {:f}'.format(angle[temp_i], angle[temp_f]))
            if leg_size_min < polar_dist(new_scan[temp_i],angle[temp_i],new_scan[temp_f],angle[temp_f]) < leg_size_max:
                mid = int(math.ceil((temp_f - temp_i)/2)+temp_i)
                if depth_min < (new_scan[temp_i] - new_scan[mid]) < depth_max and depth_min < (new_scan[temp_f] - new_scan[mid]) < depth_max:
                    ini_angle = round(180/math.pi*(angle[temp_i]),3)
                    fin_angle = round(180/math.pi*(angle[temp_f]),3)
                    leg.append([angle[mid],new_scan[mid]])
                    print('Leg between {:f} and {:f}'.format(ini_angle, fin_angle))
        if clear_flag == 1:
            temp_i = 0
            temp_f = 0
            clear_flag = 0

    # convert legs to useful information
    # leg[0][i] is the first leg entry discovered starting from negative angles
    # leg[i][0] is the angle of corresponding leg
    # leg[i][1] is the distance of corresponding leg
    if len(leg) == 0:
        if prev_x != 0 and prev_y != 0:
            x_leg = prev_x
            y_leg = prev_y
            z_leg = prev_z + 1
        else:
            x_leg = 0
            y_leg = 0
            z_leg = 0
    elif len(leg) == 1:
        x_leg = leg[0][1] * math.cos(leg[0][0])
        y_leg = leg[0][1] * math.sin(leg[0][0])
        z_leg = 1
    elif len(leg) == 2:
        if leg[1][0] - leg[0][0] < 15:
            x_leg = (leg[0][1] + leg[1][1])/2 * math.cos((leg[0][0]+leg[1][0])/2)
            y_leg = (leg[0][1] + leg[1][1])/2 * math.sin((leg[0][0]+leg[1][0])/2)
            z_leg = 1
        elif abs(leg[0][0]) < abs(leg[1][0]):
            x_leg = leg[0][1] * math.cos(leg[0][0])
            y_leg = leg[0][1] * math.sin(leg[0][0])
            z_leg = 1
        else:
            x_leg = leg[1][1] * math.cos(leg[1][0])
            y_leg = leg[1][1] * math.sin(leg[1][0])
            z_leg = 1
    else:
        if prev_x != 0 and prev_y != 0:
            x_leg = prev_x
            y_leg = prev_y
            z_leg = prev_z + 1
        else:
            x_leg = 0
            y_leg = 0
            z_leg = 0

    vector.x = x_leg
    vector.y = y_leg
    vector.z = z_leg
    prev_x = x_leg
    prev_y = y_leg
    prev_z = z_leg

    leg_pub.publish(vector)

def listener():
    rospy.Subscriber("/scan", LaserScan, read_cb, queue_size = 1)

    #spin forever
    rospy.spin()

listener()
