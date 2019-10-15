#!/usr/bin/env python
# improving leg algorithm
'''
leg finding script - version 5
last editted on 09/10/2019
by Phillip Luu
'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import math
import numpy

global ang_min
global a_inc
global prev_x
global prev_y
global prev_z
#radius = 0.55                # swing diameter of 52 cm, with tolerancing of +3cm
prev_x = 0
prev_y = 0
prev_z = 0

rospy.init_node('leg_find')

leg_pub = rospy.Publisher("vector", Vector3, queue_size = 1)

def polar_dist(r1, th1, r2, th2):
    '''calculate polar distance between points'''
    global ang_min
    global a_inc

    dist = math.sqrt(r1*r1 + r2*r2 - 2*r1*r2*math.cos(th1 - th2))
    return dist

def conf_det(pre_x, pre_y, new_x, new_y):
    '''
    determine how likely this leg is to match previous leg scan
    '''
    x_off = 0.4
    y_off = 0.1
    stride_est = 0.7
    x_scale = 1
    y_scale = 2
    x_conf = 0.4
    y_conf = y_scale/x_scale * x_conf

    x_err = abs(pre_x - new_x) - x_off
    y_err = abs(pre_y - new_y) - y_off
    #str_err = math.sqrt(x_scale*x_scale*x_err*x_err + y_scale*y_scale*y_err*y_err)
    #print('x_diff = ' + str(x_err) + ' and y_diff = ' + str(y_err))
    #if str_err < str_est:
    conf = max(0, 1 - (math.sqrt(0.5 * (x_conf*x_err*x_conf*x_err) + (y_conf*y_err*y_conf*y_err))))
    return conf

def leg_find(data):
    global ang_min
    global a_inc
    global radius
    global prev_x
    global prev_y
    global prev_z

    # define constants
    leg_size_min = 0.075
    leg_size_max = 0.15
    depth_min = 0.01
    depth_max = 0.07
    connect_dist = 0.05

    # ensure laser variables are up to date
    ang_min = data.angle_min
    ang_max = data.angle_max
    a_inc = data.angle_increment

    # reset feature variables
    angle = numpy.linspace(ang_min, ang_max, len(data.ranges))
    vector = Vector3()
    temp_i = 0
    temp_f = 0
    clear_flag = 0
    leg = []
    leg_c = 0
    dist_flag = 1
    conf_h = 0
    i_h = 0

    # initialise scan data to be mutable object
    print('New Leg')

    for c, scan in enumerate(data.ranges):
        if c == (len(data.ranges)-1):
            break
        if abs(scan - data.ranges[c+1]) < connect_dist:
            if temp_i == 0:
                temp_i = c
            continue
        elif temp_i != 0:
            temp_f = c
            clear_flag = 1
        if temp_f - temp_i > 5:
            #print('Entry {:f} and {:f}'.format(angle[temp_i], angle[temp_f]))
            if leg_size_min < polar_dist(data.ranges[temp_i],angle[temp_i],data.ranges[temp_f],angle[temp_f]) < leg_size_max:
                mid = int(math.ceil(0.5*(temp_f + temp_i)))
                quar_1 = int(math.ceil(0.5*(mid + temp_i)))
                quar_3 = int(math.ceil(0.5*(mid + temp_f)))
                depth_R = data.ranges[temp_i] - data.ranges[mid]
                depth_L = data.ranges[temp_f] - data.ranges[mid]
                depth_1 = data.ranges[quar_1]
                depth_3 = data.ranges[quar_3]
                if depth_min < depth_L < depth_max and depth_min < depth_R < depth_max:
                    if data.ranges[temp_i] >= depth_1 >= data.ranges[mid] and data.ranges[temp_f] >= depth_3 >= data.ranges[mid]:
                        ini_angle = round(180/math.pi*(angle[temp_i]),3)
                        fin_angle = round(180/math.pi*(angle[temp_f]),3)
                        leg.append([angle[mid],data.ranges[mid]])
                        print('Leg between {:f} and {:f}'.format(ini_angle, fin_angle))
        if clear_flag == 1:
            temp_i = 0
            temp_f = 0
            clear_flag = 0
        if scan < 0.3:
            dist_flag = 0

    # convert legs to useful information
    # leg[0][i] is the first leg entry discovered starting from negative angles
    # leg[i][0] is the angle of corresponding leg
    # leg[i][1] is the distance of corresponding leg
    if len(leg) == 0:
        if prev_x != 0 and prev_y != 0:
            x_leg = prev_x
            y_leg = prev_y
            z_leg = dist_flag * (math.ceil(prev_z) + 1)
        else:
            x_leg = 0
            y_leg = 0
            z_leg = 0
    elif len(leg) == 1:
        x_leg = leg[0][1] * math.cos(leg[0][0])
        y_leg = leg[0][1] * math.sin(leg[0][0])
        conf = conf_det(prev_x, prev_y, x_leg, y_leg)
        if conf < 0.75:
            x_leg = 0
            y_leg = 0
            z_leg = 0
        else:
            z_leg = dist_flag * conf
    elif len(leg) > 1:
        for i in range(len(leg)):
            if i == len(leg) - 1:
                break
            dist_check = leg[i][1] - leg[i+1][1]
            ang_check = abs(leg[i][0] - leg[i+1][0])
            if dist_check > 0.6 or ang_check > 15:
                x_t = leg[i][1] * math.cos(leg[i][0])
                y_t = leg[i][1] * math.sin(leg[i][0])
                tconf = conf_det(prev_x, prev_y, x_t, y_t)
                if tconf > conf_h:
                    conf_h = tconf
                    i_h = i
                    x_leg = x_t
                    y_leg = y_t
                    z_leg = conf_h * dist_flag
                else:
                    continue
            else:
                x_t = 0.5 * (leg[i][1] * math.cos(leg[i][0]) + leg[i+1][1] * math.cos(leg[i+1][0]))
                y_t = 0.5 * (leg[i][1] * math.sin(leg[i][0]) + leg[i+1][1] * math.sin(leg[i+1][0]))
                tconf = conf_det(prev_x, prev_y, x_t, y_t)
                if tconf > conf_h:
                    conf_h = tconf
                    i_h = i
                    x_leg = x_t
                    y_leg = y_t
                    z_leg = conf_h * dist_flag
                else:
                    continue
    else:
        if prev_x != 0 and prev_y != 0:
            x_leg = prev_x
            y_leg = prev_y
            z_leg = dist_flag * (math.ceil(prev_z) + 1)
        else:
            x_leg = 0
            y_leg = 0
            z_leg = 0
    if z_leg > 5:
        pass
    print('moving towards: x = ' + str(x_leg) + ' y = ' + str(y_leg))
    vector.x = x_leg
    vector.y = y_leg
    vector.z = z_leg
    prev_x = x_leg
    prev_y = y_leg
    prev_z = z_leg

    leg_pub.publish(vector)

def listener():
    rospy.Subscriber("/new_scan", LaserScan, leg_find, queue_size = 1)

    #spin forever
    rospy.spin()

listener()
