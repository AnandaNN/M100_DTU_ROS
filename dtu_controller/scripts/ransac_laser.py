#!/usr/bin/env python
from hokuyolx import HokuyoLX
import rospy
import numpy as np
from numpy import multiply, cos, sin, polyfit, array, append, poly1d
from math import sqrt, atan, tan, degrees, radians
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
import pandas as pd
from skimage.measure import ransac, LineModelND

from sensor_msgs.msg import LaserScan

import tf

# Max distance the laser detects 
laser_dist = 10000 #10000

# The highest angle of the wall which is accepted
max_angle = 45

# Set the script into debugging mode
debug = False

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)


if debug:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from PIL import Image, ImageTk
    import time

    import Tkinter as tk


# Makes laser scans, finds the biggest object and calculates
# the angle of it and the distance to it
def main_laser(laser, ax, canvas, debug=False):
    # Make a single measurement
    timestamp, scan = laser.get_filtered_dist(start=90*4,
                                              end=(180)*4,
                                              dmin=10, dmax=laser_dist)
    #timestamp, scan = laser.get_filtered_dist(dmin=0, dmax=100000)

    # publish scan
    scan_msg = LaserScan()
    scan_msg.header.frame_id = "drone"
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.time_increment = 1.73611151695e-05
    scan_msg.scan_time = 0.0250000003725
    scan_msg.angle_increment = 0.00436332309619
    #scan_msg.angle_min = -2.35619449615
    #scan_msg.angle_max = 2.35619449615
    scan_msg.angle_min = -1.57079631463/2.0
    scan_msg.angle_max = 1.57079631463/2.0
    scan_msg.ranges = scan[:,1]*0.001
    scan_msg.range_min = 0.2
    scan_msg.range_max = 10.0
    scan_msg.intensities = scan[:,1]*0 + 1000

    scan_pub.publish(scan_msg)

    less_than_45 = np.ones(scan.shape[0]) * -0.7
    
    
    # print(scan.shape)
    scan = scan[np.greater(scan[:,0], less_than_45), :]
    greater_than_45 = np.ones(scan.shape[0]) * 0.7
    scan = scan[np.less(scan[:,0], greater_than_45), :]
    # print(scan.shape)
    # Convert to Cartesian coordinates
    datay = multiply(scan[:, 1], cos(scan[:, 0]))
    datax = multiply(scan[:, 1], sin(scan[:, 0]))

    # Convert the data back to one array
    data = np.stack([datax, datay],1)

    while True:
        if data.shape[0] < 20:
            print("NO FIT!")
            return 0, 0, 0, ax, canvas
        # robustly fit line only using inlier data with RANSAC algorithm
        model_robust, inliers = ransac(data, LineModelND, min_samples=2,
                                       residual_threshold=50, max_trials=10)
        outliers = inliers == False

        # Calculate a line by fitting the inlier data
        a = model_robust.params[1][1]
        b = model_robust.params[0][1]

        # Calculate the angle of the line
        v = atan(a)
        angle_wall = degrees(v)

        # Check if the angle of the wall is accepted
        if abs(angle_wall) > max_angle or len(inliers) < 20:
            data = data[outliers]
        else:
            break

    # print(model_robust.params)

    # Calculate a line perpendicular to the wall that goes through 0,0
    # angle_90 = 90 + angle_wall
    # a_90 = tan(radians(angle_wall + 90))
    # b_90 = 0

    # The walls line:
    # a_wall = a
    # b_wall = b

    # Calculate the two lines interception
    # x = (b_90 - b_wall)/(a_wall - a_90)
    # y = a_wall * x + b_wall

    x = model_robust.params[0][0]
    y = model_robust.params[0][1]
    angle_wall = degrees(atan(model_robust.params[1][1] / model_robust.params[1][0]))

    ax = x
    ay = y

    bx = model_robust.params[1][0]
    by = model_robust.params[1][1]

    dx = model_robust.params[1][1]
    dy = -model_robust.params[1][0]

    L = (ax*dy - ay*dx)/(by*dx - bx*dy)

    x = ax + bx*L
    y = ay + by*L

    # a_d = model_robust.params[1][1] / model_robust.params[1][0]

    # a_wall = 1.0 / a_d



    if debug:
        try:
            ax.clear()
            # Draw all the data
            ax.scatter(datax, datay, s=5)
            # Draw the ransac data
            ax.plot(data[inliers, 0], data[inliers, 1], '.b', alpha=0.6,
                    label='Inlier data')
            # Draw the closest point found
            ax.scatter(x, y, s=200)
            # Saves the line is a format can be easily displayed
            line = append([a, b], array(data[inliers, 0]))
            fit_fn = np.poly1d([line[0], line[1]])
            # Draw the line
            ax.plot(line[2:], fit_fn(line[2:]), 'r')
            ax.set_ylim(0, np.max(datay))
            canvas.draw()
            time.sleep(0.25)
        except tk.TclError:
            return 0, 0, 0, ax, canvas

    return x, y, angle_wall, ax, canvas


def wall_position(debug=False):
    # if debugging is active then create a tkinter client and display
    # the laser scan and calculations
    if debug:
        top = tk.Tk()
        # Create label to show the image on
        label = tk.Label(top)
        label.pack()
        # Create a figure to plot the results on
        f = Figure(figsize=(5, 4), dpi=100)
        # Create the plot
        ax = f.add_subplot(111)
        ax.plot([])
        # Add the plot to the client
        canvas = FigureCanvasTkAgg(f, master=top)
        canvas.get_tk_widget().pack()
        canvas.draw()
    # If not debugging create a dummy ax and canvas
    else:
        ax = None
        canvas = None

    # Create the hokuyo laser object
    laser = HokuyoLX()
    # Create the publisher
    pub = rospy.Publisher('wall_position', Float32MultiArray, queue_size=10)
    ang_pub = rospy.Publisher('ang_pub', PoseStamped, queue_size=1)
    wall_pub = rospy.Publisher('wall_pub', PoseStamped, queue_size=1)
    pos_pub = rospy.Publisher('pos_pub', PointStamped, queue_size=1)
    # Create the node
    rospy.init_node('wall', anonymous=True)
    # Set the rate. Not sure what this should be
    rate = rospy.Rate(50)
    # Keep the loop alive
    while not rospy.is_shutdown():
        # Obtain a laser measurement
        x, y, angle, ax, canvas = main_laser(laser, ax, canvas, debug)
        # Convert it to the ros array
        msg = Float32MultiArray()
        msg.data = [x, y, angle]
        # Publish the data
        pub.publish(msg)

        # Ang msg
        ang_msg = PoseStamped()
        ang_msg.header.stamp = rospy.Time.now()
        ang_msg.header.frame_id = "laser"
        ang_msg.pose.position.x = 0
        ang_msg.pose.position.y = 0
        ang_msg.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, (-angle)*3.14159/180.0)

        ang_msg.pose.orientation.x = q[0]
        ang_msg.pose.orientation.y = q[1]
        ang_msg.pose.orientation.z = q[2]
        ang_msg.pose.orientation.w = q[3]

        ang_pub.publish(ang_msg)

        # Wall msg
        wall_msg = PoseStamped()
        wall_msg.header.stamp = rospy.Time.now()
        wall_msg.header.frame_id = "laser"
        wall_msg.pose.position.x = y*0.001
        wall_msg.pose.position.y = x*0.001
        wall_msg.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, (-angle - 90)*3.14159/180.0)

        wall_msg.pose.orientation.x = q[0]
        wall_msg.pose.orientation.y = q[1]
        wall_msg.pose.orientation.z = q[2]
        wall_msg.pose.orientation.w = q[3]

        wall_pub.publish(wall_msg)

        # Pos msg
        pos_msg = PointStamped()
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.header.frame_id = "laser"
        pos_msg.point.x = y*0.001
        pos_msg.point.y = x*0.001
        pos_msg.point.z = 0

        pos_pub.publish(pos_msg)

        # If debug is active then update the client
        if debug:
            try:
                top.update()
            # Break if the tk client is no longer there
            except tk.TclError:
                break
        # Keep the rate of the loop
        rate.sleep()

if __name__ == '__main__':
    try:
        wall_position(debug)
    except rospy.ROSInterruptException:
        pass
