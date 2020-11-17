#!/usr/bin/env python
#from hokuyolx import HokuyoLX
import rospy
import numpy as np
from numpy import multiply, cos, sin, polyfit, array, append, poly1d
from math import sqrt, atan, tan, degrees, radians
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped, Point32
from geometry_msgs.msg import QuaternionStamped
import pandas as pd
from skimage.measure import ransac, LineModelND

from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32

import math
import tf

# Max distance the laser detects 
laser_dist = 10000 #10000

# The highest angle of the wall which is accepted
max_angle = 45

# Set the script into debugging mode
debug = False

scan_pub = rospy.Publisher('ransac_scan', LaserScan, queue_size=1)

rotated_scan_pub = rospy.Publisher('rotated_ransac_scan', LaserScan, queue_size=1)
pc_rotated_scan_pub = rospy.Publisher('pc_rotated_ransac_scan', PointCloud, queue_size=1)

laser = None
roll = 0
pitch = 0
yaw = 0

raw_quat = None

if debug:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from PIL import Image, ImageTk
    import time

    import Tkinter as tk

def laserSubCallback(data):
    global laser
    laser = data
    # print("Got scan")

def attitudeCallback(data):
    global roll, pitch, yaw, raw_quat
    
    raw_quat = data

    attitude = tf.transformations.euler_from_quaternion([data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w], axes='sxyz')
    roll = attitude[0]
    pitch = attitude[1]
    yaw = attitude[2]
    


# Makes laser scans, finds the biggest object and calculates
# the angle of it and the distance to it
#def main_laser(laser, ax, canvas, debug=False):
def main_laser(ax, canvas, debug=False):

    global roll, pitch, yaw, raw_quat

    scan_cp = np.zeros((440 + 1,2))
    scan_cp[:,0] = np.arange(-0.00436332309619*220, 0.00436332309619*221, 0.00436332309619)
    #scan_cp[:,1] = np.array(laser.ranges[340:781])*1000
    scan_cp[:,1] = np.array(laser.ranges[320:761])*1000
    scan_cp[scan_cp[:,1]>10000,1] = 10000
    
    #scan = scan_cp[scan_cp[:,1]<10000,:]
    scan = scan_cp.copy()

    datay = multiply(scan[:, 1], cos(scan[:, 0]))
    datax = multiply(scan[:, 1], sin(scan[:, 0]))

    data = np.stack([datax, datay, datax*0],1)

    attitude = tf.transformations.euler_from_quaternion([raw_quat.quaternion.x, raw_quat.quaternion.y, raw_quat.quaternion.z, raw_quat.quaternion.w], axes='sxyz')
    q = tf.transformations.quaternion_from_euler( -attitude[1], attitude[0], attitude[2]*0 )
    Rrp = tf.transformations.quaternion_matrix( q )

    # print(attitude)

    rotdata = np.array( np.matrix(Rrp[:3,:3]) * data.transpose() )

    #print( laser.ranges[540], np.min(laser.ranges[320:761]*1000), np.argmin(laser.ranges[320:761]*1000) )
    minwall = np.min(laser.ranges[320:761])
    #rotx =  laser.ranges[540]*np.cos(attitude[1]) * 1000 / np.cos(scan_cp[:,0])
    rotx =  minwall*np.cos(attitude[1]) * 1000 / np.cos(scan_cp[:,0])
    

    #print(rotx)

    #rotx = scan[:,1] * np.cos(attitude[1])

    #print("rotdata")

    #print(rotdata)
    #print(rotdata[0,:])
    #print( rotdata[0,:]**2 + rotdata[1,:]**2 )

    rotLength = np.sqrt( rotdata[0,:]**2 + rotdata[1,:]**2 )
    rotLength = np.sqrt( rotdata[1,:]**2 )

    #print("rotlength")
    #print(scan[:5,1])
    #print(rotLength.shape)


    # attempts = 0

    #print(datax.shape)
    #print(rotdata[0,:].shape)
    #print("data rodata")
    
    #data = np.stack([rotdata[0,:], rotdata[1,:]],1)
    
    #print(data)
    #data = np.stack([datax, datay],1)
    #print(data.shape)

    models = []
    inliers_lists = []
    n_inliers = []
    n = 0
    attempts = 0

    inliers = np.zeros( scan_cp.shape[0], dtype=int )
    # print(inliers.shape)
    while True:
        if data.shape[0] < 20:
            break
            #print("NO FIT!")
            #return 0, 0, 0, ax, canvas, 0, 0, 0, 0
        # robustly fit line only using inlier data with RANSAC algorithm
        
        model_robust, inliers_sub = ransac(data, LineModelND, min_samples=2,
                                       residual_threshold=30, max_trials=10)
        outliers = inliers_sub == False

        #print(model_robust.params)

        angle_wall = degrees(atan(model_robust.params[1][1] / model_robust.params[1][0]))

        #print(angle_wall)

        if abs(angle_wall) < max_angle and inliers_sub.sum() > 30:

            # inliers_lists.append(inliers)
            n_inliers.append(inliers_sub.sum())
            models.append(model_robust)
            inliers_lists.append(attempts)
            n += 1

            # angle_wall = degrees(atan(model_robust.params[1][1] / model_robust.params[1][0]))

        data = data[outliers]

        if attempts == 0:
            # print(inliers)
            # print("inliers == attemspts {} {}".format(attempts, inliers.size))
            # print(inliers[inliers == attempts])
            # print(outliers.sum())
            inliers[outliers] += 1
        else:
            # print(inliers)
            # print("inliers == attemspts {} {}".format(attempts, inliers.size))
            # print(inliers[inliers == attempts])
            # print(outliers.sum())
            inliers[inliers == attempts] += np.array(outliers)

        if n == 3 or attempts > 10:
            break

        attempts += 1

    if n > 0:

        i = n_inliers.index(max(n_inliers))

        # print(n_inliers)
        # print(attempts)
        # print(inliers_lists)

        model_robust = models[i]

    if n > 0:

        x = model_robust.params[0][0]
        y = model_robust.params[0][1]
        x_est = model_robust.params[0][0]
        y_est = model_robust.params[0][1]
        angle_wall = degrees(atan(model_robust.params[1][1] / model_robust.params[1][0]))
        #yaw_est = degrees(math.atan2(model_robust.params[1][0] , model_robust.params[1][1]))
        yaw_est = degrees(math.atan(model_robust.params[1][0] / model_robust.params[1][1]))

        # print("{} {} {} {}".format(model_robust.params[1][0], model_robust.params[1][1], yaw_est, angle_wall))

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

    # # publish scan
    scan_msg = LaserScan()
    #scan_msg.header.frame_id = "drone"
    scan_msg.header.frame_id = "flat_hokuyo"
    #scan_msg.header.frame_id = "hokuyo_link"
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.time_increment = 1.73611151695e-05
    scan_msg.scan_time = 0.0250000003725
    scan_msg.angle_increment = 0.00436332309619
    # scan_msg.angle_min = -2.35619449615
    # scan_msg.angle_max = 2.35619449615
    #scan_msg.angle_min = -1.57079631463/2.0
    #scan_msg.angle_max = 1.57079631463/2.0
    # scan_msg.angle_min = -0.872664619238
    # scan_msg.angle_max = 0.872664619238
    scan_msg.angle_min = -0.00436332309619*220
    scan_msg.angle_max = 0.00436332309619*220
    # scan_msg.ranges = scan_cp[:,1]*0.001
    scan_msg.ranges = rotx*0.001
    scan_msg.range_min = 0.2
    scan_msg.range_max = 10.1
    #scan_msg.intensities = 500*np.ones((441,1),dtype=int)
    scan_msg.intensities = inliers

    #print(len(scan_cp[:,1]))
    #print(len(inliers))

    scan_pub.publish(scan_msg)
    
    # # publish scan
    #scan_msg.header.frame_id = "drone"
    scan_msg.header.frame_id = "hokuyo_link"
    scan_msg.ranges = rotLength*0.001
    
    #print(len(scan_cp[:,1]))
    #print(len(inliers))

    rotated_scan_pub.publish(scan_msg)

    pc_msg = PointCloud()
    
    pc_msg.header.frame_id = "flat_hokuyo"
    #pc_msg.header.frame_id = "hokuyo_link"
    pc_msg.header.stamp = rospy.Time.now()

    channelMsg = ChannelFloat32()
    channelMsg.name = 'intensity'
    channelMsg.values.append(100)

    for i in range(0,rotdata.shape[1]):
        #print(rotdata[:,i])
        pointmsg = Point32()
        pointmsg.y = rotdata[0,i]*0.001
        pointmsg.x = rotdata[1,i]*0.001
        pointmsg.z = rotdata[2,i]*0.001
        #print(pointmsg)
        pc_msg.points.append(pointmsg)
        pc_msg.channels.append(channelMsg)
        
    pc_rotated_scan_pub.publish(pc_msg)

    if n>0:
        return x, y, angle_wall, ax, canvas, x_est, y_est, yaw_est, n > 0
    else:
        return 0, 0, 0, ax, canvas, 0, 0, 0, 0


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
    # laser = HokuyoLX()

    global laser
    laserSub = rospy.Subscriber('scan', LaserScan, laserSubCallback)

    attitideSub = rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, attitudeCallback)

    # Create the publisher
    pub = rospy.Publisher('wall_position', Float32MultiArray, queue_size=1)
    ang_pub = rospy.Publisher('ang_pub', PoseStamped, queue_size=1)
    wall_pub = rospy.Publisher('wall_pub', PoseStamped, queue_size=1)
    pos_pub = rospy.Publisher('pos_pub', PointStamped, queue_size=1)

    est_pub = rospy.Publisher('est_pub', PoseStamped, queue_size = 1)
    # Create the node
    rospy.init_node('wall', anonymous=True)
    # Set the rate. Not sure what this should be
    rate = rospy.Rate(40)
    # Keep the loop alive
    while not rospy.is_shutdown():
        # Obtain a laser measurement
        #x, y, angle, ax, canvas, x_est, y_est, yaw_est, valid = main_laser(laser, ax, canvas, debug)
        if laser != None:
            x, y, angle, ax, canvas, x_est, y_est, yaw_est, valid = main_laser(ax, canvas, debug)
        else:
            continue
        # Convert it to the ros array
        msg = Float32MultiArray()
        msg.data = [x, y, (angle), valid]
        # Publish the data
        pub.publish(msg)

        q = tf.transformations.quaternion_from_euler( roll, pitch, (0)*3.14159/180.0)
        q1 = q.copy()
        # q = tf.transformations.quaternion_from_euler( 0, 0, (-angle)*3.14159/180.0)
        rotM = tf.transformations.quaternion_matrix(q)

        rotPose = np.matmul( rotM,  np.array([[y], [x], [0], [1]]) )
        
        # print( (rotPose[0]*0.001, rotPose[1]*0.001, np.sqrt((rotPose[0]*0.001)**2 + (rotPose[1]*0.001)**2)) )
        # print( (angle, yaw_est, angle + yaw_est) )
        

        # q = tf.transformations.quaternion_from_euler( 0, 0, (-angle)*3.14159/180.0)
        # q = tf.transformations.quaternion_from_euler(-roll, -pitch, yaw)
        
        br = tf.TransformBroadcaster()
        br.sendTransform( (rotPose[0]*0.001, rotPose[1]*0.001, rotPose[2]*0.00), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "Wall", "hokuyo_link" )

        q = tf.transformations.quaternion_from_euler( 0, 0, 0 )
        # q = tf.transformations.quaternion_from_euler( -roll, -pitch, (-angle)*3.14159/180.0)
        q = tf.transformations.quaternion_from_euler( -roll*0, -pitch*0, -angle*np.pi/180.0 )
        br.sendTransform( (y*0.001, x*0.001, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "Wall xy", "hokuyo_link" )
        
        br.sendTransform( (y_est*0.001, x_est*0.001, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "Wall xy est", "hokuyo_link" )

        q = tf.transformations.quaternion_from_euler( -roll, -pitch, 0)
        att = tf.transformations.euler_from_quaternion([q[0], q[1], q[2], q[3]], axes='rxyz')

        rr = att[0]
        rp = att[1]

        flat_dist = np.sqrt(y**2 + x**2) * 0.001
        flat_rot = angle - att[2]*180.0/np.pi

        q = tf.transformations.quaternion_from_euler( -roll, -pitch, -yaw)
        att = tf.transformations.euler_from_quaternion([q[0], q[1], q[2], q[3]], axes='sxyz')

        ry = att[2]

        #print(q1, q2)
        
        #print(att)
        print( ( flat_dist, flat_rot, ry*180/np.pi ) )
        

        #q = tf.transformations.quaternion_from_euler( -roll, -pitch, -flat_rot*np.pi/180.0 )
        q = tf.transformations.quaternion_from_euler( -roll, -pitch, 0 )
        #q = tf.transformations.quaternion_from_euler( rr, rp, -angle*np.pi/180*0 )
        #q2 = q.copy()
        br.sendTransform( (0, 0, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "flat_hokuyo", "hokuyo_link" )

        q = tf.transformations.quaternion_from_euler( 0, 0, 0 )
        #br.sendTransform( (flat_dist, 0, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "flat_wall", "hokuyo_link" )
        br.sendTransform( (flat_dist, 0, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "flat_wall", "flat_hokuyo" )

        #q = tf.transformations.quaternion_from_euler( roll, pitch, -ry )
        #br.sendTransform( (0, 0, 0.5), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "drone", "World" )

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
