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

# scan_pub = rospy.Publisher('ransac_scan', LaserScan, queue_size=1)

# rotated_scan_pub = rospy.Publisher('rotated_ransac_scan', LaserScan, queue_size=1)
pc_rotated_scan_pub = rospy.Publisher('pc_rotated_ransac_scan', PointCloud, queue_size=1)

laser = None
roll_s = 0
pitch_s = 0
yaw_s = 0

roll_r = 0
pitch_r = 0
yaw_r = 0

raw_quat = None

if debug:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    from PIL import Image, ImageTk
    import time


def laserSubCallback(data):
    global laser
    laser = data
    # print("Got scan")

def attitudeCallback(data):
    global roll_r, pitch_r, yaw_r, raw_quat, roll_s, pitch_s, yaw_s
    
    raw_quat = data

    attitude = tf.transformations.euler_from_quaternion([data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w], axes='sxyz')
    roll_s = attitude[0]
    pitch_s = attitude[1]
    yaw_s = attitude[2]
    
    attitude = tf.transformations.euler_from_quaternion([data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w], axes='rxyz')
    roll_r = attitude[0]
    pitch_r = attitude[1]
    yaw_r = attitude[2]
    


# Makes laser scans, finds the biggest object and calculates
# the angle of it and the distance to it
#def main_laser(laser, ax, canvas, debug=False):
def main_laser(ax, canvas, debug=False):


    # Define the scan 
    scan_cp = np.zeros((440 + 1,2))
    scan_cp[:,0] = np.arange(-0.00436332309619*220, 0.00436332309619*221, 0.00436332309619)
    scan_cp[:,1] = np.array(laser.ranges[320:761])*1000
    scan_cp[scan_cp[:,1]>10000,1] = 10000
    
    scan = scan_cp.copy()

    datay = multiply(scan[:, 1], cos(scan[:, 0]))
    datax = multiply(scan[:, 1], sin(scan[:, 0]))

    data = np.stack([datax, datay, datax*0],1)

    #attitude = tf.transformations.euler_from_quaternion([raw_quat.quaternion.x, raw_quat.quaternion.y, raw_quat.quaternion.z, raw_quat.quaternion.w], axes='sxyz')
    #q = tf.transformations.quaternion_from_euler( roll_r, pitch_r, yaw_r )
    q = tf.transformations.quaternion_from_euler( pitch_s, roll_s, 0 )
    Rrp = tf.transformations.inverse_matrix( tf.transformations.quaternion_matrix( q ) )
    rotdata = np.array( np.matrix(Rrp[:3,:3]) * data.transpose() )

    data = np.stack([rotdata[0,:], rotdata[1,:]],1)

    models = []
    inliers_lists = []
    n_inliers = []
    n = 0
    attempts = 0

    inliers = np.zeros( scan_cp.shape[0], dtype=int )

    while True:
        if data.shape[0] < 20:
            break

        model_robust, inliers_sub = ransac(data, LineModelND, min_samples=2,
                                       residual_threshold=30, max_trials=10)
        outliers = inliers_sub == False

        angle_wall = degrees(atan(model_robust.params[1][1] / model_robust.params[1][0]))

        if abs(angle_wall) < max_angle and inliers_sub.sum() > 30:
            n_inliers.append(inliers_sub.sum())
            models.append(model_robust)
            inliers_lists.append(attempts)
            n += 1

        data = data[outliers]

        if attempts == 0:
            inliers[outliers] += 1
        else:
            inliers[inliers == attempts] += np.array(outliers)

        if n == 3 or attempts > 10:
            break

        attempts += 1

    if n > 0:
        i = n_inliers.index(max(n_inliers))

        model_robust = models[i]

    if n > 0:
        x = model_robust.params[0][0]
        y = model_robust.params[0][1]
        x_est = model_robust.params[0][0]
        y_est = model_robust.params[0][1]
        angle_wall = degrees(atan(model_robust.params[1][1] / model_robust.params[1][0]))
        yaw_est = degrees(math.atan(model_robust.params[1][0] / model_robust.params[1][1]))

        ax = x
        ay = y

        bx = model_robust.params[1][0]
        by = model_robust.params[1][1]

        dx = model_robust.params[1][1]
        dy = -model_robust.params[1][0]

        L = (ax*dy - ay*dx)/(by*dx - bx*dy)

        x = ax + bx*L
        y = ay + by*L
    
    # Point cloud message
    pc_msg = PointCloud()
    pc_msg.header.frame_id = "flat_hokuyo"
    #pc_msg.header.frame_id = "hokuyo_link"
    pc_msg.header.stamp = rospy.Time.now()

    channelMsg = ChannelFloat32()
    channelMsg.name = 'intensity'
    channelMsg.values.append(100)

    for i in range(0,rotdata.shape[1]):
        pointmsg = Point32()
        pointmsg.y = rotdata[0,i]*0.001
        pointmsg.x = rotdata[1,i]*0.001
        pointmsg.z = rotdata[2,i]*0.000
        pc_msg.points.append(pointmsg)
        pc_msg.channels.append(channelMsg)
        
    pc_rotated_scan_pub.publish(pc_msg)

    if n>0:
        return x, y, angle_wall, ax, canvas, x_est, y_est, yaw_est, n > 0
    else:
        return 0, 0, 0, ax, canvas, 0, 0, 0, 0


def wall_position():
    # if debugging is active then create a tkinter client and display
    # the laser scan and calculations

    global laser
    laserSub = rospy.Subscriber('scan', LaserScan, laserSubCallback)

    attitideSub = rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, attitudeCallback)

    # Create the publisher
    pub = rospy.Publisher('wall_position', Float32MultiArray, queue_size=1)

    # Create the node
    rospy.init_node('wall', anonymous=True)
    # Set the rate. Not sure what this should be
    rate = rospy.Rate(40)
    # Keep the loop alive

    br = tf.TransformBroadcaster()


    while not rospy.is_shutdown():
        # Obtain a laser measurement
        #x, y, angle, ax, canvas, x_est, y_est, yaw_est, valid = main_laser(laser, ax, canvas, debug)
        
        # if laser != None:
        #     x, y, angle, ax, canvas, x_est, y_est, yaw_est, valid = main_laser(ax, canvas, debug)
        # else:
        #     continue

        x = 1.0
        y = 2.0
        angle = 4.0
        valid = True

        q = tf.transformations.quaternion_from_euler( 0, 0, -angle*np.pi/180 )
        br.sendTransform( (y*0.001, x*0.001, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "Wall xy", "flat_hokuyo" )

        Rrp1 = tf.transformations.quaternion_matrix(q)
        Rrp1[0:2,3] = [y*0.001, x*0.001]
        #print(Rrp1)

        #print(np.sqrt(y**2 + x**2), angle, angle*np.pi/180.0)
        #0.1 0 0.125

        q = tf.transformations.quaternion_from_euler( roll_s, pitch_s, 0 )
        Rrp = tf.transformations.quaternion_matrix( q )

        fquats = tf.transformations.quaternion_from_matrix( tf.transformations.inverse_matrix(Rrp) )

        br.sendTransform( (0, 0, 0), (fquats[0], fquats[1], fquats[2], fquats[3]), rospy.Time.now(), "flat_hokuyo", "hokuyo_link" )

        Rrp = tf.transformations.inverse_matrix( Rrp )

        Rrp[0:3,3] = [0.1, 0, 0.125]

        #print(Rrp1)
        #print(Rrp)
        #tf.transformations.inverse_matrix(
        m1 = np.matrix(  Rrp  )
        m2 = np.matrix(  Rrp1 )

        #print( np.linalg.inv(m1) )
        #print( m1 )

        #th = np.matrix( [[1,0,0,0.1], [0,1,0,0], [0,0,1,0.125],[0,0,0,1]] )

        print( (np.linalg.inv( m2 ) * np.linalg.inv(m1) )[0,3] )

        #print(m2[0,3])
        #print(np.linalg.inv(m2)[0,3])

        #q = tf.transformations.quaternion_from_euler( roll_s, pitch_s, yaw_s )
        #br.sendTransform( (0, 0, 0.5), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "drone", "World" )

        #q = tf.transformations.quaternion_from_euler( 0, 0, yaw_s )
        #br.sendTransform( (0, 0, 0.5), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "flat_drone", "World" )

        # Keep the rate of the loop

        # Convert it to the ros array
        msg = Float32MultiArray()
        msg.data = [x, y, (angle), valid, (np.linalg.inv( m2 ) * np.linalg.inv(m1) )[0,3]]
        # Publish the data
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        wall_position()
    except rospy.ROSInterruptException:
        pass
