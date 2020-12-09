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

# The highest angle of the wall which is accepted
max_angle = 45

class WallEstimator():
    
    def __init__(self):
        #Variables
        self.laser = None
        self.rawQuat = None

        self.roll = 0
        self.pitch = 0
        self.x = 0
        self.y = 0
        self.droneDist = 0
        self.angle = 0
        self.valid = False

        self.br = tf.TransformBroadcaster()

        # Subscribers
        self.scanSub = rospy.Subscriber('scan', LaserScan, self.laserSubCallback)
        self.attiSub = rospy.Subscriber('/dji_sdk/attitude', QuaternionStamped, self.attitudeCallback)

        # Create the publisher
        self.wallPub = rospy.Publisher('wall_position', Float32MultiArray, queue_size=1)
        self.pointCloudDataPub = rospy.Publisher('pc_rotated_ransac_scan', PointCloud, queue_size=1)

        rospy.Timer( rospy.Duration(1.0/40.0), self.estimatePositionCallback )
        rospy.Timer( rospy.Duration(1.0/40.0), self.transformCalculator )
        #rospy.Timer( rospy.Duration(1.0/40.0), self.testCallback )

    # Laser scan data callback
    def laserSubCallback(self, data):
        self.laser = data

    # Attitude callback that also calculates roll/pitch/yaw
    def attitudeCallback(self, data):

        self.raw_quat = data

        attitude = tf.transformations.euler_from_quaternion([data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w], axes='sxyz')
        self.roll = attitude[0]
        self.pitch = attitude[1]
        self.yaw = attitude[2]

    def testCallback(self, timmm):
        print("im testing time" + str(rospy.Time.now().secs) )

    # Calculates the estimated position
    def estimatePositionCallback(self, timerData):
        if self.laser == None:
            return

        # Define the scan 
        scan_cp = np.zeros((440 + 1,2))
        scan_cp[:,0] = np.arange(-0.00436332309619*220, 0.00436332309619*221, 0.00436332309619)
        scan_cp[:,1] = np.array(self.laser.ranges[320:761])*1000
        scan_cp[scan_cp[:,1]>10000,1] = 10000
        
        scan = scan_cp.copy()

        datay = multiply(scan[:, 1], cos(scan[:, 0]))
        datax = multiply(scan[:, 1], sin(scan[:, 0]))

        data = np.stack([datax, datay, datax*0],1)

        q = tf.transformations.quaternion_from_euler( self.pitch, self.roll, 0 )
        # Rrp = tf.transformations.inverse_matrix( tf.transformations.quaternion_matrix( q ) )
        Rrp = np.linalg.pinv( tf.transformations.quaternion_matrix( q ) )
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
            
        self.pointCloudDataPub.publish(pc_msg)

        if n>0:
            self.x = x
            self.y = y
            self.angle = angle_wall
            self.valid = n > 0
        else:
            self.x = 0
            self.y = 0
            self.angle = 0
            self.valid = n > 0

    def transformCalculator(self, timerData):

        q = tf.transformations.quaternion_from_euler( 0, 0, -self.angle*np.pi/180 )
        self.br.sendTransform( (self.y*0.001, self.x*0.001, 0), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "Wall xy", "flat_hokuyo" )

        Rrp1 = tf.transformations.quaternion_matrix(q)
        Rrp1[0:2,3] = [self.y*0.001, self.x*0.001]
        
        q = tf.transformations.quaternion_from_euler( self.roll, self.pitch, 0 )
        Rrp = tf.transformations.quaternion_matrix( q )

        #fquats = tf.transformations.quaternion_from_matrix( tf.transformations.inverse_matrix(Rrp) )
        Rrp = np.linalg.pinv(Rrp)
        fquats = tf.transformations.quaternion_from_matrix( Rrp )

        self.br.sendTransform( (0, 0, 0), (fquats[0], fquats[1], fquats[2], fquats[3]), rospy.Time.now(), "flat_hokuyo", "hokuyo_link" )

        self.br.sendTransform( (0.1, 0, 0.125), (0, 0, 0, 1), rospy.Time.now(), "hokuyo_link", "drone" )

        Rrp[0:3,3] = [0.1, 0, 0.125]

        m1 = np.matrix(  Rrp  )
        m2 = np.matrix(  Rrp1 )

        trueX = (np.linalg.pinv( m2 ) * np.linalg.pinv(m1) )[0,3]

        # print(  )

        # Convert it to the ros array
        msg = Float32MultiArray()
        #msg.data = [self.x, self.y, self.angle, self.valid, (np.linalg.inv( m2 ) * np.linalg.inv(m1) )[0,3]]
        msg.data = [self.x, self.y, self.angle, self.valid, trueX]
        # Publish the data
        self.wallPub.publish(msg)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('wall', anonymous=True)
    wallEstimator = WallEstimator()
    wallEstimator.run()





