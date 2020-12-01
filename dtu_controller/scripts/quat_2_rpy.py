#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import tf

# ROS messages.
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from guidance.msg import Motion

from nav_msgs.msg import Odometry

import numpy as np

RAD2DEG = 180.0/np.pi 

class QuatToEuler():
    def __init__(self):

        # Create subscribers and publishers.
        self.djiAttitudeSub   = rospy.Subscriber("/dji_sdk/attitude", QuaternionStamped, self.attitudeCallback)
        self.guidanceImuSub  = rospy.Subscriber("/guidance/imu", TransformStamped, self.guidanceImuCallback)
        self.cmdSub = rospy.Subscriber("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zvelocity/", Joy, self.cmdCallback)
        self.motionSub = rospy.Subscriber("/guidance/motion", Motion, self.motionCallback)
        
        self.odoSub = rospy.Subscriber("/odometry/filtered_map", Odometry, self.odoCallback)

        self.odoPub = rospy.Publisher("/odoRPY", Vector3, queue_size=1)

        self.attitudePub = rospy.Publisher("attitudeRPY", Vector3, queue_size=1)
        self.guidanceRPYPub = rospy.Publisher("guidanceRPY", Vector3, queue_size=1)
        self.cmdPub = rospy.Publisher("cmdRP", Vector3, queue_size=1)
        self.motionPub = rospy.Publisher("motionRPY", Vector3, queue_size=1)

        # Main while loop.
        rospy.spin()

    def odoCallback(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        outMsg = Vector3()
        outMsg.x = r*RAD2DEG
        outMsg.y = p*RAD2DEG
        outMsg.z = y
        self.odoPub.publish(outMsg)

    # Odometry callback function.
    def attitudeCallback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
        outMsg = Vector3()
        outMsg.x = r
        outMsg.y = p
        outMsg.z = -y
        self.attitudePub.publish(outMsg)

    # IMU callback function.
    def guidanceImuCallback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        outMsg = Vector3()
        outMsg.x = r
        outMsg.y = -p
        outMsg.z = y-np.pi/2.0
        self.guidanceRPYPub.publish(outMsg)

    def cmdCallback(self, msg):
        outMsg = Vector3()
        outMsg.x = msg.axes[0]
        outMsg.y = msg.axes[1]
        outMsg.z = msg.axes[3]
        self.cmdPub.publish(outMsg)

    def motionCallback(self, msg):
        outMsg = Vector3()
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.q2, msg.q1, msg.q0, msg.q3])
        outMsg = Vector3()
        outMsg.x = r
        outMsg.y = p
        outMsg.z = y
        self.motionPub.publish(outMsg)

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass