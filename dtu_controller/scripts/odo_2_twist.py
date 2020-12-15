#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import tf

# ROS messages.
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np

RAD2DEG = 180.0/np.pi 

class QuatToEuler():
    def __init__(self):

        # Create subscribers and publishers.
        self.odoSub = rospy.Subscriber("/odometry/filtered_map", Odometry, self.odoCallback)
        self.odoPub = rospy.Publisher("/dtu_controller/current_frame_pose", Twist, queue_size=1)

        # Main while loop.
        rospy.spin()

    def odoCallback(self, msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        outMsg = Twist()
        outMsg.linear.x = msg.pose.pose.position.x
        outMsg.linear.y = msg.pose.pose.position.y
        outMsg.linear.z = msg.pose.pose.position.z
        outMsg.angular.x = r
        outMsg.angular.y = p
        outMsg.angular.z = y
        self.odoPub.publish(outMsg)

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('odo_2_twist')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass