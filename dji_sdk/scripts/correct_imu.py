#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

# ROS messages.
from sensor_msgs.msg import Imu

sub = None
pub = None

# IMU callback function.
def imuCallback(msg):
    # Convert quaternions to Euler angles.
    
    #x = msg.orientation.y
    #y = -msg.orientation.x
    #msg.orientation.x = x
    #msg.orientation.y = y
    
    #w = msg.orientation.z
    #z = msg.orientation.w
    #msg.orientation.z = z
    #msg.orientation.w = w
    
    msg.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
    msg.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
    msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

    #msg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
    #msg.angular_velocity_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
    #msg.linear_acceleration_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

    pub.publish(msg)


# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('imu_w_covariance')

    pub = rospy.Publisher("/dji_sdk/imu_w_covariance", Imu, queue_size=1)
    sub = rospy.Subscriber("/dji_sdk/imu", Imu, imuCallback)

    rospy.spin()