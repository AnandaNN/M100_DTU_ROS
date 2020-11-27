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
    
    x = msg.orientation.y
    y = -msg.orientation.x
    msg.orientation.x = x
    msg.orientation.y = y
    
    w = msg.orientation.z
    z = msg.orientation.w
    msg.orientation.z = z
    msg.orientation.w = w
    
    pub.publish(msg)


# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')

    pub = rospy.Publisher("/dji_sdk/imu_corrected", Imu, queue_size=1)
    sub = rospy.Subscriber("/dji_sdk/imu", Imu, imuCallback)

    rospy.spin()