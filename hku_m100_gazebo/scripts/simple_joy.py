#! /usr/bin/python

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, QuaternionStamped, Twist
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler

from math import cos, sin

joyData = Twist()
joyData.linear.x = 0
joyData.linear.y = 0
joyData.linear.z = 0
joyData.angular.x = 0
joyData.angular.y = 0
joyData.angular.z = 0

point = PointStamped()
point.point.x = 0
point.point.y = 0
point.point.z = 0

quaternion = QuaternionStamped()
quaternion.quaternion.x = 0
quaternion.quaternion.y = 0
quaternion.quaternion.z = 0
quaternion.quaternion.w = 1

yaw = 0

def joyCallback(data):
    global joyData
    joyData.linear.x = data.axes[4]*2
    joyData.linear.y = data.axes[3]*2
    joyData.linear.z = data.axes[1]
    joyData.angular.x = 0
    joyData.angular.y = 0
    if data.axes[0] > -0.09 and data.axes[0] < 0.09:
        joyData.angular.z = 0
    else:
        joyData.angular.z = data.axes[0]/3.0

def posePubCallback(time):
    global joyData, point, quaternion, yaw

    point.point.x += (cos(yaw)*joyData.linear.x - sin(yaw)*joyData.linear.y) / 30.0
    point.point.y += (sin(yaw)*joyData.linear.x + cos(yaw)*joyData.linear.y) / 30.0
    point.point.z += joyData.linear.z / 30.0

    yaw += joyData.angular.z / 30.0


    q = quaternion_from_euler(joyData.linear.y/2 * 30.0*3.1415/180, joyData.linear.x/2 * 30.0/180*3.1415,yaw)
    quaternion.quaternion.x = q[0]
    quaternion.quaternion.y = q[1]
    quaternion.quaternion.z = q[2]
    quaternion.quaternion.w = q[3]

    pointPub.publish(point)
    attitudePub.publish(quaternion)


def start():

    rospy.init_node('joy_2_dji_faker_node')

    joySub = rospy.Subscriber("/joy", Joy, joyCallback, queue_size=1)

    global pointPub, attitudePub

    pointPub = rospy.Publisher("/dji_sdk/local_position", PointStamped, queue_size=1)
    attitudePub = rospy.Publisher("/dji_sdk/attitude", QuaternionStamped, queue_size=1)

    rospy.Timer(rospy.Duration(1.0/60.0), posePubCallback)

    rospy.spin()

if __name__ == '__main__':
    start()    

