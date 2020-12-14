#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rospy import init_node, spin, Service, Subscriber, Publisher, Time
from rospy import loginfo, loginfo_once
from sensor_msgs.msg import Joy
from math import pi
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from tf2_ros import TransformBroadcaster

from utilities import CompactTransform, saturate

RAD2DEG = pi/180.0

class Pose2Tf():
  def __init__(self):
    self.tf_broadcaster = TransformBroadcaster()

    self.ts = TransformStamped()
    self.ts.header.frame_id = 'world'
    self.ts.child_frame_id = 'drone'

    pose_sub = Subscriber('/dji_sdk/pose', PoseStamped, self.pose_callback)

  def pose_callback(self, msg):
    self.ts.header.stamp = Time.now()
    self.ts.transform.translation.x = msg.pose.position.x
    self.ts.transform.translation.y = msg.pose.position.y
    self.ts.transform.translation.z = msg.pose.position.z
    self.ts.transform.rotation.x = msg.pose.orientation.x
    self.ts.transform.rotation.y = msg.pose.orientation.y
    self.ts.transform.rotation.z = msg.pose.orientation.z
    self.ts.transform.rotation.w = msg.pose.orientation.w
    self.tf_broadcaster.sendTransform(self.ts)

if __name__ == '__main__':
  init_node('pose2tf')
  p2t = Pose2Tf()
  spin()
