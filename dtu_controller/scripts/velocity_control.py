#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rospy import init_node, spin, Service, Subscriber, Publisher
from rospy import loginfo, loginfo_once
from sensor_msgs.msg import Joy
from math import pi
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist, Vector3

from utilities import CompactTransform, saturate

RAD2DEG = pi/180.0

class VelocityControl():
  def __init__(self):
    self.Tc = CompactTransform.fromXYZRPY(0.0, 0.0, 0.0, 0.0, 0.0, -20 * RAD2DEG)
    self.got_pose = False
    self.got_reference = False

    self.velocity_pub = Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)

    joy_sub = Subscriber('/joy', Joy, self.joy_callback)
    pose_sub = Subscriber('/mavros/vision_pose/pose', PoseStamped, self.pose_callback)
    reference_sub = Subscriber('/mavros/control/reference', PoseStamped, self.reference_callback)

  def pose_callback(self, msg):
    self.TBW = CompactTransform.fromPose(msg.pose)
    self.got_pose = True
    loginfo_once('got pose')

  def reference_callback(self, msg):
    self.TRW = CompactTransform.fromPose(msg.pose)
    self.got_reference = True
    loginfo_once('got reference')

  def joy_callback(self, msg):
    a = msg.axes
    #self.velocity_pub.publish(
        #PoseStamped(
          #header = Header(
            #frame_id = 'world'),
          #pose = Pose(
            #position = Point(
              #x = a[3],
              #y = a[2],
              #z = a[1]),
            #orientation = Quaternion(
              #x = 0.0,
              #y = 0.0,
              #z = 0.0,
              #w = 1.0))))
    if a[4] < -0.5 and self.got_pose and self.got_reference:
      TBRW = self.TRW - self.TBW
      TBRWc = self.Tc * TBRW
      vr = saturate(TBRWc.p, 1.0)
      wr = saturate(TBRW.r(), 1.0)
      vx = vr[0]
      vy = vr[1]
      vz = vr[2]
      wz = wr[2]
    else:
      vx = a[3]
      vy = a[2]
      vz = a[1]
      wz = a[0]

    self.velocity_pub.publish(
        Twist(
          linear = Vector3(
            x = vx,
            y = vy,
            z = vz),
          angular = Vector3(
            x = 0.0,
            y = 0.0,
            z = wz)))

if __name__ == '__main__':
  init_node('velocity_control')
  vc = VelocityControl()
  spin()
