#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rospy import init_node, spin, Service, Subscriber, Publisher
from rospy import loginfo
from dji_sdk.srv import SDKControlAuthority, SDKControlAuthorityResponse, SDKControlAuthorityRequest
from sensor_msgs.msg import Joy
from math import pi

class ControlAuthorityServer():
  def __init__(self):
    self.authority = False

    control_authority_srv = Service('/dji_sdk/sdk_control_authority', SDKControlAuthority, self.handler)

    self.control_pub = Publisher('/dji_sdk/matlab_rpvy', Joy, queue_size = 1)
    self.wind_pub = Publisher('/dji_sdk/matlab_wind', Joy, queue_size = 1)

    joy_sub = Subscriber('/dji_sdk/xbox_joy', Joy, self.joy_callback)
    control_sub = Subscriber('/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zvelocity', Joy, self.control_callback)

  def handler(self, request):
    self.authority = request.control_enable
    loginfo('authority = %s', self.authority)
    return SDKControlAuthorityResponse(True, 0, 0, 0)

  def joy_callback(self, msg):
    a = msg.axes

    if msg.buttons[5] == 1:
      wind = Joy(axes = [5*a[3], 5*a[2], 5*a[1], 2*a[0]])
      self.wind_pub.publish(wind);
    else:
      if self.authority:
        self.authority = False
        loginfo('authority = %s', self.authority)

      joy = Joy(axes = [-a[2], a[3], 2*a[1], 2*a[0]])
      self.control_pub.publish(joy)

  def control_callback(self, msg):
    if self.authority:
      self.control_pub.publish(msg)

if __name__ == '__main__':
  init_node('simulated_control_authority_server')
  cas = ControlAuthorityServer()
  spin()
