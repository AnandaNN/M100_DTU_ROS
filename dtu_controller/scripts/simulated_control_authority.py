#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rospy import init_node, spin, Service
from dji_sdk.srv import SDKControlAuthority, SDKControlAuthorityResponse

class ControlAuthorityServer():
  def __init__(self):
    sca = Service('/dji_sdk/sdk_control_authority', SDKControlAuthority, self.handler)

  def handler(self, request):
    res = SDKControlAuthorityResponse(True, 0, 0, 0)
    return res

if __name__ == '__main__':
  init_node('simulated_control_authority_server')
  cas = ControlAuthorityServer()
  spin()
