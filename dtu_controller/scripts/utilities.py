#! /usr/bin/env python3

#  BSD 3-Clause License
#
#  Copyright (c) 2019, David Wuthier - daw@mp.aau.dk
#  Aalborg University
#  Robotics, Vision and Machine Intelligence Laboratory
#  Department of Materials and Production
#  A. C. Meyers Vaenge 15, 2450 Copenhagen SV, Denmark
#  http://rvmi.aau.dk/
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from numpy import array, matrix, arccos, clip, sqrt, zeros, arctan2
from numpy.linalg import norm
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose, Wrench, PoseStamped, WrenchStamped
from geometry_msgs.msg import Transform, TransformStamped
from tf.transformations import quaternion_conjugate, quaternion_multiply, quaternion_matrix, quaternion_from_euler

def rotationMatrix(q):
  return matrix(quaternion_matrix(q)[:3,:3])

def saturate(v, m):
  norm_v = norm(v)
  if norm_v > m:
    return m * v/norm_v
  return v

def frame2transform(F):
  return [list(F.p), list(F.M.GetQuaternion())]

class CompactTransform(object):
  def __init__(self, p = matrix([[0.0], [0.0], [0.0]]), q = array([0.0, 0.0, 0.0, 1.0])):
    self.p = p
    self.q = q

  @staticmethod
  def fromXYZRPY(x, y, z, roll, pitch, yaw):
    return CompactTransform(matrix([[x], [y], [z]]), quaternion_from_euler(roll, pitch, yaw))

  @staticmethod
  def fromRow(row):
    return CompactTransform(matrix(row[:3]).T, array(row[3:7]))

  @staticmethod
  def fromPose(pose):
    return CompactTransform(
        matrix([[pose.position.x], [pose.position.y], [pose.position.z]]),
        array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))

  @staticmethod
  def fromTransform(transform):
    return CompactTransform(
        matrix([[transform.translation.x], [transform.translation.y], [transform.translation.z]]),
        array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]))

  @staticmethod
  def fromFrame(frame):
    return CompactTransform(matrix(list(frame.p)).T, array(frame.M.GetQuaternion()))

  def shorten(self):
    if self.q[3] < 0.0:
      self.q *= -1.0

  def r(self):
    self.shorten()

    if self.q[3] < 1.0 - 1.0e-6:
      return (2.0 * arccos(clip(self.q[3], -1.0, 1.0)))/sqrt(1.0 - self.q[3]**2) * matrix(self.q[:3]).T
    else:
      return matrix(zeros([3, 1]))

  def level(self):
    R = rotationMatrix(self.q)
    yaw = arctan2(R[1,0], R[0,0])
    self.q = quaternion_from_euler(0.0, 0.0, yaw)

  def toPose(self):
    self.shorten()

    return Pose(
        position = Point(
          x = self.p[0,0], y = self.p[1,0], z = self.p[2,0]),
        orientation = Quaternion(
          x = self.q[0], y = self.q[1], z = self.q[2], w = self.q[3]))

  def toTransform(self):
    return Transform(
        translation = Vector3(x = self.p[0,0], y = self.p[1,0], z = self.p[2,0]),
        rotation = Quaternion(x = self.q[0], y = self.q[1], z = self.q[2], w = self.q[3]))

  def toRow(self):
    return [self.p[0,0], self.p[1,0], self.p[2,0], self.q[0], self.q[1], self.q[2], self.q[3]]

  def inverse(self):
    q_congugate = quaternion_conjugate(self.q)
    return CompactTransform(
        -rotationMatrix(q_congugate) * self.p, q_congugate)

  def __add__(self, other):
    return CompactTransform(
        self.p + other.p,
        quaternion_multiply(other.q, self.q))

  def __sub__(self, other):
    return CompactTransform(
        self.p - other.p,
        quaternion_multiply(self.q, quaternion_conjugate(other.q)))

  def __mul__(self, other):
    return CompactTransform(
        rotationMatrix(self.q) * other.p + self.p,
        quaternion_multiply(self.q, other.q))

class CompactWrench(object):
  def __init__(self, F = matrix([[0.0], [0.0], [0.0]]), M = matrix([[0.0], [0.0], [0.0]])):
    self.F = F
    self.M = M

  @staticmethod
  def fromWrench(wrench):
    return CompactWrench(
        matrix([[wrench.force.x], [wrench.force.y], [wrench.force.z]]),
        matrix([[wrench.torque.x], [wrench.torque.y], [wrench.torque.z]]))

