/** @file contactController.cpp
 *  @version 1.0
 *  @date October, 2020
 *
 *  @brief
 *  Concact controller class
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#include "ContactController.h"
#include "geometry_msgs/Vector3.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Vector3.h"

void ContactController::init( ros::NodeHandle nh )
{
  _ctrlAttitudePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zvelocity", 1);
  _referencePub = nh.advertise<geometry_msgs::Vector3>("/dji_sdk/debug/reference", 1);
  _statePub = nh.advertise<geometry_msgs::Vector3>("/dji_sdk/debug/state", 1);
  _commandPub = nh.advertise<geometry_msgs::Vector3>("/dji_sdk/debug/command", 1);

  _attitudeQuaternionSub = nh.subscribe<geometry_msgs::QuaternionStamped>( "/dji_sdk/attitude", 1, &ContactController::attitudeCallback, this );
  _imuSub = nh.subscribe<sensor_msgs::Imu>( "/dji_sdk/imu", 1, &ContactController::imuCallback, this );
  _rodSub = nh.subscribe<std_msgs::Bool>("/dji_sdk/rod", 1, &ContactController::rodCallback, this);

  _controlValue.axes.push_back(0);
  _controlValue.axes.push_back(0);
  _controlValue.axes.push_back(0);
  _controlValue.axes.push_back(0);

  _controlTimer = nh.createTimer(ros::Duration(1.0/_loopFrequency), &ContactController::controlTimerCallback, this);
}

void ContactController::imuCallback( const sensor_msgs::Imu imuMsg )
{
  _imuValue = imuMsg;
}

void ContactController::rodCallback(const std_msgs::Bool::ConstPtr& msg)
{
  _rodValue = msg->data;
}

float ContactController::getYawRate(const geometry_msgs::Vector3 u, const tf::Vector3 w)
{
  tf::Matrix3x3 R_y;
  tf::Matrix3x3 R_z;
  R_y.setRPY(0.0, u.y, 0.0);
  R_z.setRPY(0.0, 0.0, u.z);
  tf::Vector3 E_1 = (R_z * R_y).getColumn(0);
  tf::Vector3 E_2 = R_z.getColumn(1);
  return (w - w.dot(E_1) * E_1 - w.dot(E_2) * E_2).getZ();
}

void ContactController::attitudeCallback( const geometry_msgs::QuaternionStamped quaternion )
{
  tf::Quaternion q_B_W(quaternion.quaternion.x, quaternion.quaternion.y, quaternion.quaternion.z, quaternion.quaternion.w);
  tf::Matrix3x3 R_B_W(q_B_W);
  R_B_W.getRPY(_attitude.x, _attitude.y, _attitude.z);

  tf::Matrix3x3 R_C_B;
  R_C_B.setRPY(0.0, _contactPitch, 0.0);
  tf::Matrix3x3 R_C_W = R_B_W * R_C_B;
  R_C_W.getRPY(_contactAttitude.x, _contactAttitude.y, _contactAttitude.z);

  tf::Vector3 w_BW_B(_imuValue.angular_velocity.x, _imuValue.angular_velocity.y, _imuValue.angular_velocity.z);
  tf::Vector3 w_BW_W = R_B_W * w_BW_B;
  _yawRate = getYawRate(_attitude, w_BW_W);
  _contactYawRate = getYawRate(_contactAttitude, w_BW_W);
}

void ContactController::setControllerGains( float kpPitch, float kdPitch, float kpYaw, float kdYaw )
{
  _kpPitch = kpPitch;
  _kdPitch = kdPitch;
  _kpYaw = kdYaw;
  _kdYaw = kdYaw;
}

void ContactController::controlTimerCallback( const ros::TimerEvent& )
{
  if( _running )
  {
    _controlValue.axes[1] = _attitude.y;
    _controlValue.axes[3] = _yawRate;
    _controlValue.axes[0] = _kpYaw * (_lastContactAttitude.z - _contactAttitude.z) - _kdYaw * _contactYawRate;
    _controlValue.axes[2] = _kpPitch * (_targetPitchDegrees - rad2deg * _contactAttitude.y);

    _ctrlAttitudePub.publish( _controlValue );

    geometry_msgs::Vector3 state;
    state.x = rad2deg * _contactAttitude.x;
    state.y = rad2deg * _contactAttitude.y;
    state.z = rad2deg * _contactAttitude.z;

    _statePub.publish(state);

    geometry_msgs::Vector3 reference;
    reference.x = 0.0;
    reference.y = _targetPitchDegrees;
    reference.z = rad2deg * _lastContactAttitude.z;

    _referencePub.publish(reference);
  }
}

