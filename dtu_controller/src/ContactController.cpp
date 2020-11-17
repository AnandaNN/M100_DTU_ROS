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

const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;


void ContactController::init( ros::NodeHandle nh )
{
  _ctrlAttitudePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zvelocity", 1);

  _attitudeQuaternionSub = nh.subscribe<geometry_msgs::QuaternionStamped>( "/dji_sdk/attitude", 1, &ContactController::attitudeCallback, this );
  _imuSub = nh.subscribe<sensor_msgs::Imu>( "/dji_sdk/imu", 1, &ContactController::imuCallback, this );
  _rodSub = nh.subscribe<std_msgs::Bool>("/dji_sdk/rod", 1, &ContactController::rodCallback, this);

  _controlValue.axes.push_back(0);
  _controlValue.axes.push_back(0);
  _controlValue.axes.push_back(0);
  _controlValue.axes.push_back(0);

  _angularVelocityWorldFrame = tf::Vector3( 0.0, 0.0, 0.0 );
  _contactPointBodyFrame = tf::Vector3( 0.3995, 0, 0.0609 );

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

void ContactController::attitudeCallback( const geometry_msgs::QuaternionStamped quaternion )
{
  tf::Quaternion currentQuaternion(quaternion.quaternion.x, quaternion.quaternion.y, quaternion.quaternion.z, quaternion.quaternion.w);
  tf::Matrix3x3 R_FLU2ENU(currentQuaternion);
  R_FLU2ENU.getRPY(_attitude.x, _attitude.y, _attitude.z);
  tf::Vector3 angularVelocityBodyFrame(_imuValue.angular_velocity.x, _imuValue.angular_velocity.y, _imuValue.angular_velocity.z);
  _angularVelocityWorldFrame = R_FLU2ENU * angularVelocityBodyFrame;
  tf::Vector3 contactPointWorldFrame = R_FLU2ENU * _contactPointBodyFrame;
  _contactYaw = atan2(contactPointWorldFrame.getY(), contactPointWorldFrame.getX());
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

  if( _rodValue )
  {
    float target = _targetPitch;
    if( _disengage ) target = 0.0; 

    _controlValue.axes[1] = _attitude.y;
    _controlValue.axes[3] = _angularVelocityWorldFrame.getZ();
    _controlValue.axes[0] = _kpYaw * (_lastContactYaw - _contactYaw) - _kdYaw * _angularVelocityWorldFrame.getZ();
    _controlValue.axes[2] = _kpPitch * (target - rad2deg * _attitude.y);

    _contactBuffer = 0;
    // ROS_INFO("Contact running");
  }
  else
  {
    _lastContactYaw = _contactYaw;
    _contactBuffer++;
   
    if( _contactBuffer > _loopFrequency && _running ) stopController();

    if( _disengage ) _controlValue.axes[1] = _disengagePitch * deg2rad;
    else _controlValue.axes[1] = _targetPitch * deg2rad;

    _controlValue.axes[0] = 0;
    _controlValue.axes[2] = 0;
    _controlValue.axes[3] = 0;

  }

  if( _running ) _ctrlAttitudePub.publish( _controlValue );
}

