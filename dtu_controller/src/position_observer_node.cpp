/** @file position_observer_node.cpp
 *  @version 3.3
 *  @date June, 2020
 *
 *  @brief
 *  demo sample of joy stick with rpyrate zvel control
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#include "position_observer_node.h"
#include "dji_sdk/dji_sdk.h"
#include "type_defines.h"

// Publisher
ros::Publisher currentPosePub;

// Subscriber
ros::Subscriber localGPSPositionSub;
ros::Subscriber attitudeQuaternionSub;
ros::Subscriber ultraHeightSub;
ros::Subscriber gpsHealthSub;

// Global sub/pub variables
geometry_msgs::Twist currentPose;
geometry_msgs::Twist currentReference;

geometry_msgs::Vector3 attitude;
geometry_msgs::Vector3 localPosition;

float globalRotation = 0;
tf::Quaternion currentQuaternion;

float actualHeight = 0;
uint8_t gpsHealth = 0;

// Global random values
float loopFrequency;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_observer_node");
  ros::NodeHandle nh;

  ros::Duration(1).sleep();

  readParameters( nh );

  // Publisher for control values
  currentPosePub = nh.advertise<geometry_msgs::Twist>("/dtu_controller/current_frame_pose", 0);

  // Initialize subsrcibers
  localGPSPositionSub = nh.subscribe<geometry_msgs::PointStamped>( "/dji_sdk/local_position", 0, localPositionCallback );
  attitudeQuaternionSub = nh.subscribe<geometry_msgs::QuaternionStamped>( "/dji_sdk/attitude", 0, attitudeCallback );
  ultraHeightSub = nh.subscribe<std_msgs::Float32>("/dji_sdk/height_above_takeoff", 0, ultraHeightCallback);
  gpsHealthSub = nh.subscribe<std_msgs::UInt8>("/dji_sdk/gps_health", 0, gpsHealthCallback);

  ros::Duration(1).sleep();

  // Start the control loop timer
  
  ros::Timer loopTimer = nh.createTimer(ros::Duration(1.0/loopFrequency), observerLoopCallback);

  ros::spin();

  return 0;
}

void readParameters( ros::NodeHandle nh )
{
  // Read parameter file
  nh.getParam("/dtu_controller/position_observer/loop_hz", loopFrequency);
  ROS_INFO("Observer frequency: %f", loopFrequency);

}

void attitudeCallback( const geometry_msgs::QuaternionStamped quaternion )
{
  currentQuaternion = tf::Quaternion(quaternion.quaternion.x, quaternion.quaternion.y, quaternion.quaternion.z, quaternion.quaternion.w);
  tf::Matrix3x3 R_FLU2ENU(currentQuaternion);
  R_FLU2ENU.getRPY(attitude.x, attitude.y, attitude.z);
  globalRotation = attitude.z;
  if( (attitude.z > -M_PI) && (attitude.z < -M_PI_2 ) ) attitude.z += M_PI_2 + M_PI;
  else attitude.z -= M_PI_2;
  // attitude.z -= M_PI_2;
}

void ultraHeightCallback( const std_msgs::Float32 height )
{
  actualHeight = height.data;
}

void gpsHealthCallback( const std_msgs::UInt8 health )
{
  gpsHealth = health.data;
}

void localPositionCallback( const geometry_msgs::PointStamped localPoint )
{
  
  localPosition.x = localPoint.point.y;
  localPosition.y = -localPoint.point.x;
  localPosition.z = localPoint.point.z;

  // tf::Matrix3x3 R_FLU2ENU(currentQuaternion);

  // tf::Vector3 point( localPoint.point.x, localPoint.point.y, localPoint.point.z );

  // tf::Vector3 position = R_FLU2ENU.inverse() * point;

  // localPosition.x = position.x();
  // localPosition.y = position.y();
  // localPosition.z = position.z();

  // localPosition.x = cos(global_rotation) * localPoint.point.x + sin(global_rotation) * localPoint.point.y;
  // localPosition.y = -sin(global_rotation) * localPoint.point.x + cos(global_rotation) * localPoint.point.y;
  // localPosition.z = localPoint.point.z;
}

void observerLoopCallback( const ros::TimerEvent& )
{

  if( gpsHealth > 3 ) {
    currentPose.linear.x = localPosition.x;
    currentPose.linear.y = localPosition.y;
  }
  currentPose.linear.z = actualHeight;

  currentPose.angular.x = attitude.x;
  currentPose.angular.y = attitude.y;
  currentPose.angular.z = attitude.z;

  currentPosePub.publish(currentPose);

}