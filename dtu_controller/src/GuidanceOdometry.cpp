/** @file guidanceOdometry.cpp
 *  @version 1.0
 *  @date August, 2020
 *
 *  @brief
 *  Class for handling Guidance Visiual odometry for missions
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#include "GuidanceOdometry.h"
#include "type_defines.h"
#include <tf/transform_broadcaster.h>

/***********************************************
 * 
 * Starts the guidance based odometry with 0 at
 * the current position. Starts subscribers for
 * getting the guidance data
 * 
 * ********************************************/
void GuidanceOdometry::startOdometry( ros::NodeHandle nh )
{
  ROS_INFO( "Starting odometry" );

  // Guidance subscribers
  _ultraSonicSubscriber = nh.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic", 0, &GuidanceOdometry::ultraSonicCallback, this);
  _motionSubscriber = nh.subscribe<guidance::Motion>("/guidance/motion", 0, &GuidanceOdometry::motionCallback, this);

  // Wait for first position messages recieved
  while( _motionMsgNum < 1 && _ultraMsgNum < 1 )
  {
    ros::spinOnce();
  }

  // Recenter the odometry around the current point
  bool resetGood = false;
  while( !resetGood )
  {
    ROS_INFO("Startup resetting");
    resetGood = resetOdometry();
    ros::spinOnce();
  }

}

// 
void GuidanceOdometry::ultraSonicCallback( const sensor_msgs::LaserScan scan )
{
  _ultraScan = scan;

  if( scan.intensities[0] )
  {
    _height = scan.ranges[0];
  }

  _ultraMsgNum++;
}

/***********************************************
 * 
 * Callback function for reading the velocity
 * and position from the motion (visual odometry)
 * topic provided by the Guidance sensor
 * Around 60Hz
 * 
 * ********************************************/
void GuidanceOdometry::motionCallback( const guidance::Motion motion )
{
  _motion = motion;
  _currentQuat = tf::Quaternion( motion.q1, motion.q2, motion.q3, motion.q0 );
  
  tf::Vector3 globalPosition(motion.position_in_global_x - _offsetPosition.getX(), motion.position_in_global_y - _offsetPosition.getY(), motion.position_in_global_z - _offsetPosition.getZ());
  tf::Vector3 motionVelocity( motion.velocity_in_global_x, motion.velocity_in_global_y, motion.velocity_in_global_z );

  tf::Matrix3x3 rotation( _offsetQuat );
  tf::Matrix3x3 flip( tf::Quaternion(1, 0, 0, 0));

  _currentLocalVelocity = flip*rotation.inverse() * motionVelocity;
  _currentLocalPosition = flip*rotation.inverse() * globalPosition;
  
  // ROS_INFO("Vel : %.2f %.2f %.2f", _currentLocalVelocity.getX(), _currentLocalVelocity.getY(), _currentLocalVelocity.getZ() );
  // ROS_INFO("Pos : %.2f %.2f %.2f", _currentLocalPosition.getX(), _currentLocalPosition.getY(), _currentLocalPosition.getZ() );

  _motionMsgNum++;
}

bool GuidanceOdometry::resetOdometry()
{
  // ROS_INFO("Guidance motion odometry resetting");
  // ROS_INFO("Att: %d   Pos: %d", _motion.attitude_status, _motion.position_status);
  if( _motion.attitude_status > 0 )
  {
    _offsetQuatRaw = tf::Quaternion( _motion.q1, _motion.q2, _motion.q3, _motion.q0 );
    _offsetQuat = tf::Quaternion( 0, 0, _motion.q3, _motion.q0 );
  }
  else return false;

  if( _motion.position_status > 0 ) _offsetPosition = tf::Vector3( _motion.position_in_global_x, _motion.position_in_global_y, _motion.position_in_global_z );
  else return false;

  return true;
}
