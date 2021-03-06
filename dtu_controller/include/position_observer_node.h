/** @file position_observer_node.h
 *  @version 3.3
 *  @date June, 2020
 *
 *  @brief
 *  observer node for estimating the positions used by the controller
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#ifndef POSITION_OBSERVER_NODE_H
#define POSITION_OBSERVER_NODE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <guidance/Motion.h>
#include "GuidanceOdometry.h"

GuidanceOdometry odo;

void attitudeCallback( const geometry_msgs::QuaternionStamped quaternion );
void localPositionCallback( const geometry_msgs::PointStamped localPoint );
void ultraHeightCallback( const std_msgs::Float32 height );
void gpsHealthCallback( const std_msgs::UInt8 health );
void imuCallback( const sensor_msgs::Imu raw_imu );
void wallPositionCallback( const std_msgs::Float32MultiArray internalWallPosition );
void ultrasonicCallback( const sensor_msgs::LaserScan scan );
void guidanceMotionCallback( const guidance::Motion motion );
void visualOdometryCallback( const geometry_msgs::Point msg );
void positioningCallback( const std_msgs::UInt8 msg );
void observerLoopCallback( const ros::TimerEvent& );
void readParameters( ros::NodeHandle nh );

#endif
