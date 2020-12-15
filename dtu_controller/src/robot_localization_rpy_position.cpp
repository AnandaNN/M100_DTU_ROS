/** @file robot_localization_rpy_position.cpp
 *  @version 1.0
 *  @date December, 2020
 *
 *  @brief
 *  
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

// standard libraries
#include <algorithm>

// ROS includes
#include <tf/tf.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "pid.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Vector3.h"
#include "type_defines.h"

void readParameters( ros::NodeHandle nh );
void controlCallback( const ros::TimerEvent& );
void checkControlStatusCallback( const std_msgs::UInt8 value );
void updateReferenceCallback( const geometry_msgs::Twist reference );
void robotLocalizationCallback( const nav_msgs::Odometry odom );
void rampReferenceUpdate();

// Constants
const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;

// Control publisher
ros::Publisher controlValuePub;
ros::Publisher currentFramePub;

// Global sub/pub variables
sensor_msgs::Joy controlValue;
geometry_msgs::Twist currentReference;
geometry_msgs::Twist goalReference;
geometry_msgs::Twist referenceSteps;

nav_msgs::Odometry robotLocalizationOdom;

// Global random values
uint8_t controlStatus = STOP_CONTROLLER;
float loopFrequency;
bool referenceUpdated = false;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_localization_rpy_position");
  ros::NodeHandle nh;

  readParameters( nh );

  // Publisher for control values
  controlValuePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zvelocity", 1);
  currentFramePub = nh.advertise<geometry_msgs::Twist>("current_frame_reference", 1);

  // Subscriber
  ros::Subscriber controlStatusSub = nh.subscribe("control_status", 1, &checkControlStatusCallback );
  ros::Subscriber referenceSub = nh.subscribe("current_frame_goal_reference", 1, &updateReferenceCallback );
  ros::Subscriber robotLocalizationSub = nh.subscribe("/odometry/filtered_map", 1, &robotLocalizationCallback );

  // Initialize the 4 control values
  controlValue.axes.push_back(0);
  controlValue.axes.push_back(0);
  controlValue.axes.push_back(0);
  controlValue.axes.push_back(0);

  currentReference.linear.x = 0;
  currentReference.linear.y = 0;
  currentReference.linear.z = 0;
  currentReference.angular.x = 0;
  currentReference.angular.y = 0;
  currentReference.angular.z = 0;

  goalReference.linear.x = 0;
  goalReference.linear.y = 0;
  goalReference.linear.z = 0;
  goalReference.angular.x = 0;
  goalReference.angular.y = 0;
  goalReference.angular.z = 0;

  referenceSteps.linear.x = 0;
  referenceSteps.linear.y = 0;
  referenceSteps.linear.z = 0;
  referenceSteps.angular.x = 0;
  referenceSteps.angular.y = 0;
  referenceSteps.angular.z = 0;

  // Start the control loop timer
  
  ros::Timer loopTimer = nh.createTimer(ros::Duration(1.0/loopFrequency), controlCallback);

  ROS_INFO("XY Controller is running and ready");

  ros::spin();

  return 0;
}

void readParameters( ros::NodeHandle nh )
{
  // Read parameter file
  if ( !nh.getParam("/dtu_controller/xy_position_controller/loop_hz", loopFrequency) )
    loopFrequency = 50.0;
  ROS_INFO("Controller frequency: %f", loopFrequency);

}

double clampSymmetric(double val, double  ext)
{
  return std::max(-ext, std::min(val, ext));
}

void controlCallback( const ros::TimerEvent& )
{
  tf::Quaternion qBF(
      robotLocalizationOdom.pose.pose.orientation.x,
      robotLocalizationOdom.pose.pose.orientation.y,
      robotLocalizationOdom.pose.pose.orientation.z,
      robotLocalizationOdom.pose.pose.orientation.w);
  tf::Matrix3x3 RBF(qBF);
  double rollBF, pitchBF, yawBF;
  RBF.getRPY(rollBF, pitchBF, yawBF);
  tf::Matrix3x3 RLF;
  RLF.setRPY(0.0, 0.0, yawBF);
  tf::Vector3 pBF(
      robotLocalizationOdom.pose.pose.position.x,
      robotLocalizationOdom.pose.pose.position.y,
      robotLocalizationOdom.pose.pose.position.z);
  tf::Vector3 pRF(
      goalReference.linear.x,
      goalReference.linear.y,
      goalReference.linear.z);
  tf::Vector3 pBRF = pRF - pBF;
  tf::Vector3 pBRL = RLF.transpose() * pBRF;
  tf::Vector3 vBFB(
      robotLocalizationOdom.twist.twist.linear.x,
      robotLocalizationOdom.twist.twist.linear.y,
      robotLocalizationOdom.twist.twist.linear.z);
  tf::Vector3 vBFL = RLF.transpose() * RBF * vBFB;
  double yawRF = goalReference.angular.z;

  if( controlStatus == RUNNING)
  {

    rampReferenceUpdate();

    controlValue.axes[0] = clampSymmetric(-30.0 * deg2rad * pBRL.y() + 15.0 * deg2rad * vBFL.y(), 5.0 * deg2rad); // roll
    controlValue.axes[1] = clampSymmetric(30.0 * deg2rad * pBRL.x() - 15.0 * deg2rad * vBFL.x(), 5.0 * deg2rad); // pitch
    controlValue.axes[2] = clampSymmetric(pBRL.z(), 1); // Z rate
    controlValue.axes[3] = clampSymmetric(yawRF - yawBF, 1); // Yaw rate

    controlValue.header.stamp = ros::Time::now();
    controlValuePub.publish(controlValue);

  }
  else if( controlStatus == STOP_CONTROLLER )
  {
    controlValue.axes[0] = 0;
    controlValue.axes[1] = 0;
    controlValue.axes[2] = 0;
    controlValue.axes[3] = 0;
  }

}

void robotLocalizationCallback( const nav_msgs::Odometry odom )
{
  robotLocalizationOdom = odom;
}

void updateReferenceCallback( const geometry_msgs::Twist reference )
{
  // ROS_INFO("Reference udpated");
  goalReference = reference;
  referenceUpdated = true;
}

void rampReferenceUpdate()
{
  float maxV = 1.5; // m/s
  float maxRot = 60 * M_PI / 180.0;

  if( referenceUpdated && goalReference.angular.x ) {
    currentReference.linear.x = goalReference.linear.x;
    currentReference.linear.y = goalReference.linear.y;
    currentReference.linear.z = goalReference.linear.z;

    currentReference.angular.z = goalReference.angular.z;
    referenceUpdated = false;
  }
  else {
    // Update step sizes
    if( referenceUpdated )
    {
      float totalDist = sqrt( pow(goalReference.linear.x-currentReference.linear.x, 2) + pow(goalReference.linear.y-currentReference.linear.y, 2) + pow(goalReference.linear.z-currentReference.linear.z, 2) );
      float estTime = totalDist / maxV;
      referenceSteps.linear.x = (goalReference.linear.x - currentReference.linear.x) / (estTime * loopFrequency);
      referenceSteps.linear.y = (goalReference.linear.y - currentReference.linear.y) / (estTime * loopFrequency);
      referenceSteps.linear.z = (goalReference.linear.z - currentReference.linear.z) / (estTime * loopFrequency);

      referenceSteps.angular.z = (goalReference.angular.z - currentReference.angular.z) / (estTime * loopFrequency);

      referenceUpdated = false;

      // ROS_INFO("### Reference Updated ###");
      // ROS_INFO("Est travel time: %.2f", estTime);
      // ROS_INFO("Steps: %.4f  %.4f  %.4f", referenceSteps.linear.x, referenceSteps.linear.y, referenceSteps.linear.z);
    }
    
    float tolerance = 2;
    // Update X
    if( fabs(goalReference.linear.x - currentReference.linear.x) > fabs(tolerance * referenceSteps.linear.x) ) currentReference.linear.x += referenceSteps.linear.x;
    else currentReference.linear.x = goalReference.linear.x;
    // Update Y
    if( fabs(goalReference.linear.y - currentReference.linear.y) > fabs(tolerance * referenceSteps.linear.y) ) currentReference.linear.y += referenceSteps.linear.y;
    else currentReference.linear.y = goalReference.linear.y;
    // Update Z
    if( fabs(goalReference.linear.z - currentReference.linear.z) > fabs(tolerance * referenceSteps.linear.z) ) currentReference.linear.z += referenceSteps.linear.z;
    else currentReference.linear.z = goalReference.linear.z;

    // Update Z
    if( fabs(goalReference.angular.z - currentReference.angular.z) > fabs(tolerance * referenceSteps.angular.z) ) currentReference.angular.z += referenceSteps.angular.z;
    else currentReference.angular.z = goalReference.angular.z;

  }
  // ROS_INFO("%.2f  %.2f  %.2f", currentReference.linear.x, currentReference.linear.y, currentReference.linear.z);

  currentFramePub.publish(currentReference);

}

void checkControlStatusCallback( const std_msgs::UInt8 value )
{
  if( controlStatus != value.data ) ROS_INFO("Control Status changed to %d", value.data);
  if( value.data == RESET_CONTROLLERS ) {
    ROS_INFO("RESETING PID CONTROL");
    // Add reset control code
    controlStatus = STOP_CONTROLLER;
  }
  else
  {
    controlStatus = value.data;
  }
}
