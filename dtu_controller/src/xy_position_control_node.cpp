/** @file xy_position_control_node.cpp
 *  @version 3.3
 *  @date June, 2020
 *
 *  @brief
 *  
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

// #include "xy_position_control_node.h"

// ROS includes
#include <tf/tf.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "pid.h"
#include "type_defines.h"

void readParameters( ros::NodeHandle nh );
void controlCallback( const ros::TimerEvent& );
void checkControlStatusCallback( const std_msgs::UInt8 value );
void updateReferenceCallback( const geometry_msgs::Twist reference );
void updatePoseCallback( const geometry_msgs::Twist pose );
void robotLocalizationCallback( const nav_msgs::Odometry pose );
void rampReferenceUpdate();

// Control publisher
ros::Publisher controlValuePub;
ros::Publisher currentFramePub;

// Global sub/pub variables
sensor_msgs::Joy controlValue;
geometry_msgs::Twist currentPose;
geometry_msgs::Twist currentReference;
geometry_msgs::Twist goalReference;
geometry_msgs::Twist referenceSteps;

// Global random values
uint8_t controlStatus = STOP_CONTROLLER;
float loopFrequency;
bool referenceUpdated = false;
bool useRobotLocalization = false;

// PID Controllers
PID *xPid;
PID *yPid;
PID *zVelPid;
PID *yawRatePid;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xy_position_control_node");
  ros::NodeHandle nh;

  // ros::Duration(3).sleep();

  readParameters( nh );
  nh.param("/dtu_controller/xy_position_controller/use_robot_localization", useRobotLocalization, false);
  if( useRobotLocalization ) ROS_INFO("Using Robot Localization for position control");

  // Publisher for control values
  controlValuePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_xy_yawrate_zvelocity", 1);
  currentFramePub = nh.advertise<geometry_msgs::Twist>("current_frame_reference", 1);

  // Subscriber
  ros::Subscriber controlStatusSub = nh.subscribe("control_status", 1, &checkControlStatusCallback );
  ros::Subscriber referenceSub = nh.subscribe("current_frame_goal_reference", 1, &updateReferenceCallback );

  ros::Subscriber poseSub;
  ros::Subscriber robotLocalizationSub;
  if( useRobotLocalization )
  {
    robotLocalizationSub = nh.subscribe("/odometry/filtered_map", 1, &robotLocalizationCallback );
    ROS_INFO("Subscribed to robot localization for position control");
  }
  else{
    poseSub = nh.subscribe("current_frame_pose", 1, &updatePoseCallback );
    ROS_INFO("Subscribed to dtu_controller for position control");
  }


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
  
  currentPose.linear.x = 0;
  currentPose.linear.y = 0;
  currentPose.linear.z = 0;
  currentPose.angular.x = 0;
  currentPose.angular.y = 0;
  currentPose.angular.z = 0;

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
  nh.getParam("/dtu_controller/xy_position_controller/loop_hz", loopFrequency);
  ROS_INFO("Controller frequency: %f", loopFrequency);

  float kp, ki, kd, max, min;
  nh.getParam("/dtu_controller/xy_position_controller/x_pid_params/kp", kp);
  nh.getParam("/dtu_controller/xy_position_controller/x_pid_params/ki", ki);
  nh.getParam("/dtu_controller/xy_position_controller/x_pid_params/kd", kd);
  nh.getParam("/dtu_controller/xy_position_controller/x_pid_params/min", min);
  nh.getParam("/dtu_controller/xy_position_controller/x_pid_params/max", max);
  xPid = new PID( 1/loopFrequency, max, min, kp, kd, ki );

  nh.getParam("/dtu_controller/xy_position_controller/y_pid_params/kp", kp);
  nh.getParam("/dtu_controller/xy_position_controller/y_pid_params/ki", ki);
  nh.getParam("/dtu_controller/xy_position_controller/y_pid_params/kd", kd);
  nh.getParam("/dtu_controller/xy_position_controller/y_pid_params/min", min);
  nh.getParam("/dtu_controller/xy_position_controller/y_pid_params/max", max);
  yPid = new PID( 1/loopFrequency, max, min, kp, kd, ki );

  nh.getParam("/dtu_controller/xy_position_controller/zvel_pid_params/kp", kp);
  nh.getParam("/dtu_controller/xy_position_controller/zvel_pid_params/ki", ki);
  nh.getParam("/dtu_controller/xy_position_controller/zvel_pid_params/kd", kd);
  nh.getParam("/dtu_controller/xy_position_controller/zvel_pid_params/min", min);
  nh.getParam("/dtu_controller/xy_position_controller/zvel_pid_params/max", max);
  zVelPid = new PID( 1/loopFrequency, max, min, kp, kd, ki );

  nh.getParam("/dtu_controller/xy_position_controller/yawrate_pid_params/kp", kp);
  nh.getParam("/dtu_controller/xy_position_controller/yawrate_pid_params/ki", ki);
  nh.getParam("/dtu_controller/xy_position_controller/yawrate_pid_params/kd", kd);
  nh.getParam("/dtu_controller/xy_position_controller/yawrate_pid_params/min", min);
  nh.getParam("/dtu_controller/xy_position_controller/yawrate_pid_params/max", max);
  yawRatePid = new PID( 1/loopFrequency, max, min, kp, kd, ki );
}

void controlCallback( const ros::TimerEvent& )
{

  if( controlStatus == RUNNING)
  {

    rampReferenceUpdate();

    float xCmd = xPid->calculate(currentReference.linear.x, currentPose.linear.x);
    float yCmd = yPid->calculate(currentReference.linear.y, currentPose.linear.y);
    controlValue.axes[2] = zVelPid->calculate(currentReference.linear.z, currentPose.linear.z);
    controlValue.axes[3] = yawRatePid->calculate(currentReference.angular.z, currentPose.angular.z) ;

    controlValue.axes[0] = cos(currentPose.angular.z) * xCmd + sin(currentPose.angular.z) * yCmd;
    controlValue.axes[1] = -sin(currentPose.angular.z) * xCmd + cos(currentPose.angular.z) * yCmd;

    controlValue.header.stamp = ros::Time::now();

    controlValuePub.publish(controlValue);

  }
  else if( controlStatus == STOP_CONTROLLER )
  {
    xPid->reset_control();
    yPid->reset_control();
    zVelPid->reset_control();
    yawRatePid->reset_control();
    controlValue.axes[0] = 0;
    controlValue.axes[1] = 0;
    controlValue.axes[2] = 0;
    controlValue.axes[3] = 0;
  }

}

void robotLocalizationCallback( const nav_msgs::Odometry pose )
{
  //ROS_INFO("Pose udpated");
  currentPose.linear.x = pose.pose.pose.position.x;
  currentPose.linear.y = pose.pose.pose.position.y;
  currentPose.linear.z = pose.pose.pose.position.z;

  tf::Quaternion currentQuaternion = tf::Quaternion(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
  tf::Matrix3x3 R_FLU2ENU(currentQuaternion);
  R_FLU2ENU.getRPY(currentPose.angular.x, currentPose.angular.y, currentPose.angular.z);
}

void updatePoseCallback( const geometry_msgs::Twist pose )
{
  // ROS_INFO("Pose udpated");
  currentPose = pose;
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
    xPid->reset_control();
    yPid->reset_control();
    zVelPid->reset_control();
    yawRatePid->reset_control();
    controlStatus = STOP_CONTROLLER;
  }
  else
  {
    controlStatus = value.data;
  }
}
