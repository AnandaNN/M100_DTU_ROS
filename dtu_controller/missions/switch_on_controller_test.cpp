/** @file basic_control_mission.cpp
 *  @version 3.3
 *  @date July, 2020
 *
 *  @brief
 *  demo sample of joy stick with rpyrate zvel control
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#include "dji_sdk/dji_sdk.h"
#include "type_defines.h"
#include <ros/ros.h>

#include "general_functions.h"

// #include <dji_sdk/SDKControlAuthority.h>
// #include <dji_sdk/DroneArmControl.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_control_mission");
  ros::NodeHandle nh;

  ros::Duration(0.1).sleep();
  ControllerInterface controllerInterface;
  controllerInterface.init_interface( nh );

  // sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
  // arm_control_service = nh.serviceClient<dji_sdk::DroneArmControl> ("/dji_sdk/drone_arm_control");
  // Sleep until dji osdk has access
  ros::Duration(4).sleep();

  set_local_frame(nh);

  ros::Duration(1).sleep();

  ROS_INFO("Ready to take over");
  char input;
  std::cin >> input;

  if( set_control_authority(nh, true) ) ROS_INFO("Got control authorithy");
  else return 0;
  controllerInterface.holdPosition( nh );
  ros::Duration(0.01).sleep();
  controllerInterface.set_control_status( RUNNING );

  ROS_INFO("Ready to give up control");
  std::cin >> input;

  controllerInterface.set_reference( -4.0, 0, 1.0, 0.0 );
  std::cin >> input;

  // controllerInterface.set_reference( -3.0, 0, 1, 0.0 );
  // std::cin >> input;


  // controllerInterface.set_reference( -2.0, 0, 1, 0.0 );
  // std::cin >> input;


  // controllerInterface.set_reference( -1.5, 0.0, 1, 0.0 );
  // std::cin >> input;


  controllerInterface.set_reference( -2.0, 0.0, 1.0, 0.0 );
  std::cin >> input;


  controllerInterface.set_reference( -4, 0, 1.0, 0.0 );
  std::cin >> input;


  if( set_control_authority(nh, false) ) ROS_INFO("Released control authorithy");
  else return 0;

  // Start the control loop timer
  // ros::spin();

  return 0;
}

