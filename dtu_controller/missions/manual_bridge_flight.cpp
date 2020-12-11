/** @file autonomous_wall_touch.cpp
 *  @version 1.0
 *  @date October, 2020
 *
 *  @brief
 *  Mission for doing autonomous wall touch with visual servoing and user inputs
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */


// Outside includes
#include <ros/ros.h>

// Own includes
#include "general_functions.h"
#include "ContactController.h"

int main(int argc, char** argv)
{
  // Setup the node
  ros::init(argc, argv, "autonomous wall touch mission");
  ros::NodeHandle nh;

  ros::Duration(4).sleep();
  set_local_frame(nh);

  /******************************************************************/

  ContactController contactController;
  contactController.init( nh );

  ROS_INFO("Contact controller started. Will take over once wall is reached!");

  ros::Time begin = ros::Time::now(); 

  while( ros::ok() )
  {

    if( contactController.checkRod() )
    {
      ROS_INFO("Contact Made. Holding position for 10 seconds");
      if( set_control_authority(nh, true) ) ROS_INFO("Got control authorithy");
      else return 0;
      ros::Duration(0.05).sleep();
      contactController.startController();
      begin = ros::Time::now();
      break;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

  }

  ROS_INFO("Contact made. Contact controller started!");

  ros::Time currentTime = ros::Time::now();

  while( ros::ok() )
  {
    
    currentTime = ros::Time::now();

    if( (currentTime.sec - begin.sec) > 10 )
    {
      ROS_INFO("Contact kept for 10 seconds. Releasing!");
      contactController.disengage();
      // contactController.stopController();
      break;
    }

    if( !contactController.checkStatus() )
    {
      ROS_INFO("Contact failed. Flying away");
      break;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

  }

  for(int i = 0; i < 50; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  contactController.stopController();

  if( set_control_authority(nh, false) ) ROS_INFO("Released control authorithy");
  else return 0;

  ros::Duration(2.0).sleep();

  return 0;
}
