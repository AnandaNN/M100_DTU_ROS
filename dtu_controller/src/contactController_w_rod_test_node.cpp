/** @file joy_roll_thrust_controller_node.cpp
 *  @version 3.3
 *  @date June, 2020
 *
 *  @brief
 *  demo sample of joy stick with rpyrate zvel control with ability to switch to roll thrust command scehme for wall interaction
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

// #include "contact_controller_node.h"
// #include "dji_sdk/dji_sdk.h"
// Outside includes
#include <ros/ros.h>
//#include "dji_sdk/dji_sdk.h"

//#include <std_msgs/Bool.h>

//#include <std_msgs/UInt8.h>

//#include <geometry_msgs/Point.h>

// Own includes
#include "type_defines.h"
#include "general_functions.h"
#include "sensor_msgs/Joy.h"
#include "ContactController.h"

bool stopContact = false;
bool contactRunning = false;

void rcCallback( const sensor_msgs::Joy msg) 
{
  // ROS_INFO("%f", msg.axes[1]);
  // if( contactRunning ) 
  stopContact = (msg.axes[1] < -0.2) ? 1 : 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ContactController_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ContactController contactController;
  contactController.init( nh );

  ros::Subscriber rcSub = nh.subscribe<sensor_msgs::Joy>("/dji_sdk/rc", 1, rcCallback );

  while( ros::ok() )
  {
    if( !contactRunning )
    {
      if( contactController.checkRod() )
      {
        ROS_INFO("Contact detected. Taking over!");
        if( set_control_authority(nh, true) )
        {
          ROS_INFO("Got control authorithy");
          contactController.startController();
          contactRunning = true;
        }
        else
          ROS_INFO("Failed to get control");
      }
    }
    else if( contactRunning )
    {
      if( !contactController.checkRod() )
      {
        ROS_INFO("Lost contact. Releasing control");
        if( set_control_authority(nh, false) )
        {
          ROS_INFO("Control released");
          contactController.stopController();
          contactRunning = false;
        }
        else
          ROS_INFO("Failed to release control");
      }
    }

    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  return 0;
}

