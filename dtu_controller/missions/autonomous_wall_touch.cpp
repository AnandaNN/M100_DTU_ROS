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
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>

// Own includes
#include "type_defines.h"
#include "general_functions.h"
#include "ContactController.h"

float zError = 0;
float yError = 0;
int newError = 0;
int oldError = 0;
std_msgs::UInt8 trackerEnabled;

geometry_msgs::Twist currentPosition;

ros::Subscriber targetErrorSub;
ros::Subscriber currentPoseSub;
ros::Subscriber targetMsgSub;

void targetErrorCallback( const geometry_msgs::Point pointMsg )
{
  yError = (float)pointMsg.y;
  zError = (float)pointMsg.z;
  // ROS_INFO("Got msg point");
  newError++;
}

void currentPoseCallback( const geometry_msgs::Twist twistMsg )
{
  currentPosition = twistMsg;

  // ROS_INFO("Got msg TWIST");
}

void targetMsgCallback( const std_msgs::UInt8 msg )
{
  
  trackerEnabled = msg;
  // ROS_INFO("Got msg TWIST");
}

int main(int argc, char** argv)
{
  // Setup the node
  ros::init(argc, argv, "autonomous wall touch mission");
  ros::NodeHandle nh;

  ros::Duration(0.1).sleep();
  ControllerInterface controllerInterface;
  controllerInterface.init_interface( nh );

  ros::Duration(4).sleep();
  set_local_frame(nh);
  ros::Duration(1).sleep();

  targetErrorSub = nh.subscribe<geometry_msgs::Point>("/visual_tracker/distance_error", 1, &targetErrorCallback);
  currentPoseSub = nh.subscribe<geometry_msgs::Twist>("current_frame_pose", 1, &currentPoseCallback);

  targetMsgSub = nh.subscribe<std_msgs::UInt8>("/visual_Tracker/target_tracking_msg", 1, &targetMsgCallback);

  ROS_INFO("Ready to take over. Waiting for command");
  
  /******************************************************************/

  ContactController contactController;
  contactController.init( nh );

  // boost::shared_ptr<std_msgs::UInt8 const> sharedTrackEnable;
  //std_msgs::UInt8 trackerEnabled;
  trackerEnabled.data = 0;

  while(ros::ok()) 
  {
    // sharedTrackEnable = ros::topic::waitForMessage<std_msgs::UInt8>("/target_tracking_msg", nh);
    // if(sharedTrackEnable != NULL){
    //     trackerEnabled = *sharedTrackEnable;
    // }

    if( trackerEnabled.data == (uint8_t) 1 )
    {
      ROS_INFO("User selected a target. Starting tracking");
      if( set_control_authority(nh, true) ) ROS_INFO("Got control authorithy");
      else return 0;
      
      ROS_INFO("Stabilizing initial position (3s)");

      controllerInterface.holdPosition( nh );
      ros::Duration(0.01).sleep();
      controllerInterface.set_control_status( RUNNING );
      ros::Duration(4).sleep();
      break;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  float xTarget = currentPosition.linear.x;

  float zTarget = 0.045;

  ROS_INFO("Centering around the target! (3s)");
  controllerInterface.set_reference(xTarget, 0, zTarget, 0);
  ros::Duration(5).sleep();

  ROS_INFO("Starting the approach");
  while( ros::ok()  && xTarget < -1.1)
  {

    xTarget =currentPosition.linear.x + 0.25;
    controllerInterface.set_reference(xTarget, 0, zTarget, 0);

    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  ROS_INFO("Wall reached, preparing for contact!");
  ros::Duration(6).sleep();

  //char input;
  //std::cin >> input;

  bool approachStarted = false;

  ros::Time begin = ros::Time::now(); 

  xTarget = -1.1;

  while( ros::ok() )
  {
    if( !contactController.checkRod() )
    {
      xTarget += 0.005;
      controllerInterface.set_reference(xTarget, 0, zTarget, 0);
    }

    if( contactController.checkRod() )
    {
      controllerInterface.set_control_status( STOP_CONTROLLER );
      ros::Duration(0.05).sleep();
      contactController.startController();
      approachStarted = true;
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

  xTarget = -2.0;
  controllerInterface.set_reference(xTarget, 0, zTarget, 0);
  controllerInterface.set_control_status( RUNNING );

  ros::Duration(5).sleep();

  xTarget = -3;
  controllerInterface.set_reference(xTarget, 0, zTarget, 0);
  ros::Duration(4).sleep();

  if( set_control_authority(nh, false) ) ROS_INFO("Released control authorithy");
  else return 0;

  return 0;
}
