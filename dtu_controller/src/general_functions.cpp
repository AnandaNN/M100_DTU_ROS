#include "general_functions.h"

#include <std_msgs/UInt8.h>

#include "type_defines.h"


bool set_control_authority(ros::NodeHandle nh, bool cmd)
{

  ros::ServiceClient sdk_ctrl_authority_service;
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");

  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=cmd;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_INFO("control authority failure!");
    return false;
  }

  return true;
}

bool arm_motors(ros::NodeHandle nh, bool cmd)
{
  ros::ServiceClient arm_control_service;
  arm_control_service = nh.serviceClient<dji_sdk::DroneArmControl> ("/dji_sdk/drone_arm_control");
  
  if(cmd) ROS_INFO("ARMING MOTORS!");
  else ROS_INFO("DISARMNING MOTORS!");
  dji_sdk::DroneArmControl arm;
  arm.request.arm = cmd;
  
  arm_control_service.call(arm);

  if(!arm.response.result)
  {
    ROS_INFO("(Arm/disarmning motors failed");
    return false;
  }
  
  return true;
}

bool set_local_frame(ros::NodeHandle nh)
{
  ros::ServiceClient set_local_pos_ref;
  set_local_pos_ref = nh.serviceClient<dji_sdk::SetLocalPosRef> ("/dji_sdk/set_local_pos_ref");
  
  dji_sdk::SetLocalPosRef setReference;
  
  if(!set_local_pos_ref.call(setReference))
  {
    ROS_INFO("Failed to set local pose ref. GPS to weak");
    return false;
  }
  
  return true;
}

void ControllerInterface::init_interface( ros::NodeHandle nh )
{
  cmdStatusPub = nh.advertise<std_msgs::UInt8> ("/dtu_controller/control_status",1);
  positioningPub = nh.advertise<std_msgs::UInt8> ("/dtu_controller/positioning",1);
  referencePub = nh.advertise<geometry_msgs::Twist> ("/dtu_controller/current_frame_goal_reference",1);
}

void ControllerInterface::holdPosition( ros::NodeHandle nh )
{
  boost::shared_ptr<geometry_msgs::Twist const> sharedPose;
  geometry_msgs::Twist currentPose;
  sharedPose = ros::topic::waitForMessage<geometry_msgs::Twist>("/dtu_controller/current_frame_pose", nh);
  if(sharedPose != NULL){
    currentPose = *sharedPose;
  }
  ROS_INFO("Setting reference to:\nX = %.2f\nY = %.2f\nZ = %.2f\nYaw = %.2f\n", currentPose.linear.x, currentPose.linear.y, currentPose.linear.z, currentPose.angular.z);

  currentReference.angular.x = 1; // Force instant change and not ramp
  currentReference.angular.y = 0;
  currentReference.angular.z = currentPose.angular.z;

  currentReference.linear.x = currentPose.linear.x;
  currentReference.linear.y = currentPose.linear.y;
  currentReference.linear.z = currentPose.linear.z;

  referencePub.publish(currentReference);

}

void ControllerInterface::switchPositioning( ros::NodeHandle nh, int positioning )
{
  set_control_status(RESET_CONTROLLERS);
  ros::Duration(0.05).sleep();
  std_msgs::UInt8 msg;
  msg.data = positioning;
  positioningPub.publish(msg);
  ros::Duration(0.05).sleep();
  holdPosition( nh );
  ros::Duration(0.03).sleep();
  set_control_status(RUNNING);
}

void ControllerInterface::getCurrentPosition( ros::NodeHandle nh, geometry_msgs::Twist * currentPose )
{
  boost::shared_ptr<geometry_msgs::Twist const> sharedPose;
  sharedPose = ros::topic::waitForMessage<geometry_msgs::Twist>("/dtu_controller/current_frame_pose", nh);
  if(sharedPose != NULL){
    *currentPose = *sharedPose;
  }

}

void ControllerInterface::set_control_status( uint8_t cmd ) 
{
  std_msgs::UInt8 msg;
  msg.data = cmd;
  controlStatus = cmd;
  cmdStatusPub.publish(msg);
}

void ControllerInterface::set_reference( float x, float y, float z, float yaw)
{
  currentReference.angular.x = 0;
  currentReference.angular.y = 0;
  currentReference.angular.z = yaw;

  currentReference.linear.x = x;
  currentReference.linear.y = y;
  currentReference.linear.z = z;

  referencePub.publish(currentReference);
}

void ControllerInterface::land_copter()
{
  float current = currentReference.linear.z;
  int n = 30;
  float step = (current + 0.35)/(float) n;

  for( int i = 0; i < (n+20); i++ )
  {
    currentReference.linear.z -= step;
    referencePub.publish(currentReference);
    ros::Duration( 3.0/n ).sleep();
  }

  

}