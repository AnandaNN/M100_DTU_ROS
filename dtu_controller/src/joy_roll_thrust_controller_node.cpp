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

#include "joy_roll_thrust_controller_node.h"
#include "dji_sdk/dji_sdk.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3.h"
#include <tf/tf.h>
#include "sensor_msgs/Imu.h"

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

bool motor_status = false;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
ros::ServiceClient arm_control_service;

ros::Publisher ctrlAttitudePub;

ros::Subscriber attitudeQuaternionSub;
ros::Subscriber imuSub;

sensor_msgs::Joy controlValue;
sensor_msgs::Imu imuValue;

geometry_msgs::Vector3 attitude;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_joystick_rpy_control_node");
  ros::NodeHandle nh;

  ros::Duration(5).sleep();

  ctrlAttitudePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zvelocity", 1);
  // ctrlAttitudePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 0);
  // ctrlAttitudePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 0);

  attitudeQuaternionSub = nh.subscribe<geometry_msgs::QuaternionStamped>( "/dji_sdk/attitude", 1, attitudeCallback );
  imuSub = nh.subscribe<sensor_msgs::Imu>( "/dji_sdk/imu", 1, imuCallback );
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  arm_control_service = nh.serviceClient<dji_sdk::DroneArmControl> ("/dji_sdk/drone_arm_control");

  ros::Duration(1).sleep();

  if( obtain_control() ) ROS_INFO("GOT CONTROL!");
  else return 0;

  controlValue.axes.push_back(0);
  controlValue.axes.push_back(0);
  controlValue.axes.push_back(0);
  controlValue.axes.push_back(0);

  ros::Duration(0.5).sleep();

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/50.0), timerCallback);
  ros::Subscriber joy_subscriber = nh.subscribe("dji_sdk/joy", 0, &controlCallback);

  ros::spin();

  ROS_INFO("JOY DJI CONTROL NODE SHUTDOWN!");
  if( release_control() ) ROS_INFO("CONTROL RELEASED!");

  return 0;
}

void imuCallback( const sensor_msgs::Imu imuMsg )
{
  imuValue = imuMsg;
  // ROS_INFO("%f", imuValue.angular_velocity.z);
}

void attitudeCallback( const geometry_msgs::QuaternionStamped quaternion )
{
  tf::Quaternion currentQuaternion(quaternion.quaternion.x, quaternion.quaternion.y, quaternion.quaternion.z, quaternion.quaternion.w);
  tf::Matrix3x3 R_FLU2ENU(currentQuaternion);
  R_FLU2ENU.getRPY(attitude.x, attitude.y, attitude.z);
  
}

void controlCallback( const sensor_msgs::Joy joy_msg )
{
  int thrust_id = 2; // 1
  int yaw_id = 3; // 0
  int roll_id = 0; // 3
  int pitch_id = 1; // 2
  
  float z_vel = 2 * joy_msg.axes[thrust_id];
  float yaw = 100*deg2rad * joy_msg.axes[yaw_id];
  float roll = -25*deg2rad * joy_msg.axes[roll_id];
  float pitch = 25*deg2rad * joy_msg.axes[pitch_id];

  // x = 0, o = 1, tri = 2, sq = 3

  //int button = 
  // if( joy_msg.buttons[2] && motor_status == false ) arm_motors();
  // if( joy_msg.buttons[3] && motor_status == true ) disarm_motors();

  controlValue.axes[0] = roll;
  controlValue.axes[1] = pitch;
  controlValue.axes[2] = z_vel;
  controlValue.axes[3] = yaw;

  if( joy_msg.axes[4] < 0.5 )
  {
    controlValue.axes[0] = roll;
    controlValue.axes[1] = attitude.y;
    controlValue.axes[2] = z_vel;
    controlValue.axes[3] = imuValue.angular_velocity.z;
  }

  // ctrlAttitudePub.publish(controlValue);

  ROS_INFO("Z: %f | Yaw: %f | Roll: %f | Pitch: %f |", z_vel, yaw, roll, pitch);

}

void timerCallback( const ros::TimerEvent& )
{
  //if( motor_status )
    ctrlAttitudePub.publish(controlValue);
}

// Helper Functions

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_INFO("obtain control failed!");
    return false;
  }

  return true;
}

bool release_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=0;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_INFO("release control failed!");
    return false;
  }

  return true;
}

bool arm_motors()
{
  ROS_INFO("ARMING MOTORS!");
  dji_sdk::DroneArmControl arm;
  arm.request.arm = 1;
  
  arm_control_service.call(arm);

  if(!arm.response.result)
  {
    ROS_INFO("Arming motors failed");
    return false;
  }
  else
    motor_status = 1;
  
  return true;
}

// bool disarm_motors()
// {
//   dji_sdk::DroneArmControl arm;
//   arm.request.arm = 0;
  
//   arm_control_service.call(arm);

//   if(!arm.response.result)
//   {
//     ROS_INFO("Disarming motors failed");
//     return false;
//   }
//   else
//     motor_status = 0;
  
//   return true;
// }