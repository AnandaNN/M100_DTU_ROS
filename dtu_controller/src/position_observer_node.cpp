/** @file position_observer_node.cpp
 *  @version 3.3
 *  @date June, 2020
 *
 *  @brief
 *  demo sample of joy stick with rpyrate zvel control
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#include "position_observer_node.h"
#include "dji_sdk/dji_sdk.h"
#include "type_defines.h"


// Publisher
ros::Publisher currentPosePub;

// Subscriber
ros::Subscriber localGPSPositionSub;
ros::Subscriber attitudeQuaternionSub;
ros::Subscriber ultraHeightSub;
ros::Subscriber gpsHealthSub;

ros::Subscriber ultrasonicSub;
ros::Subscriber guidanceMotionSub;

// Global sub/pub variables
geometry_msgs::Twist currentPose;
geometry_msgs::Twist currentReference;

geometry_msgs::Twist guidanceLocalPose;
geometry_msgs::Twist guidanceOffsetPose;

geometry_msgs::Vector3 attitude;
geometry_msgs::Vector3 localPosition;

float globalRotation = 0;
tf::Quaternion currentQuaternion;

float actualHeight = 0;
uint8_t gpsHealth = 0;

// Global random values
float loopFrequency;
bool motionInitialized = false;
float guidanceX = 0;
float guidanceXOffset = 0;
float guidanceY = 0;
float guidanceYOffset = 0;
float guidanceYawOffset = 0;

bool yawInitialized = false;
float yawOffset = 0;

int USE_GUIDANCE = 1;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_observer_node");
  ros::NodeHandle nh;

  ros::Duration(1).sleep();

  readParameters( nh );

  // Publisher for control values
  currentPosePub = nh.advertise<geometry_msgs::Twist>("/dtu_controller/current_frame_pose", 0);

  // Initialize subsrcibers
  localGPSPositionSub = nh.subscribe<geometry_msgs::PointStamped>( "/dji_sdk/local_position", 0, localPositionCallback );
  attitudeQuaternionSub = nh.subscribe<geometry_msgs::QuaternionStamped>( "/dji_sdk/attitude", 0, attitudeCallback );
  gpsHealthSub = nh.subscribe<std_msgs::UInt8>("/dji_sdk/gps_health", 0, gpsHealthCallback);

  // Doesn't Work
  //ultraHeightSub = nh.subscribe<std_msgs::Float32>("/dji_sdk/height_above_takeoff", 0, ultraHeightCallback);

  ultrasonicSub = nh.subscribe<sensor_msgs::LaserScan>("/guidance_uart/ultrasonic", 0, ultrasonicCallback);
  guidanceMotionSub = nh.subscribe<guidance_uart::Motion>("/guidance_uart/motion", 0, guidanceMotionCallback);

  ros::Duration(1).sleep();

  // Start the control loop timer
  
  ros::Timer loopTimer = nh.createTimer(ros::Duration(1.0/loopFrequency), observerLoopCallback);

  ros::spin();

  return 0;
}

void readParameters( ros::NodeHandle nh )
{
  // Read parameter file
  nh.getParam("/dtu_controller/position_observer/loop_hz", loopFrequency);
  ROS_INFO("Observer frequency: %f", loopFrequency);

}

void attitudeCallback( const geometry_msgs::QuaternionStamped quaternion )
{
  currentQuaternion = tf::Quaternion(quaternion.quaternion.x, quaternion.quaternion.y, quaternion.quaternion.z, quaternion.quaternion.w);
  tf::Matrix3x3 R_FLU2ENU(currentQuaternion);
  R_FLU2ENU.getRPY(attitude.x, attitude.y, attitude.z);
  
  if( (attitude.z > -M_PI) && (attitude.z < -M_PI_2 ) ) attitude.z += M_PI_2 + M_PI;
  else attitude.z -= M_PI_2;

  if(!yawInitialized)
  {
    yawOffset = attitude.z;
    yawInitialized = true;
    ROS_INFO("YawOffset initialized to %f\n",yawOffset);
  }

  // ROS_INFO("Before Offset = %f\n",attitude.z);

  attitude.z = attitude.z - yawOffset;

  if( attitude.z < -M_PI) attitude.z += 2*M_PI;
  else if( attitude.z > M_PI) attitude.z -= 2*M_PI;

  // ROS_INFO("After  Offset = %f\n",attitude.z);

  // attitude.z -= M_PI_2;
}

// Doesn't Work
void ultraHeightCallback( const std_msgs::Float32 height )
{
  actualHeight = height.data;
}

void gpsHealthCallback( const std_msgs::UInt8 health )
{
  gpsHealth = health.data;
}

void ultrasonicCallback( const sensor_msgs::LaserScan scan )
{
  if( scan.intensities[0] )
  {
    actualHeight = scan.ranges[0];
  }
}

void guidanceMotionCallback( const guidance_uart::Motion motion )
{
  
  tf::Quaternion quat = tf::Quaternion( motion.q1, motion.q2, motion.q3, motion.q0);
  tf::Matrix3x3 R_Q2RPY(quat);
  geometry_msgs::Vector3 rawAttitude, att;
  
  R_Q2RPY.getRPY(rawAttitude.x, rawAttitude.y, rawAttitude.z);

  tf::Vector3 globalMotion(motion.position_in_global_x - guidanceOffsetPose.linear.x, motion.position_in_global_y - guidanceOffsetPose.linear.y, motion.position_in_global_z - guidanceOffsetPose.linear.z);

  tf::Quaternion rpy;
  rpy.setEuler( rawAttitude.x, rawAttitude.y, guidanceOffsetPose.angular.z ); //-guidanceYawOffset );

  tf::Matrix3x3 R_G2L(rpy);
  R_G2L.getRPY( att.x, att.y, att.z );

  tf::Vector3 poss = R_G2L.inverse() * globalMotion;
  // ROS_INFO("Ang = %.1f %.1f %.1f", att.x*180/3.14, att.y*180/3.14, att.z*180/3.14);
  // ROS_INFO("RotPos = %.2f %.2f %.2f", poss.getX(), poss.getY(), poss.getZ());
  // ROS_INFO("global = %.2f %.2f %.2f", globalMotion.getX(), globalMotion.getY(), globalMotion.getZ() );
  // ROS_INFO("offset = %.2f %.2f %.2f", guidanceOffsetPose.linear.x, guidanceOffsetPose.linear.y, guidanceOffsetPose.linear.z );
  // ROS_INFO("guidan = %.2f %.2f %.2f\n", motion.position_in_global_x, motion.position_in_global_y, motion.position_in_global_z);

  if( !motionInitialized )
  {
    if( fabs( motion.position_in_global_x ) < 0.001 || fabs( motion.position_in_global_y ) < 0.001 )
    {
      motionInitialized = false;
    } 
    else
    {
      motionInitialized = true;
      guidanceOffsetPose.linear.x = motion.position_in_global_x;
      guidanceOffsetPose.linear.y = motion.position_in_global_y;
      guidanceOffsetPose.linear.z = motion.position_in_global_z;

      guidanceOffsetPose.angular.x = 0;
      guidanceOffsetPose.angular.y = 0;
      guidanceOffsetPose.angular.z = rawAttitude.z;
    }
  }
  else
  {
    // guidanceX = poss.getX();
    // guidanceY = -poss.getY();
    guidanceLocalPose.linear.x = poss.getX();
    guidanceLocalPose.linear.y = -poss.getY();
    guidanceLocalPose.linear.z = poss.getZ();

    guidanceLocalPose.angular.x = rawAttitude.x;
    guidanceLocalPose.angular.y = rawAttitude.y;
    guidanceLocalPose.angular.z = guidanceOffsetPose.angular.z-rawAttitude.z;

  }
}


// void guidanceMotionCallback( const guidance_uart::Motion motion )
// {
//   tf::Quaternion quat = tf::Quaternion( motion.q1, motion.q2, motion.q3, motion.q0);
//   tf::Matrix3x3 R_Q2RPY(quat);
//   geometry_msgs::Vector3 att;
  
//   R_Q2RPY.getRPY(att.x, att.y, att.z);
//   float rawYaw = att.z;

//   tf::Vector3 globalMotion(motion.position_in_global_x - guidanceXOffset, motion.position_in_global_y - guidanceYOffset, motion.position_in_global_z*0);

//   tf::Quaternion rpy;
//   rpy.setEuler( 0*att.y, 0*att.x, att.z ); //-guidanceYawOffset );

//   tf::Matrix3x3 R_G2L(rpy);
//   R_G2L.getRPY( att.x, att.y, att.z );

//   tf::Vector3 poss = R_G2L * globalMotion;
//   ROS_INFO("Ang = %.1f %.1f %.1f\n", att.x*180/3.14, att.y*180/3.14, att.z*180/3.14);
//   ROS_INFO("Pos = %.2f %.2f %.2f\n", poss.getX(), poss.getY(), poss.getZ());

//   if( !motionInitialized )
//   {
//     guidanceXOffset = motion.position_in_global_x;
//     guidanceYOffset = motion.position_in_global_y;
//     guidanceYawOffset = rawYaw;
//     if( fabs( guidanceXOffset ) < 0.001 || fabs( guidanceYOffset ) < 0.001 ) motionInitialized = false;
//     else motionInitialized = true;
//     ROS_INFO("Motion offset initialized to %f\t%f\n", guidanceXOffset, guidanceYOffset);
//   }
//   else
//   {
//     guidanceX = poss.getX();
//     guidanceY = poss.getY();
//     // guidanceX = (motion.position_in_global_x - guidanceXOffset);
//     // guidanceY = -(motion.position_in_global_y - guidanceYOffset);
//   }
// }

void localPositionCallback( const geometry_msgs::PointStamped localPoint )
{
  
  localPosition.x = localPoint.point.y;
  localPosition.y = -localPoint.point.x;
  localPosition.z = localPoint.point.z;

  // tf::Matrix3x3 R_FLU2ENU(currentQuaternion);

  // tf::Vector3 point( localPoint.point.x, localPoint.point.y, localPoint.point.z );

  // tf::Vector3 position = R_FLU2ENU.inverse() * point;

  // localPosition.x = position.x();
  // localPosition.y = position.y();
  // localPosition.z = position.z();

  // localPosition.x = cos(global_rotation) * localPoint.point.x + sin(global_rotation) * localPoint.point.y;
  // localPosition.y = -sin(global_rotation) * localPoint.point.x + cos(global_rotation) * localPoint.point.y;
  // localPosition.z = localPoint.point.z;

}

void observerLoopCallback( const ros::TimerEvent& )
{

  if( USE_GUIDANCE ){
    currentPose.linear.x = guidanceLocalPose.linear.x;
    currentPose.linear.y = guidanceLocalPose.linear.y;
    currentPose.linear.z = actualHeight;

    // currentPose.angular.x = guidanceLocalPose.angular.x;
    // currentPose.angular.y = guidanceLocalPose.angular.y;
    // currentPose.angular.z = guidanceLocalPose.angular.z;

    currentPose.angular.x = attitude.x;
    currentPose.angular.y = attitude.y;
    currentPose.angular.z = attitude.z;

    currentPosePub.publish(currentPose);
  }
  else
  {
    
    currentPose.linear.x = guidanceX;
    currentPose.linear.y = guidanceY;

    // if( gpsHealth > 3 ) {
    //   currentPose.linear.x = localPosition.x;
    //   currentPose.linear.y = localPosition.y;
    // }
    currentPose.linear.z = actualHeight;

    currentPose.angular.x = attitude.x;
    currentPose.angular.y = attitude.y;
    currentPose.angular.z = attitude.z;

    currentPosePub.publish(currentPose);
    
  }
  
}