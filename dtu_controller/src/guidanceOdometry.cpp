/** @file guidanceOdometry.cpp
 *  @version 1.0
 *  @date August, 2020
 *
 *  @brief
 *  Class for handling Guidance Visiual odometry for missions
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#include "guidanceOdometry.h"
#include "type_defines.h"
#include <tf/transform_broadcaster.h>

int guidanceOdometry::test()
{
  ROS_INFO("Test complete");
}

void guidanceOdometry::ultraSonicCallback( const sensor_msgs::LaserScan _scan )
{
  internalUltraSonic = _scan;

  if( _scan.intensities[0] )
  {
    currentHeight = _scan.ranges[0];
  }
}

// void guidanceMotionCallback( const guidance::Motion motion )
// {
  
//   tf::Quaternion quat = tf::Quaternion( motion.q1, motion.q2, motion.q3, motion.q0);
//   tf::Matrix3x3 R_Q2RPY(quat);
//   geometry_msgs::Vector3 rawAttitude, att;
  
//   R_Q2RPY.getRPY(rawAttitude.x, rawAttitude.y, rawAttitude.z);

//   tf::Vector3 globalMotion(motion.position_in_global_x - guidanceOffsetPose.linear.x, motion.position_in_global_y - guidanceOffsetPose.linear.y, motion.position_in_global_z - guidanceOffsetPose.linear.z);

//   tf::Quaternion rpy;
//   rpy.setEuler( rawAttitude.x, rawAttitude.y, guidanceOffsetPose.angular.z ); //-guidanceYawOffset );

//   tf::Matrix3x3 R_G2L(rpy);
//   R_G2L.getRPY( att.x, att.y, att.z );

//   tf::Vector3 poss = R_G2L.inverse() * globalMotion;
//   // ROS_INFO("Ang = %.1f %.1f %.1f", att.x*180/3.14, att.y*180/3.14, att.z*180/3.14);
//   // ROS_INFO("RotPos = %.2f %.2f %.2f", poss.getX(), poss.getY(), poss.getZ());
//   // ROS_INFO("global = %.2f %.2f %.2f", globalMotion.getX(), globalMotion.getY(), globalMotion.getZ() );
//   // ROS_INFO("offset = %.2f %.2f %.2f", guidanceOffsetPose.linear.x, guidanceOffsetPose.linear.y, guidanceOffsetPose.linear.z );
//   // ROS_INFO("guidan = %.2f %.2f %.2f\n", motion.position_in_global_x, motion.position_in_global_y, motion.position_in_global_z);

//   motionVelocity.setX(motion.velocity_in_global_x);
//   motionVelocity.setY(motion.velocity_in_global_y);
//   motionVelocity.setZ(-motion.velocity_in_global_z);
//   // tf::Matrix3x3 R_VEL(quat);
//   motionVelocity = R_G2L.inverse() * motionVelocity;

//   // ROS_INFO("vx: %.2f  vy: %.2f  vz: %.2f", motionVelocity.getX(), motionVelocity.getY(), motionVelocity.getZ());

//   if( !motionInitialized )
//   {
//     if( fabs( motion.position_in_global_x ) < 0.001 || fabs( motion.position_in_global_y ) < 0.001 )
//     {
//       motionInitialized = false;
//     } 
//     else
//     {
//       motionInitialized = true;
//       guidanceOffsetPose.linear.x = motion.position_in_global_x;
//       guidanceOffsetPose.linear.y = motion.position_in_global_y;
//       guidanceOffsetPose.linear.z = motion.position_in_global_z;

//       guidanceOffsetPose.angular.x = 0;
//       guidanceOffsetPose.angular.y = 0;
//       guidanceOffsetPose.angular.z = rawAttitude.z;
//     }
//   }
//   else
//   {
//     guidanceLocalPose.linear.x = poss.getX();
//     guidanceLocalPose.linear.y = -poss.getY();
//     guidanceLocalPose.linear.z = poss.getZ();

//     guidanceLocalPose.angular.x = rawAttitude.x;
//     guidanceLocalPose.angular.y = rawAttitude.y;
//     guidanceLocalPose.angular.z = guidanceOffsetPose.angular.z-rawAttitude.z;

//     if( guidanceLocalPose.angular.z < -M_PI) guidanceLocalPose.angular.z += 2*M_PI;
//     else if( guidanceLocalPose.angular.z > M_PI) guidanceLocalPose.angular.z -= 2*M_PI;

//   }
// }