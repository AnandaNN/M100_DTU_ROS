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

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Publisher
ros::Publisher currentPosePub;

// Subscriber
ros::Subscriber localGPSPositionSub;
ros::Subscriber attitudeQuaternionSub;
ros::Subscriber ultraHeightSub;
ros::Subscriber gpsHealthSub;

ros::Subscriber ultrasonicSub;
ros::Subscriber guidanceMotionSub;
ros::Subscriber wallPositionSub;
ros::Subscriber imuSub;

ros::Subscriber visualOdometrySub;

ros::Subscriber positioningSub;

// Global sub/pub variables
geometry_msgs::Twist currentPose;
geometry_msgs::Twist currentReference;

geometry_msgs::Twist guidanceLocalPose;
geometry_msgs::Twist guidanceOffsetPose;

geometry_msgs::Point visualOdometryPose;

geometry_msgs::Vector3 attitude;
geometry_msgs::Vector3 localPosition;

tf::Vector3 motionVelocity;

geometry_msgs::Twist wallPosition;

float globalRotation = 0;
tf::Quaternion currentQuaternion;

float actualHeight = 0;
float ultraHeight = 0;
uint8_t gpsHealth = 0;

float gyroZ = 0;

// Global random values
float loopFrequency;
bool motionInitialized = false;

bool yawInitialized = false;
float yawOffset = 0;

bool resetPositioning = true;
int positioning = GPS;
bool simulation = 0;

bool gpsReady = false;
bool laserReady = false;
bool motionReady = false;

int tfSequence = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_observer_node");
  ros::NodeHandle nh;

  ros::Duration(1).sleep();

  if( !nh.param("/dtu_controller/position_observer/simulation", simulation, false) ) ROS_INFO("Simulation not specified");
  if( !nh.param("/dtu_controller/position_observer/positioning", positioning, (int)GPS) ) ROS_INFO("NO POSITIONING SPECIFIED");

  if( simulation ) ROS_INFO("############## Simultion is set TRUE #################");

  readParameters( nh );

  // if( !simulation && positioning != VISUAL_ODOMETRY ) odo.startOdometry( nh );

  // Publisher for control values
  currentPosePub = nh.advertise<geometry_msgs::Twist>("/dtu_controller/current_frame_pose", 1);

  // Initialize subsrcibers
  // Subscribers from DJI node
  localGPSPositionSub = nh.subscribe<geometry_msgs::PointStamped>( "/dji_sdk/local_position", 1, localPositionCallback );
  attitudeQuaternionSub = nh.subscribe<geometry_msgs::QuaternionStamped>( "/dji_sdk/attitude", 1, attitudeCallback );
  gpsHealthSub = nh.subscribe<std_msgs::UInt8>("/dji_sdk/gps_health", 1, gpsHealthCallback);
  ultraHeightSub = nh.subscribe<std_msgs::Float32>("/dji_sdk/height_above_takeoff", 1, ultraHeightCallback);
  imuSub = nh.subscribe<sensor_msgs::Imu>("/dji_sdk/imu", 1, imuCallback);

  // Guidance subscribers
  ultrasonicSub = nh.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic", 1, ultrasonicCallback);
  guidanceMotionSub = nh.subscribe<guidance::Motion>("/guidance/motion", 1, guidanceMotionCallback);

  // Laser scanner subscriber
  wallPositionSub = nh.subscribe<std_msgs::Float32MultiArray>("/dtu_controller/wall_position", 1, wallPositionCallback );

  // Visual Tracker subscriber
  visualOdometrySub = nh.subscribe<geometry_msgs::Point>("/distance_error", 1, visualOdometryCallback);

  // Positioning subscriber
  positioningSub = nh.subscribe<std_msgs::UInt8>("/dtu_controller/positioning", 1, positioningCallback);

  ros::Duration(2).sleep();

  // Spin through the callback queue
  for( int i = 0; i < 10; i++ )
  {
    // ROS_INFO("SPIN");
    ros::spinOnce();
  }

  // if( positioning == GPS ) ROS_INFO("Using GPS positioning for start!");
  // else if( positioning == GUIDANCE ) ROS_INFO("Using GUIDANCE for start!");
  // else if( positioning == WALL_POSITION ) ROS_INFO("Using Wall positioning for start!");

  // ROS_INFO("Position = %d (%d)", positioning, (WALL))

  if( positioning == GUIDANCE )
  {
    ROS_INFO("Using GUIDANCE for start!");
    currentPose.linear.x = guidanceLocalPose.linear.x;
    currentPose.linear.y = guidanceLocalPose.linear.y;
    currentPose.linear.z = ultraHeight;
    currentPose.angular.z = attitude.z;

  }
  else if( positioning == GPS )
  {
    ROS_INFO("Using GPS positioning for start!");
    if( gpsHealth > 3 ) {
      currentPose.linear.x = localPosition.x;
      currentPose.linear.y = localPosition.y;
    }
    currentPose.linear.z = ultraHeight;
    currentPose.angular.z = attitude.z;
    
  }
  else if ( positioning == WALL_POSITION || positioning == WALL_WITH_GPS_Y )
  {
    ROS_INFO("Using Wall positioning for start!");
    ROS_INFO("%f",wallPosition.linear.x);
    currentPose.linear.x = wallPosition.linear.x;
    currentPose.linear.y = wallPosition.linear.y;
    currentPose.linear.z = ultraHeight;
    currentPose.angular.z = wallPosition.angular.z;
  }
  else
  {
    ROS_INFO("Observer running");
    ROS_INFO("%f",wallPosition.linear.x);
    currentPose.linear.x = wallPosition.linear.x;
    currentPose.linear.y = 0;
    currentPose.linear.z = 0;
    currentPose.angular.z = 0;
  }

  visualOdometryPose.x = 0;
  visualOdometryPose.y = 0;
  visualOdometryPose.z = 0; 

  // Start the control loop timer

  currentPosePub.publish(currentPose); 

  ROS_INFO("loopCallback starting");

  ros::Timer loopTimer = nh.createTimer(ros::Duration(1.0/loopFrequency), observerLoopCallback);

  ROS_INFO("Observer start spinning");

  ros::spin();

  return 0;
}

void positioningCallback( const std_msgs::UInt8 msg )
{
  positioning = msg.data;
  resetPositioning = true;
}

void readParameters( ros::NodeHandle nh )
{
  // Read parameter file
  if( !nh.getParam("/dtu_controller/position_observer/loop_hz", loopFrequency) ) loopFrequency = 60.0;
  
  ROS_INFO("Observer frequency: %f", loopFrequency);

}

void visualOdometryCallback( const geometry_msgs::Point msg )
{
  // ROS_INFO("go");
  visualOdometryPose = msg;
}

void imuCallback( const sensor_msgs::Imu raw_imu )
{
  gyroZ = raw_imu.angular_velocity.z;
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

void wallPositionCallback( const std_msgs::Float32MultiArray internalWallPosition )
{

  if( internalWallPosition.data[3] > 0.1 )
  {
    // ROS_INFO("WALL CALLBACK");

    float y = 0.001 * internalWallPosition.data[0];
    float x = -0.001 * internalWallPosition.data[1];
    float ang = tf2Radians(internalWallPosition.data[2]);
    //wallPosition.linear.x = -sqrt( x*x + y*y ) * cos(attitude.y);
    wallPosition.linear.x = internalWallPosition.data[4];
    wallPosition.linear.y = 0;
    wallPosition.linear.z = 0;

    wallPosition.angular.x = 0;
    wallPosition.angular.y = 0;
    wallPosition.angular.z = wallPosition.angular.z*0.5 + ang*0.5;

    if( positioning == WALL_POSITION )
    {

      float rot = wallPosition.angular.z - guidanceLocalPose.angular.z;

      // ROS_INFO("wall = %f | global = %f | rot = %f", wallPosition.angular.z*180/3.14159, guidanceLocalPose.angular.z*180/3.14159, rot * 180/3.14159 );

      wallPosition.linear.y = guidanceLocalPose.linear.x * sin( rot ) 
                                + guidanceLocalPose.linear.y * cos( rot );
        
    }
    else if( positioning == WALL_WITH_GPS_Y)
    {

      float rot = wallPosition.angular.z - attitude.z;

      // ROS_INFO("wall = %f | global = %f | rot = %f", wallPosition.angular.z*180/3.14159, guidanceLocalPose.angular.z*180/3.14159, rot * 180/3.14159 );

      wallPosition.linear.y = localPosition.x * sin( rot ) 
                                + localPosition.y * cos( rot );
        
    }
                        
  }
}

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
    ultraHeight = scan.ranges[0];
  }
}

void guidanceMotionCallback( const guidance::Motion motion )
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

  motionVelocity.setX(motion.velocity_in_global_x);
  motionVelocity.setY(motion.velocity_in_global_y);
  motionVelocity.setZ(-motion.velocity_in_global_z);
  // tf::Matrix3x3 R_VEL(quat);
  motionVelocity = R_G2L.inverse() * motionVelocity;

  // ROS_INFO("vx: %.2f  vy: %.2f  vz: %.2f", motionVelocity.getX(), motionVelocity.getY(), motionVelocity.getZ());

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
    guidanceLocalPose.linear.x = poss.getX();
    guidanceLocalPose.linear.y = -poss.getY();
    guidanceLocalPose.linear.z = poss.getZ();

    guidanceLocalPose.angular.x = rawAttitude.x;
    guidanceLocalPose.angular.y = rawAttitude.y;
    guidanceLocalPose.angular.z = guidanceOffsetPose.angular.z-rawAttitude.z;

    if( guidanceLocalPose.angular.z < -M_PI) guidanceLocalPose.angular.z += 2*M_PI;
    else if( guidanceLocalPose.angular.z > M_PI) guidanceLocalPose.angular.z -= 2*M_PI;

  }
}

void localPositionCallback( const geometry_msgs::PointStamped localPoint )
{
  
  // localPosition.x = localPoint.point.y;
  // localPosition.y = -localPoint.point.x;

  localPosition.x = localPoint.point.y * cos(yawOffset) - localPoint.point.x * sin(yawOffset);
  localPosition.y = -localPoint.point.y * sin(yawOffset) - localPoint.point.x * cos(yawOffset);

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
  geometry_msgs::Twist truePose;

  if( positioning > NONE && positioning < LAST_VALID )
  {

    if( positioning == GUIDANCE ){
      truePose.linear.x = guidanceLocalPose.linear.x;
      truePose.linear.y = guidanceLocalPose.linear.y;

      truePose.angular.x = guidanceLocalPose.angular.x;
      truePose.angular.y = -guidanceLocalPose.angular.y;
      truePose.angular.z = guidanceLocalPose.angular.z;

    }
    else if( positioning == GPS )
    {
      
      if( gpsHealth > 3 ) {
        truePose.linear.x = localPosition.x;
        truePose.linear.y = localPosition.y;
      }
      else
      {
        truePose.linear.x = 9999;
        truePose.linear.y = 9999;
      }

      truePose.angular.x = attitude.x;
      truePose.angular.y = attitude.y;
      truePose.angular.z = attitude.z;
      
    }
    else if ( positioning == WALL_POSITION || positioning == WALL_WITH_GPS_Y )
    {
      truePose.linear.x = wallPosition.linear.x;
      truePose.linear.y = wallPosition.linear.y;

      truePose.angular.x = attitude.x;
      truePose.angular.y = attitude.y;
      truePose.angular.z = wallPosition.angular.z;
    }
    else if ( positioning == VISUAL_ODOMETRY )
    {
      truePose.linear.x = wallPosition.linear.x;
      truePose.linear.y = visualOdometryPose.y;
      truePose.linear.z = visualOdometryPose.z;

      truePose.angular.x = attitude.x;
      truePose.angular.y = attitude.y;
      truePose.angular.z = wallPosition.angular.z;

      motionVelocity.setX(0); 
      motionVelocity.setY(0); 
      motionVelocity.setZ(0);
    }

    if( simulation && positioning != VISUAL_ODOMETRY ) currentPose.linear.z = actualHeight;
    else if (positioning != VISUAL_ODOMETRY )
    {
      truePose.linear.z = ultraHeight;
    }

    float maxError = 1.2;
    bool valid = true;
    float weightYaw;
    float weight[3];

    if( !resetPositioning )
    {
      if( fabs(truePose.linear.x - currentPose.linear.x) > maxError ) valid = false;
      else if( fabs(truePose.linear.y - currentPose.linear.y) > maxError ) valid = false;
      else if( fabs(truePose.linear.z - currentPose.linear.z) > maxError ) valid = false;
      else if( fabs(truePose.angular.z - currentPose.angular.z) > maxError ) valid = false;

      if( valid ) {
        weight[0] = 0.65;
        weight[1] = 0.65;
        weight[2] = 0.65;
        weightYaw = 0.65;
      }
      else {
        weight[0] = 1;
        weight[1] = 1;
        weight[2] = 1;
        weightYaw = 1;
      }
    }
    else
    {
      weight[0] = 0;
      weight[1] = 0;
      weight[2] = 0;
      weightYaw = 0;
      resetPositioning = false;
    }
    

    if( !simulation ) {
      currentPose.linear.x = (currentPose.linear.x + motionVelocity.getX()*(1.0/loopFrequency) )*weight[0] + truePose.linear.x * (1 - weight[0]);
      currentPose.linear.y = (currentPose.linear.y + motionVelocity.getY()*(1.0/loopFrequency) )*weight[1] + truePose.linear.y * (1 - weight[1]);
      currentPose.linear.z = (currentPose.linear.z + motionVelocity.getZ()*(1.0/loopFrequency) )*weight[2] + truePose.linear.z * (1 - weight[2]);

      currentPose.angular.z = (currentPose.angular.z + gyroZ*(1.0/loopFrequency) )*weightYaw + truePose.angular.z * (1 - weightYaw);
    }
    else{
      currentPose.linear.x = truePose.linear.x;
      currentPose.linear.y = truePose.linear.y;
      currentPose.angular.z = truePose.angular.z;
      if( positioning == VISUAL_ODOMETRY ) currentPose.linear.z = truePose.linear.z;
    }

    currentPose.angular.x = truePose.angular.x;
    currentPose.angular.y = truePose.angular.y;
    // currentPose.angular.z = truePose.angular.z;
    

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(currentPose.linear.x, currentPose.linear.y, currentPose.linear.z) );
    // tf::Quaternion q;
    // q.setRPY( currentPose.angular.x, currentPose.angular.y, currentPose.angular.z );
    // // transform.setRotation(q);
    // transform.setRotation(currentQuaternion);

    // tf::StampedTransform cTF(transform, ros::Time::now(), "world", "drone");

    // br.sendTransform(cTF);

    currentPosePub.publish(currentPose); 
  }
  
}
