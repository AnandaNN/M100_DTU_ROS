/** @file ContactController.h
 *  @version 1.0
 *  @date October, 2020
 *
 *  @brief
 *  Concact controller class header
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#ifndef CONTACT_CONTROLLER_H_
#define CONTACT_CONTROLLER_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "tf/LinearMath/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/QuaternionStamped.h"

const float deg2rad = M_PI/180.0;
const float rad2deg = 180.0/M_PI;

class ContactController {

  public:

    // Runtime functions
    void init( ros::NodeHandle nh );
    void startController() { _running = true; }
    void stopController() { _running = false; _disengage = false; ROS_INFO("Contact stopped!"); }
    void disengage() { _disengage = true; }

    // Set values functions
    void setTarget( float target ) { _targetPitchDegrees = target; }
    void setControllerGains( float kpPitch, float kdPitch, float kpYaw, float kdYaw );

    // Checking functions
    bool checkStatus() { return _running; }
    bool checkRod() { return _rodValue; }

  private:

    // Control parameters
    float _contactPitchDegrees = -15.0;
    float _kpYaw = 0.3;
    float _kdYaw = 0.2;
    float _kpPitch = 0.04;
    float _kiPitch = 0.02;
    float _realTimeFactor = 0.35;
    float _integral = 0.0;
    float _pitchErrorDegrees = 0.0;
    float _kdPitch = 0;
    float _targetPitchDegrees = 7.0;
    float _disengagePitch = -4.0;

    // Variables
    float _currentPitch = 0;
    float _currentYawRate = 0;
    bool _rodValue = false;

    float _loopFrequency = 50.0;
    float _loopPeriod = 1.0/50.0;

    bool _running = false;
    bool _disengage = false;
    int _contactBuffer = 0;

    geometry_msgs::Vector3 _attitude;
    geometry_msgs::Vector3 _contactAttitude;
    geometry_msgs::Vector3 _lastContactAttitude;
    float _yawRate = 0;
    float _contactYawRate = 0;

    sensor_msgs::Joy _controlValue;
    sensor_msgs::Imu _imuValue;

    // Subscriber
    ros::Subscriber _attitudeQuaternionSub;
    ros::Subscriber _imuSub;
    ros::Subscriber _rodSub;
    ros::Timer _controlTimer;
    
    // Publisher
    ros::Publisher _ctrlAttitudePub;
    ros::Publisher _referencePub;
    ros::Publisher _statePub;
    ros::Publisher _commandPub;

    // Functions
    void controlTimerCallback( const ros::TimerEvent& );
    void attitudeCallback( const geometry_msgs::QuaternionStamped quaternion );
    void rodCallback(const std_msgs::Bool::ConstPtr& msg);
    void imuCallback( const sensor_msgs::Imu imuMsg );
    float getYawRate(const geometry_msgs::Vector3 u, const tf::Vector3 w);

};

#endif
