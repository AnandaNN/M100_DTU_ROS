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

class ContactController {

  public:

    // Runtime functions
    void init( ros::NodeHandle nh );
    void startController() { _running = true; }
    void stopController() { _running = false; _disengage = false; }
    void disengage() { _disengage = true; }

    // Set values functions
    void setTarget( float target ) { _targetPitch = target; }
    void setControllerGains( float kpPitch, float kdPitch, float kpYaw, float kdYaw );

    // Checking functions
    bool checkStatus() { return _running; }
    bool checkRod() { return _rodValue; }

  private:

    // Control parameters
    float _kpYaw = 0.3;
    float _kdYaw = 0.2;
    float _kpPitch = 0.04;
    float _kdPitch = 0;
    float _targetPitch = 7.0;
    float _disengagePitch = -5.0;

    // Variables
    float _contactYaw = 0;
    float _lastContactYaw = 0;
    float _currentPitch = 0;
    float _currentYawRate = 0;
    bool _rodValue = false;

    float _loopFrequency = 50.0;

    bool _running = false;
    bool _disengage = false;
    int _contactBuffer = 0;

    tf::Vector3 _angularVelocityWorldFrame;
    tf::Vector3 _contactPointBodyFrame;

    geometry_msgs::Vector3 _attitude;

    sensor_msgs::Joy _controlValue;
    sensor_msgs::Imu _imuValue;

    // Subscriber
    ros::Subscriber _attitudeQuaternionSub;
    ros::Subscriber _imuSub;
    ros::Subscriber _rodSub;
    ros::Timer _controlTimer;
    
    // Publisher
    ros::Publisher _ctrlAttitudePub;


    // Functions
    void controlTimerCallback( const ros::TimerEvent& );
    void attitudeCallback( const geometry_msgs::QuaternionStamped quaternion );
    void rodCallback(const std_msgs::Bool::ConstPtr& msg);
    void imuCallback( const sensor_msgs::Imu imuMsg );

};

#endif
