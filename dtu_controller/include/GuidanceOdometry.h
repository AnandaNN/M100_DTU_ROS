/** @file guidanceOdometry.h
 *  @version 1.0
 *  @date August, 2020
 *
 *  @brief
 *  Class for handling Guidance Visiual odometry for missions
 *
 *  @copyright 2020 Ananda Nielsen, DTU. All rights reserved.
 *
 */

#ifndef GUIDANCE_ODOMETRY_H
#define GUIDANCE_ODOMETRY_H

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <guidance/Motion.h>

class GuidanceOdometry {

private:

    // Internal ROS objects
    ros::Subscriber _motionSubscriber;
    ros::Subscriber _ultraSonicSubscriber;

    guidance::Motion _motion;
    sensor_msgs::LaserScan _ultraScan;
    
    // Callback functions
    void ultraSonicCallback( const sensor_msgs::LaserScan scan );
    void motionCallback( const guidance::Motion motion );

    // Internal use function

    // Internal variables
    float _height;
    int _ultraMsgNum = 0;
    int _motionMsgNum = 0;

    tf::Quaternion _offsetQuat;
    tf::Quaternion _offsetQuatRaw;
    tf::Quaternion _currentQuat;
    tf::Vector3 _offsetPosition;
    tf::Vector3 _currentPosition;
    tf::Vector3 _currentLocalPosition;
    tf::Vector3 _currentLocalVelocity;

public:

    // Public functions
    void startOdometry( ros::NodeHandle nh );
    bool resetOdometry();

    tf::Vector3 getPosition() { return _currentLocalPosition; }
    tf::Vector3 getVelocity() { return _currentLocalVelocity; }

};

#endif