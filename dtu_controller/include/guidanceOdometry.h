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

class guidanceOdometry {

private:
    guidance::Motion internalMotion;
    sensor_msgs::LaserScan internalUltraSonic;
    float currentHeight;

    void ultraSonicCallback( const sensor_msgs::LaserScan _scan );
    void motionCallback( const guidance::Motion _motion );

public:
    int test();

};

#endif