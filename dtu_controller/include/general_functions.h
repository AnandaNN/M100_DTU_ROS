#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// DJI Includes
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/SetLocalPosRef.h>

// DTU includes
#include "type_defines.h"

bool set_control_authority(ros::NodeHandle nh, bool cmd);
bool arm_motors(ros::NodeHandle nh, bool cmd);
bool set_local_frame(ros::NodeHandle nh);

class ControllerInterface {
private:
    ros::NodeHandle nh;
    
    ros::Publisher cmdStatusPub;
    ros::Publisher referencePub;
    
    uint8_t controlStatus;
    geometry_msgs::Twist currentReference;

public:
    void init_interface( ros::NodeHandle nh );

    void set_control_status( uint8_t cmd );
    void set_reference( float x, float y, float z, float yaw);
    void land_copter();

};

#endif