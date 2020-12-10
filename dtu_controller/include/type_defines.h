#ifndef TYPE_DEFINES_H
#define TYPE_DEFINES_H

enum CONTROL_STATUS_ENUM {
    STOP_CONTROLLER,
    RESET_CONTROLLERS,
    RUNNING
};

enum GLOBAL_POSITIONING {
    NONE,
    GPS, // Pure GPS
    GUIDANCE, // Pure Guidance Module
    WALL_POSITION, // Laser RANSAC for X + yaw, Guidance for Y+Z
    WALL_WITH_GPS_Y, // Laser RANSAC for X + yaw, GPS for Y+Z
    VISUAL_ODOMETRY, // Laser RANSAC for X + yaw, Visual tracker for Y+Z
    LAST_VALID
};

#endif