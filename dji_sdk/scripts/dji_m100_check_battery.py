#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

# ROS messages.
from sensor_msgs.msg import BatteryState

def batteryCallback(data):
    print('Battery level: {} %'.format(data.percentage))

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('batterySpammer')
    
    battery_sub = rospy.Subscriber('/dji_sdk/battery_state', BatteryState, batteryCallback)

    rospy.spin()