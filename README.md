# M100_DTU_ROS

This repository contains the code for DJI M100 missions and visual control.

## Package Overview

### dtu_controller
Contains the controller nodes and mission nodes for the position controlling including launch files for different missions.

### visual_tracker_w_gui
Has the tracking nodes and gui nodes. Both a tracker by Laura and Tobias as well as a CSRT tracker from OpenCV.

## Contact controller

### Class: `ContactController.cpp / h`
This class handles the contact control. Once initialized starting and stopping the controller will send the pitch and yawrate through and control the drone with roll and z-velocity. The class can also be used to read the rod values as it is subscribed to the rod topic. The controller runs at 50Hz. 
If contact to the wall is lost the controller will pitch forward for 1 second. If contact is not regained within this 1 second the controller will stop sending control commands.

_**TODO:**_ Autonomous engage and disengage wall function.

### Contact controller node: `contactController_w_rod_test_node.cpp`
This node is for testing the *ContactController*. While M100 is in F mode the pilot engages the wall. When the switch on the rod is pressed the *ContactController* will take control authority and stay on the wall. If the pilot wishes to disengage use the controller to pitch away form the wall. This will deactivate the *ContactController* and and the pilot will get control authority back.

**Launch:** `contact_controllerf_w_rod_test.launch`


### Mission: `autonomous_wall_touch.cpp`
The mission for doing an autonomous wall touch mission. Uses the position controller, laserscanner, and visual servoing as well as the gui. The mission works as follows:
1. Initialized everything including *ContactController*
2. Wait for the user to start the autonomous mode from the GUI
3. Once autonomous mode is started, start the position controller and hold the current position for 3 seconds before centering position around the target while holding the same distance.
4. Start the approach and move closer until 1 meter from the wall. 
5. Start the wall engagement by ramping closer very slowly. When contact is made stop the position controller and start the *ContactController* instead.
6. Stay on the wall for 10 seconds or until wall control fails.
7. Fly 2 meters away from the wall before releasing control authority back to the pilot and ends the mission.

**Launch:** `autonomous_wall_touch.launch`
**Launch:** `gui.launch` (from `visual_tracker_w_gui`)
