cmake_minimum_required(VERSION 2.8.3)
project(dtu_controller)

# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        dji_sdk
        image_transport # for message_filter
        std_msgs
	guidance
        tf
        )

find_package(DJIOSDK REQUIRED)


# Try to see if user has OpenCV installed
# If there's version 3.3.0+, depth perception samples will be compiled
# If it's not version 3.3.0+, the sample will not be compiled

find_package(OpenCV QUIET)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES dji_sdk_demo
        #  CATKIN_DEPENDS dji_sdk roscpp
        #  DEPENDS dji_sdk
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(dji_sdk_demo
#   src/${PROJECT_NAME}/dji_sdk_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(dji_sdk_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(dji_sdk_demo_node src/dji_sdk_demo_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(dji_sdk_demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(dji_sdk_demo_node
#   ${catkin_LIBRARIES}
# )

add_library( dtu_modules STATIC 
        src/GuidanceOdometry.cpp
        src/ContactController.cpp
        )

#######################

add_executable(demo_joy_rpyrate_zvel
        src/demo_joy_rpyrate_zvel.cpp)

target_link_libraries(demo_joy_rpyrate_zvel
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(demo_joy_rpyrate_zvel dji_sdk_generate_messages_cpp)

#######################

add_executable(joy_roll_thrust_controller_node
        src/joy_roll_thrust_controller_node.cpp)

target_link_libraries(joy_roll_thrust_controller_node
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(joy_roll_thrust_controller_node dji_sdk_generate_messages_cpp)

#######################

add_executable(rpy_position_control_node
        src/rpy_position_control_node.cpp
        src/pid.cpp
        src/PIDController.cpp)

target_link_libraries(rpy_position_control_node
        ${catkin_LIBRARIES}
        )

#######################

add_executable(contact_controller_node
        src/contact_controller_node.cpp)

target_link_libraries(contact_controller_node
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(contact_controller_node dji_sdk_generate_messages_cpp)

#######################

add_executable(contactController_w_rod_test_node
        src/contactController_w_rod_test_node.cpp
        src/general_functions.cpp)

target_link_libraries(contactController_w_rod_test_node
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        dtu_modules
        )
add_dependencies(contactController_w_rod_test_node dji_sdk_generate_messages_cpp)

########################

add_executable(xy_position_control_node
        src/xy_position_control_node.cpp
        src/pid.cpp)

target_link_libraries(xy_position_control_node
        ${catkin_LIBRARIES}
        )

########################

add_executable(robot_localization_rpy_position
        src/robot_localization_rpy_position.cpp
        )

target_link_libraries(robot_localization_rpy_position
        ${catkin_LIBRARIES}
        )

#######################

add_executable(position_observer_node
        src/position_observer_node.cpp)

target_link_libraries(position_observer_node
        dtu_modules
        ${catkin_LIBRARIES}
        )

########################################################################
# Missions #

# Basic Control mission
add_executable(basic_control_mission
        missions/basic_control_mission.cpp
        src/general_functions.cpp)

target_link_libraries(basic_control_mission
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(basic_control_mission dji_sdk_generate_messages_cpp)

# Switch on controller test
add_executable(switch_on_controller_test
        missions/switch_on_controller_test.cpp
        src/general_functions.cpp)

target_link_libraries(switch_on_controller_test
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(switch_on_controller_test dji_sdk_generate_messages_cpp)

# Visual tracking test
add_executable(visual_tracking_test
        missions/visual_tracking_test.cpp
        src/general_functions.cpp)

target_link_libraries(visual_tracking_test
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
#add_dependencies(switch_on_controller_test)

# Autonomous wall touch mission
add_executable(autonomous_wall_touch
        missions/autonomous_wall_touch.cpp
        src/general_functions.cpp)

target_link_libraries(autonomous_wall_touch
        dtu_modules
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )

# Manual bridge mission
add_executable(manual_bridge_flight
        missions/manual_bridge_flight.cpp
        src/general_functions.cpp)

target_link_libraries(manual_bridge_flight
        dtu_modules
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )

# Inspectdrone demo
add_executable(inspectdrone_demo
        missions/inspectdrone_demo.cpp
        src/general_functions.cpp)

target_link_libraries(inspectdrone_demo
        dtu_modules
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )

#############
## Install ##
#############


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
#   src/client.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dji_sdk_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
