cmake_minimum_required(VERSION 2.8.3)
project(gui_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

## Declare ROS messages and services
add_message_files(
  FILES
  coordinates.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(INCLUDE_DIRS include
CATKIN_DEPENDS roscpp std_msgs open_manipulator_msgs darknet_ros_msgs message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
