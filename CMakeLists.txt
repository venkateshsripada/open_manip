################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_teleop)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    open_manipulator_msgs
    message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  IntArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs open_manipulator_msgs message_runtime
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_manipulator_teleop_keyboard src/open_manipulator_teleop_keyboard.cpp)
add_dependencies(open_manipulator_teleop_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_keyboard ${catkin_LIBRARIES} )

add_executable(open_manipulator_teleop_keyboard_few src/open_manipulator_teleop_keyboard_few.cpp)
add_dependencies(open_manipulator_teleop_keyboard_few ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_keyboard_few ${catkin_LIBRARIES} )

add_executable(kinect_sub_test src/kinect_sub_test.cpp)
add_dependencies(kinect_sub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect_sub_test ${catkin_LIBRARIES} )

add_executable(open_manipulator_teleop_keyboard_old src/open_manipulator_teleop_keyboard_old.cpp)
add_dependencies(open_manipulator_teleop_keyboard_old ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_keyboard_old ${catkin_LIBRARIES} )

add_executable(open_manipulator_teleop_joystick src/open_manipulator_teleop_joystick.cpp)
add_dependencies(open_manipulator_teleop_joystick ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_joystick ${catkin_LIBRARIES} )

add_executable(open_manipulator_teleop_joystick_few src/open_manipulator_teleop_joystick_few.cpp)
add_dependencies(open_manipulator_teleop_joystick_few ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_manipulator_teleop_joystick_few ${catkin_LIBRARIES} )

################################################################################
# Install
################################################################################
install(TARGETS open_manipulator_teleop_keyboard open_manipulator_teleop_joystick
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
