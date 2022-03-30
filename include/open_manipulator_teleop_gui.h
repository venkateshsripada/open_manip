/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
#define OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_

#include <termios.h>

#include <chrono>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h> 
#include "std_msgs/String.h"
#include "conio.h"

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros/coordinates.h"
#include "open_manipulator_teleop/uvcoords.h"

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#define NUM_OF_JOINT 4
#define DELTA 0.03
#define JOINT_DELTA 0.1
#define PATH_TIME 0.5

class OpenManipulatorTeleop
{
 public:
  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();

  // update
  void printText();
  void setGoal(char ch);
  void firstpose();
  void secondpose();
  void thirdpose();
  void fourthpose();
  void initpose();
  void teleoppose();
  void straightpose();
  double decrementfirstpose(double joint1);
  double decrementsecondpose(double joint1);
  void decrementthirdpose();
  void decrementfourthpose();
  
  int move_gui();
  int getvalue();
  int getbutton();
  // void getGUIValue();
  std::vector<int> getGUIValue();


 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  std::vector<double> present_coordinates_;
  std::vector<double> present_uv_coordinates_;
  std::vector<int> present_gui_;
  std::vector<std::string> gui_val_;
  char gui_val;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber convert_sub_;
  ros::Subscriber uv_sub_;
  ros::Subscriber gui_sub_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void CoordsCallback(const darknet_ros::coordinates::ConstPtr &msg);
  void uvCallback(const open_manipulator_teleop::uvcoords::ConstPtr &msg);
  // void gui_position(const std_msgs::String::ConstPtr &msg);
  void gui_position(const darknet_ros::coordinates::ConstPtr &msg);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;

  void disableWaitingForEnter(void);
  void restoreTerminalSettings(void);
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
  std::vector<double> getPresentCoordinates();
  std::vector<double> getPresentUVCoordinates();
  // std::vector<int> getGUIValue();
  // int getGUIValue();
  // void getGUIValue();
  
};

#endif //OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
