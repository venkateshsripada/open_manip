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

#include "open_manipulator_teleop/open_manipulator_teleop_joystick_few.h"
#include <chrono>
#include <thread>

OpenManipulatorTeleop::OpenManipulatorTeleop()
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  ROS_INFO("OpenManipulator teleoperation using joystick start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  joy_command_sub_ = node_handle_.subscribe("joy", 10, &OpenManipulatorTeleop::joyCallback, this);
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}


void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  present_kinematic_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{

  return present_kinematic_position_;
}

void OpenManipulatorTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if (msg->axes.at(7) == 1.0) setGoal("x+");    
  else if (msg->axes.at(7) == -1.0) setGoal("x-");
  else if (msg->axes.at(6) ==  1.0) setGoal("y+");
  else if (msg->axes.at(6) == -1.0) setGoal("y-");

  else if (msg->buttons.at(3) == 1.0) setGoal("z+");
  else if (msg->buttons.at(0) == 1.0) setGoal("z-");
  else if (msg->buttons.at(1) == 1.0) setGoal("clockwise");
  else if (msg->buttons.at(2) == 1.0) setGoal("counter-clockwise");

  else if (msg->axes.at(3) >= 0.1) setGoal("init");
  else if (msg->axes.at(6) == 1.0) setGoal("rest");


  if (msg->buttons.at(4) == 1.0) setGoal("gripper close");
  else if (msg->buttons.at(5) == 1.0) setGoal("gripper open");
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::initpose()
{
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 4.0;
  joint_name.push_back("joint1"); joint_angle.push_back(1.5708);
  joint_name.push_back("joint2"); joint_angle.push_back(0.0);
  joint_name.push_back("joint3"); joint_angle.push_back(0.0);
  joint_name.push_back("joint4"); joint_angle.push_back(0.0);
  setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::rest()
{
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 4.0;
  joint_name.push_back("joint1"); joint_angle.push_back(1.5708);
  joint_name.push_back("joint2"); joint_angle.push_back(0.249);
  joint_name.push_back("joint3"); joint_angle.push_back(0.140);
  joint_name.push_back("joint4"); joint_angle.push_back(0.837);
  setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::grasp()
{
  movedown();
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  gripAndMoveUp();
}

void OpenManipulatorTeleop::movedown()
{
  // double PATH_TIME = 1.0;
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  goalPose.at(2) = -2*VERTICAL_DELTA;
  setTaskSpacePathFromPresentPositionOnly(goalPose, 1.0);
}

void OpenManipulatorTeleop::gripAndMoveUp()
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  // Grasp the object
  std::vector<double> joint_angle;
  joint_angle.push_back(-0.01);
  setToolControl(joint_angle);

  // Move up after closing the gripper
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  goalPose.at(2) = VERTICAL_DELTA;
  setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
}

void OpenManipulatorTeleop::printtext()
{

  printf("-----------------------------------------------------\n");
  printf("-----------------------------------------------------\n");
  printf("Right stick up  : Go to intial position \n");
  printf("\n");
  printf("UP button    : increase x axis in task space \n");
  printf("DOWN button  : decrease x axis in task space \n");
  printf("LEFT button  : increase y axis in task space \n");
  printf("RIGHT button : decrease y axis in task space \n");
  printf("\n");
  printf("YELLOW button  : increase z axis in task space \n");
  printf("GREEN button   : decrease z axis in task space \n");
  printf("RED button     : Move arm clockwise \n");
  printf("BLUE button    : Move arm counter-clockwise \n");
  printf("\n");
  printf("RB button      : Open the gripper\n");
  printf("LB button      : Close the gripper\n");
  printf("\n");
if (present_kinematic_position_.at(0) < 0.256 && present_kinematic_position_.at(0) > 0.179 && present_kinematic_position_.at(1) < -0.034 && present_kinematic_position_.at(1) > -0.098 && present_kinematic_position_.at(2) < 0.100)
  { 
    printf("\n\nClose to object 1; \tPress 'START' to grasp autonomously\n");
  }
if (present_kinematic_position_.at(0) < 0.345 && present_kinematic_position_.at(0) > 0.282 && present_kinematic_position_.at(1) < 0.048 && present_kinematic_position_.at(1) > -0.008 && present_kinematic_position_.at(2) < 0.100)
  { 
    printf("\n\nClose to object 2; \tPress 'START' to grasp autonomously\n");
  }
  
  printf("Present kinematics_pose: %f", present_kinematic_position_.at(0));
  printf("BACK button    : Quit\n");
  
  printf("-----------------------------------------------------\n");
  printf("-----------------------------------------------------\n");
  
}
void OpenManipulatorTeleop::setGoal(const char* str)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (str == "x+")  // Up button
  {
    printtext();
    printf("increase(++) x axis in cartesian space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "x-")   // Down button
  {
    printtext();
    printf("decrease(--) x axis in cartesian space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "y+")   // Left button
  {
    printtext();
    printf("increase(++) y axis in cartesian space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "y-")   // Right button
  {
    printtext();
    printf("decrease(--) y axis in cartesian space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "z+")   // Yellow button
  {
    printtext();
    printf("increase(++) z axis in cartesian space\n");
    goalPose.at(2) = VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "z-")   // Green button
  {
    printtext();
    printf("decrease(--) z axis in cartesian space\n");
    goalPose.at(2) = -VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "gripper open")   // RB button
  {
    printtext();
    printf("open gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
    
  }
  else if (str == "gripper close")    // LB button
  {
    printtext();
    printf("close gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);

    // Move up after closing the gripper
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    goalPose.at(2) = VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (str == "counter-clockwise")   // Blue button
  {
    printtext();
    printf("input : L1 \tcounter-clockwise - joint 1 \n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (str == "clockwise")   // Red button
  {
    printtext();
    printf("input : L1 \tclockwise - joint 1 \n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if (str == "init")   // Right stick up
  {
    printtext();
    printf("Moving to home position\n");
    initpose();
  }

  else if (str == "rest")   // Back button
  {
    printtext();
    printf("Moving to rest position\n");
    initpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    rest();
  }


  else if (str == "grasp")   // START button
  {
    printtext();
    printf("Grasping the object\n");
    grasp();
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_TELEOP");
  OpenManipulatorTeleop openManipulatorTeleop;
  ros::spin();
  return 0;
}
