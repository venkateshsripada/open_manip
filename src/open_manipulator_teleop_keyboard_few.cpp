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

#include "open_manipulator_teleop/open_manipulator_teleop_keyboard_few.h"
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

  disableWaitingForEnter();
  ROS_INFO("OpenManipulator teleoperation using keyboard start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  restoreTerminalSettings();
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
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
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

void OpenManipulatorTeleop::moveup()
{
    std::vector<double> goalPose;  goalPose.resize(3, 0.0);
    goalPose.at(2) = VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
}


double OpenManipulatorTeleop::incrementpose(double joint1, double joint2, double joint3, double joint4)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(joint1);
    joint_name.push_back("joint2"); joint_angle.push_back(joint2);
    joint_name.push_back("joint3"); joint_angle.push_back(joint3);
    joint_name.push_back("joint4"); joint_angle.push_back(joint4);
    setJointSpacePath(joint_name, joint_angle, path_time);
    return joint1 - 0.07;
}


void OpenManipulatorTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  
  printf("\n");
  printf("UP arrow: increase z axis in task space\n");
  printf("DOWN arrow: decrease z axis in task space\n");
  printf("LEFT arrow : Rotate the arm clockwise\n");
  printf("RIGHT arrow : Rotate the arm counter clockwise\n");
 
  printf("\n");
  printf("z : gripper open\n");
  printf("x : gripper close\n");
  printf("       \n");
  printf("1 : initial pose\n");
  printf("       \n");
  printf("r to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");
  if (getPresentKinematicsPose().at(0) < 0.256 && getPresentKinematicsPose().at(0) > 0.179 && getPresentKinematicsPose().at(1) < -0.034 && getPresentKinematicsPose().at(1) > -0.098 && getPresentKinematicsPose().at(2) < 0.100)
    { 
      printf("\n\nClose to object 1; \tPress 'P' to grab object\n");
      if (char inp = std::getchar() == 'p')
      {grasp();}
    }
  if (getPresentKinematicsPose().at(0) < 0.345 && getPresentKinematicsPose().at(0) > 0.282 && getPresentKinematicsPose().at(1) < 0.048 && getPresentKinematicsPose().at(1) > -0.008 && getPresentKinematicsPose().at(2) < 0.100)
    { 
      printf("\n\nClose to object 2: \tPress 'P' to grab object\n");
      if (char inp = std::getchar() == 'p')
        {grasp();}
    }
  if (getPresentKinematicsPose().at(0) < 0.208 && getPresentKinematicsPose().at(0) > 0.122 && getPresentKinematicsPose().at(1) < 0.159 && getPresentKinematicsPose().at(1) > 0.091 && getPresentKinematicsPose().at(2) < 0.100)
    { 
      printf("\n\nClose to object 3: \tPress 'P' to grab object\n");
      if (char inp = std::getchar() == 'p')
        {grasp();}
    }
}

void OpenManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'a')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'd')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'A')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'B')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'D')   // CCW
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'C')   // CW
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'z' || ch == 'Z')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if (ch == 'x' || ch == 'X')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
    
    // Move up after closing the gripper
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    goalPose.at(2) = VERTICAL_DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(1.5708);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.252);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }

  else if (ch == 'r' || ch == 'R')
  {
    printf("input : r \n \npress 'q' to confirm exit\n");

    // First move the manipulator up so that it does not hit the platform
    initpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(1.635);
    joint_name.push_back("joint2"); joint_angle.push_back(0.298);
    joint_name.push_back("joint3"); joint_angle.push_back(0.170);
    joint_name.push_back("joint4"); joint_angle.push_back(0.726);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
    /*
  -------can use this for slow trajectory----------

  else if (ch == '4')
  {

    printf("input : 4 \tconvenient pose-2\n");

    // First move the manipulator up so that it does not hit the platform
    initpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    double joint1 = -2.534;
    double joint2 = 0.201;
    double joint3 = -0.054;
    double joint4 = 0.132;
    for(double i=0; i<=10; i++)
    {
      printf("\n joint1: %f", joint1);
      joint1 = incrementpose(joint1, joint2, joint3, joint4);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    printf("\n");
  }
  */
}

void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_teleop_keyboard");
  OpenManipulatorTeleop openManipulatorTeleop;

  char ch;
  openManipulatorTeleop.printText();
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    printf("ch is: %s", &ch);
    ros::spinOnce();
    openManipulatorTeleop.printText();
    ros::spinOnce();
    openManipulatorTeleop.setGoal(ch);
  }

  return 0;
}
