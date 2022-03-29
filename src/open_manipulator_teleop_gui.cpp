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

#include "open_manipulator_teleop/open_manipulator_teleop_gui.h"

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
  convert_sub_ = node_handle_.subscribe("/box_coords", 10, &OpenManipulatorTeleop::CoordsCallback, this);
  uv_sub_ = node_handle_.subscribe("/uv_coords", 10, &OpenManipulatorTeleop::uvCallback, this);
  gui_sub_ = node_handle_.subscribe("/button", 10, &OpenManipulatorTeleop::gui_position, this);
  // Subscribe to depth also; another method for depth
}

void OpenManipulatorTeleop::uvCallback(const open_manipulator_teleop::uvcoords::ConstPtr &msg)
{
  std::vector<double> temp_uv_coordinates;
  temp_uv_coordinates.push_back(msg->x);
  temp_uv_coordinates.push_back(msg->y);
  present_uv_coordinates_ = temp_uv_coordinates;

}

// void OpenManipulatorTeleop::gui_position(const std_msgs::String::ConstPtr &msg)
// {
//   // printf("I heard: [%s]", msg->data.c_str());
//   // std::vector<std::string> temp_val;
//   // temp_val.push_back(msg->data.c_str());
//   // gui_val_ = temp_val;
//   // printf("I heard: [%s]", gui_val_[0]);

//   std::string gui_val = msg->data.c_str();
// }


void OpenManipulatorTeleop::gui_position(const darknet_ros::coordinates::ConstPtr &msg)
{
  printf("In gui_position\n");
  std::vector<int> temp_gui;
  temp_gui.push_back(msg->x);
  temp_gui.push_back(msg->header.seq);
  present_gui_ = temp_gui;

}

void OpenManipulatorTeleop::CoordsCallback(const darknet_ros::coordinates::ConstPtr &msg)
{
  std::vector<double> temp_coordinates;
  temp_coordinates.push_back(msg->x);
  temp_coordinates.push_back(msg->y);
  // temp_coordinates.push_back(msg->x - 0.4);
  // temp_coordinates.push_back(msg->y + 0.2);
  // temp_coordinates.push_back(msg->z);
  temp_coordinates.push_back(0.062);
  present_coordinates_ = temp_coordinates;

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


std::vector<int> OpenManipulatorTeleop::getGUIValue()
{
  // for (int i=0; i<gui_val_.size(); i++)
  //   {printf(gui_val_.at(i));}
  // return gui_val_;

  // printf("The GUI VALUE IS-------%d", present_gui_[0]);

  // if (std::to_string(gui_val)  == "l")
  //   {printf("It is left");}
  // else{printf("Nothing");}

  return present_gui_;
}


std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

std::vector<double> OpenManipulatorTeleop::getPresentCoordinates()
{
  printf("Moving to %f, \t%f, \t%f", present_coordinates_[0],present_coordinates_[1],present_coordinates_[2]);
  setTaskSpacePathFromPresentPositionOnly(present_coordinates_, PATH_TIME);
  return present_coordinates_;
}

std::vector<double> OpenManipulatorTeleop::getPresentUVCoordinates()
{
  printf("Moving to %f, \t%f", present_uv_coordinates_[0],present_uv_coordinates_[1]);
  setTaskSpacePathFromPresentPositionOnly(present_coordinates_, PATH_TIME);
  return present_uv_coordinates_;
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

/*Standard poses that has the end effector at 90 degrees wrt the base*/

void OpenManipulatorTeleop::initpose()
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-2.171);
    joint_name.push_back("joint2"); joint_angle.push_back(0.014);
    joint_name.push_back("joint3"); joint_angle.push_back(0.006);
    joint_name.push_back("joint4"); joint_angle.push_back(0.006);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::straightpose()
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(1.5708);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::teleoppose()
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

/*The four custom defined poses that make the manipulator move close to the object*/

void OpenManipulatorTeleop::firstpose()
{
    straightpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-1.005);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.040);
    joint_name.push_back("joint3"); joint_angle.push_back(0.163);
    joint_name.push_back("joint4"); joint_angle.push_back(0.094);
    setJointSpacePath(joint_name, joint_angle, path_time);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

double OpenManipulatorTeleop::decrementfirstpose(double joint1)
{
    // double joint1 = getPresentJointAngle().at(0) + 0.1;
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(joint1);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.040);
    joint_name.push_back("joint3"); joint_angle.push_back(0.163);
    joint_name.push_back("joint4"); joint_angle.push_back(0.094);
    setJointSpacePath(joint_name, joint_angle, path_time);
    joint1 = joint1 + 0.07;
    return joint1;
}

void OpenManipulatorTeleop::secondpose()
{
    straightpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.492);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.017);
    joint_name.push_back("joint3"); joint_angle.push_back(0.170);
    joint_name.push_back("joint4"); joint_angle.push_back(0.100);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

double OpenManipulatorTeleop::decrementsecondpose(double joint1)
{
    // double joint1 = getPresentJointAngle().at(0) - 0.1;
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(joint1);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.017);
    joint_name.push_back("joint3"); joint_angle.push_back(0.170);
    joint_name.push_back("joint4"); joint_angle.push_back(0.100);
    setJointSpacePath(joint_name, joint_angle, path_time);
    joint1 = joint1 - 0.07;
    return joint1;
}

void OpenManipulatorTeleop::thirdpose()
{
    initpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.417);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.199);
    joint_name.push_back("joint3"); joint_angle.push_back(0.736);
    joint_name.push_back("joint4"); joint_angle.push_back(-0.489);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::decrementthirdpose()
{
    double joint1 = getPresentJointAngle().at(0) - 0.1;
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(joint1);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.199);
    joint_name.push_back("joint3"); joint_angle.push_back(0.736);
    joint_name.push_back("joint4"); joint_angle.push_back(-0.489);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::fourthpose()
{
    initpose();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.304);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.078);
    joint_name.push_back("joint3"); joint_angle.push_back(0.841);
    joint_name.push_back("joint4"); joint_angle.push_back(-0.535);
    setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::decrementfourthpose()
{
    double joint1 = getPresentJointAngle().at(0) - 0.1;
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(joint1);
    joint_name.push_back("joint2"); joint_angle.push_back(0.430);
    joint_name.push_back("joint3"); joint_angle.push_back(0.360);
    joint_name.push_back("joint4"); joint_angle.push_back(-0.612);
    setJointSpacePath(joint_name, joint_angle, path_time);
}


/* Methods dependent on the GUI*/

int OpenManipulatorTeleop::getbutton()
// Gets the value of the button pressed on the GUI
{
  return getGUIValue().at(0);
}

int OpenManipulatorTeleop::getvalue()
/* Gets the sequence (how many number of times) the buttons on GUI have been pressed*/
{
  return getGUIValue().at(1);  
}


int OpenManipulatorTeleop::move_gui()
/*The "main" function. Takes input from the GUI and makes the manipulator move accordingly*/
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (getGUIValue().size() < 1)
  {
    printf("Nothing in move_gui\n");
  }

  else
  {
    int sequence = getGUIValue().at(1);    

    if (getGUIValue().at(0) == 1) //&& (new_sequence > sequence))
      {
        printf("\n%d: Left is pressed \n", getGUIValue().at(0));
        printf("input : h \tdecrease(--) joint 1 angle\n");
        std::vector<std::string> joint_name;
        joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
        joint_name.push_back("joint2");
        joint_name.push_back("joint3");
        joint_name.push_back("joint4");
        setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
        
      }       
    else if (getGUIValue().at(0) == 2)
      { printf("\n%d: Up is pressed \n", getGUIValue().at(0));
        printf("input : z \tincrease(++) z axis in task space\n"); 
        goalPose.at(2) = DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
        
      }
    else if (getGUIValue().at(0) == 3)
      {
        printf("\n%d: Right is pressed \n", getGUIValue().at(0));
        printf("input : y \tincrease(++) joint 1 angle\n");
        std::vector<std::string> joint_name;
        joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
        joint_name.push_back("joint2");
        joint_name.push_back("joint3");
        joint_name.push_back("joint4");
        setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);        
      }
    else if (getGUIValue().at(0) == 4)
      {
        printf("\n%d: Down is pressed \n", getGUIValue().at(0));
        printf("input : x \tdecrease(--) z axis in task space\n");
        goalPose.at(2) = -DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      }
    else if (getGUIValue().at(0) == 5)
      {
        printf("\n%d: Forward is pressed \n", getGUIValue().at(0));
        printf("input : s \tdecrease(--) x axis in task space\n");
        goalPose.at(0) = -DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
          
      }
    else if (getGUIValue().at(0) == 6)
      {
        printf("\n%d: Backward is pressed \n", getGUIValue().at(0));
        printf("input : w \tincrease(++) x axis in task space\n");
        goalPose.at(0) = DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      
      }
    else if (getGUIValue().at(0) == 7)
    {
      printf("\n%d: Grasp is pressed \n", getGUIValue().at(0));
      printf("input : f \tclose gripper\n");
      std::vector<double> joint_angle;
      joint_angle.push_back(-0.0);
      setToolControl(joint_angle);
    }
    else if (getGUIValue().at(0) == 8)
    {
      printf("\n%d: Drop is pressed \n", getGUIValue().at(0));
      printf("input : g \topen gripper\n");
      std::vector<double> joint_angle;
      joint_angle.push_back(0.01);
      setToolControl(joint_angle);      
    }
    else if (getGUIValue().at(0) == 9)
    {
      printf("\n%d: Home is pressed \n", getGUIValue().at(0));
      initpose();
    }
    else if (getGUIValue().at(0) == 10)
    {
      printf("\n%d: Rest is pressed \n", getGUIValue().at(0));

      initpose();
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));

      std::vector<std::string> joint_name;
      std::vector<double> joint_angle;
      double path_time = 2.0;
      joint_name.push_back("joint1"); joint_angle.push_back(-1.489);
      joint_name.push_back("joint2"); joint_angle.push_back(-0.083);
      joint_name.push_back("joint3"); joint_angle.push_back(0.623);
      joint_name.push_back("joint4"); joint_angle.push_back(0.635);
      setJointSpacePath(joint_name, joint_angle, path_time);
    }
    else if (getGUIValue().at(0) == 11)
      {
        printf("\n%d: One is pressed \n", getGUIValue().at(0));
        firstpose();
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        double joint1 = -1.005;
        while (ros::ok()){
          // printf("\nDecrementing");
          joint1 = decrementfirstpose(joint1);
          std::this_thread::sleep_for(std::chrono::milliseconds(400));
          
          ros::spinOnce();
          printf("\nSequence is : %d", sequence);
          printf("\nCurr value is : %d", getGUIValue().at(1));
          if (getGUIValue().at(1) > sequence) {
            break;
          }
        }
      }

    else if (getGUIValue().at(0) == 12)
    {
      printf("\n%d: Two is pressed \n", getGUIValue().at(0));
      secondpose();
      std::this_thread::sleep_for(std::chrono::milliseconds(700));

      double joint1 = 0.492;
        while (ros::ok()){
          // printf("\nDecrementing");
          joint1 = decrementsecondpose(joint1);
          std::this_thread::sleep_for(std::chrono::milliseconds(400));
          
          ros::spinOnce();
          printf("\nSequence is : %d", sequence);
          printf("\nCurr value is : %d", getGUIValue().at(1));
          if (getGUIValue().at(1) > sequence) {
            break;
          }
        }
    }
    else if (getGUIValue().at(0) == 13)
    {
      printf("\n%d: Three is pressed \n", getGUIValue().at(0));
      teleoppose();

      // thirdpose();
      // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    else 
    {
      printf("Nothing");
    }
    return sequence;
  }  
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
  printf("Your wish is my command");
  ros::init(argc, argv, "open_manipulator_teleop_gui");
  OpenManipulatorTeleop openManipulatorTeleop;

  /*Get the code to take GUI buttons as input. Breaks after the manipulator takes input from GUI*/
  while (ros::ok()) 
  {
    ros::spinOnce();
    // openManipulatorTeleop.printBlank();
    int val = openManipulatorTeleop.move_gui();
    printf("\nValue is: %d\n ", val);
    if (val >= 1)
      {break;}

  }

  /* Perform an action only a new button is pressed. This is achieved by subscribing to the value of "seq" and 
  checking it in every iteration */
  int old_val = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    int val = openManipulatorTeleop.getvalue();   // Gets the current sequence
    if (val > old_val){
        printf("---------\n-----------------\n-----------\n%d---------------------\n------------------------\n-----------", val);
        openManipulatorTeleop.move_gui();   
        old_val = val;
    }
    // else if((openManipulatorTeleop.getGUIValue().size() >= 1) && (openManipulatorTeleop.getbutton() == 11)){
    //   openManipulatorTeleop.decrementfirstpose();
    // }
    // else if((openManipulatorTeleop.getGUIValue().size() >= 1) && (openManipulatorTeleop.getbutton() == 12)){
    //   openManipulatorTeleop.decrementsecondpose();
    // }
    // else if((openManipulatorTeleop.getGUIValue().size() >= 1) && (openManipulatorTeleop.getbutton() == 13)){
    //   openManipulatorTeleop.decrementthirdpose();
    // }
  }
  return 0;
}
