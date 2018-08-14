/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
 *
*/

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class SubtTeleop
{
  /// \brief Constructor.
  public: SubtTeleop();

  /// \brief Callback function for a joy stick control.
  private: void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  /// \brief ROS node handler.
  private: ros::NodeHandle nh;

  /// \brief Index for the linear axis of the joy stick.
  private: int linear;

  /// \brief Index for the angular axis of the joy stick.
  private: int angular;

  /// \brief Scale value for the linear axis input.
  private: double linearScale;

  /// \brief Scale value for the linear axis input (turbo mode).
  private: double linearScaleTurbo;

  /// \brief Scale value for the angular axis input.
  private: double angularScale;

  /// \brief Scale value for the angular axis input (turbo mode).
  private: double angularScaleTurbo;

  /// \brief Index for the vertical axis (z axis) of the joy stick.
  private: int vertical;

  /// \brief Index for the horizontal axis (y axis) of the joy stick.
  private: int horizontal;

  /// \brief Scale value for the vertical axis (z axis) input.
  private: double verticalScale;

  /// \brief Scale value for the vertical axis (z axis) input (turbo mode).
  private: double verticalScaleTurbo;

  /// \brief Scale value for the horizontal axis (y axis) input.
  private: double horizontalScale;

  /// \brief Scale value for the horizontal axis (y axis) input (turbo mode).
  private: double horizontalScaleTurbo;

  /// \brief Index for the button to enable the joy stick control.
  private: int enableButton;

  /// \brief Index for the button to enable the joy stick control (Turbo mode).
  private: int enableTurboButton;

  /// \brief Index for the trigger to turn lights on.
  private: int lightOnTrigger;

  /// \brief Index for the trigger to turn lights off.
  private: int lightOffTrigger;

  /// \brief Index for horizontal axis of arrow keys.
  private: int axisArrowHorizontal;

  /// \brief Index for vertial axis of arrow keys.
  private: int axisArrowVertical;

  /// \brief Subscriber to get input values from the joy control.
  private: ros::Subscriber joySub;

  /// \brief List of robot names.
  private: std::vector<std::string> robotNames;

  /// \brief Map from a button name to an index.
  /// e.g., 'A' -> 1
  /// this mapping should be stored in a yaml file.
  private: std::map<std::string, int> joyButtonIndexMap;

  /// \brief Map from an button name to a robot name.
  private: std::map<std::string, std::string> joyButtonRobotMap;

  /// \brief Map from a robot name to its address.
  private: std::map<std::string, std::string> robotAddressMap;

  /// \brief Map from a robot name to a ROS publisher to control velocity.
  private: std::map<std::string, ros::Publisher> velPubMap;

  /// \brief Map from a robot name to a ROS publisher to control comm.
  private: std::map<std::string, ros::Publisher> commPubMap;

  /// \brief Map from a robot name to a ROS publisher to control selection LED.
  private: std::map<std::string, ros::Publisher> selPubMap;

  /// \brief Map from a robot name to a ROS publisher to control flashlights.
  private: std::map<std::string, ros::Publisher> lightPubMap;

  /// \brief the name of the robot currently under control.
  private: std::string currentRobot;
};

/////////////////////////////////////////////////
SubtTeleop::SubtTeleop():
  linear(1), angular(0), linearScale(0), linearScaleTurbo(0), angularScale(0),
  angularScaleTurbo(0), vertical(3), horizontal(2), verticalScale(0),
  verticalScaleTurbo(0), horizontalScale(0), horizontalScaleTurbo(0),
  enableButton(4), enableTurboButton(5), lightOnTrigger(6), lightOffTrigger(7),
  axisArrowHorizontal(4), axisArrowVertical(5)
{
  // Load joy control settings. Setting values must be loaded by rosparam.
  this->nh.param("axis_linear", this->linear, this->linear);
  this->nh.param("axis_angular", this->angular, this->angular);
  this->nh.param("scale_linear", this->linearScale, this->linearScale);
  this->nh.param(
    "scale_linear_turbo", this->linearScaleTurbo, this->linearScaleTurbo);
  this->nh.param("scale_angular", this->angularScale, this->angularScale);
  this->nh.param(
    "scale_angular_turbo", this->angularScaleTurbo, this->angularScaleTurbo);

  this->nh.param("axis_vertical", this->vertical, this->vertical);
  this->nh.param("axis_horizontal", this->horizontal, this->horizontal);
  this->nh.param("scale_vertical", this->verticalScale, this->verticalScale);
  this->nh.param(
    "scale_vertical_turbo", this->verticalScaleTurbo, this->verticalScaleTurbo);
  this->nh.param(
    "scale_horizontal", this->horizontalScale, this->horizontalScale);
  this->nh.param(
    "scale_horizontal_turbo",
    this->horizontalScaleTurbo, this->horizontalScaleTurbo);

  this->nh.param("enable_button", this->enableButton, this->enableButton);
  this->nh.param(
    "enable_turbo_button", this->enableTurboButton, this->enableTurboButton);

  this->nh.param(
    "light_on_trigger", this->lightOnTrigger, this->lightOnTrigger);
  this->nh.param(
    "light_off_trigger", this->lightOffTrigger, this->lightOffTrigger);

  this->nh.param(
    "axis_arrow_horizontal",
    this->axisArrowHorizontal, this->axisArrowHorizontal);
  this->nh.param(
    "axis_arrow_vertical", this->axisArrowVertical, this->axisArrowVertical);

  this->nh.getParam("button_map", this->joyButtonIndexMap);

  // Load robot config information. Setting values must be loaded by rosparam.
  this->nh.getParam("robot_names", this->robotNames);
  this->nh.getParam("button_robot_map", this->joyButtonRobotMap);
  this->nh.getParam("robot_address_map", this->robotAddressMap);

  for (auto robotName : this->robotNames)
  {
    // Create a publisher object to generate a velocity command, and associate
    // it to the corresponding robot's name.
    this->velPubMap[robotName]
      = this->nh.advertise<geometry_msgs::Twist>(
        robotName + "/cmd_vel_relay", 1, true);

    // Create a publisher object to generate a selection command, and associate
    // it to the corresponding robot's name.
    this->selPubMap[robotName]
      = this->nh.advertise<std_msgs::Bool>(
        robotName + "/select", 1, true);

    // Create a publisher object to generate a comm command, and associate
    // it to the corresponding robot's name.
    this->commPubMap[robotName]
      = this->nh.advertise<std_msgs::String>(
        robotName + "/comm", 1, true);

    // Create a publisher object to generate a light command, and associate
    // it to the corresponding robot's name.
    this->lightPubMap[robotName]
      = this->nh.advertise<std_msgs::Bool>(
        robotName + "/light", 1, true);
  }

  // Select the first robot in default
  this->currentRobot = this->joyButtonRobotMap.begin()->second;
  std_msgs::Bool msg;
  msg.data = true;
  this->selPubMap[this->currentRobot].publish(msg);

  // Subscribe "joy" topic to listen to the joy control.
  this->joySub
    = this->nh.subscribe<sensor_msgs::Joy>(
      "joy", 1, &SubtTeleop::joyCallback, this);
}

/////////////////////////////////////////////////
void SubtTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Bool msg;
  // If LT was triggered, turn the lights on.
  if (joy->buttons[this->lightOnTrigger])
  {
    msg.data = true;
    this->lightPubMap[this->currentRobot].publish(msg);
    return;
  }
  // If RT was triggered, turn the lights off.
  if (joy->buttons[this->lightOffTrigger])
  {
    msg.data = false;
    this->lightPubMap[this->currentRobot].publish(msg);
    return;
  }

  // If another button was pressed, choose the associated robot.
  for (auto &pair : this->joyButtonRobotMap)
  {
    if (joy->buttons[this->joyButtonIndexMap[pair.first]])
    {
      msg.data = false;
      this->selPubMap[this->currentRobot].publish(msg);
      this->currentRobot = pair.second;
      msg.data = true;
      this->selPubMap[this->currentRobot].publish(msg);
      return;
    }
  }

  // If an arrow key was pressed, send a comm command to the current robot so
  // it sends a message to the one associated with the key.
  if (joy->axes[this->axisArrowVertical] != 0
    || joy->axes[this->axisArrowHorizontal] != 0)
  {
    std_msgs::String addressMsg;
    unsigned int index;
    if (joy->axes[this->axisArrowVertical] == 1)
      index = 3;
    else if (joy->axes[this->axisArrowVertical] == -1)
      index = 0;
    else if (joy->axes[this->axisArrowHorizontal] == 1)
      index = 2;
    else
      index = 1;

    if (index < this->robotNames.size())
    {
      addressMsg.data = this->robotAddressMap[this->robotNames[index]];
      ROS_INFO_STREAM("sending a message to " << addressMsg.data);
      this->commPubMap[this->currentRobot].publish(addressMsg);
    }
    return;
  }

  geometry_msgs::Twist twist;
  // If the trigger values are non zero, calculate control values.
  if (joy->buttons[this->enableButton])
  {
    twist.linear.x
      = this->linearScale * joy->axes[this->linear];
    twist.linear.y
      = this->horizontalScale * joy->axes[this->horizontal];
    twist.linear.z
      = this->verticalScale * joy->axes[this->vertical];
    twist.angular.z
      = this->angularScale * joy->axes[this->angular];

    // Publish the control values.
    this->velPubMap[this->currentRobot].publish(twist);
  }
  else if (joy->buttons[this->enableTurboButton])
  {
    twist.linear.x
      = this->linearScaleTurbo * joy->axes[this->linear];
    twist.linear.y
      = this->horizontalScaleTurbo * joy->axes[this->horizontal];
    twist.linear.z
      = this->verticalScaleTurbo * joy->axes[this->vertical];
    twist.angular.z
      = this->angularScaleTurbo * joy->axes[this->angular];

    // Publish the control values.
    this->velPubMap[this->currentRobot].publish(twist);
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_node");

  SubtTeleop subtTeleop;

  ros::spin();
}
