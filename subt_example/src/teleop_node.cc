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
#include <std_srvs/SetBool.h>

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

  /// \brief Subscriber to get input values from the joy control.
  private: ros::Subscriber joySub;

  /// \brief Map from a button name to an index.
  /// e.g., 'A' -> 1
  /// this mapping should be stored in a yaml file.
  private: std::map<std::string, int> joyButtonIndexMap;

  /// \brief Map from an button name to a robot name.
  private: std::map<std::string, std::string> joyButtonRobotMap;

  /// \brief Map from a robot name to a ROS publisher object.
  private: std::map<std::string, ros::Publisher> velPubMap;

  /// \brief the name of the robot currently under control.
  private: std::string currentRobot;

  /// \brief List of light control service names (suffix).
  private: std::vector<std::string> lightSrvSuffixList;

  /// \brief Map from a service name to a service client object.
  private: std::map<std::string, ros::ServiceClient> lightSrvMap;
};

/////////////////////////////////////////////////
SubtTeleop::SubtTeleop():
  linear(1), angular(0), linearScale(0), linearScaleTurbo(0), angularScale(0),
  angularScaleTurbo(0), vertical(4), horizontal(3), verticalScale(0),
  verticalScaleTurbo(0), horizontalScale(0), horizontalScaleTurbo(0),
  enableButton(4), enableTurboButton(5), lightOnTrigger(2), lightOffTrigger(5)
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

  this->nh.getParam("button_map", this->joyButtonIndexMap);

  // Load robot config information. Setting values must be loaded by rosparam.
  std::vector<std::string> robotNames;
  this->nh.getParam("robot_names", robotNames);
  this->nh.getParam("button_robot_map", this->joyButtonRobotMap);
  this->nh.getParam("flashlight_service_suffixes", this->lightSrvSuffixList);

  for (auto robotName: robotNames)
  {
    // Create a publisher object to generate a velocity command, and associate
    // it to the corresponding robot's name.
    this->velPubMap[robotName]
      = this->nh.advertise<geometry_msgs::Twist>(
        robotName + "/cmd_vel", 1, true);

    // Create service clients to control flashlights/LEDs, and associate them
    // to the corresponding service names.
    for (auto suffix: this->lightSrvSuffixList)
    {
      // Note: a service name is formatted like, "/<robot name><suffix>"
      std::string serviceName = "/" + robotName + suffix;
      this->lightSrvMap[serviceName]
        = this->nh.serviceClient<std_srvs::SetBool>(serviceName);
    }
  }

  // Select the first robot in default
  this->currentRobot = this->joyButtonRobotMap.begin()->second;

  // Subscribe "joy" topic to listen to the joy control.
  this->joySub
    = this->nh.subscribe<sensor_msgs::Joy>(
      "joy", 1, &SubtTeleop::joyCallback, this);
}

/////////////////////////////////////////////////
void SubtTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_srvs::SetBool srv;
  // If LT was triggered, turn the lights on.
  if (joy->axes[this->lightOnTrigger] < 0)
  {
    srv.request.data = true;
    for (auto suffix: this->lightSrvSuffixList)
    {
      std::string serviceName = "/" + this->currentRobot + suffix;
      this->lightSrvMap[serviceName].call(srv);
    }
    return;
  }
  // If RT was triggered, turn the lights off.
  if (joy->axes[this->lightOffTrigger] < 0)
  {
    srv.request.data = false;
    for (auto suffix: this->lightSrvSuffixList)
    {
      std::string serviceName = "/" + this->currentRobot + suffix;
      this->lightSrvMap[serviceName].call(srv);
    }
    return;
  }

  // If another button was pressed, choose the associated robot.
  for (auto &pair: this->joyButtonRobotMap)
  {
    if (joy->buttons[this->joyButtonIndexMap[pair.first]])
    {
      this->currentRobot = pair.second;
      return;
    }
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
