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

  /// \brief Scale value for the angular axis input.
  private: double angularScale;

  /// \brief Index for the vertical axis (z axis) of the joy stick.
  private: int vertical;

  /// \brief Index for the horizontal axis (y axis) of the joy stick.
  private: int horizontal;

  /// \brief Scale value for the vertical axis (z axis) input.
  private: double verticalScale;

  /// \brief Scale value for the horizontal axis (y axis) input.
  private: double horizontalScale;

  /// \brief Index for the button to enable the joy stick control.
  private: int enableTrigger;

  /// \brief Index for the button to enable the joy stick control (Turbo mode).
  private: int enableTurboTrigger;

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
  linear(1), angular(0), linearScale(0), angularScale(0),
  vertical(4), horizontal(3), verticalScale(0), horizontalScale(0),
  enableTrigger(9), enableTurboTrigger(10)
{
  // Load joy control settings. Setting values must be loaded by rosparam.
  this->nh.param("axis_linear", this->linear, this->linear);
  this->nh.param("axis_angular", this->angular, this->angular);
  this->nh.param("scale_linear", this->linearScale, this->linearScale);
  this->nh.param("scale_angular", this->angularScale, this->angularScale);

  this->nh.param("axis_vertical", this->vertical, this->vertical);
  this->nh.param("axis_horizontal", this->horizontal, this->horizontal);
  this->nh.param("scale_vertical", this->verticalScale, this->verticalScale);
  this->nh.param(
    "scale_horizontal", this->horizontalScale, this->horizontalScale);

  this->nh.param("enable_trigger", this->enableTrigger, this->enableTrigger);
  this->nh.param("enable_turbo_trigger", this->enableTurboTrigger, -1);

  this->nh.getParam("button_map", this->joyButtonIndexMap);

  // Load robot config information. Setting values must be loaded by rosparam.
  std::vector<std::string> robotNames;
  this->nh.getParam("robot_names", robotNames);
  this->nh.getParam("button_robot_map", this->joyButtonRobotMap);
  this->nh.getParam("light_service_suffixes", this->lightSrvSuffixList);

  for (auto robotName: robotNames)
  {
    // Create a publisher object to generate a velocity command, and associate
    // it to the corresponding robot's name.
    this->velPubMap[robotName]
      = this->nh.advertise<geometry_msgs::Twist>(robotName + "/cmd_vel", 1);

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
      "joy", 10, &SubtTeleop::joyCallback, this);
}

/////////////////////////////////////////////////
void SubtTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_srvs::SetBool srv;
  // If L button was pressed, turn the lights on.
  if (joy->buttons[this->joyButtonIndexMap["L"]])
  {
    srv.request.data = true;
    for (auto suffix: this->lightSrvSuffixList)
    {
      std::string serviceName = "/" + this->currentRobot + suffix;
      this->lightSrvMap[serviceName].call(srv);
    }
    return;
  }
  // If R button was pressed, turn the lights off.
  if (joy->buttons[this->joyButtonIndexMap["R"]])
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
    }
  }

  geometry_msgs::Twist twist;
  // Since a trigger value spans from 1 to -1, it is remapped so it does from 0
  // to 1.
  double triggerRate = -0.5 * joy->axes[this->enableTrigger] + 0.5;
  if (triggerRate == 0)
    triggerRate = -0.5 * joy->axes[this->enableTurboTrigger] + 0.5;

  // If the trigger values are non zero, calculate control values.
  if (triggerRate > 0)
  {
    twist.linear.x
      = this->linearScale * joy->axes[this->linear] * triggerRate;
    twist.linear.y
      = this->horizontalScale * joy->axes[this->horizontal] * triggerRate;
    twist.linear.z
      = this->verticalScale * joy->axes[this->vertical] * triggerRate;
    twist.angular.z
      = this->angularScale * joy->axes[this->angular] * triggerRate;
  }

  // Publish the control values.
  this->velPubMap[this->currentRobot].publish(twist);
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_node");

  SubtTeleop subtTeleop;

  ros::spin();
}
