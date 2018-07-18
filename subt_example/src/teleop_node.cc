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
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <yaml-cpp/yaml.h>

class SubtTeleop
{
public:
  SubtTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh;

  int linear, angular;
  int enableTrigger;
  double linearScale, angularScale;

  int linearTurbo, angularTurbo;
  int enableTurboTrigger;
  double linearScaleTurbo, angularScaleTurbo;

  std::map<std::string, int> joyButtonIndexMap;

  std::map<std::string, ros::Publisher> velPubMap;
  std::map<std::string, std::string> joyButtonRobotMap;
  std::string currentRobot;

  ros::Subscriber joySub;
};

SubtTeleop::SubtTeleop():
  linear(1), angular(0), enableTrigger(9), linearScale(0), angularScale(0),
  linearTurbo(4), angularTurbo(3), enableTurboTrigger(10),
  linearScaleTurbo(0), angularScaleTurbo(0)
{
  this->nh.param("axis_linear", this->linear, this->linear);
  this->nh.param("axis_angular", this->angular, this->angular);
  this->nh.param("enable_trigger", this->enableTrigger, this->enableTrigger);
  this->nh.param("scale_linear", this->linearScale, this->linearScale);
  this->nh.param("scale_angular", this->angularScale, this->angularScale);

  this->nh.param("axis_linear_turbo", this->linearTurbo, this->linearTurbo);
  this->nh.param("axis_angular_turbo", this->angularTurbo, this->angularTurbo);
  this->nh.param("enable_turbo_trigger", this->enableTurboTrigger, -1);
  this->nh.param("scale_linear_turbo", this->linearScaleTurbo, this->linearScaleTurbo);
  this->nh.param("scale_angular_turbo", this->angularScaleTurbo, this->angularScaleTurbo);

  this->nh.getParam("button_map", this->joyButtonIndexMap);

  std::vector<YAML::Node> robotConf = YAML::LoadAllFromFile(ros::package::getPath("subt_example") + "/config/robot_list.yaml");
  for (std::size_t i = 0; i < robotConf.size(); ++i)
  {
    std::string name = robotConf[i]["name"].as<std::string>();
    std::string button = robotConf[i]["joy_select_button"].as<std::string>();
    // mapping from robot's name to a publisher
    this->velPubMap[name] = this->nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1);
    // mapping from button (e.g., 'A', 'B', 'X', 'Y') to robot's name
    this->joyButtonRobotMap[button] = name;
  }
  // select the first robot in default
  this->currentRobot = this->joyButtonRobotMap["A"];

  this->joySub = this->nh.subscribe<sensor_msgs::Joy>("joy", 10, &SubtTeleop::joyCallback, this);
}

void SubtTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  for (auto &pair: joyButtonIndexMap)
  {
    if (joy->buttons[pair.second])
    {
      this->currentRobot = this->joyButtonRobotMap[pair.first];
    }
  }
  geometry_msgs::Twist twist;

  double triggerRate = -0.5 * joy->axes[this->enableTrigger] + 0.5;
  double triggerTurboRate = -0.5 * joy->axes[this->enableTurboTrigger] + 0.5;

  if (triggerRate > 0)
  {
    twist.angular.z
      = this->angularScale * joy->axes[this->angular] * triggerRate;
    twist.linear.x
      = this->linearScale * joy->axes[this->linear] * triggerRate;
  }
  else if (triggerTurboRate > 0)
  {
    twist.angular.z
      = this->angularScaleTurbo
        * joy->axes[this->angularTurbo] * triggerTurboRate;
    twist.linear.x
      = this->linearScaleTurbo
        * joy->axes[this->linearTurbo] * triggerTurboRate;
  }

  this->velPubMap[this->currentRobot].publish(twist);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_node");

  SubtTeleop subtTeleop;

  ros::spin();
}
