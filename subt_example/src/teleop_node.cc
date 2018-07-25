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
#include <std_srvs/SetBool.h>

#include <ros/console.h>

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

  std::vector<std::string> lightSrvNameList;
  std::map<std::string, ros::ServiceClient> lightSrvMap;
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
  this->nh.getParam("select_button_map", this->joyButtonRobotMap);
  this->nh.getParam("light_switches", this->lightSrvNameList);

  std::vector<std::string> robotNames;
  this->nh.getParam("robot_names", robotNames);
  for (auto robotName: robotNames)
  {
    // mapping from robot's name to a publisher
    this->velPubMap[robotName]
      = this->nh.advertise<geometry_msgs::Twist>(robotName + "/cmd_vel", 1);

    for (auto suffix: this->lightSrvNameList)
    {
      std::string serviceName = "/" + robotName + suffix;
      this->lightSrvMap[serviceName]
        = this->nh.serviceClient<std_srvs::SetBool>(serviceName);
    }
  }

  // select the first robot in default
  this->currentRobot = this->joyButtonRobotMap.begin()->second;
  ROS_INFO_STREAM("First robot: " << this->currentRobot);

  this->joySub = this->nh.subscribe<sensor_msgs::Joy>("joy", 10, &SubtTeleop::joyCallback, this);
}

void SubtTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  std_srvs::SetBool srv;
  if (joy->buttons[this->joyButtonIndexMap["L"]])
  {
    srv.request.data = true;
    for (auto suffix: this->lightSrvNameList)
    {
      std::string serviceName = "/" + this->currentRobot + suffix;
      this->lightSrvMap[serviceName].call(srv);
    }
    return;
  }
  else if (joy->buttons[this->joyButtonIndexMap["R"]])
  {
    srv.request.data = false;
    for (auto suffix: this->lightSrvNameList)
    {
      std::string serviceName = "/" + this->currentRobot + suffix;
      this->lightSrvMap[serviceName].call(srv);
    }
    return;
  }
  
  for (auto &pair: this->joyButtonIndexMap)
  {
    if (joy->buttons[pair.second])
    {
      std::string name = this->joyButtonRobotMap[pair.first];
      if (name.length() > 0)
      {
        this->currentRobot = name;
      }
      return;
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
