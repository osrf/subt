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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <subt_gazebo/CommsClient.hh>

/// \brief. Example control class, running as a ROS node to control a robot.
class Controller
{
  /// \brief Constructor.
  /// The controller uses the given name as a prefix of its topics, e.g.,
  /// "/Jackal/cmd_vel" if _name is specified as "Jackal".
  /// \param[in] _name Name of the robot.
  /// \param[in] _address The address for the network
  public: Controller(const std::string &_name, const std::string &_address);

  /// \brief Callback function for selection command.
  /// \param[in] _select True if the robot is selected. False if unselected.
  public: void TeleopSelectCallback(const std_msgs::Bool::ConstPtr &_select);

  /// \brief Callback function for velocity command.
  /// \param[in] _vel Velocity to apply the robot.
  public: void TeleopVelCallback(const geometry_msgs::Twist::ConstPtr &_vel);

  /// \brief Callback function for light command.
  /// \param[in] _switch True if the flashlights are to be on. False if to be
  ///                    off.
  public: void TeleopLightCallback(const std_msgs::Bool::ConstPtr &_switch);

  /// \brief Callback function for communication command.
  /// \param[in] _dest The destination address to which the robot sends its
  ///                  packet through the network.
  public: void TeleopCommCallback(const std_msgs::String::ConstPtr &_dest);

  /// \brief Callback function for message from other comm clients.
  /// \param[in] _srcAddress The address of the robot who sent the packet.
  /// \param[in] _dstAddress The address of the robot who received the packet.
  /// \param[in] _dstPort The destination port.
  /// \param[in] _data The contents the packet holds.
  public: void CommClientCallback(const std::string &_srcAddress,
                                  const std::string &_dstAddress,
                                  const uint32_t _dstPort,
                                  const std::string &_data);

  /// \brief Helper function to flash the communication indicator.
  private: void FlashCommIndicator();

  /// \brief Name of the robot.
  private: std::string name;

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief subscriber for selection command from teleop.
  private: ros::Subscriber teleopSelectSub;

  /// \brief subscriber for velocity command from teleop.
  private: ros::Subscriber teleopVelSub;

  /// \brief subscriber for light command from teleop.
  private: ros::Subscriber teleopLightSub;

  /// \brief subscriber for communication command from teleop.
  private: ros::Subscriber teleopCommSub;

  /// \brief publisher to send cmd_vel
  private: ros::Publisher velPub;

  /// \brief List of service clients to control flashlight(s).
  private: std::vector<ros::ServiceClient> flashlightSrvList;

  /// \brief List of service clients to control selection LED(s).
  private: std::vector<ros::ServiceClient> selectLedSrvList;

  /// \brief List of service clients to control communciation LED(s).
  private: std::vector<ros::ServiceClient> commLedSrvList;

  /// \brief Communication client.
  private: subt::CommsClient client;
};

/////////////////////////////////////////////////
Controller::Controller(const std::string &_name, const std::string &_address):
  name(_name), client(_address)
{
  this->teleopSelectSub
    = this->n.subscribe<std_msgs::Bool>(
      _name + "/select", 1, &Controller::TeleopSelectCallback, this);
  this->teleopVelSub
    = this->n.subscribe<geometry_msgs::Twist>(
      _name + "/cmd_vel_relay", 1, &Controller::TeleopVelCallback, this);
  this->teleopLightSub
    = this->n.subscribe<std_msgs::Bool>(
      _name + "/light", 1, &Controller::TeleopLightCallback, this);
  this->teleopCommSub
    = this->n.subscribe<std_msgs::String>(
      _name + "/comm", 1, &Controller::TeleopCommCallback, this);

  this->client.Bind(&Controller::CommClientCallback, this);

  this->velPub
    = this->n.advertise<geometry_msgs::Twist>(_name + "/cmd_vel", 1);

  std::vector<std::string> flashlightSrvSuffixList;
  this->n.getParam(
    "flashlight_service_suffixes", flashlightSrvSuffixList);
  std::vector<std::string> selectLedSrvSuffixList;
  this->n.getParam(
    "select_led_service_suffixes", selectLedSrvSuffixList);
  std::vector<std::string> commLedSrvSuffixList;
  this->n.getParam(
    "comm_led_service_suffixes", commLedSrvSuffixList);

  // Create service clients to control flashlights, and associate them
  // to the corresponding service names.
  for (auto suffix : flashlightSrvSuffixList)
  {
    // Note: a service name is formatted like, "/<robot name><suffix>"
    std::string serviceName = "/" + this->name + suffix;
    this->flashlightSrvList.push_back(
      this->n.serviceClient<std_srvs::SetBool>(serviceName));
  }

  // Create service clients to control flashlights, and associate them
  // to the corresponding service names.
  for (auto suffix : selectLedSrvSuffixList)
  {
    // Note: a service name is formatted like, "/<robot name><suffix>"
    std::string serviceName = "/" + this->name + suffix;
    this->selectLedSrvList.push_back(
      this->n.serviceClient<std_srvs::SetBool>(serviceName));
  }

  // Create service clients to control flashlights, and associate them
  // to the corresponding service names.
  for (auto suffix : commLedSrvSuffixList)
  {
    // Note: a service name is formatted like, "/<robot name><suffix>"
    std::string serviceName = "/" + this->name + suffix;
    this->commLedSrvList.push_back(
      this->n.serviceClient<std_srvs::SetBool>(serviceName));
  }
}

/////////////////////////////////////////////////
void Controller::TeleopSelectCallback(const std_msgs::Bool::ConstPtr &_select)
{
  ROS_INFO("TeleopSelectCallback");

  std_srvs::SetBool srv;
  srv.request.data = _select->data;
  for (auto service : this->selectLedSrvList)
  {
    service.call(srv);
  }
}

/////////////////////////////////////////////////
void Controller::TeleopVelCallback(const geometry_msgs::Twist::ConstPtr &_vel)
{
  ROS_INFO("TeleopVelCallback");

  this->velPub.publish(*_vel);
}

/////////////////////////////////////////////////
void Controller::TeleopLightCallback(const std_msgs::Bool::ConstPtr &_switch)
{
  ROS_INFO_STREAM("TeleopLightCallback" << this->flashlightSrvList.size());

  std_srvs::SetBool srv;
  srv.request.data = _switch->data;
  for (auto service : this->flashlightSrvList)
  {
    service.call(srv);
  }
}

/////////////////////////////////////////////////
void Controller::TeleopCommCallback(const std_msgs::String::ConstPtr &_dest)
{
  ROS_INFO("TeleopCommCallback");
  this->client.SendTo("_data_", _dest->data);
  this->FlashCommIndicator();
}

/////////////////////////////////////////////////
void Controller::CommClientCallback(const std::string &/*_srcAddress*/,
                                const std::string &/*_dstAddress*/,
                                const uint32_t /*_dstPort*/,
                                const std::string &/*_data*/)
{
  ROS_INFO("CommClientCallback");
  this->FlashCommIndicator();
}

/////////////////////////////////////////////////
void Controller::FlashCommIndicator()
{
  std_srvs::SetBool srv;
  srv.request.data = true;
  for (auto service : this->commLedSrvList)
  {
    service.call(srv);
  }

  ros::Duration(0.1).sleep();

  srv.request.data = false;
  for (auto service : this->commLedSrvList)
  {
    service.call(srv);
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    ROS_ERROR_STREAM("Needs an argument for the competitor's name.");
    return -1;
  }

  ros::init(argc, argv, argv[1]);

  ros::NodeHandle n;
  std::map<std::string, std::string> robotAddressMap;
  n.getParam("robot_address_map", robotAddressMap);

  // Instantiate a communication handler for sending and receiving data.
  Controller controller(argv[1], robotAddressMap[argv[1]]);

  ROS_INFO("Starting competitor\n");

  ros::spin();
}
