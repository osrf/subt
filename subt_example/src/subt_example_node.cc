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

#include <chrono>
#include <iostream>
#include <thread>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <subt_gazebo/CommsClient.hh>

using namespace std::chrono_literals;

class Controller
{
    /// \brief Constructor.
    public: Controller(const std::string &_name, const std::string &_address):
      client(_address)
    {
      this->name = _name;
      this->teleopSelectSub
        = this->n.subscribe<std_msgs::Bool>(_name + "/select", 1, &Controller::teleopSelectCallback, this);
      this->teleopVelSub
        = this->n.subscribe<geometry_msgs::Twist>(_name + "/cmd_vel_in", 1, &Controller::teleopVelCallback, this);
      this->teleopLightSub
        = this->n.subscribe<std_msgs::Bool>(_name + "/light", 1, &Controller::teleopLightCallback, this);
      this->teleopCommSub
        = this->n.subscribe<std_msgs::String>(_name + "/comm", 1, &Controller::teleopCommCallback, this);

      this->client.Bind(&Controller::commClientCallback, this);

      this->velPub
        = this->n.advertise<geometry_msgs::Twist>(_name + "/cmd_vel", 1);

      ros::NodeHandle nh("~");
      std::vector<std::string> flashlightSrvSuffixList;
      nh.getParam(
        "light_service_suffixes", flashlightSrvSuffixList);
      std::vector<std::string> selectLedSrvSuffixList;
      nh.getParam(
        "select_led_service_suffixes", selectLedSrvSuffixList);
      std::vector<std::string> commLedSrvSuffixList;
      nh.getParam(
        "comm_led_service_suffixes", commLedSrvSuffixList);

      // Create service clients to control flashlights, and associate them
      // to the corresponding service names.
      for (auto suffix: flashlightSrvSuffixList)
      {
        // Note: a service name is formatted like, "/<robot name><suffix>"
        std::string serviceName = "/" + this->name + suffix;
        this->flashlightSrvList.push_back(
          this->n.serviceClient<std_srvs::SetBool>(serviceName));
      }

      // Create service clients to control flashlights, and associate them
      // to the corresponding service names.
      for (auto suffix: selectLedSrvSuffixList)
      {
        // Note: a service name is formatted like, "/<robot name><suffix>"
        std::string serviceName = "/" + this->name + suffix;
        this->selectLedSrvList.push_back(
          this->n.serviceClient<std_srvs::SetBool>(serviceName));
      }
      // Create service clients to control flashlights, and associate them
      // to the corresponding service names.
      for (auto suffix: commLedSrvSuffixList)
      {
        // Note: a service name is formatted like, "/<robot name><suffix>"
        std::string serviceName = "/" + this->name + suffix;
        this->commLedSrvList.push_back(
          this->n.serviceClient<std_srvs::SetBool>(serviceName));
      }

    }

    /// \brief Callback function for selection command.
    public: void teleopSelectCallback(const std_msgs::Bool::ConstPtr& _select)
    {
      ROS_INFO("teleopSelectCallback");
    }

    /// \brief Callback function for velocity command.
    public: void teleopVelCallback(const geometry_msgs::Twist::ConstPtr& _vel)
    {
      ROS_INFO("teleopVelCallback");
    }

    /// \brief Callback function for light command.
    public: void teleopLightCallback(const std_msgs::Bool::ConstPtr& _vel)
    {
      ROS_INFO("teleopLightCallback");
    }

    /// \brief Callback function for communication command.
    public: void teleopCommCallback(const std_msgs::String::ConstPtr& _dest)
    {
      ROS_INFO("teleopCommCallback");
    }

    /// \brief Callback function for message from other comm clients.
    public: void commClientCallback(const std::string &_srcAddress,
                                    const std::string &_dstAddress,
                                    const uint32_t _dstPort,
                                    const std::string &_data)
    {
      ROS_INFO("commClientCallback");
    }

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
int main(int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_ERROR_STREAM("Needs an argument for the competitor's name and address.");
    return -1;
  }

  ros::init(argc, argv, argv[1]);

  // Instantiate a communication handler for sending and receiving data.
  Controller controller(argv[1], argv[2]);

  ROS_INFO("Starting competitor\n");

  ros::spin();
}
