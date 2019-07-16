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
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <mutex>

#include <subt_communication_broker/common_types.h>
#include <subt_communication_broker/subt_communication_client.h>
#include <subt_ign/protobuf/artifact.pb.h>

#include <subt_example/CreatePeer.h>

/// \brief. Example control class, running as a ROS node to control a robot.
class Controller
{
  /// \brief Constructor.
  /// The controller uses the given name as a prefix of its topics, e.g.,
  /// "/Jackal/cmd_vel" if _name is specified as "Jackal".
  /// \param[in] _name Name of the robot.
  /// \param[in] _address The address for the network
 public: Controller(const std::string &_name,
                    const std::string &_address);

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
 private: std::unique_ptr<subt::CommsClient> client;

 private: ros::ServiceServer createPeerSrv;

  using CommsClientCallbackType = std::function<void(const std::string &_srcAddress,
                                                     const std::string &_dstAddress,
                                                     const uint32_t _dstPort,
                                                     const std::string &_data)>;
 private:
  // indexed by remote name, store port and Callback function
  std::map<std::string,
           std::pair<ros::Subscriber, ros::Publisher>> peer_connections;
  std::map<std::string,
           std::pair<ros::Time, ros::Publisher>> neighbor_state_pubs;

  void SetCommsActive(double timeout=1.0);
  std::mutex comms_led_mutex;
  ros::Timer active_comms_timer;

  ros::Timer neighbor_timer;
};

/////////////////////////////////////////////////
Controller::Controller(const std::string &_name,
                       const std::string &_address):
    name(_name)
{
  this->client = std::make_unique<subt::CommsClient>(_address, false);

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

  if(!this->client->Bind(&Controller::CommClientCallback, this)) {
    ROS_FATAL(
        "subt_example_node did not successfully bind to the CommsClient");
  }

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

  // Create service client to create peer-to-peer communication
  // channels (mostly for testing)
  ros::NodeHandle pnh("~");

  createPeerSrv =
      pnh.advertiseService<
        subt_example::CreatePeer::Request,
    subt_example::CreatePeer::Response
    >("create_peer",
      [this](auto& req,
             auto& /*res*/)
      {
        std::string remote = req.remote;

        if(this->peer_connections.find(remote) != this->peer_connections.end()) {
          ROS_WARN_STREAM(this->name << " is already connected to " << remote);
          return true;
        }

        ros::NodeHandle pnh("~");
        ros::Subscriber sub =
        pnh.subscribe<std_msgs::String>(remote + "/send", 100,
                                        [this, remote](const std_msgs::StringConstPtr& msg)
  {
    this->client->SendTo(msg->data, remote);
    this->SetCommsActive();
  });

        ros::Publisher pub = pnh.advertise<std_msgs::String>(remote + "/recv", 100);

        this->peer_connections.insert(std::make_pair(remote,
                                                     std::make_pair(sub, pub)));

        ros::Publisher neighbor_pub =
        pnh.advertise<std_msgs::Float64>(remote + "/rssi", 1);
        this->neighbor_state_pubs.insert(
            std::make_pair(remote,
                           std::make_pair(ros::Time(), neighbor_pub)));

        return true;
      });

  auto neighbor_cb = [this](const ros::TimerEvent& ) {
    subt::CommsClient::Neighbor_M neighbors = this->client->Neighbors();

    for(auto kvp : neighbors) {
      std::string address = kvp.first;
      double stamp = kvp.second.first;
      double rssi = kvp.second.second;

      auto neighbor_state_pub = this->neighbor_state_pubs.find(address);
      if(neighbor_state_pub == this->neighbor_state_pubs.end()) {
        // ROS_WARN("Did not run /%s/create_peer service for %s",
        //          this->name.c_str(), address.c_str());
        continue;
      }

      if(stamp > neighbor_state_pub->second.first.toSec()) {
        neighbor_state_pub->second.first.fromSec(stamp);
        std_msgs::Float64 data;
        data.data = rssi;
        neighbor_state_pub->second.second.publish(data);
      }
    }
  };

  double beacon_interval;
  pnh.param("beacon_interval", beacon_interval, 1.0);
  this->client->StartBeaconInterval(ros::Duration(beacon_interval));

  double neighbor_pub_rate;
  pnh.param("neighbor_publish_rate", neighbor_pub_rate, 5.0);
  neighbor_timer = pnh.createTimer(ros::Duration(1.0/neighbor_pub_rate), neighbor_cb);
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
  // ROS_INFO("TeleopVelCallback");

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
  this->client->SendTo("_data_", _dest->data);
  //this->FlashCommIndicator();
  this->SetCommsActive();
}

/////////////////////////////////////////////////
void Controller::CommClientCallback(const std::string &_srcAddress,
                                    const std::string &/*_dstAddress*/,
                                    const uint32_t /*_dstPort*/,
                                    const std::string &_data)
{
  // ROS_INFO("CommClientCallback");
  //this->FlashCommIndicator();
  this->SetCommsActive();

  auto peer = this->peer_connections.find(_srcAddress);
  if(peer != this->peer_connections.end()) {
    std_msgs::String msg;
    msg.data = _data;
    peer->second.second.publish(msg);
  }
}

void Controller::SetCommsActive(double timeout)
{
  std::lock_guard<std::mutex> l(comms_led_mutex);

  ros::NodeHandle nh;

  // Turn comms led on (they will blink automatically)
  {
    std_srvs::SetBool led_srv;
    led_srv.request.data = true;
    for (auto service : this->commLedSrvList)
    {
      service.call(led_srv);
    }
  }

  if(!active_comms_timer) {
    active_comms_timer =
        nh.createTimer(ros::Duration(timeout),
                       [this](const ros::TimerEvent&)
                       {
                         // Turn comms LEDs off
                         std::lock_guard<std::mutex> l(comms_led_mutex);
                         std_srvs::SetBool led_srv;
                         led_srv.request.data = false;
                         for (auto service : this->commLedSrvList)
                         {
                           service.call(led_srv);
                         }
                       }, true, true);
  }
  else {
    active_comms_timer.setPeriod(ros::Duration(timeout), true);
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

  // Instantiate a communication handler for sending and receiving data.
  Controller controller(argv[1], argv[1]);

  ROS_INFO("Starting competitor\n");

  ros::spin();
}
