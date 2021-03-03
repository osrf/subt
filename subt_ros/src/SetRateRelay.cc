/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <ros/ros.h>

#include <ignition/msgs/double.pb.h>

#include <subt_msgs/SetRate.h>

#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

/// \brief This is a Ign-ROS relay for set_rate service for cameras
/// (added in https://github.com/ignitionrobotics/ign-sensors/pull/95).
///
/// ROS parameters:
/// - ~service string Full name of the Ignition set_rate service.
/// - ~timeout uint Timeout for calling the service (in ms). Default is 1000.
///
/// Advertised ROS services:
/// - ~${service} (subt_msgs/SetRate) Set rate of the connected camera to the given
///                                   value. The ROS service is named the same as
///                                   the Ignition service it is relayed to.
class SetRateRelay
{
  /// \brief Constructor
  public: SetRateRelay();

  /// \brief Destructor
  public: ~SetRateRelay() = default;

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing the desired publishing rate for
  /// a camera.
  /// \param[out] _res The response message.
  public: bool OnSetRateCall(subt_msgs::SetRate::Request &_req,
                             subt_msgs::SetRate::Response &_res);

  /// \brief Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief The ROS node handler used for private communications.
  public: ros::NodeHandle pnh;

  /// \brief ROS service to receive a call to set camera rate.
  public: ros::ServiceServer setRateService;

  /// \brief Name of the set_rate service in both Ignition Transport and ROS.
  public: std::string serviceName;
};

//////////////////////////////////////////////////
SetRateRelay::SetRateRelay() : pnh("~")
{
  if (!this->pnh.getParam("service", this->serviceName)) {
    ROS_ERROR("Cannot operate without parameter '~service' set to the path to the Ignition "
              "Transport service set_rate for the camera.");
    ros::shutdown();
    return;
  }
  std::string topicStats;
  ignition::common::env("IGN_TRANSPORT_TOPIC_STATISTICS", topicStats, true);

  std::vector<std::string> services;
  bool firstLoop {true};
  bool waitedAtLeastOnce = false;
  while (ros::ok() && std::find(services.begin(), services.end(), this->serviceName) == services.end())
  {
    if (firstLoop)
    {
      firstLoop = false;
    }
    else
    {
      std::string topicStatsWarning;
      if (topicStats == "1" && waitedAtLeastOnce)
      {
        topicStatsWarning = " IGN_TRANSPORT_TOPIC_STATISTICS is set to 1. Make sure all parts of the simulator "
                            "run with this setting, otherwise the parts with different values of this variable "
                            "would not be able to communicate with each other.";
      }

      ROS_WARN_STREAM_DELAYED_THROTTLE(60.0,
        "Waiting for ignition service " << this->serviceName << " to appear." << topicStatsWarning);

      ros::WallDuration(1, 0).sleep();
      waitedAtLeastOnce = true;
    }
    this->node.ServiceList(services);
  }

  if (!ros::ok())
    return;

  if (waitedAtLeastOnce)
    ROS_INFO_STREAM("Ignition Service " << this->serviceName << " found.");

  this->setRateService = this->pnh.advertiseService(
    this->serviceName, &SetRateRelay::OnSetRateCall, this);

  ROS_INFO_STREAM("Started set_rate relay from Ign service " << this->serviceName
                  << " to ROS service " << this->setRateService.getService());
}

/////////////////////////////////////////////////
bool SetRateRelay::OnSetRateCall(subt_msgs::SetRate::Request &_req,
  subt_msgs::SetRate::Response &_res)
{
  ignition::msgs::Double req;
  req.set_data(_req.rate);

  ROS_DEBUG_STREAM("Relaying " << this->setRateService.getService() << ": " << _req);

  // Pass the request onto ignition transport
  return this->node.Request(this->serviceName, req);
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "subt_set_rate_relay");

  SetRateRelay relay;

  ros::spin();
  return 0;
}
