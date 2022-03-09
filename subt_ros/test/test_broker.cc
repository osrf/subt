/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

/// \brief This is a dummy message broker for tests. It has no message loss.

#include <subt_communication_broker/subt_communication_broker.h>
#include <subt_communication_model/subt_communication_model.h>
#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>
#include <ros/ros.h>

using namespace subt;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;
using namespace subt::communication_broker;

/////////////////////////////////////////////////
void setDummyComms(Broker& broker)
{
  struct radio_configuration radio;
  radio.pathloss_f = [](const double&, radio_state&, radio_state&) {
    return rf_power();
  };

  broker.SetDefaultRadioConfiguration(radio);
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);
  broker.SetPoseUpdateFunction(
    [](const std::string& name)
    {
      return std::make_tuple(true, ignition::math::Pose3d::Zero, 0.0);
    });
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_broker");
  // subscribe to /clock messages; this is needed because we do not have any
  // NodeHandle
  ros::start();

  Broker broker;
  setDummyComms(broker);

  broker.Start();

  ROS_INFO("Broker is running in partition '%s'", broker.IgnPartition().c_str());

  while (!ros::Time::waitForValid(ros::WallDuration(1.0)))
  {
    ros::spinOnce();
    ROS_WARN("Waiting for valid ROS time");
  }

  ros::Rate rate(1.0);

  while (ros::ok())
  {
    broker.DispatchMessages();
    rate.sleep();
    ROS_DEBUG("Broker dispatched");
  }

  ROS_INFO("Broker exiting");
}