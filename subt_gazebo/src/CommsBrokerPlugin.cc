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

#include <ros/ros.h>
#include <functional>
#include <mutex>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>
#include <ignition/common/Console.hh>

#include <subt_gazebo/CommsBrokerPlugin.hh>

using namespace gazebo;
using namespace subt;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;
using namespace subt::communication_broker;

GZ_REGISTER_WORLD_PLUGIN(CommsBrokerPlugin)

/////////////////////////////////////////////////
void CommsBrokerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "CommsBrokerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf,   "CommsBrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;

  // Build RF Function
  struct rf_configuration rf_config;
  rf_config.max_range = 10.0;
  auto rf_func = std::bind(&distance_based_received_power,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           std::placeholders::_3,
                           rf_config);

  // Build radio configuration (which includes RF pathloss function)
  struct radio_configuration radio;
  radio.capacity = 54000000;
  radio.default_tx_power = 27;
  radio.modulation = "QPSK";
  radio.noise_floor = -90;
  radio.pathloss_f = rf_func;

  // Instatiate two radios with this configuration
  broker.SetDefaultRadioConfiguration(radio);

  // Set communication function to use
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);

  // Build function to get pose from gazebo
  auto update_pose_func = [_world](const std::string& name) {
    auto const &model = _world->ModelByName(name);

    if (!model)
      return std::make_tuple(false, geometry_msgs::PoseStamped());

    auto gazebo_pose = model->WorldPose();

    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time(_world->SimTime().Double());
    p.header.frame_id = "gazebo";
    p.pose.position.x = gazebo_pose.Pos().X();
    p.pose.position.y = gazebo_pose.Pos().Y();
    p.pose.position.z = gazebo_pose.Pos().Z();
    p.pose.orientation.x = gazebo_pose.Rot().X();
    p.pose.orientation.y = gazebo_pose.Rot().Y();
    p.pose.orientation.z = gazebo_pose.Rot().Z();
    p.pose.orientation.w = gazebo_pose.Rot().W();

    return std::make_tuple(true, std::move(p));
  };
  broker.SetPoseUpdateFunction(update_pose_func);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CommsBrokerPlugin::OnUpdate, this));

  ignmsg << "Starting SubT comms broker" << std::endl;
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnUpdate()
{
  auto now = gazebo::physics::get_world()->SimTime();
  auto dt = (now - this->lastROSParameterCheckTime).Double();

  // It's time to query the ROS parameter server. We do it every second.
  if (dt >= 1.0 )
  {
    bool value = false;
    ros::param::get("/subt/comms/simple_mode", value);

    if (value)
    {
      igndbg << "Enabling simple mode comms" << std::endl;
      
      auto f = [](const radio_configuration& radio,
                  const rf_interface::radio_state& tx_state,
                  const rf_interface::radio_state& rx_state,
                  const uint64_t& num_bytes
                  ) { return true; };
      
      broker.SetCommunicationFunction(f);
    }
    else 
    {
      igndbg << "Disabling simple mode comms" << std::endl;      
      broker.SetCommunicationFunction(&subt::communication_model::attempt_send);
    }

    // this->commsModel->SetSimpleMode(value);
    this->lastROSParameterCheckTime = now;
  }

  // Send a message to each team member with its updated neighbors list.
  this->broker.NotifyNeighbors();

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  // this->broker.DispatchMessages(maxDataRate, this->commsModel->UdpOverhead());
  this->broker.DispatchMessages();
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::Reset()
{
  this->broker.Reset();
}
