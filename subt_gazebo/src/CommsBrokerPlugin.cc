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

#include "subt_gazebo/CommsBrokerPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(CommsBrokerPlugin)

/////////////////////////////////////////////////
void CommsBrokerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "CommsBrokerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf,   "CommsBrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;
  this->commsModel.reset(new subt::CommsModel(
    this->broker.Team(), this->world, _sdf));
  this->maxDataRatePerCycle = this->commsModel->MaxDataRate() *
      this->world->Physics()->GetMaxStepSize();
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CommsBrokerPlugin::OnUpdate, this));

  gzmsg << "Starting SubT comms broker" << std::endl;
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnUpdate()
{
  uint32_t maxDataRate = this->maxDataRatePerCycle;
  auto now = gazebo::physics::get_world()->SimTime();
  auto dt = (now - this->lastROSParameterCheckTime).Double();

  // It's time to query the ROS parameter server. We do it every second.
  if (dt >= 1.0 )
  {
    bool value = false;
    ros::param::get("/subt/comms/simple_mode", value);

    if (!this->commsModel->SimpleMode() && value)
    {
      gzmsg << "Enabling simple mode comms" << std::endl;
      maxDataRate = UINT32_MAX;
    }
    else if (this->commsModel->SimpleMode() && !value)
    {
      gzmsg << "Disabling simple mode comms" << std::endl;
    }

    this->commsModel->SetSimpleMode(value);
    this->lastROSParameterCheckTime = now;
  }

  // We need to lock the broker mutex from the outside because "commsModel"
  // accesses its "team" member variable.
  std::lock_guard<std::mutex> lock(this->broker.Mutex());

  // Update the state of the communication model.
  this->commsModel->Update();

  // Send a message to each team member with its updated neighbors list.
  this->broker.NotifyNeighbors();

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  this->broker.DispatchMessages(maxDataRate, this->commsModel->UdpOverhead());
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::Reset()
{
  this->broker.Reset();
}
