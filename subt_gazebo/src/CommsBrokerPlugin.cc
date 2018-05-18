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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/CommsBrokerPlugin.hh"
#include "subt_gazebo/protobuf/datagram.pb.h"

using namespace gazebo;
using namespace subt;

GZ_REGISTER_WORLD_PLUGIN(CommsBrokerPlugin)

/////////////////////////////////////////////////
void CommsBrokerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  GZ_ASSERT(_world, "CommsBrokerPlugin world pointer is NULL");
  this->world = _world;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CommsBrokerPlugin::OnUpdate, this));

  gzmsg << "Starting SubT comms broker" << std::endl;

  // Advertise a oneway service for centralizing all message requests.
  if (!node.Advertise(kBrokerService, &CommsBrokerPlugin::OnMessage, this))
  {
    gzerr << "Error advertising service [" << kBrokerService << "]"
          << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnUpdate()
{
  // ToDo: Step the comms model.

  std::lock_guard<std::mutex> lk(this->mutex);
  this->ProcessIncomingMsgs();
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::ProcessIncomingMsgs()
{
  while (!this->incomingMsgs.empty())
  {
    // Forward the messages.
    auto const &msg = this->incomingMsgs.front();
    auto endPoint = msg.dst_address() + ":" + std::to_string(msg.dst_port());
    this->node.Request(endPoint, msg);
    this->incomingMsgs.pop();
  }
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnMessage(const subt::msgs::Datagram &_req)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->mutex);
  this->incomingMsgs.push(_req);
}
