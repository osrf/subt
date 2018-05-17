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

#include "subt_gazebo/CommsBrokerPlugin.hh"
#include "subt_gazebo/protobuf/datagram.pb.h"

using namespace gazebo;

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
  if (!node.Advertise(this->kBrokerSrv, &CommsBrokerPlugin::OnMessage, this))
  {
    std::cerr << "Error advertising service [" << this->kBrokerSrv << "]"
              << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnUpdate()
{
  // ToDo: Step the comms model.
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnMessage(const subt::msgs::Datagram &_req)
{
  // ToDo: Use the comms model.

  // Forward the message.
  auto endPoint = _req.dst_address() + ":" + std::to_string(_req.dst_port());
  this->node.Request(endPoint, _req);
}
