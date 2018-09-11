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

#include <algorithm>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/CommsBrokerPlugin.hh"
#include "subt_gazebo/protobuf/datagram.pb.h"

using namespace gazebo;
using namespace subt;

GZ_REGISTER_WORLD_PLUGIN(CommsBrokerPlugin)

/////////////////////////////////////////////////
void CommsBrokerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "CommsBrokerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf,   "CommsBrokerPlugin::Load() error: _sdf pointer is NULL");

  this->world = _world;
    this->rndEngine = std::default_random_engine(ignition::math::Rand::Seed());
  this->commsModel.reset(new subt::CommsModel(
    this->broker.Swarm(), this->world, _sdf));
  this->maxDataRatePerCycle = this->commsModel->MaxDataRate() *
      this->world->Physics()->GetMaxStepSize();
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CommsBrokerPlugin::OnUpdate, this));

  gzmsg << "Starting SubT comms broker" << std::endl;
}

/////////////////////////////////////////////////
void CommsBrokerPlugin::OnUpdate()
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    // Update the state of the communication model.
    this->commsModel->Update();

    // Send a message to each swarm member with its updated neighbors list.
    this->broker.NotifyNeighbors();
  }

  // Dispatch all the incoming messages, deciding whether the destination gets
  // the message according to the communication model.
  // Mutex handling is done inside DispatchMessages().
  this->DispatchMessages();
}

//////////////////////////////////////////////////
void CommsBrokerPlugin::DispatchMessages()
{
  // Create a copy of the incoming message queue, then release the mutex, to
  // avoid the potential for a deadlock later if a robot calls SendTo() inside
  // its message callback.
  std::deque<subt::msgs::Datagram> incomingMsgsBuffer;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::swap(incomingMsgsBuffer, this->broker.Messages());
  }

  // Shuffle the messages.
  std::shuffle(incomingMsgsBuffer.begin(), incomingMsgsBuffer.end(),
    this->rndEngine);

  // Get the current list of endpoints and clients bound.
  const EndPoints_M &endpoints = this->broker.EndPoints();

  auto swarm = this->broker.Swarm();

  // Clear the data rate usage for each robot.
  for (const auto &member : (*swarm))
    member.second->dataRateUsage = 0;

  while (!incomingMsgsBuffer.empty())
  {
    // Get the next message to dispatch.
    const subt::msgs::Datagram msg = incomingMsgsBuffer.front();
    incomingMsgsBuffer.pop_front();

    // Sanity check: Make sure that the sender is a member of the swarm.
    if (swarm->find(msg.src_address()) == swarm->end())
    {
      gzerr << "BrokerPlugin::DispatchMessages(): Discarding message. Robot ["
            << msg.src_address() << "] is not registered as a member of the "
            << "swarm" << std::endl;
      continue;
    }

    // Get the list of neighbors of the sender.
    const Neighbors_M &neighbors = (*swarm)[msg.src_address()]->neighbors;

    // Update the data rate usage.
    auto dataSize = (msg.data().size() + this->commsModel->UdpOverhead()) * 8;
    (*swarm)[msg.src_address()]->dataRateUsage += dataSize;
    for (const auto &neighbor : neighbors)
    {
      // We account the overhead caused by the UDP/IP/Ethernet headers + the
      // payload. We convert the total amount of bytes to bits.
      (*swarm)[neighbor.first]->dataRateUsage += dataSize;
    }

    std::string dstEndPoint =
      msg.dst_address() + ":" + std::to_string(msg.dst_port());
    if (endpoints.find(dstEndPoint) != endpoints.end())
    {
      // Shuffle the clients bound to this endpoint.
      std::vector<BrokerClientInfo> clientsV = endpoints.at(dstEndPoint);
      std::shuffle(clientsV.begin(), clientsV.end(), this->rndEngine);

      for (const BrokerClientInfo &client : clientsV)
      {
        // Make sure that we're sending the message to a valid neighbor.
        if (neighbors.find(client.address) == neighbors.end())
          continue;

        // Check if the maximum data rate has been reached in the destination.
        if ((*swarm)[client.address]->dataRateUsage > this->maxDataRatePerCycle)
        {
          // Debug output
          // gzdbg << "Dropping message (max data rate) from "
          //       << msg.src_address() << " to " << client.address
          //       << " (addressed to " << msg.dst_address()
          //       << ")" << std::endl;
          continue;
        }

        // Decide whether this neighbor gets this message, according to the
        // probability of communication between them right now.
        const double &neighborProb = neighbors.at(client.address);
        if (ignition::math::Rand::DblUniform(0.0, 1.0) < neighborProb)
        {
          // Debug output
          std::cout << "Sending message from " << msg.src_address() << " to "
                    << client.address << " (addressed to " << msg.dst_address()
                    << ")" << std::endl;
          if (this->node.Request(client.address, msg))
          {
            std::cerr << "[CommsBrokerPlugin::DispatchMessages()]: Error "
                      << "sending message to [" << client.address << "]" 
                      << std::endl;
          }
        }
        // else
        // {
        //   // Debug output.
        //   gzdbg << "Dropping message from " << msg.src_address() << " to "
        //         << client.address << " (addressed to " << msg.dst_address()
        //         << ")" << std::endl;
        // }
      }
    }
  }
}
