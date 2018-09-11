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

#include <iostream>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/protobuf/neighbor_m.pb.h"
#include "subt_gazebo/Broker.hh"
#include "subt_gazebo/CommonTypes.hh"

using namespace subt;

//////////////////////////////////////////////////
Broker::Broker()
  : swarm(std::make_shared<SwarmMembership_M>())
{
  // Advertise the service for registering addresses.
  if (!this->node.Advertise(kAddrRegistrationSrv, 
        &Broker::OnAddrRegistration, this))
  {
    std::cerr << "Error advertising srv [" << kAddrRegistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise the service for registering end points.
  if (!this->node.Advertise(kEndPointRegistrationSrv,
        &Broker::OnEndPointRegistration, this))
  {
    std::cerr << "Error advertising srv [" << kEndPointRegistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise a oneway service for centralizing all message requests.
  if (!this->node.Advertise(kBrokerSrv, &Broker::OnMessage, this))
  {
    std::cerr << "Error advertising srv [" << kBrokerSrv << "]" << std::endl;
    return;
  }

  // Advertise a topic for notifying neighbor updates.
  this->neighborPub =
    this->node.Advertise<subt::msgs::Neighbor_M>(kNeighborsTopic);
  if (!this->neighborPub)
  {
    std::cerr << "Error advertising topic [" << kNeighborsTopic << "]"
              << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
bool Broker::Bind(const std::string &_clientAddress,
  const std::string &_endpoint)
{
  // Make sure that the same client didn't bind the same end point before.
  if (this->endpoints.find(_endpoint) != this->endpoints.end())
  {
    const auto &clientsV = this->endpoints[_endpoint];
    for (const auto &client : clientsV)
    {
      if (client.address == _clientAddress)
      {
        std::cerr << "Broker::Bind() error: Address [" << _clientAddress
                  << "] already used in a previous Bind()" << std::endl;
        return false;
      }
    }
  }

  BrokerClientInfo clientInfo;
  clientInfo.address = _clientAddress;
  this->endpoints[_endpoint].push_back(clientInfo);
  return true;
}

//////////////////////////////////////////////////
void Broker::Push(const msgs::Datagram &_msg)
{
  // Queue the new message.
  this->incomingMsgs.push_back(_msg);
}

//////////////////////////////////////////////////
bool Broker::Register(const std::string &_id)
{
  if (this->swarm->find(_id) != this->swarm->end())
  {
    std::cerr << "Logger::Register() error: ID [" << _id << "] already exists"
              << std::endl;
    return false;
  }

  std::cout << "Broker::Register() Name: [" << _id << "]" << std::endl;

  auto const &model = gazebo::physics::get_world()->ModelByName(_id);
  if (!model)
  {
    std::cerr << "Broker::Register(): Error getting a model"
              << "pointer for model [" << _id << "]" << std::endl;
    return false;
  }

  auto newMember = std::make_shared<SwarmMember>();

  // Name and address are the same in SubT.
  newMember->address = _id;
  newMember->name = _id;
  newMember->model = model;
  (*this->swarm)[_id] = newMember;

  return true;
}

//////////////////////////////////////////////////
const std::map<std::string, std::vector<BrokerClientInfo>>
      &Broker::EndPoints() const
{
  return this->endpoints;
}

//////////////////////////////////////////////////
std::deque<msgs::Datagram> &Broker::Messages()
{
  return this->incomingMsgs;
}

//////////////////////////////////////////////////
bool Broker::Unregister(const std::string &_id)
{
  // Sanity check: Make sure that the ID exists.
  if (this->swarm->find(_id) == this->swarm->end())
  {
    std::cerr << "Broker::Unregister() error: ID [" << _id << "] doesn't exist"
              << std::endl;
    return false;
  }

  this->swarm->erase(_id);

  // Unbind.
  for (auto &endpointKv : this->endpoints)
  {
    auto &clientsV = endpointKv.second;

    auto i = std::begin(clientsV);
    while (i != std::end(clientsV))
    {
      if (i->address == _id)
        i = clientsV.erase(i);
      else
        ++i;
    }
  }

  return true;
}

//////////////////////////////////////////////////
void Broker::Reset()
{
  this->incomingMsgs.clear();
  this->endpoints.clear();
}

//////////////////////////////////////////////////
subt::SwarmMembershipPtr Broker::Swarm()
{
  return this->swarm;
}

//////////////////////////////////////////////////
void Broker::NotifyNeighbors()
{
  subt::msgs::Neighbor_M neighbors;

  // Send neighbors updates to each member of the swarm.
  for (auto const &robot : (*this->Swarm()))
  {
    auto address = robot.first;
    auto swarmMember = (*this->Swarm())[address];

    // Populate the list of neighbors for this address.
    ignition::msgs::StringMsg_V v;
    for (auto const &neighbor : swarmMember->neighbors)
      v.add_data(neighbor.first);

    // Add the list of neighbors for each address.
    (*neighbors.mutable_neighbors())[address] = v;
  }

  // Notify all clients the updated list of neighbors.
  this->neighborPub.Publish(neighbors);
}

/////////////////////////////////////////////////
bool Broker::OnAddrRegistration(const ignition::msgs::StringMsg &_req,
    ignition::msgs::Boolean &_rep)
{
  std::string address = _req.data();
  bool result;

  {
    std::lock_guard<std::mutex> lk(this->mutex);
    result = this->Register(address);
  }

  _rep.set_data(result);

  return result;
}

/////////////////////////////////////////////////
bool Broker::OnEndPointRegistration(const ignition::msgs::StringMsg_V &_req,
    ignition::msgs::Boolean &_rep)
{
  if (_req.data().size() != 2)
  {
    std::cerr << "[Broker::OnEndPointRegistration()] Expected two strings and "
              << "got " << _req.data().size() << " instead" << std::endl;
    return false;
  }

  std::string clientAddress = _req.data(0);
  std::string endpoint = _req.data(1);

  bool result = this->Bind(clientAddress, endpoint);
  _rep.set_data(result);

  return result;
}

/////////////////////////////////////////////////
void Broker::OnMessage(const subt::msgs::Datagram &_req)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->mutex);

  // Save the message.
  this->incomingMsgs.push_back(_req);
}
