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
#include <memory>
#include <iostream>
#include <ignition/math/Rand.hh>

#include <subt_communication_broker/subt_communication_broker.h>
#include <subt_communication_broker/protobuf/datagram.pb.h>
#include <subt_communication_broker/protobuf/neighbor_m.pb.h>

#include <subt_communication_broker/common_types.h>

namespace subt
{

namespace communication_broker
{

//////////////////////////////////////////////////
Broker::Broker()
    : team(std::make_shared<TeamMembership_M>())// ,
      // rndEngine(std::default_random_engine(ignition::math::Rand::Seed()))
{

}

//////////////////////////////////////////////////
void Broker::Start()
{
  // Advertise the service for registering addresses.
  if (!this->node.Advertise(kAddrRegistrationSrv,
                            &Broker::OnAddrRegistration, this))
  {
    std::cerr << "Error advertising srv [" << kAddrRegistrationSrv << "]"
              << std::endl;
    return;
  }

  // Advertise the service for unregistering addresses.
  if (!this->node.Advertise(kAddrUnregistrationSrv,
                            &Broker::OnAddrUnregistration, this))
  {
    std::cerr << "Error advertising srv [" << kAddrUnregistrationSrv << "]"
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
void Broker::Reset()
{
  std::lock_guard<std::mutex> lk(this->mutex);
  this->incomingMsgs.clear();
  this->endpoints.clear();
}

//////////////////////////////////////////////////
TeamMembershipPtr Broker::Team()
{
  return this->team;
}

//////////////////////////////////////////////////
void Broker::NotifyNeighbors()
{
  subt::msgs::Neighbor_M neighbors;

  // Send neighbors updates to each member of the team.
  for (auto const &robot : (*this->Team()))
  {
    auto address = robot.first;
    auto teamMember = (*this->Team())[address];

    // Populate the list of neighbors for this address.
    ignition::msgs::StringMsg_V v;
    for (auto const &neighbor : teamMember->neighbors)
      v.add_data(neighbor.first);

    // Add the list of neighbors for each address.
    (*neighbors.mutable_neighbors())[address] = v;
  }

  // Notify all clients the updated list of neighbors.
  if (!this->neighborPub.Publish(neighbors))
    std::cerr << "[Broker::NotifyNeighbors(): Error on update" << std::endl;
}

//////////////////////////////////////////////////
void Broker::DispatchMessages()
{
  std::lock_guard<std::mutex> lk(this->mutex);

  if(this->incomingMsgs.empty())
    return;

  // Cannot dispatch messages if we don't have function handles for
  // pathloss and communication
  if(!communication_function) {
    std::cerr << "[Broker::DispatchMessages()] Missing function handle for communication" << std::endl;
    return;
  }

  if(!pose_update_f) {
    std::cerr << "[Broker::DispatchMessages()]: Missing function for updating pose" << std::endl;
    return;
  }

  // Update state for all members in team (only do this for members
  // which touch messages in the queue?)
  for(auto t : *(this->team)) {
    bool ret;
    std::tie(ret, t.second->rf_state.pose) = pose_update_f(t.second->name);

    if(!ret) {
      std::cerr << "Problem getting state for " << t.second->name
                << ", skipping DispatchMessages()" << std::endl;
      return;
    }
  }
  
  // Shuffle the messages.
  // std::shuffle(this->incomingMsgs.begin(), this->incomingMsgs.end(),
  //     this->rndEngine);

  while (!this->incomingMsgs.empty())
  {
    // Get the next message to dispatch.
    const subt::msgs::Datagram msg = this->incomingMsgs.front();
    this->incomingMsgs.pop_front();

    // Sanity check: Make sure that the sender is a member of the team.
    auto tx_node = this->team->find(msg.src_address());
    if (tx_node == this->team->end())
    {
      std::cerr << "Broker::DispatchMessages(): Discarding message. Robot ["
                << msg.src_address() << "] is not registered as a member of the"
                << " team" << std::endl;
      continue;
    }

    // Get the list of neighbors of the sender.
    // const Neighbors_M &neighbors = (*this->team)[msg.src_address()]->neighbors;

    std::string dstEndPoint =
        msg.dst_address() + ":" + std::to_string(msg.dst_port());

    if (this->endpoints.find(dstEndPoint) != this->endpoints.end())
    {
      // Shuffle the clients bound to this endpoint.
      std::vector<BrokerClientInfo> clientsV = this->endpoints.at(dstEndPoint);
      // std::shuffle(clientsV.begin(), clientsV.end(), this->rndEngine);

      if(clientsV.empty()) {
        std::cerr << "[Broker::DispatchMessages()]: No clients for endpoint " << dstEndPoint << std::endl;
      }

      for (const BrokerClientInfo &client : clientsV)
      {
        auto rx_node = this->team->find(client.address);
        if(rx_node == this->team->end()) {
          std::cerr << "Broker::DispatchMessages(): Skipping send attempt. Robot ["
                    << client.address << "] is not registered as a member of the"
                    << " team" << std::endl;
          continue;
        }
        
        // Query communication_model if this packet is successful,
        // forward if so
        if(!tx_node->second->radio.pathloss_f) {
          std::cerr << "No pathloss function defined for " << msg.src_address() << std::endl;
          continue;
        }
        
        if(communication_function(tx_node->second->radio,
                                  tx_node->second->rf_state,
                                  rx_node->second->rf_state,
                                  msg.data().size())) {
          
          if (!this->node.Request(client.address, msg))
          {
            std::cerr << "[CommsBrokerPlugin::DispatchMessages()]: Error "
                      << "sending message to [" << client.address << "]"
                      << std::endl;
          }
        }

      }
    }
    else {
      std::cerr << "[Broker::DispatchMessages()]: Could not find endpoint " << dstEndPoint << std::endl;
    }
  }
}

//////////////////////////////////////////////////
bool Broker::Bind(const std::string &_clientAddress,
                  const std::string &_endpoint)
{
  std::lock_guard<std::mutex> lk(this->mutex);
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

  std::cout << "New endpoint registered [" << _endpoint << "] for client ["
            << _clientAddress << "]" << std::endl;

  return true;
}

//////////////////////////////////////////////////
bool Broker::Register(const std::string &_id)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  auto kvp = this->team->find(_id);
  if (kvp != this->team->end())
  {
    std::cerr << "Broker::Register() warning: ID [" << _id << "] already exists"
              << std::endl;
  }
  else {
    auto newMember = std::make_shared<TeamMember>();

    // Name and address are the same in SubT.
    newMember->address = _id;
    newMember->name = _id;

    newMember->radio = default_radio_configuration;
    (*this->team)[_id] = newMember;

    std::cout << "New client registered [" << _id << "]" <<  std::endl;
  }

  return true;
}

//////////////////////////////////////////////////
bool Broker::Unregister(const std::string &_id)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  // Sanity check: Make sure that the ID exists.
  if (this->team->find(_id) == this->team->end())
  {
    std::cerr << "Broker::Unregister() error: ID [" << _id << "] doesn't exist"
              << std::endl;
    return false;
  }

  this->team->erase(_id);

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

/////////////////////////////////////////////////
bool Broker::OnAddrRegistration(const ignition::msgs::StringMsg &_req,
                                ignition::msgs::Boolean &_rep)
{
  std::string address = _req.data();
  bool result;

  result = this->Register(address);

  _rep.set_data(result);

  return result;
}

/////////////////////////////////////////////////
bool Broker::OnAddrUnregistration(const ignition::msgs::StringMsg &_req,
                                  ignition::msgs::Boolean &_rep)
{
  std::string address = _req.data();
  bool result;

  result = this->Unregister(address);

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

  bool result;
  std::string clientAddress = _req.data(0);
  std::string endpoint = _req.data(1);

  result = this->Bind(clientAddress, endpoint);

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

void Broker::SetRadioConfiguration(const std::string& address,
                                   communication_model::radio_configuration config)
{
  std::unique_lock<std::mutex> lk(this->mutex);
  auto node = this->team->find(address);
  if(node == this->team->end()) {

    lk.unlock();
    this->Register(address);
    lk.lock();

    node = this->team->find(address);
    if(node == this->team->end()) {
      std::cerr << "Cannot set radio configuration for " << address << std::endl;
      return;
    }
  }

  node->second->radio = config;
  
}

//////////////////////////////////////////////////
void Broker::SetDefaultRadioConfiguration(communication_model::radio_configuration config)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  this->default_radio_configuration = config;
}

//////////////////////////////////////////////////
void Broker::SetCommunicationFunction(
    communication_model::communication_function f)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  communication_function = f;
}

//////////////////////////////////////////////////
void Broker::SetPoseUpdateFunction(pose_update_function f)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  pose_update_f = f;
}

}
}
