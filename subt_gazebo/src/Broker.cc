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
#include <deque>
#include <iostream>
#include <map>
#include <string>
#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/Broker.hh"

using namespace subt;

//////////////////////////////////////////////////
bool Broker::Bind(const std::string &_clientAddress, const std::string &_endpoint)
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
  if (std::find(this->clients.begin(), this->clients.end(), _id) !=
        this->clients.end())
  {
    std::cerr << "Logger::Register() error: ID [" << _id << "] already exists"
              << std::endl;
    return false;
  }

  this->clients.push_back(_id);
  return true;
}

//////////////////////////////////////////////////
const std::vector<std::string> &Broker::Clients() const
{
  return this->clients;
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
  if (std::find(this->clients.begin(), this->clients.end(), _id) ==
        this->clients.end())
  {
    std::cerr << "Broker::Unregister() error: ID [" << _id << "] doesn't exist"
              << std::endl;
    return false;
  }

  this->clients.erase(std::remove(
    this->clients.begin(), this->clients.end(), _id), this->clients.end());

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
