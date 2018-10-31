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

#include <chrono>
#include <iostream>
#include <string>

#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/protobuf/neighbor_m.pb.h"
#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/CommsClient.hh"

using namespace subt;

//////////////////////////////////////////////////
CommsClient::CommsClient(const std::string &_localAddress,
  const bool _isPrivate)
  : localAddress(_localAddress),
    isPrivate(_isPrivate)
{
  this->enabled = false;

  // Sanity check: Verity that local address is not empty.
  if (_localAddress.empty())
  {
    std::cerr << "CommsClient::CommsClient() error: Local address shouldn't "
              << "be empty" << std::endl;
    return;
  }

  const unsigned int kMaxWaitTime = 10000u;
  const auto kStart = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::steady_clock::now() - kStart;
  this->enabled = this->Register();

  // Retry registration for some time. The broker could be unavailable or
  // the model still not inserted into the simulation.
  while (!this->enabled && std::chrono::duration_cast<
                    std::chrono::milliseconds>(elapsed).count() <= kMaxWaitTime)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    this->enabled = this->Register();
    elapsed = std::chrono::steady_clock::now() - kStart;
  }

  if (!this->enabled)
  {
    std::cerr << "[CommsClient] Validation service not available, invalid "
              << "address or model not available" << std::endl;
    return;
  }

  // Subscribe to the topic where neighbor updates are notified.
  if (!this->node.Subscribe(kNeighborsTopic, &CommsClient::OnNeighbors, this))
  {
    std::cerr << "Error subscribing to topic [" << kNeighborsTopic << "]"
              << std::endl;
    return;
  }

  this->enabled = true;
}

//////////////////////////////////////////////////
std::string CommsClient::Host() const
{
  return this->localAddress;
}

//////////////////////////////////////////////////
bool CommsClient::SendTo(const std::string &_data,
    const std::string &_dstAddress, const uint32_t _port)
{
  // Sanity check: Make sure that the communications are enabled.
  if (!this->enabled)
    return false;

  // Restrict the maximum size of a message.
  if (_data.size() > this->kMtu)
  {
    std::cerr << "[" << this->Host() << "] CommsClient::SendTo() error: "
              << "Payload size (" << _data.size() << ") is greater than the "
              << "maximum allowed (" << this->kMtu << ")" << std::endl;
    return false;
  }

  msgs::Datagram msg;
  msg.set_src_address(this->Host());
  msg.set_dst_address(_dstAddress);
  msg.set_dst_port(_port);
  msg.set_data(_data);

  return this->node.Request(kBrokerSrv, msg);
}

//////////////////////////////////////////////////
bool CommsClient::SendToBaseStation(const subt::msgs::Artifact &_artifact)
{
  // Serialize the artifact.
  std::string serializedData;
  if (!_artifact.SerializeToString(&serializedData))
  {
    std::cerr << "CommsClient::SendToBaseStation(): Error serializing message\n"
              << _artifact.DebugString() << std::endl;
    return false;
  }

  // Send data to the base station.
  return this->SendTo(serializedData, kBaseStationName);
}

//////////////////////////////////////////////////
std::vector<std::string> CommsClient::Neighbors() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->neighbors;
}

//////////////////////////////////////////////////
bool CommsClient::Register()
{
  ignition::msgs::StringMsg req;
  req.set_data(this->localAddress);

  ignition::msgs::Boolean rep;
  bool result;
  const unsigned int timeout = 3000u;

  bool executed = this->node.Request(
    kAddrRegistrationSrv, req, timeout, rep, result);

  return executed && result;
}

//////////////////////////////////////////////////
void CommsClient::OnMessage(const msgs::Datagram &_msg)
{
  auto endPoint = _msg.dst_address() + ":" + std::to_string(_msg.dst_port());

  std::lock_guard<std::mutex> lock(this->mutex);
  for (auto cb : this->callbacks)
  {
    if (cb.first == endPoint && cb.second)
    {
      cb.second(_msg.src_address(), _msg.dst_address(),
                _msg.dst_port(), _msg.data());
    }
  }
}

//////////////////////////////////////////////////
void CommsClient::OnNeighbors(const msgs::Neighbor_M &_neighbors)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  this->neighbors.clear();

  if (_neighbors.neighbors().find(this->localAddress) ==
        _neighbors.neighbors().end())
  {
    std::cerr << "[CommsClient::OnNeighborsReceived] My current address ["
              << this->localAddress << "] is not included in this neighbor "
              << "update" << std::endl;
    return;
  }

  auto currentNeighbors = _neighbors.neighbors().at(this->localAddress);
  for (int i = 0; i < currentNeighbors.data().size(); ++i)
    this->neighbors.push_back(currentNeighbors.data(i));
}
