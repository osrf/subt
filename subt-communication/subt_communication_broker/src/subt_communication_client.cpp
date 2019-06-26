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

#include <subt_communication_broker/subt_communication_client.h>

using namespace subt;
using namespace subt::communication_broker;

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
    std::cerr << "[CommsClient] Retrying register.." << std::endl;
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

  // Bind to be sure we receive beacon packets
  auto cb = [] (const std::string&,
                const std::string&,
                const uint32_t,
                const std::string&) {  };
  this->Bind(cb, "", kBeaconPort);

  this->enabled = true;
}

//////////////////////////////////////////////////
CommsClient::~CommsClient()
{
  this->Unregister();
}

//////////////////////////////////////////////////
std::string CommsClient::Host() const
{
  return this->localAddress;
}

//////////////////////////////////////////////////
bool CommsClient::Bind(std::function<void(const std::string &_srcAddress,
                                          const std::string &_dstAddress,
                                          const uint32_t _dstPort,
                                          const std::string &_data)> _cb,
                       const std::string &_address,
                       const int _port)
{
  // Sanity check: Make sure that the communications are enabled.
  if (!this->enabled) {
    std::cerr << "[" << this->Host() << "] Bind() error: Trying to bind before communications are enabled!" << std::endl;
    return false;
  }

  // Use current address if _address is not provided.
  std::string address = _address;
  if (address.empty())
    address = this->Host();

  // Sanity check: Make sure that you use your local address or multicast.
  if ((address != communication_broker::kMulticast) && (address != this->Host()))
  {
    std::cerr << "[" << this->Host() << "] Bind() error: Address ["
              << address << "] is not your local address" << std::endl;
    return false;
  }

  // Mapping the "unicast socket" to a topic name.
  const auto unicastEndPoint = address + ":" + std::to_string(_port);
  const auto bcastEndpoint = communication_broker::kBroadcast + ":" + std::to_string(_port);
  bool bcastAdvertiseNeeded;

  {
    std::lock_guard<std::mutex> lock(this->mutex);

    // Sanity check: Make sure that this address is not already used.
    if (this->callbacks.find(unicastEndPoint) != this->callbacks.end())
    {
      std::cerr << "[" << this->Host() << "] Bind() error: Address ["
                << address << "] already used" << std::endl;
      return false;
    }

    bcastAdvertiseNeeded =
        this->callbacks.find(bcastEndpoint) == this->callbacks.end();
  }

  // Register the endpoints in the broker.
  // Note that the broadcast endpoint will only be registered once.
  for (std::string endpoint : {unicastEndPoint, bcastEndpoint})
  {
    if (endpoint != bcastEndpoint || bcastAdvertiseNeeded)
    {
      ignition::msgs::StringMsg_V req;
      req.add_data(address);
      req.add_data(endpoint);

      const unsigned int timeout = 3000u;
      ignition::msgs::Boolean rep;
      bool result;
      bool executed = this->node.Request(
          communication_broker::kEndPointRegistrationSrv, req, timeout, rep, result);

      if (!executed)
      {
        std::cerr << "[CommsClient] Endpoint registration srv not available"
                  << std::endl;
        return false;
      }

      if (!result)
      {
        std::cerr << "[CommsClient] Invalid data. Did you send the address "
                  << "followed by the endpoint?" << std::endl;
        return false;
      }
    }
  }

  if(!advertised) {
    // Advertise a oneway service for receiving message requests.
    ignition::transport::AdvertiseServiceOptions opts;
    if (this->isPrivate)
      opts.SetScope(ignition::transport::Scope_t::PROCESS);


    if (!this->node.Advertise(address, &CommsClient::OnMessage, this, opts)) {
      std::cerr << "[" << this->Host() << "] Bind Error: could not advertise " << address << std::endl;
      return false;
    }

    advertised = true;
  }

  // Register the callbacks.
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    for (std::string endpoint : {unicastEndPoint, bcastEndpoint})
    {
      if (endpoint != bcastEndpoint || bcastAdvertiseNeeded)
      {
        ROS_INFO("Storing callback for %s", endpoint.c_str());
        this->callbacks[endpoint] = std::bind(_cb,
                                              std::placeholders::_1, std::placeholders::_2,
                                              std::placeholders::_3, std::placeholders::_4);
      }
      else {
        ROS_WARN("Skipping callback register for %s", endpoint.c_str());
      }
    }
  }

  return true;
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
CommsClient::Neighbor_M CommsClient::Neighbors() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->neighbors;
}

bool CommsClient::SendBeacon()
{
  return this->SendTo("hello",
               subt::communication_broker::kBroadcast,
               kBeaconPort);
}

void CommsClient::StartBeaconInterval(ros::Duration period)
{
  ros::NodeHandle nh;
  auto cb = [this](const ros::TimerEvent&) {
    this->SendBeacon();
  };
  beacon_timer = nh.createTimer(period, cb);
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

  if(!executed) {
    std::cerr << "[" << this->localAddress
              << "] CommsClient::Register: Problem registering with broker"
              << std::endl;
  }

  return executed && result;
}

//////////////////////////////////////////////////
bool CommsClient::Unregister()
{
  ignition::msgs::StringMsg req;
  req.set_data(this->localAddress);

  ignition::msgs::Boolean rep;
  bool result;
  const unsigned int timeout = 3000u;

  bool executed = this->node.Request(
    kAddrUnregistrationSrv, req, timeout, rep, result);

  return executed && result;
}

//////////////////////////////////////////////////
void CommsClient::OnMessage(const msgs::Datagram &_msg)
{
  auto endPoint = _msg.dst_address() + ":" + std::to_string(_msg.dst_port());

  std::lock_guard<std::mutex> lock(this->mutex);

  this->neighbors[_msg.src_address()] =
      std::make_pair(ros::Time::now(), _msg.rssi());

  for (auto cb : this->callbacks)
  {
    if (cb.first == endPoint && cb.second)
    {
      cb.second(_msg.src_address(), _msg.dst_address(),
                _msg.dst_port(), _msg.data());
    }
  }
}
