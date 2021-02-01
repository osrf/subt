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
#include <ignition/common/Console.hh>
#include <subt_msgs/Bind.h>
#include <subt_msgs/Register.h>
#include <subt_msgs/Unregister.h>

#include <subt_communication_broker/subt_communication_client.h>
#include <subt_communication_broker/protobuf/endpoint_registration.pb.h>

using namespace subt;
using namespace subt::communication_broker;

//////////////////////////////////////////////////
CommsClient::CommsClient(const std::string &_localAddress,
  const bool _isPrivate, const bool _useIgnition, const bool _listenBeacons,
  ros::NodeHandle* _rosNh)
  : localAddress(_localAddress),
    isPrivate(_isPrivate),
    useIgnition(_useIgnition)
{
  this->enabled = false;

  // Sanity check: Verity that local address is not empty.
  if (_localAddress.empty())
  {
    std::cerr << "CommsClient::CommsClient() error: Local address shouldn't "
              << "be empty" << std::endl;
    return;
  }

  // Subscribe to the ignition clock topic, which will only be
  // available to the base station. The base station is run as a plugin
  // alongside simulation, and does not have access to ros::Time.
  if (this->useIgnition)
    this->node.Subscribe("/clock", &CommsClient::OnClock, this);

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

    // give Ctrl-C a chance
    if (!_useIgnition)
    {
      if (!ros::ok())
        return;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds (5));
    }
  }

  if (!this->enabled)
  {
    std::cerr << "[CommsClient] Validation service not available, invalid "
              << "address or model not available" << std::endl;
    return;
  }

  if (!this->useIgnition)
  {
    ros::NodeHandle nh;
    if (_rosNh != nullptr)
      nh = *_rosNh;

    this->commsSub = nh.subscribe(
      "/" + _localAddress + "/comms",1000,
      &CommsClient::OnMessageRos, this);
  }

  if (_listenBeacons)
  {
    // Bind to be sure we receive beacon packets
    auto cb = [] (const std::string&,
                  const std::string&,
                  const uint32_t,
                  const std::string&) {  };
    if (!this->Bind(cb, "", kBeaconPort))
    {
      std::cerr << "[CommsClient] Could bind the beacon responder" << std::endl;
      this->Unregister();
      return;
    }
  }

  this->enabled = true;
}

//////////////////////////////////////////////////
CommsClient::~CommsClient()
{
  this->beaconRunning = false;
  this->Unregister();
  if (this->beaconThread)
  {
    this->beaconThread->join();
    delete this->beaconThread;
    this->beaconThread = nullptr;
  }
}

//////////////////////////////////////////////////
std::string CommsClient::Host() const
{
  return this->localAddress;
}

//////////////////////////////////////////////////
std::vector<std::pair<communication_broker::EndpointID, std::string>>
CommsClient::Bind(std::function<void(const std::string &_srcAddress,
                                     const std::string &_dstAddress,
                                     const uint32_t _dstPort,
                                     const std::string &_data)> _cb,
                  const std::string &_address,
                  const int _port)
{
  std::vector<std::pair<EndpointID, std::string>> endpoints;

  // Sanity check: Make sure that the communications are enabled.
  if (!this->enabled || this->clientId == invalidClientId)
  {
    std::cerr << "[" << this->Host()
      << "] Bind() error: Trying to bind before communications are enabled!"
      << std::endl;
    return endpoints;
  }

  // Use current address if _address is not provided.
  std::string address = _address;
  if (address.empty())
    address = this->Host();

  // Sanity check: Make sure that you use your local address or multicast.
  if ((address != communication_broker::kMulticast) &&
      (address != this->Host()))
  {
    std::cerr << "[" << this->Host() << "] Bind() error: Address ["
              << address << "] is not your local address" << std::endl;
    return endpoints;
  }

  // Mapping the "unicast socket" to an endpoint name.
  const auto unicastEndPoint = address + ":" + std::to_string(_port);
  const auto bcastEndpoint = communication_broker::kBroadcast + ":" +
    std::to_string(_port);

  // Register the endpoints in the broker.
  for (const std::string &endpoint : {unicastEndPoint, bcastEndpoint})
  {
    // If this is the basestation, then we need to use ignition transport.
    // Otherwise, the client is on a robot and needs to use ROS.
    if (this->useIgnition)
    {
      subt::msgs::EndpointRegistration req;
      req.set_client_id(this->clientId);
      req.set_endpoint(endpoint);

      const unsigned int timeout = 3000u;
      ignition::msgs::UInt32 rep;
      bool result;
      bool executed = this->node.Request(
          communication_broker::kEndPointRegistrationSrv,
          req, timeout, rep, result);

      if (!executed)
      {
        std::cerr << "[CommsClient] Endpoint registration srv not available"
          << std::endl;
        return endpoints;
      }

      const auto endpointID = rep.data();
      
      if (!result || endpointID == invalidEndpointId)
      {
        std::cerr << "[CommsClient] Invalid endpoint registration data."
                  << std::endl;
        return endpoints;
      }

      endpoints.emplace_back(std::make_pair(endpointID, endpoint));
    }
    else
    {
      subt_msgs::Bind::Request req;
      req.client_id = this->clientId;
      req.endpoint = endpoint;

      subt_msgs::Bind::Response rep;

      bool executed = ros::service::call(
          communication_broker::kEndPointRegistrationSrv, req, rep);

      if (!executed)
      {
        std::cerr << "[CommsClient] Endpoint registration srv not available"
          << std::endl;
        return endpoints;
      }

      if (rep.endpoint_id == communication_broker::invalidEndpointId)
      {
        std::cerr << "[CommsClient] Invalid endpoint registration data."
                  << std::endl;
        return endpoints;
      }

      endpoints.emplace_back(std::make_pair(rep.endpoint_id, endpoint));
    }
  }

  if (this->useIgnition)
  {
    // Ignition transport registers a datagram receive service, so we have to
    // make sure we only advertise it once
    if (!this->advertised)
    {
      // Advertise a oneway service for receiving message requests. This assumes
      // there is only a single node running the client in useIgnition mode.
      ignition::transport::AdvertiseServiceOptions opts;
      if (this->isPrivate)
        opts.SetScope(ignition::transport::Scope_t::PROCESS);

      if (!this->node.Advertise(address, &CommsClient::OnMessage, this, opts))
      {
        std::cerr << "[" << this->Host() << "] Bind Error: could not advertise "
          << address << std::endl;

        // if we cannot advertise but we have already bound the endpoints,
        // we need to unbind them before exiting with error
        for (const auto& endpoint : endpoints)
        {
          ignition::msgs::UInt32 req;
          req.set_data(endpoint.first);
          this->node.Request(kEndPointUnregistrationSrv, req);
        }
        endpoints.clear();
        return endpoints;
      }
    }

    this->advertised = true;
  }

  // Register the callbacks.
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    for (const auto& endpoint : endpoints)
    {
      const auto& endpointID = endpoint.first;
      const auto& endpointName = endpoint.second;

      ignmsg << "Storing callback for " << endpointName << std::endl;

      this->callbacks[endpointName][endpointID] = std::bind(_cb,
          std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3, std::placeholders::_4);
    }
  }

  return endpoints;
}

//////////////////////////////////////////////////
bool CommsClient::Unbind(communication_broker::EndpointID _endpointId)
{
  if (!this->enabled || this->clientId == invalidClientId)
  {
    std::cerr << "[" << this->localAddress << "] CommsClient::Unbind:"
              << "Calling Unregister() before registering the client."
              << std::endl;
    return false;
  }

  std::string endpoint;
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    for (const auto& callbackKV : this->callbacks)
    {
      const auto& endpointName = callbackKV.first;
      const auto& endpointCallbacks = callbackKV.second;

      for (const auto& endpointKV : endpointCallbacks)
      {
        const auto& endpointID = endpointKV.first;
        if (endpointID == _endpointId)
        {
          endpoint = endpointName;
          break;
        }
      }
      if (!endpoint.empty())
        break;
    }
  }

  if (endpoint.empty())
  {
    std::cerr << "Trying to unbind an endpoint that is not bound." << std::endl;
    return false;
  }

  ignition::msgs::UInt32 req;
  req.set_data(_endpointId);
  const unsigned int timeout = 3000u;
  ignition::msgs::Boolean rep;
  bool result;

  bool executed = this->node.Request(
    kEndPointUnregistrationSrv, req, timeout, rep, result);

  if (!executed)
    return false;

  if (result && rep.data())
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    this->callbacks[endpoint].erase(_endpointId);
    if (this->callbacks[endpoint].empty())
      this->callbacks.erase(endpoint);
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
bool CommsClient::SendTo(const std::string &_data,
    const std::string &_dstAddress, const uint32_t _port)
{
  // Sanity check: Make sure that the communications are enabled.
  if (!this->enabled || this->clientId == invalidClientId)
  {
    std::cerr << "[" << this->localAddress << "] CommsClient::SendTo:"
              << "Calling Unregister() before registering the client."
              << std::endl;
    return false;
  }

  // Restrict the maximum size of a message.
  if (_data.size() > this->kMtu)
  {
    std::cerr << "[" << this->Host() << "] CommsClient::SendTo() error: "
              << "Payload size (" << _data.size() << ") is greater than the "
              << "maximum allowed (" << this->kMtu << ")" << std::endl;
    return false;
  }

  if (this->useIgnition)
  {
    msgs::Datagram msg;
    msg.set_src_address(this->Host());
    msg.set_dst_address(_dstAddress);
    msg.set_dst_port(_port);
    msg.set_data(_data);

    return this->node.Request(kBrokerSrv, msg);
  }
  else
  {
    subt_msgs::DatagramRos::Request req;
    subt_msgs::DatagramRos::Response rep;
    req.src_address = this->Host();
    req.dst_address = _dstAddress;
    req.dst_port = _port;
    req.data = _data;

    return ros::service::call(kBrokerSrv, req, rep);
  }
}

//////////////////////////////////////////////////
CommsClient::Neighbor_M CommsClient::Neighbors() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->neighbors;
}

//////////////////////////////////////////////////
bool CommsClient::SendBeacon()
{
  return this->SendTo("hello",
               subt::communication_broker::kBroadcast,
               kBeaconPort);
}

//////////////////////////////////////////////////
void CommsClient::StartBeaconInterval(ros::Duration _period)
{

  // Stop the current beacon, if present
  if (this->beaconThread)
  {
    this->beaconRunning = false;
    this->beaconThread->join();
    delete this->beaconThread;
    this->beaconThread = nullptr;
  }

  this->beaconPeriodNs = _period.toNSec();
  // Start the beacon
  this->beaconThread = new std::thread([&]()
    {
      while (this->beaconRunning)
      {
        this->SendBeacon();
        std::this_thread::sleep_for(std::chrono::nanoseconds(
              this->beaconPeriodNs));
      }
    });
}

//////////////////////////////////////////////////
bool CommsClient::Register()
{
  if (this->enabled || this->clientId != invalidClientId)
  {
    std::cerr << "[" << this->localAddress
              << "] CommsClient::Register: Calling Register() on an already "
              << "registered client."
              << std::endl;
    return false;
  }

  bool executed;
  bool result;

  // Use ignition transport if this is the base station. Otherwise, use ROS.
  if (this->useIgnition)
  {
    ignition::msgs::StringMsg req;
    req.set_data(this->localAddress);

    ignition::msgs::UInt32 rep;
    const unsigned int timeout = 3000u;

    executed = this->node.Request(
        kAddrRegistrationSrv, req, timeout, rep, result);

    if (executed && result)
      this->clientId = rep.data();
  }
  else
  {
    subt_msgs::Register::Request req;
    subt_msgs::Register::Response rep;

    req.local_address = this->localAddress;

    executed = ros::service::call(kAddrRegistrationSrv, req, rep);
    result = executed;
    if (executed)
      this->clientId = rep.client_id;
  }

  if (!executed || !result || this->clientId == invalidClientId)
  {
    std::cerr << "[" << this->localAddress
      << "] CommsClient::Register: Problem registering with broker"
      << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool CommsClient::Unregister()
{
  bool executed;
  bool result;

  if (!this->enabled || this->clientId == invalidClientId)
  {
    std::cerr << "[" << this->localAddress << "] CommsClient::Unregister:"
              << "Calling Unregister() before registering the client."
              << std::endl;
    return false;
  }

  // unbind all endpoints

  // copy the callbacks array because Unbind() removes items from it
  decltype(this->callbacks) callbacksCopy;
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    callbacksCopy = this->callbacks;
  }

  for (const auto& callbackKV : callbacksCopy)
  {
    // we intentionally ignore failures in unbind as there's nothing to do
    for (const auto& endpointKV : callbackKV.second)
      this->Unbind(endpointKV.first);
  }
  // to be sure none are left there in case a later Register() is called
  this->callbacks.clear();

  // Use ignition transport if this is the base station. Otherwise, use ROS.
  if (this->useIgnition)
  {
    ignition::msgs::UInt32 req;
    req.set_data(this->clientId);

    ignition::msgs::Boolean rep;
    const unsigned int timeout = 3000u;

    executed = this->node.Request(
        kAddrUnregistrationSrv, req, timeout, rep, result);
  }
  else
  {
    subt_msgs::Unregister::Request req;
    subt_msgs::Unregister::Response rep;

    req.client_id = this->clientId;

    executed = ros::service::call(kAddrUnregistrationSrv, req, rep);
    result = rep.success;
  }

  if (executed && result)
    this->clientId = invalidClientId;

  return executed && result;
}

//////////////////////////////////////////////////
void CommsClient::OnMessage(const msgs::Datagram &_msg)
{
  auto endPoint = _msg.dst_address() + ":" + std::to_string(_msg.dst_port());

  std::lock_guard<std::mutex> lock(this->mutex);

  std::scoped_lock<std::mutex> lk(this->clockMutex);
  double time = this->clockMsg.sim().sec() +
    this->clockMsg.sim().nsec() * 1e-9;
  this->neighbors[_msg.src_address()] = std::make_pair(time, _msg.rssi());

  if (this->callbacks.find(endPoint) == this->callbacks.end())
    return;

  for (const auto& endpointCallbacksKV : this->callbacks[endPoint])
  {
    auto& callback = endpointCallbacksKV.second;
    if (callback)
    {
      callback(_msg.src_address(), _msg.dst_address(),
               _msg.dst_port(), _msg.data());
    }
  }
}

//////////////////////////////////////////////////
void CommsClient::OnMessageRos(const subt_msgs::DatagramRos::Request &_req)
{
  auto endPoint = _req.dst_address + ":" + std::to_string(_req.dst_port);

  std::lock_guard<std::mutex> lock(this->mutex);

  this->neighbors[_req.src_address] =
      std::make_pair(ros::Time::now().toSec(), _req.rssi);

  if (this->callbacks.find(endPoint) == this->callbacks.end())
    return;

  for (const auto& endpointCallbacksKV : this->callbacks[endPoint])
  {
    auto& callback = endpointCallbacksKV.second;
    if (callback)
    {
      callback(_req.src_address, _req.dst_address,
               _req.dst_port, _req.data);
    }
  }
}

//////////////////////////////////////////////////
void CommsClient::OnClock(const ignition::msgs::Clock &_clock)
{
  std::scoped_lock<std::mutex> lk(this->clockMutex);
  this->clockMsg.CopyFrom(_clock);
}
