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
#ifndef SUBT_GAZEBO_COMMSCLIENT_HH_
#define SUBT_GAZEBO_COMMSCLIENT_HH_

#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <ignition/transport/Node.hh>

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/protobuf/artifact.pb.h"
#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/protobuf/neighbor_m.pb.h"

namespace subt
{
  /// \brief ToDo.
  class CommsClient
  {
    /// \brief Constructor.
    /// \param[in] _localAddress Your local address.
    /// Important: This address must be equal to a Gazebo model name.
    public: explicit CommsClient(const std::string &_localAddress,
                                 const bool _isPrivate = false);

    /// \brief Get your local address.
    /// \return The local address.
    public: std::string Host() const;

    /// \brief This method can bind a local address and a port to a
    /// virtual socket. This is a required step if your agent needs to
    /// receive messages.
    ///
    /// \param[in] _cb Callback function to be executed when a new message is
    /// received associated to the specified <_address, port>.
    /// In the callback, "_srcAddress" contains the address of the sender of
    /// the message. "_dstAddress" contains the destination address. "_dstPort"
    /// contains the destination port. "_data" contains the payload.
    /// \param[in] _obj Instance containing the member function callback.
    /// \param[in] _address Local address or "kMulticast". If you specify your
    /// local address, you will receive all the messages sent where the
    /// destination is <YOUR_LOCAL_ADDRESS, port> or <"kBroadcast", port>. On
    /// the other hand, if you specify "kMulticast" as the _address parameter,
    /// you will be subscribed to the multicast group <"kMulticast, port>".
    /// You will receive all the messages sent from any node to this multicast
    /// group.
    /// \param[in] _port Port used to receive messages.
    /// \return True when success or false otherwise.
    ///
    /// * Example usage (bind on the local address and default port):
    ///    this->Bind(&MyClass::OnDataReceived, this, "192.168.1.3");
    /// * Example usage (Bind on the multicast group and custom port.):
    ///    this->Bind(&MyClass::OnDataReceived, this, this->kMulticast, 5123);
    public: template<typename C>
    bool Bind(void(C::*_cb)(const std::string &_srcAddress,
                            const std::string &_dstAddress,
                            const uint32_t _dstPort,
                            const std::string &_data),
              C *_obj,
              const std::string &_address = "",
              const int _port = kDefaultPort)
    {
      // Sanity check: Make sure that the communications are enabled.
      if (!this->enabled)
        return false;

      // Use current address if _address is not provided.
      std::string address = _address;
      if (address.empty())
        address = this->Host();

      // Sanity check: Make sure that you use your local address or multicast.
      if ((address != kMulticast) && (address != this->Host()))
      {
        std::cerr << "[" << this->Host() << "] Bind() error: Address ["
                  << address << "] is not your local address" << std::endl;
        return false;
      }

      // Mapping the "unicast socket" to a topic name.
      const auto unicastEndPoint = address + ":" + std::to_string(_port);
      const auto bcastEndpoint = kBroadcast + ":" + std::to_string(_port);
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
            kEndPointRegistrationSrv, req, timeout, rep, result);

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

      // Advertise a oneway service for receiving message requests.
      ignition::transport::AdvertiseServiceOptions opts;
      if (this->isPrivate)
        opts.SetScope(ignition::transport::Scope_t::PROCESS);
      if (!this->node.Advertise(address, &CommsClient::OnMessage, this, opts))
        return false;

      // Register the callbacks.
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        for (std::string endpoint : {unicastEndPoint, bcastEndpoint})
        {
          if (endpoint != bcastEndpoint || bcastAdvertiseNeeded)
          {
            this->callbacks[endpoint] = std::bind(_cb, _obj,
              std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3, std::placeholders::_4);
          }
        }
      }

      return true;
    }

    /// \brief Send some data to other/s member/s of the team.
    ///
    /// \param[in] _data Payload. The maximum size of the payload is 1500 bytes.
    /// \param[in] _dstAddress Destination address. Note that the destination
    /// address might be a unicast address, "kBroadcast" or "kMulticast".
    /// In the case of broadcast and multicast communications your node
    /// will receive your own message if you're bind to your local or the
    /// multicast address.
    /// \param[in] _port Destination port.
    /// \return True when success or false otherwise (e.g.: if the payload was
    /// bigger than 1500 bytes).
    public: bool SendTo(const std::string &_data,
                        const std::string &_dstAddress,
                        const uint32_t _port = kDefaultPort);

    /// \brief Send some data to other/s member/s of the team.
    ///
    /// \param[in] _artifact Artifact to be reported to the base station.
    /// \return True when success or false otherwise (e.g.: if the payload was
    /// bigger than 1500 bytes).
    public: bool SendToBaseStation(const subt::msgs::Artifact &_artifact);

    /// \brief Get the list of local neighbors.
    ///
    /// \return A vector of addresses from your local neighbors.
    public: std::vector<std::string> Neighbors() const;

    /// \brief Register the current address. This will make a synchronous call
    /// to the broker to validate and register the address.
    /// \return True when the address is valid or false otherwise.
    private: bool Register();

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _msg The incoming message.
    private: void OnMessage(const msgs::Datagram &_msg);

    /// \brief Callback executed each time that a neighbor update is received.
    /// The updates are coming from the broker. The broker decides which are
    /// the robots inside the communication range of each other vehicle and
    /// notifies these updates.
    ///
    /// \param[in] _neighbors The list of neighbors.
    private: void OnNeighbors(const msgs::Neighbor_M &_neighbors);

    /// \def Callback_t
    /// \brief The callback specified by the user when new data is available.
    /// The callback contains four parameters:
    ///   _srcAddress: The source address of the agent sending the message.
    ///   _dstAddress: The destination address of the message.
    ///   _dstPort: The destination port of the message.
    ///   _data: The payload of the message.
    private: using Callback_t =
      std::function<void(const std::string &_srcAddress,
                         const std::string &_dstAddress,
                         const uint32_t _dstPort,
                         const std::string &_data)>;

    /// \brief The local address.
    private: const std::string localAddress;

    /// \brief Maximum transmission payload size (octets) for each message.
    private: static const uint32_t kMtu = 1500;

    /// \brief The current list of neighbors.
    private: std::vector<std::string> neighbors;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief User callbacks. The key is the topic name
    /// (e.g.: "/subt/192.168.2.1/4000") and the value is the user callback.
    private: std::map<std::string, Callback_t> callbacks;

    /// \brief True when the broker validated my address. Enabled must be true
    /// for being able to send and receive data.
    private: bool enabled = false;

    /// \brief When true, the Ignition service will only be visible within
    /// this process.
    private: bool isPrivate = false;

    /// \brief A mutex for avoiding race conditions.
    private: mutable std::mutex mutex;
  };
}
#endif
