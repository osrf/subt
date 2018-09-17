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

/// \file Broker.hh
/// \brief Broker for handling message delivery among robots.

#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/CommonTypes.hh"

#ifndef SUBT_GAZEBO_BROKER_HH_
#define SUBT_GAZEBO_BROKER_HH_

namespace subt
{
  /// \brief Stores information about a client broker.
  class BrokerClientInfo
  {
    /// \brief Address of the client. E.g.: 192.168.2.2
    public: std::string address;
  };

  /// \def EndPoints_M
  /// \brief Map of endpoints
  using EndPoints_M = std::map<std::string, std::vector<BrokerClientInfo>>;

  /// \brief Store messages, and exposes an API for registering new clients,
  /// bind to a particular address, push new messages or get the list of
  /// messages already stored in the queue.
  class Broker
  {
    /// \brief Constructor.
    public: Broker();

    /// \brief Destructor.
    public: virtual ~Broker() = default;

    /// \brief Handle reset.
    public: void Reset();

    /// \brief Get a mutator to the team.
    /// \return A mutator to the team.
    public: subt::TeamMembershipPtr Team();

    /// \brief Send a message to each member
    /// with its updated neighbors list.
    public: void NotifyNeighbors();

    /// \brief Dispatch all incoming messages.
    /// \param[in] _maxDataRatePerCycle
    /// \param[in] _udpOverhead
    public: void DispatchMessages(const uint32_t _maxDataRatePerCycle,
                                  const uint16_t _udpOverhead);

    /// \brief Get the mutex to lock from the outside of this class.
    /// \return The mutex used in this class.
    public: std::mutex &Mutex();

    /// \brief This method associates an endpoint with a broker client and its
    /// address. An endpoint is constructed as an address followed by ':',
    /// followed by the port. E.g.: "192.168.1.5:8000" is a valid endpoint.
    /// \param[in] _clientAddress Address of the broker client.
    /// \param[in] _endpoint End point requested to bind.
    /// \return True if the operation succeed or false otherwise (if the client
    /// was already bound to the same endpoint).
    private: bool Bind(const std::string &_clientAddress,
                       const std::string &_endpoint);

    /// \brief Register a new client for message handling.
    /// \param[in] _id Unique ID of the client.
    /// \return True if the operation succeed of false otherwise (if the same
    /// id was already registered).
    public: bool Register(const std::string &_id);

    /// \brief Unregister a client and unbind from all the endpoints.
    /// \param[in] _id Unique ID of the client.
    /// \return True if the operation succeed or false otherwise (if there is
    /// no client registered for this ID).
    public: bool Unregister(const std::string &_id);

    /// \brief Callback executed when a new registration request is received.
    /// \param _req The address contained in the request.
    private: bool OnAddrRegistration(const ignition::msgs::StringMsg &_req,
                                     ignition::msgs::Boolean &_rep);

     /// \brief Callback executed when a new registration request is received.
    /// \param _req The address contained in the request.
    private: bool OnEndPointRegistration(
                                        const ignition::msgs::StringMsg_V &_req,
                                        ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new request is received.
    /// \param _req The datagram contained in the request.
    private: void OnMessage(const subt::msgs::Datagram &_req);

    /// \brief Queue to store the incoming messages received from the clients.
    protected: std::deque<msgs::Datagram> incomingMsgs;

    /// \brief List of bound endpoints. The key is an endpoint and the
    /// value is the vector of clients bounded on that endpoint.
    protected: EndPoints_M endpoints;

    /// \brief Information about the members of the team.
    protected: subt::TeamMembershipPtr team;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief The publisher for notifying neighbor updates.
    private: ignition::transport::Node::Publisher neighborPub;

    /// \brief Random engine used to shuffle the messages.
    private: std::default_random_engine rndEngine;

    /// \brief Protect data from races.
    private: std::mutex mutex;
  };
}  // namespace
#endif
