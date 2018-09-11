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
#include <string>
#include <vector>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/SwarmTypes.hh"

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

    /// \brief This method associates an endpoint with a broker client and its
    /// address. An endpoint is constructed as an address followed by ':',
    /// followed by the port. E.g.: "192.168.1.5:8000" is a valid endpoint.
    /// \param[in] _clientAddress Address of the broker client.
    /// \param[in] _endpoint End point requested to bind.
    /// \return True if the operation succeed or false otherwise (if the client
    /// was already bound to the same endpoint).
    public: bool Bind(const std::string &_clientAddress,
                      const std::string &_endpoint);

    /// \brief Queue a new message.
    /// \param[in] _msg A new message.
    public: void Push(const msgs::Datagram &_msg);

    /// \brief Register a new client for message handling.
    /// \param[in] _id Unique ID of the client.
    /// \return True if the operation succeed of false otherwise (if the same
    /// id was already registered).
    public: bool Register(const std::string &_id);

    /// \brief Get the list of endpoints bound.
    /// \return Map of endpoints. The key is the endpoint and the value is a
    /// vector containing the information of all the clients bound to this
    /// endpoint.
    /// \sa BrokerClientInfo.
    public: const EndPoints_M &EndPoints() const;

    /// \brief Get the current message queue.
    /// \return Reference to the queue of messages.
    public: std::deque<msgs::Datagram> &Messages();

    /// \brief Unregister a client and unbind from all the endpoints.
    /// \param[in] _id Unique ID of the client.
    /// \return True if the operation succeed or false otherwise (if there is
    /// no client registered for this ID).
    public: bool Unregister(const std::string &_id);

    /// \brief Handle reset.
    public: void Reset();

    /// \brief Get a mutator to the swarm.
    /// \return A mutator to the swarm.
    public: subt::SwarmMembershipPtr Swarm();

    /// \brief Send a message to each member
    /// with its updated neighbors list.
    public: void NotifyNeighbors();

    /// \brief Callback executed when a new registration request is received.
    /// \param _req The address contained in the request.
    private: bool OnRegistration(const ignition::msgs::StringMsg &_req,
                                 ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new request is received.
    /// \param _req The datagram contained in the request.
    private: void OnMessage(const subt::msgs::Datagram &_req);

    /// \brief Queue to store the incoming messages received from the clients.
    protected: std::deque<msgs::Datagram> incomingMsgs;

    /// \brief List of bound endpoints. The key is an endpoint and the
    /// value is the vector of clients bounded on that endpoint.
    protected: EndPoints_M endpoints;

    /// \brief Information about the members of the swarm.
    protected: subt::SwarmMembershipPtr swarm;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief The publisher for notifying neighbor updates.
    private: ignition::transport::Node::Publisher neighborPub;

    /// \brief Protect data from races.
    private: std::mutex mutex;
  };
}  // namespace
#endif
