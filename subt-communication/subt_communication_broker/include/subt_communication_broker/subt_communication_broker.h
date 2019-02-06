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

/// \file subt_communication_broker.h
/// \brief Broker for handling message delivery among robots.
#pragma once

#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_communication_model/subt_communication_model.h>
#include <subt_communication_broker/common_types.h>

#include <subt_communication_broker/protobuf/datagram.pb.h>
#include <subt_communication_broker/protobuf/neighbor_m.pb.h>

namespace subt
{
namespace communication_broker
{

typedef
std::function<std::tuple<bool, geometry_msgs::PoseStamped>(const std::string& name)>
pose_update_function;

  /// \brief Stores information about a client broker.
  struct BrokerClientInfo
  {
    /// \brief Address of the client. E.g.: 192.168.2.2
    std::string address;
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

    /// \brief Start handling services
    ///
    /// This function allows us to wait to advertise capabilities to
    /// clients until the broker has been entirely initialized. I.e.,
    /// after SetDefaultRadioConfiguration() has been called.
    public: void Start();

    /// \brief Handle reset.
    public: void Reset();

    /// \brief Get a mutator to the team.
    /// \return A mutator to the team.
    public: TeamMembershipPtr Team();

    /// \brief Send a message to each member
    /// with its updated neighbors list.
    public: void NotifyNeighbors();

    /// \brief Dispatch all incoming messages.
   public: void DispatchMessages();

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

    /// \brief Set the radio configuration for address
    /// \param[in] address 
    /// \param[in] radio_configuration
    public: void SetRadioConfiguration(const std::string& address,
       communication_model::radio_configuration config);

    /// \brief Set the radio configuration
    /// \param[in] radio_configuration
    public: void SetDefaultRadioConfiguration(
        communication_model::radio_configuration config);

    /// \brief Set the communication function handle to use
    /// \param[in] f Function that evaluates transmission of bytes across the
    ///            communication channel.
    public: void SetCommunicationFunction(
        communication_model::communication_function f);

    /// \brief Set the function to be used for updating pose
    /// \param[in] f Function that finds pose based on name
    public: void SetPoseUpdateFunction(pose_update_function f);

    /// \brief Callback executed when a new registration request is received.
    /// \param[in] _req The address contained in the request.
    /// \param[out] _rep The result of the service. True when the registration
    /// went OK or false otherwise (e.g.: the same address was already
    /// registered).
    private: bool OnAddrRegistration(const ignition::msgs::StringMsg &_req,
                                     ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new unregistration request is received.
    /// \param[in] _req The address contained in the request.
    /// \param[out] _rep The result of the service. True when the unregistration
    /// went OK or false otherwise (e.g.: the address wasn't registered).
    private: bool OnAddrUnregistration(const ignition::msgs::StringMsg &_req,
                                       ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new registration request is received.
    /// \param[in] _req The end point contained in the request. The first
    /// string is the client address and the second string is the end point.
    /// \param[out] _rep The result of the service. True when the registration
    /// went OK of false otherwise (e.g.: _req doesn't contain two strings).
    private: bool OnEndPointRegistration(
                                        const ignition::msgs::StringMsg_V &_req,
                                        ignition::msgs::Boolean &_rep);

    /// \brief Callback executed when a new request is received.
    /// \param[in] _req The datagram contained in the request.
    private: void OnMessage(const subt::msgs::Datagram &_req);

    /// \brief Queue to store the incoming messages received from the clients.
    protected: std::deque<msgs::Datagram> incomingMsgs;

    /// \brief List of bound endpoints. The key is an endpoint and the
    /// value is the vector of clients bounded on that endpoint.
    protected: EndPoints_M endpoints;

    /// \brief Information about the members of the team.
    protected: TeamMembershipPtr team;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief The publisher for notifying neighbor updates.
    private: ignition::transport::Node::Publisher neighborPub;

    /// \brief Random engine used to shuffle the messages.
    // private: std::default_random_engine rndEngine;

    /// \brief Protect data from races.
    private: std::mutex mutex;

    /// \brief Function handle for evaluating communication
    private: subt::communication_model::communication_function
    communication_function;

   private:
    /// \brief Default radio configuration
    subt::communication_model::radio_configuration default_radio_configuration;

    /// \brief Pose update function
   private: pose_update_function pose_update_f;
  };

}
}
