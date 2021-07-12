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
#include <iostream>
#include <memory>
#include <ignition/math/Rand.hh>

#include <subt_communication_broker/common_types.h>
#include <subt_communication_broker/protobuf/datagram.pb.h>
#include <subt_communication_broker/protobuf/neighbor_m.pb.h>
#include <subt_communication_broker/subt_communication_broker.h>

namespace subt
{
namespace communication_broker
{

/// \brief Helper class for managing bidirectional mapping of client IDs and
/// addresses. The class is not thread-safe, so callers must ensure that none
/// of the public methods get called simultaneously.
struct ClientIDs
{
  /// \brief Number of active clients for each address. This structure can be
  /// accessed by outer code for reading, but not for modification.
  std::unordered_map<std::string, size_t> numActiveClients;

  /// \brief Map of client IDs to addresses. This structure can be accessed by
  /// outer code for reading, but not for modification.
  std::unordered_map<ClientID, std::string> idToAddress;

  /// \brief Add a new client and generate its ID.
  /// \param _address Address of the client.
  /// \return ID of the client. This method should always succeed and return
  /// an ID different from invalidClientID.
  ClientID Add(const std::string& _address)
  {
    const auto clientId = this->NextID();
    this->idToAddress[clientId] = _address;
    if (this->numActiveClients.find(_address) == this->numActiveClients.end())
      this->numActiveClients[_address] = 0;
    this->numActiveClients[_address]++;
    return clientId;
  }

  /// \brief Unregister a client.
  /// \param _id ID of the client.
  /// \return Success of the unregistration. The method can fail e.g. when
  /// trying to unregister a client which has not been registered.
  bool Remove(const ClientID _id)
  {
    if (!this->Valid(_id))
      return false;
    this->numActiveClients[this->idToAddress[_id]]--;
    this->idToAddress.erase(_id);
    return true;
  }

  /// \brief Clear/reset the structure to be able to work as new.
  /// \note This cancels all registrations of all clients and resets the client
  /// ID numbering, so it is not valid to mix IDs of clients obtained before and
  /// after a Clear() call.
  void Clear()
  {
    this->numActiveClients.clear();
    this->idToAddress.clear();
    this->lastId = invalidClientId;
  }

  /// \brief Check validity of a client ID.
  /// \param _id ID to check.
  /// \return Whether a client with the given ID has been registered.
  bool Valid(const ClientID _id) const
  {
    return _id != invalidClientId &&
      this->idToAddress.find(_id) != this->idToAddress.end();
  }

  /// \brief Return an ID for a new client.
  /// \return The ID.
  private: ClientID NextID()
  {
    return ++this->lastId;
  }

  /// \brief Last ID given to a client.
  private: ClientID lastId {invalidClientId};
};

/// \brief Helper class for managing mappings between endpoint names, their IDs,
/// related clients and so on. The class is not thread-safe, so callers must
/// ensure that none of the public methods get called simultaneously.
struct EndpointIDs
{
  /// \brief Maps endpoint IDs to endpoint names. This structure can be accessed
  /// by outer code for reading, but not for modification.
  std::unordered_map<EndpointID, std::string> idToEndpoint;

  /// \brief Maps endpoint names to related clients. Each client ID stores
  /// information about the number of "connections" of this client ID to the
  /// endpoint. This structure can be accessed by outer code for reading, but
  /// not for modification.
  std::unordered_map<std::string, std::unordered_map<ClientID, size_t>>
  endpointToClientIds;

  /// \brief Maps endpoint IDs to their related client IDs. This structure can
  /// be accessed by outer code for reading, but not for modification.
  std::unordered_map<EndpointID, ClientID> endpointIdToClientId;

  /// \brief Maps client IDs to all their endpoint IDs. This structure can be
  /// accessed by outer code for reading, but not for modification.
  std::unordered_map<ClientID, std::unordered_set<EndpointID>>
  clientIdToEndpointIds;

  /// \brief Add new endpoint related to the given client ID.
  /// \param _endpoint Endpoint name.
  /// \param _clientId Client ID. This method does no validation of the IDs.
  /// \return ID of the new endpoint. Should always succeed and return an ID
  /// not equal to invalidEndpointID.
  EndpointID Add(const std::string& _endpoint, const ClientID _clientId)
  {
    const auto endpointId = this->NextID();

    this->idToEndpoint[endpointId] = _endpoint;
    this->endpointIdToClientId[endpointId] = _clientId;

    if (this->endpointToClientIds[_endpoint].find(_clientId) ==
      this->endpointToClientIds[_endpoint].end())
        this->endpointToClientIds[_endpoint][_clientId] = 0;
    this->endpointToClientIds[_endpoint][_clientId]++;

    this->clientIdToEndpointIds[_clientId].insert(endpointId);

    return endpointId;
  }

  /// \brief Remove the given endpoint ID from this structure.
  /// \param _id ID of the endpoint to remove.
  /// \return Whether the removal succeeded or not. It may fail e.g. when the
  /// removed endpoint doesn't exist.
  bool Remove(const EndpointID _id)
  {
    if (!this->Valid(_id))
      return false;

    const auto endpointName = this->idToEndpoint[_id];

    if (this->endpointIdToClientId.find(_id) ==
      this->endpointIdToClientId.end())
        return false;

    const auto clientId = this->endpointIdToClientId[_id];
    this->endpointIdToClientId.erase(_id);

    if (this->endpointToClientIds.find(endpointName) ==
      this->endpointToClientIds.end())
        return false;

    if (this->endpointToClientIds[endpointName].find(clientId) ==
      this->endpointToClientIds[endpointName].end())
        return false;

    this->endpointToClientIds[endpointName][clientId]--;
    if (this->endpointToClientIds[endpointName][clientId] == 0u)
      this->endpointToClientIds[endpointName].erase(clientId);

    if (this->clientIdToEndpointIds.find(clientId) ==
      this->clientIdToEndpointIds.end())
        return false;

    if (this->clientIdToEndpointIds[clientId].find(_id) ==
      this->clientIdToEndpointIds[clientId].end())
        return false;

    this->clientIdToEndpointIds[clientId].erase(_id);

    return true;
  }

  /// \brief Clear/reset the structure to be able to work as new.
  /// \note This cancels all registrations of all endpoints and resets the
  /// endpoint ID numbering, so it is not valid to mix IDs of endpoints obtained
  /// before and after a Clear() call.
  void Clear()
  {
    this->idToEndpoint.clear();
    this->clientIdToEndpointIds.clear();
    this->endpointToClientIds.clear();
    this->endpointIdToClientId.clear();
    this->lastId = invalidEndpointId;
  }

  /// \brief Check validity of an endpoint ID.
  /// \param _id ID to check.
  /// \return Whether an endpoint with the given ID has been registered.
  bool Valid(const EndpointID _id) const
  {
    return _id != invalidEndpointId &&
      this->idToEndpoint.find(_id) != this->idToEndpoint.end();
  }

  /// \brief Return an ID for a new client.
  /// \return The ID.
  private: EndpointID NextID()
  {
    return ++this->lastId;
  }

  /// \brief Last ID given to a client.
  private: EndpointID lastId {invalidEndpointId};
};

/// \brief PIMPL structure.
struct BrokerPrivate
{
  /// \brief IDs of registered clients.
  ClientIDs clientIDs;

  /// \brief IDs of registered endpoints.
  EndpointIDs endpointIDs;
};

//////////////////////////////////////////////////
Broker::Broker()
    : team(std::make_shared<TeamMembership_M>()),
      dataPtr(std::make_unique<BrokerPrivate>())
{
}

//////////////////////////////////////////////////
Broker::~Broker()
{
  // cannot use default destructor because of dataPtr
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

  // Advertise the service for unregistering end points.
  if (!this->node.Advertise(kEndPointUnregistrationSrv,
                            &Broker::OnEndPointUnregistration, this))
  {
    std::cerr << "Error advertising srv [" << kEndPointUnregistrationSrv << "]"
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

  std::cout << "Started communication broker in Ignition partition "
            << this->IgnPartition() << std::endl;
}

//////////////////////////////////////////////////
void Broker::Reset()
{
  std::lock_guard<std::mutex> lk(this->mutex);
  this->incomingMsgs.clear();
  this->endpoints.clear();
  this->team->clear();
  this->dataPtr->clientIDs.Clear();
  this->dataPtr->endpointIDs.Clear();
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
bool Broker::DispatchMessages()
{
  std::lock_guard<std::mutex> lk(this->mutex);

  if(this->incomingMsgs.empty())
    return true;

  // Cannot dispatch messages if we don't have function handles for
  // pathloss and communication
  if (!communication_function)
  {
    std::cerr << "[Broker::DispatchMessages()] Missing function handle for "
      << "communication" << std::endl;
    return false;
  }

  if(!pose_update_f)
  {
    std::cerr << "[Broker::DispatchMessages()]: Missing function for updating "
      << "pose" << std::endl;
    return false;
  }

  // Update state for all members in team (only do this for members
  // which touch messages in the queue?)
  for(auto t : *(this->team))
  {
    bool ret;
    std::tie(ret,
             t.second->rf_state.pose,
             t.second->rf_state.update_stamp) = pose_update_f(t.second->name);

    if (!ret)
    {
      std::cerr << "Problem getting state for " << t.second->name
                << ", skipping DispatchMessages()" << std::endl;
      return false;
    }
  }

  bool allSucceeded = true;
  while (!this->incomingMsgs.empty())
  {
    // Get the next message to dispatch.
    subt::msgs::Datagram msg = this->incomingMsgs.front();
    this->incomingMsgs.pop_front();

    // Sanity check: Make sure that the sender is a member of the team.
    auto txNode = this->team->find(msg.src_address());
    if (txNode == this->team->end())
    {
      std::cerr << "Broker::DispatchMessages(): Discarding message. Robot ["
                << msg.src_address() << "] is not registered as a member of the"
                << " team" << std::endl;
      allSucceeded = false;
      continue;
    }

    std::string dstEndPoint =
        msg.dst_address() + ":" + std::to_string(msg.dst_port());

    if (this->endpoints.find(dstEndPoint) != this->endpoints.end())
    {
      std::vector<BrokerClientInfo> clientsV = this->endpoints.at(dstEndPoint);

      if (clientsV.empty())
      {
        std::cerr << "[Broker::DispatchMessages()]: No clients for endpoint "
          << dstEndPoint << std::endl;
      }

      for (const BrokerClientInfo &client : clientsV)
      {
        auto rxNode = this->team->find(client.address);
        if (rxNode == this->team->end())
        {
          std::cerr << "Broker::DispatchMessages(): Skipping send attempt."
            << "Robot [" << client.address
            << "] is not registered as a member of the"
            << " team" << std::endl;
          allSucceeded = false;
          continue;
        }

        // Query communication_model if this packet is successful,
        // forward if so
        if (!txNode->second->radio.pathloss_f)
        {
          std::cerr << "No pathloss function defined for "
                    << msg.src_address() << std::endl;
          allSucceeded = false;
          continue;
        }

        bool sendPacket;
        double rssi;
        std::tie(sendPacket, rssi) =
          communication_function(txNode->second->radio,
                                 txNode->second->rf_state,
                                 rxNode->second->rf_state,
                                 msg.data().size());

        if (sendPacket)
        {
          msg.set_rssi(rssi);

          if (!this->node.Request(client.address, msg))
          {
            std::cerr << "[CommsBrokerPlugin::DispatchMessages()]: Error "
                      << "sending message to [" << client.address << "]"
                      << std::endl;
            allSucceeded = false;
          }
        }
      }
    }
    else
    {
      std::cerr << "[Broker::DispatchMessages()]: Could not find endpoint "
        << dstEndPoint << std::endl;
      allSucceeded = false;
    }
  }
  return allSucceeded;
}

//////////////////////////////////////////////////
EndpointID Broker::Bind(const ClientID _clientId, const std::string &_endpoint)
{
  std::lock_guard<std::mutex> lk(this->mutex);

  if (!this->dataPtr->clientIDs.Valid(_clientId))
  {
    std::cerr << "Broker::Bind() error: Client ID [" << _clientId
              << "] is invalid." << std::endl;
    return invalidEndpointId;
  }

  const auto& clientAddress = this->dataPtr->clientIDs.idToAddress[_clientId];

  auto clientFound = false;

  if (this->endpoints.find(_endpoint) != this->endpoints.end())
  {
    const auto &clientsV = this->endpoints[_endpoint];
    for (const auto &client : clientsV)
    {
      if (client.address == clientAddress)
      {
        clientFound = true;
        break;
      }
    }
  }

  if (!clientFound)
  {
    BrokerClientInfo clientInfo;
    clientInfo.address = clientAddress;
    this->endpoints[_endpoint].push_back(clientInfo);
  }

  const auto endpointId = this->dataPtr->endpointIDs.Add(_endpoint, _clientId);

  return endpointId;
}

//////////////////////////////////////////////////
bool Broker::Unbind(EndpointID _endpointId)
{
  std::lock_guard<std::mutex> lk(this->mutex);

  if (!this->dataPtr->endpointIDs.Valid(_endpointId))
    return false;

  const auto endpointName =
    this->dataPtr->endpointIDs.idToEndpoint[_endpointId];

  const auto clientId =
    this->dataPtr->endpointIDs.endpointIdToClientId[_endpointId];
  const auto clientAddress = this->dataPtr->clientIDs.idToAddress[clientId];

  if (this->endpoints.find(endpointName) == this->endpoints.end())
    return false;

  bool success = this->dataPtr->endpointIDs.Remove(_endpointId);
  if (!success)
    return false;

  if (this->dataPtr->endpointIDs.endpointToClientIds.find(endpointName) ==
    this->dataPtr->endpointIDs.endpointToClientIds.end())
      return false;

  bool hasOtherClientsOnTheSameAddress = false;
  for (const auto clientKV :
    this->dataPtr->endpointIDs.endpointToClientIds[endpointName])
  {
    if (this->dataPtr->clientIDs.idToAddress[clientKV.first] == clientAddress &&
      clientKV.second > 0u)
    {
      hasOtherClientsOnTheSameAddress = true;
      break;
    }
  }

  if (hasOtherClientsOnTheSameAddress)
    return true;

  auto& clientsV = this->endpoints[endpointName];

  auto i = std::begin(clientsV);
  while (i != std::end(clientsV))
  {
    if (i->address == clientAddress)
    {
      clientsV.erase(i);
      break;
    }
    else
    {
      ++i;
    }
  }

  return true;
}

//////////////////////////////////////////////////
ClientID Broker::Register(const std::string &_clientAddress)
{
  std::lock_guard<std::mutex> lk(this->mutex);
  auto kvp = this->team->find(_clientAddress);
  if (kvp == this->team->end())
  {
    auto newMember = std::make_shared<TeamMember>();

    // Name and address are the same in SubT.
    newMember->address = _clientAddress;
    newMember->name = _clientAddress;

    newMember->radio = default_radio_configuration;
    (*this->team)[_clientAddress] = newMember;
  }

  const auto clientId = this->dataPtr->clientIDs.Add(_clientAddress);

  return clientId;
}

//////////////////////////////////////////////////
bool Broker::Unregister(const ClientID _clientId)
{
  if (!this->dataPtr->clientIDs.Valid(_clientId))
  {
    std::cerr << "Broker::Unregister() error: Client ID [" << _clientId
              << "] is invalid." << std::endl;
    return false;
  }

  bool success = true;

  std::unordered_set<subt::communication_broker::EndpointID> endpointIds;
  {
    // make a copy because Unbind() calls will alter the structure
    std::lock_guard<std::mutex> lk(this->mutex);
    endpointIds = this->dataPtr->endpointIDs.clientIdToEndpointIds[_clientId];
  }

  for (const auto endpointId : endpointIds)
    success = success && this->Unbind(endpointId);

  {
    std::lock_guard<std::mutex> lk(this->mutex);

    const auto& clientAddress = this->dataPtr->clientIDs.idToAddress[_clientId];
    success = success && this->dataPtr->clientIDs.Remove(_clientId);

    if (this->dataPtr->clientIDs.numActiveClients[clientAddress] == 0u)
      this->team->erase(clientAddress);
  }

  return success;
}

/////////////////////////////////////////////////
bool Broker::OnAddrRegistration(const ignition::msgs::StringMsg &_req,
                                ignition::msgs::UInt32 &_rep)
{
  const auto& address = _req.data();

  const ClientID result = this->Register(address);

  _rep.set_data(result);

  return result != invalidClientId;
}

/////////////////////////////////////////////////
bool Broker::OnAddrUnregistration(const ignition::msgs::UInt32 &_req,
                                  ignition::msgs::Boolean &_rep)
{
  uint32_t clientId = _req.data();

  bool result;

  result = this->Unregister(clientId);

  _rep.set_data(result);

  return result;
}

/////////////////////////////////////////////////
bool Broker::OnEndPointRegistration(
  const subt::msgs::EndpointRegistration &_req,
  ignition::msgs::UInt32 &_rep)
{
  ClientID clientId = _req.client_id();
  const auto& endpoint = _req.endpoint();

  EndpointID result = this->Bind(clientId, endpoint);

  _rep.set_data(result);

  return result != invalidEndpointId;
}

/////////////////////////////////////////////////
bool Broker::OnEndPointUnregistration(
  const ignition::msgs::UInt32 &_req,
  ignition::msgs::Boolean &_rep)
{
  const EndpointID endpointId = _req.data();

  bool result = this->Unbind(endpointId);

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

//////////////////////////////////////////////////
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
    if(node == this->team->end())
    {
      std::cerr << "Cannot set radio configuration for "
        << address << std::endl;
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

//////////////////////////////////////////////////
const std::string& Broker::IgnPartition() const
{
  return this->node.Options().Partition();
}

}
}
