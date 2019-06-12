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
#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_communication_model/subt_communication_model.h>

namespace subt
{
namespace communication_broker
{

/// \def Neighbors_M
/// \brief Map of neighbors
/// \todo Fix second term to hold channel information (RSS, bitrate)
using Neighbors_M = std::map<std::string, double>;

/// \brief Structure used to store information about a member of the
/// team.
struct TeamMember
{
  /// \brief Name used for this agent
  std::string name;

  /// \brief Address of this agent. E.g.: 192.168.1.2
  std::string address;

  /// \brief List of neighbors and comms probabilities for this robot.
  Neighbors_M neighbors;

  /// \brief Static configuration of radio for communication
  subt::communication_model::radio_configuration radio;

  /// \brief State of the radio, e.g., pose
  subt::rf_interface::radio_state rf_state;
};

/// \def TeamMemberPtr
/// \brief Shared pointer to TeamMember
using TeamMemberPtr = std::shared_ptr<TeamMember>;

/// \def TeamMembership_M
/// \brief Map containing information about the members of the team.
/// The key is the robot address. The value is a pointer to a TeamMember
/// object that contains multiple information about the robot.
using TeamMembership_M = std::map<std::string, TeamMemberPtr>;

/// \def TeamMembershipPtr
/// \brief A shared pointer to the membership data structure.
/// \sa TeamMembership_M
using TeamMembershipPtr = std::shared_ptr<TeamMembership_M>;

/// \brief Address used to send a message to all the members of the team
/// listening on a specific port.
const std::string kBroadcast = "broadcast";

/// \brief Address used to bind to a multicast group. Note that we do not
/// support multiple multicast groups, only one.
const std::string kMulticast = "multicast";

/// \brief Service used to centralize all messages sent from the agents.
const std::string kBrokerSrv = "broker";

/// \brief Service used to validate an address.
const std::string kAddrRegistrationSrv = "address/register";

/// \brief Service used to invalidate an address.
const std::string kAddrUnregistrationSrv = "address/unregister";

/// \brief Service used to register an end point.
const std::string kEndPointRegistrationSrv = "end_point/register";

/// \brief Address used to receive neighbor updates.
const std::string kNeighborsTopic = "neighbors";

/// \brief Default port.
const uint32_t kDefaultPort = 4100u;

}
}
