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

/// \file CommsModel.hh
/// \brief Manages how the communication behave between the members of the team.

#ifndef SUBT_GAZEBO_COMMSMODEL_HH_
#define SUBT_GAZEBO_COMMSMODEL_HH_

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>
#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/VisibilityTable.hh"

namespace subt
{
  /// \brief Class used to store information about the communication model.
  class CommsModel
  {
    /// \brief Class constructor.
    ///
    /// \param[in] _team Pointer to the team.
    /// \param[in] _world Pointer to the Gazebo world.
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: CommsModel(TeamMembershipPtr _team,
                       gazebo::physics::WorldPtr _world,
                       sdf::ElementPtr _sdf);

    /// \brief Class destructor.
    public: virtual ~CommsModel() = default;

    /// \brief Update the state of the communication model (outages, visibility
    /// between nodes and neighbors).
    public: void Update();

    /// \brief Get the maximum data rate allowed (bits per second).
    /// \return Maximum data rate allowed (bps).
    public: uint32_t MaxDataRate() const;

    /// \brief Get the overhead caused by UDP+IP+Ethernet headers (bytes).
    /// \return The overhead in bytes.
    public: uint16_t UdpOverhead() const;

    /// \brief Query if the comms are set in simple mode or not.
    /// \return true when the comms are in simple mode or false otherwise.
    public: bool SimpleMode() const;

    /// \brief Enable/disable simple mode. In this mode, all messages sent will
    /// arrive to each destination.
    /// \param[in] _value True to enable simple mode or false otherwise.
    public: void SetSimpleMode(const bool _value);

    /// \brief Check if a "comms_model" block exists in the SDF element of the
    /// plugin. If so, update the value of the default parameters with the one
    /// read from the world file.
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    private: void LoadParameters(sdf::ElementPtr _sdf);

    /// \brief ToDo.
    private: void InitializeVisibility();

    /// \brief Populate a vector with the pairs of addresses that will be
    /// checked in UpdateVisibility() each iteration. Note that the vector will
    /// contain all combinations of two different elements (not permutations).
    private: void UpdateVisibilityPairs();

    /// \brief Decide if each member of the team enters into a comms outage.
    private: void UpdateOutages();

    /// \brief Update the visibility state between vehicles.
    private: void UpdateVisibility();

    /// \brief Update the neighbors list of each member of the team.
    private: void UpdateNeighbors();

    /// \brief Update the neighbor list for a single robot and notifies the
    /// robot with the updated list.
    ///
    /// \param[in] _address Address of the robot to be updated.
    private: void UpdateNeighborList(const std::string &_address);

    /// \brief Visualize the connectivity from a robot.
    /// \param[in] _req The name of a model. The connectivity is visualized
    /// relative to this robot.
    /// \param[out] _rep True when the visibility was correctly visualized or
    /// false otherwise.
    /// \return True if the service was executed.
    private: bool VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                                      ignition::msgs::Boolean &_rep);

    /// \brief Visibility between vehicles. The key is a pair with the
    /// addresses of the vehicles involved. The value is a vector of strings
    /// that stores the entity names of the first and last obstacles between the
    /// vehicles. If there is line of sight the value contains a vector of one
    /// element (empty string).
    private: std::map<std::pair<std::string, std::string>, bool> visibility;

    /// \brief Pairs of addresses used to update the visibility.
    private: std::vector<std::pair<std::string, std::string>> visibilityPairs;

    /// \brief Minimum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    private: double neighborDistanceMin = -1.0;

    /// \brief Maximum free-space distance (m) between two nodes to be
    /// neighbors. Set to <0 for no limit.
    private: double neighborDistanceMax = -1.0;

    /// \brief Minimum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors). Set to <0 for no limit.
    private: double commsDistanceMin = -1.0;

    /// \brief Maximum free-space distance (m) between two nodes to communicate
    /// (must also be neighbors). Set to <0 for no limit.
    private: double commsDistanceMax = -1.0;

    /// \brief Minimum probability of dropping a given packet.
    /// Used with uniform drop probability model.
    private: double commsDropProbabilityMin = 0.0;

    /// \brief Maximum probability of dropping a given packet.
    /// Used with uniform drop probability model.
    private: double commsDropProbabilityMax = 0.0;

    /// \brief Probability of going into a comms outage at each second.
    private: double commsOutageProbability = 0.0;

    /// \brief Minimum length of comms outage (secs). Set to <0 for no limit.
    /// Used with uniform outage duration probability model.
    private: double commsOutageDurationMin = -1.0;

    /// \brief Maximum length of comms outage (secs). Set to <0 for no limit.
    /// Used with uniform outage duration probability model.
    private: double commsOutageDurationMax = -1.0;

    /// \brief Maximum data rate allowed (bits per second).
    private: uint32_t commsDataRateMax = 54000000;

    /// \brief UDP header (8 bytes) + IPv4 header (20 bytes) +
    /// Ethernet 28 (bytes).
    private: uint16_t udpOverhead = 56;

    /// \brief Maximum cost between two sections of the world to communicate.
    /// This is the cost of the associated graph connecting all the tunnels of
    /// the world.
    private: double commsCostMax = 15.0;

    /// \brief Pointer to the team.
    private: TeamMembershipPtr team;

    /// \brief Pointer to the Gazebo world.
    private: gazebo::physics::WorldPtr world;

    /// \brief Keep track of update sim-time.
    private: gazebo::common::Time lastUpdateTime;

    /// Vector containing all the addresses of the team.
    private: std::vector<std::string> addresses;

    /// When simple mode is enabled, all messages will be delivered to the
    /// destinations.
    private: bool simpleMode = false;

    /// \brief Visibility table.
    private: subt::VisibilityTable visibilityTable;

    /// \brief An ignition transport node.
    private: ignition::transport::Node node;
  };
}  // namespace
#endif
