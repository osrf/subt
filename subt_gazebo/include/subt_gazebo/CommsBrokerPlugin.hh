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
#ifndef SUBT_GAZEBO_COMMSBROKERPLUGIN_HH_
#define SUBT_GAZEBO_COMMSBROKERPLUGIN_HH_

#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>
#include "subt_gazebo/Broker.hh"
#include "subt_gazebo/CommsModel.hh"
#include "subt_gazebo/SwarmTypes.hh"

namespace subt
{
  namespace msgs
  {
    // Forward declarations.
    class Datagram;
  }
}

namespace gazebo
{
  /// \brief A plugin that centralizes all SubT robot-to-robot communication.
  class CommsBrokerPlugin : public WorldPlugin
  {
    /// \brief Class constructor.
    public: CommsBrokerPlugin() = default;

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Send a message to each swarm member
    /// with its updated neighbors list.
    private: void NotifyNeighbors();

    /// \brief Dispatch all incoming messages.
    private: void DispatchMessages();

    /// \brief Callback executed when a new request is received.
    /// \param _req The datagram contained in the request.
    private: void OnMessage(const subt::msgs::Datagram &_req);

    /// \brief Callback executed when a new registration request is received.
    /// \param _req The address contained in the request.
    private: bool OnRegistration(const ignition::msgs::StringMsg &_req,
                                 ignition::msgs::Boolean &_rep);

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Information about the members of the swarm.
    private: subt::SwarmMembershipPtr swarm;

    /// \brief Protect data from races.
    private: std::mutex mutex;

    /// \brief Comms model that we're using.
    private: std::unique_ptr<subt::CommsModel> commsModel;

    /// \brief Broker instance.
    private: subt::Broker broker;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief Maximum data rate allowed per simulation cycle (bits).
    private: uint32_t maxDataRatePerCycle;

    /// \brief Random engine used to shuffle the messages.
    private: std::default_random_engine rndEngine;

    /// \brief Vector of registered addresses.
    private: std::vector<std::string> addresses;
  };
}
#endif
