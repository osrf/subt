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
#include <random>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>
#include "subt_gazebo/Broker.hh"
#include "subt_gazebo/CommsModel.hh"

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

    /// \brief Dispatch all incoming messages.
    private: void DispatchMessages();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Protect data from races.
    private: std::mutex mutex;

    /// \brief Comms model that we're using.
    private: std::unique_ptr<subt::CommsModel> commsModel;

    /// \brief Broker instance.
    private: subt::Broker broker;

    /// \brief Maximum data rate allowed per simulation cycle (bits).
    private: uint32_t maxDataRatePerCycle;

    /// \brief Random engine used to shuffle the messages.
    private: std::default_random_engine rndEngine;
  };
}
#endif
