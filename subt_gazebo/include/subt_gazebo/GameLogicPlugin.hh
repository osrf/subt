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
#ifndef SUBT_GAZEBO_GAMELOGICPLUGIN_HH_
#define SUBT_GAZEBO_GAMELOGICPLUGIN_HH_

#include <chrono>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief A plugin that takes care of all the SubT challenge logic.
  class GameLogicPlugin : public WorldPlugin
  {
    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Callback triggered when the start gate is crossed.
    /// \param[in] _msg The message containing if the gate was crossed or left.
    private: void OnStart(const ignition::msgs::Boolean &_msg);

    /// \brief Callback triggered when the finish gate is crossed.
    /// \param[in] _msg The message containing if the gate was crossed or left.
    private: void OnFinish(const ignition::msgs::Boolean &_msg);

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Ignition Transport node.
    private: ignition::transport::Node node;

    /// \brief Whether the task has started.
    private: bool started = false;

    /// \brief Whether the task has finished.
    private: bool finished = false;

    /// \brief Start time used for scoring.
    private: std::chrono::steady_clock::time_point startTime;

    /// \brief Finish time used for scoring.
    private: std::chrono::steady_clock::time_point finishTime;
  };
}
#endif
