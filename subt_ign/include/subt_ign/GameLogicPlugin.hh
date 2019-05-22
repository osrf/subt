/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef SUBT_IGN_GAMELOGICPLUGIN_HH_
#define SUBT_IGN_GAMELOGICPLUGIN_HH_

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <ignition/plugin/Register.hh>
#include <ignition/launch/Plugin.hh>

namespace subt
{
  class GameLogicPluginPrivate;

  /// \brief A plugin that takes care of all the SubT challenge logic.
  class GameLogicPlugin : public ignition::launch::Plugin
  {
    /// \brief Constructor
    public: GameLogicPlugin();

    /// \brief Destructor
    public: ~GameLogicPlugin();

    // Documentation inherited
    public: virtual bool Load(const tinyxml2::XMLElement *_elem) override final;

    /// \brief Callback triggered when a pair of links collide. It starts the
    /// timer if a specified start area is collided by some object.
    /// \param[in] _msg The message containing a list of collision information.
    // \todo(nkoenig) Waiting on contact plugin.
    // private: void OnStartCollision(ConstIntPtr &_msg);

    private: std::unique_ptr<GameLogicPluginPrivate> dataPtr;
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(subt::GameLogicPlugin, ignition::launch::Plugin)
#endif
