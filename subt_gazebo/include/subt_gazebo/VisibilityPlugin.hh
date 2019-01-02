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
#ifndef SUBT_GAZEBO_VISIBILITYPLUGIN_HH_
#define SUBT_GAZEBO_VISIBILITYPLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief This plugin generates a vertex lookup table with the
  /// following contents:
  ///
  /// uint64_t num_entries
  /// ...
  /// int32_t sample_x
  /// int32_t sample_y
  /// int32_t sample_z
  /// uint64_t vertex_id
  /// ...
  ///
  /// Data is stored in binary, and keys is a list of three int32_t values
  /// that represent a 3D coordinate contained within the scenario. The next
  /// uint64_t value is the vertex Id of the graph that contains the given
  /// 3D point.
  ///
  /// Example usage:
  ///   roslaunch subt_gazebo visibility.launch scenario:=tunnel_practice_1
  ///
  /// The visibility table (<WORLD_NAME>.dat) will be located in the same
  /// directory where the world file was located.
  class GAZEBO_VISIBLE VisibilityPlugin : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~VisibilityPlugin();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief World created callback
    private: void OnUpdate();

    /// \brief The world created connection.
    private: event::ConnectionPtr worldUpdateConn;
  };
}
#endif
