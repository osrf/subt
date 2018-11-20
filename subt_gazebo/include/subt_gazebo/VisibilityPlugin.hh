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
  /// \brief This plugin generates a visibility lookup table with the
  /// following contents:
  ///
  /// int max_y_value
  /// int step_size
  /// int row_size
  /// uint64_t keys
  ///
  /// Data is stored in binary, and keys is a list of unique uint64_t values
  /// that represent two coordinates that *do not* have visiblity. It is
  /// assumed that any two coordiantes separated by more than 250m are not
  /// visible.
  ///
  /// Keys are generated using a combination of an Index and Pair function.
  ///
  /// We assume a terrain, without any other objects, is used to generate
  /// the visibility lookup table.
  ///
  /// Example usage:
  ///   gzserver -s libVisibilityPlugin.so --iters 1 worlds/swarm_vis.world
  ///
  /// The visibility table will be located at /tmp/visibility.dat
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
    private: void OnWorldCreated();

    /// \brief The world created connection.
    private: event::ConnectionPtr worldCreatedConn;
  };
}
#endif
