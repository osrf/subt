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
#ifndef SUBT_IGN_VISIBILITYPLUGIN_HH_
#define SUBT_IGN_VISIBILITYPLUGIN_HH_

#include <ignition/plugin/Register.hh>
#include <ignition/tools/launch/Plugin.hh>

namespace subt
{
  class VisibilityPluginPrivate;
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
  ///   ign launch tunnel_circuit_practice.ign \
  /// worldName:=tunnel_circuit_practice_01
  ///
  /// The visibility table (<WORLD_NAME>.dat) will be located in the same
  /// directory where the world file was located.
  class VisibilityPlugin : public ignition::tools::launch::Plugin
  {
    /// \brief Destructor
    public: virtual ~VisibilityPlugin();

    // Documentation inherited
    public: virtual void Load(const tinyxml2::XMLElement *_elem) override final;

    private: std::unique_ptr<VisibilityPluginPrivate> dataPtr;
  };
}

// Register the plugin
IGNITION_ADD_PLUGIN(subt::VisibilityPlugin, ignition::tools::launch::Plugin)
#endif
