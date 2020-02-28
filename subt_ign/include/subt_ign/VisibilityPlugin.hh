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

#include <memory>
#include <ignition/gazebo/System.hh>

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
  ///   ign launch -v 4 visibility.launch worldName:=tunnel_practice_1
  ///
  /// The visibility table (<WORLD_NAME>.dat) will be located in the same
  /// directory where the world file was located.
  class VisibilityPlugin :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: VisibilityPlugin();

    /// \brief Destructor
    public: ~VisibilityPlugin();

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<VisibilityPluginPrivate> dataPtr;
  };
}

#endif
