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

#ifndef SUBT_GAZEBO_BASESTATIONPLUGIN_HH_
#define SUBT_GAZEBO_BASESTATIONPLUGIN_HH_

#include <memory>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>
#include "subt_gazebo/CommsClient.hh"

namespace subt
{
  /// \brief A plugin to receive artifact reports from the teams.
  class BaseStationPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: BaseStationPlugin();

    // Documentation inherited.
    public: void Load(gazebo::physics::ModelPtr _parent,
                      sdf::ElementPtr _sdf);

    /// \brief Callback for processing an artifact report.
    /// \param[in] _srcAddress Unused.
    /// \param[in] _dstAddress Unused.
    /// \param[in] _dstPort Unused.
    /// \param[in] _data Serialized artifact.
    public: void OnArtifact(const std::string &_srcAddress,
                            const std::string &_dstAddress,
                            const uint32_t _dstPort,
                            const std::string &_data);

    /// \brief An ignition transport node.
    private: ignition::transport::Node node;

    /// \brief SubT communication client.
    private: std::unique_ptr<subt::CommsClient> client;
  };
}
#endif
