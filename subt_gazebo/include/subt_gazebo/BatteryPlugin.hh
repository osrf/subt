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

#ifndef SUBT_GAZEBO_BATTERYPLUGIN_HH_
#define SUBT_GAZEBO_BATTERYPLUGIN_HH_

#include <memory>
#include <sdf/Element.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>

namespace subt_gazebo
{
  class BatteryPluginPrivate;

  /// \brief A plugin to track energy consumption of a model.
  class BatteryPlugin : public gazebo::ModelPlugin
  {
    /// \brief Constructor
    public: BatteryPlugin();

    /// \brief Plugin Load function
    /// \param[in] _parent Model pointer to the model defining this plugin
    /// \param[in] _sdf pointer to the SDF of the model
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback for world update events.
    public: void OnUpdate();

    /// \brief Private data pointer.
    private: std::unique_ptr<BatteryPluginPrivate> dataPtr;
  };
}
#endif
