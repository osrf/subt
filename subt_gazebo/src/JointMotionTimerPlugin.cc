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

#include <functional>
#include <subt_gazebo/JointMotionTimerPlugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>

namespace subt_gazebo
{
GZ_REGISTER_MODEL_PLUGIN(JointMotionTimerPlugin)

class JointMotionTimerPluginPrivate
{
  /// \brief Pointer to the model that defines this plugin
  public: gazebo::physics::ModelPtr model;

  /// \brief World update connection pointer.
  public: gazebo::event::ConnectionPtr updateConnection;
};

JointMotionTimerPlugin::JointMotionTimerPlugin()
  : dataPtr(new JointMotionTimerPluginPrivate)
{
}

void JointMotionTimerPlugin::Load(gazebo::physics::ModelPtr _parent,
                         sdf::ElementPtr /*_sdf*/)
{
  this->dataPtr->model = _parent;

  this->dataPtr->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointMotionTimerPlugin::OnUpdate, this));
}

void JointMotionTimerPlugin::OnUpdate()
{
}
}
