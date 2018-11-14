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
#include <memory>
#include <vector>
#include <boost/weak_ptr.hpp>  // NOLINT
#include <ignition/common/Time.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include "subt_gazebo/MotionTimerDetachPlugin.hh"

namespace subt
{
GZ_REGISTER_MODEL_PLUGIN(MotionTimerDetachPlugin)

using JointWeakPtr = boost::weak_ptr<gazebo::physics::Joint>;

/// \internal
/// \brief Private data for the MotionTimerDetachPlugin class.
class MotionTimerDetachPluginPrivate
{
  /// \brief Time after which the joints should be detached.
  public: ignition::common::Time detachTimeLimit =
          ignition::common::Time(20*60, 0);

  /// \brief Pointer to the model that defines this plugin
  public: gazebo::physics::ModelPtr model;

  /// \brief World update connection pointer.
  public: gazebo::event::ConnectionPtr updateConnection;
};

/////////////////////////////////////////////////
MotionTimerDetachPlugin::MotionTimerDetachPlugin()
  : JointMotionTimerPlugin(), dataPtr(new MotionTimerDetachPluginPrivate)
{
}

/////////////////////////////////////////////////
MotionTimerDetachPlugin::~MotionTimerDetachPlugin()
{
}

/////////////////////////////////////////////////
void MotionTimerDetachPlugin::Load(gazebo::physics::ModelPtr _parent,
                                  sdf::ElementPtr _sdf)
{
  this->JointMotionTimerPlugin::Load(_parent, _sdf);

  this->dataPtr->model = _parent;

  if (_sdf->HasElement("detach_time_limit"))
  {
    this->dataPtr->detachTimeLimit =
      ignition::common::Time(_sdf->Get<double>("detach_time_limit"));
  }

  this->dataPtr->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&MotionTimerDetachPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MotionTimerDetachPlugin::OnUpdate()
{
  if (this->ElapsedTime() > this->dataPtr->detachTimeLimit)
  {
    for (auto joint : this->Joints())
    {
      if (joint)
      {
        joint->Detach();
        joint->SetProvideFeedback(false);
      }
    }

    gzdbg << "Detaching joints in model "
          << this->dataPtr->model->GetScopedName()
          << std::endl;

    // stop the OnUpdate callbacks
    this->dataPtr->updateConnection.reset();
  }
}
}
