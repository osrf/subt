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
#include "subt_gazebo/MotionTimerLockPlugin.hh"

namespace subt
{
GZ_REGISTER_MODEL_PLUGIN(MotionTimerLockPlugin)

using JointWeakPtr = boost::weak_ptr<gazebo::physics::Joint>;

/// \internal
/// \brief Private data for the MotionTimerLockPlugin class.
class MotionTimerLockPluginPrivate
{
  /// \brief Time after which the joints should be locked.
  public: ignition::common::Time lockTimeLimit =
          ignition::common::Time(20*60, 0);

  /// \brief Pointer to the model that defines this plugin
  public: gazebo::physics::ModelPtr model;

  /// \brief World update connection pointer.
  public: gazebo::event::ConnectionPtr updateConnection;
};

/////////////////////////////////////////////////
MotionTimerLockPlugin::MotionTimerLockPlugin()
  : JointMotionTimerPlugin(), dataPtr(new MotionTimerLockPluginPrivate)
{
}

/////////////////////////////////////////////////
MotionTimerLockPlugin::~MotionTimerLockPlugin()
{
}

/////////////////////////////////////////////////
void MotionTimerLockPlugin::Load(gazebo::physics::ModelPtr _parent,
                                 sdf::ElementPtr _sdf)
{
  this->JointMotionTimerPlugin::Load(_parent, _sdf);

  this->dataPtr->model = _parent;

  if (_sdf->HasElement("lock_time_limit"))
  {
    this->dataPtr->lockTimeLimit =
      ignition::common::Time(_sdf->Get<double>("lock_time_limit"));
  }

  this->dataPtr->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&MotionTimerLockPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MotionTimerLockPlugin::OnUpdate()
{
  if (this->ElapsedTime() > this->dataPtr->lockTimeLimit)
  {
    for (auto joint : this->Joints())
    {
      if (joint)
      {
        joint->SetUpperLimit(0, joint->Position(0));
        joint->SetLowerLimit(0, joint->Position(0));
      }
    }

    gzdbg << "Locking joints in model "
          << this->dataPtr->model->GetScopedName()
          << std::endl;

    // stop the OnUpdate callbacks
    this->dataPtr->updateConnection.reset();
  }
}
}
