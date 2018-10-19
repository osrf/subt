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
#include <vector>
#include <boost/weak_ptr.hpp>
#include <subt_gazebo/JointMotionTimerPlugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

namespace subt_gazebo
{
GZ_REGISTER_MODEL_PLUGIN(JointMotionTimerPlugin)

class JointMotionTimerPluginPrivate
{
  /// \brief Elapsed time.
  public: double elapsedTime = 0.0;

  /// \brief PhysicsEngine pointer for getting timestep.
  public: gazebo::physics::PhysicsEnginePtr engine;

  /// \brief Pointer to the model that defines this plugin
  public: gazebo::physics::ModelPtr model;

  using JointWeakPtr = boost::weak_ptr<gazebo::physics::Joint>;

  /// \brief Joints to check for motion.
  public: std::vector<JointWeakPtr> joints;

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

  auto world = _parent->GetWorld();
  if (!world)
  {
    gzerr << "Unable to get world pointer." << std::endl;
    return;
  }
  this->dataPtr->engine = world->Physics();
  if (!this->dataPtr->engine)
  {
    gzerr << "Unable to get physics engine pointer." << std::endl;
    return;
  }

  this->dataPtr->updateConnection =
      gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&JointMotionTimerPlugin::OnUpdate, this));
}

void JointMotionTimerPlugin::OnUpdate()
{
  double dt = this->dataPtr->engine->GetMaxStepSize();

  bool motionDetected = false;
  for (const auto &weakJoint : this->dataPtr->joints)
  {
    gazebo::physics::JointPtr joint = weakJoint.lock();
    if (joint)
    {
      if (std::abs(joint->GetVelocity(0)) > 1e-2)
      {
        motionDetected = true;
        break;
      }
    }
  }

  if (motionDetected)
  {
    this->dataPtr->elapsedTime += dt;
  }
}
}
