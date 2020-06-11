/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "subt_ign/ConnectionValidator.hh"
#include "ConnectionValidatorPrivate.hh"

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "subt_ign/CommonTypes.hh"

IGNITION_ADD_PLUGIN(
  subt::ConnectionValidator,
  ignition::gazebo::System,
  subt::ConnectionValidator::ISystemConfigure,
  subt::ConnectionValidator::ISystemPostUpdate
)

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace subt;


/////////////////////////////////////////////////
ConnectionValidator::ConnectionValidator()
  : dataPtr(new ConnectionValidatorPrivate)
{
}

/////////////////////////////////////////////////
ConnectionValidator::~ConnectionValidator() = default;

/////////////////////////////////////////////////
void ConnectionValidator::Configure(const ignition::gazebo::Entity & /*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager & /*_ecm*/,
    ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->worldName = const_cast<sdf::Element*>(
      _sdf.get())->Get<std::string>("world_name", "world_name").first;

  igndbg << "Set world name: " << this->dataPtr->worldName << std::endl;

  this->dataPtr->Load(this->dataPtr->worldName);
  this->dataPtr->PopulateConnections();

  this->dataPtr->node.Advertise("/connection/next",
      &ConnectionValidatorPrivate::OnNext, this->dataPtr.get());
  this->dataPtr->node.Advertise("/connection/prev",
      &ConnectionValidatorPrivate::OnPrev, this->dataPtr.get());
}

//////////////////////////////////////////////////
void ConnectionValidator::PostUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
}
