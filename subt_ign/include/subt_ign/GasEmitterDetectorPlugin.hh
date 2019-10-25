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

#ifndef SUBT_IGN_GASEMITTERDETECTORPLUGIN_HH_
#define SUBT_IGN_GASEMITTERDETECTORPLUGIN_HH_

#include <ignition/gazebo/System.hh>

namespace subt
{
// Forward delcarations.
class GasDetectorPrivate;

class GasEmitter:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure
{
  /// \brief Constructor
  public: GasEmitter();

  /// \brief Destructor
  public: ~GasEmitter();

  // Documentation inherited
  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;
};

class GasDetector:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPostUpdate
{
  /// \brief Constructor
  public: GasDetector();

  /// \brief Destructor
  public: ~GasDetector();

  // Documentation inherited
  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                          const ignition::gazebo::EntityComponentManager &_ecm) override;


  /// \brief Private data pointer.
  private: std::unique_ptr<GasDetectorPrivate> dataPtr;
};
}  // namespace subt

#endif  // SUBT_IGN_OXYGENGASPLUGIN_HH_


