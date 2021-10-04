/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef SUBT_IGN_LOCALWINDPLUGIN_HH_
#define SUBT_IGN_LOCALWINDPLUGIN_HH_

#include <ignition/gazebo/System.hh>

namespace subt
{
  class LocalWindPrivate;
  /// \class LocalWind
  /// \brief Iterates over all the models and adds wind accordingly.
  ///
  /// The LocalWind plugin loads a pre-computed lookup table to
  /// determine the tile where a robot is based in its position.
  /// In each simulation step, checks all the robots tiles
  /// Then checks if the tile has wind defined, and if that's the case
  /// Add wind to the links of the robot.

  class LocalWind:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
  {
    /// \brief Constructor
    public: LocalWind();

    /// \brief Destructor
    public: ~LocalWind();

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<LocalWindPrivate> dataPtr;
  };
}
#endif
