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
#ifndef SUBT_PLAYBACK_SENSORS_HH_
#define SUBT_PLAYBACK_SENSORS_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>
#include <sdf/Sensor.hh>

namespace subt
{
  // Forward declarations.
  class SensorsPrivate;

  /// \brief This was copied from Ignition Gazebo, and modified to allow
  /// only the creation of the "spawned_camera" sensor. This lets video
  /// recording take place on a headless simulation instance.
  class PlaybackSensors:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit PlaybackSensors();

    /// \brief Destructor
    public: ~PlaybackSensors() override;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) final;

    // Documentation inherited
    public: void Update(const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm) final;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                            const ignition::gazebo::EntityComponentManager &_ecm) final;

    /// \brief Create a rendering sensor from sdf
    /// \param[in] _entity Entity of the sensor
    /// \param[in] _sdf SDF description of the sensor
    /// \param[in] _parentName Name of parent that the sensor is attached to
    /// \return Sensor name
    private : std::string CreateSensor(const ignition::gazebo::Entity &_entity,
                                       const sdf::Sensor &_sdf,
                                       const std::string &_parentName);

    /// \brief Removes a rendering sensor
    /// \param[in] _entity Entity of the sensor
    private : void RemoveSensor(const ignition::gazebo::Entity &_entity);

    /// \brief Private data pointer.
    private: std::unique_ptr<SensorsPrivate> dataPtr;
  };
}
#endif
