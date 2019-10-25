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

/// \brief A system for creating gas emitters in a simulation world.
/*
 * Each emitter is defined as a region in the simulation environment where
 * a particular gas (defined by 'type') can be detected by the mobile
 * GasDetector system.
 *
 * Example 1m cube methane leak at x=10, y=0, z=0
 *
 *  <plugin
 *    filename="libGasEmitterDetectorPlugin.so"
 *    name="subt::GasEmitter">
 *    <emitter>
 *      <type>methane</type>
 *      <pose>10 0 0 0 0 0</pose>
 *      <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
 *    </emitter>
 *   </plugin>
 */
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

/// \brief A system for creating gas detectors in a simulation world.
/*
 *
 * Each detector is defined by a type (or space delimited types).
 * The detector will return true if it is in an emitter region with a
 * matching type, otherwise false.
 *
 * Example propane detector that publishes on `/propane_detector` at 10Hz:
 *
 *  <plugin name="subt::GasDetector" filename="libGasEmitterDetectorPlugin.so">
 *    <topic>propane_detector</topic>
 *    <update_rate>10</update_rate>
 *    <type>propane</type>
 *  </plugin>
 *
 * Example smoke and carbon monoxide detector that publishes on `/smoke` at 10Hz:
 *
 *  <plugin name="subt::GasDetector" filename="libGasEmitterDetectorPlugin.so">
 *    <topic>smoke</topic>
 *    <update_rate>10</update_rate>
 *    <type>smoke co</type>
 *  </plugin>
 */
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

#endif  // SUBT_IGN_GASEMITTERDETECTORPLUGIN_HH_
