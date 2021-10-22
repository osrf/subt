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

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Inertial.hh"
#include <ignition/gazebo/components/Name.hh>
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/WindMode.hh"

#include <ignition/sensors/Noise.hh>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include "ignition/gazebo/Link.hh"

#include "subt_ign/LocalWindPlugin.hh"
#include "subt_ign/VisibilityTable.hh"

IGNITION_ADD_PLUGIN(
    subt::LocalWind,
    ignition::gazebo::System,
    subt::LocalWind::ISystemConfigure,
    subt::LocalWind::ISystemPreUpdate)

using namespace ignition;
using namespace subt;

/// \brief Wind properties for a tile.
class WindTile
{
  /// \brief Constructor
  public: WindTile() = default;

  /// \brief Load based on SDF.
  public: void Load(sdf::ElementPtr _elem);

  /// \brief Tile number
  public: uint64_t number{0};

  /// \brief Linear velocity for the tile
  public: ignition::math::Vector3d linearVelocity;

  /// \brief Noise added to linear velocity.
  public: sensors::NoisePtr noise;
};

//////////////////////////////////////////////////
class subt::LocalWindPrivate
{
  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  /// \param[in] _sdf Pointer to sdf::Element that contains configuration
  /// parameters for the system.
  public: void Load(gazebo::EntityComponentManager &_ecm,
                    const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief World entity to which this system is attached.
  public: gazebo::Entity worldEntity;

  /// \brief A map that stores the tiles that have wind defined
  // public: std::map<uint64_t, ignition::math::Vector3d> windTiles;
  public: std::map<uint64_t, WindTile> windTiles;

  /// \brief Factor to determine wind strength
  public: double forceApproximationScalingFactor{1.0};

  /// \brief Table to store the map xyz position -> tile
  public: subt::VisibilityTable visibilityTable;

  /// \brief boolean to confirm if the plugin is properly configured
  public: bool initialized{false};
};

//////////////////////////////////////////////////
LocalWind::LocalWind() :
  dataPtr(std::make_unique<LocalWindPrivate>())
{
}

//////////////////////////////////////////////////
LocalWind::~LocalWind() = default;

//////////////////////////////////////////////////
void WindTile::Load(sdf::ElementPtr _elem)
{
  this->number = _elem->Get<uint64_t>("tile",
      std::numeric_limits<uint64_t>::max()).first;

  this->linearVelocity = _elem->Get<ignition::math::Vector3d>(
      "linear_velocity", ignition::math::Vector3d::Zero).first;

  if (_elem->HasElement("noise"))
  {
    this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(
        _elem->GetElementImpl("noise"));
  }
}

//////////////////////////////////////////////////
void LocalWindPrivate::Load(
    gazebo::EntityComponentManager &_ecm,
    const std::shared_ptr<const sdf::Element> &_sdf
    )
{
  this->forceApproximationScalingFactor =
    _sdf->Get<double>("scaling", this->forceApproximationScalingFactor).first;

  // Iterate over all the local_wind instances defined
  // inside the plugin
  auto sdfClone = _sdf->Clone();
  sdf::ElementPtr localWindElement = sdfClone->GetElement("local_wind");
  while (localWindElement)
  {
    // Check if valid
    if (localWindElement->HasElement("tile") &&
        localWindElement->HasElement("linear_velocity"))
    {
      WindTile tile;
      tile.Load(localWindElement);

      // Avoid duplicates in the plugin definition
      if (this->windTiles.find(tile.number) == this->windTiles.end())
      {
        this->windTiles[tile.number] = tile;
      }
      else
      {
        ignerr << "Tile #" << tile.number
          << " was already added. Remove duplicated tiles\n";
        return;
      }
    }
    else
    {
      ignerr << "Parsing problem. Each local_wind element requires a tile"
             << "and a magnitude\n";
      return;
    }

    // parse next localWind element
    localWindElement = localWindElement->GetNextElement("local_wind");
  }

  if(this->windTiles.size() == 0)
  {
    ignwarn << "Empty LocalWindPlugin, not using LocalWindPlugin\n";
    return;
  }

  auto worldName =
    _ecm.Component<gazebo::components::Name>(this->worldEntity);
  if (!worldName){
    ignerr << "Unable to get world name\n";
    return;
  }

  if (!this->visibilityTable.Load(worldName->Data()))
  {
    ignerr << "Unable to load visibility table data files\n";
    return;
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void LocalWind::Configure(
    const gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->worldEntity = _entity;
  this->dataPtr->Load(_ecm, _sdf);
}

//////////////////////////////////////////////////
void LocalWind::PreUpdate(
    const gazebo::UpdateInfo &_info,
    gazebo::EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
    return;

  if (_info.paused)
    return;

  // Init each new link added to the world
  _ecm.EachNew<gazebo::components::Link, gazebo::components::WindMode>(
      [&](const gazebo::Entity &_entity,
          gazebo::components::Link *,
          const gazebo::components::WindMode *_windMode) -> bool
  {
    if (_windMode->Data())
    {
      // Create a WorldLinearVelocity component on the link so that
      // physics can populate it
      if (!_ecm.Component<gazebo::components::WorldLinearVelocity>(_entity))
      {
        _ecm.CreateComponent(_entity,
                             gazebo::components::WorldLinearVelocity());
      }
    }
    return true;
  });

  gazebo::Link link;

  _ecm.Each<gazebo::components::Link, gazebo::components::Inertial,
            gazebo::components::WindMode,
            gazebo::components::WorldLinearVelocity>(
    [&](const gazebo::Entity &_entity, const gazebo::components::Link *,
        const gazebo::components::Inertial *_inertial,
        const gazebo::components::WindMode *_windMode,
        const gazebo::components::WorldLinearVelocity *_linkVel
        ) -> bool
    {
      // Skip links for which the wind is disabled
      // This assumption also implies links with windMode enabled
      // were populated with
      if (!_windMode->Data())
      {
        return true;
      }

      // Get XYZ of each link
      math::Pose3d linkWorldPose = worldPose(_entity, _ecm);
      math::Vector3d linkPosition = linkWorldPose.Pos();

      int32_t x = std::round(linkPosition.X());
      int32_t y = std::round(linkPosition.Y());
      int32_t z = std::round(linkPosition.Z());
      auto roundedPos = std::make_tuple(x, y, z);

      // Get Tile defined for that XYZ
      const auto &tilesMap = this->dataPtr->visibilityTable.Vertices();

      // Skip if the robot position is not in the tile.
      if (tilesMap.find(roundedPos) == tilesMap.end())
        return true;

      const auto linkTile = tilesMap.at(roundedPos);

      // If Tile is in the tiles defined by the plugin, apply force
      auto tileLocalWind = this->dataPtr->windTiles.find(linkTile);
      if (tileLocalWind != this->dataPtr->windTiles.end())
      {

        link.ResetEntity(_entity);

        ignition::math::Vector3d linearVelocity =
          tileLocalWind->second.linearVelocity;

        // Apply noise
        if (tileLocalWind->second.noise)
        {
          linearVelocity.X() =
            tileLocalWind->second.noise->Apply(linearVelocity.X());
          linearVelocity.Y() =
            tileLocalWind->second.noise->Apply(linearVelocity.Y());
          linearVelocity.Z() =
            tileLocalWind->second.noise->Apply(linearVelocity.Z());
        }

        // Debugging:
        // igndbg << "Position: " << std::get<0>(roundedPos) << " "
        //   << std::get<1>(roundedPos) << " "
        //   << std::get<2>(roundedPos) << " "
        //   << " current tile: " << linkTile
        //   << " local_wind:" << linearVelocity << "\n";

        math::Vector3d windForce =
          _inertial->Data().MassMatrix().Mass() *
          this->dataPtr->forceApproximationScalingFactor *
          (linearVelocity - _linkVel->Data());

        // Apply force at center of mass
        link.AddWorldForce(_ecm, windForce);
      }
      return true;
    });
}
