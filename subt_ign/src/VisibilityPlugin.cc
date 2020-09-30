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
#include <algorithm>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/math/AxisAlignedBox.hh>

#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include "subt_ign/VisibilityTable.hh"
#include "subt_ign/VisibilityPlugin.hh"

#include "ign_to_fcl.hh"

IGNITION_ADD_PLUGIN(
    subt::VisibilityPlugin,
    ignition::gazebo::System,
    subt::VisibilityPlugin::ISystemConfigure,
    subt::VisibilityPlugin::ISystemPreUpdate,
    subt::VisibilityPlugin::ISystemPostUpdate)

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace subt;

/// \brief Private data class for VisibilityPlugin
class subt::VisibilityPluginPrivate
{
  /// \brief A map of model name ot its bounding box
  public: std::map<std::string, ignition::math::AxisAlignedBox> bboxes;

  public: std::map<std::string, std::shared_ptr<fcl::CollisionObject>> fclObjs;

  /// \brief Name of the world
  public: std::string worldName;
};

/////////////////////////////////////////////
VisibilityPlugin::VisibilityPlugin()
  : dataPtr(new VisibilityPluginPrivate)
{
}

/////////////////////////////////////////////
VisibilityPlugin::~VisibilityPlugin()
{
}

//////////////////////////////////////////////////
void VisibilityPlugin::Configure(const ignition::gazebo::Entity & /*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &/*_ecm*/,
                           ignition::gazebo::EventManager & /*_eventMgr*/)
{
  if (!_sdf->HasElement("world_name"))
  {
    ignerr << "Unable to load visibility plugin. <world_name> not specified."
           << std::endl;
    return;
  }

  const sdf::ElementPtr worldNameElem =
    const_cast<sdf::Element*>(_sdf.get())->GetElement("world_name");
  this->dataPtr->worldName = worldNameElem->Get<std::string>();
}

//////////////////////////////////////////////////
void VisibilityPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->worldName.empty())
    return;

  // create aabb component to be filled by physics
  _ecm.EachNew<gazebo::components::Model,
            gazebo::components::Static>(
      [&](const gazebo::Entity &_entity,
          const gazebo::components::Model *,
          const gazebo::components::Static *) -> bool
      {
        auto aabb = _ecm.Component<components::AxisAlignedBox>(_entity);
        if (!aabb)
          _ecm.CreateComponent(_entity, components::AxisAlignedBox());
        return true;
      });

  // create aabb component to be filled by physics
  _ecm.Each<gazebo::components::Model,
            gazebo::components::Static>(
      [&](const gazebo::Entity &_entity,
          const gazebo::components::Model *,
          const gazebo::components::Static *) -> bool
      {
        auto aabb = _ecm.Component<components::AxisAlignedBox>(_entity);
        if (!aabb)
          _ecm.CreateComponent(_entity, components::AxisAlignedBox());
        return true;
      });
}

//////////////////////////////////////////////////
void VisibilityPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->worldName.empty())
    return;

  int64_t s, ns;
  std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);
  if (s < 1)
    return;

  _ecm.Each<gazebo::components::Collision,
            gazebo::components::Name,
            gazebo::components::Geometry,
            gazebo::components::ParentEntity>(
      [&](const Entity &,
          const components::Collision *,
          const components::Name *_name,
          const components::Geometry *_geom,
          const components::ParentEntity *_parent) -> bool
      {
        if (_geom->Data().Type() == sdf::GeometryType::MESH)
        {
          const sdf::Mesh *meshSdf = _geom->Data().MeshShape();
          if (nullptr == meshSdf)
          {
            ignwarn << "Mesh geometry for collision [" << _name->Data()
                    << "] missing mesh shape." << std::endl;
            return true;
          }

          auto &meshManager = *ignition::common::MeshManager::Instance();
          auto fullPath = asFullPath(meshSdf->Uri(), meshSdf->FilePath());
          auto *mesh = meshManager.Load(fullPath);
          if (nullptr == mesh)
          {
            ignwarn << "Failed to load mesh from [" << fullPath
                    << "]." << std::endl;
            return true;
          }

          auto gP = _ecm.Component<gazebo::components::ParentEntity>(_parent->Data())->Data();
          auto parentPose = _ecm.Component<gazebo::components::Pose>(gP)->Data();
          auto parentName = _ecm.Component<gazebo::components::Name>(gP)->Data();
          auto collisionObj = convert_to_fcl(*mesh, parentPose);
          collisionObj->computeAABB();
          this->dataPtr->fclObjs[parentName] = collisionObj;
        }

        return true;
      });

  // get all the bounding boxes
  _ecm.Each<gazebo::components::Model,
            gazebo::components::Name,
            gazebo::components::AxisAlignedBox,
            gazebo::components::Static>(
      [&](const gazebo::Entity &,
          const gazebo::components::Model *,
          const gazebo::components::Name *_nameComp,
          const gazebo::components::AxisAlignedBox *_aabb,
          const gazebo::components::Static *) -> bool
      {
        // todo store bboxes instead
        this->dataPtr->bboxes[_nameComp->Data()] = _aabb->Data();

        auto it = this->dataPtr->fclObjs.find(_nameComp->Data());
        if (it == this->dataPtr->fclObjs.end())
        {
          std::cout << "Couldn't find fcl obj for: " << _nameComp->Data() << std::endl;
          return true;
        }

        auto fclAABB = it->second->getAABB();

        std::cout << _nameComp->Data() << std::endl;
        std::cout << fclAABB.center()[0] << " "
                  << fclAABB.center()[1] << " "
                  << fclAABB.center()[2] << std::endl;
        std::cout << _aabb->Data().Center().X() << " " << _aabb->Data().Center().Y()  << " " << _aabb->Data().Center().Z() << std::endl;

        return true;
      });

  using MapElement = std::pair<std::string, ignition::math::AxisAlignedBox>;
  auto dimCompare = [](int _dim){
    return [_dim](const MapElement &_a, const MapElement &_b)
    {
      return _a.second.Size()[_dim] < _b.second.Size()[_dim];
    };
  };
  auto maxX = std::max_element(this->dataPtr->bboxes.begin(),
                               this->dataPtr->bboxes.end(), dimCompare(0));
  auto maxY = std::max_element(this->dataPtr->bboxes.begin(),
                               this->dataPtr->bboxes.end(), dimCompare(1));
  auto maxZ = std::max_element(this->dataPtr->bboxes.begin(),
                               this->dataPtr->bboxes.end(), dimCompare(2));

  std::cout << "Max tile dimensions: X = " << maxX->second.Size()[0] << " ("
            << maxX->first << ") Y = " << maxY->second.Size()[1] << " ("
            << maxY->first << ") Z = " << maxZ->second.Size()[2] << " ("
            << maxZ->first << ")" << std::endl;
  // generate the LUT
  subt::VisibilityTable table;
  table.Load(this->dataPtr->worldName, false);
  table.SetModelBoundingBoxes(this->dataPtr->bboxes);
  table.SetModelCollisionObjects(this->dataPtr->fclObjs);
  table.Generate();

  // Send SIGINT to terminate Gazebo.
  raise(SIGINT);
}
