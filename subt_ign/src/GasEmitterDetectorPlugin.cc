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

#include <ignition/common/StringUtils.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/gazebo/Util.hh>

#include <set>

#include "subt_ign/GasEmitterDetectorPlugin.hh"

IGNITION_ADD_PLUGIN(
    subt::GasEmitter,
    ignition::gazebo::System,
    subt::GasEmitter::ISystemConfigure)

IGNITION_ADD_PLUGIN(
    subt::GasDetector,
    ignition::gazebo::System,
    subt::GasDetector::ISystemConfigure,
    subt::GasDetector::ISystemPostUpdate)

using GasType = ignition::gazebo::components::Component<std::string,
                                                        class GasTypeTag>;
IGN_GAZEBO_REGISTER_COMPONENT("subt_components.GasType", GasType)

using namespace ignition;
using namespace subt;

//////////////////////////////////////////////////
class subt::GasDetectorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish detections
  public: transport::Node::Publisher pub;

  /// \brief update rate
  public: double updateRate;

  /// \brief gas detection types
  public: std::set<std::string> detectorTypes;

  /// \brief Entity this sensor is attached to
  public: gazebo::Entity entity;
};

//////////////////////////////////////////////////
GasEmitter::GasEmitter() = default;

//////////////////////////////////////////////////
GasEmitter::~GasEmitter() = default;

//////////////////////////////////////////////////
void GasEmitter::Configure(const gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager & /*_eventMgr*/)
{
  sdf::ElementPtr gasElem = const_cast<sdf::Element*>(
      _sdf.get())->GetElement("emitter");

  while (gasElem)
  {
    auto entity = _ecm.CreateEntity();

    sdf::Geometry geometry;
    geometry.Load(gasElem->GetElement("geometry"));
    auto gasType = gasElem->Get<std::string>("type");
    auto pose = gasElem->Get<math::Pose3d>("pose");

    _ecm.SetParentEntity(entity, _entity);
    _ecm.CreateComponent(entity, GasType(gasType));
    _ecm.CreateComponent(entity, gazebo::components::WorldPose(pose));
    _ecm.CreateComponent(entity, gazebo::components::Geometry(geometry));

    gasElem = gasElem->GetNextElement("emitter");
  }
}

//////////////////////////////////////////////////
GasDetector::GasDetector() :
  dataPtr(std::make_unique<GasDetectorPrivate>())
{
}

//////////////////////////////////////////////////
GasDetector::~GasDetector() = default;

//////////////////////////////////////////////////
void GasDetector::Configure(
    const gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->entity = _entity;

  // Split up space delimited gas types
  auto gasType = _sdf->Get<std::string>("type", "").first;
  auto gasTypes = common::Split(gasType, ' ');
  if (gasType != "")
  {
    this->dataPtr->detectorTypes =
      std::set<std::string>(gasTypes.begin(), gasTypes.end());
  }

  auto topic = _sdf->Get<std::string>("topic",
      gazebo::scopedName(_entity, _ecm) + "/leak").first;
  this->dataPtr->updateRate = _sdf->Get<double>("update_rate", 0.0).first;


  if (this->dataPtr->updateRate > 0.0)
  {
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(this->dataPtr->updateRate);
    this->dataPtr->pub =  this->dataPtr->node.Advertise<msgs::Boolean>(
        topic, opts);
  }
  else
  {
    this->dataPtr->pub =  this->dataPtr->node.Advertise<msgs::Boolean>(topic);
  }
}

//////////////////////////////////////////////////
void GasDetector::PostUpdate(
    const gazebo::UpdateInfo &_info,
    const gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  auto detectorPose = gazebo::worldPose(this->dataPtr->entity, _ecm);
  auto detected = false;

  _ecm.Each<GasType,
            gazebo::components::WorldPose,
            gazebo::components::Geometry>(
    [&](const gazebo::Entity &_entity,
        const GasType *_gasType,
        const gazebo::components::WorldPose *_worldPose,
        const gazebo::components::Geometry *_geometry)
    {
      // If the set is empty, accept all types.
      if (this->dataPtr->detectorTypes.size() == 0 ||
          this->dataPtr->detectorTypes.count(_gasType->Data())) {
        auto emitterBox = _geometry->Data().BoxShape();
        if (nullptr == emitterBox)
        {
          ignerr << "Internal error: geometry of gas emitter [" << _entity
                << "] missing box." << std::endl;
        }
        else
        {
          ignition::math::OrientedBoxd emitterVolume { emitterBox->Size(),
                                                       _worldPose->Data() };
          if (emitterVolume.Contains(detectorPose.Pos())) {
            detected = true;
          }
        }
      }
      return true;
    }
  );

  msgs::Boolean msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      gazebo::convert<msgs::Time>(_info.simTime));
  msg.set_data(detected);
  this->dataPtr->pub.Publish(msg);
}
