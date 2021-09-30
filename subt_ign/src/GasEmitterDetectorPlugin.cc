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

#include "subt_ign/GasEmitterDetectorPlugin.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/double.pb.h>

#include <ignition/common/StringUtils.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <set>

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

  /// \brief publisher to publish ppm
  public: transport::Node::Publisher ppmPub;

  /// \brief update rate
  public: double updateRate;

  /// \brief gas detection types
  public: std::set<std::string> detectorTypes;

  /// \brief Entity this sensor is attached to
  public: gazebo::Entity entity;

  /// \brief response time
  /// Defaut to SCD30 sensor. Response time: (t63) with 2s sampling interval
  /// based on specification:
  /// https://www.sensirion.com/scd30/
  /// This is the time it takes to detect 63% of an instance change in gas
  // quantity. Actual response time may be longer
  public: double responseTime = 20.0;

  // accuracy +-(30ppm + 3%MV)
  public: double precision = 30.0;

  /// \brief Min CO2 concentration.
  /// Set default to global average atmospheric CO2
  public: double minConcentration = 412.0;

  /// \brief CO2 gas concentration based on subt finals artifact specification:
  /// "The gas will be released into a confined area to maintain a concentration
  /// of at least 1500 parts per million (ppm)"
  public: double gasConcentration = 1500.0;

  /// \brief Gas concentration range
  /// The range at which the gas sensor reading will vary. The closer the
  ///  gas sensor is to the emitter source, the higher the value of the
  /// readings. The max output will be gasConcentration + gasConcentrationRange.
  public: double gasConcentrationRange = 150.0;

  /// \brief max gas concentration. Negative means unspecified.
  public: double maxConcentration = -1;

  /// \brief Contentration in ppm
  public: double concentration = 0.0;
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
    _ecm.CreateComponent(entity, gazebo::components::Geometry(geometry));
    _ecm.CreateComponent(entity, gazebo::components::Pose(pose));

    if (_ecm.EntityHasComponentType(_entity,
                                    gazebo::components::World::typeId))
    {
      _ecm.CreateComponent(entity, gazebo::components::WorldPose(pose));
    }
    else if (_ecm.EntityHasComponentType(_entity,
                                         gazebo::components::Model::typeId))
    {
      auto model_pose = _ecm.Component<gazebo::components::Pose>(_entity);
      pose += model_pose->Data();
      _ecm.CreateComponent(entity, gazebo::components::WorldPose(pose));
    }


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

  // response time
  this->dataPtr->responseTime = _sdf->Get<double>("response_time",
      this->dataPtr->responseTime).first;

  // precision in ppm
  this->dataPtr->precision = _sdf->Get<double>("precision",
      this->dataPtr->precision).first;

  // min concentration in ppm
  this->dataPtr->minConcentration = _sdf->Get<double>("min_concentration",
      this->dataPtr->minConcentration).first;

  // gas concentration in ppm
  this->dataPtr->gasConcentration = _sdf->Get<double>("gas_concentration",
      this->dataPtr->gasConcentration).first;

  // gas concentration range ppm
  this->dataPtr->gasConcentrationRange = _sdf->Get<double>(
      "gas_concentration_range", this->dataPtr->gasConcentrationRange).first;

  // max concentration in ppm
  this->dataPtr->maxConcentration = _sdf->Get<double>("max_concentration",
      this->dataPtr->maxConcentration).first;

  // check max and min concentration
  if (this->dataPtr->maxConcentration < this->dataPtr->minConcentration)
  {
    ignwarn << "Max concentration is lower than min concentration. "
      << "Setting max to unbounded." << std::endl;
    this->dataPtr->maxConcentration = -1;
  }

  // initialize current gas concentration to min value
  this->dataPtr->concentration = this->dataPtr->minConcentration;

  if (this->dataPtr->updateRate > 0.0)
  {
    transport::AdvertiseMessageOptions opts;
    opts.SetMsgsPerSec(this->dataPtr->updateRate);

    this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::Boolean>(
        topic, opts);

    this->dataPtr->ppmPub = this->dataPtr->node.Advertise<msgs::Double>(
        topic + "/ppm", opts);
  }
  else
  {
    this->dataPtr->pub =  this->dataPtr->node.Advertise<msgs::Boolean>(topic);
    this->dataPtr->ppmPub = this->dataPtr->node.Advertise<msgs::Double>(
        topic + "/ppm");
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
  math::Vector3d emitterPos;
  math::Vector3d emitterBoxSize;

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
          this->dataPtr->detectorTypes.count(_gasType->Data()))
      {
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
          if (emitterVolume.Contains(detectorPose.Pos()))
          {
            detected = true;
            emitterPos = _worldPose->Data().Pos();
            emitterBoxSize = emitterBox->Size();
            return false;
          }
        }
      }
      return true;
    });

  double dtSec = std::chrono::duration<double>(_info.dt).count();
  double targetConcentration = this->dataPtr->minConcentration;

  // Step 1: model concentration level based on distance to emitter origin
  if (detected)
  {
    // assume concentration is the highest at the emitter origin
    double distToOrigin = (emitterPos - detectorPose.Pos()).Length();
    double distPercentage = distToOrigin / emitterBoxSize.Max();
    targetConcentration = this->dataPtr->gasConcentration +
      this->dataPtr->gasConcentrationRange * (1.0 - distPercentage);
  }

  // Step 2: model change in concentration by taking into account the sensor
  // response time. Currently assuming linear rate change, however, this could
  // be more like logarithmic
  double rate = 0.0;
  if (!detected ||
      (detected && this->dataPtr->concentration <
       this->dataPtr->gasConcentration))
  {
    // high concentration rate change if in recovery state (no detection) or
    // min concentration is not reached
    rate = this->dataPtr->gasConcentration / this->dataPtr->responseTime;
    rate *= (detected ? 1.0 : -1.0);
  }
  else
  {
    // change in concentration proportional to diff between current and target
    // concentration
    rate = (targetConcentration - this->dataPtr->concentration) /
      this->dataPtr->responseTime;
  }
  double step = dtSec * rate;
  this->dataPtr->concentration += step;

  this->dataPtr->concentration = std::max(this->dataPtr->concentration,
      this->dataPtr->minConcentration);
  if (this->dataPtr->maxConcentration > 0)
    this->dataPtr->concentration = std::min(this->dataPtr->concentration,
        this->dataPtr->maxConcentration);

  // Step 3: apply Gaussian noise based on sensor accuracy
  double noise = ignition::math::Rand::DblNormal(0.0, this->dataPtr->precision);
  double concentrationWithNoise = this->dataPtr->concentration + noise;

  // publish gas concentration in ppm
  msgs::Double cMsg;
  cMsg.mutable_header()->mutable_stamp()->CopyFrom(
      gazebo::convert<msgs::Time>(_info.simTime));
  cMsg.set_data(concentrationWithNoise);
  this->dataPtr->ppmPub.Publish(cMsg);

  // publish boolean value
  msgs::Boolean msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      gazebo::convert<msgs::Time>(_info.simTime));
  msg.set_data(detected);
  this->dataPtr->pub.Publish(msg);
}
