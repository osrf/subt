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

#include "subt_ign/ArtifactValidator.hh"

#include <ignition/common/Console.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Level.hh>
#include <ignition/gazebo/components/LevelEntityNames.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/transport/Node.hh>

#include "subt_ign/CommonTypes.hh"

IGNITION_ADD_PLUGIN(
  subt::ArtifactValidator,
  ignition::gazebo::System,
  subt::ArtifactValidator::ISystemConfigure,
  subt::ArtifactValidator::ISystemPostUpdate
)

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace subt;

/// \brief Simple structure to represent an artifact
struct Artifact
{
  /// \brief Artifact name
  std::string name;

  /// \brief Artifact type
  subt::ArtifactType type;

  /// \brief Artifact type as a string
  std::string typeStr;

  /// \brief Artifact pose
  ignition::math::Pose3d pose;

  /// \brief Return a string representation of the artifact
  std::string String() const
  {
    std::stringstream ss;
    ss << "<Artifact:"
       << " name=" << this->name
       << " type=" << this->typeStr
       << " pose=(" << this->pose << ")"
       << ">";
    return ss.str();
  }
};

/// \brief Private data for the artifact validator.
class subt::ArtifactValidatorPrivate
{
  /// \brief Map of artifact types to string representations.
  public: const std::array<
      const std::pair<subt::ArtifactType, std::string>, 12> kArtifactTypes
      {
        {
          {subt::ArtifactType::TYPE_BACKPACK      , "TYPE_BACKPACK"},
          {subt::ArtifactType::TYPE_DRILL         , "TYPE_DRILL"},
          {subt::ArtifactType::TYPE_DUCT          , "TYPE_DUCT"},
          {subt::ArtifactType::TYPE_ELECTRICAL_BOX, "TYPE_ELECTRICAL_BOX"},
          {subt::ArtifactType::TYPE_EXTINGUISHER  , "TYPE_EXTINGUISHER"},
          {subt::ArtifactType::TYPE_PHONE         , "TYPE_PHONE"},
          {subt::ArtifactType::TYPE_RADIO         , "TYPE_RADIO"},
          {subt::ArtifactType::TYPE_RESCUE_RANDY  , "TYPE_RESCUE_RANDY"},
          {subt::ArtifactType::TYPE_TOOLBOX       , "TYPE_TOOLBOX"},
          {subt::ArtifactType::TYPE_VALVE         , "TYPE_VALVE"},
          {subt::ArtifactType::TYPE_VENT          , "TYPE_VENT"},
          {subt::ArtifactType::TYPE_GAS           , "TYPE_GAS"}
        }
      };

  /// \brief Get the artifact enumeration from the string value.
  /// \param[in] _name - string representation of artifact
  /// \param[out] _type - enumeration representation of artifact
  public: bool ArtifactFromString(
              const std::string &_name, ArtifactType &_type);

  /// \brief Parse artifacts from an SDF file
  /// \param[in] _sdf sdf file to parse
  public: void ParseArtifacts(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Callback fired when the next service is called.
  /// \param[in] _msg Service request
  /// \param[out] _rep Service response
  public: bool OnNext(const ignition::msgs::StringMsg &_msg,
                      ignition::msgs::StringMsg &_rep);

  /// \brief Callback fired when the scan service is called.
  /// \param[in] _msg Service request
  /// \param[out] _rep Service response
  public: bool OnScan(const ignition::msgs::StringMsg &_msg,
                      ignition::msgs::StringMsg &_rep);

  /// \brief Map of artifact names to additional artifact data.
  public: std::map<std::string, Artifact> artifacts;

  /// \brief Cache of valid artifacts that have been detected.
  public: std::vector<std::string> validArtifacts;

  /// \brief Index of current artifact being validated.
  public: int index = -1;

  /// \brief Ignition transport node.
  public: transport::Node node;

  /// \brief World name of current world.
  public: std::string worldName;

  /// \brief List of level poses to check for artifacts.
  public: std::vector<ignition::math::Pose3d> posesToCheck;
};

bool ArtifactValidatorPrivate::OnScan(const ignition::msgs::StringMsg& /*_req*/,
                                      ignition::msgs::StringMsg& /*_rep*/)
{
  igndbg << "Scanning: "
    << this->posesToCheck.size() << " levels" << std::endl;

  igndbg << "Have poses for: "
    << validArtifacts.size() << "/" << artifacts.size() << std::endl;

  if (this->posesToCheck.size())
  {
    auto pose = this->posesToCheck.back();
    ignition::msgs::Pose req;
    req.set_name("validator");
    req.mutable_position()->set_x(pose.Pos().X());
    req.mutable_position()->set_y(pose.Pos().Y());
    req.mutable_position()->set_z(pose.Pos().Z());

    igndbg << pose << std::endl;

    msgs::Boolean res;
    bool result;
    unsigned int timeout = 1000;
    std::string service {"/world/" + this->worldName + "/set_pose"};
    node.Request(service, req, timeout, res, result);
    igndbg << "Result: " << res.data() << " " << result << std::endl;
    this->posesToCheck.pop_back();
  }
  return true;
}

bool ArtifactValidatorPrivate::OnNext(const ignition::msgs::StringMsg& /*_req*/,
                                      ignition::msgs::StringMsg& _rep)
{
  if (this->index == -1)
  {
    return false;
  }

  if (this->index == static_cast<int>(validArtifacts.size() - 1))
  {
    this->index = 0;
  }
  else
  {
    this->index++;
  }

  auto name = this->validArtifacts[this->index];
  ignmsg << "Moving to: " << name << std::endl;

  auto artifact = this->artifacts[name];

  ignition::msgs::Pose req;
  req.set_name("validator");
  req.mutable_position()->set_x(artifact.pose.Pos().X());
  req.mutable_position()->set_y(artifact.pose.Pos().Y());
  req.mutable_position()->set_z(artifact.pose.Pos().Z());

  _rep.set_data(name);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 1000;
  std::string service {"/world/" + this->worldName + "/set_pose"};
  node.Request(service, req, timeout, res, result);
  igndbg << "Result: " << res.data() << " " << result << std::endl;

  return true;
}

/////////////////////////////////////////////////
bool ArtifactValidatorPrivate::ArtifactFromString(const std::string &_name,
    ArtifactType &_type)
{
  auto pos = std::find_if(
    std::begin(this->kArtifactTypes),
    std::end(this->kArtifactTypes),
    [&_name](const typename std::pair<ArtifactType, std::string> &_pair)
    {
      return (std::get<1>(_pair) == _name);
    });

  if (pos == std::end(this->kArtifactTypes))
    return false;

  _type = std::get<0>(*pos);
  return true;
}

/////////////////////////////////////////////////
void ArtifactValidatorPrivate::ParseArtifacts(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
  sdf::ElementPtr artifactElem = const_cast<sdf::Element*>(
      _sdf.get())->GetElement("artifact");

  while (artifactElem)
  {
    // Sanity check: "Name" is required.
    if (!artifactElem->HasElement("name"))
    {
      ignerr << "[ArtifactValidator]: Parameter <name> not found. "
            << "Ignoring this artifact" << std::endl;
      artifactElem = artifactElem->GetNextElement("artifact");
      continue;
    }

    std::string modelName =
      artifactElem->Get<std::string>("name", "name").first;

    // Sanity check: "Type" is required.
    if (!artifactElem->HasElement("type"))
    {
      ignerr << "[ArtifactValidator]: Parameter <type> not found."
        << "Ignoring this artifact" << std::endl;
      artifactElem = artifactElem->GetNextElement("artifact");
      continue;
    }

    // Sanity check: Make sure that the artifact type is supported.
    std::string modelTypeStr = artifactElem->Get<std::string>("type",
        "type").first;
    ArtifactType modelType;
    if (!this->ArtifactFromString(modelTypeStr, modelType))
    {
      ignerr << "[ArtifactValidator]: Unknown artifact type ["
        << modelTypeStr << "]. Ignoring artifact" << std::endl;
      artifactElem = artifactElem->GetNextElement("artifact");
      continue;
    }

    // Sanity check: The artifact shouldn't be repeated.
    if (this->artifacts.find(modelName) != this->artifacts.end())
    {
        ignerr << "[ArtifactValidator]: Repeated model with name ["
          << modelName << "]. Ignoring artifact" << std::endl;
        artifactElem = artifactElem->GetNextElement("artifact");
        continue;
    }

    Artifact newArtifact;
    newArtifact.name = modelName;
    newArtifact.type = modelType;
    newArtifact.typeStr = modelTypeStr;
    ignmsg << "Adding artifact: " << newArtifact.String() << std::endl;
    this->artifacts[modelName] = newArtifact;
    artifactElem = artifactElem->GetNextElement("artifact");
  }
}

/////////////////////////////////////////////////
ArtifactValidator::ArtifactValidator()
  : dataPtr(new ArtifactValidatorPrivate)
{
}

/////////////////////////////////////////////////
ArtifactValidator::~ArtifactValidator() = default;


/////////////////////////////////////////////////
void ArtifactValidator::Configure(const ignition::gazebo::Entity & /*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->worldName = const_cast<sdf::Element*>(
      _sdf.get())->Get<std::string>("world_name", "world_name").first;

  igndbg << "Set world name: " << this->dataPtr->worldName << std::endl;

  this->dataPtr->ParseArtifacts(_sdf);

  this->dataPtr->node.Advertise("/artifact/next",
      &ArtifactValidatorPrivate::OnNext, this->dataPtr.get());

  this->dataPtr->node.Advertise("/artifact/scan",
      &ArtifactValidatorPrivate::OnScan, this->dataPtr.get());

  _ecm.Each<components::Level,
            components::LevelEntityNames,
            components::Pose>(
              [&](const Entity& ,
                  const components::Level*,
                  const components::LevelEntityNames *_entities,
                  const components::Pose *_pose) -> bool
              {
                for (const auto & name : _entities->Data())
                {
                  auto it = this->dataPtr->artifacts.find(name);
                  if (it != this->dataPtr->artifacts.end())
                  {
                    this->dataPtr->posesToCheck.push_back(_pose->Data());
                  }
                }

                return true;
              });

  igndbg << "Found artifacts: "
    << this->dataPtr->artifacts.size() << std::endl;
  igndbg << "Found levels to check: "
    << this->dataPtr->posesToCheck.size() << std::endl;
}

//////////////////////////////////////////////////
void ArtifactValidator::PostUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  _ecm.EachNew<gazebo::components::Model,
               gazebo::components::Name,
               gazebo::components::Pose>(
      [&](const gazebo::Entity &,
          const gazebo::components::Model *,
          const gazebo::components::Name *_nameComp,
          const gazebo::components::Pose *_poseComp) -> bool
      {
        auto artifact = this->dataPtr->artifacts.find(_nameComp->Data());

        if (artifact != this->dataPtr->artifacts.end())
        {
          igndbg << "Checking: " << _nameComp->Data() << std::endl;

          auto it = std::find(this->dataPtr->validArtifacts.begin(),
                    this->dataPtr->validArtifacts.end(),
                    _nameComp->Data());

          if (it != this->dataPtr->validArtifacts.end())
          {
            igndbg << "Skipping: " << _nameComp->Data() << "already found";
            igndbg << "Have poses for: "
              << this->dataPtr->validArtifacts.size() << "/"
              << this->dataPtr->artifacts.size() << std::endl;
            return true;
          }

          artifact->second.pose = _poseComp->Data();
          this->dataPtr->validArtifacts.push_back(_nameComp->Data());
          if (this->dataPtr->index == -1)
          {
            this->dataPtr->index = 0;
          }
          igndbg << "Updated artifact: "
            << artifact->second.String() << std::endl;

          igndbg << "Have poses for: "
            << this->dataPtr->validArtifacts.size() << "/"
            << this->dataPtr->artifacts.size() << std::endl;
        }
        return true;
      });
}
