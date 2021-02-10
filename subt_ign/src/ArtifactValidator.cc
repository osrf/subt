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


/*

Use the following to iterate the through the artifacts.
The service will respond with the artifact name that you are viewing.
You can use the GUI to look through the two cameras (one top-down, one at a 45 degree angle).
Hint: Using the "follow" functionality in the GUI is very helpful here.

ign service -s /artifact/next \
  --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.StringMsg \
  --timeout 1000 \
  --req 'data: ""'

ign service -s /artifact/prev \
  --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.StringMsg \
  --timeout 1000 \
  --req 'data: ""'

ign service -s /artifact/move_to \
  --reqtype ignition.msgs.StringMsg \
  --reptype ignition.msgs.StringMsg \
  --timeout 1000 \
  --req 'data: "artifact_origin"'

*/

#include "subt_ign/ArtifactValidator.hh"

#include <ignition/common/Console.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/StringUtils.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Level.hh>
#include <ignition/gazebo/components/LevelEntityNames.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/transport/Node.hh>

#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/World.hh>

#include "subt_ign/Common.hh"

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

/// \brief Private data for the artifact validator.
class subt::ArtifactValidatorPrivate
{
  /// \brief Parse artifacts from the SDF root.
  public: void ParseArtifacts();

  /// \brief Callback fired when the move_to service is called.
  /// \param[in] _msg Service request
  /// \param[out] _rep Service response
  public: bool MoveTo(const ignition::msgs::StringMsg &_msg,
                      ignition::msgs::StringMsg &_rep);

  /// \brief Convenience method for calling the move_to service with a string
  /// \param[in] _msg artifact to move to
  /// \param[out] _rep Service response
  public: bool MoveToString(const std::string &_msg,
                      ignition::msgs::StringMsg &_rep);

  /// \brief Callback fired when the next service is called.
  /// \param[in] _msg Service request
  /// \param[out] _rep Service response
  public: bool OnNext(const ignition::msgs::StringMsg &_msg,
                      ignition::msgs::StringMsg &_rep);

  /// \brief Callback fired when the next service is called.
  /// \param[in] _msg Service request
  /// \param[out] _rep Service response
  public: bool OnPrev(const ignition::msgs::StringMsg &_msg,
                      ignition::msgs::StringMsg &_rep);

  /// \brief Map of artifact names to additional artifact data.
  public: std::map<std::string, Artifact> artifacts;

  /// \brief Iterator to current position in map
  public: std::map<std::string, Artifact>::iterator artifactsIter;

  /// \brief The SDF root object.
  public: sdf::Root sdfRoot;

  /// \brief Ignition transport node.
  public: transport::Node node;

  /// \brief World name of current world.
  public: std::string worldName;
};

/////////////////////////////////////////////////
bool ArtifactValidatorPrivate::MoveTo(const ignition::msgs::StringMsg &_msg,
                                      ignition::msgs::StringMsg &_rep)
{
  ignmsg << "Moving to: " << _msg.data() << std::endl;

  auto newIter = this->artifacts.find(_msg.data());
  if (newIter == this->artifacts.end())
  {
    ignerr << "Artifact: " << _msg.data() << " does not exist" << std::endl;
    return false;
  }
  this->artifactsIter = newIter;
  auto artifact = this->artifactsIter->second;

  ignition::msgs::Pose req;
  req.set_name("validator");
  req.mutable_position()->set_x(artifact.pose.Pos().X());
  req.mutable_position()->set_y(artifact.pose.Pos().Y());
  req.mutable_position()->set_z(artifact.pose.Pos().Z());
  _rep.set_data(_msg.data());

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 1000;
  std::string service {"/world/" + this->worldName + "/set_pose"};
  node.Request(service, req, timeout, res, result);
  igndbg << "Result: " << res.data() << " " << result << std::endl;
  return result;
}

/////////////////////////////////////////////////
bool ArtifactValidatorPrivate::MoveToString(const std::string &_name,
                                      ignition::msgs::StringMsg &_rep)
{
  ignition::msgs::StringMsg msg;
  msg.set_data(_name);
  return this->MoveTo(msg, _rep);
}

/////////////////////////////////////////////////
bool ArtifactValidatorPrivate::OnNext(const ignition::msgs::StringMsg& /*_req*/,
                                      ignition::msgs::StringMsg& _rep)
{
  this->artifactsIter++;

  if (this->artifactsIter == this->artifacts.end())
  {
    this->artifactsIter = this->artifacts.begin();
  }

  return this->MoveToString(this->artifactsIter->first, _rep);
}

/////////////////////////////////////////////////
bool ArtifactValidatorPrivate::OnPrev(const ignition::msgs::StringMsg& /*_req*/,
                                      ignition::msgs::StringMsg &_rep)
{
  this->artifactsIter--;

  auto ret = this->MoveToString(this->artifactsIter->first, _rep);

  if (this->artifactsIter == this->artifacts.begin())
  {
    this->artifactsIter = this->artifacts.end();
    this->artifactsIter--;
  }
  return ret;
}

/////////////////////////////////////////////////
void ArtifactValidatorPrivate::ParseArtifacts()
{
  // Assuming 1 world per SDF, which is true for SubT.
  auto world = this->sdfRoot.WorldByIndex(0);

  for (uint64_t modelIndex = 0; modelIndex < world->ModelCount(); ++modelIndex)
  {
    auto model = world->ModelByIndex(modelIndex);
    auto pose = model->RawPose();
    auto name = model->Name();

    if (name == "artifact_origin")
    {
      Artifact newArtifact;
      newArtifact.name = name;
      newArtifact.pose = pose;
      artifacts[name] = newArtifact;
    }

    for (auto [str, type] : kArtifactNames)
    {
      if (name.find(str) != std::string::npos)
      {
        Artifact newArtifact;
        newArtifact.name = name;
        newArtifact.type = type;
        StringFromArtifact(type, newArtifact.typeStr);
        newArtifact.pose = pose;
        artifacts[name] = newArtifact;
      }
    }
  }

  for (auto artifact : this->artifacts)
  {
    igndbg << artifact.second.String() << "\n";
  }

  this->artifactsIter = this->artifacts.end();
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
    ignition::gazebo::EntityComponentManager & /*_ecm*/,
    ignition::gazebo::EventManager & /*_eventMgr*/)
{
  auto worldName = const_cast<sdf::Element*>(
      _sdf.get())->Get<std::string>("world_name", "world_name").first;

  if (worldName.empty())
  {
    ignerr << "File path not available.\n";
    return;
  }

  this->dataPtr->worldName = worldName;

  std::string fullPath;
  subt::FullWorldPath(worldName, fullPath);

  common::SystemPaths systemPaths;
  systemPaths.SetFilePathEnv("IGN_GAZEBO_RESOURCE_PATH");
  systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);
  std::string filePath = systemPaths.FindFile(fullPath);
  ignmsg << "Loading SDF world file[" << filePath << "].\n";

  auto errors = this->dataPtr->sdfRoot.Load(filePath);

  if (!errors.empty())
  {
    for (auto &err : errors)
      ignerr << err << "\n";
    return;
  }

  this->dataPtr->ParseArtifacts();

  this->dataPtr->node.Advertise("/artifact/move_to",
      &ArtifactValidatorPrivate::MoveTo, this->dataPtr.get());
  this->dataPtr->node.Advertise("/artifact/next",
      &ArtifactValidatorPrivate::OnNext, this->dataPtr.get());
  this->dataPtr->node.Advertise("/artifact/prev",
      &ArtifactValidatorPrivate::OnPrev, this->dataPtr.get());
}

/////////////////////////////////////////////////
void ArtifactValidator::PostUpdate(
                const ignition::gazebo::UpdateInfo &/*_info*/,
                const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  if (this->dataPtr->artifactsIter == this->dataPtr->artifacts.end())
  {
    this->dataPtr->artifactsIter = this->dataPtr->artifacts.begin();
    ignition::msgs::StringMsg rep;
    this->dataPtr->MoveToString(this->dataPtr->artifactsIter->first, rep);
  }
}

