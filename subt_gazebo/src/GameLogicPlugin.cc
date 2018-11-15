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

#include <std_msgs/Int32.h>
#include <algorithm>
#include <functional>
#include <utility>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/GameLogicPlugin.hh"
#include "subt_gazebo/protobuf/artifact.pb.h"

using namespace gazebo;
using namespace subt;

GZ_REGISTER_WORLD_PLUGIN(GameLogicPlugin)

/////////////////////////////////////////////////
void GameLogicPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Default log path is /dev/null.
  std::string logPath = "/dev/null";

  // Check if the game logic plugin has a <logging> element.
  // The <logging> element can contain a <filename_prefix> child element.
  // The <filename_prefix> is used to specify the log filename prefix. For
  // example:
  // <logging>
  //   <filename_prefix>subt_tunnel_qual</filename_prefix>
  // </logging>
  if (_sdf->HasElement("logging"))
  {
    sdf::ElementPtr loggingElem = _sdf->GetElement("logging");

    // Get the log filename prefix.
    std::string filenamePrefix = loggingElem->Get<std::string>(
        "filename_prefix", "subt").first;

    // Make sure that we can access the HOME environment variable.
    char *homePath = getenv("HOME");
    if (!homePath)
    {
      gzerr << "Unable to get HOME environment variable. Report this error to "
        << "https://bitbucket.org/osrf/subt/issues/new. "
        << "SubT logging will be disabled.\n";
    }
    else
    {
      // Construct the final log filename.
      logPath = homePath;
      logPath += "/" + filenamePrefix + "_" +
        common::Time::GetWallTimeAsISOString() + ".log";
    }
  }

  // Open the log file.
  this->logStream.open(logPath.c_str(), std::ios::out);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM(
      "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so'"
      << " in the gazebo_ros package)");
    return;
  }

  GZ_ASSERT(_world, "GameLogicPlugin world pointer is NULL");
  this->world = _world;

  // We save the pose of the fiducial in the staging area for scoring.
  auto artifactOriginModel = this->world->ModelByName("artifact_origin");
  if (!artifactOriginModel)
  {
    gzerr << "Unable to find [artifact_origin] model" << std::endl;
    return;
  }
  this->artifactOriginPose = artifactOriginModel->WorldPose();

  // Parse the artifacts.
  this->ParseArtifacts(_sdf);

  // Initialize the ROS node.
  this->rosnode.reset(new ros::NodeHandle("subt"));

  // Advertise the service to receive artifact reports.
  // Note that we're setting the scope to this service to SCOPE_T, so only
  // nodes within the same process will be able to reach this plugin.
  // The reason for this is to avoid the teams to use this service directly.
  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::PROCESS);
  this->node.Advertise(kNewArtifactSrv,
    &GameLogicPlugin::OnNewArtifact, this, opts);

  this->scorePub = this->rosnode->advertise<std_msgs::Int32>("score", 1000);

  // Publish the score.
  this->scoreTimer = this->rosnode->createTimer(
    ros::Duration(1.0), &GameLogicPlugin::PublishScore, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GameLogicPlugin::OnUpdate, this));

  // Gazebo transport
  this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gzNode->Init();

  this->startCollisionSub = this->gzNode->Subscribe("/subt/start/touched",
    &GameLogicPlugin::OnStartCollision, this);

  // ROS service to receive a command to finish the game.
  ros::NodeHandle n;
  this->finishService = n.advertiseService(
    "/subt/finish", &GameLogicPlugin::OnFinishCall, this);

  gzmsg << "Starting SubT" << std::endl;
  this->Log() << "starting" << std::endl;
}

/////////////////////////////////////////////////
void GameLogicPlugin::ParseArtifacts(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr artifactElem = _sdf->GetElement("artifact");
  while (artifactElem)
  {
    // Sanity check: "Name" is required.
    if (!artifactElem->HasElement("name"))
    {
      gzerr << "[GameLogicPlugin]: Parameter <name> not found. Ignoring this "
            << "artifact" << std::endl;
      this->Log() << "error Parameter <name> not found. Ignoring this artifact"
        << std::endl;
      artifactElem = _sdf->GetNextElement("artifact");
      continue;
    }
    std::string modelName = artifactElem->Get<std::string>("name");

    // Sanity check: "Type" is required.
    if (!artifactElem->HasElement("type"))
    {
      gzerr << "[GameLogicPlugin]: Parameter <type> not found. Ignoring this "
            << "artifact" << std::endl;
      this->Log() << "error Parameter <type> not found. Ignoring this artifact"
        << std::endl;

      artifactElem = _sdf->GetNextElement("artifact");
      continue;
    }

    // Sanity check: The model should exist.
    physics::ModelPtr modelPtr = this->world->ModelByName(modelName);
    if (!modelPtr)
    {
       gzerr << "[GameLogicPlugin]: Unable to find model with name ["
             << modelName << "]. Ignoring artifact" << std::endl;
      this->Log() << "error Unable to find model with name ["
             << modelName << "]. Ignoring artifact" << std::endl;
       artifactElem = _sdf->GetNextElement("artifact");
       continue;
    }

    // Sanity check: Make sure that the artifact type is supported.
    std::string modelTypeStr = artifactElem->Get<std::string>("type");
    ArtifactType modelType;
    if (!this->ArtifactFromString(modelTypeStr, modelType))
    {
      gzerr << "[GameLogicPlugin]: Unknown artifact type ["
             << modelTypeStr << "]. Ignoring artifact" << std::endl;
      this->Log() << "error Unknown artifact type ["
             << modelTypeStr << "]. Ignoring artifact" << std::endl;
      artifactElem = _sdf->GetNextElement("artifact");
      continue;
    }

    // Sanity check: The artifact shouldn't be repeated.
    if (this->artifacts.find(modelType) != this->artifacts.end())
    {
      const auto &modelNames = this->artifacts[modelType];
      if (modelNames.find(modelName) != modelNames.end())
      {
        gzerr << "[GameLogicPlugin]: Repeated model with name ["
               << modelName << "]. Ignoring artifact" << std::endl;
        this->Log() << "error Repeated model with name ["
               << modelName << "]. Ignoring artifact" << std::endl;
        artifactElem = _sdf->GetNextElement("artifact");
        continue;
      }
    }

    this->artifacts[modelType][modelName] = modelPtr;
    artifactElem = artifactElem->GetNextElement("artifact");
  }
}

/////////////////////////////////////////////////
void GameLogicPlugin::OnNewArtifact(const subt::msgs::Artifact &_req)
{
  this->Log() << "new_artifact_reported" << std::endl;

  ArtifactType artifactType;
  if (!this->ArtifactFromInt(_req.type(), artifactType))
  {
    gzerr << "Unknown artifact code. The number should be between 0 and "
          << this->kArtifactTypes.size() - 1 << " but we received "
          << _req.type() << std::endl;

    this->Log() << "error Unknown artifact code. The number should be between "
      << "0 and " << this->kArtifactTypes.size() - 1
      << " but we received " << _req.type() << std::endl;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->totalScore += this->ScoreArtifact(artifactType, _req.pose());
    gzmsg << "Total score: " << this->totalScore << std::endl << std::endl;
    this->Log() << "new_total_score "
      << this->totalScore << std::endl;
  }
}

/////////////////////////////////////////////////
double GameLogicPlugin::ScoreArtifact(const ArtifactType &_type,
  const ignition::msgs::Pose &_pose)
{
  // Sanity check: Make sure that we have crossed the starting gate.
  if (!this->started)
  {
    gzmsg << "  The task hasn't started yet" << std::endl;
    this->Log() << "task_not_started" << std::endl;
    return 0.0;
  }

  // Sanity check: Make sure that we still have artifacts.
  if (this->artifacts.find(_type) == this->artifacts.end())
  {
    gzmsg << "  No artifacts remaining" << std::endl;
    this->Log() << "no_remaining_artifacts_of_specified_type" << std::endl;
    return 0.0;
  }

  // The teams are reporting the artifact poses relative to the fiducial located
  //in the staging area. Now, we convert the reported pose to world coordinates.
  ignition::math::Pose3d artifactPose = ignition::msgs::Convert(_pose);
  ignition::math::Pose3d pose = artifactPose - this->artifactOriginPose;

  auto &potentialArtifacts = this->artifacts[_type];

  // From the list of potential artifacts, find out which one is
  // closer (Euclidean distance) to the located by this request.
  ignition::math::Vector3d observedObjectPose = pose.Pos();
  std::tuple<std::string, ignition::math::Vector3d, double> minDistance =
    {"", ignition::math::Vector3d(), std::numeric_limits<double>::infinity()};
  for (auto const object : potentialArtifacts)
  {
    auto objName = object.first;
    auto objPosition = object.second->WorldPose().Pos();
    double distance = observedObjectPose.Distance(objPosition);

    if (distance < std::get<2>(minDistance))
    {
      std::get<0>(minDistance) = objName;
      std::get<1>(minDistance) = objPosition;
      std::get<2>(minDistance) = distance;
    }
  }

  const double kBaseValue = 1.0;
  double score = kBaseValue;

  // Calculate the score based on accuracy in the location.
  double distance = std::get<2>(minDistance);
  if (distance < 0.5)
  {
    gzmsg << "  [Distance bonus]: x3" << std::endl;
    this->Log() << "distance_bonus_x3" << std::endl;
    score *= 3;
  }
  else if (distance < 2.0)
  {
    gzmsg << "  [Distance bonus]: x2" << std::endl;
    this->Log() << "distance_bonus_x2" << std::endl;
    score *= 2;
  }
  else if (distance < 4.0)
  {
    gzmsg << "  [Distance bonus]: x1" << std::endl;
    this->Log() << "distance_bonus_x1" << std::endl;
    score *= 1;
  }
  else
  {
    gzmsg << "  [Distance bonus]: -1" << std::endl;
    this->Log() << "distance_bonus_-1" << std::endl;
    score += -1;
  }

  // Apply factors based on the time since the start of the run.
  auto now = std::chrono::steady_clock::now();
  auto elapsedSecs = std::chrono::duration_cast<std::chrono::seconds>(
    now - this->startTime).count();
  if (elapsedSecs < 60 * 20)
  {
    gzmsg << "  [Time bonus]: x3" << std::endl;
    this->Log() << "time_bonus_x3" << std::endl;
    score *= 3.0;
  }
  else if (elapsedSecs < 60 * 40)
  {
    gzmsg << "  [Time bonus]: x2" << std::endl;
    this->Log() << "time_bonus_x2" << std::endl;
    score *= 2.0;
  }
  else
  {
    gzmsg << "  [Time bonus]: x1" << std::endl;
    this->Log() << "time_bonus_x1" << std::endl;
    score *= 1.0;
  }

  gzmsg << "  [Total]: " << score << std::endl;
  this->Log() << "modified_score " << score << std::endl;

  // Remove this artifact to avoid getting score from the same artifact
  // multiple times.
  potentialArtifacts.erase(std::get<0>(minDistance));
  if (potentialArtifacts.empty())
    this->artifacts.erase(_type);

  return score;
}

/////////////////////////////////////////////////
void GameLogicPlugin::PublishScore(const ros::TimerEvent &/*_event*/)
{
  std_msgs::Int32 msg;

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->started)
      return;

    msg.data = this->totalScore;
    this->scorePub.publish(msg);
  }
}

/////////////////////////////////////////////////
bool GameLogicPlugin::ArtifactFromString(const std::string &_name,
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
bool GameLogicPlugin::ArtifactFromInt(const uint32_t &_typeInt,
  ArtifactType &_type)
{
  if (_typeInt > this->kArtifactTypes.size())
    return false;

  _type = static_cast<ArtifactType>(_typeInt);
  return true;
}

/////////////////////////////////////////////////
void GameLogicPlugin::OnUpdate()
{
}

/////////////////////////////////////////////////
void GameLogicPlugin::OnStartCollision(ConstIntPtr &/*_msg*/)
{
  if (this->started)
    return;

  this->started = true;
  this->startTime = std::chrono::steady_clock::now();
  gzmsg << "Scoring has Started" << std::endl;
  this->Log() << "scoring_started" << std::endl;
}

/////////////////////////////////////////////////
bool GameLogicPlugin::OnFinishCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  _res.success = false;
  if (this->started && !this->finished && _req.data)
  {
    this->finished = true;
    this->finishTime = std::chrono::steady_clock::now();

    auto elapsed = this->finishTime - this->startTime;
    gzmsg << "Scoring has finished. Elapsed time: "
          << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
          << " seconds" << std::endl;
    _res.success = true;
    this->Log() << "finished_elapsed_time "
      << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
      << " s." << std::endl;
    this->Log() << "finished_score " << this->totalScore << std::endl;
    this->logStream.flush();
  }
  return true;
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPlugin::Log()
{
  this->logStream << this->world->SimTime().sec
    << " " << this->world->SimTime().nsec << " ";
  return this->logStream;
}
