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

#include <tinyxml2.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <ignition/msgs/float.pb.h>
#include <subt_msgs/PoseFromArtifact.h>

#include <chrono>
#include <map>
#include <mutex>
#include <utility>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "subt_ign/CommonTypes.hh"
#include "subt_ign/GameLogicPlugin.hh"
#include "subt_ign/protobuf/artifact.pb.h"

using namespace ignition;
using namespace subt;

class subt::GameLogicPluginPrivate
{
  /// \brief Mapping between enum types and strings.
  public: const std::array<
      const std::pair<subt::ArtifactType, std::string>, 10> kArtifactTypes
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
          {subt::ArtifactType::TYPE_VALVE         , "TYPE_VALVE"}
        }
      };

  /// \brief Write a simulation timestamp to a logfile.
  /// \return A file stream that can be used to write additional
  /// information to the logfile.
  public: std::ofstream &Log();

  // Configure ros functionality
  public: void SetupRos();

  /// \brief Handle gazebo pose messages
  /// \param[in] _msg New set of poses.
  public: void OnPose(const ignition::msgs::Pose_V &_msg);

  /// \brief Create an ArtifactType from a string.
  /// \param[in] _name The artifact in string format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  public: bool ArtifactFromString(const std::string &_name,
                                  subt::ArtifactType &_type);

  /// \brief Calculate the score of a new artifact request.
  /// \param[in] _type The object type. See ArtifactType.
  /// \param[in] _pose The object pose.
  /// \return The score obtained for this object.
  public: double ScoreArtifact(const subt::ArtifactType &_type,
                               const ignition::msgs::Pose &_pose);

  /// \brief Create an ArtifactType from an integer.
  //
  /// \param[in] _typeInt The artifact in int format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  public: bool ArtifactFromInt(const uint32_t &_typeInt,
                               subt::ArtifactType &_type);

  /// \brief Callback executed to process a new artifact request
  /// sent by a team.
  /// \param[in] _req The service request.
  public: void OnNewArtifact(const subt::msgs::Artifact &_req);

  /// \brief Parse all the artifacts.
  /// \param[in] _sdf The SDF element containing the artifacts.
  public: void ParseArtifacts(const tinyxml2::XMLElement *_elem);

  /// \brief Publish the score.
  /// \param[in] _event Unused.
  public: void PublishScore();

  public: void Finish();

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in]  _req The message containing the robot name.
  /// \param[out] _res The response message.
  /// \return true on success.
  public: bool OnPoseFromArtifact(
               const ignition::msgs::StringMsg &_req,
               ignition::msgs::Pose &_res);


  private: bool PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  private: bool OnStartCall(std_srvs::SetBool::Request &_req,
                            std_srvs::SetBool::Response &_res);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  private: bool OnFinishCall(std_srvs::SetBool::Request &_req,
                             std_srvs::SetBool::Response &_res);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing the robot name.
  /// \param[out] _res The response message.
  private: bool OnPoseFromArtifactRos(
               subt_msgs::PoseFromArtifact::Request &_req,
               subt_msgs::PoseFromArtifact::Response &_res);

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Current simulation time.
  public: ignition::msgs::Time simTime;

  /// \brief Thread on which scores are published
  public: std::unique_ptr<std::thread> publishThread = nullptr;

  /// \brief Whether the task has started.
  public: bool started = false;

  /// \brief Whether the task has finished.
  public: bool finished = false;

  /// \brief Start time used for scoring.
  public: std::chrono::steady_clock::time_point startTime;

  /// \brief Finish time used for scoring.
  public: std::chrono::steady_clock::time_point finishTime;

  /// \brief Store all artifacts.
  /// The key is the object type. See ArtifactType for all supported values.
  /// The value is another map, where the key is the model name and the value
  /// is a Model pointer.
  public: std::map<subt::ArtifactType,
                    std::map<std::string, ignition::math::Pose3d>> artifacts;

  public: std::map<std::string, ignition::math::Pose3d> poses;

  /// \brief Total score.
  public: double totalScore = 0.0;

  /// \brief A mutex.
  public: std::mutex mutex;

  /// \brief Log file output stream.
  public: std::ofstream logStream;

  /// \brief The pose of the object marking the origin of the artifacts.
  public: ignition::math::Pose3d artifactOriginPose;

  /// \brief The ROS node handler used for communications.
  public: std::unique_ptr<ros::NodeHandle> rosnode;

  /// \brief ROS service to receive a call to finish the game.
  public: ros::ServiceServer finishServiceRos;

  /// \brief ROS service to receive a call to start the game.
  public: ros::ServiceServer startServiceRos;

  /// \brief ROS service server to receive the location of a robot relative to
  /// the origin artifact.
  public: ros::ServiceServer poseFromArtifactServiceRos;

  /// \brief A ROS asynchronous spinner.
  public: std::unique_ptr<ros::AsyncSpinner> spinner;
};

//////////////////////////////////////////////////
GameLogicPlugin::GameLogicPlugin()
  : dataPtr(new GameLogicPluginPrivate)
{
}

//////////////////////////////////////////////////
GameLogicPlugin::~GameLogicPlugin()
{
  this->dataPtr->finished = true;
  if (this->dataPtr->publishThread)
    this->dataPtr->publishThread->join();
}

//////////////////////////////////////////////////
bool GameLogicPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  // Default log path is /dev/null.
  std::string logPath = "/dev/null";

  // Check if the game logic plugin has a <logging> element.
  // The <logging> element can contain a <filename_prefix> child element.
  // The <filename_prefix> is used to specify the log filename prefix. For
  // example:
  // <logging>
  //   <path>/tmp</path>
  //   <filename_prefix>subt_tunnel_qual</filename_prefix>
  // </logging>
  const tinyxml2::XMLElement *loggingElem = _elem->FirstChildElement("logging");
  const tinyxml2::XMLElement *fileElem = nullptr;
  if (loggingElem &&
      (fileElem = loggingElem->FirstChildElement("filename_prefix")))
  {
    // Get the log filename prefix.
    std::string filenamePrefix = fileElem->GetText();
    const tinyxml2::XMLElement *pathElem =
      loggingElem->FirstChildElement("path");

    // Get the logpath from the <path> element, if it exists.
    if (pathElem)
    {
      logPath = pathElem->GetText();
    }
    else
    {
      // Make sure that we can access the HOME environment variable.
      char *homePath = getenv("HOME");
      if (!homePath)
      {
        ignerr << "Unable to get HOME environment variable. Report this error to "
          << "https://bitbucket.org/osrf/subt/issues/new. "
          << "SubT logging will be disabled.\n";
      }
      else
      {
        logPath = homePath;
      }
    }

    // Construct the final log filename.
    logPath += "/" + filenamePrefix + "_" +
      ignition::common::systemTimeISO() + ".log";
  }

  // Open the log file.
  this->dataPtr->logStream.open(logPath.c_str(), std::ios::out);

  this->dataPtr->SetupRos();

  // Advertise the service to receive artifact reports.
  // Note that we're setting the scope to this service to SCOPE_T, so only
  // nodes within the same process will be able to reach this plugin.
  // The reason for this is to avoid the teams to use this service directly.
  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::PROCESS);
  this->dataPtr->node.Advertise(kNewArtifactSrv,
    &GameLogicPluginPrivate::OnNewArtifact, this->dataPtr.get(), opts);

  this->dataPtr->ParseArtifacts(_elem);

  const tinyxml2::XMLElement *worldNameElem =
    _elem->FirstChildElement("world_name");
  std::string worldName = "default";
  if (worldNameElem)
  {
    worldName = worldNameElem->GetText();
  }
  else
  {
    ignerr << "Missing <world_name>, the GameLogicPlugin will assume a "
      << " world name of 'default'. This could lead to incorrect scoring\n";
  }

  // Subscribe to pose messages. We will pull out model and artifact
  // information from the published message.
  this->dataPtr->node.Subscribe("/world/" + worldName + "/pose/info",
      &GameLogicPluginPrivate::OnPose, this->dataPtr.get());

  this->dataPtr->node.Advertise("/subt/pose_from_artifact_origin",
      &GameLogicPluginPrivate::OnPoseFromArtifact, this->dataPtr.get());

  this->dataPtr->publishThread.reset(new std::thread(
        &GameLogicPluginPrivate::PublishScore, this->dataPtr.get()));

  this->dataPtr->spinner.reset(new ros::AsyncSpinner(1));
  this->dataPtr->spinner->start();

  ignmsg << "Starting SubT" << std::endl;

  return true;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::OnPose(const ignition::msgs::Pose_V &_msg)
{
  // Store sim time if present
  if (_msg.has_header() && _msg.header().has_stamp())
    this->simTime = _msg.header().stamp();

  // Check the all the published poses
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    const ignition::msgs::Pose &pose = _msg.pose(i);
    this->poses[pose.name()] = ignition::msgs::Convert(pose);

    // Update artifact positions.
    for (std::pair<const subt::ArtifactType,
         std::map<std::string, ignition::math::Pose3d>> &artifactPair :
         this->artifacts)
    {
      for (std::pair<const std::string, ignition::math::Pose3d> &artifact :
          artifactPair.second)
      {
        if (artifact.first == pose.name())
        {
          artifact.second = ignition::msgs::Convert(pose);
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::OnNewArtifact(const subt::msgs::Artifact &_req)
{
  this->Log() << "new_artifact_reported" << std::endl;

  ArtifactType artifactType;
  if (!this->ArtifactFromInt(_req.type(), artifactType))
  {
    ignerr << "Unknown artifact code. The number should be between 0 and "
          << this->kArtifactTypes.size() - 1 << " but we received "
          << _req.type() << std::endl;

    this->Log() << "error Unknown artifact code. The number should be between "
                << "0 and " << this->kArtifactTypes.size() - 1
                << " but we received " << _req.type() << std::endl;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->totalScore += this->ScoreArtifact(
        artifactType, _req.pose());
    ignmsg << "Total score: " << this->totalScore << std::endl;
    this->Log() << "new_total_score " << this->totalScore << std::endl;
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::ArtifactFromInt(const uint32_t &_typeInt,
    ArtifactType &_type)
{
  if (_typeInt > this->kArtifactTypes.size())
    return false;

  _type = static_cast<ArtifactType>(_typeInt);
  return true;
}

/////////////////////////////////////////////////
double GameLogicPluginPrivate::ScoreArtifact(const ArtifactType &_type,
  const ignition::msgs::Pose &_pose)
{
  // Sanity check: Make sure that we have crossed the starting gate.
  if (!this->started)
  {
    ignmsg << "  The task hasn't started yet" << std::endl;
    this->Log() << "task_not_started" << std::endl;
    return 0.0;
  }

  // Sanity check: Make sure that we still have artifacts.
  if (this->artifacts.find(_type) == this->artifacts.end())
  {
    ignmsg << "  No artifacts remaining" << std::endl;
    this->Log() << "no_remaining_artifacts_of_specified_type" << std::endl;
    return 0.0;
  }

  // The teams are reporting the artifact poses relative to the fiducial located
  // in the staging area. Now, we convert the reported pose to world coordinates
  ignition::math::Pose3d artifactPose = ignition::msgs::Convert(_pose);
  ignition::math::Pose3d pose = artifactPose +
    this->artifactOriginPose;

  double score = 0.0;
  std::map<std::string, ignition::math::Pose3d> &potentialArtifacts =
    this->artifacts[_type];

  // From the list of potential artifacts, find out which one is
  // closer (Euclidean distance) to the located by this request.
  ignition::math::Vector3d observedObjectPose = pose.Pos();
  std::tuple<std::string, ignition::math::Vector3d, double> minDistance =
    {"", ignition::math::Vector3d(), std::numeric_limits<double>::infinity()};
  for (const std::pair<std::string, ignition::math::Pose3d> &object :
       potentialArtifacts)
  {
    double distance = observedObjectPose.Distance(object.second.Pos());

    if (distance < std::get<2>(minDistance))
    {
      std::get<0>(minDistance) = object.first;
      std::get<1>(minDistance) = object.second.Pos();
      std::get<2>(minDistance) = distance;
    }
  }

  // Calculate the score based on accuracy in the location.
  if (std::get<2>(minDistance) < 5)
  {
    score = 1.0;
    // Remove this artifact to avoid getting score from the same artifact
    // multiple times.
    potentialArtifacts.erase(std::get<0>(minDistance));
    if (potentialArtifacts.empty())
      this->artifacts.erase(_type);
  }

  ignmsg << "  [Total]: " << score << std::endl;
  this->Log() << "modified_score " << score << std::endl;

  return score;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::ArtifactFromString(const std::string &_name,
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
void GameLogicPluginPrivate::ParseArtifacts(const tinyxml2::XMLElement *_elem)
{
  const tinyxml2::XMLElement *artifactElem =
    _elem->FirstChildElement("artifact");
  while (artifactElem)
  {
    const tinyxml2::XMLElement *nameElem =
      artifactElem->FirstChildElement("name");

    // Sanity check: "Name" is required.
    if (!nameElem)
    {
      ignerr << "[GameLogicPlugin]: Parameter <name> not found. Ignoring this "
            << "artifact" << std::endl;
      this->Log() << "error Parameter <name> not found. Ignoring this artifact"
                  << std::endl;
      artifactElem = artifactElem->NextSiblingElement("artifact");
      continue;
    }
    std::string modelName = nameElem->GetText();

    const tinyxml2::XMLElement *typeElem =
      artifactElem->FirstChildElement("type");
    // Sanity check: "Type" is required.
    if (!typeElem)
    {
      ignerr << "[GameLogicPlugin]: Parameter <type> not found. Ignoring this "
        << "artifact" << std::endl;
      this->Log() << "error Parameter <type> not found. Ignoring this artifact"
                  << std::endl;

      artifactElem = artifactElem->NextSiblingElement("artifact");
      continue;
    }

    // Sanity check: Make sure that the artifact type is supported.
    std::string modelTypeStr = typeElem->GetText();
    ArtifactType modelType;
    if (!this->ArtifactFromString(modelTypeStr, modelType))
    {
      ignerr << "[GameLogicPlugin]: Unknown artifact type ["
        << modelTypeStr << "]. Ignoring artifact" << std::endl;
      this->Log() << "error Unknown artifact type ["
                  << modelTypeStr << "]. Ignoring artifact" << std::endl;
      artifactElem = artifactElem->NextSiblingElement("artifact");
      continue;
    }

    // Sanity check: The artifact shouldn't be repeated.
    if (this->artifacts.find(modelType) != this->artifacts.end())
    {
      const std::map<std::string, ignition::math::Pose3d> &modelNames =
        this->artifacts[modelType];

      if (modelNames.find(modelName) != modelNames.end())
      {
        ignerr << "[GameLogicPlugin]: Repeated model with name ["
          << modelName << "]. Ignoring artifact" << std::endl;
        this->Log() << "error Repeated model with name ["
                    << modelName << "]. Ignoring artifact" << std::endl;
        artifactElem = artifactElem->NextSiblingElement("artifact");
        continue;
      }
    }

    ignmsg << "Adding artifact name[" << modelName << "] type["
      << modelTypeStr << "]\n";
    this->artifacts[modelType][modelName] = ignition::math::Pose3d::Zero;
        artifactElem = artifactElem->NextSiblingElement("artifact");
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishScore()
{
  transport::Node::Publisher scorePub =
    this->node.Advertise<ignition::msgs::Float>("/subt/score");
  ros::Publisher rosScorePub =
    this->rosnode->advertise<std_msgs::Int32>("score", 1000);

  ignition::msgs::Float msg;
  std_msgs::Int32 rosMsg;

  while (!this->finished)
  {
    msg.set_data(this->totalScore);
    rosMsg.data = this->totalScore;

    scorePub.Publish(msg);
    rosScorePub.publish(rosMsg);
    IGN_SLEEP_S(1);
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::SetupRos()
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "ignition", ros::init_options::NoSigintHandler);
  }

  // Initialize the ROS node.
  this->rosnode.reset(new ros::NodeHandle("subt"));

  // ROS service to receive a command to finish the game.
  ros::NodeHandle n;
  this->finishServiceRos = n.advertiseService(
      "/subt/finish", &GameLogicPluginPrivate::OnFinishCall, this);

  this->startServiceRos = n.advertiseService(
      "/subt/start", &GameLogicPluginPrivate::OnStartCall, this);

  // ROS service to request the robot pose relative to the origin artifact.
  // Note that this service is only available for robots in the staging area.
  this->poseFromArtifactServiceRos = n.advertiseService(
      "/subt/pose_from_artifact_origin",
      &GameLogicPluginPrivate::OnPoseFromArtifactRos, this);
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnFinishCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  if (this->started && _req.data && !this->finished)
  {
    this->Finish();
    _res.success = true;
  }
  else
    _res.success = false;

  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnStartCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  if (_req.data && !this->started && !this->finished)
  {
    _res.success = true;
    this->started = true;
    this->startTime = std::chrono::steady_clock::now();
    ignmsg << "Scoring has Started" << std::endl;
    this->Log() << "scoring_started" << std::endl;
  }
  else
    _res.success = false;

  return true;
}


/////////////////////////////////////////////////
void GameLogicPluginPrivate::Finish()
{
  if (this->started && !this->finished)
  {
    this->finished = true;
    this->finishTime = std::chrono::steady_clock::now();

    auto elapsed = this->finishTime - this->startTime;
    ignmsg << "Scoring has finished. Elapsed time: "
          << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
          << " seconds" << std::endl;
    this->Log() << "finished_elapsed_time "
      << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
      << " s." << std::endl;
    this->Log() << "finished_score " << this->totalScore << std::endl;
    this->logStream.flush();
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result)
{
  // Get an iterator to the robot's pose.
  std::map<std::string, ignition::math::Pose3d>::iterator robotIter =
    this->poses.find(_robot);

  if (robotIter == this->poses.end())
  {
    ignerr << "[GameLogicPlugin]: Unable to find robot with name ["
           << _robot << "]. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  // Get an iterator to the base station's pose.
  std::map<std::string, ignition::math::Pose3d>::iterator baseIter =
    this->poses.find(subt::kBaseStationName);

  // Sanity check: Make sure that the robot is in the stagging area, as this
  // service is only available in that zone.
  if (baseIter == this->poses.end())
  {
    ignerr << "[GameLogicPlugin]: Unable to find the staging area  ["
      << subt::kBaseStationName
      << "]. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  if (baseIter->second.Pos().Distance(robotIter->second.Pos()) > 15)
  {
    ignerr << "[GameLogicPlugin]: Robot [" << _robot << "] is too far from the "
      << "staging area. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  // Get the artifact origin's pose.
  std::map<std::string, ignition::math::Pose3d>::iterator originIter =
    this->poses.find(subt::kArtifactOriginName);
  if (originIter == this->poses.end())
  {
    ignerr << "[GameLogicPlugin]: Unable to find the artifact origin ["
      << subt::kArtifactOriginName
      << "]. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }
  this->artifactOriginPose = originIter->second;

  // Pose.
  _result = robotIter->second - this->artifactOriginPose;
  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnPoseFromArtifact(
    const ignition::msgs::StringMsg &_req,
    ignition::msgs::Pose &_res)
{
  ignition::math::Pose3d pose;
  bool result = this->PoseFromArtifactHelper(_req.data(), pose);
  ignition::msgs::Set(&_res, pose);
  return result;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnPoseFromArtifactRos(
  subt_msgs::PoseFromArtifact::Request &_req,
  subt_msgs::PoseFromArtifact::Response &_res)
{
  ignition::math::Pose3d result;
  _res.success = this->PoseFromArtifactHelper(_req.robot_name.data, result);
  _res.pose.pose.position.x = result.Pos().X();
  _res.pose.pose.position.y = result.Pos().Y();
  _res.pose.pose.position.z = result.Pos().Z();
  _res.pose.pose.orientation.x = result.Rot().X();
  _res.pose.pose.orientation.y = result.Rot().Y();
  _res.pose.pose.orientation.z = result.Rot().Z();
  _res.pose.pose.orientation.w = result.Rot().W();

  // Header.
  _res.pose.header.stamp = ros::Time(
    this->simTime.sec(), this->simTime.nsec());
  _res.pose.header.frame_id = subt::kArtifactOriginName;

  return _res.success;
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPluginPrivate::Log()
{
  this->logStream << this->simTime.sec()
                  << " " << this->simTime.nsec() << " ";
  return this->logStream;
}
