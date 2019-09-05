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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>

#include <algorithm>
#include <chrono>
#include <map>
#include <mutex>
#include <utility>

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/DepthCamera.hh>
#include <ignition/gazebo/components/GpuLidar.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "subt_ign/CommonTypes.hh"
#include "subt_ign/GameLogicPlugin.hh"
#include "subt_ign/protobuf/artifact.pb.h"

IGNITION_ADD_PLUGIN(
    subt::GameLogicPlugin,
    ignition::gazebo::System,
    subt::GameLogicPlugin::ISystemConfigure,
    subt::GameLogicPlugin::ISystemPostUpdate)

using namespace ignition;
using namespace gazebo;
using namespace systems;
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

  /// \brief Create a string from ArtifactType.
  //
  /// \param[in] _type The artifact type.
  /// \param[out] _strType The artifact string.
  /// \return True when the conversion succeed or false otherwise.
  public: bool StringFromArtifact(const subt::ArtifactType &_type,
                                  std::string &_strType);

  /// \brief Callback executed to process a new artifact request
  /// sent by a team.
  /// \param[in] _req The service request.
  public: bool OnNewArtifact(const subt::msgs::Artifact &_req,
                             subt::msgs::ArtifactScore &_resp);

  /// \brief Parse all the artifacts.
  /// \param[in] _sdf The SDF element containing the artifacts.
  public: void ParseArtifacts(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Publish the score.
  /// \param[in] _event Unused.
  public: void PublishScore();

  /// \brief Finish game and generate log files
  public: void Finish();

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in]  _req The message containing the robot name.
  /// \param[out] _res The response message.
  /// \return true on success.
  public: bool OnPoseFromArtifact(
               const ignition::msgs::StringMsg &_req,
               ignition::msgs::Pose &_res);

  /// \brief Update the score.yml and summary.yml files. This function also
  /// returns the time point used to calculate the elapsed real time. By
  /// returning this time point, we can make sure that the ::Finish function
  /// uses the same time point.
  /// \return The time point used to calculate the elapsed real time.
  public: std::chrono::steady_clock::time_point UpdateScoreFiles() const;

  private: bool PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnStartCall(const ignition::msgs::Boolean &_req,
                            ignition::msgs::Boolean &_res);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(const ignition::msgs::Boolean &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Current simulation time.
  public: ignition::msgs::Time simTime;

  /// \brief The simulation time of the start call.
  public: ignition::msgs::Time startSimTime;

  /// \brief Number of simulation seconds allowed.
  public: std::chrono::seconds runDuration{0};

  /// \brief Thread on which scores are published
  public: std::unique_ptr<std::thread> publishThread = nullptr;

  /// \brief Whether the task has started.
  public: bool started = false;

  /// \brief Whether the task has finished.
  public: bool finished = false;

  /// \brief Start time used for scoring.
  public: std::chrono::steady_clock::time_point startTime;

  /// \brief Store all artifacts.
  /// The key is the object type. See ArtifactType for all supported values.
  /// The value is another map, where the key is the model name and the value
  /// is a Model pointer.
  public: std::map<subt::ArtifactType,
                    std::map<std::string, ignition::math::Pose3d>> artifacts;

  /// \brief Artifacts that were found.
  public: std::set<std::string> foundArtifacts;

  public: std::map<std::string, ignition::math::Pose3d> poses;

  /// \brief Counter to track unique identifiers.
  public: uint32_t reportCount = 0u;

  /// The maximum number of times that a team can attempt an
  /// artifact report is this number multiplied by the total number
  /// of artifacts.
  public: uint32_t reportCountLimitFactor = 2u;

  /// \brief The total number of artifacts.
  public: uint32_t artifactCount = 0u;

  /// \brief Total score.
  public: double totalScore = 0.0;

  /// \brief A mutex.
  public: std::mutex mutex;

  /// \brief Log file output stream.
  public: std::ofstream logStream;

  /// \brief The pose of the object marking the origin of the artifacts.
  public: ignition::math::Pose3d artifactOriginPose;

  /// \brief Ignition transport start publisher. This is needed by cloudsim
  /// to know when a run has been started.
  public: transport::Node::Publisher startPub;

  /// \brief Logpath.
  public: std::string logPath{"/dev/null"};

  /// \brief Names of the spawned robots.
  public: std::set<std::string> robotNames;

  /// \brief The unique artifact reports received.
  public: std::vector<std::string> uniqueReports;

  /// \brief Ignition transport status publisher. This publisher
  /// periodically sends a status messages.
  public: transport::Node::Publisher statusPub;

  /// \brief Current state.
  public: std::string state="init";
};

//////////////////////////////////////////////////
GameLogicPlugin::GameLogicPlugin()
  : dataPtr(new GameLogicPluginPrivate)
{
}

//////////////////////////////////////////////////
GameLogicPlugin::~GameLogicPlugin()
{
  this->dataPtr->Finish();
  this->dataPtr->finished = true;
  if (this->dataPtr->publishThread)
    this->dataPtr->publishThread->join();
}

//////////////////////////////////////////////////
void GameLogicPlugin::Configure(const ignition::gazebo::Entity & /*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager & /*_ecm*/,
                           ignition::gazebo::EventManager & /*_eventMgr*/)
{
  // Check if the game logic plugin has a <logging> element.
  // The <logging> element can contain a <filename_prefix> child element.
  // The <filename_prefix> is used to specify the log filename prefix. For
  // example:
  // <logging>
  //   <path>/tmp</path>
  //   <filename_prefix>subt_tunnel_qual</filename_prefix>
  // </logging>
  const sdf::ElementPtr loggingElem =
    const_cast<sdf::Element*>(_sdf.get())->GetElement("logging");

  std::string filenamePrefix;
  if (loggingElem && loggingElem->HasElement("filename_prefix"))
  {
    // Get the log filename prefix.
    filenamePrefix =
      loggingElem->Get<std::string>("filename_prefix", "subt").first;

    // Get the logpath from the <path> element, if it exists.
    if (loggingElem->HasElement("path"))
    {
      this->dataPtr->logPath =
        loggingElem->Get<std::string>("path", "/dev/null").first;
    }
    else
    {
      // Make sure that we can access the HOME environment variable.
      char *homePath = getenv("HOME");
      if (!homePath)
      {
        ignerr << "Unable to get HOME environment variable. Report this error "
          << "to https://bitbucket.org/osrf/subt/issues/new. "
          << "SubT logging will be disabled.\n";
      }
      else
      {
        this->dataPtr->logPath = homePath;
      }
    }
  }

  // Open the log file.
  this->dataPtr->logStream.open(
      (this->dataPtr->logPath + "/" + filenamePrefix + "_" +
      ignition::common::systemTimeISO() + ".log").c_str(), std::ios::out);

  // Advertise the service to receive artifact reports.
  // Note that we're setting the scope to this service to SCOPE_T, so only
  // nodes within the same process will be able to reach this plugin.
  // The reason for this is to avoid the teams to use this service directly.
  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::PROCESS);
  this->dataPtr->node.Advertise(kNewArtifactSrv,
    &GameLogicPluginPrivate::OnNewArtifact, this->dataPtr.get(), opts);

  this->dataPtr->ParseArtifacts(_sdf);

  // Get the duration seconds.
  if (_sdf->HasElement("duration_seconds"))
  {
    this->dataPtr->runDuration = std::chrono::seconds(
        _sdf->Get<int>("duration_seconds"));

    ignmsg << "Run duration set to " << this->dataPtr->runDuration.count()
      << " seconds.\n";
  }

  std::string worldName = "default";
  if (_sdf->HasElement("world_name"))
  {
    worldName = _sdf->Get<std::string>("world_name", "subt").first;
  }
  else
  {
    ignerr << "Missing <world_name>, the GameLogicPlugin will assume a "
      << " world name of 'default'. This could lead to incorrect scoring\n";
  }

  this->dataPtr->node.Advertise("/subt/pose_from_artifact_origin",
      &GameLogicPluginPrivate::OnPoseFromArtifact, this->dataPtr.get());

  this->dataPtr->node.Advertise("/subt/start",
      &GameLogicPluginPrivate::OnStartCall, this->dataPtr.get());

  this->dataPtr->node.Advertise("/subt/finish",
      &GameLogicPluginPrivate::OnFinishCall, this->dataPtr.get());

  this->dataPtr->startPub =
    this->dataPtr->node.Advertise<ignition::msgs::StringMsg>("/subt/start");

  ignition::transport::AdvertiseMessageOptions statusOpts;
  statusOpts.SetMsgsPerSec(1);
  this->dataPtr->statusPub =
    this->dataPtr->node.Advertise<ignition::msgs::StringMsg>("/subt/status",
        statusOpts);

  this->dataPtr->publishThread.reset(new std::thread(
        &GameLogicPluginPrivate::PublishScore, this->dataPtr.get()));

  ignmsg << "Starting SubT" << std::endl;

  // Make sure that there are score files.
  this->dataPtr->UpdateScoreFiles();
}

//////////////////////////////////////////////////
void GameLogicPlugin::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  // Store sim time
  int64_t s, ns;
  std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);
  this->dataPtr->simTime.set_sec(s);
  this->dataPtr->simTime.set_nsec(ns);

  // Capture the names of the robots. We only do this until the team
  // triggers the start signal.
  if (!this->dataPtr->started)
  {
    _ecm.Each<gazebo::components::Sensor,
              gazebo::components::ParentEntity>(
        [&](const gazebo::Entity &,
            const gazebo::components::Sensor *,
            const gazebo::components::ParentEntity *_parent) -> bool
        {
          // Get the model. We are assuming that a sensor is attached to
          // a link.
          auto model = _ecm.Component<gazebo::components::ParentEntity>(
              _parent->Data());

          if (model)
          {
            // Get the model name
            auto mName =
              _ecm.Component<gazebo::components::Name>(model->Data());
            this->dataPtr->robotNames.insert(mName->Data());
          }
          return true;
        });
  }

  // Update pose information
  _ecm.Each<gazebo::components::Model,
            gazebo::components::Name,
            gazebo::components::Pose,
            gazebo::components::Static>(
      [&](const gazebo::Entity &,
          const gazebo::components::Model *,
          const gazebo::components::Name *_nameComp,
          const gazebo::components::Pose *_poseComp,
          const gazebo::components::Static *) -> bool
      {
        this->dataPtr->poses[_nameComp->Data()] = _poseComp->Data();
        for (std::pair<const subt::ArtifactType,
            std::map<std::string, ignition::math::Pose3d>> &artifactPair :
            this->dataPtr->artifacts)
        {
          for (std::pair<const std::string, ignition::math::Pose3d> &artifact :
              artifactPair.second)
          {
            if (artifact.first == _nameComp->Data())
            {
              artifact.second = _poseComp->Data();
              break;
            }
          }
        }
        return true;
      });

  // Set the artifact origin pose
  if (this->dataPtr->artifactOriginPose == ignition::math::Pose3d::Zero)
  {
    static bool errorSent = false;

    // Get the artifact origin's pose.
    std::map<std::string, ignition::math::Pose3d>::iterator originIter =
      this->dataPtr->poses.find(subt::kArtifactOriginName);
    if (originIter == this->dataPtr->poses.end() && !errorSent)
    {
      ignerr << "[GameLogicPlugin]: Unable to find the artifact origin ["
        << subt::kArtifactOriginName
        << "]. Ignoring PoseFromArtifact request" << std::endl;
      errorSent = true;
    }
    else
    {
      this->dataPtr->artifactOriginPose = originIter->second;
    }
  }

  // Get the start sim time in nanoseconds.
  auto startSimTime = std::chrono::nanoseconds(
      this->dataPtr->startSimTime.sec() * 1000000000 +
      this->dataPtr->startSimTime.nsec());
  // Check if the allowed time has elapsed. If so, then mark as finished.
  if ((this->dataPtr->started && !this->dataPtr->finished) &&
      this->dataPtr->runDuration != std::chrono::seconds(0) &&
      _info.simTime - startSimTime > this->dataPtr->runDuration)
  {
    ignmsg << "Time limit[" <<  this->dataPtr->runDuration.count()
      << "s] reached.\n";
    this->dataPtr->Finish();
  }

  ignition::msgs::StringMsg msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(this->dataPtr->simTime);
  msg.set_data(this->dataPtr->state);
  this->dataPtr->statusPub.Publish(msg);
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnNewArtifact(const subt::msgs::Artifact &_req,
                                           subt::msgs::ArtifactScore &_resp)
{
  this->Log() << "new_artifact_reported" << std::endl;
  auto realTime = std::chrono::steady_clock::now().time_since_epoch();
  auto s = std::chrono::duration_cast<std::chrono::seconds>(realTime);
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(realTime-s);

  *_resp.mutable_artifact() = _req;

  _resp.mutable_submitted_datetime()->set_sec(s.count());
  _resp.mutable_submitted_datetime()->set_nsec(ns.count());
  *_resp.mutable_sim_time() = this->simTime;

  // TODO(anyone) Where does run information come from?
  _resp.set_run(1);

  ArtifactType artifactType;

  if (this->started && this->finished)
  {
    _resp.set_report_status("scoring finished");
  }
  else if (!this->started && !this->finished)
  {
    _resp.set_report_status("run not started");
  }
  else if (this->reportCount >
      this->artifactCount * this->reportCountLimitFactor)
  {
    _resp.set_report_status("report limit exceeded");
  }
  else if (!this->ArtifactFromInt(_req.type(), artifactType))
  {
    ignerr << "Unknown artifact code. The number should be between 0 and "
          << this->kArtifactTypes.size() - 1 << " but we received "
          << _req.type() << std::endl;

    this->Log() << "error Unknown artifact code. The number should be between "
                << "0 and " << this->kArtifactTypes.size() - 1
                << " but we received " << _req.type() << std::endl;
    _resp.set_report_status("scored");
  }
  else
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    auto scoreDiff = this->ScoreArtifact(artifactType, _req.pose());
    _resp.set_score_change(scoreDiff);
    _resp.set_report_status("scored");
    this->totalScore += scoreDiff;

    ignmsg << "Total score: " << this->totalScore << std::endl;
    this->Log() << "new_total_score " << this->totalScore << std::endl;
  }

  _resp.set_report_id(this->reportCount);

  // Finish if the maximum score has been reached, or if the maximum number
  // of artifact reports has been reached..
  if (this->totalScore >= this->artifactCount)
  {
    ignmsg << "Max score has been reached. Congratulations!" << std::endl;
    this->Finish();
    return true;
  }

  if (!this->finished &&
      this->reportCount > this->artifactCount * this->reportCountLimitFactor)
  {
    _resp.set_report_status("report limit exceeded");
    ignmsg << "Report limit exceed." << std::endl;
    this->Finish();
    return true;
  }

  // Update the score files, in case something bad happens.
  // The ::Finish functions, used above, will also call the UpdateScoreFiles
  // function.
  this->UpdateScoreFiles();

  return true;
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
  if (this->foundArtifacts.size() >= this->artifactCount)
  {
    ignmsg << "  No artifacts remaining" << std::endl;
    this->Log() << "no_remaining_artifacts_of_specified_type" << std::endl;
    return 0.0;
  }

  // Type converted into a string.
  std::string reportType;
  if (!this->StringFromArtifact(_type, reportType))
  {
    ignmsg << "Unknown artifact type" << std::endl;
    this->Log() << "Unkown artifact type reported" << std::endl;
    return 0.0;
  }

  // Pose converted into a string.
  std::string reportPose = std::to_string(_pose.position().x()) + "_" +
                           std::to_string(_pose.position().y()) + "_" +
                           std::to_string(_pose.position().z());

  // Unique report Id: Type and pose combined into a string.
  std::string uniqueReport = reportType + "_" + reportPose;

  // Check whether we received the same report before.
  if (std::find(this->uniqueReports.begin(),
                this->uniqueReports.end(),
                uniqueReport) != this->uniqueReports.end())
  {
    ignmsg << "This report has been received before" << std::endl;
    this->Log() << "This report has been received before" << std::endl;
    return 0.0;
  }

  // This is a new unique report, let's save it.
  this->uniqueReports.push_back(uniqueReport);

  // This is a unique report.
  this->reportCount++;

  // The teams are reporting the artifact poses relative to the fiducial located
  // in the staging area. Now, we convert the reported pose to world coordinates
  ignition::math::Pose3d artifactPose = ignition::msgs::Convert(_pose);
  ignition::math::Pose3d pose = artifactPose + this->artifactOriginPose;

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
    // make sure the artifact has been loaded
    ignition::math::Vector3d artifactPos = object.second.Pos();
    if (std::isinf(artifactPos.X()) || std::isinf(artifactPos.Y())
        || std::isinf(artifactPos.Z()))
      continue;

    double distance = observedObjectPose.Distance(artifactPos);

    if (distance < std::get<2>(minDistance))
    {
      std::get<0>(minDistance) = object.first;
      std::get<1>(minDistance) = object.second.Pos();
      std::get<2>(minDistance) = distance;
    }
  }

  // Calculate the score based on accuracy in the location. Make sure that
  // the artifact was not already reported.
  if (std::get<2>(minDistance) < 5 &&
      this->foundArtifacts.find(std::get<0>(minDistance)) ==
      this->foundArtifacts.end())
  {
    score = 1.0;

    // Keep track of the artifacts that were found.
    this->foundArtifacts.insert(std::get<0>(minDistance));
    this->Log() << "found_artifact " << std::get<0>(minDistance) << std::endl;
  }

  this->Log() << "calculated_dist " << std::get<2>(minDistance) << std::endl;

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
bool GameLogicPluginPrivate::StringFromArtifact(const ArtifactType &_type,
    std::string &_strType)
{
  auto pos = std::find_if(
    std::begin(this->kArtifactTypes),
    std::end(this->kArtifactTypes),
    [&_type](const typename std::pair<ArtifactType, std::string> &_pair)
    {
      return (std::get<0>(_pair) == _type);
    });

  if (pos == std::end(this->kArtifactTypes))
    return false;

  _strType = std::get<1>(*pos);
  return true;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::ParseArtifacts(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
  sdf::ElementPtr artifactElem = const_cast<sdf::Element*>(
      _sdf.get())->GetElement("artifact");

  while (artifactElem)
  {
    // Sanity check: "Name" is required.
    if (!artifactElem->HasElement("name"))
    {
      ignerr << "[GameLogicPlugin]: Parameter <name> not found. Ignoring this "
            << "artifact" << std::endl;
      this->Log() << "error Parameter <name> not found. Ignoring this artifact"
                  << std::endl;
      artifactElem = artifactElem->GetNextElement("artifact");
      continue;
    }
    std::string modelName = artifactElem->Get<std::string>("name",
        "name").first;

    // Sanity check: "Type" is required.
    if (!artifactElem->HasElement("type"))
    {
      ignerr << "[GameLogicPlugin]: Parameter <type> not found. Ignoring this "
        << "artifact" << std::endl;
      this->Log() << "error Parameter <type> not found. Ignoring this artifact"
                  << std::endl;

      artifactElem = artifactElem->GetNextElement("artifact");
      continue;
    }

    // Sanity check: Make sure that the artifact type is supported.
    std::string modelTypeStr = artifactElem->Get<std::string>("type",
        "type").first;
    ArtifactType modelType;
    if (!this->ArtifactFromString(modelTypeStr, modelType))
    {
      ignerr << "[GameLogicPlugin]: Unknown artifact type ["
        << modelTypeStr << "]. Ignoring artifact" << std::endl;
      this->Log() << "error Unknown artifact type ["
                  << modelTypeStr << "]. Ignoring artifact" << std::endl;
      artifactElem = artifactElem->GetNextElement("artifact");
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
        artifactElem = artifactElem->GetNextElement("artifact");
        continue;
      }
    }

    ignmsg << "Adding artifact name[" << modelName << "] type string["
      << modelTypeStr << "] typeid[" << static_cast<int>(modelType) << "]\n";
    this->artifacts[modelType][modelName] =
        ignition::math::Pose3d(ignition::math::INF_D, ignition::math::INF_D,
            ignition::math::INF_D, 0, 0, 0);
        artifactElem = artifactElem->GetNextElement("artifact");

    // Helper variable that is the total number of artifacts.
    this->artifactCount++;
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishScore()
{
  transport::Node::Publisher scorePub =
    this->node.Advertise<ignition::msgs::Float>("/subt/score");
  ignition::msgs::Float msg;

  while (!this->finished)
  {
    msg.set_data(this->totalScore);

    scorePub.Publish(msg);
    IGN_SLEEP_S(1);
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnFinishCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  if (this->started && _req.data() && !this->finished)
  {
    this->Finish();
    _res.set_data(true);
  }
  else
    _res.set_data(false);

  return true;
}


/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnStartCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  if (_req.data() && !this->started && !this->finished)
  {
    _res.set_data(true);
    this->started = true;
    this->startTime = std::chrono::steady_clock::now();
    this->startSimTime = this->simTime;
    ignmsg << "Scoring has Started" << std::endl;
    this->Log() << "scoring_started" << std::endl;

    ignition::msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(this->simTime);
    msg.set_data("started");
    this->state = "started";
    this->startPub.Publish(msg);
  }
  else
    _res.set_data(false);

  // Update files when scoring has started.
  this->UpdateScoreFiles();

  return true;
}


/////////////////////////////////////////////////
void GameLogicPluginPrivate::Finish()
{
  if (this->finished)
    return;

  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;

  // Update the score.yml and summary.yml files. This function also
  // returns the time point used to calculate the elapsed real time. By
  // returning this time point, we can make sure that this function (the
  // ::Finish function) uses the same time point.
  std::chrono::steady_clock::time_point currTime = this->UpdateScoreFiles();

  if (this->started)
  {
    realElapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currTime - this->startTime).count();

    ignmsg << "Scoring has finished. Elapsed real time: "
          << realElapsed << " seconds. Elapsed sim time: "
          << simElapsed << " seconds. " << std::endl;

    this->Log() << "finished_elapsed_real_time " << realElapsed
      << " s." << std::endl;
    this->Log() << "finished_elapsed_sim_time " << simElapsed
      << " s." << std::endl;
    this->Log() << "finished_score " << this->totalScore << std::endl;
    this->logStream.flush();

    // \todo(nkoenig) After the tunnel circuit, change the /subt/start topic
    // to /sub/status.
    ignition::msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(this->simTime);
    msg.set_data("finished");
    this->state = "finished";
    this->startPub.Publish(msg);
  }

  this->finished = true;
}

/////////////////////////////////////////////////
std::chrono::steady_clock::time_point
GameLogicPluginPrivate::UpdateScoreFiles() const
{
  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;
  std::chrono::steady_clock::time_point currTime =
    std::chrono::steady_clock::now();

  if (this->started)
  {
    simElapsed = this->simTime.sec() - this->startSimTime.sec();
    realElapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currTime - this->startTime).count();
  }

  // Output a run summary
  std::ofstream summary(this->logPath + "/summary.yml", std::ios::out);
  summary << "was_started: " << this->started << std::endl;
  summary << "sim_time_duration_sec: " << simElapsed << std::endl;
  summary << "real_time_duration_sec: " << realElapsed << std::endl;
  summary << "model_count: " << this->robotNames.size() << std::endl;
  summary.flush();

  // Output a score file with just the final score
  std::ofstream score(this->logPath + "/score.yml", std::ios::out);
  score << totalScore << std::endl;
  score.flush();

  return currTime;
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

  if (baseIter->second.Pos().Distance(robotIter->second.Pos()) > 18)
  {
    ignerr << "[GameLogicPlugin]: Robot [" << _robot << "] is too far from the "
      << "staging area. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }


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

  _res.mutable_header()->mutable_stamp()->CopyFrom(this->simTime);
  ignition::msgs::Header::Map *frame = _res.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(subt::kArtifactOriginName);

  return result;
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPluginPrivate::Log()
{
  this->logStream << this->simTime.sec()
                  << " " << this->simTime.nsec() << " ";
  return this->logStream;
}
