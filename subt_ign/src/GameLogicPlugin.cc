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

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <rosbag/recorder.h>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <utility>

#include <ignition/gazebo/components/BatterySoC.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Performer.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/DepthCamera.hh>
#include <ignition/gazebo/components/GpuLidar.hh>
#include <ignition/gazebo/components/RgbdCamera.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/SourceFilePath.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/PoseStamped.h>
#include "subt_ros/ArtifactReport.h"
#include "subt_ros/KinematicStates.h"
#include "subt_ros/RegionEvent.h"
#include "subt_ros/Robot.h"
#include "subt_ros/RobotEvent.h"
#include "subt_ros/RunStatistics.h"
#include "subt_ros/RunStatus.h"
#include "subt_ign/Common.hh"
#include "subt_ign/GameLogicPlugin.hh"
#include "subt_ign/protobuf/artifact.pb.h"
#include "subt_ign/RobotPlatformTypes.hh"

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
  /// \brief Write a simulation timestamp to a logfile.
  /// \param[in] _simTime Current sim time.
  /// \return A file stream that can be used to write additional
  /// information to the logfile.
  public: std::ofstream &Log(const ignition::msgs::Time &_simTime);

  /// \brief Calculate the score of a new artifact request.
  /// \param[in] _simTime Simulation time.
  /// \param[in] _type The object type. See ArtifactType.
  /// \param[in] _pose The object pose.
  /// \return A tuple where the first parameter is the score obtained for this
  /// report, and the second parameter is true if the artifact report is a
  /// duplicate and false otherwise.
  public: std::tuple<double, bool> ScoreArtifact(
              const ignition::msgs::Time &_simTime,
              const subt::ArtifactType &_type,
              const ignition::msgs::Pose &_pose,
              subt_ros::ArtifactReport &_artifactMsg);

  /// \brief Callback executed to process a new artifact request
  /// sent by a team.
  /// \param[in] _req The service request.
  public: bool OnNewArtifact(const subt::msgs::Artifact &_req,
                             subt::msgs::ArtifactScore &_resp);

  /// \brief Publish the score.
  /// \param[in] _event Unused.
  public: void PublishScore();

  /// \brief Log robot pos data
  public: void LogRobotPosData();

  /// \brief Log robot and artifact data
  /// \param[in] _simTime Current sim time.
  /// \param[in] _realElapsed Elapsed real time in seconds.
  /// \param[in] _simElapsed Elapsed sim time in seconds.
  public: void LogRobotArtifactData(
              const ignition::msgs::Time &_simTime,
              int _realElapsed,
              int _simElapsed) const;

  /// \brief Finish game and generate log files
  /// \param[in] _simTime Simulation time.
  public: void Finish(const ignition::msgs::Time &_simTime);

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
  /// \param[in] _simTime Current sim time.
  /// \return The time point used to calculate the elapsed real time.
  public: std::chrono::steady_clock::time_point UpdateScoreFiles(
              const ignition::msgs::Time &_simTime);

  /// \brief Performer detector subscription callback.
  /// \param[in] _msg Pose message of the event.
  public: void OnEvent(const ignition::msgs::Pose &_msg);

  /// \brief Log an event to the eventStream.
  /// \param[in] _event The event to log.
  public: void LogEvent(const std::string &_event);

  /// \brief Publish a robot event.
  /// \param[in] _simTime Current sim time.
  /// \param[in] _type Event type.
  /// \param[in] _robot Robot name.
  /// \param[in] _eventId Unique ID of the event.
  public: void PublishRobotEvent(
    const ignition::msgs::Time &_simTime,
    const std::string &_type,
    const std::string &_robot,
    int _eventId);

  /// \brief Publish a region event.
  /// \param[in] _simTime Current sim time.
  /// \param[in] _type Event type.
  /// \param[in] _robot Robot name.
  /// \param[in] _eventId Unique ID of the event.
  public: void PublishRegionEvent(
    const ignition::msgs::Time &_simTime,
    const std::string &_type,
    const std::string &_robot, const std::string &_detector,
    const std::string &_state,
    int _eventId);

  /// \brief Marsupial detach subscription callback.
  /// \param[in] _msg Detach message.
  /// \param[in] _info Message information.
  public: void OnDetachEvent(const ignition::msgs::Empty &_msg,
    const transport::MessageInfo &_info);

  /// \brief Breadcrumb deploy subscription callback.
  /// \param[in] _msg Deploy message.
  /// \param[in] _info Message information.
  public: void OnBreadcrumbDeployEvent(const ignition::msgs::Empty &_msg,
    const transport::MessageInfo &_info);

  /// \brief Breadcrumb remaining subscription callback.
  /// \param[in] _msg Remaining count.
  /// \param[in] _info Message information.
  public: void OnBreadcrumbDeployRemainingEvent(
    const ignition::msgs::Int32 &_msg,
    const transport::MessageInfo &_info);

  /// \brief Rock fall remaining subscription callback.
  /// \param[in] _msg Remaining count.
  /// \param[in] _info Message information.
  public: void OnRockFallDeployRemainingEvent(
    const ignition::msgs::Int32 &_msg,
    const transport::MessageInfo &_info);

  /// \brief Battery subscription callback.
  /// \param[in] _msg Battery message.
  /// \param[in] _info Message information.
  public: void OnBatteryMsg(const ignition::msgs::BatteryState &_msg,
    const transport::MessageInfo &_info);

  private: bool PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnStartCall(const ignition::msgs::Boolean &_req,
                            ignition::msgs::Boolean &_res);

  /// \brief Helper function to start the competition.
  /// \param[in] _simTime Simulation time.
  /// \return True if the run was started.
  public: bool Start(const ignition::msgs::Time &_simTime);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(const ignition::msgs::Boolean &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Checks if a robot has flipped.
  public: void CheckRobotFlip();

  /// \brief Round input number n down to the nearest mulitple of m
  /// \param[in] _n input number
  /// \param[in] _m the input number will be rounded down to the nearest
  /// multiple of this number.
  /// \return Result
  public: double FloorMultiple(double _n, double _m);

  /// \brief Convert a world pose to be in the artifact origin frame.
  /// \param[in] _pose Pose to convert.
  /// \return Pose in the artifact origin frame.
  public: ignition::math::Pose3d ConvertToArtifactOrigin(
    const ignition::math::Pose3d &_pose) const;

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Current simulation time.
  public: ignition::msgs::Time simTime;

  /// \brief Amount of allowed warmup time in seconds.
  public: int warmupTimeSec = 900;

  /// \brief The simulation time of the start call.
  public: ignition::msgs::Time startSimTime;

  /// \brief Number of simulation seconds allowed.
  public: std::chrono::seconds runDuration{0};

  /// \brief Thread on which scores are published
  public: std::unique_ptr<std::thread> publishThread = nullptr;

  /// \brief Thread on which the ROS bag recorder runs.
  public: std::unique_ptr<std::thread> bagThread = nullptr;

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

  /// \brief Counter to track duplicate artifact reports
  public: uint32_t duplicateReportCount = 0u;

  /// The maximum number of times that a team can attempt an
  /// artifact report.
  public: uint32_t reportCountLimit = 40u;

  /// \brief The total number of artifacts.
  public: uint32_t artifactCount = 20u;

  /// \brief Total score.
  public: double totalScore = 0.0;

  /// \brief Closest artifact report. The elements are: artifact name, type,
  /// true pos, reported pos, distance between true pos and reported pos
  public: std::tuple<std::string, std::string, ignition::math::Vector3d,
      ignition::math::Vector3d, double> closestReport =
    {"", "", ignition::math::Vector3d(), ignition::math::Vector3d(), -1};

  /// \brief First artifact report time
  public: double firstReportTime = -1;

  /// \brief Last artifact report time
  public: double lastReportTime = -1;

  /// \brief A map of robot name and a vector of timestamped position data
  public: std::map<std::string, std::vector<std::pair<
      std::chrono::steady_clock::duration, ignition::math::Pose3d>>>
      robotPoseData;

  /// \brief A map of robot name and its starting pose
  public: std::map<std::string, ignition::math::Pose3d> robotStartPose;

  /// \brief A map of robot name and distance traveled
  public: std::map<std::string, double> robotDistance;

  /// \brief Step size for elevation gain / loss
  public: double elevationStepSize = 5.0;

  /// \brief A map of robot name and elevation gain (cumulative)
  public: std::map<std::string, double> robotElevationGain;

  /// \brief A map of robot name and elevation loss (cumulative)
  public: std::map<std::string, double> robotElevationLoss;

  /// \brief A map of robot name and max euclidean distance traveled
  public: std::map<std::string, double> robotMaxEuclideanDistance;

  /// \brief A map of robot name and its average velocity
  public: std::map<std::string, double> robotAvgVel;

  /// \brief A map of robot name and its previous pose
  public: std::map<std::string, ignition::math::Pose3d> robotPrevPose;

  /// \brief A map of robot name to its flip information.
  /// The value is a pair stating the sim time where the most recent flip started,
  /// and if the robot is currently flipped.
  public: std::map<std::string, std::pair<int64_t, bool>> robotFlipInfo;

  /// \brief Robot name with the max velocity
  public: std::pair<std::string, double> maxRobotVel = {"", 0};

  /// \brief Robot name with the max average velocity
  public: std::pair<std::string, double> maxRobotAvgVel = {"", 0};

  /// \brief Robot name with the max distance traveled;
  public: std::pair<std::string, double> maxRobotDistance = {"", 0};

  /// \brief Robot name with the max euclidean distance from starting area;
  public: std::pair<std::string, double> maxRobotEuclideanDistance  = {"", 0};

  /// \brief Robot name with the max cumulative elevation gain;
  public: std::pair<std::string, double> maxRobotElevationGain = {"", 0};

  /// \brief Robot name with the max cumulative elevation loss;
  public: std::pair<std::string, double> maxRobotElevationLoss = {"", 0};

  /// \brief Robot name with the max elevation reached;
  public: std::pair<std::string, double> maxRobotElevation = {"", 0};

  /// \brief Robot name with the min elevation reached;
  public: std::pair<std::string, double> minRobotElevation = {"", 0};

  /// \brief Total distanced traveled by all robots
  public: double robotsTotalDistance = 0;

  /// \brief Total cumulative elevation gain by all robots
  public: double robotsTotalElevationGain = 0;

  /// \brief Total cumulative elevation loss by all robots
  public: double robotsTotalElevationLoss = 0;

  /// \brief A map of robot name and its pos output stream
  public: std::map<std::string, std::shared_ptr<std::ofstream>> robotPosStream;

  /// \brief A mutex.
  public: std::mutex logMutex;

  /// \brief A mutex.
  public: std::mutex mutex;

  /// \brief Mutex to protect the poses data structure.
  public: std::mutex posesMutex;

  /// \brief Log file output stream.
  public: std::ofstream logStream;

  /// \brief Event file output stream.
  public: std::ofstream eventStream;

  /// \brief Mutex to protect the eventStream.
  public: std::mutex eventMutex;

  /// \brief The pose of the object marking the origin of the artifacts.
  public: ignition::math::Pose3d artifactOriginPose;

  /// \brief Ignition transport start publisher. This is needed by cloudsim
  /// to know when a run has been started.
  public: transport::Node::Publisher startPub;

  /// \brief Ignition transport competition clock publisher.
  public: transport::Node::Publisher competitionClockPub;

  /// \brief Ignition transport for the remaining artifact reports.
  public: transport::Node::Publisher artifactReportPub;

  /// \brief Ignition transport that publishes robot name and type info.
  public: transport::Node::Publisher robotPub;

  /// \brief Logpath.
  public: std::string logPath{"/dev/null"};

  /// \brief Names of the spawned robots.
  public: std::set<std::string> robotNames;

  /// \brief Robot types for keeping track of unique robot platform types.
  public: std::set<std::string> robotTypes;

  /// \brief Map of robot name to {platform, config}. For example:
  /// {"MY_ROBOT_NAME", {"X1", "X1 SENSOR CONFIG 1"}}.
  public: std::map<std::string, std::pair<std::string, std::string>>
          robotFullTypes;

  /// \brief The unique artifact reports received, and the score it received.
  public: std::map<std::string, double> uniqueReports;

  /// \brief Current state.
  public: std::string state="init";

  /// \brief Time at which the last status publication took place.
  public: std::chrono::steady_clock::time_point lastStatusPubTime;

  /// \brief Time at which the summary.yaml file was last updated.
  public: mutable std::chrono::steady_clock::time_point lastUpdateScoresTime;

  /// \brief Distance from the base station after which the competition is
  /// automatically started, and a robot can no longer receive the artifact
  /// origin frame.
  public: const double allowedDistanceFromBase = 21.0;

  /// \brief The world name.
  public: std::string worldName = "default";

  /// \brief Offsets from each artifact's origin to its localization point.
  /// The gas artifact has a location point of zero because it is placed at
  /// the center point of the entry door threshold at floor level.
  ///
  /// Refer to:
  /// https://subtchallenge.com/resources/SubT_Urban_Artifacts_Specification.pdf
  public: std::map<subt::ArtifactType, ignition::math::Vector3d>
          artifactOffsets =
  {
    {subt::ArtifactType::TYPE_BACKPACK,
      ignition::math::Vector3d(0, -0.12766, 0.25668)},
    {subt::ArtifactType::TYPE_DRILL,
      ignition::math::Vector3d(0, 0.059073, 0.158863)},
    {subt::ArtifactType::TYPE_EXTINGUISHER,
      ignition::math::Vector3d(-0.03557, -0.03509, 0.3479)},
    {subt::ArtifactType::TYPE_GAS,
      ignition::math::Vector3d(0, 0, 0)},
    {subt::ArtifactType::TYPE_HELMET,
      ignition::math::Vector3d(0, 0, 0.165)},
    {subt::ArtifactType::TYPE_PHONE,
      ignition::math::Vector3d(0, -0.004, 0.08)},
    {subt::ArtifactType::TYPE_RESCUE_RANDY,
      ignition::math::Vector3d(-0.071305, 0.021966, 0.39217)},
    {subt::ArtifactType::TYPE_ROPE,
      ignition::math::Vector3d(0.004, -0.03, 0.095)},
    {subt::ArtifactType::TYPE_VENT,
      ignition::math::Vector3d(0, 0, 0.138369)},
    {subt::ArtifactType::TYPE_CUBE,
      ignition::math::Vector3d(0, 0, 0.2)}
  };

  /// \brief Event manager for pausing simulation
  public: EventManager *eventManager;

  /// \brief The set of marsupial pairs.
  public: std::map<std::string, std::string> marsupialPairs;

  /// \brief Models with dead batteries.
  public: std::set<std::string> deadBatteries;

  /// \brief Map of model name to {sim_time_sec, deployments_over_max}. This
  /// map is used to log when no more rock falls are possible for a rock
  /// fall model.
  public: std::map<std::string, std::pair<int, int>> rockFallsMax;

  /// \brief Map of model name to deployments_over_max. This map
  /// is used to log when no more breadcrumb deployments are possible.
  public: std::map<std::string, int> breadcrumbsMax;

  /// \brief The ROS node handler used for communications.
  public: std::unique_ptr<ros::NodeHandle> rosnode;
  public: ros::Publisher rosStatsPub;
  public: ros::Publisher rosStatusPub;
  public: ros::Publisher rosArtifactPub;
  public: ros::Publisher rosRobotEventPub;
  public: ros::Publisher rosRegionEventPub;
  public: std::map<std::string, ros::Publisher> rosRobotPosePubs;
  public: std::map<std::string, ros::Publisher> rosRobotKinematicPubs;

  /// \brief Pointer to the ROS bag recorder.
  public: std::unique_ptr<rosbag::Recorder> rosRecorder;

  public: std::string prevPhase = "";

  /// \brief Counter to create unique id for events
  public: int eventCounter = 0;

  /// \brief Mutex to protect the eventCounter.
  public: std::mutex eventCounterMutex;
};

//////////////////////////////////////////////////
GameLogicPlugin::GameLogicPlugin()
  : dataPtr(new GameLogicPluginPrivate)
{
}

//////////////////////////////////////////////////
GameLogicPlugin::~GameLogicPlugin()
{
  this->dataPtr->Finish(this->dataPtr->simTime);

  if (this->dataPtr->publishThread)
    this->dataPtr->publishThread->join();
}

//////////////////////////////////////////////////
void GameLogicPlugin::Configure(const ignition::gazebo::Entity & /*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager & /*_ecm*/,
                           ignition::gazebo::EventManager & _eventMgr)
{
  this->dataPtr->eventManager = &_eventMgr;

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
          << "to https://github.com/osrf/subt/issues/new. "
          << "SubT logging will be disabled.\n";
      }
      else
      {
        this->dataPtr->logPath = homePath;
      }
    }
    // Read elevation step size. Elevation data will be discretized and
    // rounded down to the nearest multiple of the input step size
    // during logging
    if (loggingElem->HasElement("elevation_step_size"))
    {
      this->dataPtr->elevationStepSize =
          loggingElem->Get<double>("elevation_step_size",
          this->dataPtr->elevationStepSize).first;
    }
  }

  const sdf::ElementPtr rosElem =
    const_cast<sdf::Element*>(_sdf.get())->GetElement("ros");
  if (rosElem)
  {
    std::string value = rosElem->Get<std::string>();
    std::transform(value.begin(), value.end(), value.begin(), ::toupper);
    if (value == "TRUE" || value == "1")
    {
      std::map<std::string, std::string> args;
      ros::init(args, "subt_stats", ros::init_options::NoSigintHandler);
      // Initialize the ROS node.
      this->dataPtr->rosnode.reset(new ros::NodeHandle("subt"));
      this->dataPtr->rosStatusPub =
        this->dataPtr->rosnode->advertise<subt_ros::RunStatus>(
            "status", 100, true);
      this->dataPtr->rosStatsPub =
        this->dataPtr->rosnode->advertise<subt_ros::RunStatistics>(
            "run_statistics", 100);
      this->dataPtr->rosArtifactPub =
        this->dataPtr->rosnode->advertise<subt_ros::ArtifactReport>(
            "artifact_reports", 100);
      this->dataPtr->rosRobotEventPub =
        this->dataPtr->rosnode->advertise<subt_ros::RobotEvent>(
            "robot_event", 100);
      this->dataPtr->rosRegionEventPub =
        this->dataPtr->rosnode->advertise<subt_ros::RegionEvent>(
            "region_event", 100);

      // Setup a ros bag recorder.
      rosbag::RecorderOptions recorderOptions;
      recorderOptions.append_date=false;
      recorderOptions.prefix = ignition::common::joinPaths(
        this->dataPtr->logPath, "cloudsim");
      recorderOptions.regex=true;
      recorderOptions.topics.push_back("/subt/.*");

      // Spawn thread for recording /subt/ data to rosbag.
      this->dataPtr->rosRecorder.reset(new rosbag::Recorder(recorderOptions));
      this->dataPtr->bagThread.reset(new std::thread([&](){
        this->dataPtr->rosRecorder->run();
      }));
    }
  }

  // Open the log file.
  this->dataPtr->logStream.open(
      (this->dataPtr->logPath + "/" + filenamePrefix + "_" +
      ignition::common::systemTimeISO() + ".log").c_str(), std::ios::out);

  // Open the event log file.
  this->dataPtr->eventStream.open(
      (this->dataPtr->logPath + "/events.yml").c_str(), std::ios::out);

  // Advertise the service to receive artifact reports.
  // Note that we're setting the scope to this service to SCOPE_T, so only
  // nodes within the same process will be able to reach this plugin.
  // The reason for this is to avoid the teams to use this service directly.
  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::PROCESS);
  this->dataPtr->node.Advertise(kNewArtifactSrv,
    &GameLogicPluginPrivate::OnNewArtifact, this->dataPtr.get(), opts);

  // Get the duration seconds.
  if (_sdf->HasElement("duration_seconds"))
  {
    this->dataPtr->runDuration = std::chrono::seconds(
        _sdf->Get<int>("duration_seconds"));

    ignmsg << "Run duration set to " << this->dataPtr->runDuration.count()
      << " seconds.\n";
  }

  if (_sdf->HasElement("world_name"))
  {
    this->dataPtr->worldName =
      _sdf->Get<std::string>("world_name", "subt").first;
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

  this->dataPtr->competitionClockPub =
    this->dataPtr->node.Advertise<ignition::msgs::Clock>("/subt/run_clock");

  this->dataPtr->artifactReportPub =
    this->dataPtr->node.Advertise<ignition::msgs::Int32>("/subt/artifact_reports_remaining");

  this->dataPtr->robotPub =
    this->dataPtr->node.Advertise<ignition::msgs::Param_V>("/subt/robots");

  this->dataPtr->publishThread.reset(new std::thread(
        &GameLogicPluginPrivate::PublishScore, this->dataPtr.get()));

  this->dataPtr->node.Subscribe("/subt_performer_detector",
      &GameLogicPluginPrivate::OnEvent, this->dataPtr.get());

  ignmsg << "Starting SubT" << std::endl;

  // Set the report limit to 25 for final worlds.
  this->dataPtr->reportCountLimit =
    this->dataPtr->worldName.find("final") != std::string::npos ? 25 :
    this->dataPtr->reportCountLimit;

  // Make sure that there are score files.
  this->dataPtr->UpdateScoreFiles(this->dataPtr->simTime);
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnBatteryMsg(
    const ignition::msgs::BatteryState &_msg,
    const transport::MessageInfo &_info)
{
  ignition::msgs::Time localSimTime(this->simTime);
  if (_msg.percentage() <= 0)
  {
    std::vector<std::string> topicParts = common::split(_info.Topic(), "/");
    std::string name = "_unknown_";

    // Get the name of the model from the topic name, where the topic name
    // look like '/model/{model_name}/detach'.
    if (topicParts.size() > 1)
      name = topicParts[1];

    // Make sure the event is logged once.
    if (this->deadBatteries.find(name) == this->deadBatteries.end())
    {
      this->deadBatteries.emplace(name);
      {
        std::lock_guard<std::mutex> lock(this->eventCounterMutex);
        std::ostringstream stream;
        stream
          << "- event:\n"
          << "  id: " << this->eventCounter << "\n"
          << "  type: dead_battery\n"
          << "  time_sec: " << localSimTime.sec() << "\n"
          << "  robot: " << name << std::endl;

        this->LogEvent(stream.str());
        this->PublishRobotEvent(localSimTime, "dead_battery", name,
            this->eventCounter);
        this->eventCounter++;
      }
    }
  }
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnBreadcrumbDeployEvent(
    const ignition::msgs::Empty &/*_msg*/,
    const transport::MessageInfo &_info)
{
  ignition::msgs::Time localSimTime(this->simTime);
  std::vector<std::string> topicParts = common::split(_info.Topic(), "/");
  std::string name = "_unknown_";

  // Get the name of the model from the topic name, where the topic name
  // look like '/model/{model_name}/deploy'.
  if (topicParts.size() > 1)
    name = topicParts[1];

  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter << "\n"
      << "  type: breadcrumb_deploy\n"
      << "  time_sec: " << localSimTime.sec() << "\n"
      << "  robot: " << name << std::endl;

    this->LogEvent(stream.str());

    // Only publish if max breadcrumbs has not been reached.
    if (this->breadcrumbsMax.find(name) == this->breadcrumbsMax.end() ||
        this->breadcrumbsMax[name] <= 0)
    {
      this->PublishRobotEvent(localSimTime, "breadcrumb_deploy", name,
          this->eventCounter);
    }
    this->eventCounter++;
  }
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnBreadcrumbDeployRemainingEvent(
    const ignition::msgs::Int32 &_msg,
    const transport::MessageInfo &_info)
{
  ignition::msgs::Time localSimTime(this->simTime);
  if (_msg.data() == 0)
  {
    std::vector<std::string> topicParts = common::split(_info.Topic(), "/");
    std::string name = "_unknown_";

    // Get the name of the model from the topic name, where the topic name
    // look like '/model/{model_name}/deploy'.
    if (topicParts.size() > 1)
      name = topicParts[1];

    if (this->breadcrumbsMax[name] > 0)
    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter++ << "\n"
        << "  type: max_breadcrumb_deploy\n"
        << "  time_sec: " << localSimTime.sec() << "\n"
        << "  robot: " << name << std::endl;

      this->LogEvent(stream.str());
      // Do not publish if max breadcrumbs have already been deployed
      // if (this->breadcrumbsMax[name] == 1)
      //  this->PublishRobotEvent(localSimTime, "max_breadcrumb_deploy", name);
    }

    this->breadcrumbsMax[name]++;
  }
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnRockFallDeployRemainingEvent(
    const ignition::msgs::Int32 &_msg,
    const transport::MessageInfo &_info)
{
  ignition::msgs::Time localSimTime(this->simTime);
  std::vector<std::string> topicParts = common::split(_info.Topic(), "/");
  std::string name = "_unknown_";

  // Get the name of the model from the topic name, where the topic name
  // look like '/model/{model_name}/deploy'.
  if (topicParts.size() > 1)
    name = topicParts[1];

  if (_msg.data() == 0)
  {
    // Sim time is used to make sure that we report only once per rock fall,
    // and not once for every rock in the rock fall.
    if (this->rockFallsMax[name].first != localSimTime.sec())
    {
      if (this->rockFallsMax[name].second == 1)
      {
        std::lock_guard<std::mutex> lock(this->eventCounterMutex);
        std::ostringstream stream;
        stream
          << "- event:\n"
          << "  id: " << this->eventCounter++ << "\n"
          << "  type: max_rock_falls\n"
          << "  time_sec: " << localSimTime.sec() << "\n"
          << "  model: " << name << std::endl;

        this->LogEvent(stream.str());

        // Don't publish this event.
        // this->PublishRegionEvent(localSimTime, "max_rock_falls", "n/a", name,
        //   "max_rock_falls");
      }
      else if (this->rockFallsMax[name].second == 0)
      {
        std::lock_guard<std::mutex> lock(this->eventCounterMutex);
        std::ostringstream stream;
        stream
          << "- event:\n"
          << "  id: " << this->eventCounter << "\n"
          << "  type: rock_fall\n"
          << "  time_sec: " << localSimTime.sec() << "\n"
          << "  model: " << name << std::endl;

        this->LogEvent(stream.str());
        this->PublishRegionEvent(localSimTime, "rock_fall", "", name,
            "rock_fall", this->eventCounter);
        this->eventCounter++;
      }

      this->rockFallsMax[name].second++;
      this->rockFallsMax[name].first = localSimTime.sec();
    }
  }
  // Sim time is used to make sure that we report only once per rock fall,
  // and not once for every rock in the rock fall.
  else if (this->rockFallsMax[name].first != localSimTime.sec())
  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter << "\n"
      << "  type: rock_fall\n"
      << "  time_sec: " << localSimTime.sec() << "\n"
      << "  model: " << name << std::endl;

    this->LogEvent(stream.str());
    this->PublishRegionEvent(localSimTime, "rock_fall", "", name, "rock_fall",
        this->eventCounter);
    this->eventCounter++;
    this->rockFallsMax[name].first = localSimTime.sec();
  }
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnDetachEvent(
    const ignition::msgs::Empty &/*_msg*/,
    const transport::MessageInfo &_info)
{
  ignition::msgs::Time localSimTime(this->simTime);
  std::vector<std::string> topicParts = common::split(_info.Topic(), "/");
  std::string name = "_unknown_";

  // Get the name of the model from the topic name, where the topic name
  // look like '/model/{model_name}/detach'.
  if (topicParts.size() > 1)
    name = topicParts[1];

  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter << "\n"
      << "  type: detach\n"
      << "  time_sec: " << localSimTime.sec() << "\n"
      << "  robot: " << name << std::endl;

    this->LogEvent(stream.str());
    this->PublishRobotEvent(localSimTime, "detach", name, this->eventCounter);
    this->eventCounter++;
  }
}

//////////////////////////////////////////////////
void GameLogicPluginPrivate::OnEvent(const ignition::msgs::Pose &_msg)
{
  ignition::msgs::Time localSimTime(this->simTime);
  std::string frameId = "nil";
  std::string state = "nil";
  std::map<std::string, std::string> extraData;

  for (int i = 0; i < _msg.header().data_size(); ++i)
  {
    if (_msg.header().data(i).key() == "frame_id")
      frameId = _msg.header().data(i).value(0);
    else if (_msg.header().data(i).key() == "state")
    {
      if (_msg.header().data(i).value(0) == "1")
        state = "enter";
      else
        state = "exit";
    }
    else
    {
      extraData[_msg.header().data(i).key()] =  _msg.header().data(i).value(0);
    }
  }

  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    // Default to detect
    std::string regionEventType;
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter << "\n"
      << "  type: detect\n"
      << "  time_sec: " << _msg.header().stamp().sec() << "\n"
      << "  detector: " << frameId << "\n"
      << "  robot: " << _msg.name() << "\n"
      << "  state: " << state << std::endl;
    if (!extraData.empty())
    {
      stream << "  extra:\n";
      for (const auto &data : extraData)
      {
        // there should be only 1 key-value pair. Just in case, we will grab
        // only the first. The key is currently always "type", which we can
        // ignore when sending the ROS message.
        if (regionEventType.empty())
        {
          if (data.second.find("performer_detector_rockfall") ==
              std::string::npos)
          {
            regionEventType = data.second;
          }
        }

        stream << "    "
          << data.first << ": " << data.second << std::endl;
      }
    }
    // Default to "detect" if not set.
    if (regionEventType.empty())
      regionEventType = "detect";
    this->LogEvent(stream.str());
    this->PublishRegionEvent(localSimTime,
        regionEventType, _msg.name(), frameId, state, this->eventCounter);
    this->eventCounter++;
  }
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
    // Get an iterator to the base station's pose.
    std::map<std::string, ignition::math::Pose3d>::iterator baseIter =
      this->dataPtr->poses.find(subt::kBaseStationName);

    _ecm.Each<gazebo::components::DetachableJoint>(
        [&](const gazebo::Entity &,
            const gazebo::components::DetachableJoint *_detach) -> bool
        {
          auto parentModel = _ecm.Component<gazebo::components::ParentEntity>(
              _detach->Data().parentLink);
          auto childModel = _ecm.Component<gazebo::components::ParentEntity>(
              _detach->Data().childLink);

          bool parentPerformer =
            !_ecm.ChildrenByComponents(parentModel->Data(),
                gazebo::components::Performer()).empty();
          bool childPerformer = !_ecm.ChildrenByComponents(childModel->Data(),
              gazebo::components::Performer()).empty();

          if (parentPerformer && childPerformer)
          {
            auto parentName = _ecm.Component<gazebo::components::Name>(
                parentModel->Data());

            auto childName = _ecm.Component<gazebo::components::Name>(
                childModel->Data());
            this->dataPtr->marsupialPairs[parentName->Data()] =
              childName->Data();
          }
          return true;
        });

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
            // Check if the robot has moved into the tunnel. In this case,
            // we need to trigger the /subt/start.
            if (!this->dataPtr->started && baseIter !=
                this->dataPtr->poses.end())
            {
              auto mPose =
                _ecm.Component<gazebo::components::Pose>(model->Data());
              // Execute the start logic if a robot has moved into the tunnel.
              if (mPose &&
                  baseIter->second.Pos().Distance(mPose->Data().Pos()) >
                  this->dataPtr->allowedDistanceFromBase)
              {
                ignition::msgs::Boolean req, res;
                req.set_data(true);
                this->dataPtr->OnStartCall(req, res);
              }
            }

            // Get the model name
            auto mName =
              _ecm.Component<gazebo::components::Name>(model->Data());
            if (this->dataPtr->robotNames.find(mName->Data()) ==
                this->dataPtr->robotNames.end())
            {
              this->dataPtr->robotNames.insert(mName->Data());

              auto filePath =
                _ecm.Component<gazebo::components::SourceFilePath>(
                model->Data());

              // Store unique robot platform information.
              for (const std::string &type : robotPlatformTypes)
              {
                std::string platformNameUpper = filePath->Data();
                std::transform(platformNameUpper.begin(),
                    platformNameUpper.end(),
                    platformNameUpper.begin(), ::toupper);
                if (platformNameUpper.find(type) != std::string::npos)
                {
                  this->dataPtr->robotTypes.insert(type);

                  // The full type is in the directory name, which is third
                  // from the end (.../TYPE/VERSION/model.sdf).
                  std::vector<std::string> pathParts =
                    ignition::common::split(platformNameUpper, "/");
                  this->dataPtr->robotFullTypes[mName->Data()] =
                    {type, pathParts[pathParts.size()-3]};
                }
              }

              // Subscribe to detach topics. We are doing a blanket
              // subscribe even though a robot model may not be marsupial.
              // This is fine since non-marsupial vehicles won't create the
              // publisher, and no extra logic is required here.
              std::string detachTopic = std::string("/model/") +
                mName->Data() + "/detach";
              this->dataPtr->node.Subscribe(detachTopic,
                  &GameLogicPluginPrivate::OnDetachEvent, this->dataPtr.get());

              // Subscribe to breadcrumb deploy topics. We are doing a blanket
              // subscribe even though a robot model may not have
              // breadcrumbs.
              std::string deployTopic = std::string("/model/") +
                mName->Data() + "/breadcrumb/deploy";
              this->dataPtr->node.Subscribe(deployTopic,
                  &GameLogicPluginPrivate::OnBreadcrumbDeployEvent,
                  this->dataPtr.get());

              std::string deployRemainingTopic = std::string("/model/") +
                mName->Data() + "/breadcrumb/deploy/remaining";
              this->dataPtr->node.Subscribe(deployRemainingTopic,
                  &GameLogicPluginPrivate::OnBreadcrumbDeployRemainingEvent,
                  this->dataPtr.get());

              // Subscribe to battery state in order to log battery events.
              std::string batteryTopic = std::string("/model/") +
                mName->Data() + "/battery/linear_battery/state";
              this->dataPtr->node.Subscribe(batteryTopic,
                  &GameLogicPluginPrivate::OnBatteryMsg, this->dataPtr.get());
            }
          }
          return true;
        });

    // Start automatically if warmup time has elapsed.
    if (this->dataPtr->simTime.sec() >= this->dataPtr->warmupTimeSec)
    {
      this->dataPtr->Start(this->dataPtr->simTime);
    }
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
        // Subscribe to remaining rock fall deploy topics. We are doing a
        // blanket subscribe even though a model in this function may not be
        // a rock fall.
        if (this->dataPtr->rockFallsMax.find(_nameComp->Data()) ==
            this->dataPtr->rockFallsMax.end())
        {
          std::string deployRemainingTopic = std::string("/model/") +
                  _nameComp->Data() + "/breadcrumbs/Rock/deploy/remaining";
          this->dataPtr->rockFallsMax[_nameComp->Data()] = {0, 0};
          this->dataPtr->node.Subscribe(deployRemainingTopic,
              &GameLogicPluginPrivate::OnRockFallDeployRemainingEvent,
              this->dataPtr.get());
        }

        {
          std::lock_guard<std::mutex> lock(this->dataPtr->posesMutex);
          this->dataPtr->poses[_nameComp->Data()] = _poseComp->Data();
        }

        // Iterate over possible artifact names
        for (size_t kArtifactNamesIdx = 0;
             kArtifactNamesIdx < kArtifactNames.size();
             ++kArtifactNamesIdx)
        {
          // If the name of the model is a possible artifact, then add it to
          // our list of artifacts.
          if (_nameComp->Data().find(
                kArtifactNames[kArtifactNamesIdx].first) == 0)
          {
            bool add = true;
            // Check to make sure the artifact has not already been added.
            for (const std::pair<std::string, ignition::math::Pose3d>
                &artifactPair : this->dataPtr->artifacts[
                kArtifactNames[kArtifactNamesIdx].second])
            {
              if (artifactPair.first == _nameComp->Data())
              {
                add = false;
                break;
              }
            }

            if (add)
            {
              ignmsg << "Adding artifact name[" << _nameComp->Data()
                << "] type string["
                << kArtifactTypes[kArtifactNamesIdx].second
                << "] typeid["
                << static_cast<int>(
                    kArtifactNames[kArtifactNamesIdx].second)
                << "]\n";

              this->dataPtr->artifacts[
                kArtifactNames[
                  kArtifactNamesIdx].second][_nameComp->Data()] =
                    ignition::math::Pose3d(ignition::math::INF_D,
                        ignition::math::INF_D, ignition::math::INF_D, 0, 0, 0);

              // Helper variable that is the total number of artifacts.
              this->dataPtr->artifactCount++;
            }
          }
        }

        for (std::pair<const subt::ArtifactType,
            std::map<std::string, ignition::math::Pose3d>> &artifactPair :
            this->dataPtr->artifacts)
        {
          for (std::pair<const std::string, ignition::math::Pose3d> &artifact :
              artifactPair.second)
          {
            if (artifact.first == _nameComp->Data())
            {
              // Get a rotation matrix for the artifact
              ignition::math::Matrix3d mat(_poseComp->Data().Rot());

              // Compute the localization point
              ignition::math::Vector3d localizationPoint =
                _poseComp->Data().Pos() +
                mat * this->dataPtr->artifactOffsets[artifactPair.first];

              // Store the final localization point.
              artifact.second.Pos() = localizationPoint;
              break;
            }
          }
        }

        return true;
      });

    // log robot pose and vel data
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

          if (!model)
            return true;

          // robot pose.
          auto poseComp =
              _ecm.Component<gazebo::components::Pose>(model->Data());
          if (!poseComp)
            return true;
          math::Pose3d pose = poseComp->Data();

          // robot name
          auto nameComp =
              _ecm.Component<gazebo::components::Name>(model->Data());
          if (!nameComp)
            return true;
          std::string name = nameComp->Data();

          // sim time
          double t = s + static_cast<double>(ns)*1e-9;
          auto tDur =
              std::chrono::duration_cast<std::chrono::steady_clock::duration>
              (std::chrono::seconds(s) + std::chrono::nanoseconds(ns));

          // store robot pose and velocity data only if robot has traveled
          // more than 1 meter
          auto robotPoseDataIt = this->dataPtr->robotPoseData.find(name);
          if (robotPoseDataIt == this->dataPtr->robotPoseData.end())
          {
            this->dataPtr->robotPoseData[name].push_back(
                std::make_pair(tDur, pose));

            this->dataPtr->robotStartPose[name] = pose;
            this->dataPtr->robotDistance[name] = 0.0;
            this->dataPtr->robotMaxEuclideanDistance[name] = 0.0;
            this->dataPtr->robotAvgVel[name] = 0.0;
            this->dataPtr->robotElevationGain[name] = 0.0;
            this->dataPtr->robotElevationLoss[name] = 0.0;

            if (this->dataPtr->rosnode)
            {
              this->dataPtr->rosRobotPosePubs[name] =
                this->dataPtr->rosnode->advertise<geometry_msgs::PoseStamped>(
                    "poses/" + name, 1000);
              this->dataPtr->rosRobotKinematicPubs[name] =
                this->dataPtr->rosnode->advertise<subt_ros::KinematicStates>(
                    "kinematic_states/" + name, 1000);
            }

            this->dataPtr->robotPrevPose[name] = pose;
            return true;
          }
          // Send robot pose information if the robot has traveled more then
          // 1m or 1second of simulation time has elapsed.
          else if (!robotPoseDataIt->second.empty() &&
              (robotPoseDataIt->second.back().second.Pos().Distance(
                pose.Pos()) > 1.0 ||
               t - robotPoseDataIt->second.back().first.count() * 1e-9 > 1.0))
          {
            //  time passed since last pose sample
            double prevT = robotPoseDataIt->second.back().first.count() * 1e-9;
            double dt = t - prevT;

            // sim paused?
            if (dt <= 0)
              return true;
            geometry_msgs::PoseStamped msg;
            msg.header.stamp.sec = this->dataPtr->simTime.sec();
            msg.header.stamp.nsec = this->dataPtr->simTime.nsec();
            msg.header.frame_id = "artifact_origin";
            ignition::math::Pose3d p = this->dataPtr->ConvertToArtifactOrigin(
                pose);
            msg.pose.position.x = p.Pos().X();
            msg.pose.position.y = p.Pos().Y();
            msg.pose.position.z = p.Pos().Z();
            msg.pose.orientation.x = p.Rot().X();
            msg.pose.orientation.y = p.Rot().Y();
            msg.pose.orientation.z = p.Rot().Z();
            msg.pose.orientation.w = p.Rot().W();

            if (this->dataPtr->rosnode)
              this->dataPtr->rosRobotPosePubs[name].publish(msg);

            // calculate robot velocity and speed
            math::Vector3d p1 = p.Pos();
            math::Vector3d p2 = this->dataPtr->ConvertToArtifactOrigin(
                robotPoseDataIt->second.back().second).Pos();
            double dx = p1.X() - p2.X();
            double dy = p1.Y() - p2.Y();
            double dz = p1.Z() - p2.Z();
            double dist = sqrt(std::pow(dx, 2) + std::pow(dy, 2) +
                std::pow(dz, 2));
            double vel = dist / dt;

            // publish the pose, velocity, and speed
            subt_ros::KinematicStates kmsg;
            kmsg.header.stamp.sec = this->dataPtr->simTime.sec();
            kmsg.header.stamp.nsec = this->dataPtr->simTime.nsec();
            kmsg.header.frame_id = "artifact_origin";
            kmsg.pose.position.x = p.Pos().X();
            kmsg.pose.position.y = p.Pos().Y();
            kmsg.pose.position.z = p.Pos().Z();
            kmsg.pose.orientation.x = p.Rot().X();
            kmsg.pose.orientation.y = p.Rot().Y();
            kmsg.pose.orientation.z = p.Rot().Z();
            kmsg.pose.orientation.w = p.Rot().W();
            kmsg.velocity.x = dx / dt;
            kmsg.velocity.y = dy / dt;
            kmsg.velocity.z = dz / dt;
            kmsg.speed = vel;

            if (this->dataPtr->rosnode)
              this->dataPtr->rosRobotKinematicPubs[name].publish(kmsg);

            // greatest max velocity by a robot
            if (vel > this->dataPtr->maxRobotVel.second)
            {
              this->dataPtr->maxRobotVel.first = name;
              this->dataPtr->maxRobotVel.second = vel;
            }

            // avg vel for this robot
            size_t velCount = robotPoseDataIt->second.size();
            double avgVel =
                (this->dataPtr->robotAvgVel[name] * velCount + vel) /
                (velCount + 1);
            this->dataPtr->robotAvgVel[name] = avgVel;

            // greatest avg vel by a robot
            if (avgVel > this->dataPtr->maxRobotAvgVel.second)
            {
              this->dataPtr->maxRobotAvgVel.first = name;
              this->dataPtr->maxRobotAvgVel.second = avgVel;
            }

            robotPoseDataIt->second.push_back(std::make_pair(tDur, pose));

            // compute and log greatest / total distance traveled and
            // elevation changes

            // distance traveled by this robot
            double distanceDiff =
                this->dataPtr->robotPrevPose[name].Pos().Distance(pose.Pos());
            double distanceTraveled = this->dataPtr->robotDistance[name] +
                distanceDiff;
            this->dataPtr->robotDistance[name] = distanceTraveled;

            // greatest distance traveled by a robot
            if (distanceTraveled > this->dataPtr->maxRobotDistance.second)
            {
              this->dataPtr->maxRobotDistance.first = name;
              this->dataPtr->maxRobotDistance.second = distanceTraveled;
            }

            // max euclidean from starting pose for this robot
            double euclideanDist =
                pose.Pos().Distance(this->dataPtr->robotStartPose[name].Pos());
            if (euclideanDist > this->dataPtr->robotMaxEuclideanDistance[name])
                this->dataPtr->robotMaxEuclideanDistance[name] = euclideanDist;

            // greatest euclidean distance traveled by a robot
            if (euclideanDist > this->dataPtr->maxRobotEuclideanDistance.second)
            {
              this->dataPtr->maxRobotEuclideanDistance.first = name;
              this->dataPtr->maxRobotEuclideanDistance.second = euclideanDist;
            }

            // total distance traveled by all robots
            this->dataPtr->robotsTotalDistance += distanceDiff;

            // greatest elevation gain / loss
            // Elevations are rounded down to nearest mulitple of the elevation
            // step size
            double elevationDiff = this->dataPtr->FloorMultiple(
                 pose.Pos().Z(), this->dataPtr->elevationStepSize) -
                 this->dataPtr->FloorMultiple(
                 this->dataPtr->robotPrevPose[name].Pos().Z(),
                 this->dataPtr->elevationStepSize);

            if (elevationDiff > 0)
            {
              double elevationGain = this->dataPtr->robotElevationGain[name]
                  + elevationDiff;
              this->dataPtr->robotElevationGain[name] = elevationGain;
              if (elevationGain > this->dataPtr->maxRobotElevationGain.second)
              {
                this->dataPtr->maxRobotElevationGain.first = name;
                this->dataPtr->maxRobotElevationGain.second = elevationGain;
              }
              // total elevation gain by all robots
              this->dataPtr->robotsTotalElevationGain += elevationDiff;
            }
            else
            {
              double elevationLoss = this->dataPtr->robotElevationLoss[name]
                  + elevationDiff;
              this->dataPtr->robotElevationLoss[name] = elevationLoss;
              if (elevationLoss < this->dataPtr->maxRobotElevationLoss.second)
              {
                this->dataPtr->maxRobotElevationLoss.first = name;
                this->dataPtr->maxRobotElevationLoss.second = elevationLoss;
              }
              // total elevation loss by all robots
              this->dataPtr->robotsTotalElevationLoss += elevationDiff;
            }

            // min / max elevation reached
            double elevation = this->dataPtr->FloorMultiple(pose.Pos().Z(),
                this->dataPtr->elevationStepSize);
            if (elevation > this->dataPtr->maxRobotElevation.second ||
                this->dataPtr->maxRobotElevation.first.empty())
            {
              this->dataPtr->maxRobotElevation.first = name;
              this->dataPtr->maxRobotElevation.second = elevation;
            }

            if (elevation < this->dataPtr->minRobotElevation.second ||
                this->dataPtr->minRobotElevation.first.empty())
            {
              this->dataPtr->minRobotElevation.first = name;
              this->dataPtr->minRobotElevation.second = elevation;
            }
            this->dataPtr->robotPrevPose[name] = pose;
          }
          return true;
        });


  // Set the artifact origin pose
  if (this->dataPtr->artifactOriginPose == ignition::math::Pose3d::Zero)
  {
    static bool errorSent = false;

    std::lock_guard<std::mutex> lock(this->dataPtr->posesMutex);
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

  // Compute the elapsed competition time.
  auto elapsedCompetitionTime = this->dataPtr->started ?
    _info.simTime - startSimTime : std::chrono::seconds(0);

  // Compute the remaining competition time.
  auto remainingCompetitionTime = this->dataPtr->started &&
    this->dataPtr->runDuration != std::chrono::seconds(0) ?
    this->dataPtr->runDuration - elapsedCompetitionTime :
    std::chrono::seconds(0) ;

  // Check if the allowed time has elapsed. If so, then mark as finished.
  if ((this->dataPtr->started && !this->dataPtr->finished) &&
      this->dataPtr->runDuration != std::chrono::seconds(0) &&
      remainingCompetitionTime <= std::chrono::seconds(0))
  {
    ignmsg << "Time limit[" <<  this->dataPtr->runDuration.count()
      << "s] reached.\n";
    this->dataPtr->Finish(this->dataPtr->simTime);
  }

  auto currentTime = std::chrono::steady_clock::now();
  if (currentTime - this->dataPtr->lastStatusPubTime > std::chrono::seconds(1))
  {
    ignition::msgs::Clock competitionClockMsg;
    ignition::msgs::Header::Map *mapData =
      competitionClockMsg.mutable_header()->add_data();
    mapData->set_key("phase");
    if (this->dataPtr->started)
    {
      mapData->add_value(this->dataPtr->finished ? "finished" : "run");
      auto secondsRemaining = std::chrono::duration_cast<std::chrono::seconds>(
          remainingCompetitionTime);
      competitionClockMsg.mutable_sim()->set_sec(secondsRemaining.count());
    }
    else if (!this->dataPtr->finished)
    {
      mapData->add_value("setup");
      competitionClockMsg.mutable_sim()->set_sec(
          this->dataPtr->warmupTimeSec - this->dataPtr->simTime.sec());
    }
    else
    {
      // It's possible for a team to call Finish before starting.
      mapData->add_value("finished");
    }

    if (this->dataPtr->prevPhase != mapData->value(0))
    {
      subt_ros::RunStatus statusMsg;
      statusMsg.status = mapData->value(0);
      statusMsg.timestamp.sec = this->dataPtr->simTime.sec();
      statusMsg.timestamp.nsec = this->dataPtr->simTime.nsec();

      this->dataPtr->prevPhase = mapData->value(0);
      if (this->dataPtr->rosnode)
        this->dataPtr->rosStatusPub.publish(statusMsg);
    }

    this->dataPtr->competitionClockPub.Publish(competitionClockMsg);

    ignition::msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(this->dataPtr->simTime);
    msg.set_data(this->dataPtr->state);
    this->dataPtr->startPub.Publish(msg);
    this->dataPtr->lastStatusPubTime = currentTime;

    // Publish the remaining artifact reports
    ignition::msgs::Int32 limitMsg;
    limitMsg.set_data(this->dataPtr->reportCountLimit -
        this->dataPtr->reportCount);
    this->dataPtr->artifactReportPub.Publish(limitMsg);

    // Publish robot name and type information.
    ignition::msgs::Param_V robotMsg;
    for (const std::pair<std::string,
         std::pair<std::string, std::string>> &robot :
         this->dataPtr->robotFullTypes)
    {
      ignition::msgs::Param *param = robotMsg.add_param();
      (*param->mutable_params())["name"].set_type(
          ignition::msgs::Any::STRING);
      (*param->mutable_params())["name"].set_string_value(
          robot.first);

      (*param->mutable_params())["config"].set_type(
          ignition::msgs::Any::STRING);
      (*param->mutable_params())["config"].set_string_value(
          robot.second.second);

      (*param->mutable_params())["platform"].set_type(
          ignition::msgs::Any::STRING);
      (*param->mutable_params())["platform"].set_string_value(
          robot.second.first);
    }

    // Add in marsupial pairs.
    for (const std::pair<std::string, std::string> &pair :
         this->dataPtr->marsupialPairs)
    {
      ignition::msgs::Param *param = robotMsg.add_param();
      (*param->mutable_params())["marsupial_parent"].set_type(
          ignition::msgs::Any::STRING);
      (*param->mutable_params())["marsupial_parent"].set_string_value(
          pair.first);
      (*param->mutable_params())["marsupial_child"].set_type(
          ignition::msgs::Any::STRING);
      (*param->mutable_params())["marsupial_child"].set_string_value(
          pair.second);
    }
    this->dataPtr->robotPub.Publish(robotMsg);
  }

  this->dataPtr->CheckRobotFlip();

  // Periodically update the score file.
  if (!this->dataPtr->finished && currentTime -
      this->dataPtr->lastUpdateScoresTime > std::chrono::seconds(30))
  {
    this->dataPtr->UpdateScoreFiles(this->dataPtr->simTime);
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnNewArtifact(const subt::msgs::Artifact &_req,
                                           subt::msgs::ArtifactScore &_resp)
{
  ignition::msgs::Time localSimTime(this->simTime);

  this->Log(localSimTime) << "new_artifact_reported" << std::endl;
  ignmsg << "SimTime[" << localSimTime.sec() << " " << localSimTime.nsec()
         << "] OnNewArtifact Msg=" << _req.DebugString() << std::endl;

  auto realTime = std::chrono::steady_clock::now().time_since_epoch();
  auto s = std::chrono::duration_cast<std::chrono::seconds>(realTime);
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(realTime-s);

  *_resp.mutable_artifact() = _req;

  _resp.mutable_submitted_datetime()->set_sec(s.count());
  _resp.mutable_submitted_datetime()->set_nsec(ns.count());
  *_resp.mutable_sim_time() = localSimTime;

  // TODO(anyone) Where does run information come from?
  _resp.set_run(1);

  ArtifactType artifactType;

  if (this->started && this->finished)
  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    _resp.set_report_status("scoring finished");
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter++ << "\n"
      << "  type: artifact_report_score_finished\n"
      << "  time_sec: " << localSimTime.sec() << "\n"
      << "  total_score: " << this->totalScore << std::endl;
    this->LogEvent(stream.str());
  }
  else if (!this->started && !this->finished)
  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    _resp.set_report_status("run not started");
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter++ << "\n"
      << "  type: artifact_report_not_started\n"
      << "  time_sec: " << localSimTime.sec() << "\n"
      << "  total_score: " << this->totalScore << std::endl;
    this->LogEvent(stream.str());
  }
  else if (this->reportCount >= this->reportCountLimit)
  {
    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      _resp.set_report_status("report limit exceeded");
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter++ << "\n"
        << "  type: artifact_report_limit_exceeded\n"
        << "  time_sec: " << localSimTime.sec() << "\n"
        << "  total_score: " << this->totalScore << std::endl;
      this->LogEvent(stream.str());
    }
    this->Finish(localSimTime);
  }
  else if (!ArtifactFromInt(_req.type(), artifactType))
  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter++ << "\n"
      << "  type: artifact_report_unknown_artifact\n"
      << "  time_sec: " << localSimTime.sec() << "\n"
      << "  total_score: " << this->totalScore << std::endl;
    this->LogEvent(stream.str());

    ignerr << "Unknown artifact code. The number should be between 0 and "
          << kArtifactTypes.size() - 1 << " but we received "
          << _req.type() << std::endl;

    this->Log(localSimTime)
      <<"error Unknown artifact code. The number should be between "
      << "0 and " << kArtifactTypes.size() - 1
      << " but we received " << _req.type() << std::endl;
    _resp.set_report_status("scored");
  }
  else
  {
    subt_ros::ArtifactReport artifactMsg;
    artifactMsg.timestamp.sec = localSimTime.sec();
    artifactMsg.timestamp.nsec = localSimTime.nsec();
    artifactMsg.points_scored = 0;
    artifactMsg.total_score = this->totalScore;

    std::lock_guard<std::mutex> lock(this->mutex);
    auto [scoreDiff, duplicate] = this->ScoreArtifact(localSimTime,
        artifactType, _req.pose(), artifactMsg);

    _resp.set_score_change(scoreDiff);
    _resp.set_report_status("scored");

    if (!duplicate)
    {
      this->totalScore += scoreDiff;
      if (this->rosnode)
        this->rosArtifactPub.publish(artifactMsg);
    }

    ignmsg << "Total score: " << this->totalScore << std::endl;
    this->Log(localSimTime)
      << "new_total_score " << this->totalScore << std::endl;
  }

  _resp.set_report_id(this->reportCount);

  // Finish if the maximum score has been reached, or if the maximum number
  // of artifact reports has been reached..
  if (!this->finished && this->totalScore >= this->artifactCount)
  {
    ignmsg << "Max score has been reached. Congratulations!" << std::endl;
    this->Finish(localSimTime);
    return true;
  }

  if (!this->finished && this->reportCount >= this->reportCountLimit)
  {
    _resp.set_report_status("report limit reached");
    this->Log(localSimTime) << "report_limit_reached" << std::endl;
    ignmsg << "Report limit reached." << std::endl;

    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter++ << "\n"
        << "  type: artifact_report_limit_reached\n"
        << "  time_sec: " << localSimTime.sec() << "\n"
        << "  total_score: " << this->totalScore << std::endl;
      this->LogEvent(stream.str());
    }

    this->Finish(localSimTime);
    return true;
  }

  // Update the score files, in case something bad happens.
  // The ::Finish functions, used above, will also call the UpdateScoreFiles
  // function.
  this->UpdateScoreFiles(localSimTime);

  return true;
}

/////////////////////////////////////////////////
ignition::math::Pose3d GameLogicPluginPrivate::ConvertToArtifactOrigin(
    const ignition::math::Pose3d &_pose) const
{
  return _pose - this->artifactOriginPose;
}

/////////////////////////////////////////////////
std::tuple<double, bool> GameLogicPluginPrivate::ScoreArtifact(
    const ignition::msgs::Time &_simTime,
    const ArtifactType &_type, const ignition::msgs::Pose &_pose,
    subt_ros::ArtifactReport &_artifactMsg)
{
  // Sanity check: Make sure that we have crossed the starting gate.
  if (!this->started)
  {
    ignmsg << "  The task hasn't started yet" << std::endl;
    this->Log(_simTime) << "task_not_started" << std::endl;
    return {0.0, false};
  }

  // Sanity check: Make sure that we still have artifacts.
  if (this->foundArtifacts.size() >= this->artifactCount)
  {
    ignmsg << "  No artifacts remaining" << std::endl;
    this->Log(_simTime) << "no_remaining_artifacts_of_specified_type"
      << std::endl;
    return {0.0, false};
  }

  // The teams are reporting the artifact poses relative to the fiducial located
  // in the staging area. Now, we convert the reported pose to world coordinates
  ignition::math::Pose3d artifactPose = ignition::msgs::Convert(_pose);
  ignition::math::Pose3d pose = artifactPose + this->artifactOriginPose;
  ignition::math::Vector3d observedObjectPose = pose.Pos();

  // Type converted into a string.
  std::string reportType;
  if (!StringFromArtifact(_type, reportType))
  {
    ignmsg << "Unknown artifact type" << std::endl;
    this->Log(_simTime) << "Unkown artifact type reported" << std::endl;

    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter++ << "\n"
        << "  type: unknown_artifact_type\n"
        << "  time_sec: " << _simTime.sec() << "\n"
        << "  reported_pose_world_frame: " << observedObjectPose << "\n"
        << "  reported_pose_artifact_frame: " << artifactPose.Pos() << "\n"
        << "  reported_artifact_type: " << reportType << "\n";
      this->LogEvent(stream.str());
    }

    return {0.0, false};
  }

  // Pose converted into a string.
  std::string reportPose = std::to_string(_pose.position().x()) + "_" +
                           std::to_string(_pose.position().y()) + "_" +
                           std::to_string(_pose.position().z());

  // Unique report Id: Type and pose combined into a string.
  std::string uniqueReportStr = reportType + "_" + reportPose;

  auto uniqueReport = this->uniqueReports.find(uniqueReportStr);

  // Check whether we received the same report before.
  if (uniqueReport != this->uniqueReports.end())
  {
    ignmsg << "This report has been received before" << std::endl;
    this->Log(_simTime) << "This report has been received before" << std::endl;

    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter++ << "\n"
        << "  type: duplicate_artifact_report\n"
        << "  time_sec: " << _simTime.sec() << "\n"
        << "  reported_pose_world_frame: " << observedObjectPose << "\n"
        << "  reported_pose_artifact_frame: " << artifactPose.Pos() << "\n"
        << "  reported_artifact_type: " << reportType << "\n";
      this->LogEvent(stream.str());
    }

    this->duplicateReportCount++;
    return {uniqueReport->second, true};
  }

  // This is a unique report.
  this->reportCount++;

  double score = 0.0;
  std::map<std::string, ignition::math::Pose3d> &potentialArtifacts =
    this->artifacts[_type];

  // From the list of potential artifacts, find out which one is
  // closer (Euclidean distance) to the located by this request.
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
  double distToArtifact = std::get<2>(minDistance);
  if (distToArtifact < 5)
  {
    std::string artifactName = std::get<0>(minDistance);
    if (this->foundArtifacts.find(artifactName) ==
        this->foundArtifacts.end())
    {
      score = 1.0;

      // Keep track of the artifacts that were found.
      this->foundArtifacts.insert(artifactName);
      this->Log(_simTime) << "found_artifact "
        << std::get<0>(minDistance) << std::endl;

      // collect artifact report data for logging
      // update closest artifact reported so far
      double closestDist = std::get<4>(this->closestReport);
      if (closestDist < 0.0 || distToArtifact < closestDist)
      {
        std::string artifactType;
        if (!StringFromArtifact(_type, artifactType))
          artifactType = "";
        // the elements are name, type, true pos, reported pos, dist
        std::get<0>(this->closestReport) = artifactName;
        std::get<1>(this->closestReport) = artifactType;
        std::get<2>(this->closestReport) = std::get<1>(minDistance);
        std::get<3>(this->closestReport) = observedObjectPose;
        std::get<4>(this->closestReport) = std::get<2>(minDistance);
      }
      // compute sim time of this report
      double reportTime = _simTime.sec() + _simTime.nsec() * 1e-9;
      this->lastReportTime = reportTime;
      if (this->firstReportTime < 0)
        this->firstReportTime = reportTime;
    }
  }

  // This is a new unique report, let's save it.
  this->uniqueReports[uniqueReportStr] = score;

  auto outDist = std::isinf(std::get<2>(minDistance)) ? -1 :
    std::get<2>(minDistance);

  {
    std::lock_guard<std::mutex> lock(this->eventCounterMutex);
    std::ostringstream stream;
    stream
      << "- event:\n"
      << "  id: " << this->eventCounter++ << "\n"
      << "  type: artifact_report_attempt\n"
      << "  time_sec: " << _simTime.sec() << "\n"
      << "  reported_pose_world_frame: " << observedObjectPose << "\n"
      << "  reported_pose_artifact_frame: " << artifactPose.Pos() << "\n"
      << "  reported_artifact_type: " << reportType << "\n"
      << "  closest_artifact_name: " << std::get<0>(minDistance) << "\n"
      << "  distance: " <<  outDist << "\n"
      << "  points_scored: " << score << "\n"
      << "  total_score: " << this->totalScore + score << std::endl;
    this->LogEvent(stream.str());
  }

  _artifactMsg.reported_artifact_type = reportType;
  _artifactMsg.reported_artifact_position.x = artifactPose.Pos().X();
  _artifactMsg.reported_artifact_position.y = artifactPose.Pos().Y();
  _artifactMsg.reported_artifact_position.z = artifactPose.Pos().Z();
  _artifactMsg.closest_artifact_name = std::get<0>(minDistance);
  _artifactMsg.distance = outDist;
  _artifactMsg.points_scored = score;
  _artifactMsg.total_score = this->totalScore + score;

  this->Log(_simTime) << "calculated_dist[" << std::get<2>(minDistance)
    << "] for artifact[" << std::get<0>(minDistance) << "] reported_pos["
    << artifactPose.Pos() << "]" << std::endl;

  ignmsg << "  [Total]: " << score << std::endl;
  this->Log(_simTime) << "modified_score " << score << std::endl;

  return {score, false};
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
  ignition::msgs::Time localSimTime(this->simTime);
  if (this->started && _req.data() && !this->finished)
  {
    this->Finish(localSimTime);
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
  ignition::msgs::Time localSimTime(this->simTime);
  if (_req.data())
    _res.set_data(this->Start(localSimTime));
  else
    _res.set_data(false);

  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::Start(const ignition::msgs::Time &_simTime)
{
  bool result = false;

  if (!this->started && !this->finished)
  {
    result = true;
    this->started = true;
    this->startTime = std::chrono::steady_clock::now();
    this->startSimTime = _simTime;
    ignmsg << "Scoring has Started" << std::endl;
    this->Log(_simTime) << "scoring_started" << std::endl;

    ignition::msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(_simTime);
    msg.set_data("started");
    this->state = "started";
    this->startPub.Publish(msg);
    this->lastStatusPubTime = std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter << "\n"
        << "  type: started\n"
        << "  time_sec: " << _simTime.sec() << std::endl;
      this->LogEvent(stream.str());

      if (this->rosnode)
      {
        this->prevPhase = "run";
        subt_ros::RunStatus statusMsg;
        statusMsg.status = "run";
        statusMsg.timestamp.sec = _simTime.sec();
        statusMsg.timestamp.nsec = _simTime.nsec();
        this->rosStatusPub.publish(statusMsg);
      }
      this->eventCounter++;
    }
  }

  // Update files when scoring has started.
  this->UpdateScoreFiles(_simTime);

  return result;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::Finish(const ignition::msgs::Time &_simTime)
{
  // Pause simulation when finished. Always send this request, just to be
  // safe.
  this->eventManager->Emit<events::Pause>(true);

  if (this->finished)
    return;

  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;

  // Update the score.yml and summary.yml files. This function also
  // returns the time point used to calculate the elapsed real time. By
  // returning this time point, we can make sure that this function (the
  // ::Finish function) uses the same time point.
  std::chrono::steady_clock::time_point currTime =
    this->UpdateScoreFiles(_simTime);

  if (this->started)
  {
    realElapsed = std::chrono::duration_cast<std::chrono::seconds>(
        currTime - this->startTime).count();

    simElapsed = _simTime.sec() - this->startSimTime.sec();

    ignmsg << "Scoring has finished. Elapsed real time: "
          << realElapsed << " seconds. Elapsed sim time: "
          << simElapsed << " seconds. " << std::endl;

    this->Log(_simTime) << "finished_elapsed_real_time " << realElapsed
      << " s." << std::endl;
    this->Log(_simTime) << "finished_elapsed_sim_time " << simElapsed
      << " s." << std::endl;
    this->Log(_simTime) << "finished_score " << this->totalScore << std::endl;
    this->logStream.flush();

    {
      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter << "\n"
        << "  type: finished\n"
        << "  time_sec: " << _simTime.sec() << "\n"
        << "  elapsed_real_time: " << realElapsed << "\n"
        << "  elapsed_sim_time: " << simElapsed << "\n"
        << "  total_score: " << this->totalScore << std::endl;
      this->LogEvent(stream.str());
      this->eventCounter++;
    }

    {
      // \todo(nkoenig) After the tunnel circuit, change the /subt/start topic
      // to /sub/status.
      ignition::msgs::StringMsg msg;
      msg.mutable_header()->mutable_stamp()->CopyFrom(_simTime);
      msg.set_data("finished");
      this->state = "finished";
      this->startPub.Publish(msg);
      this->lastStatusPubTime = std::chrono::steady_clock::now();

      if (this->rosnode)
      {
        this->Log(_simTime) << "ROS node exists, time to shutdown."
          << std::endl;

        this->prevPhase = "finished";
        subt_ros::RunStatus statusMsg;
        statusMsg.status = "finished";
        statusMsg.timestamp.sec = _simTime.sec();
        statusMsg.timestamp.nsec = _simTime.nsec();
        this->rosStatusPub.publish(statusMsg);

        if (this->bagThread && this->bagThread->joinable())
        {
          this->Log(_simTime) << "Calling ros::shutdown." << std::endl;

          // Shutdown ros. this makes the ROS bag recorder stop.
          ros::shutdown();
          this->bagThread->join();
        }

        this->Log(_simTime) << "ROS has been shutdown." << std::endl;
      }

      // Send the recording_complete message after ROS has shutdown, if ROS
      // has been enabled.
      ignition::msgs::StringMsg completeMsg;
      completeMsg.mutable_header()->mutable_stamp()->CopyFrom(
          this->simTime);
      completeMsg.set_data("recording_complete");
      this->startPub.Publish(completeMsg);

      std::lock_guard<std::mutex> lock(this->eventCounterMutex);
      std::ostringstream stream;
      stream
        << "- event:\n"
        << "  id: " << this->eventCounter << "\n"
        << "  type: recording_complete\n"
        << "  time_sec: " << _simTime.sec() << "\n"
        << "  elapsed_real_time: " << realElapsed << "\n"
        << "  elapsed_sim_time: " << simElapsed << "\n"
        << "  total_score: " << this->totalScore << std::endl;
      this->LogEvent(stream.str());
      this->eventCounter++;
    }
  }

  this->finished = true;
}

/////////////////////////////////////////////////
std::chrono::steady_clock::time_point GameLogicPluginPrivate::UpdateScoreFiles(
    const ignition::msgs::Time &_simTime)
{
  std::lock_guard<std::mutex> lock(this->logMutex);

  // Elapsed time
  int realElapsed = 0;
  int simElapsed = 0;
  std::chrono::steady_clock::time_point currTime =
    std::chrono::steady_clock::now();

  if (this->started)
  {
    simElapsed = _simTime.sec() - this->startSimTime.sec();
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

  this->LogRobotPosData();
  this->LogRobotArtifactData(_simTime, realElapsed, simElapsed);

  this->lastUpdateScoresTime = currTime;
  return currTime;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::LogRobotPosData()
{
  std::string path = common::joinPaths(this->logPath, "pos-data");

  // remove previous pos data on start
  if (this->lastUpdateScoresTime.time_since_epoch().count() == 0u)
  {
    common::removeAll(path);
    common::createDirectory(path);
  }

  // log robot pos data
  for (auto &it : this->robotPoseData)
  {
    if (it.second.empty())
      return;
    std::string robotName = it.first;

    // create the pos data file if it does not exist yet
    std::shared_ptr<std::ofstream> posStream;
    auto streamIt = this->robotPosStream.find(robotName);
    bool fileExists = true;
    if (streamIt == this->robotPosStream.end())
    {
      std::string file = common::joinPaths(path, robotName + "-pos.data");
      std::shared_ptr<std::ofstream> logFile =
          std::make_shared<std::ofstream>(file, std::ios::ate);
      this->robotPosStream[robotName] = logFile;
      posStream = logFile;
      fileExists = false;
    }
    else
    {
      posStream = streamIt->second;
    }

    // write to file only if there are new data, or it is the first time.
    // Note that robot pos data should always have at least 1 (latest) pos data
    // in the vector which is used during PostUpdate for computating robot
    // distance traveled and vel data
    if (!fileExists || it.second.size() > 1u)
    {
      auto poseData = std::move(it.second);
      auto posIt = poseData.begin();

      // if file already exists, it is not the first time we are writing out
      // pos data. So the first entry in the vector should be the last pos
      // data that was already written out to file so skip it
      if (fileExists)
        posIt++;
      for(; posIt != poseData.end(); ++posIt)
      {
        auto t = posIt->first;
        int64_t s, ns;
        std::tie(s, ns) = ignition::math::durationToSecNsec(posIt->first);
        math::Vector3d pos = posIt->second.Pos();
        // sec nsec x y z
        *posStream << s << " " << ns << " " << pos << std::endl;
      }
      posStream->flush();

      // make sure to push the latest pos data back in the vector
      it.second.push_back({poseData.back().first, poseData.back().second});
    }
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::LogRobotArtifactData(
    const ignition::msgs::Time &_simTime,
    int _realElapsed,
    int _simElapsed) const
{
  // output robot and artifact data to a yml file
  // 1. Number of artifacts found.
  // 2. Robot count
  // 3. Unique robot count (for example, a team of two X1_SENSOR_CONFIG_1
  // robots would have a unique count of 1).
  // 4. Total simulation time.
  // 5. Total real time.
  // 6. Number of artifact report attempts
  // 7. Number of duplicate artifact reports.
  // 8. Closest artifact report:
  //      a. Distance in meters.
  //      b. True position of the artifact.
  //      c. Reported position.
  //      d. Artifact type and name.
  // 9. First artifact report time.
  // 10. Last artifact report time.
  // 11. Mean time between success reports
  // 12. Greatest distance traveled by a robot on the team.
  // 13. Greatest euclidean distance traveled by a robot from the staging area.
  // 14. Total distance traveled by all the robots.
  // 15. Greatest maximum velocity by a robot.
  // 16. Greatest average velocity.
  // 17. Greatest cumulative elevation gain by a robot on the team
  // 18. Greatest cumulative elevation loss by a robot on the team
  // 19. Total cumulative elevation gain by all the robots.
  // 20. Total cumulative elevation loss by all the robots.
  // 21. Max elevation reached by a robot
  // 22. Min elevation reached by a robot
  // 23. Robot configurations and marsupial pairs.

  subt_ros::RunStatistics statsMsg;

  statsMsg.timestamp.sec = _simTime.sec();
  statsMsg.timestamp.nsec = _simTime.nsec();
  statsMsg.world_name = this->worldName;

  YAML::Emitter out;
  out << YAML::BeginMap;

  out << YAML::Key << "world_name";
  out << YAML::Value  << this->worldName;
  out << YAML::Key << "report_count_limit";
  out << YAML::Value  << this->reportCountLimit;
  out << YAML::Key << "robots";
  out << YAML::Value << YAML::BeginMap;
  for (auto const &pair : this->robotFullTypes)
  {
    subt_ros::Robot robotMsg;
    robotMsg.name = pair.first;
    robotMsg.platform = pair.second.first;
    robotMsg.type = pair.second.second;
    statsMsg.robots.push_back(robotMsg);

    out << YAML::Key << pair.first;
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "platform" << YAML::Value << pair.second.first;
    out << YAML::Key << "config" << YAML::Value << pair.second.second;
    out << YAML::EndMap;
  }
  out << YAML::EndMap;

  out << YAML::Key << "marsupials";
  out << YAML::Value << YAML::BeginMap;
  for (auto const &pair : this->marsupialPairs)
  {
    out << YAML::Key << pair.first << YAML::Value << pair.second;
    subt_ros::Marsupial marsupialMsg;
    marsupialMsg.parent = pair.first;
    marsupialMsg.child = pair.second;
    statsMsg.marsupials.push_back(marsupialMsg);
  }
  out << YAML::EndMap;

  // artifact data
  out << YAML::Key << "artifacts_found";
  out << YAML::Value << this->foundArtifacts.size();
  statsMsg.artifacts_found = this->foundArtifacts.size();

  out << YAML::Key << "robot_count";
  out << YAML::Value << this->robotNames.size();
  statsMsg.robot_count = this->robotNames.size();

  out << YAML::Key << "unique_robot_count";
  out << YAML::Value << this->robotTypes.size();
  statsMsg.unique_robot_count = this->robotTypes.size();

  out << YAML::Key << "sim_time";
  out << YAML::Value << _simElapsed;
  statsMsg.sim_time_elapsed = _simElapsed;

  out << YAML::Key << "real_time";
  out << YAML::Value << _realElapsed;
  statsMsg.real_time_elapsed = _realElapsed;

  out << YAML::Key << "artifact_report_count";
  out << YAML::Value << this->reportCount;
  statsMsg.artifact_report_count = this->reportCount;

  out << YAML::Key << "duplicate_report_count";
  out << YAML::Value << this->duplicateReportCount;
  statsMsg.duplicate_report_count = this->duplicateReportCount;

  std::stringstream artifactPos;
  artifactPos << std::get<2>(this->closestReport);
  std::stringstream reportedPos;
  reportedPos << std::get<3>(this->closestReport);

  out << YAML::Key << "closest_artifact_report";
  out << YAML::Value << YAML::BeginMap
      << YAML::Key << "name"
      << YAML::Value << std::get<0>(this->closestReport)
      << YAML::Key << "type"
      << YAML::Value << std::get<1>(this->closestReport)
      << YAML::Key << "true_pos"
      << YAML::Value << artifactPos.str()
      << YAML::Key << "reported_pos"
      << YAML::Value << reportedPos.str()
      << YAML::Key << "distance"
      << YAML::Value << std::get<4>(this->closestReport)
      << YAML::EndMap;
  out << YAML::Key << "first_artifact_report";
  out << YAML::Value << this->firstReportTime;
  out << YAML::Key << "last_artifact_report";
  out << YAML::Value << this->lastReportTime;
  statsMsg.closest_artifact_report_name = std::get<0>(this->closestReport);
  statsMsg.closest_artifact_report_type = std::get<1>(this->closestReport);
  statsMsg.closest_artifact_report_true_pos.x = std::get<2>(this->closestReport).X();
  statsMsg.closest_artifact_report_true_pos.y = std::get<2>(this->closestReport).Y();
  statsMsg.closest_artifact_report_true_pos.z = std::get<2>(this->closestReport).Z();
  statsMsg.closest_artifact_report_reported_pos.x = std::get<3>(this->closestReport).X();
  statsMsg.closest_artifact_report_reported_pos.y = std::get<3>(this->closestReport).Y();
  statsMsg.closest_artifact_report_reported_pos.z = std::get<3>(this->closestReport).Z();
  statsMsg.closest_artifact_report_distance = std::get<4>(this->closestReport);
  statsMsg.first_artifact_report_time = this->firstReportTime;
  statsMsg.last_artifact_report_time = this->lastReportTime;

  double meanReportTime = 0;
  if (this->foundArtifacts.size() > 1)
  {
    meanReportTime = (this->lastReportTime-this->firstReportTime) /
        (this->foundArtifacts.size() - 1);
  }
  out << YAML::Key << "mean_time_between_successful_artifact_reports";
  out << YAML::Value << meanReportTime;
  statsMsg.mean_time_between_successful_artifact_reports = meanReportTime;

  // robot distance traveled and vel data
  out << YAML::Key << "greatest_distance_traveled";
  out << YAML::Value << this->maxRobotDistance.second;
  out << YAML::Key << "greatest_distance_traveled_robot";
  out << YAML::Value << this->maxRobotDistance.first;
  statsMsg.greatest_distance_traveled.name = this->maxRobotDistance.first;
  statsMsg.greatest_distance_traveled.data = this->maxRobotDistance.second;

  out << YAML::Key << "greatest_euclidean_distance_from_start";
  out << YAML::Value << this->maxRobotEuclideanDistance.second;
  out << YAML::Key << "greatest_euclidean_distance_from_start_robot";
  out << YAML::Value << this->maxRobotEuclideanDistance.first;
  statsMsg.greatest_euclidean_distance_from_start.name =
    this->maxRobotEuclideanDistance.first;
  statsMsg.greatest_euclidean_distance_from_start.data =
    this->maxRobotEuclideanDistance.second;

  out << YAML::Key << "total_distance_traveled";
  out << YAML::Value << this->robotsTotalDistance;
  statsMsg.total_distance_traveled =this->robotsTotalDistance;

  out << YAML::Key << "greatest_max_vel";
  out << YAML::Value << this->maxRobotVel.second;
  out << YAML::Key << "greatest_max_vel_robot";
  out << YAML::Value << this->maxRobotVel.first;
  statsMsg.greatest_max_vel.name = this->maxRobotVel.first;
  statsMsg.greatest_max_vel.data = this->maxRobotVel.second;

  out << YAML::Key << "greatest_avg_vel";
  out << YAML::Value << this->maxRobotAvgVel.second;
  out << YAML::Key << "greatest_avg_vel_robot";
  out << YAML::Value << this->maxRobotAvgVel.first;
  statsMsg.greatest_avg_vel.name = this->maxRobotAvgVel.first;
  statsMsg.greatest_avg_vel.data = this->maxRobotAvgVel.second;

  // robot elevation data
  out << YAML::Key << "greatest_elevation_gain";
  out << YAML::Value << this->maxRobotElevationGain.second;
  out << YAML::Key << "greatest_elevation_gain_robot";
  out << YAML::Value << this->maxRobotElevationGain.first;
  statsMsg.greatest_elevation_gain.name = this->maxRobotElevationGain.first;
  statsMsg.greatest_elevation_gain.data = this->maxRobotElevationGain.second;

  out << YAML::Key << "greatest_elevation_loss";
  out << YAML::Value << this->maxRobotElevationLoss.second;
  out << YAML::Key << "greatest_elevation_loss_robot";
  out << YAML::Value << this->maxRobotElevationLoss.first;
  statsMsg.greatest_elevation_loss.name = this->maxRobotElevationLoss.first;
  statsMsg.greatest_elevation_loss.data = this->maxRobotElevationLoss.second;

  out << YAML::Key << "total_elevation_gain";
  out << YAML::Value << this->robotsTotalElevationGain;
  statsMsg.total_elevation_gain = this->robotsTotalElevationGain;

  out << YAML::Key << "total_elevation_loss";
  out << YAML::Value << this->robotsTotalElevationLoss;
  statsMsg.total_elevation_loss = this->robotsTotalElevationLoss;

  out << YAML::Key << "max_elevation_reached";
  out << YAML::Value << this->maxRobotElevation.second;
  out << YAML::Key << "max_elevation_reached_robot";
  out << YAML::Value << this->maxRobotElevation.first;
  statsMsg.max_elevation_reached.name = this->maxRobotElevation.first;
  statsMsg.max_elevation_reached.data = this->maxRobotElevation.second;

  out << YAML::Key << "min_elevation_reached";
  out << YAML::Value << this->minRobotElevation.second;
  out << YAML::Key << "min_elevation_reached_robot";
  out << YAML::Value << this->minRobotElevation.first;
  statsMsg.min_elevation_reached.name = this->minRobotElevation.first;
  statsMsg.min_elevation_reached.data = this->minRobotElevation.second;

  out << YAML::EndMap;

  std::ofstream logFile(this->logPath + "/run.yml", std::ios::out);
  logFile << out.c_str() << std::endl;
  logFile.flush();

  if (this->rosnode)
    this->rosStatsPub.publish(statsMsg);
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result)
{
  ignition::math::Pose3d robotPose, basePose;

  {
    std::lock_guard<std::mutex> lock(this->posesMutex);
    // Get an iterator to the robot's pose.
    std::map<std::string, ignition::math::Pose3d>::iterator robotIter =
      this->poses.find(_robot);

    if (robotIter == this->poses.end())
    {
      ignerr << "[GameLogicPlugin]: Unable to find robot with name ["
        << _robot << "]. Ignoring PoseFromArtifact request" << std::endl;
      return false;
    }
    robotPose = robotIter->second;

    // Get an iterator to the base station's pose.
    std::map<std::string, ignition::math::Pose3d>::iterator baseIter =
      this->poses.find(subt::kBaseStationName);

    // Sanity check: Make sure that the robot is in the staging area, as this
    // service is only available in that zone.
    if (baseIter == this->poses.end())
    {
      ignerr << "[GameLogicPlugin]: Unable to find the staging area  ["
        << subt::kBaseStationName
        << "]. Ignoring PoseFromArtifact request" << std::endl;
      return false;
    }

    basePose = baseIter->second;
  }

  if (basePose.Pos().Distance(robotPose.Pos()) >
      this->allowedDistanceFromBase)
  {
    ignerr << "[GameLogicPlugin]: Robot [" << _robot << "] is too far from the "
      << "staging area. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  // Pose.
  _result = robotPose - this->artifactOriginPose;
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
void GameLogicPluginPrivate::LogEvent(const std::string &_event)
{
  std::lock_guard<std::mutex> lock(this->eventMutex);
  this->eventStream << _event;
  this->eventStream.flush();
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishRobotEvent(
    const ignition::msgs::Time &_simTime,
    const std::string &_type,
    const std::string &_robot,
    int _eventId)
{
  subt_ros::RobotEvent msg;
  msg.timestamp.sec = _simTime.sec();
  msg.timestamp.nsec = _simTime.nsec();
  msg.event_type = _type;
  msg.robot_name = _robot;
  msg.event_id = _eventId;
  if (this->rosnode)
    this->rosRobotEventPub.publish(msg);
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishRegionEvent(
    const ignition::msgs::Time &_simTime,
    const std::string &_type,
    const std::string &_robot, const std::string &_detector,
    const std::string &_state,
    int _eventId)
{
  subt_ros::RegionEvent msg;
  msg.timestamp.sec = _simTime.sec();
  msg.timestamp.nsec = _simTime.nsec();
  msg.event_type = _type;
  msg.robot_name = _robot;
  msg.detector = _detector;
  msg.state = _state;
  msg.event_id = _eventId;
  if (this->rosnode)
    this->rosRegionEventPub.publish(msg);
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPluginPrivate::Log(
    const ignition::msgs::Time &_simTime)
{
  this->logStream << _simTime.sec()
                  << " " << _simTime.nsec() << " ";
  return this->logStream;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::CheckRobotFlip()
{
  for (const auto &posePair : this->robotPrevPose)
  {
    auto name = posePair.first;
    auto pose = posePair.second;

    if (this->robotFlipInfo.find(name) == this->robotFlipInfo.end())
    {
      this->robotFlipInfo[name] = {this->simTime.sec(), false};
      continue;
    }

    // Get cos(theta) between the world's z-axis and the robot's z-axis
    // If they are in opposite directions (cos(theta) close to -1), robot is flipped
    ignition::math::Vector3d a = pose.Rot() * ignition::math::Vector3d(0,0,1);
    auto cos_theta = a.Z();
    if (std::abs(-1 - cos_theta) <= 0.1 )
    {
      // make sure the robot has been flipped for a few seconds before logging a flip
      // (avoid false positives)
      auto simElapsed = this->simTime.sec() - this->robotFlipInfo[name].first;
      if (!this->robotFlipInfo[name].second && (simElapsed >= 3))
      {
        this->robotFlipInfo[name].second = true;
        ignition::msgs::Time localSimTime(this->simTime);

        {
          std::lock_guard<std::mutex> lock(this->eventCounterMutex);
          std::ostringstream stream;
          stream
            << "- event:\n"
            << "  id: " << this->eventCounter << "\n"
            << "  type: flip\n"
            << "  time_sec: " << localSimTime.sec() << "\n"
            << "  robot: " << name << "\n";
          this->LogEvent(stream.str());
          this->PublishRobotEvent(localSimTime, "flip", name,
              this->eventCounter);
          this->eventCounter++;
        }
      }
    }
    else
    {
      this->robotFlipInfo[name] = {this->simTime.sec(), false};
    }
  }
}

/////////////////////////////////////////////////
double GameLogicPluginPrivate::FloorMultiple(double _n, double _m)
{
  double out = _n - fmod(_n, _m);
  if (_n < 0)
    out -= _m;
  return out;
}
