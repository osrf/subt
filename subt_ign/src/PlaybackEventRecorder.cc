/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/time.pb.h>
#include <ignition/msgs/Utility.hh>

#include <ignition/transport/log/Log.hh>

#include <list>
#include <mutex>
#include <set>

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/transport/Node.hh>


#include "PlaybackEventRecorder.hh"

IGNITION_ADD_PLUGIN(
    subt::PlaybackEventRecorder,
    ignition::gazebo::System,
    subt::PlaybackEventRecorder::ISystemConfigure,
    subt::PlaybackEventRecorder::ISystemPreUpdate,
    subt::PlaybackEventRecorder::ISystemPostUpdate)

namespace subt
{
  /// \brief Data structure to store event info
  class Event
  {
    public: void GenerateFilename(const std::string &_format)
    {
      this->filename = std::to_string(
          static_cast<int>(this->startRecordTime)) + "-" + this->type;
      if (!this->detector.empty())
        this->filename += "-" + this->detector;
      if (!this->model.empty())
        this->filename += "-" + this->model;
      if (!this->extra.empty())
        this->filename += "-" + this->extra;
      this->filename += "-" + this->robot +
        + "-" + std::to_string(this->id) +"." + _format;
    }

    /// \brief Event id
    public: unsigned int id;

    /// \brief Type of event
    public: std::string type;

    /// \brief Time of event in seconds
    public: double time = 0;

    /// \brief Robot associated with the event
    public: std::string robot;

    /// \brief For "detect" event type only - the detector of the event
    public: std::string detector;

    /// \brief For "detect" event type only - the state of the event,
    /// e.g. enter / exit
    public: std::string state;

    /// \brief Model associated with this event
    public: std::string model;

    /// \brief Extra typpe info associate with event
    public: std::string extra;

    /// \brief Start time for recording video
    public: double startRecordTime = 0;

    /// \brief End time for recording video
    public: double endRecordTime = 0;

    /// \brief Name of the output file.
    public: std::string filename;

    public: bool skip{false};
  };

  /// \brief Playback recording state
  enum State
  {
    /// \brief Idle
    IDLE = 0,

    /// \brief Seek log to time of event
    SEEK_EVENT = 1,

    /// \brief Seek log to time before event
    SEEK_BEGIN = 2,

    /// \brief Recording in progress
    RECORDING = 3,
  };

  /// \brief Camera modes when recording
  enum CameraMode
  {
    /// \brief Static camera pose
    STATIC_CAMERA = 0,

    /// \brief Camera follows robot
    FOLLOW_CAMERA = 1
  };
}

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace subt;

/// \brief Private data class for PlaybackEventRecorder
class subt::PlaybackEventRecorderPrivate
{
  /// \brief Update the camera to track an entity.
  /// \param[in] _info Update information
  /// \param[in] _ecm The ECM.
  public: void FollowEntityUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Seek to time in log playback
  /// \param[in] _t Time in sim seconds
  public: void Seek(double _t);

  /// \brief Move gui camera to pose
  /// \param[in] _pose Pose to move to
  public: void MoveTo(const math::Pose3d &_pose);

  /// \brief Move gui camera to entity
  /// \param[in] _entity Entity to move to
  public: void MoveTo(const std::string &_entity);

  /// \brief Move gui camera to follow entity
  /// \param[in] _entity Entity to follow
  public: void Follow(const std::string &_entity);

  /// \brief Start/stop video recording
  /// \param[in] _record True to start, false to stop
  public: void Record(bool _record);

  /// \brief Spawn a light
  public: void SpawnLight();

  /// \brief Spawn a camera sensor
  public: void SpawnCamera();

  /// \brief Callback for a video recorder stats msg
  /// \param[in] _msg Message containing video recorder stats
  public: void OnRecorderStats(const msgs::Time &_msg);

  /// \brief Recorder stats msg.
  public: msgs::Time recorderStatsMsg;

  /// \brief Mutex to protect recorder stats msg.
  public: std::mutex recorderStatsMutex;

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Name of the world stored in the log
  public: std::string logWorldName;

  /// \brief True to spawn light on playback
  public: bool spawnLight = false;

  /// \brief True to make camera follow robot instead of staying in fixed
  /// position
  public: bool cameraFollow = false;

  /// \brief True if camera is currently following the target
  public: bool cameraFollowing = false;

  /// \brief True if entity (that the camera is asked to follow) exists in
  /// the world
  public: bool entityExists = false;

  /// \brief Time when system is loaded
  public: std::chrono::time_point<std::chrono::system_clock> loadTime;

  /// \brief If scene has been initialized
  public: bool started = false;

  /// \brief Indicate if the system requested to wait for certain amount of
  /// time (real time) before continuing to the playback recording process.
  /// This is needed for example to wait until the gui camera has arrived at
  /// the specified pose
  public: bool waiting = false;

  /// \brief Wait for scene to be created on the gui side
  public: bool waitForScene = false;

  /// \brief Start of wait time in wall clock time
  public: std::chrono::time_point<std::chrono::system_clock> waitStartTime;

  /// \brief Auto exit when log playback recording ends
  public: bool exitOnFinish = false;

  /// \brief Pointer to the event manager
  public: EventManager *eventManager{nullptr};

  /// \brief current event being recorded.
  public: Event event;

  /// \brief Path to the directory containing the state log file
  public: std::string logPath;

  /// \brief A list of events to record
  public: std::list<Event> events;

  /// \brief Move to pose service name
  public: std::string moveToPoseService;

  /// \brief Move to service name
  public: std::string moveToService;

  /// \brief Seek service name
  public: std::string seekService;

  /// \brief Time when video recording stop request is made
  public: std::chrono::time_point<std::chrono::system_clock> recordStopTime;

  /// \brief Video record service name
  public: std::string videoRecordService;

  /// \brief Video encoding format
  public: std::string videoFormat{"mp4"};

  /// \brief Filename of temp video recording
  public: std::string tmpVideoFilename =
      "tmp_video_recording." + this->videoFormat;

  /// \brief Request to stop video recording
  public: bool recordStopRequested{false};

  /// \brief Current state of the system
  public: State state = IDLE;

  /// \brief A map of event type to its start and end recording time
  public: std::map<std::string, std::pair<double, double>> eventRecordDuration;

  /// \brief A map of event type to the camera mode
  public: std::map<std::string, CameraMode> eventCameraMode;

  /// \brief Dynamic rock pose
  public: std::map<std::string, math::Pose3d> rockModelPose;

  /// \brief A list of unique detectors
  public: std::set<std::string> detectors;

  /// \brief A list of unique robots
  public: std::set<std::string> robotNames;

  /// \brief Number of robots
  public: unsigned int robotCount = 0;

  /// \brief Log file start sim time
  public: double logStartTime = 0;

  /// \brief total duration of the run in sim time
  public: unsigned int logEndTime = 0;

  /// \brief Time when all robots have been spawned
  public: double robotsSpawnTime = 0;

  /// \brief Time to append to the end of record time
  public: double endTimeBuffer = 0;

  /// \brief True if recorder needs to catch up on recording so it
  /// does not lag behind too much
  public: bool catchupOnRecording = false;

  /// \brief Status log
  public: std::ofstream statusLog;

  public: std::chrono::steady_clock::duration lastUpdate{0};

  /// \brief Current target to follow
  public: Entity targetEntity{kNullEntity};

  public: math::Vector3d targetOffset{-1.5, 0, 0.5};

  /// \brief Name of entity to follow.
  public: std::string targetEntityName;

  /// \brief Camera entity used to create videos.
  public: Entity cameraEntity{kNullEntity};
};

/////////////////////////////////////////////
PlaybackEventRecorder::PlaybackEventRecorder()
  : dataPtr(new PlaybackEventRecorderPrivate)
{
  // set up event types and recording duration:
  //   * robot deployed a marsupial child (follow camera)
  //   * robot deployed a breadcrumb (follow camera)
  //   * robot flipped over (follow camera)
  //   * robot enter proximity of artifact (follow camera)
  //   * robot exited staging area (static camera)
  //   * robot triggered rock fall (static camera)
  this->dataPtr->eventRecordDuration["detach"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["breadcrumb_deploy"] =
    std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["flip"] = std::make_pair(120, 30);
  this->dataPtr->eventRecordDuration["detect"] = std::make_pair(60, 60);
  this->dataPtr->eventRecordDuration["rock_fall"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["fog"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["stair"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["stairs"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["collision"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["dynamic_collapse"] = std::make_pair(60, 120);

  this->dataPtr->eventCameraMode["detach"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["breadcrumb_deploy"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["flip"] = FOLLOW_CAMERA;
  // detect event type - follow for artifacts
  // There is a special check added later to use static camera for staging area
  this->dataPtr->eventCameraMode["detect"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["rock_fall"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["fog"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["stair"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["stairs"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["collision"] = FOLLOW_CAMERA;
  this->dataPtr->eventCameraMode["dynamic_collapse"] = FOLLOW_CAMERA;

  // breadcrumb and flip events no longer needed
  this->dataPtr->detectors.insert("staging_area");

  // artifact proximity events no longer needed
  this->dataPtr->detectors.insert("backpack");
  this->dataPtr->detectors.insert("phone");
  this->dataPtr->detectors.insert("rescue_randy");
  this->dataPtr->detectors.insert("rope");
  this->dataPtr->detectors.insert("helmet");
  this->dataPtr->detectors.insert("extinguisher");
  this->dataPtr->detectors.insert("drill");
  this->dataPtr->detectors.insert("vent");
  this->dataPtr->detectors.insert("gas");
  this->dataPtr->detectors.insert("cube");

  // region of interest events - decision, vertical, elevation
  this->dataPtr->detectors.insert("tile");
  this->dataPtr->detectors.insert("sec_");
}

/////////////////////////////////////////////
PlaybackEventRecorder::~PlaybackEventRecorder()
{
}

//////////////////////////////////////////////////
void PlaybackEventRecorder::Configure(const ignition::gazebo::Entity &,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &/*_ecm*/,
                           ignition::gazebo::EventManager &_eventMgr)
{
  // Get an event id to record, if set. This mechanism is used to record
  // a single event, such as a "fog" or "dynamic_collapse" event.
  std::string eventId;
  if (!ignition::common::env("SUBT_EVENT_ID", eventId))
    eventId = "-1";

  std::cout << "\n\n\nEVENT ID[" << eventId << "]\n\n\n\n";

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  if (_sdf->HasElement("exit_on_finish"))
  {
    this->dataPtr->exitOnFinish = ptr->Get<bool>("exit_on_finish");
  }

  if (!_sdf->HasElement("log_path"))
  {
    ignerr << "Unable to record playback video. "
           << "Please specify 'log_path'." << std::endl;
    return;
  }

  if (_sdf->HasElement("spawn_light"))
  {
    this->dataPtr->spawnLight = ptr->Get<bool>("spawn_light");
  }

  const sdf::ElementPtr logPathElem = ptr->GetElement("log_path");
  this->dataPtr->logPath = logPathElem->Get<std::string>();

  // load run.yml
  YAML::Node runNode = YAML::LoadFile(
      common::joinPaths(this->dataPtr->logPath, "run.yml"));

  // parse number of robots
  this->dataPtr->robotCount = runNode["robot_count"].as<unsigned int>();

  // get total duration of log
  std::unique_ptr<transport::log::Log> log =
      std::make_unique<transport::log::Log>();
  std::string dbPath =
      common::joinPaths(this->dataPtr->logPath, "state.tlog");
  if (!log->Open(dbPath))
  {
    ignerr << "Failed to open log file [" << dbPath << "]" << std::endl;
  }

  this->dataPtr->logEndTime =
      std::chrono::duration_cast<std::chrono::seconds>(
      log->EndTime()).count();


  std::string eventFilename = "../filtered_events.yaml";
  if (!common::exists(common::joinPaths(this->dataPtr->logPath, eventFilename)))
    eventFilename = "events.yml";

  this->dataPtr->statusLog.open("record_status.log", std::ofstream::app);

  // load filtered_events.yml
  YAML::Node node = YAML::LoadFile(
      common::joinPaths(this->dataPtr->logPath, eventFilename));

  // staging area event - there should be one in each playback
  // record video until last robot exists staging area
  Event stagingAreaEvent;
  stagingAreaEvent.type = "detect";
  stagingAreaEvent.detector = "staging_area";
  stagingAreaEvent.time = 0;
  stagingAreaEvent.state = "exit";
  stagingAreaEvent.startRecordTime = this->dataPtr->logStartTime;
  stagingAreaEvent.endRecordTime = std::min(60u, this->dataPtr->logEndTime-1);
  std::map<std::string, double> stagingAreaEventTime;

  // parse and store the list of events
  for (const auto &n : node)
  {
    std::string type = n["type"].as<std::string>();

    Event e;
    e.type = type;
    e.time = n["time_sec"].as<double>();

    // Skip the event when:
    //   1. An eventId is specified that doesn't match this event's id, or
    //   2. No eventId is specified and the type is "fog" or
    //   "dynamic_collapse".
    if ((eventId != "-1" && n["id"].as<std::string>() != eventId) ||
        (eventId == "-1" && (type == "fog" || type == "dynamic_collapse")))
    {
      continue;
    }

    // Use the started event as the log start time. This makes sure the
    // levels have been loaded.
    if (type == "started")
    {
      std::cout << "Log start time[" << e.time << "]\n";
      this->dataPtr->logStartTime = e.time;
      stagingAreaEvent.startRecordTime = this->dataPtr->logStartTime;
    }

    auto it = this->dataPtr->eventRecordDuration.find(type);
    if (it == this->dataPtr->eventRecordDuration.end())
       continue;

    // check for detect event type and record videos only for detectors that we
    // are interested in.
    // Currently only interested in staging area event
    if (type == "detect")
    {
      std::string detector;
      std::string state;
      if (auto detectorParam = n["detector"])
      {
        // filter detector events to the list defined in this->dataPtr->detector
        detector = detectorParam.as<std::string>();
        bool validDetector = false;
        for (const auto &d : this->dataPtr->detectors)
        {
          if (detector.find(d) != std::string::npos)
          {
            validDetector = true;
            break;
          }
        }
        if (!validDetector)
          continue;
      }
      else
        continue;

      if (auto stateParam = n["state"])
      {
        state = stateParam.as<std::string>();
      }
      else
      {
        continue;
      }

      // Get the time when the last robot exits the staging area
      // take into accout that some robots may never leave, in which case
      // record until the last unique robot exit event
      if (detector == "staging_area")
      {
        if (state == "exit" && stagingAreaEventTime.size() <
            this->dataPtr->robotCount)
        {
          std::string robot = n["robot"].as<std::string>();
          double time = n["time_sec"].as<double>();
          if (stagingAreaEventTime.find(robot) == stagingAreaEventTime.end())
          {
            stagingAreaEventTime[robot] = time;
            if (time > stagingAreaEvent.time)
            {
              stagingAreaEvent.time = time;
              stagingAreaEvent.robot = robot;
              stagingAreaEvent.endRecordTime =
                std::min(time + it->second.second,
                    static_cast<double>(this->dataPtr->logEndTime-1));
              stagingAreaEvent.id = n["id"].as<int>();
            }
          }
        }
        // continue because we don't need to store every staging area detector
        // event in the list. There should only be one, which we manually push
        // into the events list later
        continue;
      }

      if (auto extraParam = n["extra"])
      {
        if (auto extraTypeParam = extraParam["type"])
        {
          e.extra = extraTypeParam.as<std::string>();
        }
      }

      e.detector = detector;
    }

    if (n["id"])
      e.id = n["id"].as<unsigned int>();
    else
      e.id = this->dataPtr->events.size();

    // for rock fall events, we need to check the corresponding performer
    // detector events to get the robot associated with this event
    if (type == "rock_fall" || type == "fog" || type =="dynamic_collapse")
    {
      std::string model = n["model"].as<std::string>();
      std::string suffix = model.substr(model.rfind("_"));
      std::string detectorName;
      double allowedTimeDiff = 0.0;
      if (type == "dynamic_collapse")
        allowedTimeDiff = 10;

      if (type == "rock_fall")
        detectorName = "rockfall" + suffix;
      else
        detectorName = model;

      for (const auto &ev : node)
      {
        if (ev["type"].as<std::string>() == "detect" &&
            e.time - ev["time_sec"].as<double>() <= allowedTimeDiff &&
            ev["detector"].as<std::string>().find(detectorName)
            != std::string::npos)
        {
          e.robot = ev["robot"].as<std::string>();
          e.model = model;
          break;
        }
      }
      if (e.robot.empty())
        ignerr << "Unable to get robot name for event id[" << e.id
          << "] using detector[" << detectorName << "].\n";
      else
        igndbg << "Using robot[" << e.robot << "] for event id["
          << e.id << "] using detector[" << detectorName << "].\n";
    }
    else
    {
      e.robot = n["robot"].as<std::string>();
    }

    e.startRecordTime = std::max(e.time - it->second.first,
        this->dataPtr->logStartTime);
    e.endRecordTime = std::min(e.time + it->second.second,
        static_cast<double>(this->dataPtr->logEndTime-1));
    e.GenerateFilename(this->dataPtr->videoFormat);
    if (!ignition::common::exists(this->dataPtr->logPath + "/../" + e.filename))
    {
      e.skip = false;
      // Don't record events where the robot name is empty.
      if (!e.robot.empty())
        this->dataPtr->events.push_back(e);
      else
      {
        auto now = std::chrono::steady_clock::now();
        this->dataPtr->statusLog << ignition::math::timePointToString(now)
          << " unable to get robot for event id " << e.id << std::endl;
      }
    }
    else
    {
      e.skip = true;
      this->dataPtr->events.push_back(e);

      std::cout << e.filename << " exists, skipping\n";
      auto now = std::chrono::steady_clock::now();
      this->dataPtr->statusLog << ignition::math::timePointToString(now)
        << " " << e.filename << " exists, skipping" << std::endl;
    }
  }

  // merge artifact proximity detector and collision events
  // we don't want to record mulitple videos for each artifact detector and
  // collision events if mulitple events occurred for the same robot and
  // artifact/collision within some time period, then merge the events.
  double maxTimeDiff = 60.0;
  std::set<unsigned int> toRemove;
  for (auto it = this->dataPtr->events.begin();
       it != this->dataPtr->events.end(); ++it)
  {
    auto &e = *it;
    if (toRemove.find(e.id) != toRemove.end())
      continue;
    if (e.type != "detect" && e.type != "collision")
      continue;

    // merge current event with other detector events for the same robot
    // if time difference between the two is less than maxTimeDiff
    for (auto it2 = std::next(it, 1); it2 != this->dataPtr->events.end();
        ++it2)
    {
      auto &e2 = *it2;
      if ((e2.type == "detect" || e2.type == "collision") &&
          e2.robot == e.robot && e2.detector == e.detector &&
          e2.extra == e.extra)
      {

        double dt = e2.startRecordTime - e.startRecordTime;

        if (dt < maxTimeDiff && dt >= 0)
        {
          std::cout << "Merging " << e.id << " with " << e2.id << std::endl;
          e.endRecordTime = std::min(e2.endRecordTime,
              static_cast<double>(this->dataPtr->logEndTime-1));
          toRemove.insert(e2.id);
        }
      }
    }
  }
  // remove events that were merged and marked for removal
  this->dataPtr->events.remove_if(
      [&toRemove](Event &e) {return toRemove.find(e.id) != toRemove.end();});

  // remove events marked for skipping
  this->dataPtr->events.remove_if([&](Event &e) {return e.skip;});

  // append buffer time to the end
  for (auto &e : this->dataPtr->events)
  {
    e.endRecordTime += this->dataPtr->endTimeBuffer;
    e.endRecordTime = std::min(e.endRecordTime,
        static_cast<double>(this->dataPtr->logEndTime-1));
  }
  stagingAreaEvent.endRecordTime += this->dataPtr->endTimeBuffer;
  stagingAreaEvent.endRecordTime = std::min(stagingAreaEvent.endRecordTime,
      static_cast<double>(this->dataPtr->logEndTime-1));

  // add the staging area event
  if (!stagingAreaEventTime.empty())
  {
    stagingAreaEvent.GenerateFilename(this->dataPtr->videoFormat);
    if (!ignition::common::exists(this->dataPtr->logPath + "/../" +
          stagingAreaEvent.filename))
    {
      // this->dataPtr->events.push_back(stagingAreaEvent);
    }
    else
    {
      std::cout << stagingAreaEvent.filename << " exists, skipping\n";
      auto now = std::chrono::steady_clock::now();
      this->dataPtr->statusLog << ignition::math::timePointToString(now)
        << " " << stagingAreaEvent.filename << " exists, skipping" << std::endl;
    }
  }

  // don't do anything if there are no events
  if (this->dataPtr->events.empty())
  {
    std::cout << "No events to record: " << std::endl;
    return;
  }
  std::cout << "Events to record: " << std::endl;
  for (const auto &e : this->dataPtr->events)
  {
    std::cout << "Event: " << std::endl;
    std::cout << "  type: " << e.type << std::endl;
    std::cout << "  id: " << e.id << std::endl;
    std::cout << "  filename: " << e.filename << std::endl;
    std::cout << "  robot: " << e.robot << std::endl;
    std::cout << "  time: " << e.time << std::endl;
    std::cout << "  detector: " << ((e.detector.empty()) ? "N/A" : e.detector)
              << std::endl;
    std::cout << "  state: " << ((e.state.empty()) ? "N/A" : e.state)
              << std::endl;
    std::cout << "  model: " << ((e.model.empty()) ? "N/A" : e.model)
              << std::endl;
    std::cout << "  extra: " << ((e.extra.empty()) ? "N/A" : e.extra)
              << std::endl;
    std::cout << "  start time: " << e.startRecordTime << std::endl;
    std::cout << "  end time: " << e.endRecordTime << std::endl;

  }

  this->dataPtr->eventManager = &_eventMgr;

  // For move to service requests
  this->dataPtr->moveToPoseService = "/gui/move_to/pose";
  this->dataPtr->moveToService = "/gui/move_to";

  // For video record requests
  this->dataPtr->videoRecordService = "/subt/camera/record_video";

  this->dataPtr->node.Subscribe("/camera/stats",
      &PlaybackEventRecorderPrivate::OnRecorderStats, this->dataPtr.get());

  this->dataPtr->loadTime = std::chrono::system_clock::now();
}

//////////////////////////////////////////////////
void PlaybackEventRecorder::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  auto entity = _ecm.EntityByComponents(
      components::Name("spawned_camera"));
  if (entity)
    _ecm.PinEntity(entity);

  if (!_info.paused)
    this->dataPtr->FollowEntityUpdate(_info, _ecm);
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::FollowEntityUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // Time delta
  std::chrono::duration<double> dtDuration = _info.simTime -
      this->lastUpdate;

  this->lastUpdate = _info.simTime;

  // Is there a follow target?
  if (this->targetEntity == kNullEntity)
  {
    if (!this->targetEntityName.empty())
    {
      this->targetEntity = _ecm.EntityByComponents(components::Name(
            this->targetEntityName));

      if (this->targetEntity == kNullEntity)
      {
        ignerr << "Unable to get entity with name["
          << this->targetEntityName << "]\n";
        return;
      }
    }
    else
      return;
  }

  // Current world pose
  auto poseComp = _ecm.Component<components::Pose>(
      this->cameraEntity);

  if (!this->cameraEntity)
    return;
  else
  {
    // Prevent the cameraEntity from being removed during log seeks.
    static bool once = true;
    if (once)
    {
      once = false;
      _ecm.PinEntity(this->cameraEntity);
    }
  }

  // Safety checks
  if (!poseComp)
    return;
  if (!_ecm.Component<components::Pose>(this->targetEntity))
    return;

  auto modelPose = poseComp->Data();

  // Current target
  auto targetPose = _ecm.Component<components::Pose>(
      this->targetEntity)->Data();

  auto cameraTargetPos = targetPose.Pos() +
    targetPose.Rot().RotateVector(this->targetOffset);

  // Direction from camera current position to target position
  auto dir = cameraTargetPos - modelPose.Pos();

  dir.Normalize();
  modelPose.Pos() = cameraTargetPos;

  dir = targetPose.Pos() - modelPose.Pos();
  dir.Normalize();

  // Towards target
  math::Angle yaw = atan2(dir.Y(), dir.X());
  yaw.Normalize();

  // Compute the pitch angle
  dir = modelPose.Pos() - targetPose.Pos();
  double dist = sqrt(dir.X()*dir.X()+dir.Y()*dir.Y());
  double pitch = atan2(dir.Z(), dist);

  modelPose.Rot() = math::Quaterniond(0, pitch, yaw.Radian());

  // Update actor root pose
  *poseComp = components::Pose(modelPose);
  // Mark as a one-time-change so that the change is propagated to the GUI
  _ecm.SetChanged(this->cameraEntity,
      components::Pose::typeId, ComponentState::OneTimeChange);
}

//////////////////////////////////////////////////
void PlaybackEventRecorder::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->state == IDLE &&
      this->dataPtr->events.empty())
  {
    if (this->dataPtr->exitOnFinish)
      exit(0);
    return;
  }

  // Get world name recorded in log
  if (this->dataPtr->logWorldName.empty())
  {
    _ecm.Each<components::World, components::Name>(
        [&](const Entity & /*_entity*/,
          const components::World *,
          const components::Name *_name)->bool
        {
          this->dataPtr->logWorldName = _name->Data();
          return true;
        });
  }

  // Get rock fall model pose.
  {
    _ecm.Each<components::Model, components::Name, components::Pose,
        components::Static>(
        [&](const Entity & /*_entity*/,
          const components::Model *,
          const components::Name *_name,
          const components::Pose *_pose,
          const components::Static *)->bool
        {
          std::string rockName = _name->Data();
          if (this->dataPtr->rockModelPose.find(rockName)
              == this->dataPtr->rockModelPose.end() &&
              rockName.find("dynamic_rocks") != std::string::npos)
          {
            this->dataPtr->rockModelPose[rockName] = _pose->Data();
          }
          return true;
        });
  }

  // wait for a few seconds before doing anything
  std::chrono::time_point<std::chrono::system_clock> t =
      std::chrono::system_clock::now();
  if (t - this->dataPtr->loadTime < std::chrono::seconds(5))
    return;

  int64_t s, ns;
  std::tie(s, ns) = ignition::math::durationToSecNsec(_info.simTime);

  int64_t rs, rns;
  std::tie(rs, rns) = ignition::math::durationToSecNsec(_info.realTime);

  // step the sim for a few seconds for scene to load on gui side
  if (!this->dataPtr->started)
  {
    if (_info.paused)
      this->dataPtr->eventManager->Emit<events::Pause>(false);

    // look for robots
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
            auto mName =
              _ecm.Component<gazebo::components::Name>(model->Data());
            if (this->dataPtr->robotNames.find(mName->Data()) ==
                this->dataPtr->robotNames.end())
              this->dataPtr->robotNames.insert(mName->Data());
          }

          return true;
        });

    // wait for robots to spawn
    if (s < 60 && this->dataPtr->robotNames.size() < this->dataPtr->robotCount)
    {
      static bool informed = false;
      if (!informed)
      {
        ignmsg << "Waiting for robots to spawn" << std::endl;
        informed = true;
      }
      return;
    }
    else if (this->dataPtr->robotsSpawnTime <= 0)
    {
      this->dataPtr->robotsSpawnTime = s;
    }

    // robots have spawned, now wait for the initial levels to load
    if (s < this->dataPtr->robotsSpawnTime + 15.0)
    {
      static bool informed = false;
      if (!informed)
      {
        ignmsg << "Waiting for levels to load" << std::endl;
        informed = true;
      }

      return;
    }

    this->dataPtr->started = true;
  }

  // Wait for a camera to spawn.
  this->dataPtr->cameraEntity = _ecm.EntityByComponents(
      components::Name("spawned_camera"));
  if (this->dataPtr->cameraEntity == kNullEntity)
  {
    static bool once = true;
    if (once)
    {
      this->dataPtr->SpawnCamera();
      once = false;
    }
    return;
  }

  // idle state - get next event
  if (this->dataPtr->state == IDLE)
  {
    if (this->dataPtr->events.empty())
    {
      if (this->dataPtr->exitOnFinish)
        exit(0);
      return;
    }

    // get next event to record
    this->dataPtr->event = this->dataPtr->events.front();

    this->dataPtr->events.pop_front();

    auto now = std::chrono::steady_clock::now();
    this->dataPtr->statusLog << ignition::math::timePointToString(now)
        << " " << this->dataPtr->event.filename << " starting. "
        << this->dataPtr->events.size() <<  " remaining." << std::endl;

    // set camera mode
    auto cameraMode = this->dataPtr->eventCameraMode[this->dataPtr->event.type];
    this->dataPtr->cameraFollow = (cameraMode == FOLLOW_CAMERA) &&
        (this->dataPtr->event.detector != "staging_area");

    ignmsg << "Playing event: " << this->dataPtr->event.startRecordTime << " "
           << this->dataPtr->event.type << " "
           << (this->dataPtr->event.detector.empty() ?
               "" : this->dataPtr->event.detector) << " "
           << (this->dataPtr->event.model.empty() ?
               "" : this->dataPtr->event.model) << " "
           << (this->dataPtr->event.extra.empty() ?
               "" : this->dataPtr->event.extra) << " "
           << this->dataPtr->event.robot<< std::endl;

    this->dataPtr->Seek(this->dataPtr->event.startRecordTime);
    this->dataPtr->state = SEEK_BEGIN;
    ignmsg << "State: Transitioning to SEEK_BEGIN" <<  std::endl;
    return;
  }

  // seek event state - seek to time of event and get robot pose so we can
  // move camera to a pose where the event occurred
//   if (this->dataPtr->state == SEEK_EVENT)
//   {
//     // time of event - find robot and set up camera
//     if (s == static_cast<int>(this->dataPtr->event.time))
//     {
//       // wait for a few real time seconds after arriving at time of event
//       if (!this->dataPtr->waiting)
//       {
//         this->dataPtr->eventManager->Emit<events::Pause>(true);
//         this->dataPtr->waiting = true;
//         this->dataPtr->waitStartTime = std::chrono::system_clock::now();
//         return;
//       }
//       else if (t - this->dataPtr->waitStartTime > std::chrono::seconds(5))
//       {
//         this->dataPtr->waiting = false;
//         this->dataPtr->eventManager->Emit<events::Pause>(false);
//       }
//       else
//       {
//         return;
//       }
//
//       // get pose of robot for the current event
//       auto entity = _ecm.EntityByComponents(
//           components::Name(this->dataPtr->event.robot),
//           components::Model());
//       if (entity == kNullEntity)
//       {
//         ignerr << "Unable to record event. Failed to get robot with name: "
//                << this->dataPtr->event.robot << std::endl;
//         this->dataPtr->state = IDLE;
//         return;
//       }
//
//       auto poseComp = _ecm.Component<components::Pose>(entity);
//       if (!poseComp)
//       {
//         ignerr << "Unable to record event. Failed to get pose for robot: "
//                << this->dataPtr->event.robot << std::endl;
//         this->dataPtr->state = IDLE;
//         return;
//       }
//
//       math::Pose3d pose = poseComp->Data();
//
//       // rock fall event: move camera to some offset above rock fall model and
//       // orient it to face down
//       if (this->dataPtr->event.type == "rock_fall")
//       {
//         auto it = this->dataPtr->rockModelPose.find(this->dataPtr->event.model);
//         if (it != this->dataPtr->rockModelPose.end())
//         {
//           math::Pose3d p = it->second;
//           p.Pos() += math::Vector3d(-12.5, -12.5, 5.5);
//           p.Rot() = math::Quaterniond(0, 0.4, IGN_PI/4.0);
//           this->dataPtr->MoveTo(p);
//         }
//         else
//         {
//           this->dataPtr->MoveTo(this->dataPtr->event.robot);
//         }
//       }
//       // staging area event: move camera to somewhere above the staging area
//       // looking down at all the robots
//       else if (this->dataPtr->event.type == "detect" &&
//           this->dataPtr->event.detector == "staging_area")
//       {
//         math::Pose3d p(-3.5, 0, 5, 0, 0.4, 0);
//         this->dataPtr->MoveTo(p);
//       }
//       // all other events: move gui camera to robot for now
//       else
//       {
//         this->dataPtr->MoveTo(this->dataPtr->event.robot);
//       }
//
//       // seek to a time x min before the event.
//       this->dataPtr->Seek(this->dataPtr->event.startRecordTime);
//       this->dataPtr->state = SEEK_BEGIN;
//       ignmsg << "State: Transitioning to SEEK_BEGIN" <<  std::endl;
//     }
//
//     return;
//   }

  // seek begin state - seek playback to x min before the event and start
  // recording
  if (this->dataPtr->state == SEEK_BEGIN)
  {
    if (s == static_cast<int>(this->dataPtr->event.startRecordTime))
    {
      // wait for a few real time seconds after arriving at time before event
      if (!this->dataPtr->waiting)
      {
        this->dataPtr->eventManager->Emit<events::Pause>(true);
        this->dataPtr->waiting = true;
        this->dataPtr->waitStartTime = std::chrono::system_clock::now();
        return;
      }
      else if (t - this->dataPtr->waitStartTime > std::chrono::seconds(5))
      {
        this->dataPtr->waiting = false;
        this->dataPtr->eventManager->Emit<events::Pause>(false);
      }
      else
      {
        return;
      }

      // spawn a light if needed
      // we need to spawn a light on every seek event because new entities
      // that get spawned in playback are removed when jumping back in time
      // if (this->dataPtr->spawnLight)
      // {
      //   auto lightEntity = _ecm.EntityByComponents(
      //       components::Name("spawned_light"));
      //   if (lightEntity == kNullEntity)
      //     this->dataPtr->SpawnLight();
      // }


      // if in camera follow mode, reset entity exists values so we can do
      // check to make sure the entity exists first before asking the gui
      // camera to follow it
      if (this->dataPtr->cameraFollow)
      {
        this->dataPtr->entityExists = false;
        this->dataPtr->cameraFollowing = false;
      }
      // static camera mode
      else
      {
        // rock fall event: move camera to some offset above rock fall model and
        // orient it to face down
        if (this->dataPtr->event.type == "rock_fall")
        {
          auto it = this->dataPtr->rockModelPose.find(this->dataPtr->event.model);
          if (it != this->dataPtr->rockModelPose.end())
          {
            math::Pose3d p = it->second;
            p.Pos() += math::Vector3d(-12.5, -12.5, 5.5);
            p.Rot() = math::Quaterniond(0, 0.4, IGN_PI/4.0);
            this->dataPtr->MoveTo(p);
          }
          else
          {
            ignerr << "Unable to move camera to dynamic rock model: "
                   << this->dataPtr->event.model << std::endl;
          }
        }
        // staging area event: move camera to somewhere above the staging area
        // looking down at all the robots
        else if (this->dataPtr->event.type == "detect" &&
            this->dataPtr->event.detector == "staging_area")
        {
          math::Pose3d p(-3.5, 0, 5, 0, 0.4, 0);
          this->dataPtr->MoveTo(p);
        }
      }

      this->dataPtr->waitForScene = true;
      this->dataPtr->state = RECORDING;
      ignmsg << "State: Transitioning to RECORDING" <<  std::endl;
    }
  }

  // recording state - record video to disk until y min after time of event
  if (this->dataPtr->state == RECORDING)
  {
    // pause and wait a few seconds for scene to initialize on gui
    if (this->dataPtr->waitForScene)
    {
      if (!this->dataPtr->waiting)
      {
        this->dataPtr->waitStartTime = t;
        this->dataPtr->waiting = true;
      }
      // play for a small period of time to get scene state msg over to gui
      else if (t - this->dataPtr->waitStartTime > std::chrono::milliseconds(10))
      {
        // pause and wait for scene to initialize on gui
        this->dataPtr->eventManager->Emit<events::Pause>(true);
        if (t - this->dataPtr->waitStartTime > std::chrono::seconds(15))
        {
          this->dataPtr->eventManager->Emit<events::Pause>(false);
          this->dataPtr->waiting = false;
          this->dataPtr->recorderStatsMsg.Clear();
          this->dataPtr->catchupOnRecording = false;
          // start video recording
          this->dataPtr->Record(true);
          this->dataPtr->waitForScene = false;
          ignmsg << "Recording started: " << s << "s (sim time), "
                 << rs << "s (real time)" << std::endl;
        }
      }
      return;
    }

    // catch edge case. If we seek to a time in playback that the robot has
    // not been spawned yet, e.g. beginning of sim, then we need to wait
    // for robot to spawn before sending the follow cmd
    if (this->dataPtr->cameraFollow)
    {
      if (!this->dataPtr->entityExists)
      {
        // check if robot exists
        auto entity = _ecm.EntityByComponents(
            components::Name(this->dataPtr->event.robot),
            components::Model());
        if (entity != kNullEntity)
        {
          this->dataPtr->entityExists = true;
          this->dataPtr->Follow(this->dataPtr->event.robot);
        }
      }
      else if (!this->dataPtr->cameraFollowing)
      {
        // wait for a few real time seconds for the robot entity data to be
        // ready on gui side
        if (!this->dataPtr->waiting)
        {
          this->dataPtr->waiting = true;
          this->dataPtr->waitStartTime = std::chrono::system_clock::now();
          return;
        }
        else if (t - this->dataPtr->waitStartTime > std::chrono::seconds(10))
        {
          this->dataPtr->waiting = false;
          this->dataPtr->Follow(this->dataPtr->event.robot);
          this->dataPtr->cameraFollowing = true;
        }
      }
    }

    // record until we reached the end record time or end of playback
    msgs::Time recorderStats;
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->recorderStatsMutex);
      recorderStats = this->dataPtr->recorderStatsMsg;

    }
    if (!this->dataPtr->recordStopRequested)
    {
      // check for end of playback, indicated by paused state and
      // also double check against log end time
      bool endOfPlayback = false;
      if (_info.paused && recorderStats.sec() >=
          (this->dataPtr->logEndTime -
           this->dataPtr->event.startRecordTime - 1.0))
      {
        endOfPlayback = true;
        ignmsg << "Possible end of Playback reached. Stopping video recorder"
               << std::endl;
      }
      else
      {
        // check if we need to catch up on recordiing
        int lag = (s - this->dataPtr->event.startRecordTime) -
            recorderStats.sec();
        if (!this->dataPtr->catchupOnRecording)
        {
          if (recorderStats.sec() != 0 && lag > 10)
          {
            this->dataPtr->eventManager->Emit<events::Pause>(true);
            this->dataPtr->catchupOnRecording = true;
            ignmsg << "Recording Video: " << recorderStats.sec()
                   << "s (sim time)" << std::endl;
          }
        }
        else
        {
          if (lag < 5)
          {
            this->dataPtr->eventManager->Emit<events::Pause>(false);
            this->dataPtr->catchupOnRecording = false;
          }
        }
      }

      if (endOfPlayback || recorderStats.sec() >=
          static_cast<int>(this->dataPtr->event.endRecordTime -
          this->dataPtr->event.startRecordTime))
      {
        // stop recording
        this->dataPtr->Record(false);
        this->dataPtr->recordStopRequested = true;
        this->dataPtr->recordStopTime = std::chrono::system_clock::now();
        ignmsg << "Recording stopped: " << recorderStats.sec()
               << "s (sim time), " << rs << "s (real time)" << std::endl;

        // disable camera following
        if (this->dataPtr->cameraFollow)
        {
          this->dataPtr->Follow(std::string());
          this->dataPtr->cameraFollowing = false;
          this->dataPtr->cameraFollow = false;
        }
        return;
      }
    }

    // Video recording stopped. We need to save a copy of the video file
    if (this->dataPtr->recordStopRequested)
    {
      // give it some time for video encording to finiish encoding
      std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
      // Wait for the tmp file to exist
      if (!common::exists(this->dataPtr->tmpVideoFilename) &&
          (now - this->dataPtr->recordStopTime < std::chrono::seconds(120)))
      {
        return;
      }

      if (common::exists(this->dataPtr->tmpVideoFilename))
      {
        common::moveFile(this->dataPtr->tmpVideoFilename,
            this->dataPtr->event.filename);

        ignmsg << "Saving video recording to:  "
          << this->dataPtr->event.filename <<  std::endl;

        auto now = std::chrono::steady_clock::now();
        this->dataPtr->statusLog << ignition::math::timePointToString(now)
          << " Saving video recording to:  "
          << this->dataPtr->event.filename <<  std::endl;

        // Remove old temp file, if it exists.
        std::remove(this->dataPtr->tmpVideoFilename.c_str());
      }
      this->dataPtr->recordStopRequested = false;

      this->dataPtr->state = IDLE;
      ignmsg << "State: Transitioning to IDLE" << std::endl;
      return;
    }
  }
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::MoveTo(const math::Pose3d &_pose)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending move to request" << std::endl;
  };

  ignition::msgs::GUICamera req;
  msgs::Set(req.mutable_pose(), _pose);
  if (this->node.Request(this->moveToPoseService, req, cb))
  {
    igndbg << "Move to pose: " << _pose << std::endl;
  }
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::MoveTo(const std::string &_entity)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending move to request" << std::endl;
  };

  ignition::msgs::StringMsg req;
  req.set_data(_entity);
  if (this->node.Request(this->moveToService, req, cb))
  {
    igndbg << "Move to entity: " << _entity << std::endl;
  }
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::Follow(const std::string &_entity)
{
  this->targetEntityName = _entity;
  this->targetEntity = kNullEntity;

  if (this->event.type == "dynamic_collapse")
     this->targetOffset.X(1.5);
   else
     this->targetOffset.X(-1.5);
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::Seek(double _timeSec)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending playback control request" << std::endl;
  };

  msgs::LogPlaybackControl playbackMsg;

  playbackMsg.mutable_seek()->set_sec(_timeSec);
  playbackMsg.mutable_seek()->set_nsec(0.0);
  playbackMsg.set_pause(false);
  if (this->node.Request(
      "/world/default/playback/control",
      playbackMsg, cb))
  {
    igndbg << "Seek to time: " << _timeSec << std::endl;
  }
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::Record(bool _record)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending record request" << std::endl;
  };

  ignition::msgs::VideoRecord req;

  if (_record)
  {
    std::string filename = this->tmpVideoFilename;
    req.set_start(true);
    req.set_format(this->videoFormat);
    req.set_save_filename(filename);
    igndbg << "Recording video " << filename << std::endl;
  }
  else
  {
    igndbg << "Stopping video recorder" << std::endl;
    req.set_stop(true);
  }
  this->node.Request(this->videoRecordService, req, cb);
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::SpawnLight()
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &, const bool _result)
  {
    if (!_result)
      ignerr << "Error spawning light request" << std::endl;
  };

  ignition::msgs::EntityFactory req;
  std::string spawnStr = "<?xml version=\"1.0\" ?>" \
      "<sdf version=\"1.6\">"\
      "<light name=\"spawned_light\" type=\"directional\">"\
      "<pose>0 0 10 0 0 0</pose>"\
      "</light>"\
      "</sdf>";
  req.set_sdf(spawnStr);
  req.set_allow_renaming(false);
  // for factory service requests
  std::string createService = "/world/" + this->logWorldName
      + "/create";
  this->node.Request(createService, req, cb);
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::SpawnCamera()
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error spawning camera request" << std::endl;
  };

  std::string spawnStr = R"spawn(<?xml version="1.0"?>
    <sdf version="1.6">
    <model name="spawned_camera">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <camera>
            <horizontal_fov>1.5708</horizontal_fov>
            <image>
              <width>1920</width>
              <height>1080</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>100</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>true</visualize>
          <topic>camera</topic>

          <plugin
            filename="ignition-gazebo-camera-video-recorder-system"
            name="ignition::gazebo::systems::CameraVideoRecorder">
            <service>/subt/camera/record_video</service>
            <use_sim_time>true</use_sim_time>
            <bitrate>10070000</bitrate>
          </plugin>

        </sensor>
      </link>
    </model>
    </sdf>)spawn";

  ignition::msgs::EntityFactory req;
  req.set_sdf(spawnStr);
  req.set_allow_renaming(false);
  // for factory service requests
  std::string createService = "/world/" + this->logWorldName + "/create";
  this->node.Request(createService, req, cb);
}

//////////////////////////////////////////////////
void PlaybackEventRecorderPrivate::OnRecorderStats(const msgs::Time &_msg)
{
  std::lock_guard<std::mutex> lock(this->recorderStatsMutex);
  this->recorderStatsMsg = _msg;
}
