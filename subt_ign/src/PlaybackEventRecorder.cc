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

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/Utility.hh>


#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include "ignition/gazebo/Events.hh"
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

    /// \brief Start time for recording video
    public: double startRecordTime = 0;

    /// \brief End time for recording video
    public: double endRecordTime = 0;
  };


  // Playback recording state
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

}

using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace subt;

/// \brief Private data class for PlaybackEventRecorder
class subt::PlaybackEventRecorderPrivate
{
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

  /// \brief Move to service name
  public: std::string followService;

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

  /// \brief A list of unique detectors
  public: std::set<std::string> detectors;
};

/////////////////////////////////////////////
PlaybackEventRecorder::PlaybackEventRecorder()
  : dataPtr(new PlaybackEventRecorderPrivate)
{
  // set up event types and recording duration:
  //   * robot deployed a marsupial child (record 1min before to 2min after)
  //   * robot deployed a breadcrumb (record 1min before to 2min after)
  //   * robot entered/exited staging area (record ~1min before entry to 1min
  //     after exit)
  //   * robot entered/exited artifact proximity (record ~1min before entry and
  //     1min after exit)
  //   * robot flipped (record 2 min before to 30sec after)
  this->dataPtr->eventRecordDuration["detach"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["breadcrumb_deploy"] = std::make_pair(60, 120);
  this->dataPtr->eventRecordDuration["detect"] = std::make_pair(60, 60);
  this->dataPtr->eventRecordDuration["flip"] = std::make_pair(120, 60);
  this->dataPtr->eventRecordDuration["rock_fall"] = std::make_pair(60, 60);

  this->dataPtr->detectors.insert("staging_area");
  this->dataPtr->detectors.insert("backpack");
  this->dataPtr->detectors.insert("phone");
  this->dataPtr->detectors.insert("rescue_randy");
  this->dataPtr->detectors.insert("rope");
  this->dataPtr->detectors.insert("helmet");
}

/////////////////////////////////////////////
PlaybackEventRecorder::~PlaybackEventRecorder()
{
}

//////////////////////////////////////////////////
void PlaybackEventRecorder::Configure(const ignition::gazebo::Entity & /*_entity*/,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &/*_ecm*/,
                           ignition::gazebo::EventManager &_eventMgr)
{
  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  if (!_sdf->HasElement("log_path"))
  {
    ignerr << "Unable to load events.yml file. <log_path> not specified."
           << std::endl;
    return;
  }

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

  if (_sdf->HasElement("camera_follow"))
  {
    this->dataPtr->cameraFollow = ptr->Get<bool>("camera_follow");
  }

  const sdf::ElementPtr logPathElem = ptr->GetElement("log_path");
  this->dataPtr->logPath = logPathElem->Get<std::string>();

  // load events.yml
  YAML::Node node = YAML::LoadFile(
      common::joinPaths(this->dataPtr->logPath, "events.yml"));

  // parse and store the list of events
  for (const auto &n : node)
  {
    std::string type = n["type"].as<std::string>();
    auto it = this->dataPtr->eventRecordDuration.find(type);
    if (it == this->dataPtr->eventRecordDuration.end())
       continue;

    Event e;
    e.type = type;

    // check for detect event type
    // and record videos only for detectors that we are interested in.
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
        state = stateParam.as<std::string>();
      else
        continue;

      e.detector = detector;
      e.state = state;
    }

    // for rock fall events, we need to check the corresponding performer
    // detector events to get the robot associated with this event
    if (type == "rock_fall")
    {
      std::string model = n["model"].as<std::string>();
      std::string suffix = model.substr(model.rfind("_"));
      for (const auto &ev : node)
      {
        if (ev["type"].as<std::string>() == "detect" &&
            ev["detector"].as<std::string>().find("rockfall" + suffix)
            != std::string::npos)
        {
          e.robot = ev["robot"].as<std::string>();
          e.time = ev["time_sec"].as<double>();
          break;
        }
      }
    }
    else
    {
      e.robot = n["robot"].as<std::string>();
      e.time = n["time_sec"].as<double>();
    }

    e.startRecordTime = std::max(e.time - it->second.first, 0.0);
    e.endRecordTime = e.time + it->second.second;
    this->dataPtr->events.push_back(e);
  }

  // don't do anything if there are not events
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
    std::cout << "  robot: " << e.robot << std::endl;
    std::cout << "  time: " << e.time << std::endl;
    std::cout << "  detector: " << ((e.detector.empty()) ? "N/A" : e.detector)
              << std::endl;
    std::cout << "  state: " << ((e.state.empty()) ? "N/A" : e.state)
              << std::endl;
  }

  this->dataPtr->eventManager = &_eventMgr;

  // For move to service requests
  this->dataPtr->moveToPoseService = "/gui/move_to/pose";
  this->dataPtr->moveToService = "/gui/move_to";

  // For follow service requests
  this->dataPtr->followService = "/gui/follow";

  // For video record requests
  this->dataPtr->videoRecordService = "/gui/record_video";

  this->dataPtr->loadTime = std::chrono::system_clock::now();
}

//////////////////////////////////////////////////
void PlaybackEventRecorder::PreUpdate(
    const ignition::gazebo::UpdateInfo &/*_info*/,
    ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
}

//////////////////////////////////////////////////
void PlaybackEventRecorder::PostUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->events.empty())
  {
    if (this->dataPtr->exitOnFinish)
      exit(0);
    return;
  }

  if (this->dataPtr->logWorldName.empty())
  {
    // Get world name recorded in log
    _ecm.Each<components::World, components::Name>(
        [&](const Entity & /*_entity*/,
          const components::World *,
          const components::Name *_name)->bool
        {
          this->dataPtr->logWorldName = _name->Data();
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

  // step the sim for a few seconds for scene to load on gui side
  if (!this->dataPtr->started)
  {
    if (_info.paused)
      this->dataPtr->eventManager->Emit<events::Pause>(false);
    if (s < 10)
      return;
    this->dataPtr->started = true;
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

    ignmsg << "Playing event: " << this->dataPtr->event.robot << ": "
           << this->dataPtr->event.type << std::endl;

    // seek to time when event occurred
    double t = this->dataPtr->event.time;
    this->dataPtr->Seek(t);
    this->dataPtr->state = SEEK_EVENT;
    ignmsg << "State: Transitioning to SEEK_EVENT" <<  std::endl;
  }

  // seek event state - seek to time of event and get robot pose so we can
  // move camera to a pose where the event occurred
  if (this->dataPtr->state == SEEK_EVENT)
  {
    // time of event - find robot and set up camera
    if (s == static_cast<int>(this->dataPtr->event.time))
    {
      // wait for 2 real time seconds after arriving at time of event
      if (!this->dataPtr->waiting)
      {
        this->dataPtr->eventManager->Emit<events::Pause>(true);
        this->dataPtr->waiting = true;
        this->dataPtr->waitStartTime = std::chrono::system_clock::now();
        return;
      }
      else if (t - this->dataPtr->waitStartTime > std::chrono::seconds(2))
      {
        this->dataPtr->waiting = false;
        this->dataPtr->eventManager->Emit<events::Pause>(false);
      }
      else
      {
        return;
      }

      // get pose of robot for the current event
      auto entity = _ecm.EntityByComponents(
          components::Name(this->dataPtr->event.robot),
          components::Model());
      if (entity == kNullEntity)
      {
        ignerr << "Unable to record event. Failed to get robot with name: "
               << this->dataPtr->event.robot << std::endl;
        this->dataPtr->state = IDLE;
        return;
      }

      auto poseComp = _ecm.Component<components::Pose>(entity);
      if (!poseComp)
      {
        ignerr << "Unable to record event. Failed to get pose for robot: "
               << this->dataPtr->event.robot << std::endl;
        this->dataPtr->state = IDLE;
        return;
      }

      math::Pose3d pose = poseComp->Data();

      // \todo(anyone) get closest static camera or move gui camera to preset pose?
      // move gui camera to robot for now
      this->dataPtr->MoveTo(this->dataPtr->event.robot);

      // seek to a time x min before the event.
      this->dataPtr->Seek(this->dataPtr->event.startRecordTime);
      this->dataPtr->state = SEEK_BEGIN;
      ignmsg << "State: Transitioning to SEEK_BEGIN" <<  std::endl;
    }

    return;
  }

  // seek begin state - seek playback to x min before the event and start
  // recording
  if (this->dataPtr->state == SEEK_BEGIN)
  {
    // make a service request to start video recording
    if (s == static_cast<int>(this->dataPtr->event.startRecordTime))
    {
      // wait for 2 real time seconds after arriving at time before event
      if (!this->dataPtr->waiting)
      {
        this->dataPtr->eventManager->Emit<events::Pause>(true);
        this->dataPtr->waiting = true;
        this->dataPtr->waitStartTime = std::chrono::system_clock::now();
        return;
      }
      else if (t - this->dataPtr->waitStartTime > std::chrono::seconds(2))
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
      if (this->dataPtr->spawnLight)
        this->dataPtr->SpawnLight();

      // if in camera follow mode, reset entity exists values so we can do
      // check to make sure the entity exists first before asking the gui
      // camera to follow it
      if (this->dataPtr->cameraFollow)
      {
        this->dataPtr->entityExists = false;
        this->dataPtr->cameraFollowing = false;
      }

      // start video recording
      this->dataPtr->Record(true);
      this->dataPtr->state = RECORDING;
      ignmsg << "State: Transitioning to RECORDING" <<  std::endl;
    }
  }

  // recording state - record video to disk until y min after time of event
  if (this->dataPtr->state == RECORDING)
  {
    // catch edge case. If we seek to a time in playback that the robot has
    // not been spawned yet, e.g. beginning of sim, then we need to wait
    // for robot to spawn before sending the follow cmd
    if (this->dataPtr->cameraFollow)
    {
      if (!this->dataPtr->entityExists)
      {
        // check if robot exists
        _ecm.Each<components::Model, components::Name>(
            [&](const Entity & /*_entity*/,
              const components::Model *,
              const components::Name *_name)->bool
            {
              if (_name->Data() == this->dataPtr->event.robot)
                this->dataPtr->entityExists = true;
              return true;
            });
      }
      else if (!this->dataPtr->cameraFollowing)
      {
        // wait for 2 real time seconds for the robot entity data to be ready on
        // gui side
        if (!this->dataPtr->waiting)
        {
          this->dataPtr->waiting = true;
          this->dataPtr->waitStartTime = std::chrono::system_clock::now();
          return;
        }
        else if (t - this->dataPtr->waitStartTime > std::chrono::seconds(2))
        {
          this->dataPtr->waiting = false;
          this->dataPtr->Follow(this->dataPtr->event.robot);
          this->dataPtr->cameraFollowing = true;
        }
      }
    }

    // wait until we reached end record time or end of playback (indicated by info.pause)
    if (!this->dataPtr->recordStopRequested &&
      (_info.paused || s == static_cast<int>(this->dataPtr->event.endRecordTime)))
    {
      // stop recording
      this->dataPtr->Record(false);
      this->dataPtr->recordStopRequested = true;
      this->dataPtr->recordStopTime = std::chrono::system_clock::now();

      // disable camera following
      if (this->dataPtr->cameraFollow)
      {
        this->dataPtr->Follow(std::string());
        this->dataPtr->cameraFollowing = false;
      }

      return;
    }

    // Video recording stopped. We need to save a copy of the video file
    if (this->dataPtr->recordStopRequested)
    {
      // give it some time for video encording to finiish encoding
      std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
      if (now - this->dataPtr->recordStopTime < std::chrono::seconds(5))
        return;

      if (common::exists(this->dataPtr->tmpVideoFilename))
      {
        std::string filename = this->dataPtr->event.robot + "_" +
            this->dataPtr->event.type + "_";
        if (!this->dataPtr->event.detector.empty())
        {
          filename += this->dataPtr->event.detector + "_" +
            this->dataPtr->event.state + "_";
        }
        filename += std::to_string(static_cast<int>(this->dataPtr->event.time))
            + "." + this->dataPtr->videoFormat;
        common::moveFile(this->dataPtr->tmpVideoFilename, filename);

        ignmsg << "Saving recording video to:  " << filename <<  std::endl;

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

  ignition::msgs::Pose req;
  req = msgs::Convert(_pose);
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
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending follow request" << std::endl;
  };

  ignition::msgs::StringMsg req;
  req.set_data(_entity);
  if (this->node.Request(this->followService, req, cb))
  {
    igndbg << "Follow entity: " << _entity << std::endl;
  }
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
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
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
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending record request" << std::endl;
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
