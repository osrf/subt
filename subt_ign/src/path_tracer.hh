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

#include <mutex>
#include <map>
#include <iostream>
#include <condition_variable>

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/common/Time.hh>
#include <ignition/transport/log/Playback.hh>

// # Usage:
//
// 1. Run the SubT world using the `path_tracer.ign` launch file along with
// an IGN_PARTITION name of PATH_TRACER. For example
//
//     $ IGN_PARTITION=PATH_TRACER ign launch -v 4 path_tracer.ign worldName:=cave_qual
//
// 2. Run this program by passing in the directory that contains the simulation log files. Optionally specify a configuration file. For example:
//
//     $ ./path_tracer /data/logs/ /home/developer/path_tracer.yml
//
//
// # Sample YAML configuration file:
//
// rtf: 4.0
// incorrect_report_color:
//   ambient:
//     r: 1.0
//     g: 0.0
//     b: 0.0
//     a: 0.5
//   diffuse:
//     r: 1.0
//     g: 0.0
//     b: 0.0
//     a: 0.5
//   emissive:
//     r: 0.2
//     g: 0.0
//     b: 0.0
//     a: 0.1
// correct_report_color:
//   ambient:
//     r: 0.0
//     g: 1.0
//     b: 0.0
//     a: 1.0
//   diffuse:
//     r: 0.0
//     g: 1.0
//     b: 0.0
//     a: 1.0
//   emissive:
//     r: 0.0
//     g: 1.0
//     b: 0.0
//     a: 1.0
// artficat_location_color:
//   ambient:
//     r: 0.0
//     g: 1.0
//     b: 1.0
//     a: 0.5
//   diffuse:
//     r: 0.0
//     g: 1.0
//     b: 1.0
//     a: 0.5
//   emissive:
//     r: 0.0
//     g: 0.2
//     b: 0.2
//     a: 0.5
// robot_colors:
//   - color:
//     ambient:
//       r: 0.6
//       g: 0.0
//       b: 1.0
//       a: 1.0
//     diffuse:
//       r: 0.6
//       g: 0.0
//       b: 1.0
//       a: 1.0
//     emissive:
//       r: 0.6
//       g: 0.0
//       b: 1.0
//       a: 1.0
//   - color:
//     ambient:
//       r: 0.678
//       g: 0.2
//       b: 1.0
//       a: 1.0
//     diffuse:
//       r: 0.678
//       g: 0.2
//       b: 1.0
//       a: 1.0
//     emissive:
//       r: 0.678
//       g: 0.2
//       b: 1.0
//       a: 1.0

// Forward declaration.
class Processor;

/// \brief Type of data, which is used to determine how to visualize the
/// data.
enum DataType
{
  ROBOT  = 0,
  REPORT = 1,
  PHASE = 2
};

/// \brief Color properties for a marker.
class MarkerColor
{
  /// \brief Default constructor.
  public: MarkerColor() = default;

  /// \brief Create from YAML.
  /// \param[in] _node YAML node.
  public: MarkerColor(const YAML::Node &_node);

  /// \brief Copy constructor
  /// \param[in] _clr MarkerColor to copy.
  public: MarkerColor(const MarkerColor &_clr);

  /// \brief Constructor
  /// \param[in] _ambient Ambient color
  /// \param[in] _diffuse Diffuse color
  /// \param[in] _emissive Emissive color
  public: MarkerColor(const ignition::math::Color &_ambient,
                      const ignition::math::Color &_diffuse,
                      const ignition::math::Color &_emissive);

  /// \brief The ambient value
  public: ignition::math::Color ambient;

  /// \brief The diffuse value
  public: ignition::math::Color diffuse;

  /// \brief The emissive value
  public: ignition::math::Color emissive;
};

/// \brief Base class for data visualization
class Data
{
  /// \brief The type of data.
  public: DataType type;

  /// \brief Pure virtual render function.
  /// \param[in] _p Pointer to the processor.
  public: virtual void Render(Processor *_p) = 0;
};

/// \brief RobotPoseData contains all pose data for a single pose
/// message callback.
class RobotPoseData : public Data
{
  /// \brief All pose information.
  public: std::map<std::string, std::vector<ignition::math::Pose3d>> poses;

  /// \brief The render function for robot pose data.
  public: void Render(Processor *_p);
};

/// \brief Artifact report data
class ReportData : public Data
{
  /// \brief Position of the artifact report.
  public: ignition::math::Vector3d pos;

  /// \brief Change in score.
  public: int score;

  /// \brief The render function for artifact report data.
  public: void Render(Processor *_p);
};

/// \brief Phase change data
class PhaseData : public Data
{
  public: void Render(Processor *_p);
};

/// \brief The log file processor.
class Processor
{
  /// \brief Constructor, which also kicks off all of the data
  /// visualization.
  /// \param[in] _path Path to the directory containing the log files.
  /// \param[in] _configPath Path to the configuration file, or empty string.
  /// \param[in] _partition Ignition transport partition.
  /// \param[in] _cameraPose Initial camera pose.
  /// \param[in] _worldName Name of the world.
  /// \param[in] _onlyCheck True to only check, anot make a video.
  public: Processor(const std::string &_path,
                    const std::string &_configPath,
                    const std::string &_partition,
                    const std::string &_cameraPose,
                    const std::string &_worldName,
                    bool _onlyCheck);

  /// \brief Destructor
  public: ~Processor();

  /// \brief Clear all of the markers.
  public: void ClearMarkers();

  /// Playback a log file.
  public: void Playback(std::string _path);

  /// Subscribe to the artifact poses.
  public: void SubscribeToArtifactPoseTopics();

  /// \brief Get the artifact poses.
  /// \param[in] _msg Pose message.
  public: void ArtifactCb(const ignition::msgs::Pose_V &_msg);

  /// \brief Display the poses.
  public: void DisplayPoses();

  /// \brief Display the artifacts.
  public: void DisplayArtifacts();

  /// \brief Helper function that spawns a visual marker.
  /// \param[in] _color Color of the visual marker.
  /// \param[in] _pos Position of the visual marker.
  /// \param[in] _type Type of the visual marker.
  /// \param[in] _scale scale of the visual marker.
  public: void SpawnMarker(MarkerColor &_color,
    const ignition::math::Vector3d &_pos,
    ignition::msgs::Marker::Type _type,
    const ignition::math::Vector3d &_scale);

  /// \brief This callback is triggered on every pose message in the log file.
  public: void Cb(const ignition::msgs::Pose_V &_msg);

  /// \brief Callback for the clock message.
  /// \param[in] _msg Clock message
  public: void ClockCb(const ignition::msgs::Clock &_msg);

  /// \brief Recorder stats callback.
  /// \param[in] _msg Recorder stats message.
  public: void RecorderStatsCb(const ignition::msgs::Time &_msg);

  /// \brief Step simulation to a specific time.
  /// \param[in] _sec Simulation seconds.
  public: void StepUntil(int _sec);

  /// \brief Pause simulation.
  /// \param[in] _pause True to pause simulation.
  public: void Pause(bool _pause);

  /// \brief Mapping of robot name to color
  public: std::map<std::string, MarkerColor> robots;

  /// \brief The colors used to represent artifacts and reports.
  public: std::map<std::string, MarkerColor> artifactColors;

  /// \brief The colors used to represent each robot.
  public: std::vector<MarkerColor> robotColors;

  /// \brief Last pose of a robot. This is used to reduce the number of markers.
  private: std::map<std::string, ignition::math::Pose3d> prevPose;

  /// \brief Artifacts and their pose information.
  private:std::map<std::string, ignition::math::Pose3d> artifacts;

  /// \brief Marker ID, used to create unique markers.
  private: int markerId = 0;

  /// \brief Node that will display the visual markers.
  private: std::unique_ptr<ignition::transport::Node> markerNode;

  /// \brief A mutex.
  private: std::mutex mutex;

  /// \brief A condition variable.
  private: std::condition_variable cv;

  /// \brief All of the pose data.
  private: std::map<int, std::vector<std::unique_ptr<Data>>> logData;

  /// \brief Realtime factor for playback.
  private: double rtf = 1.0;

  private: std::vector<int> scoreTimes;

  private: int simTime;
  private: int startSimTime = 0;
  private: int nextSimTime = -1;
  private: std::mutex stepMutex;
  private: std::condition_variable stepCv;
  private: std::string worldName;

  /// \brief Recorder stats msg.
  public: ignition::msgs::Time recorderStatsMsg;


  /// \brief Mutex to protect recorder stats msg.
  public: std::mutex recorderStatsMutex;
};
