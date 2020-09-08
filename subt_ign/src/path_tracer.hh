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

// Usage:
//
// 1. Run the SubT world using the `path_tracer.ign` launch file along with
// an IGN_PARTITION name of PATH_TRACER. For example
//
//     $ IGN_PARTITION=PATH_TRACER ign launch -v 4 path_tracer.ign worldName:=cave_qual
//
// 2. Run this program by passing in the directory that contains the simulation log files. Optionally specify the number of milliseconds to sleep between time step playback. If not specified a value of 20ms will be used. For example:
//
//     $ ./path_tracer /data/logs/ 100

// Forward declaration.
class Processor;

/// \brief Type of data, which is used to determine how to visualize the
/// data.
enum DataType
{
  ROBOT  = 0,
  REPORT = 1
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

/// \brief The log file processor.
class Processor
{
  /// \brief Constructor, which also kicks off all of the data
  /// visualization.
  /// \param[in] _path Path to the directory containing the log files.
  /// \param[in] _rtf Real time factor for playback.
  public: Processor(const std::string &_path, double _rtf);

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
  public: void SpawnMarker(ignition::math::Color &_color,
    const ignition::math::Vector3d &_pos,
    ignition::msgs::Marker::Type _type,
    const ignition::math::Vector3d &_scale);

  /// \brief This callback is triggered on every pose message in the log file.
  public: void Cb(const ignition::msgs::Pose_V &_msg);

  /// \brief Mapping of robot name to color
  public: std::map<std::string, ignition::math::Color> robots;

  /// \brief The colors used to represent each robot.
  public: std::vector<ignition::math::Color> colors =
  {
    // Bad Report locations
    {1.0, 0.0, 0.0, 0.1},

    // Good Artifact locations
    {0.0, 1.0, 0.0, 0.1},

    // Artifact locations.
    {0.0, 1.0, 1.0, 0.1},

    // Robot colors
    {153/255.0, 0, 1},
    {173/255.0, 51/255.0, 1},
    {194/255.0, 102/255.0, 1},
    {214/255.0, 153/255.0, 1},

    {1, 153/255.0, 0},
    {1, 173/255.0, 51/255.0},
    {1, 194/255.0, 102/255.0},
  };

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
};
