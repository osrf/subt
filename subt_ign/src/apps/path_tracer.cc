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
#include "path_tracer.hh"

//////////////////////////////////////////////////
// Load a color from YAML helper function.
// \param[in] _node YAML node that contains color information
// \param[in] _def Default color value that is used if a color property is
// missing.
ignition::math::Color loadColor(const YAML::Node &_node, double _def = 1.0)
{
  ignition::math::Color clr(_def, _def, _def, _def);

  if (_node["r"])
    clr.R(_node["r"].as<double>());

  if (_node["g"])
    clr.G(_node["g"].as<double>());

  if (_node["b"])
    clr.B(_node["b"].as<double>());

  if (_node["a"])
    clr.A(_node["a"].as<double>());

  return clr;
}

//////////////////////////////////////////////////
MarkerColor::MarkerColor(const YAML::Node &_node)
{
  if (_node["ambient"])
    this->ambient = loadColor(_node["ambient"]);

  if (_node["diffuse"])
    this->diffuse = loadColor(_node["diffuse"]);

  if (_node["emissive"])
    this->emissive = loadColor(_node["emissive"]);
}

//////////////////////////////////////////////////
MarkerColor::MarkerColor(const ignition::math::Color &_ambient,
    const ignition::math::Color &_diffuse,
    const ignition::math::Color &_emissive)
  : ambient(_ambient), diffuse(_diffuse), emissive(_emissive)
{
}

//////////////////////////////////////////////////
Processor::Processor(const std::string &_path, const std::string &_configPath)
{
  YAML::Node cfg;
  if (!_configPath.empty())
  {
    if (ignition::common::exists(_configPath))
    {
      try
      {
        cfg = YAML::LoadFile(_configPath);
      }
      catch (YAML::Exception &ex)
      {
        std::cerr << "Unable to load configuration file["
          << _configPath << "]: "
          << "error at line " << ex.mark.line + 1 << ", column "
          << ex.mark.column + 1 << ": " << ex.msg << "\n";
      }
    }
    else
    {
      std::cerr << "Configuration file[" << _configPath << "] doesn't exist\n";
    }
  }

  // Set the RTF value
  if (cfg && cfg["rtf"])
    this->rtf = cfg["rtf"].as<double>();

  // Color of incorrect reports.
  if (cfg && cfg["incorrect_report_color"])
  {
    this->artifactColors["incorrect_report_color"] =
      MarkerColor(cfg["incorrect_report_color"]);
  }
  else
  {
    this->artifactColors["incorrect_report_color"] =
      MarkerColor({1.0, 0.0, 0.0, 0.5},
                  {1.0, 0.0, 0.0, 0.5},
                  {0.2, 0.0, 0.0, 0.1});
  }

  // Color of correct reports.
  if (cfg && cfg["correct_report_color"])
  {
    this->artifactColors["correct_report_color"] =
      MarkerColor(cfg["correct_report_color"]);
  }
  else
  {
    this->artifactColors["correct_report_color"] =
      MarkerColor({0.0, 1.0, 0.0, 1.0},
                  {0.0, 1.0, 0.0, 1.0},
                  {0.0, 1.0, 0.0, 1.0});
  }

  // Color of artifact locations
  if (cfg && cfg["artifact_location_color"])
  {
    this->artifactColors["artifact_location_color"] =
      MarkerColor(cfg["artifact_location_color"]);
  }
  else
  {
    this->artifactColors["artifact_location_color"] =
      MarkerColor({0.0, 1.0, 1.0, 0.5},
                  {0.0, 1.0, 1.0, 0.5},
                  {0.0, 0.2, 0.2, 0.5});
  }

  // Color of robot paths
  if (cfg && cfg["robot_colors"])
  {
    for (std::size_t i = 0; i < cfg["robot_colors"].size(); ++i)
    {
      this->robotColors.push_back(MarkerColor(cfg["robot_colors"][i]));
    }
  }
  else
  {
    this->robotColors.push_back(
      MarkerColor({0.6, 0.0, 1.0, 1.0},
                  {0.6, 0.0, 1.0, 1.0},
                  {0.6, 0.0, 1.0, 1.0}));
    this->robotColors.push_back(
      MarkerColor({0.678, 0.2, 1.0, 1.0},
                  {0.678, 0.2, 1.0, 1.0},
                  {0.678, 0.2, 1.0, 1.0}));
    this->robotColors.push_back(
      MarkerColor({0.761, 0.4, 1.0, 1.0},
                  {0.761, 0.4, 1.0, 1.0},
                  {0.761, 0.4, 1.0, 1.0}));
    this->robotColors.push_back(
      MarkerColor({0.839, 0.6, 1.0, 1.0},
                  {0.839, 0.6, 1.0, 1.0},
                  {0.839, 0.6, 1.0, 1.0}));
    this->robotColors.push_back(
      MarkerColor({1.0, 0.6, 0.0, 1.0},
                  {1.0, 0.6, 0.0, 1.0},
                  {1.0, 0.6, 0.0, 1.0}));
    this->robotColors.push_back(
      MarkerColor({1.0, 0.678, 0.2, 1.0},
                  {1.0, 0.678, 0.2, 1.0},
                  {1.0, 0.678, 0.2, 1.0}));
    this->robotColors.push_back(
      MarkerColor({1.0, 0.761, 0.4, 1.0},
                  {1.0, 0.761, 0.4, 1.0},
                  {1.0, 0.761, 0.4, 1.0}));
  }

  // Create the transport node with the partition used by simulation
  // world.
  ignition::transport::NodeOptions opts;
  opts.SetPartition("PATH_TRACER");
  this->markerNode = std::make_unique<ignition::transport::Node>(opts);
  this->ClearMarkers();

  // Subscribe to the artifact poses.
  this->SubscribeToArtifactPoseTopics();

  // Playback the log file.
  std::unique_lock<std::mutex> lock(this->mutex);
  std::thread playbackThread(std::bind(&Processor::Playback, this, _path));

  this->cv.wait(lock);

  // Create a transport node that uses the default partition.
  ignition::transport::Node node;

  // Subscribe to the robot pose topic
  bool subscribed = false;
  for (int i = 0; i < 5 && !subscribed; ++i)
  {
    std::vector<std::string> topics;
    node.TopicList(topics);

    // Subscribe to the first /dynamic_pose/info topic
    for (auto const &topic : topics)
    {
      if (topic.find("/dynamic_pose/info") != std::string::npos)
      {
        // Subscribe to a topic by registering a callback.
        if (!node.Subscribe(topic, &Processor::Cb, this))
        {
          std::cerr << "Error subscribing to topic ["
            << topic << "]" << std::endl;
          return;
        }
        subscribed = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Wait for log playback to end.
  playbackThread.join();

  // Process the events log file.
  std::string eventsFilepath = _path + "/events.yml";
  if (ignition::common::exists(eventsFilepath))
  {
    YAML::Node events;
    try
    {
      events = YAML::LoadFile(eventsFilepath);
    }
    catch (...)
    {
      // There was a bug in the events.yml generation that will be fixed
      // before Cave Circuit. The replaceAll can be removed after Cave Circuit,
      // but leaving this code in place also shouldn't hurt anything.
      std::ifstream t(eventsFilepath);
      std::string ymlStr((std::istreambuf_iterator<char>(t)),
          std::istreambuf_iterator<char>());
      ignition::common::replaceAll(ymlStr, ymlStr,
          "_time ", "_time: ");
      try
      {
        events = YAML::Load(ymlStr);
      }
      catch (...)
      {
        std::cerr << "Error processing " << eventsFilepath
          << ". Please check the the YAML file has correct syntax. "
          << "There will be no artifact report visualization.\n";
      }
    }

    for (std::size_t i = 0; i < events.size(); ++i)
    {
      if (events[i]["type"].as<std::string>() == "artifact_report_attempt")
      {
        ignition::math::Vector3d reportedPos;

        // Read the reported pose.
        std::stringstream stream;
        stream << events[i]["reported_pose"].as<std::string>();
        stream >> reportedPos;

        int sec = events[i]["time_sec"].as<int>();
        std::unique_ptr<ReportData> data = std::make_unique<ReportData>();
        data->type = REPORT;
        data->pos = reportedPos;
        data->score = events[i]["points_scored"].as<int>();

        this->logData[sec].push_back(std::move(data));
      }
    }
  }
  else
  {
    std::cerr << "Missing " << eventsFilepath
      << ". There will be no artifact report visualization.\n";
  }
  // Display all of the artifacts using visual markers.
  this->DisplayArtifacts();

  // Display all of the poses using visual markers.
  this->DisplayPoses();
}

//////////////////////////////////////////////////
Processor::~Processor()
{
}

//////////////////////////////////////////////////
void Processor::ClearMarkers()
{
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  markerNode->Request("/marker", markerMsg);
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

//////////////////////////////////////////////////
void Processor::Playback(std::string _path)
{
  std::unique_lock<std::mutex> lock(this->mutex);
  ignition::transport::log::Playback player(_path + "/state.tlog");
  bool valid = false;
  ignition::transport::log::PlaybackHandlePtr handle;

  // Playback all topics
  const int64_t addTopicResult = player.AddTopic(std::regex(".*"));
  if (addTopicResult == 0)
  {
    std::cerr << "No topics to play back\n";
  }
  else if (addTopicResult < 0)
  {
    std::cerr << "Failed to advertise topics: " << addTopicResult << "\n";
  }
  else
  {
    // Begin playback
    handle = player.Start(std::chrono::seconds(5), false);
    if (!handle)
    {
      std::cerr << "Failed to start playback\n";
      return;
    }
    valid = true;
  }
  cv.notify_all();
  lock.unlock();

  // Wait until the player stops on its own
  if (valid)
  {
    std::cerr << "Playing all messages in the log file\n";
    handle->WaitUntilFinished();
  }
}

/////////////////////////////////////////////////
void Processor::SubscribeToArtifactPoseTopics()
{
  bool subscribed = false;
  for (int i = 0; i < 5 && !subscribed; ++i)
  {
    std::vector<std::string> topics;
    this->markerNode->TopicList(topics);

    for (auto const &topic : topics)
    {
      if (topic.find("/pose/info") != std::string::npos)
      {
        this->markerNode->Subscribe(topic, &Processor::ArtifactCb, this);
        subscribed = true;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

/////////////////////////////////////////////////
void Processor::ArtifactCb(const ignition::msgs::Pose_V &_msg)
{
  // Process each pose in the message.
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    // Only consider artifacts.
    std::string name = _msg.pose(i).name();
    if (name.find("rescue") == 0 ||
        name.find("backpack") == 0 ||
        name.find("vent") == 0 ||
        name.find("gas") == 0 ||
        name.find("drill") == 0 ||
        name.find("extinguisher") == 0 ||
        name.find("phone") == 0 ||
        name.find("rope") == 0 ||
        name.find("helmet") == 0 ||
        name.find("cube") == 0)
    {
      ignition::math::Pose3d pose = ignition::msgs::Convert(_msg.pose(i));
      this->artifacts[name] = pose;
    }
  }
}

//////////////////////////////////////////////////
void Processor::DisplayPoses()
{
  for (std::map<int, std::vector<std::unique_ptr<Data>>>::iterator iter =
       this->logData.begin(); iter != this->logData.end(); ++iter)
  {
    auto start = std::chrono::steady_clock::now();
    printf("\r %ds/%ds (%06.2f%%)", iter->first, this->logData.rbegin()->first,
        static_cast<double>(iter->first) / this->logData.rbegin()->first * 100);
    fflush(stdout);

    for (std::unique_ptr<Data> &data : iter->second)
    {
      data->Render(this);
    }

    // Get the next time stamp, and sleep the correct amount of time.
    auto next = std::next(iter, 1);
    if (next != this->logData.end())
    {
      int sleepTime = (((next->first - iter->first) / this->rtf)*1000);
      auto duration = std::chrono::steady_clock::now() - start;
      std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime)  -
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            duration));
    }
  }
}

/////////////////////////////////////////////////
void Processor::DisplayArtifacts()
{
  for (const auto &artifact : this->artifacts)
  {
    this->SpawnMarker(this->artifactColors["artifact_location_color"],
        artifact.second.Pos(),
        ignition::msgs::Marker::SPHERE,
        ignition::math::Vector3d(8, 8, 8));
  }
}

//////////////////////////////////////////////////
void Processor::SpawnMarker(MarkerColor &_color,
    const ignition::math::Vector3d &_pos,
    ignition::msgs::Marker::Type _type,
    const ignition::math::Vector3d &_scale)
{
  // Create the marker message
  ignition::msgs::Marker markerMsg;
  ignition::msgs::Material matMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(this->markerId++);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(_type);
  markerMsg.set_visibility(ignition::msgs::Marker::GUI);

  // Set color
  markerMsg.mutable_material()->mutable_ambient()->set_r(_color.ambient.R());
  markerMsg.mutable_material()->mutable_ambient()->set_g(_color.ambient.G());
  markerMsg.mutable_material()->mutable_ambient()->set_b(_color.ambient.B());
  markerMsg.mutable_material()->mutable_ambient()->set_a(_color.ambient.A());
  markerMsg.mutable_material()->mutable_diffuse()->set_r(_color.diffuse.R());
  markerMsg.mutable_material()->mutable_diffuse()->set_g(_color.diffuse.G());
  markerMsg.mutable_material()->mutable_diffuse()->set_b(_color.diffuse.B());
  markerMsg.mutable_material()->mutable_diffuse()->set_a(_color.diffuse.A());
  markerMsg.mutable_material()->mutable_emissive()->set_r(_color.emissive.R());
  markerMsg.mutable_material()->mutable_emissive()->set_g(_color.emissive.G());
  markerMsg.mutable_material()->mutable_emissive()->set_b(_color.emissive.B());
  markerMsg.mutable_material()->mutable_emissive()->set_a(_color.emissive.A());

  ignition::msgs::Set(markerMsg.mutable_scale(), _scale);

  // The rest of this function adds different shapes and/or modifies shapes.
  // Read the terminal statements to figure out what each node.Request
  // call accomplishes.
  ignition::msgs::Set(markerMsg.mutable_pose(),
      ignition::math::Pose3d(_pos.X(), _pos.Y(), _pos.Z(), 0, 0, 0));
  this->markerNode->Request("/marker", markerMsg);
}

//////////////////////////////////////////////////
void Processor::Cb(const ignition::msgs::Pose_V &_msg)
{
  std::unique_ptr<RobotPoseData> data(new RobotPoseData);

  // Process each pose in the message.
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    // Only conder robots.
    std::string name = _msg.pose(i).name();
    if (name.find("_wheel") != std::string::npos ||
        name.find("rotor_") != std::string::npos || name == "base_link") {
      continue;
    }

    ignition::math::Pose3d pose = ignition::msgs::Convert(_msg.pose(i));

    if (this->robots.find(name) == this->robots.end())
    {
      this->robots[name] = this->robotColors[
        this->robots.size() % this->robotColors.size()];
      this->prevPose[name] = pose;
    }

    // Filter poses.
    if (this->prevPose[name].Pos().Distance(pose.Pos()) > 1.0)
    {
      data->poses[name].push_back(pose);
      this->prevPose[name] = pose;
    }
  }

  int sec = _msg.header().stamp().sec();
  // Store data.
  if (!data->poses.empty())
    this->logData[sec].push_back(std::move(data));
}

/////////////////////////////////////////////////
void RobotPoseData::Render(Processor *_p)
{
  // Render the paths using colored spheres.
  for (std::map<std::string,
      std::vector<ignition::math::Pose3d>>::const_iterator
      iter = this->poses.begin(); iter != this->poses.end(); iter++)
  {
    for (const ignition::math::Pose3d &p : iter->second)
    {
      _p->SpawnMarker(_p->robots[iter->first],
          p.Pos() + ignition::math::Vector3d(0, 0, 0.5),
          ignition::msgs::Marker::SPHERE,
          ignition::math::Vector3d(1, 1, 1));
    }
  }
}

/////////////////////////////////////////////////
void ReportData::Render(Processor *_p)
{
  // If scored, then render a green sphere.
  // Otherwise render a red box.
  if (this->score > 0)
  {
    _p->SpawnMarker(_p->artifactColors["correct_report_color"], this->pos,
        ignition::msgs::Marker::SPHERE,
        ignition::math::Vector3d(4, 4, 4));
  }
  else
  {
    _p->SpawnMarker(_p->artifactColors["incorrect_report_color"], this->pos,
        ignition::msgs::Marker::BOX,
        ignition::math::Vector3d(4, 4, 4));
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Create and run the processor.
  if (_argc > 2)
    Processor p(_argv[1], _argv[2]);
  else
    Processor p(_argv[1]);
  std::cout << "\nPlayback complete.\n";
  return 0;
}
