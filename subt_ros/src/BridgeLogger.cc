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
#include <chrono>
#include <mutex>
#include <thread>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <theora_image_transport/Packet.h>
#include <rosgraph_msgs/Clock.h>
#include <topic_tools/shape_shifter.h>
#include <ignition/common/StringUtils.hh>


// The BridgeLogger logs sensor message sequence numbers and simulation time
// for SubT sensor topics.
class BridgeLogger
{
  /// \brief Constructor
  public: BridgeLogger();

  /// \brief Destructor
  public: ~BridgeLogger();

  /// \brief Callback for the updateTimer.
  /// \param[in] _evt ros timer event.
  public: void Update(const ros::TimerEvent &_evt);

  /// \brief Callback for all sensor messages.
  /// \param[in] _msg The message.
  /// \param[in] _topic The name of the topic.
  private: void OnSensorMsg(const topic_tools::ShapeShifter::ConstPtr &_msg,
                            const std::string &_topic);

  /// \brief Check heartbeat for all topics monitored
  private: void CheckHeartbeat();

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief All of the subscribers.
  private: std::vector<ros::Subscriber> subscribers;

  /// \brief Log streams.
  private: std::map<std::string, std::ofstream> streams;

  /// \brief Previous sequence numbers. This is used to detect errors.
  private: std::map<std::string, int32_t> prevSeq;
  private: std::map<std::string, std::chrono::time_point<std::chrono::steady_clock>> startTime;

  /// \brief Just a mutex.
  private: std::mutex mutex;

  /// \brief Timer that triggers the update function.
  private: ros::Timer updateTimer;

  /// \brief A map of topic names and time when their last heartbeat was
  /// received.
  private: std::map<std::string,
      std::chrono::time_point<std::chrono::steady_clock>> topicTimestamp;

  /// \brief Check heartbeat in separate thread
  private: std::thread heartbeatThread;
};

/////////////////////////////////////////////////
BridgeLogger::BridgeLogger()
{
  // Wait for the clock.
  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->n);

  this->updateTimer = this->n.createTimer(ros::Duration(10.0),
      &BridgeLogger::Update, this);

  this->heartbeatThread = std::thread(&BridgeLogger::CheckHeartbeat, this);
}

/////////////////////////////////////////////////
void BridgeLogger::Update(const ros::TimerEvent &)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  ros::master::V_TopicInfo masterTopics;
  ros::master::getTopics(masterTopics);

  using namespace ignition::common;

  // Subscribe to some of the sensor messages.
  for (ros::master::V_TopicInfo::iterator it = masterTopics.begin();
      it != masterTopics.end(); ++it)
  {
    const ros::master::TopicInfo &info = *it;

    // Skip if the topic has already been added to `this->streams`.
    if (this->streams.find(info.name) != this->streams.end())
      continue;

    // These data are recorded in a bagfile, so no need to log them here.
    if (StartsWith(info.name, "/robot_data/"))
      continue;

    const auto parts = Split(info.name, '/');
    auto HasPart = [&parts](const std::string& _part) -> bool
    {
      return std::find(parts.begin(), parts.end(), _part) != parts.end();
    };
    if (EndsWith(info.name, "/front_scan") ||
        EndsWith(info.name, "/points") ||
        EndsWith(info.name, "/image_raw") ||
        EndsWith(info.name, "/depth") ||
        EndsWith(info.name, "/depth/image") ||  // explorer_r2
        (HasPart("imu") && HasPart("data")) ||
        HasPart("magnetic_field") ||
        EndsWith(info.name, "/air_pressure") ||
        EndsWith(info.name, "/battery_state"))
    {
      boost::function<
        void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
      callback = [this, info](
          const topic_tools::ShapeShifter::ConstPtr &_msg) -> void
      {
        this->OnSensorMsg(_msg, info.name);
      };

      // Replace "/" with "-".
      std::string cleanTopicName = info.name;
      size_t pos = 0;
      while ((pos = cleanTopicName.find("/", pos)) != std::string::npos)
      {
        cleanTopicName = cleanTopicName.replace(pos, 1, "-");
        pos ++;
      }

      // Construct the filename
      std::string home = std::getenv("HOME");
      std::string filename = home + "/.ros/bridge_logger-" +
        cleanTopicName + ".log";

      // Create the log stream.
      this->streams[info.name].open(filename.c_str());

      // Init the previous sequence map
      this->prevSeq[info.name] = -1;
      this->startTime[info.name] = std::chrono::steady_clock::now();

      // Subscribe to the topic.
      this->subscribers.push_back(
          this->n.subscribe(info.name, 1000, callback));
    }
  }
}

/////////////////////////////////////////////////
BridgeLogger::~BridgeLogger()
{
}

/////////////////////////////////////////////////
void BridgeLogger::OnSensorMsg(const topic_tools::ShapeShifter::ConstPtr &_msg,
                             const std::string &_topic)
{
  auto systemTime = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(this->mutex);
  ros::Time currentTime = ros::Time::now();

  uint32_t seq = 0;

  // Get the sequence numbers.
  if (_msg->getDataType() == "sensor_msgs/CompressedImage")
  {
    const auto msg = _msg->instantiate<sensor_msgs::CompressedImage>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "sensor_msgs/Image")
  {
    const auto msg = _msg->instantiate<sensor_msgs::Image>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "sensor_msgs/Imu")
  {
    const auto msg = _msg->instantiate<sensor_msgs::Imu>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "sensor_msgs/LaserScan")
  {
    const auto msg = _msg->instantiate<sensor_msgs::LaserScan>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "sensor_msgs/BatteryState")
  {
    const auto msg = _msg->instantiate<sensor_msgs::BatteryState>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "sensor_msgs/PointCloud2")
  {
    const auto msg = _msg->instantiate<sensor_msgs::PointCloud2>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "theora_image_transport/Packet")
  {
    const auto msg = _msg->instantiate<theora_image_transport::Packet>();
    seq = msg->header.seq;
  }
  else if (_msg->getDataType() == "sensor_msgs/MagneticField")
  {
    const auto msg = _msg->instantiate<sensor_msgs::MagneticField>();
    seq = msg->header.seq;
  }
  else
  {
    // Debug output.
    // ROS_ERROR("Data type[%s] not handled", _msg->getDataType().c_str());
    return;
  }

  // log msg time received in wall clock time
  this->topicTimestamp[_topic] = systemTime;

  // Check if a message was missed.
  if (this->prevSeq[_topic] + 1 != seq && this->prevSeq[_topic] >= 0)
    this->streams[_topic] << "***Error: Missed message(s) ***\n";

  std::chrono::duration<double> diff = systemTime - this->startTime[_topic];

  // Log the data.
  this->streams[_topic] << seq << " "
    << currentTime.sec + currentTime.nsec*1e-9 << " "
    << diff.count() << std::endl;

  this->prevSeq[_topic] = seq;
}

/////////////////////////////////////////////////
void BridgeLogger::CheckHeartbeat()
{
  // check heartbeat
  using namespace std::chrono_literals;
  double heartbeatCheckPeriod = 5.0;
  while (ros::ok)
  {
    std::this_thread::sleep_for(5s);
    auto systemTime = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      // check topic heartbeat
      for (auto &tIt : this->streams)
      {
        std::string topic = tIt.first;
        auto it = this->topicTimestamp.find(topic);
        if (it != this->topicTimestamp.end())
        {
          auto d = systemTime - it->second;
          auto seconds =
              std::chrono::duration_cast<std::chrono::milliseconds>(d).count() /
              1000.0;
          if (seconds > heartbeatCheckPeriod)
          {
            this->streams[topic] << "***Error: No messages received for more "
                                 << "than 5 seconds ***\n";
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  // Initialize ros
  ros::init(argc, argv, "bridge_logger");

  ROS_INFO("Starting BridgeLogger");

  // Create the logger
  BridgeLogger logger;

  // Spin asynchronously.
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
