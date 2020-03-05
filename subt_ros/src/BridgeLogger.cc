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
#include <mutex>
#include <fstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <theora_image_transport/Packet.h>
#include <rosgraph_msgs/Clock.h>
#include <topic_tools/shape_shifter.h>


// The BridgeLogger logs sensor message sequence numbers and simulation time
// for SubT sensor topics.
class BridgeLogger
{
  /// \brief Constructor
  public: BridgeLogger();

  /// \brief Destructor
  public: ~BridgeLogger();

  /// \brief Callback for all sensor messages.
  /// \param[in] _msg The message.
  /// \param[in] _topic The name of the topic.
  private: void OnSensorMsg(const topic_tools::ShapeShifter::ConstPtr &_msg,
                            const std::string &_topic);

  /// \brief ROS node handler.
  private: ros::NodeHandle n;

  /// \brief All of the subscribers.
  private: std::vector<ros::Subscriber> subscribers;

  /// \brief Log streams.
  private: std::map<std::string, std::ofstream> streams;

  /// \brief Previous sequence numbers. This is used to detect errors.
  private: std::map<std::string, int32_t> prevSeq;

  /// \brief Just a mutex.
  private: std::mutex mutex;
};

/////////////////////////////////////////////////
BridgeLogger::BridgeLogger()
{
  // Wait for the clock.
  ros::topic::waitForMessage<rosgraph_msgs::Clock>("/clock", this->n);

  ros::master::V_TopicInfo masterTopics;
  ros::master::getTopics(masterTopics);

  // Subscribe to some of the sensor messages.
  for (ros::master::V_TopicInfo::iterator it = masterTopics.begin();
      it != masterTopics.end(); ++it)
  {
    const ros::master::TopicInfo &info = *it;

    if (info.name.find("parameter_descriptions") != std::string::npos ||
        info.name.find("parameter_updates") != std::string::npos ||
        info.name.find("compressedDepth") != std::string::npos)
      continue;

    if (info.name.find("front_scan") != std::string::npos ||
        info.name.find("points") != std::string::npos ||
        info.name.find("image_raw") != std::string::npos ||
        info.name.find("depth") != std::string::npos ||
        info.name.find("imu") != std::string::npos ||
        info.name.find("magnetic_field") != std::string::npos ||
        info.name.find("air_pressure") != std::string::npos ||
        info.name.find("battery_state") != std::string::npos ||
        info.name.find("imu") != std::string::npos)
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
  else
  {
    // Debug output.
    ROS_ERROR("Data type[%s] not handled", _msg->getDataType().c_str());
    return;
  }

  // Check if a message was missed.
  if (this->prevSeq[_topic] + 1 != seq && this->prevSeq[_topic] >= 0)
    this->streams[_topic] << "***Error: Missed message(s) ***\n";

  // Log the data.
  this->prevSeq[_topic] = seq;
  this->streams[_topic] << seq << " "
    << currentTime.sec + currentTime.nsec*1e-9 << std::endl;
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
