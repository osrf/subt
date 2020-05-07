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

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include <ros/ros.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>


/// \brief Optical frame publisher interface
class OpticalFramePublisherInterface
{
  /// \brief Initialize the optical frame publisher
  /// \param[in] _topic Topic to subscribe to
  public: virtual void Init(const std::string &_topic) = 0;
};

/// \brief A class that converts data from robot frame to optical frame
/// It subscribes to existing topic, modifies the frame_id of the message to
/// a new optical frame, then republishes the updated data to a new topic.
template <class T>
class OpticalFramePublisher
  : public OpticalFramePublisherInterface
{
  // Documentation inherited
  public: void Init(const std::string &_topic) override;

  /// \brief Subscriber callback which updates the frame id of the message
  /// and republishes the message.
  /// \param[in] _msg Message whose frame id is to be updated
  private: void UpdateFrame(const typename T::Ptr &_msg);

  /// \brief Publish static tf data between the original frame of the msg
  /// and the new optical frame
  /// \param[in] _frame Original message frame id
  /// \param[in] _childFrame Optical frame id
  private: void PublishTF(const std::string &_frame,
    const std::string &_childFrame);

  /// \brief ROS node handle
  private: ros::NodeHandle node;

  /// \brief ROS subscriber that subscribes to original topic
  private: ros::Subscriber sub;

  /// \brief ROS publisher that publishes data with the new optical frame
  private: ros::Publisher pub;

  /// \brief Optical frame id
  private: std::string newFrameId;
};

//////////////////////////////////////////////////
template <class T>
void OpticalFramePublisher<T>::Init(const std::string &_topic)
{
  size_t idx = _topic.rfind("/");
  std::string prefix;
  std::string suffix;
  if (idx == std::string::npos)
  {
    suffix = _topic;
  }
  else
  {
    prefix = _topic.substr(0, idx);
    suffix = _topic.substr(idx+1);
  }

  std::string outTopic = prefix;
  if (!prefix.empty())
    outTopic += "/";
  outTopic += "optical/" + suffix;

  this->sub = this->node.subscribe(_topic, 10,
      &OpticalFramePublisher::UpdateFrame, this);
  this->pub = this->node.advertise<T>(outTopic, 10);
  ROS_INFO("Optical Frame Publisher Ready");
}

//////////////////////////////////////////////////
template<class T>
void OpticalFramePublisher<T>::UpdateFrame(const typename T::Ptr &_msg)
{
  if (this->newFrameId.empty())
  {
    this->newFrameId = _msg->header.frame_id + "_optical";
    this->PublishTF(_msg->header.frame_id, this->newFrameId);
  }

  _msg->header.frame_id = this->newFrameId;
  this->pub.publish(_msg);
}

//////////////////////////////////////////////////
template<class T>
void OpticalFramePublisher<T>::PublishTF(const std::string &_frame,
    const std::string &_childFrame)
{
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.frame_id = _frame;
  tfStamped.child_frame_id = _childFrame;
  tfStamped.transform.translation.x = 0.0;
  tfStamped.transform.translation.y = 0.0;
  tfStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  // converts x forward to z forward
  q.setRPY(M_PI/2.0, 0, M_PI/2.0);
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();

  static tf2_ros::StaticTransformBroadcaster brStatic;
  brStatic.sendTransform(tfStamped);
}
