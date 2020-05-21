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

#include <iostream>

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

/// \brief A class that converts data from robot frame to optical frame
/// It subscribes to existing topic, modifies the frame_id of the message to
/// a new optical frame, then republishes the updated data to a new topic.
class OpticalFramePublisher
{
  // Documentation inherited
  public: void Init(bool _cameraInfo = true);

  /// \brief Subscriber callback which updates the frame id of the message
  /// and republishes the message.
  /// \param[in] _msg Message whose frame id is to be updated
  private: void UpdateImageFrame(const sensor_msgs::Image::Ptr &_msg);

  /// \brief Subscriber callback which updates the frame id of the message
  /// and republishes the message.
  /// \param[in] _msg Message whose frame id is to be updated
  private: void UpdateCameraInfoFrame(const sensor_msgs::CameraInfo::Ptr &_msg);

  /// \brief Publish static tf data between the original frame of the msg
  /// and the new optical frame
  /// \param[in] _frame Original message frame id
  /// \param[in] _childFrame Optical frame id
  private: void PublishTF(const std::string &_frame,
    const std::string &_childFrame);

  /// \brief Callback when subscriber connects to the image topic
  private: void ImageConnect();

  /// \brief Callback when subscriber connects to the camera info topic
  private: void CameraInfoConnect();

  /// \brief ROS node handle
  private: ros::NodeHandle node;

  /// \brief ROS subscriber that subscribes to original image topic
  private: std::unique_ptr<ros::Subscriber> sub;

  /// \brief ROS subscriber that subscribes to original camera info topic
  private: std::unique_ptr<ros::Subscriber> ciSub;

  /// \brief ROS publisher that publishes image msg with the new optical frame
  private: ros::Publisher pub;

  /// \brief ROS publisher that publishes camera info msg with the new optical
  /// frame
  private: ros::Publisher ciPub;

  /// \brief Optical frame id
  private: std::string newFrameId;
};

//////////////////////////////////////////////////
void OpticalFramePublisher::Init(bool _cameraInfo)
{
  this->pub = this->node.advertise<sensor_msgs::Image>("output/image", 10,
      std::bind(&OpticalFramePublisher::ImageConnect, this));

  if (_cameraInfo)
  {
    this->ciPub = this->node.advertise<sensor_msgs::CameraInfo>(
        "output/camera_info", 10,
        std::bind(&OpticalFramePublisher::CameraInfoConnect, this));
  }

  ROS_INFO("Optical Frame Publisher Ready");
}

//////////////////////////////////////////////////
void OpticalFramePublisher::UpdateImageFrame(
    const sensor_msgs::Image::Ptr &_msg)
{
  if (this->pub.getNumSubscribers() == 0u && this->sub)
  {
    this->sub.reset();
    return;
  }

  if (this->newFrameId.empty())
  {
    this->newFrameId = _msg->header.frame_id + "_optical";
    this->PublishTF(_msg->header.frame_id, this->newFrameId);
  }

  _msg->header.frame_id = this->newFrameId;
  this->pub.publish(_msg);
}

//////////////////////////////////////////////////
void OpticalFramePublisher::UpdateCameraInfoFrame(
    const sensor_msgs::CameraInfo::Ptr &_msg)
{
  if (this->ciPub.getNumSubscribers() == 0u && this->ciSub)
  {
    this->ciSub.reset();
    return;
  }

  if (this->newFrameId.empty())
  {
    this->newFrameId = _msg->header.frame_id + "_optical";
    this->PublishTF(_msg->header.frame_id, this->newFrameId);
  }

  _msg->header.frame_id = this->newFrameId;
  this->ciPub.publish(_msg);
}

//////////////////////////////////////////////////
void OpticalFramePublisher::PublishTF(const std::string &_frame,
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
  q.setRPY(-M_PI/2.0, 0, -M_PI/2.0);
  tfStamped.transform.rotation.x = q.x();
  tfStamped.transform.rotation.y = q.y();
  tfStamped.transform.rotation.z = q.z();
  tfStamped.transform.rotation.w = q.w();

  static tf2_ros::StaticTransformBroadcaster brStatic;
  brStatic.sendTransform(tfStamped);
}

//////////////////////////////////////////////////
void OpticalFramePublisher::ImageConnect()
{
  if (this->sub)
    return;
  this->sub = std::make_unique<ros::Subscriber>(
      this->node.subscribe("input/image", 10,
      &OpticalFramePublisher::UpdateImageFrame, this));
}

//////////////////////////////////////////////////
void OpticalFramePublisher::CameraInfoConnect()
{
  if (this->ciSub)
    return;
  this->ciSub = std::make_unique<ros::Subscriber>(
      this->node.subscribe("input/camera_info", 10,
      &OpticalFramePublisher::UpdateCameraInfoFrame, this));
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  std::string type;
  if (_argc == 2)
  {
    type = _argv[2];
  }

  ros::init(_argc, _argv, "optical_frame_publisher");

  OpticalFramePublisher pub;
  if (type == "depth")
    pub.Init(false);
  else
    pub.Init();

  ros::spin();

  return 0;
}
