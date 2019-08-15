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
#include <ros/ros.h>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>

#include <subt_msgs/SetPose.h>

#include <ignition/transport/Node.hh>

class SetPoseRelay
{
  /// \brief Constructor
  public: SetPoseRelay();

  /// \brief Destructor
  public: ~SetPoseRelay();

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnSetPoseCall(subt_msgs::SetPose::Request &_req,
                             subt_msgs::SetPose::Response &_res);

  /// \brief Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief The ROS node handler used for communications.
  public: ros::NodeHandle nh;

  /// \brief ROS service to receive a call to set robot pose.
  public: ros::ServiceServer setPoseService;

  /// \brief World name for set_pose topic name
  public: std::string worldName;
};

//////////////////////////////////////////////////
SetPoseRelay::SetPoseRelay()
{
  if (!this->nh.getParam("/world_name", this->worldName)) {
    ROS_ERROR("Cannot operate without world_name set");
    ros::shutdown();
    return;
  }

  this->setPoseService = this->nh.advertiseService(
      "/subt/set_pose", &SetPoseRelay::OnSetPoseCall, this);
}

//////////////////////////////////////////////////
SetPoseRelay::~SetPoseRelay()
{
}

/////////////////////////////////////////////////
bool SetPoseRelay::OnSetPoseCall(subt_msgs::SetPose::Request &_req,
  subt_msgs::SetPose::Response &_res)
{
  ignition::msgs::Pose req;
  ignition::msgs::Boolean rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_name(_req.performer);
  req.mutable_position()->set_x(_req.pose.position.x);
  req.mutable_position()->set_y(_req.pose.position.y);
  req.mutable_position()->set_z(_req.pose.position.z);
  req.mutable_orientation()->set_w(_req.pose.orientation.w);
  req.mutable_orientation()->set_x(_req.pose.orientation.x);
  req.mutable_orientation()->set_y(_req.pose.orientation.y);
  req.mutable_orientation()->set_z(_req.pose.orientation.z);

  // Pass the request onto ignition transport
  std::string topic = { "/world/" + this->worldName + "/set_pose" };
  this->node.Request(topic, req, timeout, rep, result);
  _res.success = rep.data();

  ROS_DEBUG_STREAM("Relaying set_pose: " << _req);

  return result;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "subt_set_pose_relay");

  SetPoseRelay relay;

  ros::spin();
  return 0;
}
