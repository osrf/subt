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
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <subt_msgs/PoseFromArtifact.h>

#include <ignition/transport/Node.hh>

class SubtRosRelay
{
  /// \brief Constructor
  public: SubtRosRelay();

  /// \brief Destructor
  public: ~SubtRosRelay();

  /// \brief Ign callback for score topic and the data is republished to ROS
  /// \param[in] _msg Score msg
  public: void OnScore(const ignition::msgs::Float &_msg);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnStartCall(std_srvs::SetBool::Request &_req,
                           std_srvs::SetBool::Response &_res);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(std_srvs::SetBool::Request &_req,
                            std_srvs::SetBool::Response &_res);

  /// \brief ROS service callback triggered when the service is called.
  /// \param[in]  _req The message containing the robot name.
  /// \param[out] _res The response message.
  public: bool OnPoseFromArtifact(
               subt_msgs::PoseFromArtifact::Request &_req,
               subt_msgs::PoseFromArtifact::Response &_res);

  /// \brief Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief The ROS node handler used for communications.
  public: std::unique_ptr<ros::NodeHandle> rosnode;

  /// \brief ROS service to receive a call to finish the game.
  public: ros::ServiceServer finishService;

  /// \brief ROS service to receive a call to start the game.
  public: ros::ServiceServer startService;

  /// \brief ROS publisher to publish score data
  public: ros::Publisher rosScorePub;

  /// \brief ROS service server to receive the location of a robot relative to
  /// the origin artifact.
  public: ros::ServiceServer poseFromArtifactService;

  /// \brief A ROS asynchronous spinner.
  public: std::unique_ptr<ros::AsyncSpinner> spinner;
};

//////////////////////////////////////////////////
SubtRosRelay::SubtRosRelay()
{
  // Initialize the ROS node.
  this->rosnode.reset(new ros::NodeHandle("subt_relay"));

  // ROS service to receive a command to finish the game.
  ros::NodeHandle n;
  this->finishService = n.advertiseService(
      "/subt/finish", &SubtRosRelay::OnFinishCall, this);

  this->startService = n.advertiseService(
      "/subt/start", &SubtRosRelay::OnStartCall, this);

  // ROS service to request the robot pose relative to the origin artifact.
  // Note that this service is only available for robots in the staging area.
  this->poseFromArtifactService = n.advertiseService(
      "/subt/pose_from_artifact_origin",
      &SubtRosRelay::OnPoseFromArtifact, this);

  this->node.Subscribe("/subt/score", &SubtRosRelay::OnScore, this);

  this->rosScorePub =
    this->rosnode->advertise<std_msgs::Int32>("score", 1000);
}

//////////////////////////////////////////////////
SubtRosRelay::~SubtRosRelay()
{
}

/////////////////////////////////////////////////
void SubtRosRelay::OnScore(const ignition::msgs::Float &_msg)
{
  std_msgs::Int32 rosMsg;
  rosMsg.data = _msg.data();
  this->rosScorePub.publish(rosMsg);
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnFinishCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  ignition::msgs::Boolean req;
  ignition::msgs::Boolean rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.data);

  // Pass the request onto ignition transport
  this->node.Request("/subt/finish", req, timeout, rep, result);
  _res.success = rep.data();

  return result;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnStartCall(std_srvs::SetBool::Request &_req,
  std_srvs::SetBool::Response &_res)
{
  ignition::msgs::Boolean req;
  ignition::msgs::Boolean rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.data);

  // Pass the request onto ignition transport
  this->node.Request("/subt/start", req, timeout, rep, result);
  _res.success = rep.data();

  return result;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnPoseFromArtifact(
  subt_msgs::PoseFromArtifact::Request &_req,
  subt_msgs::PoseFromArtifact::Response &_res)
{
  ignition::msgs::StringMsg req;
  ignition::msgs::Pose rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.robot_name.data);

  // Pass the request onto ignition transport
  _res.success = this->node.Request("/subt/pose_from_artifact_origin",
      req, timeout, rep, result);

  // Construct the ROS response
  _res.pose.pose.position.x = rep.position().x();
  _res.pose.pose.position.y = rep.position().y();
  _res.pose.pose.position.z = rep.position().z();
  _res.pose.pose.orientation.x = rep.orientation().x();
  _res.pose.pose.orientation.y = rep.orientation().y();
  _res.pose.pose.orientation.z = rep.orientation().z();
  _res.pose.pose.orientation.w = rep.orientation().w();

  // Header.
  _res.pose.header.stamp = ros::Time(
    rep.header().stamp().sec(), rep.header().stamp().nsec());
  _res.pose.header.frame_id = rep.header().data(0).value(0);

  return result;
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "subt_ros_relay");

  SubtRosRelay relay;

  ros::spin();
  return 0;
}
