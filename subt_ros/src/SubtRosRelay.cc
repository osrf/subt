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
#include <boost/lockfree/spsc_queue.hpp>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Time.h>
#include <ignition/common/Util.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <subt_msgs/Bind.h>
#include <subt_msgs/DatagramRos.h>
#include <subt_msgs/PoseFromArtifact.h>
#include <subt_msgs/Register.h>
#include <subt_msgs/Unregister.h>
#include <subt_communication_broker/protobuf/datagram.pb.h>
#include <subt_communication_broker/common_types.h>
#include <subt_ros/CompetitionClock.h>
#include <rosbag/recorder.h>

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

  /// \brief Ign callback for competition clock and the data is republished
  /// to ROS
  /// \param[in] _msg Clock msg
  public: void OnCompetitionClock(const ignition::msgs::Clock &_msg);

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

  /// \brief ROS service callback triggered when a comms client binds to an
  /// address.
  /// \param[in] _req The requested address and endpoint.
  /// \param[out] _res True if the bind was successful, false otherwise.
  public: bool OnBind(subt_msgs::Bind::Request &_req,
                      subt_msgs::Bind::Response &_res);

  /// \brief ROS service callback triggerered when a comms client sends a
  /// message to an address.
  /// \param[in] _req The requested message to send.
  /// \param[out] _res The response.
  public: bool OnSendTo(subt_msgs::DatagramRos::Request &_req,
                        subt_msgs::DatagramRos::Response &_res);

  /// \brief ROS service callback triggerered when a comms client is
  /// registered.
  /// \param[in] _req The request.
  /// \param[out] _res The response.
  public: bool OnRegister(subt_msgs::Register::Request &_req,
                          subt_msgs::Register::Response &_res);

  /// \brief ROS service callback triggerered when a comms client is
  /// unregistered.
  /// \param[in] _req The request.
  /// \param[out] _res The response.
  public: bool OnUnregister(subt_msgs::Unregister::Request &_req,
                            subt_msgs::Unregister::Response &_res);

  /// \brief Ignition service callback triggerered when a message is received.
  /// The received message is added to a message queue to be handled by a
  /// separate thread.
  /// \param[in] _msg The message.
  public: void OnMessage(const subt::msgs::Datagram &_msg);

  /// \brief Process messages in consumed from the message queue
  /// The message is forwarded via a ROS service call.
  /// \param[in] _req The message.
  public: void ProcessMessage(const subt::msgs::Datagram &_req);

  /// \brief Creates an AsyncSpinner and handles received messages
  public: void Spin();

  /// \brief Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief The ROS node handler used for communications.
  public: std::unique_ptr<ros::NodeHandle> rosnode;

  /// \brief ROS service to receive a call to finish the game.
  public: ros::ServiceServer finishService;

  /// \brief ROS service to receive a call to start the game.
  public: ros::ServiceServer startService;

  /// \brief ROS service to receive a bind request.
  public: ros::ServiceServer commsModelBindService;

  /// \brief ROS service to receive a send to request.
  public: ros::ServiceServer commsModelSendToService;

  /// \brief ROS service to receive a register request.
  public: ros::ServiceServer commsModelRegisterService;

  /// \brief ROS service to receive a unregister request.
  public: ros::ServiceServer commsModelUnregisterService;

  /// \brief ROS publisher to publish score data
  public: ros::Publisher rosScorePub;

  /// \brief ROS publisher for competition clock data.
  public: ros::Publisher rosCompetitionClockPub;

  /// \brief ROS service server to receive the location of a robot relative to
  /// the origin artifact.
  public: ros::ServiceServer poseFromArtifactService;

  /// \brief The set of bound address. This is bookkeeping that helps
  /// to reduce erroneous error output in the ::Bind function.
  public: std::set<std::string> boundAddresses;

  /// \brief Lock free queue for holding msgs from Transport. This is needed to
  /// avoid deadlocks between the Transport thread that invokes callbacks and
  /// the main thread that handles messages.
  public: boost::lockfree::spsc_queue<subt::msgs::Datagram> msgQueue{10};

  /// \brief This mutex is used in conjunction with notifyCond to notify the
  /// main thread the arrival of new messages.
  public: std::mutex notifyMutex;

  /// \brief Condition variable for notifying arrival of new messages.
  public: std::condition_variable notifyCond;

  /// \brief Name of the robot this relay is associated with.
  public: std::vector<std::string> robotNames;

  /// \brief Thread on which the ROS bag recorder runs.
  public: std::unique_ptr<std::thread> bagThread = nullptr;

  /// \brief Pointer to the ROS bag recorder.
  public: std::unique_ptr<rosbag::Recorder> rosRecorder;
};

//////////////////////////////////////////////////
SubtRosRelay::SubtRosRelay()
{
  // Initialize the ROS node.
  this->rosnode.reset(new ros::NodeHandle("subt"));

  ros::NodeHandle n;

  std::string stringNames;
  if (!n.getParam("/robot_names", stringNames)) {
    ROS_ERROR("Cannot operate without robot_names set");
    ros::shutdown();
    return;
  }
  this->robotNames = ignition::common::split(stringNames, ",");

  // ROS service to receive a command to finish the game.
  this->finishService = n.advertiseService(
      "/subt/finish", &SubtRosRelay::OnFinishCall, this);

  this->startService = n.advertiseService(
      "/subt/start", &SubtRosRelay::OnStartCall, this);

  // ROS service to request the robot pose relative to the origin artifact.
  // Note that this service is only available for robots in the staging area.
  this->poseFromArtifactService = n.advertiseService(
      "/subt/pose_from_artifact_origin",
      &SubtRosRelay::OnPoseFromArtifact, this);

  this->commsModelBindService = n.advertiseService(
      subt::communication_broker::kEndPointRegistrationSrv,
      &SubtRosRelay::OnBind, this);

  this->commsModelSendToService = n.advertiseService(
      subt::communication_broker::kBrokerSrv,
      &SubtRosRelay::OnSendTo, this);

  this->commsModelRegisterService = n.advertiseService(
      subt::communication_broker::kAddrRegistrationSrv,
      &SubtRosRelay::OnRegister, this);

  this->commsModelUnregisterService = n.advertiseService(
      subt::communication_broker::kAddrUnregistrationSrv,
      &SubtRosRelay::OnUnregister, this);

  this->node.Subscribe("/subt/score", &SubtRosRelay::OnScore, this);

  this->node.Subscribe("/subt/run_clock",
      &SubtRosRelay::OnCompetitionClock, this);

  this->rosScorePub =
    this->rosnode->advertise<std_msgs::Int32>("score", 1000);
  this->rosCompetitionClockPub =
    this->rosnode->advertise<subt_ros::CompetitionClock>("run_clock", 1000);

  // Setup a ros bag recorder.
  rosbag::RecorderOptions recorderOptions;
  recorderOptions.append_date=false;
  recorderOptions.split=true;
  recorderOptions.max_splits=0;

  // This equation is sourced from line 133 in
  // http://docs.ros.org/en/noetic/api/rosbag/html/c++/record_8cpp_source.html
  recorderOptions.max_size=1048576 * 1000;

  // Prefix the rosbag file(s) name with a prefix composed of robot_data, the
  // IGN_PARTITION (if set), and the list of robots that generate this data.
  // This allows running multiple simulations of multiple robots even on
  // local catkin workspace.
  std::string part;
  ignition::common::env("IGN_PARTITION", part);
  recorderOptions.prefix="robot_data";
  if (!part.empty())
    recorderOptions.prefix += "_" + part;
  recorderOptions.prefix += "_" + stringNames;

  recorderOptions.regex=true;
  recorderOptions.topics.push_back("/robot_data(.*)");

  // Spawn thread for recording /subt/ data to rosbag.
  this->rosRecorder.reset(new rosbag::Recorder(recorderOptions));
  this->bagThread.reset(new std::thread([&](){
        this->rosRecorder->run();
  }));
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
void SubtRosRelay::OnCompetitionClock(const ignition::msgs::Clock &_msg)
{
  subt_ros::CompetitionClock clockMsg;
  if (_msg.has_header() && _msg.header().data_size() > 0)
  {
    for (int i = 0; i < _msg.header().data_size(); ++i)
    {
      if (_msg.header().data(i).key() == "phase" &&
          _msg.header().data(i).value_size() > 0)
      {
        clockMsg.phase = _msg.header().data(i).value(0);
      }
    }
  }
  clockMsg.data.sec = _msg.sim().sec();
  clockMsg.data.nsec = _msg.sim().nsec();
  this->rosCompetitionClockPub.publish(clockMsg);

  // Shutdown when the phase == finished. This makes sure that the rosbag
  // ends cleanly.
  if (clockMsg.phase == "finished" &&
      this->bagThread && this->bagThread->joinable())
  {
    // Shutdown ros. this makes the ROS bag recorder stop.
    ros::shutdown();
    this->bagThread->join();
  }
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
  if (std::find(this->robotNames.begin(), this->robotNames.end(),
                _req.robot_name.data) == this->robotNames.end())
  {
    ROS_ERROR_STREAM("OnPoseFromArtifact address does not match origination."
        << "Attempted impersonation of a robot as robot["
        <<_req.robot_name.data << "].\n");
    return false;
  }

  ignition::msgs::StringMsg req;
  ignition::msgs::Pose rep;
  unsigned int timeout = 5000;
  bool result;

  req.set_data(_req.robot_name.data);

  // Pass the request onto ignition transport
  _res.success = this->node.Request("/subt/pose_from_artifact_origin",
      req, timeout, rep, result);

  // Request failed, ignore response
  if (!result)
    return result;

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

/////////////////////////////////////////////////
bool SubtRosRelay::OnBind(subt_msgs::Bind::Request &_req,
                          subt_msgs::Bind::Response &_res)
{
  if (std::find(this->robotNames.begin(), this->robotNames.end(),
                _req.address) ==this->robotNames.end())
  {
    ROS_ERROR_STREAM("OnBind address does not match origination. Attempted "
        << "impersonation of a robot as robot[" << _req.address << "].\n");
      return false;
  }

  ignition::msgs::StringMsg_V req;
  req.add_data(_req.address);
  req.add_data(_req.endpoint);

  const unsigned int timeout = 3000u;
  ignition::msgs::Boolean rep;
  bool result;
  bool executed = this->node.Request(
      subt::communication_broker::kEndPointRegistrationSrv,
      req, timeout, rep, result);

  _res.success = result;

  if (executed && result &&
      // Only establish the Ignition service once per client.
      this->boundAddresses.find(_req.address) == this->boundAddresses.end())
  {
    if (!this->node.Advertise(_req.address, &SubtRosRelay::OnMessage, this))
    {
      std::cerr << "Bind Error: could not advertise "
        << _req.address << std::endl;
      return false;
    }
    else
    {
      this->boundAddresses.insert(_req.address);
    }
  }

  return executed;
}

/////////////////////////////////////////////////
bool SubtRosRelay::OnSendTo(subt_msgs::DatagramRos::Request &_req,
                            subt_msgs::DatagramRos::Response &_res)
{
  if (std::find(this->robotNames.begin(), this->robotNames.end(),
                _req.src_address) == this->robotNames.end())
  {
    ROS_ERROR_STREAM("OnSendTo address does not match origination. Attempted "
        << "impersonation of a robot as robot[" <<_req.src_address << "].\n");
    return false;
  }

  subt::msgs::Datagram msg;
  msg.set_src_address(_req.src_address);
  msg.set_dst_address(_req.dst_address);
  msg.set_dst_port(_req.dst_port);
  msg.set_rssi(_req.rssi);
  msg.set_data(_req.data);

  return this->node.Request(subt::communication_broker::kBrokerSrv, msg);
}

//////////////////////////////////////////////////
bool SubtRosRelay::OnRegister(subt_msgs::Register::Request &_req,
                              subt_msgs::Register::Response &_res)
{
  ignition::msgs::StringMsg req;
  req.set_data(_req.local_address);

  ignition::msgs::Boolean rep;
  bool result;
  const unsigned int timeout = 3000u;

  bool executed = this->node.Request(
    subt::communication_broker::kAddrRegistrationSrv,
    req, timeout, rep, result);

  _res.success = result;
  return executed;
}

//////////////////////////////////////////////////
bool SubtRosRelay::OnUnregister(subt_msgs::Unregister::Request &_req,
                                subt_msgs::Unregister::Response &_res)
{
  ignition::msgs::StringMsg req;
  req.set_data(_req.local_address);

  ignition::msgs::Boolean rep;
  bool result;
  const unsigned int timeout = 3000u;

  bool executed = this->node.Request(
    subt::communication_broker::kAddrUnregistrationSrv,
    req, timeout, rep, result);

  _res.success = result;

  return executed;
}

//////////////////////////////////////////////////
void SubtRosRelay::OnMessage(const subt::msgs::Datagram &_req)
{
  this->msgQueue.push(_req);
  // Notify the main thread
  this->notifyCond.notify_one();
}

//////////////////////////////////////////////////
void SubtRosRelay::ProcessMessage(const subt::msgs::Datagram &_req)
{
  subt_msgs::DatagramRos::Request req;
  subt_msgs::DatagramRos::Response res;
  req.src_address = _req.src_address();
  req.dst_address = _req.dst_address();
  req.dst_port = _req.dst_port();
  req.data = _req.data();
  req.rssi = _req.rssi();

  if (_req.dst_address() == subt::communication_broker::kBroadcast)
  {
    for (const std::string &dest : this->robotNames)
    {
      ros::service::call(dest, req, res);
    }
  }
  else
  {
    ros::service::call(_req.dst_address(), req, res);
  }
}

//////////////////////////////////////////////////
void SubtRosRelay::Spin()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok())
  {
    {
      // The code in this scope is only used for receiving notifications from
      // the `OnMessage` function. The mutex is not actually used for protecting
      // `msgQueue` since that is a lock free data structure. It is important
      // that the lock is released before calling `ProcessMessage` to avoid
      // deadlocks.
      std::unique_lock<std::mutex> lock(this->notifyMutex);
      this->notifyCond.wait(lock,
          [this] { return this->msgQueue.read_available(); });
    }

    // Process each message in FIFO order.
    this->msgQueue.consume_all(
        boost::bind(&SubtRosRelay::ProcessMessage, this, _1));
  }
}

//////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "subt_ros_relay");

  SubtRosRelay relay;
  relay.Spin();

  return 0;
}
