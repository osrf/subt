/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <cmath>
#include <sstream>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

using namespace gazebo;

// information to record for the lights in the enviornment
struct RecordInfo
{
  double duration;
  double interval;
  bool flashing;
  common::Time lastUpdate;
};

/////////////////////////////////////////////////
/// \brief A fixture class for testing the ROS flashlight plugin within Gazebo.
class FlashLightTest : public testing::Test
{
  // Constructor.
  public: FlashLightTest(): testing::Test()
  {
  }

  // Destructor.
  public: virtual ~FlashLightTest()
  {
  }

  // Callback for Responses about entity_info
  public: void ResponseCb(ConstResponsePtr &_msg);

  /// \brief Initializer.
  protected: void InitRec();

  /// \brief Send requests for entity information for a specified time.
  public: void SendRequests();

  /// \brief A function to check if we got the assumed results.
  /// \param[in] _updated Whether the light has been updated within its phase.
  /// \param[in] _duration Expected duration time.
  /// \param[in] _interval Expected interval time.
  /// \param[in] _maxErr Maximum error which can be accepted in seconds.
  protected: void CheckRec(
    const std::vector<bool> &_updated,
    const std::vector<double> &_duration,
    const std::vector<double> &_interval,
    const double &_maxErr);

  /// \brief Node for Gazebo transport.
  protected: gazebo::transport::NodePtr node;

  /// \brief Publisher to request for entity_info
  protected: transport::PublisherPtr entityInfoPub;

  /// \brief Subscriber to get entity_info.
  protected: transport::SubscriberPtr entityInfoSub;

  /// \brief ROS node handler.
  protected: ros::NodeHandle n;

  /// \brief ROS service client.
  protected: ros::ServiceClient client;

  /// \brief A list of records about actual duration/interval.
  private: struct RecordInfo flashLight[4];

  /// \brief True if a response is sent back.
  private: bool responded;

  /// \brief The time when the records were initialized.
  private: common::Time startTime;

  /// \brief Protect data from races.
  private: std::mutex mutexData;

  /// \brief A thread to send requests for entity_info
  protected: std::thread requester;

  /// \brief True if the thread should keep running.
  protected: bool running;

  /// \brief Protect the flag.
  protected: std::mutex mutexRunning;
};

/////////////////////////////////////////////////
void FlashLightTest::ResponseCb(ConstResponsePtr &_msg)
{
  // Get the current time.
  common::Time currentSimTime(ros::Time::now().toSec());

  // Extract necessary information from the response message.
  msgs::Model modelMsg;
  modelMsg.ParseFromString(_msg->serialized_data());

  int countLight = 0;
  for (int ilink = 0; ilink < modelMsg.link_size(); ++ilink)
  {
    msgs::Link linkMsg = modelMsg.link(ilink);
    for (int ilight = 0; ilight < linkMsg.light_size(); ++ilight)
    {
      msgs::Light lightMsg = linkMsg.light(ilight);

      // Update to flash
      {
        std::lock_guard<std::mutex> lk(this->mutexData);

        // dim -> flash
        if (!this->flashLight[countLight].flashing)
        {
          if (lightMsg.range() > 0)
          {
            this->flashLight[countLight].interval
              = currentSimTime.Double()
                - this->flashLight[countLight].lastUpdate.Double();
            // Update the last update time
            this->flashLight[countLight].lastUpdate = currentSimTime;
            // Update the current flashing state.
            this->flashLight[countLight].flashing = true;
          }
        }
        // flash -> dim
        else
        {
          if (lightMsg.range() == 0)
          {
            this->flashLight[countLight].duration
              = currentSimTime.Double()
                - this->flashLight[countLight].lastUpdate.Double();
            // Update the last update time
            this->flashLight[countLight].lastUpdate = currentSimTime;
            // Update the current flashing state.
            this->flashLight[countLight].flashing = false;
          }
        }
      }

      ++countLight;
    }
  }
  {
    std::lock_guard<std::mutex> lk(this->mutexData);
    this->responded = true;
  }
}

/////////////////////////////////////////////////
void FlashLightTest::InitRec()
{
  common::Time currentSimTime(ros::Time::now().toSec());
  {
    std::lock_guard<std::mutex> lk(this->mutexData);
    this->flashLight[0].duration = -1;
    this->flashLight[1].duration = -1;
    this->flashLight[2].duration = -1;
    this->flashLight[3].duration = -1;
    this->flashLight[0].interval = -1;
    this->flashLight[1].interval = -1;
    this->flashLight[2].interval = -1;
    this->flashLight[3].interval = -1;
    this->flashLight[0].flashing = false;
    this->flashLight[1].flashing = false;
    this->flashLight[2].flashing = false;
    this->flashLight[3].flashing = false;
    this->flashLight[0].lastUpdate = currentSimTime;
    this->flashLight[1].lastUpdate = currentSimTime;
    this->flashLight[2].lastUpdate = currentSimTime;
    this->flashLight[3].lastUpdate = currentSimTime;
    this->startTime = currentSimTime;
    this->responded = false;
  }
}

/////////////////////////////////////////////////
void FlashLightTest::SendRequests()
{
  double timeToSleep = 0.1;
  while(true)
  {
    msgs::Request msg;
    msg.set_id(0);
    msg.set_request("entity_info");
    msg.set_data("light_model");
    this->entityInfoPub->Publish(msg);
    ros::Duration(timeToSleep).sleep();

    std::lock_guard<std::mutex> lk(this->mutexRunning);
    if (!running)
    {
      break;
    }
  }
}

/////////////////////////////////////////////////
void FlashLightTest::CheckRec(
  const std::vector<bool> &_updated,
  const std::vector<double> &_duration,
  const std::vector<double> &_interval,
  const double &_maxErr)
{
  // Get necessary time information.
  common::Time endTime(ros::Time::now().toSec());
  double lastUp[4];
  std::lock_guard<std::mutex> lk(this->mutexData);
  ASSERT_TRUE(this->responded) << "The callback funciton was not called.";
  lastUp[0] = this->flashLight[0].lastUpdate.Double();
  lastUp[1] = this->flashLight[1].lastUpdate.Double();
  lastUp[2] = this->flashLight[2].lastUpdate.Double();
  lastUp[3] = this->flashLight[3].lastUpdate.Double();

  // Check records of each light
  // NOTE: Taking some errors caused by callback functions into consideration.
  // NOTE: If the interval is 0, the callback is not called.
  for (int i = 0; i < 4; ++i)
  {
    if (_updated[i] && _interval[i] > 0)
    {
      // The light is assumed to have been updated within its phase.
      EXPECT_LE(endTime.Double() - lastUp[i],
                std::max(_duration[i], _interval[i]) + _maxErr)
      << "flashLight[" << i << "] is supposed to be updated.";

      // The light has been flashing by the assumed duration and interval.
      EXPECT_NEAR(flashLight[i].duration, _duration[i], _maxErr)
      << "flashLight[" << i << "] has a wrong duration time.";
      EXPECT_NEAR(flashLight[i].interval, _interval[i], _maxErr)
      << "flashLight[" << i << "] has a wrong interval time.";
    }
    else
    {
      // The light is assumed to have never been updated from the beginning.
      EXPECT_LE(lastUp[i] - this->startTime.Double(), _maxErr)
      << "flashLight[" << i << "] is supposed not to be updated.";
    }
  }
}

/////////////////////////////////////////////////
TEST_F(FlashLightTest, switchOffAndOn)
{
  // ROS spinning
  std::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  // Initialize the transport node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->entityInfoPub
    = this->node->Advertise<msgs::Request>("~/request");
  // Subcribe for entity_info.
  this->entityInfoSub
    = this->node->Subscribe("~/response",
                            &FlashLightTest::ResponseCb,
                            dynamic_cast<FlashLightTest*>(this));

  // Get a ROS service client.
  this->client
    = this->n.serviceClient<std_srvs::SetBool>("/light_model/light_switch");
  ASSERT_TRUE(client.isValid());
  ASSERT_TRUE(client.waitForExistence());

  // Start the thread to repeatedly publishing a request.
  this->running = true;
  this->requester = std::thread(&FlashLightTest::SendRequests,
                                dynamic_cast<FlashLightTest*>(this));

  ros::Duration(0.5).sleep();

  // Initialize the records.
  this->InitRec();

  ros::Duration(2.0).sleep();

  // Check if the actual duration and interval are the supposed ones.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {true, true, false, false};
    std::vector<double> duration = {0.4, 0.2, 2.0, 1.0};
    std::vector<double> interval = {1.2, 0.2, 0.4, 0.0};
    this->CheckRec(updated, duration, interval, 0.25);
  }

  // Turn them off.
  {
    std_srvs::SetBool srv;
    srv.request.data = false;
    bool success = this->client.call(srv);
    EXPECT_TRUE(success);
  }

  // Allow some time for the plugin to stop blinking.
  // After this, the callback function is not supposed to be called any more.
  ros::Duration(0.2).sleep();

  // Initialize the records.
  this->InitRec();

  ros::Duration(2.0).sleep();

  // Check if all of them stopped to be updated.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {false, false, false, false};
    std::vector<double> duration = {-1, -1, -1, -1};
    std::vector<double> interval = {-1, -1, -1, -1};
    this->CheckRec(updated, duration, interval, 0.25);
  }

  // Turn them on.
  {
    std_srvs::SetBool srv;
    srv.request.data = true;
    bool success = this->client.call(srv);
    EXPECT_TRUE(success);
  }

  // Allow some time for the plugin to stop blinking.
  // After this, the callback function is not supposed to be called any more.
  ros::Duration(0.2).sleep();

  // Initialize the records.
  this->InitRec();

  ros::Duration(3.0).sleep();

  // Check if the duration and interval of lights were changed as expected.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {true, true, true, true};
    std::vector<double> duration = {0.4, 0.2, 2.0, 1.0};
    std::vector<double> interval = {1.2, 0.2, 0.4, 0.0};
    this->CheckRec(updated, duration, interval, 0.25);
  }

  {
    std::lock_guard<std::mutex> lk(this->mutexRunning);
    this->running = false;
  }
  this->requester.join();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // Start ROS
  ros::init(argc,argv,"test_flashlight");

  // Start Gazebo client
  gazebo::client::setup(argc, argv);

  return RUN_ALL_TESTS();
}
