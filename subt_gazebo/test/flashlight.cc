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

#include <string>
#include <sstream>
#include <cmath>
#include <vector>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include <gtest/gtest.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

#include "test/test_config.h"

using namespace gazebo;
using namespace subt;

// information to record for the lights in the enviornment
struct RecordInfo
{
  double duration;
  double interval;
  common::Time lastUpdate;
};

/////////////////////////////////////////////////
/// \brief A fixture class for testing the ROS flashlight plugin within Gazebo.
class FlashLightTest : public testing::Test
{
  // Constructor.
  public: FlashLightTest(): testing::Test(){}

  // Destructor.
  public: ~FlashLightTest()
  {
    if (node != nullptr)
    {
      node->Fini();
    }
  }

  // Callback for light/modify topic
  public: void lightCb(ConstLightPtr &_msg);

  /// \brief Initializer.
  protected: void InitRec();

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

  /// \brief Gazebo transport subscriber.
  protected: transport::SubscriberPtr sceneSub;

  /// \brief ROS node handler.
  protected: ros::NodeHandle n;

  /// \brief ROS service client.
  protected: ros::ServiceClient client;

  /// \brief A list of records about actual duration/interval.
  private: struct RecordInfo flashLight[4];

  /// \brief A flag indicating whether the callback was called.
  private: bool called;

  /// \brief The time when the records were initialized.
  private: common::Time startTime;

  /// \brief Protect data from races.
  private: std::mutex mutexData;
};

/////////////////////////////////////////////////

void FlashLightTest::lightCb(ConstLightPtr &_msg)
{
  // Determine which light is to be updated
  int indx;
  std::string name = _msg->name();
  std::stringstream ss;
  ss << name.substr(name.length() - 1);
  ss >> indx;
  indx--;

  bool correctIndex = true;
  if (indx < 0 || 3 < indx)
  {
    correctIndex = false;
  }
  ASSERT_TRUE(correctIndex);

  // Get the current time
  common::Time currentSimTime(ros::Time::now().toSec());

  // Update to flash
  {
    std::lock_guard<std::mutex> lk(this->mutexData);
    if (_msg->range() > 0)
    {
      this->flashLight[indx].interval
        = currentSimTime.Double() - this->flashLight[indx].lastUpdate.Double();
    }
    // Update to dim
    else
    {
      this->flashLight[indx].duration
        = currentSimTime.Double() - this->flashLight[indx].lastUpdate.Double();
    }

    // Update the last update time
    this->flashLight[indx].lastUpdate = currentSimTime;

    this->called = true;
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
    this->flashLight[0].lastUpdate = currentSimTime;
    this->flashLight[1].lastUpdate = currentSimTime;
    this->flashLight[2].lastUpdate = currentSimTime;
    this->flashLight[3].lastUpdate = currentSimTime;
    this->startTime = currentSimTime;
    this->called = false;
  }
}

/////////////////////////////////////////////////
void FlashLightTest::CheckRec(
  const std::vector<bool> &_updated,
  const std::vector<double> &_duration,
  const std::vector<double> &_interval,
  const double &_maxErr)
{
  // Determine if the callback function must have been called.
  // If at least one of the lights is to be updated, it is supposed to have
  // been called.
  bool someCalled = false;
  for (int i = 0; i < 4; ++i)
  {
    if (_updated[i])
    {
      someCalled = true;
    }
  }

  // Get necessary time information.
  common::Time endTime(ros::Time::now().toSec());
  double lastUp[4];
  std::lock_guard<std::mutex> lk(this->mutexData);
  lastUp[0] = this->flashLight[0].lastUpdate.Double();
  lastUp[1] = this->flashLight[1].lastUpdate.Double();
  lastUp[2] = this->flashLight[2].lastUpdate.Double();
  lastUp[3] = this->flashLight[3].lastUpdate.Double();

  if (someCalled)
  {
    // The callback function is assumed to have been called.
    ASSERT_TRUE(this->called);
  }
  else
  {
    // the callback function is assumed to have never been called.
    ASSERT_FALSE(this->called);
  }

  // Check records of each light
  // NOTE: Taking some errors caused by callback functions into consideration.
  // NOTE: If the interval is 0, the callback is not called.
  for (int i = 0; i < 4; ++i)
  {
    if (_updated[i] && _interval[i] > 0)
    {
      // The light is assumed to have been updated within its phase.
      EXPECT_LE(endTime.Double() - lastUp[i],
                std::max(_duration[i], _interval[i]) + _maxErr);

      // The light has been flashing by the assumed duration and interval.
      EXPECT_NEAR(flashLight[i].duration, _duration[i], _maxErr);
      EXPECT_NEAR(flashLight[i].interval, _interval[i], _maxErr);
    }
    else
    {
      // The light is assumed to have never been updated from the beginning.
      EXPECT_LE(lastUp[i] - this->startTime.Double(), _maxErr);
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

  //ros::Duration(5.0).sleep();

  // Initialize the transport node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  // Subscribe to plugin notifications.
  this->sceneSub
    = this->node->Subscribe("~/light/modify",
                            &FlashLightTest::lightCb,
                            dynamic_cast<FlashLightTest*>(this));


  // Get a ROS service client.
  this->client
    = this->n.serviceClient<std_srvs::SetBool>("/light_switch");
  ASSERT_TRUE(client.isValid());
  ASSERT_TRUE(client.waitForExistence());

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval.
  ros::Duration(1.5).sleep();

  // Check if the actual duration and interval are the supposed ones.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {true, true, false, false};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(updated, duration, interval, 0.01);
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
  ros::Duration(0.1).sleep();

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval.
  ros::Duration(0.5).sleep();

  // Check if all of them stopped to be updated.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {false, false, false, false};
    std::vector<double> duration = {-1, -1, -1, -1};
    std::vector<double> interval = {-1, -1, -1, -1};
    this->CheckRec(updated, duration, interval, 0.01);
  }

  // Turn them on.
  {
    std_srvs::SetBool srv;
    srv.request.data = true;
    bool success = this->client.call(srv);
    EXPECT_TRUE(success);
  }

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval.
  ros::Duration(2.0).sleep();

  // Check if the duration and interval of lights were changed as expected.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {true, true, true, true};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(updated, duration, interval, 0.01);
  }

  node->Fini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{

  initGazeboEnv();

  ::testing::InitGoogleTest(&argc, argv);

  // Start ROS
  ros::init(argc,argv,"test_flashlight");

  // Start Gazebo client
  gazebo::client::setup(argc, argv);

  return RUN_ALL_TESTS();
}
