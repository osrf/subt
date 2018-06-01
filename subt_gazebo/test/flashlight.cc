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
#include <algorithm>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include <gtest/gtest.h>

#include <gazebo/test/ServerFixture.hh>

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
/// \brief A fixture class for testing the LightControl plugin within Gazebo
/// and clients using the LightControlClient library.
class FlashLightTest : public ServerFixture
{
  /// \brief Constructor.
  public: FlashLightTest()
  {
  }

  /// \brief Initializer.
  protected: void InitRec()
  {
    std::lock_guard<std::mutex> lk(this->mutex);
    common::Time currentTime = physics::get_world()->SimTime();
    this->flashLight[0].duration = -1;
    this->flashLight[1].duration = -1;
    this->flashLight[2].duration = -1;
    this->flashLight[3].duration = -1;
    this->flashLight[0].interval = -1;
    this->flashLight[1].interval = -1;
    this->flashLight[2].interval = -1;
    this->flashLight[3].interval = -1;
    this->flashLight[0].lastUpdate = currentTime;
    this->flashLight[1].lastUpdate = currentTime;
    this->flashLight[2].lastUpdate = currentTime;
    this->flashLight[3].lastUpdate = currentTime;
    this->called = false;
    this->startTime = currentTime;
  }

  // Callback for light/modify topic
  public: void lightCb(ConstLightPtr &_msg)
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
    EXPECT_TRUE(correctIndex);

    physics::WorldPtr world = physics::get_world();

    // Do the following only if the world is available
    if (world != NULL)
    {
      // Get the current time

      common::Time currentTime = world->SimTime();

      // Update to flash
      std::lock_guard<std::mutex> lk(this->mutex);
      if (_msg->range() > 0)
      {
        this->flashLight[indx].interval
          = currentTime.Double() - this->flashLight[indx].lastUpdate.Double();
      }
      // Update to dim
      else
      {
        this->flashLight[indx].duration
          = currentTime.Double() - this->flashLight[indx].lastUpdate.Double();
      }

      // Update the last update time
      this->flashLight[indx].lastUpdate = currentTime;
    }

    this->called = true;
  }

  /// \brief A function to check if we got the assumed results.
  /// \param[in] _updated Whether the light has been updated within its phase.
  /// \param[in] _duration Expected duration time.
  /// \param[in] _interval Expected interval time.
  /// \param[in] _maxErr Maximum error which can be accepted in seconds.
  protected: void CheckRec(
    const std::vector<bool> &_updated,
    const std::vector<double> &_duration,
    const std::vector<double> &_interval,
    const double &_maxErr)
  {
    // Determine if the callback function must have been called.
    // If at least one of the lights is to be updated, it is supposed to have
    // been called.
    bool called = false;
    for (int i = 0; i < 4; ++i)
    {
      if (_updated[i])
      {
        called = true;
      }
    }

    // Get necessary time information.
    std::lock_guard<std::mutex> lk(this->mutex);
    common::Time &startTime = this->startTime;
    common::Time endTime = physics::get_world()->SimTime();
    double lastUp[4];
    lastUp[0] = this->flashLight[0].lastUpdate.Double();
    lastUp[1] = this->flashLight[1].lastUpdate.Double();
    lastUp[2] = this->flashLight[2].lastUpdate.Double();
    lastUp[3] = this->flashLight[3].lastUpdate.Double();

    if (called)
    {
      // The callback function is assumed to have been called.
      EXPECT_TRUE(this->called);
    }
    else
    {
      // the callback function is assumed to have never been called.
      EXPECT_FALSE(this->called);
    }

    // Check records of each light
    // NOTE: Taking some errors caused by callback functions into consideration.
    // NOTE: If the interval is 0, the callback is not called.
    for (int i = 0; i < 4; ++i)
    {
      gzmsg << "checking light [" << i << "]" << std::endl;
      if (_updated[i] && _interval[i] > 0)
      {
        // The light is assumed to have been updated within its phase.
        EXPECT_LE(endTime.Double() - lastUp[i],
                  std::max(_duration[i], _interval[i]) + _maxErr);

        // The light has been flashing by the assumed duration and interval.
        EXPECT_NEAR(this->flashLight[i].duration, _duration[i], _maxErr);
        EXPECT_NEAR(this->flashLight[i].interval, _interval[i], _maxErr);
      }
      else
      {
        // The light is assumed to have never been updated from the beginning.
        EXPECT_LE(lastUp[i] - startTime.Double(), _maxErr);
      }
    }
  }

  /// \brief A list of records about actual duration/interval.
  protected: struct RecordInfo flashLight[4];

  /// \brief A flag indicating whether the callback was called.
  protected: bool called;

  /// \brief The time when the records were initialized.
  protected: common::Time startTime;

  /// \brief Protect data from races.
  protected: std::mutex mutex;
};

/////////////////////////////////////////////////
TEST_F(FlashLightTest, OffAndOn)
{
  using namespace std::chrono_literals;


  // Load a world.
  Load("flash_light_example.world", false);
  // Get a pointer to the world, make sure world loads.
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Subscribe to plugin notifications.
  transport::SubscriberPtr sceneSub
    = this->node->Subscribe("~/light/modify", &FlashLightTest::lightCb,
      (FlashLightTest*)this);

  // Get a ROS service client.
  ros::NodeHandle n;
  ros::ServiceClient client
    = n.serviceClient<std_srvs::SetBool>("/light_switch");
  EXPECT_TRUE(client.isValid());
  EXPECT_TRUE(client.waitForExistence());

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval.
  std::this_thread::sleep_for(1s);

  // Check if the actual duration and interval are the supposed ones.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {true, true, false, false};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(updated, duration, interval, 0.01);
  }

  // Turn them off.
  gzmsg << "Turning off all lights" << std::endl;
  {
    std_srvs::SetBool srv;
    srv.request.data = false;
    bool success = client.call(srv);
    EXPECT_TRUE(success);
  }

  // Allow some time for the plugin to stop blinking.
  // After this, the callback function is not supposed to be called any more.
  std::this_thread::sleep_for(100ms);

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval.
  std::this_thread::sleep_for(500ms);

  // Check if all of them stopped to be updated.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {false, false, false, false};
    std::vector<double> duration = {-1, -1, -1, -1};
    std::vector<double> interval = {-1, -1, -1, -1};
    this->CheckRec(updated, duration, interval, 0.01);
  }

  // Turn them on.
  gzmsg << "Turning on all lights" << std::endl;
  {
    std_srvs::SetBool srv;
    srv.request.data = true;
    bool success = client.call(srv);
    EXPECT_TRUE(success);
  }

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval.
  std::this_thread::sleep_for(2s);

  // Check if the duration and interval of lights were changed as expected.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> updated = {true, true, true, true};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(updated, duration, interval, 0.01);
  }

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{

  initGazeboEnv();

  ::testing::InitGoogleTest(&argc, argv);

  // Start ROS
  ros::init(argc, argv, "test_flashlight");
  
  return RUN_ALL_TESTS();
}
