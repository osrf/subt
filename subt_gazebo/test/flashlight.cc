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

#include <gtest/gtest.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/test/ServerFixture.hh>

#include "subt_gazebo/LightControlClient.hh"
#include "test/test_config.h"

using namespace gazebo;
using namespace subt;

// information to record for the lights in the enviornment
struct RecordInfo
{
  double duration;
  double interval;
  common::Time last_update;
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
    common::Time current_time = physics::get_world()->SimTime();
    this->flash_light[0].duration = -1;
    this->flash_light[1].duration = -1;
    this->flash_light[2].duration = -1;
    this->flash_light[3].duration = -1;
    this->flash_light[0].interval = -1;
    this->flash_light[1].interval = -1;
    this->flash_light[2].interval = -1;
    this->flash_light[3].interval = -1;
    this->flash_light[0].last_update = current_time;
    this->flash_light[1].last_update = current_time;
    this->flash_light[2].last_update = current_time;
    this->flash_light[3].last_update = current_time;
    this->f_called = false;
    this->s_time = current_time;
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

    bool f_correct_index = true;
    if (indx < 0 || 3 < indx)
    {
      f_correct_index = false;
    }
    EXPECT_TRUE(f_correct_index);

    physics::WorldPtr world = physics::get_world();

    // Do the following only if the world is available
    if (world != NULL)
    {
      // Get the current time

      common::Time current_time = world->SimTime();

      // Update to flash
      std::lock_guard<std::mutex> lk(this->mutex);
      if (_msg->range() > 0)
      {
        this->flash_light[indx].interval
          = current_time.Double() - this->flash_light[indx].last_update.Double();
      }
      // Update to dim
      else
      {
        this->flash_light[indx].duration
          = current_time.Double() - this->flash_light[indx].last_update.Double();
      }

      // Update the last update time
      this->flash_light[indx].last_update = current_time;

      this->f_called = true;
    }
  }

  /// \brief A function to check if we got the assumed results.
  /// \param[in] _f_update Whether the light has been updated within its phase.
  /// \param[in] _duration Expected duration time.
  /// \param[in] _interval Expected interval time.
  /// \param[in] _max_err Maximum error which can be accepted in seconds.
  protected: void CheckRec(
    const std::vector<bool> &_f_update,
    const std::vector<double> &_duration,
    const std::vector<double> &_interval,
    const double &_max_err)
  {
    // Determine if the callback function must have been called.
    // If at least one of the lights is to be updated, it is supposed to have
    // been called.
    bool f_called = false;
    for (int i = 0; i < 4; ++i)
    {
      if (_f_update[i])
      {
        f_called = true;
      }
    }

    // Get necessary time information.
    std::lock_guard<std::mutex> lk(this->mutex);
    common::Time &s_time = this->s_time;
    common::Time e_time = physics::get_world()->SimTime();
    double last_up[4];
    last_up[0] = this->flash_light[0].last_update.Double();
    last_up[1] = this->flash_light[1].last_update.Double();
    last_up[2] = this->flash_light[2].last_update.Double();
    last_up[3] = this->flash_light[3].last_update.Double();

    if (f_called)
    {
      // The callback function is assumed to have been called.
      EXPECT_TRUE(this->f_called);
    }
    else
    {
      // the callback function is assumed to have never been called.
      EXPECT_FALSE(this->f_called);
    }

    // Check records of each light
    // NOTE: Taking some errors caused by callback functions into consideration.
    // NOTE: If the interval is 0, the callback is not called.
    for (int i = 0; i < 4; ++i)
    {
      gzmsg << "checking light [" << i << "]" << std::endl;
      if (_f_update[i] && _interval[i] > 0)
      {
        // The light is assumed to have been updated within its phase.
        EXPECT_LE(e_time.Double() - last_up[i],
                  std::max(_duration[i], _interval[i]) + _max_err);

        // The light has been flashing by the assumed duration and interval.
        EXPECT_NEAR(this->flash_light[i].duration, _duration[i], _max_err);
        EXPECT_NEAR(this->flash_light[i].interval, _interval[i], _max_err);
      }
      else
      {
        // The light is assumed to have never been updated from the beginning.
        EXPECT_LE(last_up[i] - s_time.Double(), _max_err);
      }
    }
  }

  /// \brief A list of records about actual duration/interval.
  protected: struct RecordInfo flash_light[4];

  /// \brief A flag indicating whether the callback was called.
  protected: bool f_called;

  /// \brief The time when the records were initialized.
  protected: common::Time s_time;

  /// \brief Protect data from races.
  protected: std::mutex mutex;

  /// \brief Node.
  protected: transport::NodePtr node;

  protected: transport::SubscriberPtr sub;

};

/////////////////////////////////////////////////
TEST_F(FlashLightTest, Default)
{
  using namespace std::chrono_literals;

  // Load a world
  Load("flash_light_example.world", false);
  // Get a pointer to the world, make sure world loads.
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Subscribe to plugin notifications
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->sub
    = node->Subscribe("~/light/modify", &FlashLightTest::lightCb,
      (FlashLightTest*)this);

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval
  std::this_thread::sleep_for(1s);

  // Check if the actual duration and interval are the supposed ones.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> f_update = {true, true, false, false};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(f_update, duration, interval, 0.01);
  }

  // Initialize the records.
  this->InitRec();

  // Make clients
  gzmsg << "Creating Clients" << std::endl;
  subt::LightControlClient client1("light_source1", "cylinder");
  subt::LightControlClient client2("light_source2", "cylinder");
  subt::LightControlClient client3("light_source3", "box");
  subt::LightControlClient client4("light_source4", "box");

  // Wait for a while to measure the actual duration and interval
  std::this_thread::sleep_for(1s);

  // Check if the lights are not changed.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> f_update = {true, true, false, false};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(f_update, duration, interval, 0.01);
  }

  // Toggle them
  gzmsg << "Toggling the lights" << std::endl;
  client1.TurnOff();
  client2.TurnOff();
  client3.TurnOn();
  client4.TurnOn();

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval
  std::this_thread::sleep_for(2s);

  // Check if the lights are not changed.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> f_update = {false, false, true, true};
    std::vector<double> duration = {0.1, 0.05, 1.0, 1.0};
    std::vector<double> interval = {0.4, 0.05, 0.1, 0.0};
    this->CheckRec(f_update, duration, interval, 0.01);
  }

  // Turn them on
  gzmsg << "Turning on all lights" << std::endl;
  client1.TurnOn();
  client2.TurnOn();
  client3.TurnOn();
  client4.TurnOn();

  // Change their duration and interval
  gzmsg << "Changing duration and interval" << std::endl;
  client1.ChangeDuration(0.85);
  client1.ChangeInterval(0.1);
  client2.ChangeDuration(0.1);
  client2.ChangeInterval(0.0);
  client3.ChangeDuration(0.4);
  client3.ChangeInterval(0.25);
  client4.ChangeDuration(0.25);
  client4.ChangeInterval(0.55);

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval
  std::this_thread::sleep_for(2s);

  // Check if the duration and interval of lights were changed as expected.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> f_update = {true, true, true, true};
    std::vector<double> duration = {0.85, 0.1, 0.4, 0.25};
    std::vector<double> interval = {0.1, 0.0, 0.25, 0.55};
    this->CheckRec(f_update, duration, interval, 0.01);
  }

  // Turn them off
  gzmsg << "Turning off all lights" << std::endl;
  client1.TurnOff();
  client2.TurnOff();
  client3.TurnOff();
  client4.TurnOff();

  // Allow some time for the plugin to stop blinking
  // After this, the callback function is not supposed to be called any more.
  std::this_thread::sleep_for(100ms);

  // Initialize the records.
  this->InitRec();

  // Wait for a while to measure the actual duration and interval
  std::this_thread::sleep_for(500ms);

  // Check if all of them stopped to be updated.
  // NOTE: maximum error is set to 0.01 sec.
  {
    std::vector<bool> f_update = {false, false, false, false};
    std::vector<double> duration = {-1, -1, -1, -1};
    std::vector<double> interval = {-1, -1, -1, -1};
    this->CheckRec(f_update, duration, interval, 0.01);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  initGazeboEnv();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
