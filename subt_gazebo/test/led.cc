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

#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/transport/transport.hh>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

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
class LedTest : public testing::Test
{
  // Constructor.
  public: LedTest(): testing::Test()
  {
  }

  // Destructor.
  public: virtual ~LedTest()
  {
  }

  // Callback for camera image
  public: void CameraCb(ConstImageStampedPtr &_msg);

  /// \brief Initializer.
  protected: void InitRec();

  /// \brief A function to check if we got the assumed results.
  /// \param[in] _updated Whether the light has been updated within its phase.
  /// \param[in] _duration Expected duration time.
  /// \param[in] _interval Expected interval time.
  /// \param[in] _maxErr Maximum error which can be accepted in seconds.
  protected: void CheckRec(
    const bool &_updated,
    const double &_duration,
    const double &_interval,
    const double &_maxErr);

  /// \brief Node for Gazebo transport.
  protected: gazebo::transport::NodePtr node;

  /// \brief Subscriber to get the camera image.
  protected: transport::SubscriberPtr cameraSub;

  /// \brief ROS node handler.
  protected: ros::NodeHandle n;

  /// \brief ROS service client.
  protected: ros::ServiceClient client;

  /// \brief A record about actual duration/interval.
  private: struct RecordInfo led;

  /// \brief True if a response is sent back.
  private: bool called;

  /// \brief The time when the records were initialized.
  private: common::Time startTime;

  /// \brief Protect data from races.
  private: std::mutex mutexData;
};

/////////////////////////////////////////////////
void LedTest::CameraCb(ConstImageStampedPtr &_msg)
{
  // Get the current time.
  common::Time currentSimTime(ros::Time::now().toSec());

  // Get the camera image
  msgs::Image imageMsg = _msg->image();
  common::Image image;
  msgs::Set(image, imageMsg);
  // Get the left top pixel.
  ignition::math::Color color = image.Pixel(0, 0);

  bool flashing = false;
  if (color.R() > 0.9 && color.G() > 0.9 && color.B() > 0.9)
    flashing = true;

  // Update to flash
  // dim -> flash
  if (!this->led.flashing)
  {
    if (flashing)
    {
      std::lock_guard<std::mutex> lk(this->mutexData);
      this->led.interval
        = currentSimTime.Double()
          - this->led.lastUpdate.Double();
      // Update the last update time
      this->led.lastUpdate = currentSimTime;
      // Update the current flashing state.
      this->led.flashing = true;
    }
  }
  // flash -> dim
  else
  {
    if (!flashing)
    {
      std::lock_guard<std::mutex> lk(this->mutexData);
      this->led.duration
        = currentSimTime.Double()
          - this->led.lastUpdate.Double();
      // Update the last update time
      this->led.lastUpdate = currentSimTime;
      // Update the current flashing state.
      this->led.flashing = false;
    }
  }
  {
    std::lock_guard<std::mutex> lk(this->mutexData);
    this->called = true;
  }
}

/////////////////////////////////////////////////
void LedTest::InitRec()
{
  common::Time currentSimTime(ros::Time::now().toSec());
  {
    std::lock_guard<std::mutex> lk(this->mutexData);
    this->led.duration = -1;
    this->led.interval = -1;
    this->led.flashing = false;
    this->led.lastUpdate = currentSimTime;
    this->startTime = currentSimTime;
    this->called = false;
  }
}

/////////////////////////////////////////////////
void LedTest::CheckRec(
  const bool &_updated,
  const double &_duration,
  const double &_interval,
  const double &_maxErr)
{
  // Get necessary time information.
  common::Time endTime(ros::Time::now().toSec());
  std::lock_guard<std::mutex> lk(this->mutexData);
  ASSERT_TRUE(this->called) << "The callback funciton was not called.";
  double lastUp = this->led.lastUpdate.Double();

  // Check records of each light
  // NOTE: Taking some errors caused by callback functions into consideration.
  // NOTE: If the interval is 0, the callback is not called.
  if (_updated && _interval > 0)
  {
    // The light is assumed to have been updated within its phase.
    EXPECT_LE(endTime.Double() - lastUp,
              std::max(_duration, _interval) + _maxErr);

    // The light has been flashing by the assumed duration and interval.
    EXPECT_NEAR(led.duration, _duration, _maxErr);
    EXPECT_NEAR(led.interval, _interval, _maxErr);
  }
  else
  {
    // The light is assumed to have never been updated from the beginning.
    EXPECT_LE(lastUp - this->startTime.Double(), _maxErr);
  }
}

/////////////////////////////////////////////////
TEST_F(LedTest, switchOffAndOn)
{
  // NOTE: this additional time is to make sure that a visual object to update
  // has been created in the environment before publishing a message.
  // Otherwise, a duplicate object will be created and the original one will
  // never be updated.
  // This problem is solved by the patch (Pull Request # 2983), which has
  // been merged into gazebo7 as of July 16, 2018. This if satement should be
  // removed once the patch is forwarded up to gazebo9.
  ros::Duration(1.5).sleep();

  // ROS spinning
  std::shared_ptr<ros::AsyncSpinner> async_ros_spin_;
  async_ros_spin_.reset(new ros::AsyncSpinner(0));
  async_ros_spin_->start();

  // Initialize the transport node
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  // Subcribe for camera.
  this->cameraSub
    = this->node->Subscribe("~/camera/link/camera/image",
                            &LedTest::CameraCb,
                            dynamic_cast<LedTest*>(this));

  // Get a ROS service client.
  this->client
    = this->n.serviceClient<std_srvs::SetBool>("/light_model/light_switch");
  ASSERT_TRUE(this->client.isValid());
  ASSERT_TRUE(this->client.waitForExistence());

  ros::Duration(0.5).sleep();

  this->InitRec();

  ros::Duration(2.0).sleep();

  // Check if the actual duration and interval are the supposed ones.
  this->CheckRec(true, 0.8, 0.6, 0.25);

  // Turn it off.
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

  // Check if it stopped to be updated.
  this->CheckRec(false, -1, -1, 0.25);

  // Turn it on.
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

  ros::Duration(2.0).sleep();

  // Check if the duration and interval of lights were changed as expected.
  this->CheckRec(true, 0.8, 0.6, 0.25);

  this->node->Fini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // Start ROS
  ros::init(argc,argv,"test_led");

  // Start Gazebo client
  gazebo::client::setup(argc, argv);

  return RUN_ALL_TESTS();
}
