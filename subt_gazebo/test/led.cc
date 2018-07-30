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

  // Callback for Responses about entity_info
  public: void ResponseCb(ConstResponsePtr &_msg);

  /// \brief Send requests for entity information for a specified time.
  public: void SendRequests();

  /// \brief Node for Gazebo transport.
  protected: gazebo::transport::NodePtr node;

  /// \brief Subscriber to get the camera image.
  protected: transport::SubscriberPtr cameraSub;

  /// \brief Publisher to request for entity_info
  protected: transport::PublisherPtr entityInfoPub;

  /// \brief Subscriber to get entity_info.
  protected: transport::SubscriberPtr entityInfoSub;

  /// \brief ROS node handler.
  protected: ros::NodeHandle n;

  /// \brief ROS service client.
  protected: ros::ServiceClient client;

  /// \brief True if a response is sent back.
  protected: bool cameraCalled;

  /// \brief True if a visual object is flashing.
  protected: bool visual_flashing;

  /// \brief Protect data from races.
  protected: std::mutex mutexCamera;

  /// \brief True if a response is sent back.
  protected: bool responded;

  /// \brief True if a light object is flashing.
  protected: bool light_flashing;

  /// \brief Protect data from races.
  protected: std::mutex mutexResponse;

  /// \brief A thread to send requests for entity_info
  protected: std::thread requester;

  /// \brief True if the thread should keep running.
  protected: bool running;

  /// \brief Protect the flag.
  protected: std::mutex mutexRunning;
};

/////////////////////////////////////////////////
void LedTest::CameraCb(ConstImageStampedPtr &_msg)
{
  // Get the camera image
  msgs::Image imageMsg = _msg->image();
  common::Image image;
  msgs::Set(image, imageMsg);
  // Get the left top pixel.
  ignition::math::Color color = image.Pixel(0, 0);

  std::lock_guard<std::mutex> lk(this->mutexCamera);
  this->visual_flashing = false;
  if (color.R() > 0.9 && color.G() > 0.9 && color.B() > 0.9)
    this->visual_flashing = true;

  this->cameraCalled = true;
}

/////////////////////////////////////////////////
void LedTest::ResponseCb(ConstResponsePtr &_msg)
{
  // Extract necessary information from the response message.
  msgs::Model modelMsg;
  modelMsg.ParseFromString(_msg->serialized_data());

  ASSERT_TRUE(modelMsg.link_size() > 0) << "The model must have a link.";
  msgs::Link linkMsg = modelMsg.link(0);
  ASSERT_TRUE(linkMsg.light_size() > 0) << "The link must have a light.";
  msgs::Light lightMsg = linkMsg.light(0);

  std::lock_guard<std::mutex> lk(this->mutexResponse);

  if (lightMsg.range() > 0)
    this->light_flashing = true;
  else
    this->light_flashing = false;

  this->responded = true;
}

/////////////////////////////////////////////////
void LedTest::SendRequests()
{
  double timeToSleep = 0.2;
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

  this->cameraCalled = false;
  this->responded = false;
  this->light_flashing = false;
  this->visual_flashing = false;

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

  this->entityInfoPub
    = this->node->Advertise<msgs::Request>("~/request");
  // Subcribe for entity_info.
  this->entityInfoSub
    = this->node->Subscribe("~/response",
                            &LedTest::ResponseCb,
                            dynamic_cast<LedTest*>(this));

  // Get a ROS service client.
  this->client
    = this->n.serviceClient<std_srvs::SetBool>("/light_model/light_switch");
  ASSERT_TRUE(this->client.isValid());
  ASSERT_TRUE(this->client.waitForExistence());

  // Start the thread to repeatedly publishing a request.
  this->running = true;
  this->requester = std::thread(&LedTest::SendRequests,
                                dynamic_cast<LedTest*>(this));

  ros::Duration(0.5).sleep();

  {
    std::lock_guard<std::mutex> lk(this->mutexResponse);
    ASSERT_TRUE(this->responded);
  }
  {
    std::lock_guard<std::mutex> lk(this->mutexCamera);
    ASSERT_TRUE(this->cameraCalled);
  }

  // Check if the light is flashing.
  {
    std::lock_guard<std::mutex> lk(this->mutexResponse);
    EXPECT_TRUE(this->light_flashing) << "The light is not flashing.";
  }
  {
    std::lock_guard<std::mutex> lk(this->mutexCamera);
    EXPECT_TRUE(this->visual_flashing) << "The visual is not flashing.";
  }

  // Turn it off.
  {
    std_srvs::SetBool srv;
    srv.request.data = false;
    bool success = this->client.call(srv);
    EXPECT_TRUE(success);
  }

  ros::Duration(0.5).sleep();

  // Check the results.
  {
    std::lock_guard<std::mutex> lk(this->mutexResponse);
    EXPECT_FALSE(this->light_flashing) << "The light was not turned off.";
  }
  {
    std::lock_guard<std::mutex> lk(this->mutexCamera);
    EXPECT_FALSE(this->visual_flashing) << "The visual was not turned off.";
  }

  // Turn it on again.
  {
    std_srvs::SetBool srv;
    srv.request.data = true;
    bool success = this->client.call(srv);
    EXPECT_TRUE(success);
  }

  ros::Duration(0.5).sleep();

  // Check if the light is flashing.
  {
    std::lock_guard<std::mutex> lk(this->mutexResponse);
    EXPECT_TRUE(this->light_flashing) << "The light is not flashing.";
  }
  {
    std::lock_guard<std::mutex> lk(this->mutexCamera);
    EXPECT_TRUE(this->visual_flashing) << "The visual is not flashing.";
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
  ros::init(argc,argv,"test_led");

  // Start Gazebo client
  gazebo::client::setup(argc, argv);

  return RUN_ALL_TESTS();
}
