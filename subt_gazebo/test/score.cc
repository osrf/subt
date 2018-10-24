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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <chrono>
#include <memory>
#include <gazebo/gazebo.hh>

#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/CommsClient.hh"
#include "subt_gazebo/protobuf/artifact.pb.h"
#include "test/test_config.h"
#include "TestUtils.hh"

using namespace subt;

/////////////////////////////////////////////////
/// \brief A fixture class for testing the GameLogic plugin.
class ScoreTest : public testing::Test, public subt::GazeboTest
{
  /// \brief Constructor.
  public: ScoreTest()
  {
    this->client.reset(new subt::CommsClient("my_address"));

    // Wait until Gazebo is ready.
    using namespace std::chrono_literals;
    EXPECT_TRUE(this->WaitForGazebo(120s));
  }

  /// \brief Reset the member variables used for checking test expectations.
  protected: void Reset()
  {
    this->SetUp();
  }

  /// \brief Check the score after the competition has started.
  protected: void TestScoreAfterStart()
  {
    // Make sure that we start receiving score updates.
    ASSERT_TRUE(this->WaitUntilScoreIs(0));

    // Report an artifact with high accuracy (x3): +9 points.
    ignition::msgs::Pose pose;
    pose.mutable_position()->set_x(140.0);
    pose.mutable_position()->set_y(35.0);
    pose.mutable_position()->set_z(-20.0);
    uint32_t type = static_cast<uint32_t>(
      gazebo::GameLogicPlugin::ArtifactType::TYPE_BACKPACK);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(9));

    // Report an artifact with medium accuracy (x2): +6 points.
    pose.mutable_position()->set_x(241.0);
    pose.mutable_position()->set_y(25.0);
    pose.mutable_position()->set_z(-35.0);
    type = static_cast<uint32_t>(
      gazebo::GameLogicPlugin::ArtifactType::TYPE_TOOLBOX);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(15));

    // Report an artifact with low accuracy (x1): +3 points.
    pose.mutable_position()->set_x(133.0);
    pose.mutable_position()->set_y(2.2);
    pose.mutable_position()->set_z(-20.0);
    type = static_cast<uint32_t>(
      gazebo::GameLogicPlugin::ArtifactType::TYPE_EXTINGUISHER);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(18));

    // Report an artifact with bad accuracy (-1): 0 points.
    pose.mutable_position()->set_x(127.5);
    pose.mutable_position()->set_y(-65.0);
    pose.mutable_position()->set_z(-30.0);
    type = static_cast<uint32_t>(
      gazebo::GameLogicPlugin::ArtifactType::TYPE_VALVE);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(18));
  }

  /// \brief Callback registered for receiving score updates.
  private: void OnScore(const std_msgs::Int32 &_msg)
  {
    this->score = _msg.data;
  }

  /// \brief Documentation inherited.
  private: virtual void SetUp()
  {
    this->score = 0;
    this->scoreSub = this->nodeHandle.subscribe(
      "/subt/score", 100, &ScoreTest::OnScore, this);
  }

  /// \brief Wait until the score reaches a given target score.
  /// \param[in] _targetScore The desired target score.
  /// \return True if the score reached the target score or false otherwise.
  private: bool WaitUntilScoreIs(const int32_t _targetScore) const
  {
    unsigned int retries = 0u;
    while (this->score != _targetScore && retries < 20u)
    {
      ++retries;
      ros::spinOnce();

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(std::chrono::milliseconds(200ms));
    }

    return this->score == _targetScore;
  }

  /// \brief Report a new artifact.
  /// \param[in] _type The artifact type.
  /// \param[in] _pose The artifact pose.
  private: void ReportArtifact(const uint32_t _type,
                               const ignition::msgs::Pose &_pose)
  {
    subt::msgs::Artifact artifact;
    artifact.set_type(_type);
    artifact.mutable_pose()->CopyFrom(_pose);
    this->client->SendToBaseStation(artifact);
  }

  /// \brief Whether a unicast/broadcast message has been received or not.
  protected: int32_t score;

  /// \brief The ROS node comms handler.
  protected: ros::NodeHandle nodeHandle;

  /// \brief The ROS topic subscriber to receive score updates.
  protected: ros::Subscriber scoreSub;

  /// \brief Communication client.
  protected: std::unique_ptr<subt::CommsClient> client;
};

/////////////////////////////////////////////////
TEST_F(ScoreTest, ScoreAfterStart)
{
  this->TestScoreAfterStart();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "comms_test");

  return RUN_ALL_TESTS();
}
