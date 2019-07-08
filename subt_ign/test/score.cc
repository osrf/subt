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
#include <subt_msgs/PoseFromArtifact.h>
#include <chrono>
#include <memory>

#include "subt_ign/CommonTypes.hh"
#include "subt_communication_broker/subt_communication_client.h"
#include "subt_ign/protobuf/artifact.pb.h"
#include "TestUtils.hh"

using namespace subt;

/////////////////////////////////////////////////
/// \brief A fixture class for testing the GameLogic plugin.
class ScoreTest : public testing::Test, public subt::GazeboTest
{
  /// \brief Constructor.
  public: ScoreTest()
  {
    // Wait until Gazebo is ready.
    using namespace std::chrono_literals;
    EXPECT_TRUE(this->WaitForGazebo(120s));

    this->client.reset(new subt::CommsClient("X2"));
    this->client->Bind(&ScoreTest::OnArtifactAck, this);
  }

  protected: void OnArtifactAck(const std::string &_srcAddress,
                                const std::string &_dstAddress,
                                const uint32_t _dstPort,
                                const std::string &_data)
  {
    subt::msgs::ArtifactScore ack;
    if (!ack.ParseFromString(_data))
    {
      std::cerr << "Error parsing artifact score response" << std::endl;
    }
    else
    {
      std::cerr << "OnArtifactAck: " << ack.report_status() << std::endl;
    }
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

    // Check the service for reporting a robot position relative to the origin
    // artifact.
    ros::ServiceClient client =
      this->nodeHandle.serviceClient<subt_msgs::PoseFromArtifact>(
        "/subt/pose_from_artifact_origin");

    subt_msgs::PoseFromArtifact srv;
    srv.request.robot_name.data = "X1";
    EXPECT_TRUE(client.call(srv));
    geometry_msgs::Pose origin = srv.response.pose.pose;

    EXPECT_NEAR(2.0,     origin.position.x,    0.1);
    EXPECT_NEAR(1.0,     origin.position.y,    0.1);
    EXPECT_NEAR(-0.3687, origin.position.z,    0.1);
    EXPECT_NEAR(0,       origin.orientation.x, 0.1);
    EXPECT_NEAR(0,       origin.orientation.y, 0.1);
    EXPECT_NEAR(0,       origin.orientation.z, 0.1);
    EXPECT_NEAR(1,       origin.orientation.w, 0.1);

    ignition::math::Pose3d robotPose(4, 5, 0.131, 0, 0, 0);

    // Report an artifact with high accuracy (x3): +1 point.
    ignition::math::Pose3d artifact1Pose(81.953, 72.097, 1.298, 0, 0, 0);
    double err = 0.0;
    ignition::msgs::Pose pose;
    pose.mutable_position()->set_x(
      artifact1Pose.Pos().X() - robotPose.Pos().X() + origin.position.x + err);
    pose.mutable_position()->set_y(
      artifact1Pose.Pos().Y() - robotPose.Pos().Y() + origin.position.y);
    pose.mutable_position()->set_z(
      artifact1Pose.Pos().Z() - robotPose.Pos().Z() + origin.position.z);
    uint32_t type = static_cast<uint32_t>(subt::ArtifactType::TYPE_BACKPACK);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(1));

    // Report an artifact with medium accuracy (x2): +1 point.
    ignition::math::Pose3d artifact2Pose(103.841, 26.259, 0.751, -1.671, 0, 0);
    err = 1.0;
    pose.mutable_position()->set_x(
      artifact2Pose.Pos().X() - robotPose.Pos().X() + origin.position.x + err);
    pose.mutable_position()->set_y(
      artifact2Pose.Pos().Y() - robotPose.Pos().Y() + origin.position.y);
    pose.mutable_position()->set_z(
      artifact2Pose.Pos().Z() - robotPose.Pos().Z() + origin.position.z);
    type = static_cast<uint32_t>(subt::ArtifactType::TYPE_PHONE);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(2));

    // Report an artifact with low accuracy (x1): +1 point.
    ignition::math::Pose3d artifact3Pose(88.490, 133.324, 0.719, 0, 0, 3.1415);
    err = 4.99;
    pose.mutable_position()->set_x(
      artifact3Pose.Pos().X() - robotPose.Pos().X() + origin.position.x + err);
    pose.mutable_position()->set_y(
      artifact3Pose.Pos().Y() - robotPose.Pos().Y() + origin.position.y);
    pose.mutable_position()->set_z(
      artifact3Pose.Pos().Z() - robotPose.Pos().Z() + origin.position.z);
    type = static_cast<uint32_t>(subt::ArtifactType::TYPE_EXTINGUISHER);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(3));

    // Report an artifact with bad accuracy (-1): 0 points.
    ignition::math::Pose3d artifact4Pose(128.810, 74.807, 0.844, 0, 0, 0);
    err = 5.1;
    pose.mutable_position()->set_x(
      artifact4Pose.Pos().X() - robotPose.Pos().X() + origin.position.x + err);
    pose.mutable_position()->set_y(
      artifact4Pose.Pos().Y() - robotPose.Pos().Y() + origin.position.y);
    pose.mutable_position()->set_z(
      artifact4Pose.Pos().Z() - robotPose.Pos().Z() + origin.position.z);
    type = static_cast<uint32_t>(subt::ArtifactType::TYPE_DRILL);
    this->ReportArtifact(type, pose);
    {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(std::chrono::milliseconds(200ms));
    }
    ASSERT_TRUE(this->WaitUntilScoreIs(3));
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

    // Serialize the artifact.
    std::string serializedData;
    if (!artifact.SerializeToString(&serializedData)) {
      std::cerr
          << "ReportArtifact(): Error serializing message\n"
          << artifact.DebugString() << std::endl;
      ASSERT_TRUE(false);
    }

    this->client->SendTo(serializedData, subt::kBaseStationName);
  }

  /// \brief Whether a unicast/broadcast message has been received or
  /// not.
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
  ros::init(argc, argv, "score_test");

  return RUN_ALL_TESTS();
}
