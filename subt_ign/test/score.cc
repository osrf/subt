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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <queue>
#include <ignition/transport/Node.hh>

#include "subt_ign/CommonTypes.hh"
#include "subt_communication_broker/subt_communication_client.h"
#include "subt_ign/protobuf/artifact.pb.h"
#include "TestUtils.hh"

using namespace subt;
using namespace std::chrono_literals;

/////////////////////////////////////////////////
/// \brief A fixture class for testing the GameLogic plugin.
class ScoreTest : public testing::Test, public subt::GazeboTest
{
  /// \brief Constructor.
  public: ScoreTest()
  {
    // Wait for models to spawn.
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // Wait until Gazebo is ready.
    using namespace std::chrono_literals;
    EXPECT_TRUE(this->WaitForGazebo(120s));

    this->client.reset(new subt::CommsClient("X2", false, true));
    this->client->Bind(&ScoreTest::OnArtifactAck, this);
  }

  /// \brief Callback for artifact score acknowledgement
  protected: void OnArtifactAck(const std::string &/*_srcAddress*/,
                                const std::string &/*_dstAddress*/,
                                const uint32_t /*_dstPort*/,
                                const std::string &_data)
  {
    subt::msgs::ArtifactScore ack;
    if (!ack.ParseFromString(_data))
    {
      std::cerr << "Error parsing artifact score response" << std::endl;
    }
    else
    {
      scoreAcks.push(ack);
    }
  }

  /// \brief Reset the member variables used for checking test expectations.
  protected: void Reset()
  {
    this->SetUp();
  }

  /// \brief Check that artifacts before scoring begins are rejected.
  protected: void TestScoreBeforeStart()
  {
    ignition::msgs::Pose pose;
    uint32_t type = static_cast<uint32_t>(subt::ArtifactType::TYPE_BACKPACK);
    this->ReportArtifact(type, pose);

    {
      ASSERT_TRUE(this->WaitUntilScoreAck());
      auto ack = this->scoreAcks.front();
      EXPECT_EQ(ack.report_id(), this->reportCount);
      EXPECT_EQ(ack.artifact().type(), type);
      EXPECT_EQ(ack.run(), 1u);
      EXPECT_EQ(ack.report_status(), "run not started");
      EXPECT_EQ(ack.score_change(), 0);
      this->scoreAcks.pop();
    }
  }

  /// \brief Check the score after the competition has started.
  protected: void TestScoreAfterStart()
  {
    // Make sure that we start receiving score updates.
    ASSERT_TRUE(this->WaitUntilScoreIs(0));

    ignition::msgs::StringMsg req;
    ignition::msgs::Pose rep;
    unsigned int timeout = 5000;
    bool result;
    req.set_data("X1");

    EXPECT_TRUE(this->node.Request("/subt/pose_from_artifact_origin",
      req, timeout, rep, result));
    EXPECT_TRUE(result);

    EXPECT_NEAR(-6.0,    rep.position().x(),    0.1);
    EXPECT_NEAR(5.0,     rep.position().y(),    0.1);
    EXPECT_NEAR(0.1303,  rep.position().z(),    0.1);
    EXPECT_NEAR(0,       rep.orientation().x(), 0.1);
    EXPECT_NEAR(0,       rep.orientation().y(), 0.1);
    EXPECT_NEAR(0,       rep.orientation().z(), 0.1);
    EXPECT_NEAR(1,       rep.orientation().w(), 0.1);

    req.set_data("X2");
    EXPECT_TRUE(this->node.Request("/subt/pose_from_artifact_origin",
      req, timeout, rep, result));
    EXPECT_TRUE(result);

    EXPECT_NEAR(-6.0,    rep.position().x(),    0.1);
    EXPECT_NEAR(3.0,     rep.position().y(),    0.1);
    EXPECT_NEAR(0.1303,  rep.position().z(),    0.1);
    EXPECT_NEAR(0,       rep.orientation().x(), 0.1);
    EXPECT_NEAR(0,       rep.orientation().y(), 0.1);
    EXPECT_NEAR(0,       rep.orientation().z(), 0.1);
    EXPECT_NEAR(1,       rep.orientation().w(), 0.1);

    ignition::math::Pose3d x2Pose(4, 3, 0.131, 0, 0, 0);

    // The test positions are associated with artifacts in
    /// the tunnel_qual_ign.sdf world

    // Report an artifact with high accuracy (phone_2): +1 point.
    ignition::math::Pose3d artifact1Pose(201.8, 20.0, -15.0, 0, 0, 0);
    double err = 0.0;
    ignition::msgs::Pose pose;
    pose.mutable_position()->set_x(
      artifact1Pose.Pos().X() -
      (x2Pose.Pos().X() - rep.position().x()) + err);
    pose.mutable_position()->set_y(
      artifact1Pose.Pos().Y() - (x2Pose.Pos().Y() - rep.position().y()));
    pose.mutable_position()->set_z(
      artifact1Pose.Pos().Z() - (x2Pose.Pos().Z() - rep.position().z()));
    uint32_t type = static_cast<uint32_t>(
        subt::ArtifactType::TYPE_PHONE);

    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(1));

    {
      ASSERT_TRUE(this->WaitUntilScoreAck());
      auto ack = this->scoreAcks.front();
      EXPECT_EQ(ack.report_id(), this->reportCount);
      EXPECT_EQ(ack.artifact().type(), type);
      EXPECT_EQ(ack.artifact().pose().position().x(), pose.position().x());
      EXPECT_EQ(ack.artifact().pose().position().y(), pose.position().y());
      EXPECT_EQ(ack.artifact().pose().position().z(), pose.position().z());
      EXPECT_EQ(ack.run(), 1u);
      EXPECT_EQ(ack.report_status(), "scored");
      EXPECT_EQ(ack.score_change(), 1);
      this->scoreAcks.pop();
    }

    // Report an artifact with medium accuracy (phone_1): +1 point.
    ignition::math::Pose3d artifact2Pose(260, 160, -15, 0, 0, 0);
    err = 1.0;
    pose.mutable_position()->set_x(
      artifact2Pose.Pos().X() -
      (x2Pose.Pos().X() - rep.position().x()) + err);
    pose.mutable_position()->set_y(
      artifact2Pose.Pos().Y() - (x2Pose.Pos().Y() - rep.position().y()));
    pose.mutable_position()->set_z(
      artifact2Pose.Pos().Z() - (x2Pose.Pos().Z() - rep.position().z()));
    type = static_cast<uint32_t>(subt::ArtifactType::TYPE_PHONE);
    this->ReportArtifact(type, pose);
    ASSERT_TRUE(this->WaitUntilScoreIs(2));

    {
      ASSERT_TRUE(this->WaitUntilScoreAck());
      auto ack = this->scoreAcks.front();
      EXPECT_EQ(ack.report_id(), this->reportCount);
      EXPECT_EQ(ack.artifact().type(), type);
      EXPECT_EQ(ack.artifact().pose().position().x(), pose.position().x());
      EXPECT_EQ(ack.artifact().pose().position().y(), pose.position().y());
      EXPECT_EQ(ack.artifact().pose().position().z(), pose.position().z());
      EXPECT_EQ(ack.run(), 1u);
      EXPECT_EQ(ack.report_status(), "scored");
      EXPECT_EQ(ack.score_change(), 1);
      this->scoreAcks.pop();
    }

    // Report an artifact with bad accuracy (-1): 0 points.
    ignition::math::Pose3d artifact4Pose(128.810, 74.807, 0.844, 0, 0, 0);
    err = 5.1;
    pose.mutable_position()->set_x(
      artifact4Pose.Pos().X() - x2Pose.Pos().X() - rep.position().x() + err);
    pose.mutable_position()->set_y(
      artifact4Pose.Pos().Y() - x2Pose.Pos().Y() - rep.position().y());
    pose.mutable_position()->set_z(
      artifact4Pose.Pos().Z() - x2Pose.Pos().Z() - rep.position().z());
    type = static_cast<uint32_t>(subt::ArtifactType::TYPE_DRILL);
    this->ReportArtifact(type, pose);
    {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(std::chrono::milliseconds(200ms));
    }
    ASSERT_TRUE(this->WaitUntilScoreIs(2));

    {
      ASSERT_TRUE(this->WaitUntilScoreAck());
      auto ack = this->scoreAcks.front();
      EXPECT_EQ(ack.report_id(), this->reportCount);
      EXPECT_EQ(ack.artifact().type(), type);
      EXPECT_EQ(ack.artifact().pose().position().x(), pose.position().x());
      EXPECT_EQ(ack.artifact().pose().position().y(), pose.position().y());
      EXPECT_EQ(ack.artifact().pose().position().z(), pose.position().z());
      EXPECT_EQ(ack.run(), 1u);
      EXPECT_EQ(ack.report_status(), "scored");
      EXPECT_EQ(ack.score_change(), 0);
      this->scoreAcks.pop();
    }

    /// Test artifact report limits. Send multiple bad reports.
    /// We are currently expect a limit of: artifact_count * 2 == 6.
    /// The world we are testing with has 3 artifacts.
    pose.mutable_position()->set_x(-100000);
    for (; this->reportCount < 6u;)
    {
      this->ReportArtifact(type, pose);
      {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(std::chrono::milliseconds(200ms));
      }

      ASSERT_TRUE(this->WaitUntilScoreAck());
      auto ack = this->scoreAcks.front();
      EXPECT_EQ(this->reportCount, ack.report_id());
      this->scoreAcks.pop();
    }
    // Send one more report
    this->ReportArtifact(type, pose);
    {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(std::chrono::milliseconds(200ms));
    }
    auto ack = this->scoreAcks.front();
    EXPECT_EQ(this->reportCount, ack.report_id());
    EXPECT_EQ("report limit exceeded", ack.report_status());
    this->scoreAcks.pop();
  }

  /// \brief Check that artifacts after scoring ends are rejected.
  protected: void TestScoreAfterFinish()
  {
    ignition::msgs::Pose pose;
    uint32_t type = static_cast<uint32_t>(subt::ArtifactType::TYPE_BACKPACK);
    this->ReportArtifact(type, pose);

    {
      ASSERT_TRUE(this->WaitUntilScoreAck());
      auto ack = this->scoreAcks.front();
      EXPECT_EQ(ack.report_id(), this->reportCount);
      EXPECT_EQ(ack.artifact().type(), type);
      EXPECT_EQ(ack.run(), 1u);
      EXPECT_EQ(ack.report_status(), "scoring finished");
      EXPECT_EQ(ack.score_change(), 0);
      this->scoreAcks.pop();
    }
  }

  /// \brief Callback registered for receiving score updates.
  private: void OnScore(const ignition::msgs::Float &_msg)
  {
    this->score = _msg.data();
  }

  /// \brief Documentation inherited.
  private: virtual void SetUp()
  {
    this->score = 0;
    this->node.Subscribe("/subt/score", &ScoreTest::OnScore, this);
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
      std::this_thread::sleep_for(200ms);
    }

    return this->score == _targetScore;
  }

  private: bool WaitUntilScoreAck() const
  {
    unsigned int retries = 0u;
    while (this->scoreAcks.empty() && retries < 20u)
    {
      ++retries;
      std::this_thread::sleep_for(400ms);
    }

    return !this->scoreAcks.empty();
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
    this->reportCount++;
  }

  /// \brief Whether a unicast/broadcast message has been received or
  /// not.
  protected: float score;

  /// \brief Track number of reports submitted.
  protected: uint32_t reportCount;

  /// \brief Queue of incoming score acknowledgements.
  protected: std::queue<subt::msgs::ArtifactScore> scoreAcks;

  /// \brief The ignition transport node comms handler.
  protected: ignition::transport::Node node;

  /// \brief Communication client.
  protected: std::unique_ptr<subt::CommsClient> client;
};

/////////////////////////////////////////////////
TEST_F(ScoreTest, TestScoring)
{
  this->TestScoreBeforeStart();

  // Start the scoring
  {
    ignition::msgs::Boolean req;
    ignition::msgs::Boolean rep;
    unsigned int timeout = 5000;
    bool result;
    req.set_data(true);
    EXPECT_TRUE(this->node.Request("/subt/start", req, timeout, rep, result));
    EXPECT_TRUE(result);
  }

  this->TestScoreAfterStart();

  // Finish the scoring
  {
    ignition::msgs::Boolean req;
    ignition::msgs::Boolean rep;
    unsigned int timeout = 5000;
    bool result;
    req.set_data(true);
    EXPECT_TRUE(this->node.Request("/subt/finish", req, timeout, rep, result));
    EXPECT_TRUE(result);
  }

  this->TestScoreAfterFinish();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_score");

  return RUN_ALL_TESTS();
}
