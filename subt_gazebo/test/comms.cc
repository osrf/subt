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
#include <chrono>
#include <gazebo/physics/physics.hh>
#include <gazebo/test/ServerFixture.hh>

#include "subt_gazebo/protobuf/datagram.pb.h"
#include "subt_gazebo/CommonTypes.hh"
#include "subt_gazebo/CommsClient.hh"
#include "test/test_config.h"

using namespace gazebo;
using namespace subt;

/////////////////////////////////////////////////
/// \brief A fixture class for testing the CommsBroker plugin within Gazebo and
/// clients using the CommsClient library.
class CommsTest : public ServerFixture
{
  /// \brief Constructor.
  public: CommsTest()
  {
    EXPECT_TRUE(this->client1.Bind(&CommsTest::OnUnicastMsg, this));
    EXPECT_TRUE(this->client1.Bind(&CommsTest::OnMulticastMsg, this,
      kMulticast));
  }

  /// \brief Reset the member variables used for checking test expectations.
  protected: void Reset()
  {
    this->SetUp();
  }

  /// \brief Test different user errors.
  protected: void TestBadUsage()
  {
    {
      // An empty address is not allowed.
      subt::CommsClient client("");
      EXPECT_FALSE(client.Bind(&CommsTest::OnUnicastMsg, this));
      EXPECT_FALSE(client.Bind(&CommsTest::OnMulticastMsg, this, kMulticast));
    }

    {
      // Bind() to an address that is not local is not allowed.
      subt::CommsClient client("addr");
      EXPECT_FALSE(client.Bind(&CommsTest::OnUnicastMsg, this, "other"));
    }

    {
      // A double Bind() is not allowed.
      subt::CommsClient client("addr");
      EXPECT_TRUE(client.Bind(&CommsTest::OnUnicastMsg, this));
      EXPECT_FALSE(client.Bind(&CommsTest::OnUnicastMsg, this));
      EXPECT_TRUE(client.Bind(&CommsTest::OnMulticastMsg, this, kMulticast));
      EXPECT_FALSE(client.Bind(&CommsTest::OnMulticastMsg, this, kMulticast));
    }
  }

  /// \brief Callback registered for receiving unicast (and broadcast) messages.
  private: void OnUnicastMsg(const std::string &_srcAddress,
                             const std::string &_dstAddress,
                             const uint32_t _dstPort,
                             const std::string &_data)
  {
    this->unicastCallbackExecuted = true;
    EXPECT_EQ("addr2", _srcAddress);
    EXPECT_TRUE(_dstAddress == "addr1" || _dstAddress == subt::kBroadcast);
    EXPECT_EQ(subt::kDefaultPort, _dstPort);
    EXPECT_EQ("_data_", _data);
  }

  /// \brief Callback registered for receiving multicast messages.
  private: void OnMulticastMsg(const std::string &_srcAddress,
                               const std::string &_dstAddress,
                               const uint32_t _dstPort,
                               const std::string &_data)
  {
    this->multicastCallbackExecuted = true;
    EXPECT_EQ("addr2", _srcAddress);
    EXPECT_EQ(subt::kMulticast, _dstAddress);
    EXPECT_EQ(subt::kDefaultPort, _dstPort);
    EXPECT_EQ("_data_", _data);
  }

  /// \brief Documentation inherited.
  private: virtual void SetUp()
  {
    this->unicastCallbackExecuted   = false;
    this->multicastCallbackExecuted = false;
  }

  /// \brief Whether a unicast/broadcast message has been received or not.
  protected: bool unicastCallbackExecuted = false;

  /// \brief Whether a multicast message has been received or not.
  protected: bool multicastCallbackExecuted = false;

  /// \brief A comms client.
  protected: subt::CommsClient client1{"addr1"};

  /// \brief A comms client.
  protected: subt::CommsClient client2{"addr2"};
};

/////////////////////////////////////////////////
TEST_F(CommsTest, BadUsage)
{
  this->TestBadUsage();
}

/////////////////////////////////////////////////
TEST_F(CommsTest, Comms)
{
  using namespace std::chrono_literals;

  // Load a world
  Load("lava_tube.world", false);

  // Get a pointer to the world, make sure world loads.
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Unicast.
  this->client2.SendTo("_data_", "addr1");

  waitUntilBoolVar(this->unicastCallbackExecuted, 1ms, 300);
  EXPECT_TRUE(this->unicastCallbackExecuted);
  EXPECT_FALSE(this->multicastCallbackExecuted);

  this->Reset();

  // Multicast.
  this->client2.SendTo("_data_", subt::kMulticast);

  waitUntilBoolVar(this->multicastCallbackExecuted, 1ms, 300);
  EXPECT_FALSE(this->unicastCallbackExecuted);
  EXPECT_TRUE(this->multicastCallbackExecuted);

  this->Reset();

  // Broadcast.
  this->client2.SendTo("_data_", subt::kBroadcast);

  waitUntilBoolVar(this->unicastCallbackExecuted, 1ms, 300);
  EXPECT_TRUE(this->unicastCallbackExecuted);
  EXPECT_FALSE(this->multicastCallbackExecuted);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  initGazeboEnv();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
