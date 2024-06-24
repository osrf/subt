/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <subt_communication_broker/subt_communication_client.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <ignition/common/Console.hh>

using namespace std;
using namespace subt;
using namespace subt::communication_broker;

std::shared_ptr<ros::NodeHandle> nh;
ros::Publisher clockPub;
ros::Time currentTime {0, 0};

/////////////////////////////////////////////////
void advanceTime()
{
  currentTime += ros::Duration(1);
  ros::Time::setNow(currentTime);
  rosgraph_msgs::Clock msg;
  msg.clock = currentTime;
  clockPub.publish(msg);
  ros::spinOnce();
  ros::WallDuration(0.01).sleep();
}

/////////////////////////////////////////////////
TEST(relay, unicast)
{

  // Test communication on broadcast utilizing SubtRosRelay and a dummy broker.

  ros::Time::init();
  advanceTime();

  // give other nodes time to spin up
  ros::WallDuration(1).sleep();

  CommsClient broadcaster("b", false, false, false, nh.get());
  advanceTime();
  CommsClient c1("c1", false, false, false, nh.get());
  advanceTime();
  CommsClient c2("c2", false , false, false, nh.get());
  advanceTime();
  CommsClient c3("c3", false , false, false, nh.get());
  advanceTime();

  ROS_INFO_STREAM("Test running on master " << ros::master::getURI());

  vector<string> receivedData1, receivedData2;
  auto c1_cb = [=,&receivedData1](const std::string& src,
                                  const std::string& dst,
                                  const uint32_t port,
                                  const std::string& data)
  {
    receivedData1.emplace_back(data);
  };
  auto c2_cb = [=,&receivedData2](const std::string& src,
                                  const std::string& dst,
                                  const uint32_t port,
                                  const std::string& data)
  {
    receivedData2.emplace_back(data);
  };
  auto c3_cb = [](const std::string& src,
                  const std::string& dst,
                  const uint32_t port,
                  const std::string& data)
  {
    GTEST_NONFATAL_FAILURE_("c3 callback should not have been called");
  };

  // intentionally bind c1 twice to get twice as many data and verify one client
  // can bind one port multiple times
  ASSERT_FALSE(!c1.Bind(c1_cb, "", 123)); advanceTime();
  ASSERT_FALSE(!c1.Bind(c1_cb, "", 123)); advanceTime();
  ASSERT_FALSE(!c2.Bind(c2_cb, "", 123)); advanceTime();
  // c3 binds to a different port and thus should receive nothing
  ASSERT_FALSE(!c3.Bind(c3_cb, "", 124)); advanceTime();

  // give the bound endpoints time to set up
  advanceTime();  // advance the test broker
  ros::WallDuration(1).sleep();
  ros::spinOnce();

  vector<string> sentData;
  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello clients, " << i;
    ASSERT_TRUE(broadcaster.SendTo(oss.str(), "c1", 123));
    sentData.emplace_back(oss.str());
    advanceTime();  // advance the test broker
    ros::spinOnce();
  }

  ros::spinOnce();
  advanceTime();  // advance the test broker
  ros::spinOnce();

  ASSERT_EQ(2u * sentData.size(), receivedData1.size());
  ASSERT_EQ(0u, receivedData2.size());

  for (size_t i = 0; i < sentData.size(); ++i)
  {
    EXPECT_EQ(sentData[i], receivedData1[2*i]);
    EXPECT_EQ(sentData[i], receivedData1[2*i + 1]);
  }

  receivedData1.clear();
  receivedData2.clear();
  sentData.clear();

  ros::spinOnce();
  advanceTime();  // advance the test broker
  ros::spinOnce();

  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello clients, " << i;
    ASSERT_TRUE(broadcaster.SendTo(oss.str(), "c2", 123));
    sentData.emplace_back(oss.str());
    advanceTime();  // advance the test broker
    ros::spinOnce();
  }

  ros::spinOnce();
  advanceTime();  // advance the test broker
  ros::spinOnce();

  ASSERT_EQ(0u, receivedData1.size());
  ASSERT_EQ(sentData.size(), receivedData2.size());

  for (size_t i = 0; i < sentData.size(); ++i)
  {
    EXPECT_EQ(sentData[i], receivedData2[i]);
  }

  // selftest - test situation when a client sends a message to an endpoint it
  // has also bound

  receivedData1.clear();
  receivedData2.clear();
  sentData.clear();

  ros::spinOnce();
  advanceTime();  // advance the test broker
  ros::spinOnce();

  ASSERT_TRUE(c2.SendTo("Selftest", "c2", 123));

  sentData.emplace_back("Selftest");
  advanceTime();  // advance the test broker
  ros::spinOnce();

  EXPECT_EQ(0u, receivedData1.size());
  EXPECT_EQ(1u, receivedData2.size());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_comms_relay");
  nh.reset(new ros::NodeHandle);
  clockPub = nh->advertise<rosgraph_msgs::Clock>("/clock", 1, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


