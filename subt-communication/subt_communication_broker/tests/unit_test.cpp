/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <gtest/gtest.h>

#define private public
#define protected public
#include <subt_communication_broker/subt_communication_broker.h>
#include <subt_communication_broker/subt_communication_client.h>
#include <subt_communication_model/subt_communication_model.h>
#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>
#undef protected
#undef private

using namespace subt;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;
using namespace subt::communication_broker;

void setDummyComms(Broker& broker)
{
  struct radio_configuration radio;
  radio.pathloss_f = [](const double&, radio_state&, radio_state&) {
    return rf_power();
  };

  broker.SetDefaultRadioConfiguration(radio);
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);
  broker.SetPoseUpdateFunction(
    [](const std::string& name)
    {
      return std::make_tuple(true, ignition::math::Pose3d::Zero, 0.0);
    });
}

TEST(broker, instatiate)
{
  Broker broker;
}

TEST(broker, communicate)
{
  Broker broker;

  // Build RF Function
  struct rf_configuration rf_config;
  rf_config.max_range = 10.0;
  auto rf_func = std::bind(&distance_based_received_power,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           std::placeholders::_3,
                           rf_config);

  // Build radio configuration (which includes RF pathloss function)
  struct radio_configuration radio;
  radio.capacity = 54000000;
  radio.default_tx_power = 27;
  radio.modulation = "QPSK";
  radio.noise_floor = -90;
  radio.pathloss_f = rf_func;

  // Instatiate two radios with this configuration
  broker.SetDefaultRadioConfiguration(radio);

  // Set communication function to use
  broker.SetCommunicationFunction(&subt::communication_model::attempt_send);

  auto pose_update_func = [](const std::string& name)
  {
    static double now;
    now += 1.0;
    if(name == "1")
    {
      return std::make_tuple(true, ignition::math::Pose3d::Zero, now);
    }

    if(name == "2")
    {
      static double x = 0;
      x += 1.0;
      std::cout << "Moving (2) to x=" << x << std::endl;
      return std::make_tuple(true,
          ignition::math::Pose3d(x, 0, 0, 0, 0, 0), now);
    }

    return std::make_tuple(false, ignition::math::Pose3d::Zero, 0.0);
  };

  broker.SetPoseUpdateFunction(pose_update_func);
  broker.Start();

  CommsClient c1("1", false, true, false);
  CommsClient c2("2", false , true, false);

  std::vector<std::string> receivedData;
  auto c2_cb = [=,&receivedData](const std::string& src,
                   const std::string& dst,
                   const uint32_t port,
                   const std::string& data)
  {
    std::cout << "Received " << data.size() << "("
      << data << ") bytes from " << src << std::endl;
    receivedData.emplace_back(data);
  };

  c2.Bind(c2_cb);

  std::unordered_set<std::string> sentData;
  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello c2, " << i;
    c1.SendTo(oss.str(), "2");
    sentData.insert(oss.str());
    broker.DispatchMessages();
  }

  // there is some packet loss on the path
  ASSERT_LT(8u, receivedData.size());

  for (const auto& data : receivedData)
  {
    EXPECT_NE(sentData.end(), sentData.find(data));
  }

  // geometry_msgs::PoseStamped a, b;
  // a.header.frame_id = "world";
  // b = a;

  // ASSERT_TRUE(attempt_send(radio,
  //                          {a, 0},  // TX state
  //                          {b, 1},  // RX state
  //                          1000)    // 1Kb packet
  //             );
}

TEST(broker, broadcast)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  CommsClient broadcaster("b", false, true, false);
  CommsClient c1("1", false, true, false);
  CommsClient c2("2", false , true, false);
  CommsClient c3("3", false , true, false);

  std::vector<std::string> receivedData1, receivedData2;
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
  EXPECT_FALSE(!c1.Bind(c1_cb, "", 123));
  EXPECT_FALSE(!c1.Bind(c1_cb, "", 123));
  EXPECT_FALSE(!c2.Bind(c2_cb, "", 123));
  // c3 binds to a different port and thus should receive nothing
  EXPECT_FALSE(!c3.Bind(c3_cb, "", 124));

  std::vector<std::string> sentData;
  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello clients, " << i;
    broadcaster.SendTo(oss.str(), kBroadcast, 123);
    sentData.emplace_back(oss.str());
    broker.DispatchMessages();
  }

  ASSERT_EQ(2u * sentData.size(), receivedData1.size());
  ASSERT_EQ(sentData.size(), receivedData2.size());

  for (size_t i = 0; i < sentData.size(); ++i)
  {
    EXPECT_EQ(sentData[i], receivedData1[2*i]);
    EXPECT_EQ(sentData[i], receivedData1[2*i + 1]);
    EXPECT_EQ(sentData[i], receivedData2[i]);
  }
}

// multicast is broken in ignition-only setup, because it advertises a service
// called "/multicast" in each client... obviously, this can't work with more
// than one client... fortunately, in case SubtRosRelay is used, this relay
// is the only client that "physically" interacts with the ignition layer, so
// that should work
// this could be fixed by rewriting the ignition layer to work via messages
// rather than services
#if 0
TEST(broker, multicast)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  CommsClient broadcaster("b", false, true, false);
  CommsClient c1("1", false, true, false);
  CommsClient c2("2", false , true, false);
  CommsClient c3("3", false , true, false);

  std::vector<std::string> receivedData1, receivedData2;
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

  // intentionally bind c1 twice, but once to multicast address and once on
  // unicast
  EXPECT_FALSE(!c1.Bind(c1_cb, kMulticast, 123));
  EXPECT_FALSE(!c1.Bind(c1_cb, "", 123));
  EXPECT_FALSE(!c2.Bind(c2_cb, kMulticast, 123));
  EXPECT_FALSE(!c3.Bind(c3_cb, "", 123));

  std::vector<std::string> sentData;
  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello clients, " << i;
    broadcaster.SendTo(oss.str(), kMulticast, 123);
    sentData.emplace_back(oss.str());
    broker.DispatchMessages();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  ASSERT_EQ(sentData.size(), receivedData1.size());
  ASSERT_EQ(sentData.size(), receivedData2.size());

  for (size_t i = 0; i < sentData.size(); ++i)
  {
    EXPECT_EQ(sentData[i], receivedData1[i]);
    EXPECT_EQ(sentData[i], receivedData2[i]);
  }
}
#endif

TEST(broker, unicast)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  CommsClient broadcaster("b", false, true, false);
  CommsClient c1("1", false, true, false);
  CommsClient c2("2", false , true, false);
  CommsClient c3("3", false , true, false);

  std::vector<std::string> receivedData1, receivedData2;
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
  EXPECT_FALSE(!c1.Bind(c1_cb, "", 123));
  EXPECT_FALSE(!c1.Bind(c1_cb, "", 123));
  EXPECT_FALSE(!c2.Bind(c2_cb, "", 123));
  // c3 binds to a different port and thus should receive nothing
  EXPECT_FALSE(!c3.Bind(c3_cb, "", 124));

  std::vector<std::string> sentData;
  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello clients, " << i;
    broadcaster.SendTo(oss.str(), "1", 123);
    sentData.emplace_back(oss.str());
    broker.DispatchMessages();
  }

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

  for(unsigned int i=0; i < 15; ++i)
  {
    std::ostringstream oss;
    oss << "Hello clients, " << i;
    broadcaster.SendTo(oss.str(), "2", 123);
    sentData.emplace_back(oss.str());
    broker.DispatchMessages();
  }

  ASSERT_EQ(0u, receivedData1.size());
  ASSERT_EQ(sentData.size(), receivedData2.size());

  for (size_t i = 0; i < sentData.size(); ++i)
  {
    EXPECT_EQ(sentData[i], receivedData2[i]);
  }
}

TEST(broker, notTwoClientsForSameAddressWithIgnition)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  CommsClient c1("1", false, true, false);
  CommsClient c2("1", false, true, false);

  auto cb = [](const std::string& src, const std::string& dst,
    const uint32_t port, const std::string& data)
  {
  };

  EXPECT_LT(0u, c1.Bind(cb).size());
  // this should be 0 because the ign service /1 has already been advertised
  // that doesn't happen due to
  // https://github.com/ignitionrobotics/ign-transport/issues/217
  // EXPECT_EQ(0u, c2.Bind(cb).size());
}

TEST(brokerUnit, registerOnce)
{
  Broker broker;
  broker.Start();

  ClientID client1, client2;

  EXPECT_EQ(0, broker.Team()->size());
  EXPECT_NE(invalidClientId, client1 = broker.Register("1"));
  EXPECT_EQ(1, broker.Team()->size());
  EXPECT_NE(invalidClientId, client2 = broker.Register("2"));
  EXPECT_EQ(2, broker.Team()->size());

  EXPECT_NE(client1, client2);

  EXPECT_FALSE(broker.Unregister(3));
  EXPECT_EQ(2, broker.Team()->size());

  EXPECT_TRUE(broker.Unregister(client1));
  EXPECT_EQ(1, broker.Team()->size());
  EXPECT_FALSE(broker.Unregister(client1));
  EXPECT_EQ(1, broker.Team()->size());
  EXPECT_TRUE(broker.Unregister(client2));
  EXPECT_EQ(0, broker.Team()->size());
}

TEST(brokerUnit, registerTwice)
{
  Broker broker;
  broker.Start();

  ClientID client11, client12, client21, client22;

  EXPECT_EQ(0, broker.Team()->size());
  EXPECT_NE(invalidClientId, client11 = broker.Register("1"));
  EXPECT_EQ(1, broker.Team()->size());
  EXPECT_NE(invalidClientId, client12 = broker.Register("1"));
  EXPECT_EQ(1, broker.Team()->size());
  EXPECT_NE(invalidClientId, client21 = broker.Register("2"));
  EXPECT_EQ(2, broker.Team()->size());
  EXPECT_NE(invalidClientId, client22 = broker.Register("2"));
  EXPECT_EQ(2, broker.Team()->size());

  EXPECT_NE(client11, client22);
  EXPECT_NE(client12, client21);
  EXPECT_NE(client12, client22);
  EXPECT_NE(client21, client22);

  EXPECT_TRUE(broker.Unregister(client11));
  EXPECT_EQ(2, broker.Team()->size());
  EXPECT_FALSE(broker.Unregister(8));
  EXPECT_EQ(2, broker.Team()->size());
  EXPECT_TRUE(broker.Unregister(client21));
  EXPECT_EQ(2, broker.Team()->size());
  EXPECT_TRUE(broker.Unregister(client22));
  EXPECT_EQ(1, broker.Team()->size());
  EXPECT_TRUE(broker.Unregister(client12));
  EXPECT_EQ(0, broker.Team()->size());
  EXPECT_FALSE(broker.Unregister(client11));
}

TEST(brokerUnit, registrationWorks)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  ASSERT_TRUE(broker.Team()->empty());
  ASSERT_TRUE(broker.endpoints.empty());

  ClientID client1;
  ASSERT_NE(invalidClientId, client1 = broker.Register("1"));

  ASSERT_EQ(1u, broker.Team()->size());
  ASSERT_TRUE(broker.endpoints.empty());
  ASSERT_NE(broker.Team()->end(), broker.Team()->find("1"));
  EXPECT_EQ("1", broker.Team()->at("1")->address);
  EXPECT_EQ("1", broker.Team()->at("1")->name);

  ClientID client2;
  ASSERT_NE(invalidClientId, client2 = broker.Register("2"));

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_TRUE(broker.endpoints.empty());
  ASSERT_NE(broker.Team()->end(), broker.Team()->find("2"));
  EXPECT_EQ("2", broker.Team()->at("2")->address);
  EXPECT_EQ("2", broker.Team()->at("2")->name);

  EndpointID endpoint11;
  ASSERT_NE(invalidEndpointId, endpoint11 = broker.Bind(client1, "1:1"));

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(1u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_EQ(1u, broker.endpoints.at("1:1").size());
  EXPECT_EQ("1", broker.endpoints.at("1:1")[0].address);

  EndpointID endpoint12;
  ASSERT_NE(invalidEndpointId, endpoint12 = broker.Bind(client1, "1:2"));
  EXPECT_NE(endpoint12, endpoint11);

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:2"));
  ASSERT_EQ(1u, broker.endpoints.at("1:2").size());
  EXPECT_EQ("1", broker.endpoints.at("1:2")[0].address);

  EndpointID endpoint21;
  ASSERT_NE(invalidEndpointId, endpoint21 = broker.Bind(client2, "2:1"));
  EXPECT_NE(endpoint12, endpoint21);

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(3u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  ASSERT_EQ(1u, broker.endpoints.at("2:1").size());
  EXPECT_EQ("2", broker.endpoints.at("2:1")[0].address);

  EndpointID endpoint22;
  ASSERT_NE(invalidEndpointId, endpoint22 = broker.Bind(client2, "2:2"));
  EXPECT_NE(endpoint22, endpoint11);

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(4u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:2"));
  ASSERT_EQ(1u, broker.endpoints.at("2:2").size());
  EXPECT_EQ("2", broker.endpoints.at("2:2")[0].address);

  // The broker library doesn't check whether endpoints and client addresses
  // match
  EndpointID endpointWeird;
  ASSERT_NE(invalidEndpointId, endpointWeird = broker.Bind(client1, "2:2"));
  EXPECT_NE(endpoint22, endpointWeird);

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(4u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:2"));
  ASSERT_EQ(2u, broker.endpoints.at("2:2").size());
  EXPECT_EQ("1", broker.endpoints.at("2:2")[1].address);

  // make sure broadcast endpoints work
  EndpointID endpointBcast11;
  ASSERT_NE(invalidEndpointId,
            endpointBcast11 = broker.Bind(client1, "broadcast:1"));
  EXPECT_NE(endpointWeird, endpointBcast11);

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(5u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("broadcast:1"));
  ASSERT_EQ(1u, broker.endpoints.at("broadcast:1").size());
  EXPECT_EQ("1", broker.endpoints.at("broadcast:1")[0].address);

  EndpointID endpointBcast21;
  ASSERT_NE(invalidEndpointId,
            endpointBcast21 = broker.Bind(client2, "broadcast:1"));
  EXPECT_NE(endpointBcast21, endpointBcast11);

  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(5u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("broadcast:1"));
  ASSERT_EQ(2u, broker.endpoints.at("broadcast:1").size());
  EXPECT_EQ("2", broker.endpoints.at("broadcast:1")[1].address);

  // test multiple binds to the same endpoint from the same client
  EndpointID endpointBcast111;
  ASSERT_NE(invalidEndpointId,
            endpointBcast111 = broker.Bind(client1, "broadcast:1"));
  EXPECT_NE(endpointBcast21, endpointBcast111);

  EXPECT_NE(endpointBcast11, endpointBcast111);
  ASSERT_EQ(2u, broker.Team()->size());
  ASSERT_EQ(5u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("broadcast:1"));
  ASSERT_EQ(2u, broker.endpoints.at("broadcast:1").size());
  EXPECT_EQ("1", broker.endpoints.at("broadcast:1")[0].address);
}

TEST(brokerUnit, unbind)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  ClientID client11 = broker.Register("1");
  ClientID client2 = broker.Register("2");
  ClientID client12 = broker.Register("1");

  ASSERT_NE(invalidClientId, client11);
  ASSERT_NE(invalidClientId, client2);
  ASSERT_NE(invalidClientId, client12);

  EndpointID client11_ep11_1 = broker.Bind(client11, "1:1");
  EndpointID client11_ep11_2 = broker.Bind(client11, "1:1");
  EndpointID client12_ep11_1 = broker.Bind(client12, "1:1");
  EndpointID client12_ep11_2 = broker.Bind(client12, "1:1");
  EndpointID client2_ep21_1 = broker.Bind(client2, "2:1");
  EndpointID client2_ep21_2 = broker.Bind(client2, "2:1");
  EndpointID client2_ep11_1 = broker.Bind(client2, "1:1");

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(1u, broker.endpoints["2:1"].size());

  EXPECT_TRUE(broker.Unbind(client2_ep21_1));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(1u, broker.endpoints["2:1"].size());

  // cannot unbind it twice
  EXPECT_FALSE(broker.Unbind(client2_ep21_1));

  EXPECT_TRUE(broker.Unbind(client2_ep21_2));

  // the endpoint key remains, but it is empty
  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());

  EXPECT_TRUE(broker.Unbind(client11_ep11_1));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());

  EXPECT_TRUE(broker.Unbind(client11_ep11_2));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());

  EXPECT_TRUE(broker.Unbind(client2_ep11_1));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(1u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());

  EXPECT_TRUE(broker.Unbind(client12_ep11_1));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(1u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());

  EXPECT_TRUE(broker.Unbind(client12_ep11_2));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(0u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());

  EXPECT_FALSE(broker.Unbind(client11_ep11_1));
  EXPECT_FALSE(broker.Unbind(client11_ep11_2));
  EXPECT_FALSE(broker.Unbind(client12_ep11_1));
  EXPECT_FALSE(broker.Unbind(client12_ep11_2));
  EXPECT_FALSE(broker.Unbind(client2_ep11_1));
  EXPECT_FALSE(broker.Unbind(client2_ep21_1));
  EXPECT_FALSE(broker.Unbind(client2_ep21_2));
}


TEST(brokerUnit, unregisterUnbinds)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  ClientID client11 = broker.Register("1");
  ClientID client2 = broker.Register("2");
  ClientID client12 = broker.Register("1");

  ASSERT_NE(invalidClientId, client11);
  ASSERT_NE(invalidClientId, client2);
  ASSERT_NE(invalidClientId, client12);

  EndpointID client11_ep11_1 = broker.Bind(client11, "1:1");
  EndpointID client11_ep11_2 = broker.Bind(client11, "1:1");
  EndpointID client12_ep11_1 = broker.Bind(client12, "1:1");
  EndpointID client12_ep11_2 = broker.Bind(client12, "1:1");
  EndpointID client2_ep21_1 = broker.Bind(client2, "2:1");
  EndpointID client2_ep21_2 = broker.Bind(client2, "2:1");
  EndpointID client2_ep11_1 = broker.Bind(client2, "1:1");

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(1u, broker.endpoints["2:1"].size());
  EXPECT_NE(broker.Team()->end(), broker.Team()->find("1"));
  EXPECT_NE(broker.Team()->end(), broker.Team()->find("2"));

  EXPECT_TRUE(broker.Unregister(client12));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(2u, broker.endpoints["1:1"].size());
  EXPECT_EQ(1u, broker.endpoints["2:1"].size());
  EXPECT_NE(broker.Team()->end(), broker.Team()->find("1"));
  EXPECT_NE(broker.Team()->end(), broker.Team()->find("2"));

  EXPECT_TRUE(broker.Unregister(client11));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(1u, broker.endpoints["1:1"].size());
  EXPECT_EQ(1u, broker.endpoints["2:1"].size());
  EXPECT_EQ(broker.Team()->end(), broker.Team()->find("1"));
  EXPECT_NE(broker.Team()->end(), broker.Team()->find("2"));

  EXPECT_TRUE(broker.Unregister(client2));

  ASSERT_EQ(2u, broker.endpoints.size());
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("1:1"));
  ASSERT_NE(broker.endpoints.end(), broker.endpoints.find("2:1"));
  EXPECT_EQ(0u, broker.endpoints["1:1"].size());
  EXPECT_EQ(0u, broker.endpoints["2:1"].size());
  EXPECT_EQ(broker.Team()->end(), broker.Team()->find("1"));
  EXPECT_EQ(broker.Team()->end(), broker.Team()->find("2"));

  EXPECT_FALSE(broker.Unregister(client11));
  EXPECT_FALSE(broker.Unregister(client12));
  EXPECT_FALSE(broker.Unregister(client2));

  EXPECT_FALSE(broker.Unbind(client11_ep11_1));
  EXPECT_FALSE(broker.Unbind(client11_ep11_2));
  EXPECT_FALSE(broker.Unbind(client12_ep11_1));
  EXPECT_FALSE(broker.Unbind(client12_ep11_2));
  EXPECT_FALSE(broker.Unbind(client2_ep11_1));
  EXPECT_FALSE(broker.Unbind(client2_ep21_1));
  EXPECT_FALSE(broker.Unbind(client2_ep21_2));
}

TEST(brokerUnit, sendRequiresRegisterAndBind)
{
  Broker broker;
  setDummyComms(broker);

  broker.Start();

  ClientID client2;
  ASSERT_NE(invalidClientId, client2 = broker.Register("2"));

  subt::msgs::Datagram msg;
  msg.set_src_address("1");
  msg.set_dst_address("2");
  msg.set_dst_port(42);
  msg.set_rssi(-30);

  // sender has to be registered
  broker.OnMessage(msg);
  EXPECT_FALSE(broker.DispatchMessages());

  // endpoint has to be registered
  msg.set_src_address("2");
  msg.set_dst_address("1");
  broker.OnMessage(msg);
  EXPECT_FALSE(broker.DispatchMessages());

  // destination has to be registered
  ClientID client1;
  EXPECT_NE(invalidClientId, client1 = broker.Register("1"));
  broker.OnMessage(msg);
  EXPECT_FALSE(broker.DispatchMessages());

  // endpoint has to be bound
  ASSERT_NE(invalidEndpointId, broker.Bind(client1, "1:42"));
  broker.OnMessage(msg);
  EXPECT_TRUE(broker.DispatchMessages());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
