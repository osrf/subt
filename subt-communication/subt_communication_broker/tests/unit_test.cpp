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

#include <subt_communication_broker/subt_communication_broker.h>
#include <subt_communication_broker/subt_communication_client.h>
#include <subt_communication_model/subt_communication_model.h>
#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>

using namespace subt;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;
using namespace subt::communication_broker;

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

  auto pose_update_func = [](const std::string& name) {
    static ros::Time now;
    now += ros::Duration(1.0);
    if(name == "1") {
      return std::make_tuple(true, ignition::math::Pose3<double>(0, 0, 0, 0, 0, 0), now);
    }

    if(name == "2") {
      static double x = 0;
      x += 1.0;
      std::cout << "Moving (2) to x=" << x << std::endl;
      return std::make_tuple(true, ignition::math::Pose3<double>(x, 0, 0, 0, 0, 0), now);
    }

    return std::make_tuple(false, ignition::math::Pose3<double>(), ros::Time());
  };

  broker.SetPoseUpdateFunction(pose_update_func);
  broker.Start();

  CommsClient c1("1");
  CommsClient c2("2");

  auto c2_cb = [=](const std::string& src,
                   const std::string& dst,
                   const uint32_t port,
                   const std::string& data) {
    std::cout << "Received " << data.size() << "(" << data << ") bytes from " << src << std::endl;
  };

  c2.Bind(c2_cb);

  for(unsigned int i=0; i < 15; ++i) {
    std::ostringstream oss;
    oss << "Hello c2, " << i;
    c1.SendTo(oss.str(), "2");
    broker.DispatchMessages();
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


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

