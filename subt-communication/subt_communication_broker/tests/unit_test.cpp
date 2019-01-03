#include <string>
#include <gtest/gtest.h>

#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>

#include <subt_communication_model/subt_communication_model.h>

#include <subt_communication_broker/subt_communication_broker.h>
#include <subt_communication_broker/subt_communication_client.h>

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
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
      
    if(name == "1") {
      p.pose.position.x = 0;
      return std::make_tuple(true, p);
    }

    if(name == "2") {
      static double x = 0;
      p.pose.position.x = x;
      x += 1.0;
      std::cout << "Moving (2) to x=" << x << std::endl;
      return std::make_tuple(true, p);
    }

    return std::make_tuple(false, geometry_msgs::PoseStamped());
  };

  broker.SetPoseUpdateFunction(pose_update_func);

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

