#include <string>
#include <gtest/gtest.h>

#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>

#include <subt_communication_model/subt_communication_model.h>

using namespace subt;
using namespace subt::communication_model;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;

TEST(range_based, co_located)
{
  // Build RF Function
  struct rf_configuration rf_config;
  rf_config.max_range = 10.0;
  auto rf_func = std::bind(&distance_based_received_power,
                           std::placeholders::_1,
                           std::placeholders::_2,
                           std::placeholders::_3,
                           rf_config);

  
  struct radio_configuration radio;
  radio.capacity = 54000000;
  radio.default_tx_power = 27;
  radio.modulation = "QPSK";
  radio.noise_floor = -90;
  radio.pathloss_f = rf_func;

  rf_interface::radio_state tx, rx;

  ASSERT_TRUE(attempt_send(radio,
                           tx,  // TX state
                           rx,  // RX state
                           1000)    // 1Kb packet
              );
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

