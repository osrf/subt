#include <string>
#include <gtest/gtest.h>

#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>

#include <limits>

using namespace subt;
using namespace subt::rf_interface;
using namespace subt::rf_interface::range_model;

TEST(range_based, co_located)
{
  struct rf_configuration config;

  rf_interface::radio_state tx, rx;
  tx.pose.header.frame_id = "world";
  rx.pose.header.frame_id = "world";

  double tx_power = 10.0;
  ASSERT_DOUBLE_EQ(
      distance_based_received_power(tx_power,
                                    tx, rx, config), tx_power);
}

TEST(range_based, under_range)
{
  struct rf_configuration config;
  config.max_range = 1.0;

  rf_interface::radio_state tx, rx;
  tx.pose.header.frame_id = "world";
  rx.pose.header.frame_id = "world";
  
  double tx_power = 10.0;

  // Test within range, no path loss
  rx.pose.pose.position.x = 0.5;
  ASSERT_DOUBLE_EQ(
      distance_based_received_power(tx_power,
                                    tx, rx, config), tx_power);
}

TEST(range_based, over_range)
{
  struct rf_configuration config;
  config.max_range = 1.0;

  rf_interface::radio_state tx, rx;
  tx.pose.header.frame_id = "world";
  rx.pose.header.frame_id = "world";

  double tx_power = 10.0;

  // Test outside range, full loss
  rx.pose.pose.position.x = 2.0;
  ASSERT_DOUBLE_EQ(
      distance_based_received_power(tx_power,
                                    tx, rx, config),
      -std::numeric_limits<double>::infinity());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

