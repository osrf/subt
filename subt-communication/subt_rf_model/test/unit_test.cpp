#include <string>
#include <gtest/gtest.h>

#include <subt_rf_model/subt_rf_model.h>

using namespace subt;
using namespace subt::rf_model;

TEST(range_based, co_located)
{
  struct rf_configuration config;

  geometry_msgs::PoseStamped a, b;
  a.header.frame_id = "world";

  b = a;
  double tx_power = 10.0;
  ASSERT_DOUBLE_EQ(
      distance_based_received_power(tx_power,
                                    {a, 0},
                                    {b, 1}, config), tx_power);
}

TEST(range_based, under_range)
{
  struct rf_configuration config;
  config.max_range = 1.0;

  geometry_msgs::PoseStamped a, b;
  a.header.frame_id = "world";
  b.header.frame_id = "world";

  double tx_power = 10.0;

  // Test within range, no path loss
  b.pose.position.x = 0.5;
  ASSERT_DOUBLE_EQ(
      distance_based_received_power(tx_power,
                                    {a, 0},
                                    {b, 1}, config), tx_power);
}

TEST(range_based, over_range)
{
  struct rf_configuration config;
  config.max_range = 1.0;

  geometry_msgs::PoseStamped a, b;
  a.header.frame_id = "world";
  b.header.frame_id = "world";

  double tx_power = 10.0;

  // Test outside range, full loss
  b.pose.position.x = 2.0;
  ASSERT_DOUBLE_EQ(
      distance_based_received_power(tx_power,
                                    {a, 0},
                                    {b, 1}, config), 0.0);  
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

