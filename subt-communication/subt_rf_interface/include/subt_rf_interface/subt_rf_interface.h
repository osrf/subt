#pragma once

#include <geometry_msgs/PoseStamped.h>

namespace subt
{

namespace rf_interface
{

struct radio_state
{
  geometry_msgs::PoseStamped pose; // Location
  uint64_t id;              // Unique radio ID
  double antenna_gain;      // Isotropic antenna gain

};

struct rf_configuration
{
  double max_range;
  rf_configuration() : max_range(10.0) { }
};

struct rf_power
{
  double mean;
  double variance;
  operator double() { return mean; }
};


typedef std::function<rf_power(const double&, // tx_power
                               const radio_state&, // tx_state
                               const radio_state&  //rx_state
                               )> pathloss_function;


}
}
