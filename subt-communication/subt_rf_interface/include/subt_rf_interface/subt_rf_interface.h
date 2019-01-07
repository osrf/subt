#pragma once

#include <geometry_msgs/PoseStamped.h>

namespace subt
{

namespace rf_interface
{

struct radio_state
{
  geometry_msgs::PoseStamped pose; // Location
  double antenna_gain;      // Isotropic antenna gain
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
