#pragma once

#include <geometry_msgs/PoseStamped.h>

namespace subt
{

namespace rf_interface
{

struct radio_state
{
  geometry_msgs::PoseStamped pose; // Location
  std::list<std::pair<ros::Time, uint64_t>> bytes_sent;
  uint64_t bytes_sent_this_epoch;
  std::list<std::pair<ros::Time, uint64_t>> bytes_received;
  uint64_t bytes_received_this_epoch;  
  double antenna_gain;      // Isotropic antenna gain
};

struct rf_power
{
  double mean;
  double variance;
  operator double() { return mean; }
};


typedef std::function<rf_power(const double&, // tx_power
                               radio_state&, // tx_state
                               radio_state&  //rx_state
                               )> pathloss_function;


}
}
