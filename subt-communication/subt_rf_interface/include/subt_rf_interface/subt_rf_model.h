#pragma once

#include <subt_rf_interface/subt_rf_interface.h>

namespace subt
{
namespace rf_interface
{
namespace range_model
{

struct rf_configuration
{
  double max_range;
  double fading_exponent;  
  double L0;               // PL at 1m
  double sigma;
  rf_configuration() :
      max_range(50.0),
      fading_exponent(2.5),
      L0(40),
      sigma(10)
  { }
};

std::ostream& operator<<(std::ostream& oss, const rf_configuration& config)
{
  oss << "RF Configuration (range-based)" << std::endl
      << "-- max_range: " << config.max_range << std::endl
      << "-- fading_exponent: " << config.fading_exponent << std::endl
      << "-- L0: " << config.L0 << std::endl
      << "-- sigma: " << config.sigma << std::endl;

  return oss;
}

double distance(const geometry_msgs::Point& a,
                const geometry_msgs::Point& b);

rf_power distance_based_received_power(const double& tx_power,
                                       radio_state& tx_state,
                                       radio_state& rx_state,
                                       const rf_configuration& config);

rf_power log_normal_received_power(const double& tx_power,
                                   radio_state& tx_state,
                                   radio_state& rx_state,
                                   const rf_configuration& config);
}
}
}
