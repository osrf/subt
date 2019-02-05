#pragma once

#include <subt_rf_interface/subt_rf_interface.h>

namespace subt
{
namespace rf_interface
{
namespace range_model
{

/// \struct rf_configuration
/// \brief Physical layer radio parameterization
///
/// Parameters for simple log-normal fading model
struct rf_configuration
{
  double max_range;        ///< Hard limit on range
  double fading_exponent;  ///< Fading exponent
  double L0;               ///< Received power at 1m (in dBm)
  double sigma;            ///< Standard deviation for received power
  rf_configuration() :
      max_range(50.0),
      fading_exponent(2.5),
      L0(40),
      sigma(10)
  { }
};

/// Output stream operator
/// @param oss Stream
/// @param config configuration to output
std::ostream& operator<<(std::ostream& oss, const rf_configuration& config)
{
  oss << "RF Configuration (range-based)" << std::endl
      << "-- max_range: " << config.max_range << std::endl
      << "-- fading_exponent: " << config.fading_exponent << std::endl
      << "-- L0: " << config.L0 << std::endl
      << "-- sigma: " << config.sigma << std::endl;

  return oss;
}

/// Compute distance between two points.
/// @param a Point a
/// @param b Point b
double distance(const geometry_msgs::Point& a,
                const geometry_msgs::Point& b);

/// Compute received power based on distance.
///
/// Compute the pathloss based on distance between two nodes and
/// return the received power.
/// 
/// @param tx_power Transmit power (dBm)
/// @param tx_state Transmit state (pose)
/// @param rx_state Receiver state (pose)
/// @param config Physical-layer configuration
rf_power distance_based_received_power(const double& tx_power,
                                       radio_state& tx_state,
                                       radio_state& rx_state,
                                       const rf_configuration& config);

/// Compute received power based on distance.
///
/// Compute the pathloss based on distance between two nodes and
/// return the received power. Vary return by drawing from normal
/// distribution.
/// 
/// @param tx_power Transmit power (dBm)
/// @param tx_state Transmit state (pose)
/// @param rx_state Receiver state (pose)
/// @param config Physical-layer configuration
rf_power log_normal_received_power(const double& tx_power,
                                   radio_state& tx_state,
                                   radio_state& rx_state,
                                   const rf_configuration& config);
}
}
}
