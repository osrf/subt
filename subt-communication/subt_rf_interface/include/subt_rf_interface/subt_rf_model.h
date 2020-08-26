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

#ifndef SUBT_RF_INTERFACE__SUBT_RF_MODEL_H_
#define SUBT_RF_INTERFACE__SUBT_RF_MODEL_H_

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
  double scaling_factor;   ///< Scaling factor
  double range_per_hop;    ///< Extra ranged added per breadcrumb
  rf_configuration() :
      max_range(50.0),
      fading_exponent(2.5),
      L0(40),
      sigma(10),
      scaling_factor(1.0),
      range_per_hop(2.0)
  { }

  /// Output stream operator
  /// @param oss Stream
  /// @param config configuration to output
  friend std::ostream& operator<<(std::ostream& oss,
                                  const rf_configuration& config)
  {
    oss << "RF Configuration (range-based)" << std::endl
        << "-- max_range: " << config.max_range << std::endl
        << "-- fading_exponent: " << config.fading_exponent << std::endl
        << "-- L0: " << config.L0 << std::endl
        << "-- sigma: " << config.sigma << std::endl
        << "-- scaling_factor: " << config.scaling_factor << std::endl
        << "-- range_per_hop: " << config.range_per_hop << std::endl;

    return oss;
  }
};

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

/// Compute received power based on distance.
///
/// Compute the pathloss based on distance between two nodes and
/// return the received power. Vary return by drawing from normal
/// distribution.
///
/// @param tx_power Transmit power (dBm)
/// @param range Greatest distance in a single hope (m)
/// @param num_hops Number of breadcrumbs crossed
/// @param config Physical-layer configuration
rf_power log_normal_v2_received_power(const double& tx_power,
                                      const double& range,
                                      const unsigned int& num_hops,
                                      const rf_configuration& config);

/// Compute received power based on visibility information only.
///
/// Compute the pathloss based on the visibility cost between two nodes and
/// return the received power.Vary return by drawing from normal
/// distribution.
///
/// @param tx_power Transmit power (dBm)
/// @param config Physical-layer configuration
rf_power visibility_only_received_power(const double& tx_power,
                                        const rf_configuration& config);
}
}
}
#endif
