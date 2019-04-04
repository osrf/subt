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

#ifndef SUBT_COMMUNICATION_MODEL__SUBT_COMMUNICATION_MODEL_H_
#define SUBT_COMMUNICATION_MODEL__SUBT_COMMUNICATION_MODEL_H_

#include <subt_rf_interface/subt_rf_interface.h>

namespace subt
{
namespace communication_model
{

/// \struct radio_configuration
/// \brief Radio configuration parameters.
/// 
/// In addition to static parameters such as channel capacity and
/// default transmit power, this structure holds a function which can
/// be called to compute the pathloss between two antenna poses.
struct radio_configuration
{
  double capacity;         ///< Capacity of radio in bits-per-second
  double default_tx_power; ///< Default transmit power in dBm
  std::string modulation;  ///< Modulation scheme, e.g., QPSK
  double noise_floor;      ///< Noise floor of the radio in dBm
  rf_interface::pathloss_function pathloss_f; ///< Function handle for
                                              ///computing pathloss

  radio_configuration() :
      capacity(54000000),   // 54Mbps
      default_tx_power(27), // 27dBm or 500mW
      modulation("QPSK"),   // Quadrature Phase Shift Keyring
      noise_floor(-90)      // dBm
  {}
};

/// Output stream operator.
/// @param oss Stream
/// @param config configuration to output
std::ostream& operator<<(std::ostream& oss, const radio_configuration& config)
{
  oss << "Radio Configuration" << std::endl
      << "-- capacity: " << config.capacity << std::endl
      << "-- default_tx_power: " << config.default_tx_power << std::endl
      << "-- noise_floor: " << config.noise_floor << std::endl
      << "-- modulation: " << config.modulation << std::endl;

  return oss;
}

/// Attempt communication between two nodes.
///
/// The radio configuration, transmitter and receiver state, and
/// packet size are all used to compute the probability of successful
/// communication (i.e., based on both SNR and bitrate
/// limitations). This probability is then used to determine if the
/// packet is successfully communicated.
///
/// @param radio Static configuration for the radio
/// @param tx_state Current state of the transmitter (pose)
/// @param rx_state Current state of the receiver (pose)
/// @param num_bytes Size of the packet
/// @return bool reporting if the packet should be delivered
bool attempt_send(const radio_configuration& radio,
                  rf_interface::radio_state& tx_state,
                  rf_interface::radio_state& rx_state,
                  const uint64_t& num_bytes
                  );

/// Function signature for the communication model.
typedef std::function<bool(const radio_configuration&,
                           rf_interface::radio_state&,
                           rf_interface::radio_state&,
                           const uint64_t&)> communication_function;

}


}
#endif

