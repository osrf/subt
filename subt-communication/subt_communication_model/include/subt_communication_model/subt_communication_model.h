#pragma once

#include <subt_rf_interface/subt_rf_interface.h>

namespace subt
{
namespace communication_model
{

struct radio_configuration
{
  double capacity;         // Bits-per-second
  double default_tx_power; // dBm
  std::string modulation;       // E.g., QPSK
  double noise_floor;      // dBm
  rf_interface::pathloss_function pathloss_f; // Function for computing pathloss

  radio_configuration() :
      capacity(54000000),   // 54Mbps
      default_tx_power(27), // 27dBm or 500mW
      modulation("QPSK"),   // Quadrature Phase Shift Keyring
      noise_floor(-90)      // dBm
  {}
};


std::ostream& operator<<(std::ostream& oss, const radio_configuration& config)
{
  oss << "Radio Configuration" << std::endl
      << "-- capacity: " << config.capacity << std::endl
      << "-- default_tx_power: " << config.default_tx_power << std::endl
      << "-- noise_floor: " << config.noise_floor << std::endl
      << "-- modulation: " << config.modulation << std::endl;

  return oss;
}

bool attempt_send(const radio_configuration& radio,
                  const rf_interface::radio_state& tx_state,
                  const rf_interface::radio_state& rx_state,
                  const uint64_t& num_bytes
                  );

typedef std::function<bool(const radio_configuration&,
                           const rf_interface::radio_state&,
                           const rf_interface::radio_state&,
                           const uint64_t&)> communication_function;

}


}
