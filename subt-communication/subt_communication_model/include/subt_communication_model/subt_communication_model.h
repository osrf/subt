#pragma once

#include <subt_rf_interface/subt_rf_interface.h>

namespace subt
{
namespace communication_model
{

struct channel_configuration
{
  double capacity;         // Bits-per-second
  double default_tx_power; // dBm
  std::string modulation;       // E.g., QPSK
  double noise_floor;      // dBm
  rf_interface::pathloss_function pathloss_f; // Function for computing pathloss

  channel_configuration() :
      capacity(54000000),   // 54Mbps
      default_tx_power(27), // 27dBm or 500mW
      modulation("QPSK"),   // Quadrature Phase Shift Keyring
      noise_floor(-90)      // dBm
  {}
};

bool attempt_send(const channel_configuration& channel,
                  const rf_interface::radio_state& tx_state,
                  const rf_interface::radio_state& rx_state,
                  const uint64_t& num_bytes
                  );
}

}
