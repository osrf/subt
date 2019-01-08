#include <ros/console.h>
#include <subt_communication_model/subt_communication_model.h>

#include <math.h>
#include <random>

namespace subt
{
namespace communication_model
{

inline double dbmToPow(double x) { return 0.001 * pow(10., x / 10.); }
inline double QPSKPowerToBER(double P, double N) { return erfc(sqrt(P / N)); }

bool packetSuccess(double ber, uint64_t size)
{

}


bool attempt_send(const radio_configuration& radio,
                  const rf_interface::radio_state& tx_state,
                  const rf_interface::radio_state& rx_state,
                  const uint64_t& num_bytes)
{
  // Do capacity checks here

  // Get the received power based on TX power and position of each
  // node
  auto rx_power_dist = radio.pathloss_f(radio.default_tx_power,
                                        tx_state,
                                        rx_state);

  double rx_power = rx_power_dist.mean;
  if(rx_power_dist.variance > 0.0) {

    static std::default_random_engine rndEngine;
    std::normal_distribution<> d{rx_power_dist.mean, sqrt(rx_power_dist.variance)};

    rx_power = d(rndEngine);
  }

  // Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER)
  double ber = 0.0;
  if(radio.modulation == "QPSK") {
    ber = QPSKPowerToBER( dbmToPow(rx_power),
                          dbmToPow(radio.noise_floor) );
  }
  else {
    ROS_WARN("Using unsupported modulation scheme!");
  }

  double packet_drop_prob = 1.0 - exp(num_bytes*log(1-ber));

  ROS_DEBUG_STREAM("TX power (dBm): " << radio.default_tx_power << "\n" <<
                   "RX power (dBm): " << rx_power << "\n" <<
                   "BER: " << ber << "\n" <<
                   "# Bytes: " << num_bytes << "\n" <<
                   "PER: " << packet_drop_prob);

  double rand_draw = (rand() % 1000) / 1000.0;
  return rand_draw > packet_drop_prob;  
}

}
}
