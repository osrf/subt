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

#include <ignition/common/Console.hh>
#include <subt_communication_model/subt_communication_model.h>

#include <math.h>
#include <random>
#include <limits>

namespace subt
{
namespace communication_model
{

/////////////////////////////////////////////
inline double dbmToPow(double x) { return 0.001 * pow(10., x / 10.); }

/////////////////////////////////////////////
inline double QPSKPowerToBER(double P, double N) { return erfc(sqrt(P / N)); }

/////////////////////////////////////////////
std::tuple<bool, double>
attempt_send(const radio_configuration& radio,
             rf_interface::radio_state& tx_state,
             rf_interface::radio_state& rx_state,
             const uint64_t& num_bytes)
{
  // Do capacity checks here
  static double epoch_duration = 1.0;

  double now = tx_state.update_stamp;

  // Maintain running window of bytes sent over the last epoch, e.g.,
  // 1s
  while(!tx_state.bytes_sent.empty() &&
        tx_state.bytes_sent.front().first < now - epoch_duration)
  {
    tx_state.bytes_sent_this_epoch -= tx_state.bytes_sent.front().second;
    tx_state.bytes_sent.pop_front();
  }

  // ignmsg << "bytes sent: " <<  tx_state.bytes_sent_this_epoch << " + "
  //        << num_bytes << " = "
  //        << tx_state.bytes_sent_this_epoch + num_bytes << std::endl;

  // Compute prospective accumulated bits along with time window
  // (including this packet)
  double bits_sent = (tx_state.bytes_sent_this_epoch + num_bytes)*8;

  // Check current epoch bitrate vs capacity and fail to send
  // accordingly
  if(bits_sent > radio.capacity*epoch_duration)
  {
    // ignwarn << "Bitrate limited: " << bits_sent << "bits sent (limit: "
    // << radio.capacity  * epoch_duration.toSec() << std::endl;
    return std::make_tuple(false, std::numeric_limits<double>::lowest());
  }

  // Record these bytes
  tx_state.bytes_sent.push_back(std::make_pair(now, num_bytes));
  tx_state.bytes_sent_this_epoch += num_bytes;

  // Get the received power based on TX power and position of each node
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
    ignwarn << "Using unsupported modulation scheme!";
  }

  double packet_drop_prob = 1.0 - exp(num_bytes*log(1-ber));

  // ignmsg << "TX power (dBm): " << radio.default_tx_power << "\n" <<
  //           "RX power (dBm): " << rx_power << "\n" <<
  //           "BER: " << ber << "\n" <<
  //           "# Bytes: " << num_bytes << "\n" <<
  //           "PER: " << packet_drop_prob << std::endl;

  double rand_draw = (rand() % 1000) / 1000.0;

  bool packet_received = rand_draw > packet_drop_prob;

  if(!packet_received)
    return std::make_tuple(false, std::numeric_limits<double>::lowest());

  // Maintain running window of bytes received over the last epoch, e.g.,
  // 1s
  while(!rx_state.bytes_received.empty() &&
        rx_state.bytes_received.front().first < now - epoch_duration)
  {
    rx_state.bytes_received_this_epoch -=
      rx_state.bytes_received.front().second;
    rx_state.bytes_received.pop_front();
  }

  // ignmsg << "bytes received: " << rx_state.bytes_received_this_epoch
  // << " + " << num_bytes
  // << " = " << rx_state.bytes_received_this_epoch + num_bytes << std::endl;

  // Compute prospective accumulated bits along with time window
  // (including this packet)
  double bits_received = (rx_state.bytes_received_this_epoch + num_bytes)*8;

  // Check current epoch bitrate vs capacity and fail to send
  // accordingly
  if(bits_received > radio.capacity*epoch_duration)
  {
    // ignwarn < <"Bitrate limited: " <<  bits_received
    // << "bits received (limit: " << radio.capacity * epoch_duration.toSec()
    // << )\n";
    return std::make_tuple(false, std::numeric_limits<double>::lowest());
  }

  // Record these bytes
  rx_state.bytes_received.push_back(std::make_pair(now, num_bytes));
  rx_state.bytes_received_this_epoch += num_bytes;

  return std::make_tuple(true, rx_power);
}

}
}
