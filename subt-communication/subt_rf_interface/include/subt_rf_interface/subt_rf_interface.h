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

#ifndef SUBT_RF_INTERFACE__SUBT_RF_INTERFACE_H_
#define SUBT_RF_INTERFACE__SUBT_RF_INTERFACE_H_

#include <functional>
#include <list>
#include <ignition/math/Pose3.hh>

namespace subt
{

namespace rf_interface
{

/// \struct radio_state
/// \brief Store radio state
///
/// Structure to hold radio state including the pose and book-keeping
/// necessary to implement bitrate limits.
struct radio_state
{
  double update_stamp;      ///< Timestamp of last update
  ignition::math::Pose3<double> pose;  ///< Pose of the radio
  std::list<std::pair<double, uint64_t>> bytes_sent; ///< Recent
                                                        ///sent packet
                                                        ///history
  uint64_t bytes_sent_this_epoch = 0; ///< Accumulation of bytes sent
                                      /// in an epoch
  std::list<std::pair<double, uint64_t>> bytes_received; ///<
                                                            ///Recent
                                                            ///received
                                                            ///packet
                                                            ///history
  uint64_t bytes_received_this_epoch = 0;  ///< Accumulation of bytes
                                           /// received in an epoch
  double antenna_gain;      ///< Isotropic antenna gain
};
/// \struct rf_power
///
/// \brief Type for holding RF power as a Normally distributed random
/// variable
struct rf_power
{
  double mean; ///< Expected value of RF power
  double variance; ///< Variance of RF power
  operator double() { return mean; }
};

/// Function signature for computing pathloss.
typedef std::function<rf_power(const double&, // tx_power
                               radio_state&, // tx_state
                               radio_state&,  //rx_state
                               bool& // using breadcumbs?
                               )> pathloss_function;


}
}
#endif

