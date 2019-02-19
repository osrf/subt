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

#include <subt_rf_interface/subt_rf_model.h>
#include <limits>

namespace subt
{
namespace rf_interface
{

namespace range_model
{

/////////////////////////////////////////////
rf_power distance_based_received_power(const double& tx_power,
                                       radio_state& tx_state,
                                       radio_state& rx_state,
                                       const rf_configuration& config)
{
  double range = tx_state.pose.Pos().Distance(rx_state.pose.Pos());

  if(config.max_range > 0.0 &&
     range > config.max_range) {
    return {-std::numeric_limits<double>::infinity(), 0.0};
  }

  return {tx_power, 0.0};
}

/////////////////////////////////////////////
rf_power log_normal_received_power(const double& tx_power,
                                   radio_state& tx_state,
                                   radio_state& rx_state,
                                   const rf_configuration& config)
{
  double range = tx_state.pose.Pos().Distance(rx_state.pose.Pos());

  if(config.max_range > 0.0 &&
     range > config.max_range) {
    return {-std::numeric_limits<double>::infinity(), 0.0};
  }

  double PL = config.L0 + 10 * config.fading_exponent * log10(range);
  
  return {tx_power - PL, config.sigma};
}

}
}
}
