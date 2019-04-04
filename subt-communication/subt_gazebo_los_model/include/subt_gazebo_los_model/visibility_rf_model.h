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

#ifndef SUBT_GAZEBO_LOS_MODEL__VISIBILITY_RF_MODEL_H_
#define SUBT_GAZEBO_LOS_MODEL__VISIBILITY_RF_MODEL_H_

#include <ignition/common.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <subt_gazebo_los_model/VisibilityTable.hh>
#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>

#include <ros/console.h>

namespace subt
{

namespace rf_interface
{

namespace visibility_model
{

/// \struct rf_configuration
/// \brief RF configuration for LOS model
struct rf_configuration
{
  double visibility_cost_to_fading_exponent; ///< Fraction to increase
                                             ///fading exponent for
                                             ///each unit of
                                             ///visibility cost.
  double comms_cost_max; ///< Maximum comms cost to consider (saturate
                         ///here)
  rf_configuration() :
      visibility_cost_to_fading_exponent(0.2),
      comms_cost_max(15.0)
  { }
};

/// Output stream operator.
///
/// @param oss Stream
/// @param config RF Configuration to output
std::ostream& operator<<(std::ostream& oss, const rf_configuration& config)
{
  oss << "RF Configuration (visibility-based)" << std::endl
      << "-- visibility_cost_to_fading_exponent: "
      << config.visibility_cost_to_fading_exponent << std::endl
      << "-- comms_cost_max: "
      << config.comms_cost_max << std::endl;
  
  return oss;
}

/// \class VisibilityModel
/// \brief Maintain state of the visibility model.
///
/// The visibility model loads a pre-computed lookup table to
/// determine the heuristic "cost" to communicate between SUBT
/// environment tiles. This heuristic cost is used to adjust a fading
/// exponent for a typical log-normal fading pathloss model.
class VisibilityModel
{
 public:
  VisibilityModel(visibility_model::rf_configuration _visibility_config,
                  range_model::rf_configuration _range_config);

  /// Compute received power function that will be given to
  /// communcation model.
  ///
  /// @param tx_power Transmit power (dBm)
  /// @param tx_state Transmitter state
  /// @param rx_state Receiver state
  rf_power compute_received_power(const double& tx_power,
                                  radio_state& tx_state,
                                  radio_state& rx_state);

  /// Function to visualize visibility cost in Gazebo.
  bool VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                           ignition::msgs::Boolean &_rep);

 private:

  ignition::transport::Node node;
  subt::VisibilityTable visibilityTable;
  gazebo::physics::WorldPtr world;

  visibility_model::rf_configuration visibility_config;
  range_model::rf_configuration default_range_config;
  
};

}
}
}
#endif

