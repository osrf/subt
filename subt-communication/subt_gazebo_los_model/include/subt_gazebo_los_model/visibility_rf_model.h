#pragma once

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

struct rf_configuration
{
  //! Fraction to increase fading exponent for each unit of visibility
  //! cost.
  double visibility_cost_to_fading_exponent;
  double comms_cost_max;
  rf_configuration() :
      visibility_cost_to_fading_exponent(0.2),
      comms_cost_max(15.0)
  { }
};

std::ostream& operator<<(std::ostream& oss, const rf_configuration& config)
{
  oss << "RF Configuration (visibility-based)" << std::endl
      << "-- visibility_cost_to_fading_exponent: "
      << config.visibility_cost_to_fading_exponent << std::endl
      << "-- comms_cost_max: "
      << config.comms_cost_max << std::endl;
  
  return oss;
}

class VisibilityModel
{
 public:
  VisibilityModel(visibility_model::rf_configuration _visibility_config,
                  range_model::rf_configuration _range_config);

  rf_power compute_received_power(const double& tx_power,
                                  radio_state& tx_state,
                                  radio_state& rx_state);

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
