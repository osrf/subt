#pragma once

#include <ignition/common.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <subt_gazebo_los_model/VisibilityTable.hh>
#include <subt_rf_interface/subt_rf_interface.h>

#include <ros/console.h>

namespace subt
{

namespace rf_interface
{

namespace visibility_model
{

class VisibilityModel
{
 public:
  VisibilityModel();

  rf_power compute_received_power(const double& tx_power,
                                  radio_state& tx_state,
                                  radio_state& rx_state);

  bool VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                           ignition::msgs::Boolean &_rep);

 private:

  ignition::transport::Node node;
  subt::VisibilityTable visibilityTable;
  gazebo::physics::WorldPtr world;

  double commsCostMax = 15.0;
  
};

}
}
}
