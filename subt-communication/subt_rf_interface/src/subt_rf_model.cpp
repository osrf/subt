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
