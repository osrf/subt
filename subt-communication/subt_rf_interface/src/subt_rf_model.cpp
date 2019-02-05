#include <subt_rf_interface/subt_rf_model.h>
#include <limits>

namespace subt
{
namespace rf_interface
{

namespace range_model
{

/////////////////////////////////////////////
double distance(const geometry_msgs::Point& a,
                const geometry_msgs::Point& b)

{
  return sqrt( (a.x - b.x)*(a.x - b.x) +
               (a.y - b.y)*(a.y - b.y) +
               (a.z - b.z)*(a.z - b.z) );
}

/////////////////////////////////////////////
rf_power distance_based_received_power(const double& tx_power,
                                       radio_state& tx_state,
                                       radio_state& rx_state,
                                       const rf_configuration& config)
{
  // Ensure that TX and RX poses are in same frame. We don't want to
  // do TF transformations here since this function may be called
  // rapidly.
  assert(tx_state.pose.header.frame_id == rx_state.pose.header.frame_id);

  double range = distance(tx_state.pose.pose.position,
                          rx_state.pose.pose.position);

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
  // Ensure that TX and RX poses are in same frame. We don't want to
  // do TF transformations here since this function may be called
  // rapidly.
  assert(tx_state.pose.header.frame_id == rx_state.pose.header.frame_id);

  double range = distance(tx_state.pose.pose.position,
                          rx_state.pose.pose.position);

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
