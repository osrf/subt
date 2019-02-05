#pragma once

#include <geometry_msgs/PoseStamped.h>

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
  geometry_msgs::PoseStamped pose;  ///< Pose of the radio
  std::list<std::pair<ros::Time, uint64_t>> bytes_sent; ///< Recent
                                                        ///sent packet
                                                        ///history
  uint64_t bytes_sent_this_epoch = 0; ///< Accumulation of bytes sent
                                      /// in an epoch
  std::list<std::pair<ros::Time, uint64_t>> bytes_received; ///<
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
                               radio_state&  //rx_state
                               )> pathloss_function;


}
}
