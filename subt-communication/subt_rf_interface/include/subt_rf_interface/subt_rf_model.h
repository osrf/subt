#pragma once

#include <subt_rf_interface/subt_rf_interface.h>

namespace subt
{
namespace rf_interface
{
namespace range_model
{


rf_power distance_based_received_power(const double& tx_power,
                                       const radio_state& tx_state,
                                       const radio_state& rx_state,
                                       const rf_configuration& config);
}
}
}
