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
#ifndef SUBT_IGN_VISIBILITYRFMODEL_HH_
#define SUBT_IGN_VISIBILITYRFMODEL_HH_

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

#include <subt_ign/VisibilityTable.hh>
#include <subt_rf_interface/subt_rf_interface.h>
#include <subt_rf_interface/subt_rf_model.h>

namespace subt
{
  namespace rf_interface
  {
    namespace visibilityModel
    {
      /// \struct RfConfiguration
      /// \brief RF configuration for LOS model
      struct RfConfiguration
      {
        /// \brief Fraction to increase fading exponent foreach unit of
        /// visibility cost.
        public: double visibilityCostToFadingExponent = 0.2;

        /// \brief  Maximum comms cost to consider (saturate here)
        public: double commsCostMax = 15.0;

        /// Output stream operator.
        ///
        /// @param oss Stream
        /// @param config RF Configuration to output
        friend std::ostream& operator<<(std::ostream &_oss,
                                        const RfConfiguration &_config)
        {
          _oss << "RF Configuration (visibility-based)" << std::endl
               << "-- visibilityCostToFadingExponent: "
               << _config.visibilityCostToFadingExponent << std::endl
               << "-- commsCostMax: "
               << _config.commsCostMax << std::endl;

          return _oss;
        }
      };

      /// \class VisibilityModel
      /// \brief Maintain state of the visibility model.
      ///
      /// The visibility model loads a pre-computed lookup table to
      /// determine the heuristic "cost" to communicate between SUBT
      /// environment tiles. This heuristic cost is used to adjust a fading
      /// exponent for a typical log-normal fading pathloss model.
      class VisibilityModel
      {
        /// \brief Constructor
        public: VisibilityModel(
                    visibilityModel::RfConfiguration _visibilityConfig,
                    range_model::rf_configuration _rangeConfig,
                    const std::string &_worldName);

        /// Compute received power function that will be given to
        /// communcation model.
        ///
        /// @param tx_power Transmit power (dBm)
        /// @param tx_state Transmitter state
        /// @param rx_state Receiver state
        public: rf_power ComputeReceivedPower(const double &_txPower,
                                              radio_state &_txState,
                                              radio_state &_rxState);

        /// Function to visualize visibility cost in Gazebo.
        private: bool VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                                          ignition::msgs::Boolean &_rep);
        /// \brief Handle gazebo pose messages
        /// \param[in] _msg New set of poses.
        private: void OnPose(const ignition::msgs::Pose_V &_msg);

        /// \brief Transport node
        private: ignition::transport::Node node;
        private: subt::VisibilityTable visibilityTable;
        private: visibilityModel::RfConfiguration visibilityConfig;
        private: range_model::rf_configuration defaultRangeConfig;
        private: std::map<std::string, ignition::math::Pose3d> poses;
      };
    }
  }
}
#endif
