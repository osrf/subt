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
#include <ignition/common/Console.hh>
#include <subt_ign/VisibilityRfModel.hh>

#include <ignition/math.hh>
#include <ignition/common.hh>

using namespace subt;
using namespace rf_interface;
using namespace visibilityModel;

/////////////////////////////////////////////
// RF power functions from SubT communication model
inline double dbmToPow(double x) { return 0.001 * pow(10., x / 10.); }
inline double QPSKPowerToBER(double P, double N) { return erfc(sqrt(P / N)); }

/////////////////////////////////////////////
VisibilityModel::VisibilityModel(
    visibilityModel::RfConfiguration _visibilityConfig,
    range_model::rf_configuration _rangeConfig,
    const std::string &_worldName)
    : visibilityConfig(_visibilityConfig),
      defaultRangeConfig(_rangeConfig)
{
  if (!this->visibilityTable.Load(_worldName))
  {
    ignerr << "Unable to load visibility table data files\n";
    return;
  }

  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::HOST);
  this->node.Advertise("/subt/comms_model/visualize",
      &VisibilityModel::VisualizeVisibility, this, opts);

  // Subscribe to pose messages.
  this->node.Subscribe("/world/" + _worldName + "/pose/info",
      &VisibilityModel::OnPose, this);

  this->initialized = true;
}

/////////////////////////////////////////////
bool VisibilityModel::Initialized() const
{
  return this->initialized;
}

/////////////////////////////////////////////
rf_power VisibilityModel::ComputeReceivedPower(const double &_txPower,
                                               radio_state &_txState,
                                               radio_state &_rxState)
{
  // Use this->visibilityTable.Cost(_txState, _rxState) to compute
  // pathloss and thus, received power
  VisibilityCost visibilityCost =
    this->visibilityTable.Cost(_txState.pose.Pos(), _rxState.pose.Pos());

  if (visibilityCost.cost > this->visibilityConfig.commsCostMax)
    return {-std::numeric_limits<double>::infinity(), 0.0};

  range_model::rf_configuration localConfig = this->defaultRangeConfig;

  // Augment fading exponent based on visibility cost
  localConfig.fading_exponent +=
    this->visibilityConfig.visibilityCostToFadingExponent * visibilityCost.cost;

  // Calculate the range.
  double range;
  if (visibilityCost.route.empty())
  {
    // No breadcrumbs in the route
    range = _txState.pose.Pos().Distance(_rxState.pose.Pos());
  }
  else
  {
    // One or more breadcrumbs to cross.
    double distSourceToFirstBreadcrumb =
      _txState.pose.Pos().Distance(visibilityCost.posFirstBreadcrumb);
    double distLastBreadcrumbToDestination =
      visibilityCost.posLastBreadcrumb.Distance(_rxState.pose.Pos());

    // This block considers breadcrumbs located in the tile of the destination
    // robot. Note that this breadcrumb is not included in the route but it
    // will be used for computing ranges.
    {
      double distLastMile = 0;
      int32_t x = std::round(_rxState.pose.Pos().X());
      int32_t y = std::round(_rxState.pose.Pos().Y());
      int32_t z = std::round(_rxState.pose.Pos().Z());
      auto vertexId = std::make_tuple(x, y, z);

      auto const &vertices = this->visibilityTable.Vertices();
      auto it = vertices.find(vertexId);
      if (it != vertices.end())
      {
        auto tileId = it->second;
        auto const &breadcrumbs = this->visibilityTable.Breadcrumbs();
        auto breadcrumbIt = breadcrumbs.find(tileId);
        if (breadcrumbIt != breadcrumbs.end())
        {
          auto poseLastBc = breadcrumbIt->second.front();
          distLastMile = poseLastBc.Distance(_rxState.pose.Pos());
          distLastBreadcrumbToDestination = distLastMile;
        }
      }
    }

    range = std::max(distSourceToFirstBreadcrumb,
                     std::max(visibilityCost.greatestDistanceSingleHop,
                              distLastBreadcrumbToDestination));
  }

  // Option 1: Using log_normal_v2_received_power.
  rf_power rx = range_model::log_normal_v2_received_power(
    _txPower, range, visibilityCost.route.size(), localConfig);
  igndbg << "Range: " << range << ", Exp: " << localConfig.fading_exponent
         << ", Num hops: " << visibilityCost.route.size()
         << ", TX: " << _txPower << ", RX: " << rx.mean << std::endl;
  // End option 1.

  // Option 2: Using log_normal_received_power.
  // range = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  // rf_power rx = range_model::log_normal_received_power(_txPower,
  //                                                      _txState,
  //                                                      _rxState,
  //                                                      localConfig);
  // igndbg << "Range: " << range << ", Exp: " << localConfig.fading_exponent
  //   << ", TX: " << _txPower << ", RX: " << rx.mean << std::endl;
  // End option 2.

  // Option 3: Using visibility_only_received_power.
  // rf_power rx = range_model::visibility_only_received_power(_txPower,
  //                                                           localConfig);
  // igndbg << "Exp: " << localConfig.fading_exponent
  //        << ", TX: " << _txPower << ", RX: " << rx.mean << std::endl;
  // End option 3.

  return std::move(rx);
}

/////////////////////////////////////////////
void VisibilityModel::PopulateVisibilityInfo(
  const std::set<ignition::math::Vector3d> &_relayPoses)
{
  this->visibilityTable.PopulateVisibilityInfo(_relayPoses);
}

/////////////////////////////////////////////
bool VisibilityModel::VisualizeVisibility(const ignition::msgs::StringMsg &_req,
                                          ignition::msgs::Boolean &_rep)
{
  _rep.set_data(false);

  std::map<int, ignition::msgs::Marker> perCostMarkers;

  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  markerMsg.mutable_lifetime()->set_sec(0.0);
  markerMsg.mutable_lifetime()->set_nsec(0.0);

  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 0, 0, 0, 0, 0));

  // Available colors
  //
  // GreenGlow, TurquoiseGlow, BlueGlow, YellowGlow, RedGlow
  // High (good)                                    Low (bad)
  std::map<int, ignition::math::Color> indexToColor;
  indexToColor[0] = ignition::math::Color(0, 1, 0);
  indexToColor[1] = ignition::math::Color(0, 1, 1);
  indexToColor[2] = ignition::math::Color(0, 0, 1);
  indexToColor[3] = ignition::math::Color(1, 1, 0);
  indexToColor[4] = ignition::math::Color(1, 0, 0);
  indexToColor[5] = ignition::math::Color(1, 1, 1);

  for (int i = 0; i < 6; ++i)
  {
    markerMsg.set_id(i);

    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    ignition::msgs::Set(matMsg->mutable_ambient(), indexToColor[i]);
    ignition::msgs::Set(matMsg->mutable_diffuse(), indexToColor[i]);
    ignition::msgs::Set(matMsg->mutable_emissive(), indexToColor[i]);
     ignition::msgs::Set(markerMsg.mutable_scale(),
                         ignition::math::Vector3d(1.0, 1.0, 1.0));

     perCostMarkers.insert(std::make_pair(i, markerMsg));
  }

  std::string modelName = _req.data();
  std::map<std::string, ignition::math::Pose3d>::iterator iter =
    this->poses.find(modelName);
  if (iter == this->poses.end())
  {
    ignerr << "[" << modelName << "] model not found" << std::endl;
    return true;
  }

  ignition::math::Vector3d from = iter->second.Pos();

  for (auto const &entry : this->visibilityTable.Vertices())
  {
    const auto &toTuple = entry.first;
    ignition::math::Vector3d to = ignition::math::Vector3d(
      std::get<0>(toTuple), std::get<1>(toTuple), std::get<2>(toTuple));
    VisibilityCost visibilityCost = this->visibilityTable.Cost(from, to);
    if (visibilityCost.cost <= this->visibilityConfig.commsCostMax)
    {
      /// Calculations from subt_communication_model/src/subt_communication_model.cpp
      double txPower = 20.0; // Hardcoded from cave_circuit.ign
      double noise_floor = -90.0; // Hardcoded from cave_circuit.ign
      rf_interface::radio_state tx, rx;
      // Set and calculate rf power
      tx.pose.Set(from.X(), from.Y(), from.Z(), 0, 0, 0);
      rx.pose.Set(to.X(), to.Y(), to.Z(), 0, 0, 0);
      rf_power rf_pow = ComputeReceivedPower(txPower, tx, rx);
      // Based on rx_power, noise value, and modulation, compute the bit error rate (BER)
      double ber = QPSKPowerToBER( dbmToPow(rf_pow.mean), dbmToPow(noise_floor) );
      int num_bytes = 100; // Hardcoded number of bytes
      double packet_drop_prob = 1.0 - exp(num_bytes*log(1-ber));
      // Scale packet drop probability to align with the color scheme
      int pdp = floor(packet_drop_prob*5.00001);

      auto m = perCostMarkers.find(static_cast<int>(pdp));

      if (m == perCostMarkers.end())
      {
        ignwarn << "Have not pre-allocated a marker for cost: " << packet_drop_prob
          << " (" << pdp << ")\n";
        continue;
      }

      ignition::msgs::Set(m->second.add_point(),
          ignition::math::Vector3d(to.X(), to.Y(), to.Z()));
    }
  }

  this->node.Request("/marker", perCostMarkers[0]);
  this->node.Request("/marker", perCostMarkers[1]);
  this->node.Request("/marker", perCostMarkers[2]);
  this->node.Request("/marker", perCostMarkers[3]);
  this->node.Request("/marker", perCostMarkers[4]);

  _rep.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void VisibilityModel::OnPose(const ignition::msgs::Pose_V &_msg)
{
  // Check the all the published poses
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    const ignition::msgs::Pose &pose = _msg.pose(i);
    this->poses[pose.name()] = ignition::msgs::Convert(pose);
  }
}
